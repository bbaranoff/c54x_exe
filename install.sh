#!/bin/bash
# =============================================================================
#  install.sh - construit c54x_exe (le DSP Calypso hors QEMU) et pose sa ROM
# =============================================================================
#
#  Autonome : il faut les SOURCES de qosmo (clonees, pas construites :
#  c54x_exe compile hw/arm/calypso/l1-dsp/ de cet arbre) et libosmocore avec
#  libosmocoding, visibles de pkg-config - ce qu a deja qui a sa pile Osmocom.
#  C est aussi CE script qu appellent le Dockerfile d osmo-operator (stage l1),
#  son Dockerfile.run, start.sh, l ISO et l installation native
#  (install_modules/45-calypso.sh) : une seule liste de commandes.
#
#      ./install.sh                 tout : build, rom, verify
#      ./install.sh --check         prerequis et etat, NE MODIFIE RIEN
#      ./install.sh --list          les etapes et les valeurs retenues
#      ./install.sh --print-deps    les paquets apt, sur une ligne
#      sudo ./install.sh --deps     installe les paquets apt manquants, rien d autre
#      sudo ./install.sh --with-deps   les paquets manquants, puis tout
#      ./install.sh --only rom      une ou des etapes (liste a virgules)
#      ./install.sh --skip verify   toutes sauf celles-la
#      ./install.sh --reinstall     rejoue meme ce qui est deja fait
#      ./install.sh -v              la sortie des commandes a l ecran (sinon : journal)
#
#  Options, et la variable qui fait la meme chose (l option l emporte) :
#      --qosmo DIR      QOSMO        $GSM_ROOT/qosmo     les sources compilees
#      --rom-dir DIR    ROM_DIR      $GSM_ROOT           ou poser la ROM (repetable) ; sans
#                                                        l option : ROM_DIR ET rom/ de ce depot
#                                                        (Dockerfile:736). c54x_exe la lit dans
#                                                        --rom-dir, /opt/GSM par defaut (src/main.c:79)
#      --portable       PORTABLE=1   0                   CFLAGS du Dockerfile:735, sans -march=native :
#                                                        pour un binaire qui tournera sur un autre CPU
#      --dest DIR                                        copie ce depot dans DIR (sans .git ni binaires)
#                                                        et y construit : l arbre courant n est pas touche
#                       GSM_ROOT     /opt/GSM     ROM_VERSION 3606     LOG_DIR /tmp/c54x_exe-install
#                       FREECALYPSO_URL : miroir de la ROM (rom/fetch-rom.sh)
#
#  Correspondance avec osmo-operator/Dockerfile : chaque etape cite ses lignes,
#  celles du Dockerfile au commit 4266c8b, quand il portait encore ces commandes
#  (stage l1, 723-741 ; paquets 156-205). Il appelle desormais ce script.
# -----------------------------------------------------------------------------
set -uo pipefail
ICI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ═════════════════════════════════════════════════════════════════════════════
#  LE CONTRAT - celui de osmo-operator/install_modules/_lib/inst.sh
# ═════════════════════════════════════════════════════════════════════════════
#  Copie IDENTIQUE dans qosmo/install.sh, c54x_exe/install.sh et
#  grgsm_exe/install.sh : chaque depot doit pouvoir s installer seul, sans
#  osmo-operator. Une correction ici se reporte dans les deux autres.
#
#  Quatre fonctions par etape, `run` seule obligatoire :
#     inst_<etape>_check    prerequis, LECTURE SEULE        0 ok · 1 manque · 4 sans objet
#     inst_<etape>_done     deja fait ?                     0 oui · 1 non
#     inst_<etape>_run      fait le travail                 0 ok · 1 echec · 3 deja fait
#     inst_<etape>_verify   controle APRES coup             0 ok · 1 echec
# -----------------------------------------------------------------------------
readonly INST_RC_OK=0 INST_RC_FAIL=1 INST_RC_DONE=3 INST_RC_NA=4
declare -a INST_ORDER=()
declare -A INST_DESC=()
INST_REGISTER() { INST_ORDER+=("$1"); INST_DESC[$1]="$2"; }

_INST_REASON=""
_INST_HINT=""
inst_ok()   { _INST_REASON=""   ; return $INST_RC_OK; }
inst_fail() { _INST_REASON="$*" ; return $INST_RC_FAIL; }
inst_done() { _INST_REASON="$*" ; return $INST_RC_DONE; }
inst_na()   { _INST_REASON="$*" ; return $INST_RC_NA; }
inst_hint() { _INST_HINT="$*"; }
inst_say()  { printf '%s\n' "$*"; }

have_cmd()  { command -v "$1" >/dev/null 2>&1; }
have_file() { [ -f "$1" ]; }
have_pkg()  { dpkg -s "$1" >/dev/null 2>&1; }
# Un chemin est inscriptible s il l est, ou si son plus proche parent existant
# l est (on pourra le creer).
ecrivable() { local d="$1"; while [ ! -e "$d" ] && [ "$d" != / ]; do d="$(dirname "$d")"; done; [ -w "$d" ]; }
# Chemin absolu sans exiger qu il existe (configure, make -C et les --dest
# changent de dossier : un chemin relatif n y voudrait plus rien dire).
absolu() { case "$1" in /*) printf '%s\n' "$1" ;; *) printf '%s\n' "$PWD/$1" ;; esac; }

# exige "ce qu il faut" "conseil si absent" commande... : une condition de
# prerequis. En --check elle est imprimee (ok / MANQUE + conseil) ; sinon elle
# est memorisee, et la premiere qui manque donne la raison de l echec.
_MANQUE=()
exige() {
    local quoi="$1" conseil="$2"; shift 2
    if "$@" >/dev/null 2>&1; then
        [ "$ACTION" = check ] && printf '      ok      %s\n' "$quoi"
        return 0
    fi
    if [ "$ACTION" = check ]; then
        printf '      %sMANQUE%s  %s\n' "$C_KO" "$C_Z" "$quoi"
        [ -n "$conseil" ] && printf '              → %s\n' "$conseil"
    fi
    [ ${#_MANQUE[@]} -eq 0 ] && [ -n "$conseil" ] && inst_hint "$conseil"
    _MANQUE+=("$quoi")
    return 1
}
# Verdict d une fonction check qui a appele exige.
bilan_exige() {
    [ ${#_MANQUE[@]} -eq 0 ] && { inst_ok; return $INST_RC_OK; }
    local plus=""; [ ${#_MANQUE[@]} -gt 1 ] && plus=" (+$(( ${#_MANQUE[@]} - 1 )) autre(s), --check)"
    inst_fail "manque : ${_MANQUE[0]}$plus"
}
# ═════════════════════════════════════════════════════════════════════════════
#  LE COMPOSANT - c54x_exe
# ═════════════════════════════════════════════════════════════════════════════
COMPOSANT=c54x_exe
: "${GSM_ROOT:=/opt/GSM}"
: "${QOSMO:=$GSM_ROOT/qosmo}"
: "${ROM_DIR:=$GSM_ROOT}"
: "${ROM_VERSION:=3606}"
: "${PORTABLE:=0}"
: "${LOG_DIR:=/tmp/c54x_exe-install}"
ROM_DIRS=()
DEST=""
# Dockerfile:735 - les drapeaux du Makefile, sans -march=native : un binaire
# -march=native construit ailleurs meurt en SIGILL au demarrage (Dockerfile:726-728).
CFLAGS_PORTABLE="-O3 -g -Wall -Werror=format -Werror=format-extra-args -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare"
ROMS="DROM PDROM PROM0 PROM1 PROM2 PROM3"

# Paquets apt, repris de la liste unique d osmo-operator/Dockerfile :
#   :159      build-essential pkg-config curl       compilation ; curl pour rom/fetch-rom.sh
#   :166      python3                               tools/dsp_txt2bin.py (conversion de la ROM)
# libosmocore / libosmocoding ne sont PAS des paquets apt dans l image : elle les
# compile (Dockerfile:260-311). Ici on les exige seulement, par pkg-config.
PAQUETS=(build-essential pkg-config python3 curl)

options_composant() {
    case "$1" in
        --qosmo)    _val "$@"; QOSMO="$2";       _n=2 ;;
        --rom-dir)  _val "$@"; ROM_DIRS+=("$2"); _n=2 ;;
        --dest)     _val "$@"; DEST="$2";        _n=2 ;;
        --portable) PORTABLE=1;                  _n=1 ;;
    esac
}

apres_options() {
    QOSMO="$(absolu "$QOSMO")"
    if [ ${#ROM_DIRS[@]} -eq 0 ]; then ROM_DIRS=("$ROM_DIR" "$ICI/rom"); fi
    local d l=(); for d in "${ROM_DIRS[@]}"; do
        d="$(absolu "${d%/}")"; in_liste "$d" "${l[@]+"${l[@]}"}" || l+=("$d")
    done
    ROM_DIRS=("${l[@]}")
    # --dest : on construit une COPIE. Le binaire en service (un banc qui tourne
    # sur $ICI/c54x_exe) n est pas reecrit sous ses pieds.
    if [ -n "$DEST" ] && [ "$ACTION" = install ]; then
        DEST="$(absolu "$DEST")"
        if [ "$DEST" != "$ICI" ]; then
            mkdir -p "$DEST" || { printf 'impossible de creer %s\n' "$DEST" >&2; exit 1; }
            tar -C "$ICI" --exclude=./.git --exclude=./c54x_exe --exclude='./c54x_exe.*' \
                --exclude=./isa_test -cf - . | tar -C "$DEST" -xf - \
                || { printf 'copie vers %s impossible\n' "$DEST" >&2; exit 1; }
            printf 'arbre copie dans %s (sans .git ni binaires) - la suite s y deroule\n' "$DEST"
            local a sauter=0 args=()
            for a in "${_ARGS_ORIG[@]}"; do
                [ "$sauter" = 1 ] && { sauter=0; continue; }
                [ "$a" = --dest ] && { sauter=1; continue; }
                args+=("$a")
            done
            # sans --rom-dir, la ROM va dans ROM_DIR et rom/ de la COPIE
            exec bash "$DEST/install.sh" "${args[@]+"${args[@]}"}"
        fi
    fi
}
in_liste() { local x="$1"; shift; local y; for y in "$@"; do [ "$x" = "$y" ] && return 0; done; return 1; }

resume_composant() {
    printf 'depot=%s\nQOSMO=%s\nROM (%s) -> %s\nCFLAGS=%s\n' "$ICI" "$QOSMO" "$ROM_VERSION" "${ROM_DIRS[*]}" \
        "$([ "$PORTABLE" = 1 ] && echo "portables (Dockerfile:735)" || echo "ceux du Makefile (-march=native)")"
}
avant_etapes() { :; }
apres_etapes() {
    [ -x "$ICI/c54x_exe" ] && printf '  binaire : %s/c54x_exe\n' "$ICI"
    printf '  ROM     : %s\n' "${ROM_DIRS[*]}"
    [ "${ROM_DIRS[0]}" = /opt/GSM ] || printf '  (hors /opt/GSM : lancer c54x_exe avec --rom-dir %s)\n' "${ROM_DIRS[0]}"
    printf '  lancer  : PONT=1 %s/run.sh   (wiki osmo-operator, Installer-c54x_exe)\n' "$ICI"
}

# La ROM d un dossier est-elle complete et conforme a rom/SHA256SUMS.<version> ?
# (le meme test que bon() de rom/fetch-rom.sh, plus l instantane de registres)
rom_valide() {
    local s="$ICI/rom/SHA256SUMS.$ROM_VERSION" r
    [ -f "$s" ] || return 1
    for r in $ROMS Registers; do [ -f "$1/calypso_dsp.$r.bin" ] || return 1; done
    ( cd "$1" && sha256sum -c --quiet "$s" ) >/dev/null 2>&1
}
# Une copie deja verifiee sur cette machine, parmi les destinations et rom/.
rom_source_locale() {
    local d; for d in "${ROM_DIRS[@]}" "$ICI/rom"; do rom_valide "$d" && { printf '%s\n' "$d"; return 0; }; done
    return 1
}

# ── build ─────────────────────────────────────────────────────────────────────
# Pas de « deja fait » : le Makefile recompile a CHAQUE appel, c est voulu
# (Makefile:46-52, les sources viennent d un autre depot).
INST_REGISTER build "Compilation de c54x_exe (sources de qosmo)"
_conseil_osmo() {
    if [ -f /usr/local/lib/pkgconfig/libosmocoding.pc ]; then
        printf 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig  (libosmocore est en /usr/local, Dockerfile:92-94)'
    else
        printf 'installez libosmocore (sources : https://gitea.osmocom.org/osmocom/libosmocore, ou le paquet libosmocore-dev de votre depot Osmocom)'
    fi
}
inst_build_check() {
    _MANQUE=()
    exige "sources qosmo ($QOSMO/hw/arm/calypso/l1-dsp/calypso_c54x.c)" \
        "git clone https://github.com/bbaranoff/qosmO $QOSMO, ou --qosmo DIR" \
        test -f "$QOSMO/hw/arm/calypso/l1-dsp/calypso_c54x.c"
    exige "cales hors QEMU ($QOSMO/contrib/hors-qemu/cales-qemu.c)" "qosmo trop ancien : git -C $QOSMO pull" \
        test -f "$QOSMO/contrib/hors-qemu/cales-qemu.c"
    exige "compilateur C (${CC:-gcc})" "apt-get install build-essential" have_cmd "${CC:-gcc}"
    exige "make" "apt-get install build-essential" have_cmd make
    exige "pkg-config" "apt-get install pkg-config" have_cmd pkg-config
    # Le Makefile (ligne 14) avale l erreur de pkg-config (2>/dev/null) : sans
    # ce controle, l echec n arrive qu au link, sur des gsm0503_* introuvables.
    exige "libosmocoding + libosmocore vus de pkg-config" "$(_conseil_osmo)" \
        pkg-config --exists libosmocoding libosmocore
    exige "$ICI inscriptible (le binaire y est ecrit)" "--dest DIR pour construire une copie ailleurs" test -w "$ICI"
    bilan_exige
}
inst_build_run() {
    # Dockerfile:733-735 : cd c54x_exe && make QOSMO=/opt/GSM/qosmo CFLAGS="<sans -march=native>"
    local args=("QOSMO=$QOSMO")
    [ "$PORTABLE" = 1 ] && args+=("CFLAGS=$CFLAGS_PORTABLE")
    make -C "$ICI" "${args[@]}" || { inst_fail "echec de compilation"; return $INST_RC_FAIL; }
    inst_ok
}
inst_build_verify() {
    [ -x "$ICI/c54x_exe" ] || { inst_fail "$ICI/c54x_exe absent apres make"; return $INST_RC_FAIL; }
    local nf
    nf="$(ldd "$ICI/c54x_exe" 2>/dev/null | awk '/not found/ {print $1}' | tr '\n' ' ')"
    [ -z "$nf" ] || { inst_hint "sudo ldconfig, ou LD_LIBRARY_PATH=/usr/local/lib"; inst_fail "bibliotheques introuvables : $nf"; return $INST_RC_FAIL; }
    inst_ok
}

# ── rom : la mask-ROM TI, telechargee (pas dans le depot) ─────────────────────
INST_REGISTER rom "ROM DSP $ROM_VERSION (FreeCalypso, somme verifiee)"
inst_rom_done() { local d; for d in "${ROM_DIRS[@]}"; do rom_valide "$d" || return 1; done; }
inst_rom_check() {
    _MANQUE=()
    exige "rom/fetch-rom.sh et rom/SHA256SUMS.$ROM_VERSION" "" test -f "$ICI/rom/fetch-rom.sh" -a -f "$ICI/rom/SHA256SUMS.$ROM_VERSION"
    exige "sha256sum" "apt-get install coreutils" have_cmd sha256sum
    local d; for d in "${ROM_DIRS[@]}"; do
        exige "$d inscriptible (ROM)" "sudo, ou --rom-dir DIR" ecrivable "$d"
    done
    # Le reseau, python3 et curl ne servent que s il n y a nulle part de copie valide.
    if ! rom_source_locale >/dev/null; then
        exige "python3 (tools/dsp_txt2bin.py)" "apt-get install python3" have_cmd python3
        exige "curl ou wget (telechargement FreeCalypso)" "apt-get install curl" sh -c 'command -v curl || command -v wget'
    fi
    bilan_exige
}
inst_rom_run() {
    # 1. Une copie deja verifiee sur cette machine ? on la recopie, sans reseau.
    #    (Dockerfile.run:188 et start.sh faisaient cp c54x_exe/rom/*.bin /opt/GSM/.)
    local src d r args=()
    if src="$(rom_source_locale)"; then
        for d in "${ROM_DIRS[@]}"; do
            [ "$d" = "$src" ] && continue
            rom_valide "$d" && continue
            mkdir -p "$d" || { inst_fail "impossible de creer $d"; return $INST_RC_FAIL; }
            for r in $ROMS Registers; do
                install -m644 "$src/calypso_dsp.$r.bin" "$d/calypso_dsp.$r.bin" \
                    || { inst_fail "copie de calypso_dsp.$r.bin vers $d impossible"; return $INST_RC_FAIL; }
            done
            inst_say "ROM DSP $ROM_VERSION recopiee de $src vers $d (somme verifiee)"
        done
    fi
    # 2. rom/fetch-rom.sh fait le reste : telecharge, convertit, verifie, depose
    #    (Dockerfile:736). Tout deja en place : il le dit et ne telecharge rien.
    for d in "${ROM_DIRS[@]}"; do args+=(--dest "$d"); done
    bash "$ICI/rom/fetch-rom.sh" --version "$ROM_VERSION" "${args[@]}" \
        || { inst_hint "reseau ? un miroir : FREECALYPSO_URL=..."; inst_fail "rom/fetch-rom.sh en echec"; return $INST_RC_FAIL; }
    inst_ok
}
inst_rom_verify() { inst_rom_done && inst_ok || inst_fail "ROM incomplete ou somme differente dans ${ROM_DIRS[*]}"; }

# ── verify : le DSP seul ──────────────────────────────────────────────────────
# README.md:66 : « la ROM seule, sans ARM : boote-t-elle ? ». Hors --arm,
# c54x_exe travaille sur une API RAM PRIVEE : ni /dev/shm, ni socket, ni
# /tmp/c54x-pont - un banc en marche a cote n en voit rien. Les CALYPSO_* de
# l appelant (sondes, traces) sont retires pour que ca le reste.
INST_REGISTER verify "Controle : c54x_exe --trames 200 (la ROM seule)"
sans_calypso_env() {
    local v u=(); for v in $(compgen -e); do case "$v" in CALYPSO_*) u+=(-u "$v") ;; esac; done
    env "${u[@]+"${u[@]}"}" "$@"
}
inst_verify_check() {
    _MANQUE=()
    etape_retenue build || exige "binaire $ICI/c54x_exe" "./install.sh --only build" test -x "$ICI/c54x_exe"
    etape_retenue rom   || exige "ROM DSP dans ${ROM_DIRS[0]}" "./install.sh --only rom" rom_valide "${ROM_DIRS[0]}"
    bilan_exige
}
inst_verify_run() {
    local out rc
    out="$(sans_calypso_env timeout 300 "$ICI/c54x_exe" --rom-dir "${ROM_DIRS[0]}" --trames 200 2>&1)"; rc=$?
    printf '%s\n' "$out" | tail -n 14
    [ $rc -eq 0 ] || { inst_fail "c54x_exe --trames 200 rend $rc"; return $INST_RC_FAIL; }
    grep -q 'sections chargees' <<<"$out" && grep -q 'bilan sur 200 trames' <<<"$out" \
        || { inst_fail "sortie inattendue (ni « sections chargees » ni « bilan sur 200 trames »)"; return $INST_RC_FAIL; }
    inst_ok
}
# ═════════════════════════════════════════════════════════════════════════════
#  LE MOTEUR - copie IDENTIQUE dans les trois depots (voir LE CONTRAT)
# ═════════════════════════════════════════════════════════════════════════════
#  Meme deroule que osmo-operator/install.sh : pour chaque etape retenue,
#  « deja fait ? » (sauf --reinstall), prerequis, travail, controle. Le premier
#  echec arrete tout, dit pourquoi et montre la fin du journal de l etape.
#  Le composant fournit : COMPOSANT, PAQUETS, LOG_DIR, les etapes, et les
#  crochets options_composant / apres_options / resume_composant /
#  avant_etapes / apres_etapes.
# -----------------------------------------------------------------------------
ACTION=install ONLY="" SKIP="" WITH_DEPS=0
: "${REINSTALL:=0}" "${VERBOSE:=0}"
_ARGS_ORIG=("$@")
# _val OPTION [VALEUR] : une option qui attend une valeur doit l avoir.
_val() { [ $# -ge 2 ] && [ -n "$2" ] || { printf '%s : valeur attendue   (--help)\n' "$1" >&2; exit 2; }; }
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help)    ACTION=help ;;
        --check)      ACTION=check ;;
        --list)       ACTION=list ;;
        --deps)       ACTION=deps ;;
        --print-deps) ACTION=print-deps ;;
        --with-deps)  WITH_DEPS=1 ;;
        --only)       _val "$@"; ONLY="$2"; shift ;;
        --skip)       _val "$@"; SKIP="$2"; shift ;;
        --reinstall)  REINSTALL=1 ;;
        -v|--verbose) VERBOSE=1 ;;
        *)  _n=0; options_composant "$@"
            [ "$_n" -gt 0 ] || { printf 'option inconnue : %s   (--help)\n' "$1" >&2; exit 2; }
            shift $((_n - 1)) ;;
    esac
    shift
done

if [ "$ACTION" = help ]; then
    awk 'NR > 2 && /^# -{10}/ { exit } NR > 2 && !/^# =+$/ { sub(/^# ?/, ""); print }' "$0"
    exit 0
fi
apres_options

if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    TTY=1; C_OK=$'\033[32m'; C_KO=$'\033[31m'; C_SK=$'\033[33m'; C_DIM=$'\033[2m'; C_Z=$'\033[0m'
else
    TTY=0; C_OK=""; C_KO=""; C_SK=""; C_DIM=""; C_Z=""
fi
begin() { if [ $TTY -eq 1 ] && [ "$VERBOSE" != 1 ]; then printf '[ %s..%s ] %s' "$C_DIM" "$C_Z" "$1"; else printf '[ .. ] %s\n' "$1"; fi; }
end()   { [ $TTY -eq 1 ] && [ "$VERBOSE" != 1 ] && printf '\r\033[K'
          printf '[%s%s%s] %s' "$2" "$1" "$C_Z" "$3"
          [ -n "${4:-}" ] && printf ' %s(%s)%s' "$C_DIM" "$4" "$C_Z"; printf '\n'; }

# --- selection des etapes (--only / --skip, listes a virgules) ---------------
in_csv() { case ",$1," in *",$2,"*) return 0;; esac; return 1; }
for _s in ${ONLY//,/ } ${SKIP//,/ }; do
    [ -n "${INST_DESC[$_s]+x}" ] || { printf 'etape inconnue : %s   (etapes : %s)\n' "$_s" "${INST_ORDER[*]}" >&2; exit 2; }
done
SELECTED=()
for _s in "${INST_ORDER[@]}"; do
    [ -n "$ONLY" ] && ! in_csv "$ONLY" "$_s" && continue
    [ -n "$SKIP" ] && in_csv "$SKIP" "$_s" && continue
    SELECTED+=("$_s")
done
# Une etape qui produit ce qu une autre exige : en --check, on ne reproche pas
# a la seconde l absence de ce que la premiere va justement fabriquer.
etape_retenue() { in_csv "$(IFS=,; echo "${SELECTED[*]}")" "$1"; }

# --- paquets apt --------------------------------------------------------------
# rend la liste des absents ; code 2 sans dpkg (pas une Debian/Ubuntu)
paquets_absents() {
    have_cmd dpkg || return 2
    local p; for p in "${PAQUETS[@]}"; do have_pkg "$p" || printf '%s ' "$p"; done
    return 0
}
installer_paquets() {
    local manque rc
    manque="$(paquets_absents)"; rc=$?
    if [ $rc -eq 2 ]; then
        printf 'dpkg absent : installez l equivalent de : %s\n' "${PAQUETS[*]}" >&2; return 1
    fi
    if [ -z "${manque// }" ]; then
        printf 'paquets apt : les %d sont presents\n' "${#PAQUETS[@]}"; return 0
    fi
    if [ "$(id -u)" -ne 0 ]; then
        printf 'paquets apt manquants : %s\n  sudo apt-get install -y --no-install-recommends %s\n' "$manque" "$manque" >&2
        return 1
    fi
    printf 'apt-get install : %s\n' "$manque"
    apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $manque || return 1
    manque="$(paquets_absents)"
    [ -z "${manque// }" ] || { printf 'toujours absents apres apt-get : %s\n' "$manque" >&2; return 1; }
}

case "$ACTION" in
    print-deps) printf '%s\n' "${PAQUETS[*]}"; exit 0 ;;
    deps)       installer_paquets; exit $? ;;
    list)
        printf '%s - %d etape(s), dans cet ordre\n\n' "$COMPOSANT" "${#SELECTED[@]}"
        for _s in "${SELECTED[@]}"; do printf '  %-10s %s\n' "$_s" "${INST_DESC[$_s]}"; done
        printf '\n'; resume_composant | sed 's/^/  /'
        exit 0 ;;
    check)
        printf '%s : prerequis et etat (rien n est modifie)\n\n' "$COMPOSANT"
        resume_composant | sed 's/^/  /'
        _manque="$(paquets_absents)"; _rc=$?
        if [ $_rc -eq 2 ]; then printf '\n  paquets apt : dpkg absent, non verifies (%s)\n' "${PAQUETS[*]}"
        elif [ -n "${_manque// }" ]; then printf '\n  paquets apt absents (installes autrement ? sinon --deps) : %s\n' "$_manque"
        else printf '\n  paquets apt : les %d sont presents\n' "${#PAQUETS[@]}"; fi
        _rc=0
        for _s in "${SELECTED[@]}"; do
            _p="inst_${_s//-/_}"
            if ! declare -F "${_p}_done" >/dev/null; then _etat="pas de controle"
            elif "${_p}_done" >/dev/null 2>&1; then _etat="deja fait"
            else _etat="a faire"; fi
            printf '\n  %-10s %s  %s[%s]%s\n' "$_s" "${INST_DESC[$_s]}" "$C_DIM" "$_etat" "$C_Z"
            declare -F "${_p}_check" >/dev/null || continue
            _INST_REASON=""; _INST_HINT=""; _MANQUE=()
            "${_p}_check"; _r=$?
            case $_r in
                "$INST_RC_FAIL") _rc=1 ;;
                "$INST_RC_NA")   printf '      sans objet : %s\n' "$_INST_REASON" ;;
            esac
        done
        printf '\n'
        if [ $_rc -eq 0 ]; then printf '%sprerequis satisfaits%s\n' "$C_OK" "$C_Z"
        else printf '%sprerequis manquants%s (voir MANQUE ci-dessus)\n' "$C_KO" "$C_Z"; fi
        exit $_rc ;;
esac

# --- installation ---------------------------------------------------------------
if [ "$WITH_DEPS" = 1 ]; then
    installer_paquets || { printf 'paquets apt : echec - rien n a ete construit\n' >&2; exit 1; }
else
    _manque="$(paquets_absents 2>/dev/null)"
    [ -n "${_manque// }" ] && printf '%snote : paquets apt absents (installes autrement ? sinon --with-deps) : %s%s\n' "$C_DIM" "$_manque" "$C_Z"
fi
if [ "$VERBOSE" != 1 ]; then
    mkdir -p "$LOG_DIR" || { printf 'journal impossible : %s (LOG_DIR=...)\n' "$LOG_DIR" >&2; exit 1; }
fi
# _jouer FONCTION : la sortie va au journal de l etape, ou a l ecran en -v.
# Pas de tube : la fonction tourne dans CE shell (sa raison d echec survit).
_jouer() { if [ "$VERBOSE" = 1 ]; then "$1"; else "$1" >>"$_log" 2>&1; fi; }
_echec() {   # _echec ETAPE RAISON [journal] : dit tout ce qu on sait, et s arrete
    end FAIL "$C_KO" "${INST_DESC[$1]}" "$2"
    [ -n "$_INST_HINT" ] && printf '       → %s\n' "$_INST_HINT"
    # la fin du journal, quand c est le travail (run, verify) qui a echoue
    if [ -n "${3:-}" ] && [ "$VERBOSE" != 1 ] && [ -s "$_log" ]; then
        printf '       %sjournal : %s (fin ci-dessous)%s\n' "$C_DIM" "$_log" "$C_Z"
        tail -n 25 "$_log" | sed 's/^/       | /'
    fi
    printf '\n%s : installation interrompue a l etape %s\n' "$COMPOSANT" "$1"
    exit 1
}
avant_etapes
_nb_ok=0; _nb_skip=0
for _s in "${SELECTED[@]}"; do
    _p="inst_${_s//-/_}"; _log="$LOG_DIR/$_s.log"
    _INST_REASON=""; _INST_HINT=""; _MANQUE=()
    [ "$VERBOSE" = 1 ] || printf '\n===== %s  %s =====\n' "$(date '+%F %T')" "$_s" >>"$_log"
    begin "${INST_DESC[$_s]}"
    if [ "$REINSTALL" != 1 ] && declare -F "${_p}_done" >/dev/null && _jouer "${_p}_done"; then
        end SKIP "$C_SK" "${INST_DESC[$_s]}" "deja fait"; _nb_skip=$((_nb_skip + 1)); continue
    fi
    if declare -F "${_p}_check" >/dev/null; then
        _jouer "${_p}_check"; _r=$?
        case $_r in
            "$INST_RC_OK")   ;;
            "$INST_RC_NA")   end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-sans objet}"
                             _nb_skip=$((_nb_skip + 1)); continue ;;
            *)               _echec "$_s" "${_INST_REASON:-prerequis non satisfait}" ;;
        esac
    fi
    _jouer "${_p}_run"; _r=$?
    case $_r in
        "$INST_RC_OK")   ;;
        "$INST_RC_DONE"|"$INST_RC_NA")
                         end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-rien a faire}"
                         _nb_skip=$((_nb_skip + 1)); continue ;;
        *)               _echec "$_s" "${_INST_REASON:-echec}" journal ;;
    esac
    # installer n est pas avoir installe : on controle apres coup
    if declare -F "${_p}_verify" >/dev/null && ! _jouer "${_p}_verify"; then
        _echec "$_s" "faite, mais le controle echoue : ${_INST_REASON:-}" journal
    fi
    end " OK " "$C_OK" "${INST_DESC[$_s]}" "${_INST_REASON:-}"; _nb_ok=$((_nb_ok + 1))
done
printf '\n%s : %d ok · %d ignoree(s) · 0 echec\n' "$COMPOSANT" "$_nb_ok" "$_nb_skip"
apres_etapes
exit 0
