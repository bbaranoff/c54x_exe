#!/bin/bash
# =============================================================================
#  install.sh - builds c54x_exe (the Calypso DSP outside QEMU) and installs its ROM
# =============================================================================
#
#  Standalone: it needs the qosmo SOURCES (cloned, not built: c54x_exe compiles
#  hw/arm/calypso/l1-dsp/ of that tree) and libosmocore with libosmocoding,
#  visible to pkg-config - which anyone with an Osmocom stack already has.
#  This is also THE script called by the osmo-operator Dockerfile (stage l1),
#  its Dockerfile.run, start-docker.sh, the ISO and the native installation
#  (install_modules/45-calypso.sh): a single list of commands.
#
#      ./install.sh                 everything: build, rom, verify
#      ./install.sh --check         prerequisites and state, MODIFIES NOTHING
#      ./install.sh --list          the steps and the values retained
#      ./install.sh --print-deps    the apt packages, on one line
#      sudo ./install.sh --deps     installs the missing apt packages, nothing else
#      sudo ./install.sh --with-deps   the missing packages, then everything
#      ./install.sh --only rom      one or more steps (comma-separated list)
#      ./install.sh --skip verify   all but those
#      ./install.sh --reinstall     replays even what is already done
#      ./install.sh -v              command output on screen (otherwise: log)
#
#  Options, and the variable that does the same (the option wins):
#      --qosmo DIR      QOSMO        $GSM_ROOT/qosmo     the compiled sources
#      --rom-dir DIR    ROM_DIR      $GSM_ROOT           where to put the ROM (repeatable); without
#                                                        the option: ROM_DIR AND rom/ of this repo
#                                                        (Dockerfile:736). c54x_exe reads it from
#                                                        --rom-dir, /opt/GSM by default (src/main.c:79)
#      --portable       PORTABLE=1   0                   CFLAGS of Dockerfile:735, without -march=native:
#                                                        for a binary that will run on another CPU
#      --dest DIR                                        copies this repo into DIR (without .git or binaries)
#                                                        and builds there: the current tree is untouched
#                       GSM_ROOT     /opt/GSM     ROM_VERSION 3606     LOG_DIR /tmp/c54x_exe-install
#                       FREECALYPSO_URL: ROM mirror (rom/fetch-rom.sh)
#
#  Correspondence with osmo-operator/Dockerfile: each step cites its lines,
#  those of the Dockerfile at commit 4266c8b, when it still carried these
#  commands (stage l1, 723-741; packages 156-205). It now calls this script.
# -----------------------------------------------------------------------------
set -uo pipefail
ICI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ═════════════════════════════════════════════════════════════════════════════
#  THE CONTRACT - that of osmo-operator/install_modules/_lib/inst.sh
# ═════════════════════════════════════════════════════════════════════════════
#  IDENTICAL copy in qosmo/install.sh, c54x_exe/install.sh and
#  grgsm_exe/install.sh: each repo must be able to install on its own, without
#  osmo-operator. A fix here is carried over to the other two.
#
#  Four functions per step, only `run` is mandatory:
#     inst_<step>_check     prerequisites, READ ONLY        0 ok · 1 missing · 4 not applicable
#     inst_<step>_done      already done?                   0 yes · 1 no
#     inst_<step>_run       does the work                   0 ok · 1 failure · 3 already done
#     inst_<step>_verify    check AFTER the fact            0 ok · 1 failure
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
# A path is writable if it is, or if its closest existing parent is (it can
# then be created).
ecrivable() { local d="$1"; while [ ! -e "$d" ] && [ "$d" != / ]; do d="$(dirname "$d")"; done; [ -w "$d" ]; }
# Absolute path without requiring it to exist (configure, make -C and the
# --dest change directory: a relative path would mean nothing there).
absolu() { case "$1" in /*) printf '%s\n' "$1" ;; *) printf '%s\n' "$PWD/$1" ;; esac; }

# exige "what is needed" "advice if absent" command...: a prerequisite
# condition. Under --check it is printed (ok / MISSING + advice); otherwise it
# is remembered, and the first missing one gives the reason for the failure.
_MANQUE=()
exige() {
    local quoi="$1" conseil="$2"; shift 2
    if "$@" >/dev/null 2>&1; then
        [ "$ACTION" = check ] && printf '      ok      %s\n' "$quoi"
        return 0
    fi
    if [ "$ACTION" = check ]; then
        printf '      %sMISSING%s %s\n' "$C_KO" "$C_Z" "$quoi"
        [ -n "$conseil" ] && printf '              → %s\n' "$conseil"
    fi
    [ ${#_MANQUE[@]} -eq 0 ] && [ -n "$conseil" ] && inst_hint "$conseil"
    _MANQUE+=("$quoi")
    return 1
}
# Verdict of a check function that called exige.
bilan_exige() {
    [ ${#_MANQUE[@]} -eq 0 ] && { inst_ok; return $INST_RC_OK; }
    local plus=""; [ ${#_MANQUE[@]} -gt 1 ] && plus=" (+$(( ${#_MANQUE[@]} - 1 )) more, --check)"
    inst_fail "missing: ${_MANQUE[0]}$plus"
}
# ═════════════════════════════════════════════════════════════════════════════
#  THE COMPONENT - c54x_exe
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
# Dockerfile:735 - the Makefile flags, without -march=native: a -march=native
# binary built elsewhere dies with SIGILL at startup (Dockerfile:726-728).
CFLAGS_PORTABLE="-O3 -g -Wall -Werror=format -Werror=format-extra-args -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare"
ROMS="DROM PDROM PROM0 PROM1 PROM2 PROM3"

# apt packages, taken from the single list of osmo-operator/Dockerfile:
#   :159      build-essential pkg-config curl       compilation; curl for rom/fetch-rom.sh
#   :166      python3                               tools/dsp_txt2bin.py (ROM conversion)
# libosmocore / libosmocoding are NOT apt packages in the image: it compiles
# them (Dockerfile:260-311). Here they are only required, through pkg-config.
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
    # --dest: a COPY is built. The binary in service (a bench running on
    # $ICI/c54x_exe) is not rewritten under its feet.
    if [ -n "$DEST" ] && [ "$ACTION" = install ]; then
        DEST="$(absolu "$DEST")"
        if [ "$DEST" != "$ICI" ]; then
            mkdir -p "$DEST" || { printf 'cannot create %s\n' "$DEST" >&2; exit 1; }
            tar -C "$ICI" --exclude=./.git --exclude=./c54x_exe --exclude='./c54x_exe.*' \
                --exclude=./isa_test -cf - . | tar -C "$DEST" -xf - \
                || { printf 'copy to %s failed\n' "$DEST" >&2; exit 1; }
            printf 'tree copied to %s (without .git or binaries) - the rest runs there\n' "$DEST"
            local a sauter=0 args=()
            for a in "${_ARGS_ORIG[@]}"; do
                [ "$sauter" = 1 ] && { sauter=0; continue; }
                [ "$a" = --dest ] && { sauter=1; continue; }
                args+=("$a")
            done
            # without --rom-dir, the ROM goes into ROM_DIR and rom/ of the COPY
            exec bash "$DEST/install.sh" "${args[@]+"${args[@]}"}"
        fi
    fi
}
in_liste() { local x="$1"; shift; local y; for y in "$@"; do [ "$x" = "$y" ] && return 0; done; return 1; }

resume_composant() {
    printf 'repo=%s\nQOSMO=%s\nROM (%s) -> %s\nCFLAGS=%s\n' "$ICI" "$QOSMO" "$ROM_VERSION" "${ROM_DIRS[*]}" \
        "$([ "$PORTABLE" = 1 ] && echo "portable (Dockerfile:735)" || echo "the Makefile ones (-march=native)")"
}
avant_etapes() { :; }
apres_etapes() {
    [ -x "$ICI/c54x_exe" ] && printf '  binary  : %s/c54x_exe\n' "$ICI"
    printf '  ROM     : %s\n' "${ROM_DIRS[*]}"
    [ "${ROM_DIRS[0]}" = /opt/GSM ] || printf '  (outside /opt/GSM: run c54x_exe with --rom-dir %s)\n' "${ROM_DIRS[0]}"
    printf '  run     : PONT=1 %s/run.sh   (osmo-operator wiki, Installer-c54x_exe)\n' "$ICI"
}

# Is the ROM of a directory complete and matching rom/SHA256SUMS.<version>?
# (the same test as bon() of rom/fetch-rom.sh, plus the register snapshot)
rom_valide() {
    local s="$ICI/rom/SHA256SUMS.$ROM_VERSION" r
    [ -f "$s" ] || return 1
    for r in $ROMS Registers; do [ -f "$1/calypso_dsp.$r.bin" ] || return 1; done
    ( cd "$1" && sha256sum -c --quiet "$s" ) >/dev/null 2>&1
}
# A copy already verified on this machine, among the destinations and rom/.
rom_source_locale() {
    local d; for d in "${ROM_DIRS[@]}" "$ICI/rom"; do rom_valide "$d" && { printf '%s\n' "$d"; return 0; }; done
    return 1
}

# ── build ─────────────────────────────────────────────────────────────────────
# No "already done": the Makefile recompiles on EVERY call, by design
# (Makefile:46-52, the sources come from another repo).
INST_REGISTER build "Build c54x_exe (qosmo sources)"
_conseil_osmo() {
    if [ -f /usr/local/lib/pkgconfig/libosmocoding.pc ]; then
        printf 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig  (libosmocore is in /usr/local, Dockerfile:92-94)'
    else
        printf 'install libosmocore (sources: https://gitea.osmocom.org/osmocom/libosmocore, or the libosmocore-dev package from your Osmocom repository)'
    fi
}
inst_build_check() {
    _MANQUE=()
    exige "qosmo sources ($QOSMO/hw/arm/calypso/l1-dsp/calypso_c54x.c)" \
        "git clone https://github.com/bbaranoff/qosmO $QOSMO, or --qosmo DIR" \
        test -f "$QOSMO/hw/arm/calypso/l1-dsp/calypso_c54x.c"
    exige "out-of-QEMU shims ($QOSMO/contrib/hors-qemu/cales-qemu.c)" "qosmo too old: git -C $QOSMO pull" \
        test -f "$QOSMO/contrib/hors-qemu/cales-qemu.c"
    exige "C compiler (${CC:-gcc})" "apt-get install build-essential" have_cmd "${CC:-gcc}"
    exige "make" "apt-get install build-essential" have_cmd make
    exige "pkg-config" "apt-get install pkg-config" have_cmd pkg-config
    # The Makefile (line 14) swallows the pkg-config error (2>/dev/null): without
    # this check, the failure only shows at link time, on missing gsm0503_*.
    exige "libosmocoding + libosmocore visible to pkg-config" "$(_conseil_osmo)" \
        pkg-config --exists libosmocoding libosmocore
    exige "$ICI writable (the binary is written there)" "--dest DIR to build a copy elsewhere" test -w "$ICI"
    bilan_exige
}
inst_build_run() {
    # Dockerfile:733-735: cd c54x_exe && make QOSMO=/opt/GSM/qosmo CFLAGS="<without -march=native>"
    local args=("QOSMO=$QOSMO")
    [ "$PORTABLE" = 1 ] && args+=("CFLAGS=$CFLAGS_PORTABLE")
    make -C "$ICI" "${args[@]}" || { inst_fail "build failed"; return $INST_RC_FAIL; }
    inst_ok
}
inst_build_verify() {
    [ -x "$ICI/c54x_exe" ] || { inst_fail "$ICI/c54x_exe missing after make"; return $INST_RC_FAIL; }
    local nf
    nf="$(ldd "$ICI/c54x_exe" 2>/dev/null | awk '/not found/ {print $1}' | tr '\n' ' ')"
    [ -z "$nf" ] || { inst_hint "sudo ldconfig, or LD_LIBRARY_PATH=/usr/local/lib"; inst_fail "libraries not found: $nf"; return $INST_RC_FAIL; }
    inst_ok
}

# ── rom: the TI mask-ROM, downloaded (not in the repo) ────────────────────────
INST_REGISTER rom "DSP ROM $ROM_VERSION (FreeCalypso, checksum verified)"
inst_rom_done() { local d; for d in "${ROM_DIRS[@]}"; do rom_valide "$d" || return 1; done; }
inst_rom_check() {
    _MANQUE=()
    exige "rom/fetch-rom.sh and rom/SHA256SUMS.$ROM_VERSION" "" test -f "$ICI/rom/fetch-rom.sh" -a -f "$ICI/rom/SHA256SUMS.$ROM_VERSION"
    exige "sha256sum" "apt-get install coreutils" have_cmd sha256sum
    local d; for d in "${ROM_DIRS[@]}"; do
        exige "$d writable (ROM)" "sudo, or --rom-dir DIR" ecrivable "$d"
    done
    # The network, python3 and curl are only needed if no valid copy exists anywhere.
    if ! rom_source_locale >/dev/null; then
        exige "python3 (tools/dsp_txt2bin.py)" "apt-get install python3" have_cmd python3
        exige "curl or wget (FreeCalypso download)" "apt-get install curl" sh -c 'command -v curl || command -v wget'
    fi
    bilan_exige
}
inst_rom_run() {
    # 1. A copy already verified on this machine? copy it over, without network.
    #    (the equivalent of cp c54x_exe/rom/*.bin /opt/GSM/ in Dockerfile.run:188 and start-docker.sh)
    local src d r args=()
    if src="$(rom_source_locale)"; then
        for d in "${ROM_DIRS[@]}"; do
            [ "$d" = "$src" ] && continue
            rom_valide "$d" && continue
            mkdir -p "$d" || { inst_fail "cannot create $d"; return $INST_RC_FAIL; }
            for r in $ROMS Registers; do
                install -m644 "$src/calypso_dsp.$r.bin" "$d/calypso_dsp.$r.bin" \
                    || { inst_fail "copy of calypso_dsp.$r.bin to $d failed"; return $INST_RC_FAIL; }
            done
            inst_say "DSP ROM $ROM_VERSION copied from $src to $d (checksum verified)"
        done
    fi
    # 2. rom/fetch-rom.sh does the rest: downloads, converts, verifies, installs
    #    (Dockerfile:736). Everything already in place: it says so and downloads nothing.
    for d in "${ROM_DIRS[@]}"; do args+=(--dest "$d"); done
    bash "$ICI/rom/fetch-rom.sh" --version "$ROM_VERSION" "${args[@]}" \
        || { inst_hint "network? a mirror: FREECALYPSO_URL=..."; inst_fail "rom/fetch-rom.sh failed"; return $INST_RC_FAIL; }
    inst_ok
}
inst_rom_verify() { inst_rom_done && inst_ok || inst_fail "ROM incomplete or checksum mismatch in ${ROM_DIRS[*]}"; }

# ── verify: the DSP alone ─────────────────────────────────────────────────────
# README.md:66: "the ROM alone, without ARM: does it boot?". Outside --arm,
# c54x_exe works on a PRIVATE API RAM: no /dev/shm, no socket, no
# /tmp/c54x-pont - a bench running alongside sees nothing of it. The caller's
# CALYPSO_* (probes, traces) are removed so that it stays that way.
INST_REGISTER verify "Check: c54x_exe --trames 200 (the ROM alone)"
sans_calypso_env() {
    local v u=(); for v in $(compgen -e); do case "$v" in CALYPSO_*) u+=(-u "$v") ;; esac; done
    env "${u[@]+"${u[@]}"}" "$@"
}
inst_verify_check() {
    _MANQUE=()
    etape_retenue build || exige "binary $ICI/c54x_exe" "./install.sh --only build" test -x "$ICI/c54x_exe"
    etape_retenue rom   || exige "DSP ROM in ${ROM_DIRS[0]}" "./install.sh --only rom" rom_valide "${ROM_DIRS[0]}"
    bilan_exige
}
inst_verify_run() {
    local out rc
    out="$(sans_calypso_env timeout 300 "$ICI/c54x_exe" --rom-dir "${ROM_DIRS[0]}" --trames 200 2>&1)"; rc=$?
    printf '%s\n' "$out" | tail -n 14
    [ $rc -eq 0 ] || { inst_fail "c54x_exe --trames 200 returned $rc"; return $INST_RC_FAIL; }
    grep -qE 'sections (chargees|loaded)' <<<"$out" && grep -qE 'bilan sur 200 trames|summary over 200 frames' <<<"$out" \
        || { inst_fail "unexpected output (no 'sections loaded' or no 'summary over 200 frames')"; return $INST_RC_FAIL; }
    inst_ok
}
# ═════════════════════════════════════════════════════════════════════════════
#  THE ENGINE - IDENTICAL copy in the three repos (see THE CONTRACT)
# ═════════════════════════════════════════════════════════════════════════════
#  Same flow as osmo-operator/install.sh: for each retained step,
#  "already done?" (except --reinstall), prerequisites, work, check. The first
#  failure stops everything, says why and shows the tail of the step's log.
#  The component provides: COMPOSANT, PAQUETS, LOG_DIR, the steps, and the
#  hooks options_composant / apres_options / resume_composant /
#  avant_etapes / apres_etapes.
# -----------------------------------------------------------------------------
ACTION=install ONLY="" SKIP="" WITH_DEPS=0
: "${REINSTALL:=0}" "${VERBOSE:=0}"
_ARGS_ORIG=("$@")
# _val OPTION [VALUE]: an option that expects a value must have one.
_val() { [ $# -ge 2 ] && [ -n "$2" ] || { printf '%s: value expected   (--help)\n' "$1" >&2; exit 2; }; }
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
            [ "$_n" -gt 0 ] || { printf 'unknown option: %s   (--help)\n' "$1" >&2; exit 2; }
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

# --- step selection (--only / --skip, comma-separated lists) -----------------
in_csv() { case ",$1," in *",$2,"*) return 0;; esac; return 1; }
for _s in ${ONLY//,/ } ${SKIP//,/ }; do
    [ -n "${INST_DESC[$_s]+x}" ] || { printf 'unknown step: %s   (steps: %s)\n' "$_s" "${INST_ORDER[*]}" >&2; exit 2; }
done
SELECTED=()
for _s in "${INST_ORDER[@]}"; do
    [ -n "$ONLY" ] && ! in_csv "$ONLY" "$_s" && continue
    [ -n "$SKIP" ] && in_csv "$SKIP" "$_s" && continue
    SELECTED+=("$_s")
done
# A step that produces what another requires: under --check, the second one
# is not blamed for the absence of what the first one is about to build.
etape_retenue() { in_csv "$(IFS=,; echo "${SELECTED[*]}")" "$1"; }

# --- apt packages -------------------------------------------------------------
# returns the list of missing ones; code 2 without dpkg (not a Debian/Ubuntu)
paquets_absents() {
    have_cmd dpkg || return 2
    local p; for p in "${PAQUETS[@]}"; do have_pkg "$p" || printf '%s ' "$p"; done
    return 0
}
installer_paquets() {
    local manque rc
    manque="$(paquets_absents)"; rc=$?
    if [ $rc -eq 2 ]; then
        printf 'dpkg missing: install the equivalent of: %s\n' "${PAQUETS[*]}" >&2; return 1
    fi
    if [ -z "${manque// }" ]; then
        printf 'apt packages: all %d present\n' "${#PAQUETS[@]}"; return 0
    fi
    if [ "$(id -u)" -ne 0 ]; then
        printf 'missing apt packages: %s\n  sudo apt-get install -y --no-install-recommends %s\n' "$manque" "$manque" >&2
        return 1
    fi
    printf 'apt-get install : %s\n' "$manque"
    apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $manque || return 1
    manque="$(paquets_absents)"
    [ -z "${manque// }" ] || { printf 'still missing after apt-get: %s\n' "$manque" >&2; return 1; }
}

case "$ACTION" in
    print-deps) printf '%s\n' "${PAQUETS[*]}"; exit 0 ;;
    deps)       installer_paquets; exit $? ;;
    list)
        printf '%s - %d step(s), in this order\n\n' "$COMPOSANT" "${#SELECTED[@]}"
        for _s in "${SELECTED[@]}"; do printf '  %-10s %s\n' "$_s" "${INST_DESC[$_s]}"; done
        printf '\n'; resume_composant | sed 's/^/  /'
        exit 0 ;;
    check)
        printf '%s: prerequisites and state (nothing is modified)\n\n' "$COMPOSANT"
        resume_composant | sed 's/^/  /'
        _manque="$(paquets_absents)"; _rc=$?
        if [ $_rc -eq 2 ]; then printf '\n  apt packages: dpkg missing, not checked (%s)\n' "${PAQUETS[*]}"
        elif [ -n "${_manque// }" ]; then printf '\n  apt packages missing (installed another way? otherwise --deps): %s\n' "$_manque"
        else printf '\n  apt packages: all %d present\n' "${#PAQUETS[@]}"; fi
        _rc=0
        for _s in "${SELECTED[@]}"; do
            _p="inst_${_s//-/_}"
            if ! declare -F "${_p}_done" >/dev/null; then _etat="no check"
            elif "${_p}_done" >/dev/null 2>&1; then _etat="already done"
            else _etat="to do"; fi
            printf '\n  %-10s %s  %s[%s]%s\n' "$_s" "${INST_DESC[$_s]}" "$C_DIM" "$_etat" "$C_Z"
            declare -F "${_p}_check" >/dev/null || continue
            _INST_REASON=""; _INST_HINT=""; _MANQUE=()
            "${_p}_check"; _r=$?
            case $_r in
                "$INST_RC_FAIL") _rc=1 ;;
                "$INST_RC_NA")   printf '      not applicable: %s\n' "$_INST_REASON" ;;
            esac
        done
        printf '\n'
        if [ $_rc -eq 0 ]; then printf '%sprerequisites met%s\n' "$C_OK" "$C_Z"
        else printf '%sprerequisites missing%s (see MISSING above)\n' "$C_KO" "$C_Z"; fi
        exit $_rc ;;
esac

# --- installation ---------------------------------------------------------------
if [ "$WITH_DEPS" = 1 ]; then
    installer_paquets || { printf 'apt packages: failed - nothing was built\n' >&2; exit 1; }
else
    _manque="$(paquets_absents 2>/dev/null)"
    [ -n "${_manque// }" ] && printf '%snote: apt packages missing (installed another way? otherwise --with-deps): %s%s\n' "$C_DIM" "$_manque" "$C_Z"
fi
if [ "$VERBOSE" != 1 ]; then
    mkdir -p "$LOG_DIR" || { printf 'cannot create log: %s (LOG_DIR=...)\n' "$LOG_DIR" >&2; exit 1; }
fi
# _jouer FUNCTION: the output goes to the step's log, or to the screen with -v.
# No pipe: the function runs in THIS shell (its failure reason survives).
_jouer() { if [ "$VERBOSE" = 1 ]; then "$1"; else "$1" >>"$_log" 2>&1; fi; }
_echec() {   # _echec STEP REASON [log]: says everything known, and stops
    end FAIL "$C_KO" "${INST_DESC[$1]}" "$2"
    [ -n "$_INST_HINT" ] && printf '       → %s\n' "$_INST_HINT"
    # the tail of the log, when it is the work (run, verify) that failed
    if [ -n "${3:-}" ] && [ "$VERBOSE" != 1 ] && [ -s "$_log" ]; then
        printf '       %slog: %s (tail below)%s\n' "$C_DIM" "$_log" "$C_Z"
        tail -n 25 "$_log" | sed 's/^/       | /'
    fi
    printf '\n%s: installation stopped at step %s\n' "$COMPOSANT" "$1"
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
        end SKIP "$C_SK" "${INST_DESC[$_s]}" "already done"; _nb_skip=$((_nb_skip + 1)); continue
    fi
    if declare -F "${_p}_check" >/dev/null; then
        _jouer "${_p}_check"; _r=$?
        case $_r in
            "$INST_RC_OK")   ;;
            "$INST_RC_NA")   end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-not applicable}"
                             _nb_skip=$((_nb_skip + 1)); continue ;;
            *)               _echec "$_s" "${_INST_REASON:-prerequisite not met}" ;;
        esac
    fi
    _jouer "${_p}_run"; _r=$?
    case $_r in
        "$INST_RC_OK")   ;;
        "$INST_RC_DONE"|"$INST_RC_NA")
                         end SKIP "$C_SK" "${INST_DESC[$_s]}" "${_INST_REASON:-nothing to do}"
                         _nb_skip=$((_nb_skip + 1)); continue ;;
        *)               _echec "$_s" "${_INST_REASON:-failed}" journal ;;
    esac
    # installing is not having installed: checked after the fact
    if declare -F "${_p}_verify" >/dev/null && ! _jouer "${_p}_verify"; then
        _echec "$_s" "done, but the check fails: ${_INST_REASON:-}" journal
    fi
    end " OK " "$C_OK" "${INST_DESC[$_s]}" "${_INST_REASON:-}"; _nb_ok=$((_nb_ok + 1))
done
printf '\n%s: %d ok · %d skipped · 0 failed\n' "$COMPOSANT" "$_nb_ok" "$_nb_skip"
apres_etapes
exit 0
