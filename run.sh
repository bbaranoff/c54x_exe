#!/usr/bin/env bash
# run.sh - le cote MOBILE du banc Calypso, processus par processus, sans le reseau.
#
# Deux montages (MODE) :
#   dsp    (defaut)  1. c54x_exe --arm   le DSP TMS320C54x, mask-ROM TI, hors QEMU
#                    2. qosmo            l'ARM + layer1 osmocom-bb (CALYPSO_DSP_EXTERN=1)
#                    3. osmocon          romload sur le pty serial0, relais L1CTL
#                    4. mobile           couche 2/3 osmocom-bb
#                    5. pont.py          (PONT=1) bursts du BTS -> DSP (--dsp-port 6702)
#   grgsm            2. qosmo            l'ARM + layer1 avec la couche 1 gr-gsm (shunt)
#                    3. osmocon  4. mobile  5. pont.py (PONT=1) bursts du BTS -> L1 gr-gsm
# Le pont a besoin du BTS (osmo-bts-trx, TRXD 5700) : sans reseau il demarre et attend.
#
#   ./run.sh                 tout lancer            ./run.sh --status    qui tourne
#   MODE=grgsm ./run.sh      montage gr-gsm         ./run.sh --logs      suivre les journaux
#   PONT=1 ./run.sh          avec le pont           ./run.sh --stop      tout arreter, nettoyer
#   ./run.sh --step N        une seule etape (les precedentes doivent tourner)
# Variables : MODE, PONT, LOCKSTEP (1 : QEMU attend le DSP a chaque trame), INSNS (200000),
#   VERB (-v), IQ (none|fcch|cell|...), AMP (30000),
#   QOSMO, FIRMWARE_ELF, FIRMWARE_BIN, OSMOCON, MOBILE, MOBILE_CFG, PONT_PY, RUNDIR, L2_SOCK.
# Details, attendus et verifications : LAUNCH.md a cote.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${MODE:-dsp}"
PONT="${PONT:-0}"
QOSMO="${QOSMO:-/opt/GSM/qosmo}"
QEMU="${QEMU:-$QOSMO/build/qemu-system-arm}"
FIRMWARE_ELF="${FIRMWARE_ELF:-/opt/GSM/firmware/board/compal_e88/layer1.highram.elf}"
FIRMWARE_BIN="${FIRMWARE_BIN:-${FIRMWARE_ELF%.elf}.bin}"
OSMOCON="${OSMOCON:-/opt/GSM/osmocom-bb/src/host/osmocon/osmocon}"
MOBILE="${MOBILE:-$(command -v mobile || echo /usr/local/bin/mobile)}"
MOBILE_CFG="${MOBILE_CFG:-$HERE/mobile_pont.cfg}"
# [2026-09-23] Deux points d'entree pour le meme paquet pont/ : pont.py pour le
# montage dsp (--dsp-port 6702), pont_uncipher.py pour le montage grgsm, qui
# garde ses defauts d'avant le decoupage. PONT_PY force l'un ou l'autre.
if [ "$MODE" = grgsm ]; then
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont_uncipher.py}"
else
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont.py}"
fi
RUNDIR="${RUNDIR:-/tmp/c54x-pont}"
L2_SOCK="${L2_SOCK:-/tmp/osmocom_l2}"
MONITOR="${MONITOR:-/tmp/qemu-monitor-pont.sock}"
INSNS="${INSNS:-200000}"
# [2026-09-20] Pas-a-pas DSP/QEMU par defaut (LOCKSTEP=0 pour le mode horloge murale) :
# le C54x emule coute ~6,7 ms par trame contre 4,615 ms de temps reel, QEMU sautait
# donc 3 trames sur 4 (« DSP en retard, tick saute »), la ROM ne voyait qu'une trame
# sur 4 a 6, son compteur de blocs FB n'avancait pas et le TOA valait 1251 quelle que
# soit la distance de la FCCH. En pas-a-pas QEMU n'avance la trame que quand le DSP
# a fini la precedente : TOA = 23 + n x 1250, delay=10, SB decodable.
LOCKSTEP="${LOCKSTEP:-1}"
[ "$MODE" = dsp ] && [ "$LOCKSTEP" = 1 ] && export CALYPSO_PONT_LOCKSTEP=1
VERB="${VERB:--v}"
IQ="${IQ:-none}"
AMP="${AMP:-30000}"
DSP_SOCK=/tmp/calypso_dsp.sock
DSP_SHM=/dev/shm/calypso_api_ram

# [2026-09-23] LA VITRINE : LES MEMES JOURNAUX QU'EN MODE SHUNT. Le panneau
# (tmux calypso, osmo-fft-snap) suit /run/user/0/osmo-nitb/logs/{qemu,osmocon,
# mobile}.log et /dev/shm/pont.log, ceux qu'ecrit le montage grgsm. Ce banc-ci
# n'ecrivait que dans $RUNDIR : les volets restaient a 0 octet. Le VRAI fichier
# est donc pose au chemin du panneau et $RUNDIR/<nom>.log devient un lien vers
# lui -- pas l'inverse : un `tail -F` deja ouvert refuse un fichier remplace par
# un lien (« untailable symbolic link »). PANNEAU_LOGS=none pour ne rien toucher.
PANNEAU_LOGS="${PANNEAU_LOGS:-/run/user/0/osmo-nitb/logs}"
# La FFT du panneau lit /tmp/iq_fft_ms.fifo, que pont.py ne remplit que si
# l'enregistrement est actif : --no-record la rendait muette. PONT_AIRREC=0 le coupe.
PONT_AIRREC="${PONT_AIRREC:-1}"
vitrine() {   # vitrine <nom> : a appeler AVANT le lancement qui ecrit $RUNDIR/<nom>.log
    rm -f "$RUNDIR/$1.log"
    [ "$PANNEAU_LOGS" = none ] && return 0
    local cible="$PANNEAU_LOGS/$1.log"
    [ "$1" = pont ] && cible=/dev/shm/pont.log
    [ -d "$(dirname "$cible")" ] || return 0
    rm -f "$cible" && : > "$cible" && ln -s "$cible" "$RUNDIR/$1.log"
    return 0
}

dire()  { printf '\033[1m[run %s]\033[0m %s\n' "$MODE" "$*"; }
rater() { printf '\033[1;31m[run] ECHEC :\033[0m %s\n' "$*" >&2; exit 1; }
pid_de() { [ -f "$RUNDIR/$1.pid" ] && cat "$RUNDIR/$1.pid"; }
vivant() { local p; p="$(pid_de "$1")"; [ -n "$p" ] && kill -0 "$p" 2>/dev/null; }
attendre() { local n=$(( $1 * 10 )); shift; while [ "$n" -gt 0 ]; do "$@" && return 0; sleep 0.1; n=$((n - 1)); done; return 1; }
[ "$MODE" = dsp ] || [ "$MODE" = grgsm ] || rater "MODE=$MODE inconnu (dsp|grgsm)"

etape1() {   # le DSP (montage dsp seulement)
    [ "$MODE" = dsp ] || { dire "1. (montage grgsm : pas de c54x_exe, la couche 1 est dans QEMU)"; return; }
    vivant dsp && { dire "1. c54x_exe deja lance (pid $(pid_de dsp))"; return; }
    [ -x "$HERE/c54x_exe" ] || make -C "$HERE" >/dev/null || rater "make c54x_exe"
    rm -f "$DSP_SHM" "$DSP_SOCK"
    ( cd "$HERE" && CALYPSO_IQDUMP_FCCH=1 exec ./c54x_exe --arm --insns "$INSNS" --iq "$IQ" --amp "$AMP" $VERB ) > "$RUNDIR/dsp.log" 2>&1 &
    echo $! > "$RUNDIR/dsp.pid"
    attendre 5 test -S "$DSP_SOCK" || rater "c54x_exe n'a pas ouvert $DSP_SOCK (voir $RUNDIR/dsp.log)"
    dire "1. c54x_exe --arm  pid $(pid_de dsp)  ($INSNS insn/trame, iq=$IQ, $DSP_SHM, $DSP_SOCK)"
}

etape2() {   # l'ARM
    vivant qemu && { dire "2. QEMU deja lance (pid $(pid_de qemu))"; return; }
    [ -x "$QEMU" ] || rater "QEMU absent : $QEMU (ninja -C $QOSMO/build)"
    [ -r "$FIRMWARE_ELF" ] || rater "firmware absent : $FIRMWARE_ELF"
    local extern=""
    if [ "$MODE" = dsp ]; then [ -S "$DSP_SOCK" ] || rater "le DSP ne tourne pas (etape 1 d'abord)"; extern=1; fi
    rm -f "$MONITOR"
    vitrine qemu
    CALYPSO_DSP_EXTERN="$extern" "$QEMU" -M calypso -cpu arm946 -display none -parallel none \
        -serial pty -serial pty -monitor "unix:$MONITOR,server,nowait" \
        -kernel "$FIRMWARE_ELF" > "$RUNDIR/qemu.log" 2>&1 &
    echo $! > "$RUNDIR/qemu.pid"
    attendre 10 grep -aq "label serial0" "$RUNDIR/qemu.log" || rater "QEMU n'a pas publie son pty (voir $RUNDIR/qemu.log)"
    if [ "$MODE" = dsp ]; then
        grep -aq "pont DSP : API RAM partagee" "$RUNDIR/qemu.log" || rater "QEMU n'a pas rejoint le DSP (voir $RUNDIR/qemu.log)"
    else
        attendre 5 grep -aq "backend gr-gsm" "$RUNDIR/qemu.log" || rater "la couche 1 gr-gsm ne s'est pas annoncee (voir $RUNDIR/qemu.log)"
    fi
    sed -n 's/.*redirected to \(\/dev\/pts\/[0-9]*\) (label serial0).*/\1/p' "$RUNDIR/qemu.log" | head -1 > "$RUNDIR/modem.pty"
    dire "2. qemu-system-arm  pid $(pid_de qemu)  pty modem $(cat "$RUNDIR/modem.pty")  moniteur $MONITOR  L1=$([ "$MODE" = dsp ] && echo "DSP externe" || echo "gr-gsm (udp 4730/4731)")"
}

etape3() {   # osmocon
    vivant osmocon && { dire "3. osmocon deja lance (pid $(pid_de osmocon))"; return; }
    vivant qemu || rater "QEMU ne tourne pas (etape 2 d'abord)"
    [ -x "$OSMOCON" ] || rater "osmocon absent : $OSMOCON"
    [ -r "$FIRMWARE_BIN" ] || rater "image .bin absente : $FIRMWARE_BIN"
    local pty; pty="$(cat "$RUNDIR/modem.pty")"
    rm -f "$L2_SOCK"
    vitrine osmocon
    stdbuf -oL -eL "$OSMOCON" -m romload -i 100 -p "$pty" -s "$L2_SOCK" "$FIRMWARE_BIN" > "$RUNDIR/osmocon.log" 2>&1 &
    echo $! > "$RUNDIR/osmocon.pid"
    if ! attendre 30 grep -aq "your code is running now" "$RUNDIR/osmocon.log"; then
        rater "osmocon n'a pas fini le romload (voir $RUNDIR/osmocon.log). Un osmocon tue a mi-bloc laisse
       le stub romload de l'UART en attente : ./run.sh --stop puis ./run.sh"
    fi
    dire "3. osmocon  pid $(pid_de osmocon)  L1CTL sur $L2_SOCK"
}

etape4() {   # le mobile
    vivant mobile && { dire "4. mobile deja lance (pid $(pid_de mobile))"; return; }
    [ -S "$L2_SOCK" ] || rater "pas de socket L1CTL $L2_SOCK (etape 3 d'abord)"
    [ -x "$MOBILE" ] || rater "mobile absent : $MOBILE"
    [ -r "$MOBILE_CFG" ] || rater "config mobile absente : $MOBILE_CFG"
    local vty; vty="$(sed -n 's/^ *bind 127.0.0.1 \([0-9]*\).*/\1/p' "$MOBILE_CFG" | head -1)"
    if [ -n "$vty" ] && ss -ltn 2>/dev/null | grep -q ":$vty "; then
        rater "le port VTY $vty de $MOBILE_CFG est deja pris par : $(ss -ltnp 2>/dev/null | grep ":$vty " | grep -o 'users:.*' | head -1)
       changez la ligne « bind 127.0.0.1 $vty » de la config (les 42xx sont ceux du reseau du banc)"
    fi
    vitrine mobile
    stdbuf -oL "$MOBILE" -c "$MOBILE_CFG" > "$RUNDIR/mobile.log" 2>&1 &
    echo $! > "$RUNDIR/mobile.pid"
    sleep 2
    vivant mobile || rater "mobile s'est arrete : $(sed 's/\x1b\[[0-9;]*m//g' "$RUNDIR/mobile.log" | grep -iE 'cannot|error|unable' | tail -1)"
    dire "4. mobile  pid $(pid_de mobile)  config $MOBILE_CFG  vty telnet 127.0.0.1 ${vty:-?}"
}

etape5() {   # le pont TRX (PONT=1) : bursts du BTS vers la couche 1
    [ "$PONT" = 1 ] || { dire "5. (PONT=0 : pas de pont.py, aucun burst du BTS n'arrive)"; return; }
    vivant pont && { dire "5. pont.py deja lance (pid $(pid_de pont))"; return; }
    [ -r "$PONT_PY" ] || rater "pont.py absent : $PONT_PY"
    local extra=""; [ "$MODE" = dsp ] && extra="--dsp-port 6702"
    [ "$PONT_AIRREC" = 0 ] && extra="$extra --no-record"
    vitrine pont
    ( cd "$(dirname "$PONT_PY")/.." && exec python3 "$PONT_PY" $extra ) > "$RUNDIR/pont.log" 2>&1 &
    echo $! > "$RUNDIR/pont.pid"
    attendre 10 grep -aq "pont TRX : ports" "$RUNDIR/pont.log" || rater "pont.py ne s'est pas annonce (voir $RUNDIR/pont.log)"
    dire "5. pont.py  pid $(pid_de pont)  TRXD 5700-5702 <- BTS ; vers $([ "$MODE" = dsp ] && echo "le DSP (udp 6702)" || echo "la L1 gr-gsm (udp 4730/4731)")"
}

arreter() {
    local n
    for n in pont mobile osmocon qemu dsp; do
        if vivant "$n"; then kill "$(pid_de "$n")" 2>/dev/null; dire "arret $n (pid $(pid_de "$n"))"; fi
        rm -f "$RUNDIR/$n.pid"
    done
    sleep 1
    rm -f "$DSP_SHM" "$DSP_SOCK" "$L2_SOCK" "$MONITOR" "$RUNDIR/modem.pty"
    # [2026-09-22] Sockets du mobile : elles ne disparaissent pas avec lui. Le
    # « sap » etait deja oublie, et « ms_data » (tch-data, mobile_pont.cfg)
    # arrive avec la meme faiblesse : un fichier reste fait echouer le bind du
    # run suivant sur EADDRINUSE, et le mobile demarre sans sa voie donnees
    # sans le dire.
    rm -f /tmp/osmocom_sap /tmp/ms_data
    # [2026-09-22] Horloge du banc (calypso_bsp.c -> pont/trx.py) : laissee en
    # place, pont.py asservirait sa premiere seconde sur la trame d'une session
    # morte et la BTS resterait figee en attendant un DSP qui n'existe plus.
    rm -f /dev/shm/calypso_horloge
    # Side-bands du lien montant (src/montant.c) : sinon pont.py relit le
    # dernier enregistrement d'une session precedente au demarrage.
    rm -f /dev/shm/calypso_rach /dev/shm/calypso_sdcch_ul \
          /dev/shm/calypso_tch_facch_ul /dev/shm/calypso_tch_sacch_ul \
          /dev/shm/calypso_tch_ul
}

statut() {
    local n
    for n in dsp qemu osmocon mobile pont; do
        if vivant "$n"; then printf '  %-8s pid %-7s  %s\n' "$n" "$(pid_de "$n")" "$RUNDIR/$n.log"
        else printf '  %-8s arrete\n' "$n"; fi
    done
    [ -f "$RUNDIR/dsp.log" ] && grep -a "fn=" "$RUNDIR/dsp.log" | tail -1
    [ -f "$RUNDIR/osmocon.log" ] && grep -a "FB0\|FB1\|SB\|BSIC" "$RUNDIR/osmocon.log" | tail -1
}

mkdir -p "$RUNDIR"
case "${1:-}" in
    --stop)   arreter ;;
    --status) statut ;;
    --logs)   exec tail -n 5 -F "$RUNDIR"/dsp.log "$RUNDIR"/qemu.log "$RUNDIR"/osmocon.log "$RUNDIR"/mobile.log "$RUNDIR"/pont.log 2>/dev/null ;;
    --step)   case "${2:-}" in 1) etape1;; 2) etape2;; 3) etape3;; 4) etape4;; 5) etape5;; *) rater "--step 1|2|3|4|5";; esac ;;
    -h|--help) sed -n '2,22p' "$0" ;;
    "")       etape1; etape2; etape3; etape4; etape5
              dire "tout tourne. Journaux : $RUNDIR/*.log   suivre : ./run.sh --logs   arreter : ./run.sh --stop" ;;
    *)        rater "option inconnue : $1 (voir --help)" ;;
esac
