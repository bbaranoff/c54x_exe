#!/usr/bin/env bash
# run.sh - the MOBILE side of the Calypso bench, process by process, without the network.
#
# Two setups (MODE):
#   dsp    (default) 1. c54x_exe --arm   the TMS320C54x DSP, TI mask-ROM, outside QEMU
#                    2. qosmo            the ARM + osmocom-bb layer1 (CALYPSO_DSP_EXTERN=1)
#                    3. osmocon          romload on the serial0 pty, L1CTL relay
#                    4. mobile           osmocom-bb layer 2/3
#                    5. pont_dsp.py      (PONT=1) BTS bursts -> DSP (--dsp-port 6702);
#                                        the DSP bridge (pont/dsp/), TCH switch followed from the firmware
#   grgsm            2. qosmo            the ARM + layer1 with the gr-gsm layer 1 (shunt)
#                    3. osmocon  4. mobile  5. pont.py (PONT=1) BTS bursts -> gr-gsm L1
# The bridge needs the BTS (osmo-bts-trx, TRXD 5700): without network it starts and waits.
#   OSMO_TRX=1       THE PHONE IS THE TRX OF BTS#0 (start-native.sh --osmo-trx):
#                    2. qosmo on trx.highram.elf (osmocom-bb-transceiver, "trx" app by Sylvain Munaut)
#                    4. transceiver (host, osmocom-bb-transceiver, patches/osmocom-bb-transceiver-qemu-bench)
#                       INSTEAD OF mobile: L1CTL on $L2_SOCK, TRXD 5700-5702 facing osmo-bts-trx,
#                       --no-sync (no cell to listen to), a single RX timeslot (OSMO_TRX_TS_MASK=0x01);
#                       the TX mask stays 0xe3 (TS 5,6,7,0,1): prim_bts.c only serves the TS0 BCCH if
#                       tx_start == 5 (TX window shifted by 3 slots). -m 0x01 would also set
#                       TX=0x01 -> tx_start 0 -> dummy on every frame (DUMMY only, FB 0, SB 0).
#                    5. no bridge: it would hold the same ports 5700-5702 (PONT forced to 0)
#                    OSMO_TRX_ARGS (default "--no-sync -e 3", + "-r $OSMO_TRX_RT" SCHED_RR 20), TRANSCEIVER (binary path).
#   INSTANCE=2       A SECOND COMPLETE CHAIN next to the first (banc-mobile while banc-trx runs):
#                    run.sh re-executes itself in a private mount namespace where /dev/shm IS
#                    /dev/shm/calypso2 (as seen from the host), and the host's real /dev/shm is
#                    visible at /run/calypso-shm-hote; no /dev/shm/... path in the code changes.
#                    RUNDIR /tmp/c54x-pont2, L1CTL /tmp/osmocom_l2_2, DSP socket /tmp/calypso_dsp2.sock,
#                    monitor, gdb 1235/44445, keyboard /tmp/calypso_keys2.sock, BSP udp 6712, mobile VTY
#                    +100, 4347 (copy of mobile_pont.cfg; 4257 belongs to osmo-ggsn), PANNEAU_LOGS=none. AIR=1 (default in instance 2):
#                    step 5 = pont/air.py, the air between the two Calypsos (TX bursts of one to the BSP
#                    of the other) instead of the bridge.
#   FIRMWARE_APP=rssi  a firmware app ALONE on the ARM + the DSP (rssi, menu, hello_world,
#                    snake_game...: board/compal_e88/<app>.highram.elf): steps 1-3, neither mobile
#                    nor bridge. The C123 screen and keyboard are emulated (qosmo:
#                    calypso_st7558.c -> /dev/shm/calypso_lcd, calypso_armio.c <- /tmp/calypso_keys.sock);
#                    the window: /usr/bin/python3 /opt/GSM/osmo-operator/tools/osmo-c123-gtk.py
#
#   ./run.sh                 launch everything      ./run.sh --status    what is running
#   MODE=grgsm ./run.sh      gr-gsm setup           ./run.sh --logs      follow the logs
#   PONT=1 ./run.sh          with the bridge        ./run.sh --stop      stop everything, clean up
#   ./run.sh --step N        a single step (the previous ones must be running)
# Variables: MODE, PONT, LOCKSTEP (1: QEMU waits for the DSP on every frame), INSNS (80000),
#   VERB (-v), IQ (none|fcch|cell|...), AMP (30000),
#   QOSMO, FIRMWARE_ELF, FIRMWARE_BIN, OSMOCON, MOBILE, MOBILE_CFG, PONT_PY, RUNDIR, L2_SOCK.
#   UL_RAW (1: the bridge sends the ROM's final bursts as they are; 0: host coding), MONTANT_EFFACE_TASK_RA.
#   Real time: RT_NICE (-10 by default, empty = none), RT_FIFO (SCHED_FIFO, e.g. 20, root), RT_CPUS (taskset,
#   e.g. 1-3), RT_RENICE_AUTRES (e.g. 15: renice of the panel tools); ./run.sh --charge  who consumes what.
# Details, expectations and checks: LAUNCH.md alongside. Noise (BRUIT_MODE, inactive by default): BRUIT_ block at the end.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${MODE:-dsp}"
PONT="${PONT:-0}"
# INSTANCE=N (N != 1): private /dev/shm = the host's /dev/shm/calypsoN; the real /dev/shm stays
# readable at /run/calypso-shm-hote (the air reads the TX ring of instance 1 there). The processes
# launched here inherit the namespace; --stop/--status re-run with INSTANCE=N enter it too.
INSTANCE="${INSTANCE:-1}"
# PER-INSTANCE SETTINGS: instances/instance-$INSTANCE.env (written by the web page tools/osmo-reglages.py,
# template instances/instance.env.example). `: "${VAR:=value}"` lines: the environment (start-native.sh, command
# line) keeps priority, then the file, then the defaults below. Exported (set -a) for c54x_exe/QEMU/bridge.
INSTANCE_ENV="${INSTANCE_ENV:-/opt/GSM/osmo-operator/instances/instance-$INSTANCE.env}"
if [ -r "$INSTANCE_ENV" ]; then
    set -a; . "$INSTANCE_ENV"; set +a
    printf '\033[1m[run dsp]\033[0m instance %s : reglages de %s\n' "$INSTANCE" "$INSTANCE_ENV"
fi
HOTE_SHM=/run/calypso-shm-hote
AIR_DEMANDE="${AIR:-}"   # AIR set by the caller (otherwise default per instance / app, below)
if [ "$INSTANCE" != 1 ] && [ -z "${CALYPSO_NS:-}" ]; then
    command -v unshare >/dev/null || { echo "[run] ECHEC : unshare absent (util-linux)" >&2; exit 1; }
    _d="/dev/shm/calypso$INSTANCE"; mkdir -p "$_d" "$HOTE_SHM" && chmod 1777 "$_d"
    exec unshare -m --propagation private bash -c '
        mount --bind /dev/shm "$4" && mount --bind "$1" /dev/shm && export CALYPSO_NS="$2" && exec bash "$3" "${@:5}"' \
        _ "$_d" "$INSTANCE" "$0" "$HOTE_SHM" "$@"
fi
if [ "$INSTANCE" != 1 ]; then
    RUNDIR="${RUNDIR:-/tmp/c54x-pont$INSTANCE}"
    L2_SOCK="${L2_SOCK:-/tmp/osmocom_l2_$INSTANCE}"
    MONITOR="${MONITOR:-/tmp/qemu-monitor-pont$INSTANCE.sock}"
    DSP_SOCK="${DSP_SOCK:-/tmp/calypso_dsp$INSTANCE.sock}"
    GDB_STUB="${GDB_STUB:-$((1233 + INSTANCE))}"
    GDB_TELNET="${GDB_TELNET:-$((44443 + INSTANCE))}"
    PANNEAU_LOGS="${PANNEAU_LOGS:-none}"
    # instance >= 3: +30, because 6722 (= 6702 + 20) is the "MS" TRXD port of fake_trx.py (-p 6720, sidecar of
    # BTS 1): bound on 127.0.0.1:6722 it would steal the uplink bursts of phone 2 (instance 3: 6752).
    export CALYPSO_BSP_PORT="${CALYPSO_BSP_PORT:-$((6702 + 10 * (INSTANCE - 1) + (INSTANCE >= 3 ? 30 : 0)))}"
    export CALYPSO_KEYS_SOCK="${CALYPSO_KEYS_SOCK:-/tmp/calypso_keys$INSTANCE.sock}"
    AIR="${AIR:-1}"
fi
AIR="${AIR:-0}"
AIR_PY="${AIR_PY:-/opt/GSM/osmo-operator/pont/air.py}"
QOSMO="${QOSMO:-/opt/GSM/qosmo}"
QEMU="${QEMU:-$QOSMO/build/qemu-system-arm}"
# Firmware: the original layer1.highram, unmodified. FIRMWARE_ELF=... for another binary.
_FW=/opt/GSM/firmware/board/compal_e88
# OSMO_TRX=1: trx.highram firmware (the Calypso transmits the BTS), host app transceiver.
OSMO_TRX="${OSMO_TRX:-0}"
FIRMWARE_APP="${FIRMWARE_APP:-}"
if [ -n "$FIRMWARE_APP" ]; then
    FIRMWARE_ELF="$_FW/$FIRMWARE_APP.highram.elf"   # also overrides /etc/default/osmo-banc
    FIRMWARE_BIN="${FIRMWARE_ELF%.elf}.bin"
    PONT=0
fi
# RSSI: realistic air by default (calypso_bsp.c CALYPSO_AIR_NIVEAUX: the cell only on its ARFCN, DL/UL
# levels, noise elsewhere) and, with a classic BTS (pont_dsp.py alive), no air.py: the downlink arrives
# through the bridge's copy (PONT_DSP_COPIES, udp 6762). Set here and not in banc-rssi, so that any
# launcher gets the same behaviour.
if [ "$FIRMWARE_APP" = rssi ]; then
    export CALYPSO_AIR_NIVEAUX="${CALYPSO_AIR_NIVEAUX:-1}"
    export CALYPSO_FAUX_AIR="${CALYPSO_FAUX_AIR:-$( . /etc/osmocom/radio-plan.env 2>/dev/null; echo "${PLAN_ARFCN:-}")}"
    [ -z "$AIR_DEMANDE" ] && pgrep -f 'pont/pont_dsp\.py' >/dev/null && AIR=0
fi
# OSMO_TRX_TS1=1 (instance 1, with OSMO_TRX=1): a SECOND Calypso TRX for TS1 (TCH/F, the calls): the
# instance OSMO_TRX_TS1_INSTANCE (3) is launched before the transceiver (steps 1-3 only, OSMO_TRX_PHONE=2:
# trx firmware, L1CTL /tmp/osmocom_l2_3, no transceiver, no air) and the transceiver takes
# "-2 -L /tmp/osmocom_l2_3" (phone 2 = TS1 TX+RX, prim_bts.c SLOT 0 / tn 1). Its BSP (udp 6752) has no
# SB to align on: CALYPSO_BSP_TRX_OFFSET=-1 (firmware fn = tick - 1, measured on phone 1). The air
# (instance 2) also connects this ring (--c-ring/--c-port).
# OSMO_TRX_TS1=0 by default: the 2nd phone has an independent fn (downlink TCH ~80 bits wrong for the
# mobile, ASSIGNMENT FAILURE); only the ONE Calypso TS0+TS1 setup (OSMO_TRX_TCH=1) completes a call.
# start-native.sh --ts 2 (or OSMO_TRX_TS1=1) for the two-phone setup.
OSMO_TRX_TS1="${OSMO_TRX_TS1:-0}"
OSMO_TRX_TS1_INSTANCE="${OSMO_TRX_TS1_INSTANCE:-3}"
OSMO_TRX_PHONE="${OSMO_TRX_PHONE:-1}"
# The mobile over the air (INSTANCE=N, AIR=1) is never the TRX: start-native.sh --bts exports OSMO_TRX=1
# and instance 2 must not inherit it (trx firmware + second transceiver -> "UDP port 5700 already taken").
if [ "$OSMO_TRX" = 1 ] && [ "$AIR" = 1 ] && [ "$INSTANCE" != 1 ] && [ "$OSMO_TRX_PHONE" != 2 ]; then
    printf '\033[1m[run dsp]\033[0m INSTANCE=%s AIR=1 : OSMO_TRX=1 herite ignore, ce Calypso est le mobile (le TRX est l instance 1)\n' "$INSTANCE"
    OSMO_TRX=0
fi
if [ "$OSMO_TRX" = 1 ]; then
    # The trx firmware OVERRIDES: /etc/default/osmo-banc (GPRS profile) sets FIRMWARE_ELF=layer1.highram.gprs
    # in the environment, and a "${FIRMWARE_ELF:-trx}" would let it through: the transceiver would speak
    # BTS_MODE (0x1f) to a layer1 that does not know it. OSMO_TRX_FIRMWARE=<elf> for another trx.
    [ -n "${FIRMWARE_ELF:-}" ] && [ "${FIRMWARE_ELF##*/}" != "${OSMO_TRX_FIRMWARE:-trx.highram.elf}" ] \
        && printf '\033[1m[run dsp]\033[0m OSMO_TRX=1 : firmware %s ignore, trx.highram.elf a la place\n' "$FIRMWARE_ELF"
    FIRMWARE_ELF="${OSMO_TRX_FIRMWARE:-$_FW/trx.highram.elf}"
    FIRMWARE_BIN="${FIRMWARE_ELF%.elf}.bin"
    # BSIC of the cell: osmo-bts-trx only sends SETTSC (BCC), the NCC comes from bts 0 of the BSC
    OSMO_TRX_BSIC="${OSMO_TRX_BSIC:-$(awk '/^ *bts 0$/{f=1} f&&/base_station_id_code/{print $2; exit}' /etc/osmocom/osmo-bsc.cfg 2>/dev/null)}"
    TRANSCEIVER="${TRANSCEIVER:-/opt/GSM/osmocom-bb-transceiver/src/host/layer23/src/transceiver/transceiver}"
    [ -x "$TRANSCEIVER" ] || TRANSCEIVER="$(command -v transceiver || echo "$TRANSCEIVER")"
    # OSMO_TRX_TCH=1 (default): the phone serves TS0 AND TS1 (TCH/F, the calls): masks 0x03, the patched
    # firmware transmits both slots of its TX window (prim_bts.c) and reports TS1 from rx[2]; c54x_exe
    # publishes both bursts from the page (CALYPSO_TRX_TX_SLOTS=2) and writes the TS1 uplink into the
    # page. OSMO_TRX_TCH=0: TS0 only (0x01).
    # with phone 2 (OSMO_TRX_TS1=1) phone 1 keeps TS0 only: OSMO_TRX_TCH=0
    OSMO_TRX_TCH="${OSMO_TRX_TCH:-$([ "$OSMO_TRX_TS1" = 1 ] && echo 0 || echo 1)}"
    if [ "$OSMO_TRX_TCH" = 1 ] && [ "$OSMO_TRX_PHONE" != 2 ]; then
        OSMO_TRX_TS_MASK="${OSMO_TRX_TS_MASK:-0x03}"
        OSMO_TRX_TX_MASK="${OSMO_TRX_TX_MASK:-0x03}"
        export CALYPSO_TRX_TX_SLOTS="${CALYPSO_TRX_TX_SLOTS:-2}"
    fi
    OSMO_TRX_TS_MASK="${OSMO_TRX_TS_MASK:-0x01}"
    # TX mask 0x01: ONE TX window (tx_start 0, TPU slot 5 = downlink TS0), the one the ROM plays as a
    # RACH and that c54x_exe captures (TSP script 0x3cbb). The original 0xe3 (multiple window 5,6,7,0,1)
    # is not served by the bench: only the first burst (dummy) would go out. The fixed trx firmware
    # (prim_bts.c, SLOT/tn) targets TS0 whatever tx_start.
    OSMO_TRX_TX_MASK="${OSMO_TRX_TX_MASK:-0x01}"
    # -r PRIO (SCHED_RR, main.c rt_prio; OSMO_TRX_RT= empty to do without); -e 1 for debugging (-e 3 by default)
    OSMO_TRX_RT="${OSMO_TRX_RT-20}"
    OSMO_TRX_ARGS="${OSMO_TRX_ARGS:---no-sync -e 3}${OSMO_TRX_RT:+ -r $OSMO_TRX_RT}"
    PONT=0
    # the BTS repeats its bursts (dummy, FB): c54x_exe publishes every frame with a TX task, not just new ones
    export MONTANT_ROM_TX_CHAQUE="${MONTANT_ROM_TX_CHAQUE:-1}"
    # the BSP plays the uplink bursts (air.py, GSM fn of the mobile) at the trx firmware's frame, aligned on
    # the fn of the transmitted SB (pont.c -> calypso_bsp_trx_calage); CALYPSO_BSP_TRX_DELTA adjusts (default -2
    # with the patched firmware BTS_RX_LAG 3: uplink burst fn X is played one frame after the native window,
    # margin for the air)
    export CALYPSO_BSP_TRX="${CALYPSO_BSP_TRX:-1}"
    # trx firmware BTS_RX_LAG 12 (prim_bts.c): uplink burst fn X is played 11 frames after its native
    # window; margin for the round-trip latency of the air + the lag of the emulated mobile
    export CALYPSO_BSP_TRX_DELTA="${CALYPSO_BSP_TRX_DELTA:--11}"
    # phone 2 (TS1): no SB transmitted, fixed fn/tick offset (see OSMO_TRX_TS1)
    [ "$OSMO_TRX_PHONE" = 2 ] && export CALYPSO_BSP_TRX_OFFSET="${CALYPSO_BSP_TRX_OFFSET:--1}"
fi
FIRMWARE_ELF="${FIRMWARE_ELF:-$_FW/layer1.highram.elf}"
FIRMWARE_BIN="${FIRMWARE_BIN:-${FIRMWARE_ELF%.elf}.bin}"
# FIRMWARE_FLASH=<4 MiB image>: a REAL firmware in flash (FreeCalypso Magnetite, c139 target:
# Compal bootloader in sector 0 + fwimage.bin at 0x10000, see osmo-operator/scripts/magnetite-c139.sh
# and doc/magnetite.md) instead of the osmocom-bb layer1. QEMU starts at 0 in the flash (no -kernel,
# CPU ti925t = ARMv4T like the ARM7TDMI), the screen is the C139's (CALYPSO_LCD=c139, uWire), the
# L23 stack and the UI are INSIDE the firmware: no osmocon (rvinterf decodes the RVTMUX traces on
# the modem pty, socket $RUNDIR/rvinterf.sock for fc-shell / fc-fsio), no mobile. The DSP (step 1)
# and the air or the bridge (step 5) are unchanged. start-native.sh --ms-magnetite sets it.
FIRMWARE_FLASH="${FIRMWARE_FLASH:-}"
# the TRX of the BTS (OSMO_TRX=1) or phone 2 of the TRX are NEVER Magnetite
if [ -n "$FIRMWARE_FLASH" ] && { [ "$OSMO_TRX" = 1 ] || [ "$OSMO_TRX_PHONE" = 2 ]; }; then
    printf '\033[1m[run dsp]\033[0m OSMO_TRX=1 : FIRMWARE_FLASH=%s ignore, cette instance est le TRX de la BTS (firmware trx)\n' "$FIRMWARE_FLASH"
    FIRMWARE_FLASH=""
fi
# Magnetite (real TI firmware in flash): no pacer catch-up. With catch-up (20), the ARM chains the
# late frames, the L1 saturates and the watchdog resets the firmware (freeze).
if [ -n "$FIRMWARE_FLASH" ]; then
    export CALYPSO_PACER_RATTRAPAGE="${CALYPSO_PACER_RATTRAPAGE:-0}"
    export CALYPSO_PONT_ARM_LOCK="${CALYPSO_PONT_ARM_LOCK:-1}"
    # TI L1: SB window opened at the end of the previous frame (otherwise BCCH shifted by one frame,
    # CRC KO on every block) and NB TOA targeted at 3 (margin 1); see calypso_bsp.c sb_fin_trame()
    export CALYPSO_BSP_SB_FIN_TRAME="${CALYPSO_BSP_SB_FIN_TRAME:-1}"
    export CALYPSO_BSP_NB_MARGE="${CALYPSO_BSP_NB_MARGE:-1}"
    # NB TOA canned at the TI target (pont.c). Off by default: it requires a c54x_exe built with the
    # init_done guard, without which the canning overwrites the DSP code during the boot.
    export PONT_CAN_NB_TOA="${PONT_CAN_NB_TOA:-}"
    # fake air: noise outside the cell's ARFCN (calypso_bsp.c faux_air_*), ARFCN of the radio plan
    # SB modulated like FB/NB: the SB settings made for the osmocom layer1 (instant 0.35, spread 1.0,
    # noise 3000), which ignores the SB angle, bias the measured angle (+166 at 0 Hz); the TI L1 uses
    # it for the AFC (-700 -> -673, ~224 Hz of real offset) and would lose the SB then the BCCH
    export CALYPSO_BSP_SB_DEC="${CALYPSO_BSP_SB_DEC:-0.5}" CALYPSO_BSP_SB_SYM="${CALYPSO_BSP_SB_SYM:-0.3}" CALYPSO_BSP_SB_NOISE="${CALYPSO_BSP_SB_NOISE:-0}"
    export CALYPSO_BSP_BRUIT_FOND="${CALYPSO_BSP_BRUIT_FOND:-1}"   # noise instead of zeros (PM = 0 -> L1 recovery)
    # catch up with the BTS live feed (calypso_bsp.c): OVER THE AIR ONLY (AIR=1, the BTS has its own clock).
    # With a classic BTS pont_dsp.py paces osmo-bts on the frames delivered by the DSP: each +33 jump would
    # speed the BTS up (bridge at ~350 frames/s), hence more lag and more jumps, and IMM ASS never matched.
    export CALYPSO_BSP_RATTRAPE="${CALYPSO_BSP_RATTRAPE:-$AIR}"
    export CALYPSO_RACH_VERS_AIR="${CALYPSO_RACH_VERS_AIR:-1}"   # RACH of the TI firmware published to the air (pont.c)
    # classic BTS: the SDCCH/SACCH bursts of the TI firmware are in calypso_tx_rom (TSC 7, fn of the ROM),
    # but pont_dsp would drop them for lack of an announced dedicated channel (calypso_dcch_cfg comes from
    # the QEMU L1CTL tap, absent without the osmocom layer1). PONT_UL_TN_DEFAUT=0: TS0 (SDCCH/4 of the
    # bench) when nothing is announced. PONT_UL_RAW_DCCH_HOTE=1 (host coding) remains available.
    export PONT_UL_TN_DEFAUT="${PONT_UL_TN_DEFAUT:-0}"
    # uplink TCH by the HOST (FACCH, SACCH/T, speech coded by the bridge from a_fu/a_du): the ROM's TSP
    # script only yields 2 "unknown" bursts on the TI firmware's TCH, so the FACCH SABM would never reach
    # the BTS. 14-demarrage.sh exports PONT_UL_RAW_TCH=1 for the ROM vocoder of the osmocom mobile: forced
    # to 0 here (PONT_UL_RAW_TCH_MAGNETITE to override).
    export PONT_UL_RAW_TCH="${PONT_UL_RAW_TCH_MAGNETITE:-0}"
    # NO fn or RA correction (AIR_B_FN_DELTA, MONTANT_RACH_DIFFERE: 0). The TI L1's RA_R traces the REQUEST
    # (MPHC_RA_REQ); the RACH leaves after the random draw, just before RA_C, and c54x_exe publishes the
    # same ra at the RA_C fn: fn and RA are right without correction.
    export AIR_B_FN_DELTA="${AIR_B_FN_DELTA:-0}"
    export MONTANT_RACH_DIFFERE="${MONTANT_RACH_DIFFERE:-0}"
    export CALYPSO_FAUX_AIR="${CALYPSO_FAUX_AIR:-$( . /etc/osmocom/radio-plan.env 2>/dev/null; echo "${PLAN_ARFCN:-}")}"
    [ -r "$FIRMWARE_FLASH" ] || { printf '\033[1;31m[run] ECHEC :\033[0m image flash absente : %s\n' "$FIRMWARE_FLASH" >&2; exit 1; }
    QEMU_CPU="${QEMU_CPU:-ti925t}"
    export CALYPSO_LCD="${CALYPSO_LCD:-c139}"
    RVINTERF_BIN="${RVINTERF_BIN:-/opt/GSM/fc-host-tools/rvinterf/lowlevel/rvinterf}"
fi
OSMOCON="${OSMOCON:-/opt/GSM/osmocom-bb/src/host/osmocon/osmocon}"
MOBILE="${MOBILE:-$(command -v mobile || echo /usr/local/bin/mobile)}"
MOBILE_CFG="${MOBILE_CFG:-$HERE/mobile_pont.cfg}"
# Two entry points for the same pont/ package: pont_dsp.py for the dsp setup
# (--dsp-port 6702), pont.py for the grgsm setup, which keeps its own
# defaults. PONT_PY forces one or the other.
if [ "$MODE" = grgsm ]; then
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont.py}"
else
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont_dsp.py}"
fi
RUNDIR="${RUNDIR:-/tmp/c54x-pont}"
# REAL TIME. Under load (test campaign, panel, ffmpeg) the chain c54x_exe <-> QEMU <-> osmocon/mobile
# <-> bridge misses ticks and the sound cuts out: the frame is 4.615 ms, the work takes ~1.2 ms
# ([chrono]: A 0.33 + go 0.40 + B 0.16 + qemu <0.32), and the CFS scheduler readily sets a process
# aside for 5 to 10 ms when the cores are busy. The five processes of the chain are therefore
# launched with a priority: RT_NICE=-10 by default (RT_NICE= empty to do nothing), RT_FIFO=<prio> for
# SCHED_FIFO (chrt -f, root; stay below 50, the kernel threads are above), RT_CPUS=<list> to pin them
# (taskset -c, e.g. 1-3: core 0 stays for the rest of the system). RT_RENICE_AUTRES=<n> additionally
# lowers the panel tools (osmo-fft-snap, osmo-panel, osmo-topzone, osmo-ts-probe, conky) to nice n.
# ./run.sh --charge shows who consumes what.
RT_NICE="${RT_NICE--10}"
RT_FIFO="${RT_FIFO:-}"
RT_CPUS="${RT_CPUS:-}"
RT_RENICE_AUTRES="${RT_RENICE_AUTRES:-}"
PRIO=()
[ -n "$RT_CPUS" ] && PRIO+=(taskset -c "$RT_CPUS")
if [ -n "$RT_FIFO" ]; then PRIO+=(chrt -f "$RT_FIFO"); elif [ -n "$RT_NICE" ]; then PRIO+=(nice -n "$RT_NICE"); fi
renice_autres() {
    [ -n "$RT_RENICE_AUTRES" ] || return 0
    local pids; pids="$(pgrep -f 'osmo-fft-snap|osmo-panel|osmo-topzone|osmo-ts-probe|conky' | tr '\n' ' ')"
    [ -n "$pids" ] && renice -n "$RT_RENICE_AUTRES" -p $pids >/dev/null 2>&1 && dire "   outils du panneau a nice $RT_RENICE_AUTRES : $pids"
    return 0
}
charge() {
    echo "chaine (pid %cpu nice politique) :"
    local n
    for n in dsp qemu osmocon rvinterf mobile transceiver pont air; do
        if vivant "$n"; then printf '  %-8s %s\n' "$n" "$(ps -o pid=,pcpu=,ni=,cls= -p "$(pid_de "$n")" | tr -s ' ')"; else printf '  %-8s arrete\n' "$n"; fi
    done
    echo "processus les plus gourmands :"
    ps -eo pid,pcpu,ni,cls,comm --sort=-pcpu | head -12 | sed 's/^/  /'
    echo "charge moyenne $(cut -d' ' -f1-3 /proc/loadavg) sur $(nproc) coeurs ; priorite de la chaine : ${PRIO[*]:-aucune}"
}
L2_SOCK="${L2_SOCK:-/tmp/osmocom_l2}"
MONITOR="${MONITOR:-/tmp/qemu-monitor-pont.sock}"
GDB="${GDB:-1}"                              # QEMU gdbstub + telnet console (step 2)
GDB_STUB="${GDB_STUB:-1234}"
GDB_TELNET="${GDB_TELNET:-44444}"
GDB_TELNET_PY="${GDB_TELNET_PY:-$QOSMO/tools/gdb-telnet.py}"
# 120000 by default: the budget of the TRX's DSP (instance 1, booted at the first IDLE); a smaller
# budget (16000) leaves the DSP of the Magnetite instance in "occupe boot" forever, and a TCH
# frame needs up to 87000 insn/frame (see start-native.sh).
INSNS="${INSNS:-120000}"
# DSP/QEMU lockstep by default (LOCKSTEP=0 for wall-clock mode): the emulated C54x costs
# ~6.7 ms per frame against 4.615 ms of real time, so on the wall clock QEMU skips 3 frames
# out of 4 ("DSP en retard, tick saute"), the ROM only sees one frame in 4 to 6, its FB
# block counter does not advance and the TOA reads 1251 whatever the distance of the FCCH.
# In lockstep QEMU only advances the frame once the DSP has finished the previous one:
# TOA = 23 + n x 1250, delay=10, SB decodable.
LOCKSTEP="${LOCKSTEP:-1}"
[ "$MODE" = dsp ] && [ "$LOCKSTEP" = 1 ] && export CALYPSO_PONT_LOCKSTEP=1
VERB="${VERB:--v}"
IQ="${IQ:-none}"
AMP="${AMP:-30000}"
DSP_SOCK="${DSP_SOCK:-/tmp/calypso_dsp.sock}"
DSP_SHM=/dev/shm/calypso_api_ram

# THE SHOWCASE: THE SAME LOGS AS IN SHUNT MODE. The panel (tmux calypso,
# osmo-fft-snap) follows /run/user/0/osmo-nitb/logs/{qemu,osmocon,mobile}.log
# and /dev/shm/pont.log, those written by the grgsm setup. The REAL file is
# therefore placed at the panel's path and $RUNDIR/<name>.log becomes a link to
# it -- not the other way round: an already open `tail -F` refuses a file
# replaced by a link ("untailable symbolic link"). PANNEAU_LOGS=none to touch
# nothing.
PANNEAU_LOGS="${PANNEAU_LOGS:-/run/user/0/osmo-nitb/logs}"
# The panel's FFT reads /tmp/iq_fft_ms.fifo, which pont.py only fills when
# recording is active: --no-record would silence it. PONT_AIRREC=0 turns it off.
PONT_AIRREC="${PONT_AIRREC:-1}"
vitrine() {   # vitrine <name>: to be called BEFORE the launch that writes $RUNDIR/<name>.log
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

etape1() {   # the DSP (dsp setup only)
    renice_autres
    [ "$MODE" = dsp ] || { dire "1. (montage grgsm : pas de c54x_exe, la couche 1 est dans QEMU)"; return; }
    vivant dsp && { dire "1. c54x_exe deja lance (pid $(pid_de dsp))"; return; }
    [ -x "$HERE/c54x_exe" ] || make -C "$HERE" >/dev/null || rater "make c54x_exe"
    rm -f "$DSP_SHM" "$DSP_SOCK"
    # The TCH announcement of a dead session (pont/dsp/tch.py) must not be
    # read again by montant.c at startup.
    rm -f /dev/shm/calypso_tch_cfg
    # Wait (at most 40 ms) for a frame the BTS delivers late rather than
    # playing it as an erasure (calypso_bsp.c, bsp_attendre_trame).
    ( cd "$HERE" && bruit_env_dsp && CALYPSO_IQDUMP_FCCH=1 CALYPSO_BSP_ATTENTE_MS="${CALYPSO_BSP_ATTENTE_MS:-40}" exec ${PRIO[@]+"${PRIO[@]}"} "${C54X_BIN:-./c54x_exe}" --arm "$DSP_SOCK" --insns "$INSNS" --iq "$IQ" --amp "$AMP" $VERB ) > "$RUNDIR/dsp.log" 2>&1 &
    echo $! > "$RUNDIR/dsp.pid"
    attendre 5 test -S "$DSP_SOCK" || rater "c54x_exe n'a pas ouvert $DSP_SOCK (voir $RUNDIR/dsp.log)"
    # shm at 0666: the umask lowers c54x_exe's 0666 request to 0644, which prevents a harvard.sh
    # run under the user account from writing the API RAM (d@!, sb/si3 demos). Forced here.
    attendre 3 test -e "$DSP_SHM" && chmod 0666 "$DSP_SHM" 2>/dev/null || true
    dire "1. c54x_exe --arm  pid $(pid_de dsp)  ($INSNS insn/trame, iq=$IQ, $DSP_SHM, $DSP_SOCK)"; bruit_dire_dsp
}

etape2() {   # the ARM
    vivant qemu && { dire "2. QEMU deja lance (pid $(pid_de qemu))"; return; }
    [ -x "$QEMU" ] || rater "QEMU absent : $QEMU (ninja -C $QOSMO/build)"
    [ -r "$FIRMWARE_ELF" ] || rater "firmware absent : $FIRMWARE_ELF"
    local extern=""
    if [ "$MODE" = dsp ]; then [ -S "$DSP_SOCK" ] || rater "le DSP ne tourne pas (etape 1 d'abord)"; extern="$DSP_SOCK"; fi
    rm -f "$MONITOR"
    vitrine qemu
    # gdb on the ARM: QEMU gdbstub on tcp::$GDB_STUB, and the qosmo telnet
    # console (tools/gdb-telnet.py) on port $GDB_TELNET:
    #     telnet 0 44444      (Ctrl-C stops the ARM, "continue &" resumes it)
    # As long as nobody is connected, gdb does not run and the ARM is not
    # touched. GDB=0: neither stub nor console.
    local gdb_opt=()
    [ "$GDB" = 1 ] && gdb_opt=(-gdb "tcp:127.0.0.1:$GDB_STUB")
    # ASSEMBLY_LOGS=1 (start-native.sh --assembly-logs): asm trace of the ARM in
    # qemu-asm.log, each block once, at its translation. in_asm only by default.
    # ASSEMBLY_LOGS_FLAGS=in_asm,exec,nochain also logs every executed block
    # (nochain, otherwise chained blocks are only seen once): the ARM then takes
    # ~62 ms per frame instead of 0.4 ([chrono] go); the DSP in lockstep follows,
    # but the bridge's slaved clock (osmo-operator/pont/trx.py, ceiling
    # 4 x FRAME_DUR) runs thousands of frames ahead, the BSP no longer finds the
    # bursts and mobile sees no cell (FBSB result=255).
    # THE TRACE GOES TO THE DISK, NOT TO THE TMPFS. $RUNDIR is in /tmp (4.7 GB
    # tmpfs): with exec,nochain the trace fills /tmp within minutes, and an rm
    # frees nothing as long as QEMU keeps the file open (/proc/<pid>/fd/N must be
    # truncated). Written to ASSEMBLY_LOGS_DIR (/var/tmp/c54x-pont, on the disk)
    # and $RUNDIR/qemu-asm.log becomes a link; the archives leave it alone
    # (garder_journaux).
    if [ "${ASSEMBLY_LOGS:-0}" = 1 ]; then
        local asm_dir="${ASSEMBLY_LOGS_DIR:-/var/tmp/c54x-pont}"
        mkdir -p "$asm_dir" 2>/dev/null || asm_dir="$RUNDIR"
        rm -f "$RUNDIR/qemu-asm.log" "$asm_dir/qemu-asm.log"
        [ "$asm_dir" != "$RUNDIR" ] && ln -s "$asm_dir/qemu-asm.log" "$RUNDIR/qemu-asm.log"
        local asm_cible="$asm_dir/qemu-asm.log"
        # ASSEMBLY_LOGS_FIFO=1: QEMU writes into a FIFO on /dev/shm and a cat drains
        # it to the file on the disk. qemu_log_unlock() does an fflush() on EVERY
        # line (util/log.c): with exec,nochain that is one write(2) per executed
        # block (~830 per frame), about ten us on the disk against 1 to 2 us on a
        # pipe. The reader is launched BEFORE QEMU: its fopen(fifo, "w") blocks as
        # long as nobody reads.
        if [ "${ASSEMBLY_LOGS_FIFO:-0}" = 1 ]; then
            local asm_fifo="/dev/shm/qemu-asm.fifo"
            rm -f "$asm_fifo"; mkfifo "$asm_fifo"
            ${PRIO[@]+"${PRIO[@]}"} cat "$asm_fifo" > "$asm_cible" 2>/dev/null &
            echo $! > "$RUNDIR/asmcat.pid"
            asm_cible="$asm_fifo"
        fi
        gdb_opt+=(-d "${ASSEMBLY_LOGS_FLAGS:-in_asm}" -D "$asm_cible")
        [ -n "${ASSEMBLY_LOGS_FILTRE:-}" ] && gdb_opt+=(-dfilter "$ASSEMBLY_LOGS_FILTRE")
        # QEMU_LOG_BUFFERED (qosmo's util/log.c): fully buffered -D file, no more
        # fflush() per line. 4 MiB by default when tracing.
        export QEMU_LOG_BUFFERED="${QEMU_LOG_BUFFERED:-4194304}"
        # RING OF EXECUTED BLOCKS, BY DEFAULT WITH ASSEMBLY_LOGS=1. The TCG plugin tbring
        # (qosmo/tools/qosmo-tbring) pushes the PC of every EXECUTED block, chained blocks
        # included, into an mmap ring: 8 bytes per block, no text in the loop, no nochain,
        # the ARM runs as in production (-d exec,nochain: 150 MB/s of text, 20 GB in 130 s).
        # 64 GiB ring = 2^33 blocks, i.e. ~12 h of ARM at 180 000 blocks/s; SPARSE file on
        # the disk ($asm_dir, not the tmpfs), it only weighs what has been written. Read
        # back live, at any time, without stopping QEMU:
        #     tbring-dump -n 500 -f $asm_dir/qemu-tbring -e $FIRMWARE_ELF -a $asm_dir/qemu-asm.log
        # (the in_asm trace, one block per translation, serves as disassembly table).
        # ASSEMBLY_LOGS_RING=0 turns it off; ASSEMBLY_LOGS_RING_ENTRIES (power of 2) and
        # ASSEMBLY_LOGS_RING_FILE change the size and the location.
        if [ "${ASSEMBLY_LOGS_RING:-1}" = 1 ]; then
            local tbring="${TBRING_SO:-$QOSMO/tools/qosmo-tbring/libtbring.so}"
            [ -f "$tbring" ] || tbring=/usr/local/lib/qosmo/libtbring.so
            if [ -f "$tbring" ]; then
                local ring="${ASSEMBLY_LOGS_RING_FILE:-$asm_dir/qemu-tbring}"
                rm -f "$ring" "$RUNDIR/qemu-tbring"
                [ "$(dirname "$ring")" != "$RUNDIR" ] && ln -s "$ring" "$RUNDIR/qemu-tbring"
                # anneau= and not file=: file= is QEMU's keyword for the plugin path
                gdb_opt+=(-plugin "$tbring,anneau=$ring,entries=${ASSEMBLY_LOGS_RING_ENTRIES:-8589934592}")
            else
                dire "   anneau tbring ignore : libtbring.so introuvable (make -C $QOSMO/tools/qosmo-tbring)"
            fi
        fi
    fi
    # QEMU_DEBUG_FLAGS=int,guest_errors: log of the ARM's exceptions (PC of the Data
    # Abort, access to an unmapped address) in $RUNDIR/qemu-debug.log. Without
    # ASSEMBLY_LOGS only (a single -d/-D per QEMU command line).
    if [ -n "${QEMU_DEBUG_FLAGS:-}" ] && [ "${ASSEMBLY_LOGS:-0}" != 1 ]; then
        gdb_opt+=(-d "$QEMU_DEBUG_FLAGS" -D "$RUNDIR/qemu-debug.log")
    fi
    # QEMU_ICOUNT=auto|shift=N,... (start-native.sh --icount): -icount.
    case "${QEMU_ICOUNT:-}" in ''|0|off) ;; *) gdb_opt+=(-icount "$QEMU_ICOUNT") ;; esac
    # QEMU polls the DSP every frame/64 (0.07 ms) instead of frame/16
    # (0.29 ms): the serial part of a frame (DONE of phase A, GO) pays this
    # latency twice -- [chrono] measurement on TCH.
    CALYPSO_PONT_RETRY_DIV="${CALYPSO_PONT_RETRY_DIV:-64}" \
    # FIRMWARE_FLASH: the flash (working copy in $RUNDIR, kept between two launches for the
    # FFS; FLASH_RESET=1 or a newer image replace it) replaces -kernel
    local fw_opt=(-kernel "$FIRMWARE_ELF")
    if [ -n "$FIRMWARE_FLASH" ]; then
        if [ "${FLASH_RESET:-0}" != 0 ] || [ ! -f "$RUNDIR/flash.img" ] || [ "$FIRMWARE_FLASH" -nt "$RUNDIR/flash.img" ]; then
            cp "$FIRMWARE_FLASH" "$RUNDIR/flash.img" || rater "copie de $FIRMWARE_FLASH impossible"
            dire "   flash de travail $RUNDIR/flash.img (re)creee depuis $FIRMWARE_FLASH"
        fi
        fw_opt=(-drive "if=pflash,file=$RUNDIR/flash.img,format=raw")
    fi
    CALYPSO_DSP_EXTERN="$extern" ${PRIO[@]+"${PRIO[@]}"} "$QEMU" -M calypso -cpu "${QEMU_CPU:-arm946}" -display none -parallel none \
        -serial pty -serial pty -monitor "unix:$MONITOR,server,nowait" "${gdb_opt[@]}" \
        "${fw_opt[@]}" > "$RUNDIR/qemu.log" 2>&1 &
    echo $! > "$RUNDIR/qemu.pid"
    if [ "$GDB" = 1 ] && [ -f "$GDB_TELNET_PY" ] && ! vivant gdb; then
        python3 "$GDB_TELNET_PY" --port "$GDB_TELNET" --stub "$GDB_STUB" --elf "$FIRMWARE_ELF" \
            > "$RUNDIR/gdb.log" 2>&1 &
        echo $! > "$RUNDIR/gdb.pid"
    fi
    attendre 10 grep -aq "label serial0" "$RUNDIR/qemu.log" || rater "QEMU n'a pas publie son pty (voir $RUNDIR/qemu.log)"
    if [ "$MODE" = dsp ]; then
        attendre 10 grep -aq "pont DSP : API RAM partagee" "$RUNDIR/qemu.log" || rater "QEMU n'a pas rejoint le DSP (voir $RUNDIR/qemu.log)"
    else
        attendre 5 grep -aq "backend gr-gsm" "$RUNDIR/qemu.log" || rater "la couche 1 gr-gsm ne s'est pas annoncee (voir $RUNDIR/qemu.log)"
    fi
    sed -n 's/.*redirected to \(\/dev\/pts\/[0-9]*\) (label serial0).*/\1/p' "$RUNDIR/qemu.log" | head -1 > "$RUNDIR/modem.pty"
    dire "2. qemu-system-arm  pid $(pid_de qemu)  pty modem $(cat "$RUNDIR/modem.pty")  moniteur $MONITOR  L1=$([ "$MODE" = dsp ] && echo "DSP externe" || echo "gr-gsm (udp 4730/4731)")"
    [ "$GDB" = 1 ] && dire "   gdb ARM : telnet 0 $GDB_TELNET  (stub tcp::$GDB_STUB, journal $RUNDIR/gdb.log)"
    case "${QEMU_ICOUNT:-}" in ''|0|off) ;; *) dire "   icount : $QEMU_ICOUNT" ;; esac
    [ -n "${QEMU_DEBUG_FLAGS:-}" ] && [ "${ASSEMBLY_LOGS:-0}" != 1 ] && dire "   journal QEMU -d $QEMU_DEBUG_FLAGS : $RUNDIR/qemu-debug.log"
    # mailbox monitor (CALYPSO_MAILBOX=1): two files, one per side
    if [ -n "${CALYPSO_MAILBOX:-}" ] && [ "${CALYPSO_MAILBOX}" != 0 ]; then
        local _mb="${CALYPSO_MAILBOX_FILE:-${LOG_DIR:-/tmp/calypso/logs}/mailbox.log}"
        dire "   mailbox ARM<->DSP : ${_mb} (cote DSP, c54x_exe) et ${_mb%.log}-arm.log (cote ARM, QEMU) ; CALYPSO_MAILBOX=$CALYPSO_MAILBOX"
    fi
    [ "${ASSEMBLY_LOGS:-0}" = 1 ] && dire "   trace asm ARM : $(readlink -f "$RUNDIR/qemu-asm.log") (-d ${ASSEMBLY_LOGS_FLAGS:-in_asm}${ASSEMBLY_LOGS_FILTRE:+, -dfilter $ASSEMBLY_LOGS_FILTRE}${ASSEMBLY_LOGS_FIFO:+, via FIFO}, QEMU_LOG_BUFFERED=$QEMU_LOG_BUFFERED)$([ "${ASSEMBLY_LOGS_RING:-1}" = 1 ] && echo " ; anneau des blocs executes : $(readlink -f "$RUNDIR/qemu-tbring" 2>/dev/null) (tbring-dump)")"
}

etape3_rvinterf() {   # FIRMWARE_FLASH: rvinterf (FreeCalypso) instead of osmocon
    vivant rvinterf && { dire "3. rvinterf deja lance (pid $(pid_de rvinterf))"; return; }
    [ -x "$RVINTERF_BIN" ] || { dire "3. (rvinterf absent : $RVINTERF_BIN ; pas de traces RVTMUX)"; return; }
    local pty; pty="$(cat "$RUNDIR/modem.pty")"
    rm -f "$RUNDIR/rvinterf.sock"
    "$RVINTERF_BIN" -n -l "$RUNDIR/rvinterf.log" -s "$RUNDIR/rvinterf.sock" "$pty" > "$RUNDIR/rvinterf.out" 2>&1 &
    echo $! > "$RUNDIR/rvinterf.pid"
    dire "3. rvinterf  pid $(pid_de rvinterf)  traces RVTMUX -> $RUNDIR/rvinterf.log  socket $RUNDIR/rvinterf.sock (fc-shell -s / fc-fsio -s)"
}
etape3() {   # osmocon
    [ -n "$FIRMWARE_FLASH" ] && { etape3_rvinterf; return; }
    vivant osmocon && { dire "3. osmocon deja lance (pid $(pid_de osmocon))"; return; }
    vivant qemu || rater "QEMU ne tourne pas (etape 2 d'abord)"
    [ -x "$OSMOCON" ] || rater "osmocon absent : $OSMOCON"
    [ -r "$FIRMWARE_BIN" ] || rater "image .bin absente : $FIRMWARE_BIN"
    local pty; pty="$(cat "$RUNDIR/modem.pty")"
    rm -f "$L2_SOCK"
    vitrine osmocon
    ${PRIO[@]+"${PRIO[@]}"} stdbuf -oL -eL "$OSMOCON" -m romload -i 100 -p "$pty" -s "$L2_SOCK" "$FIRMWARE_BIN" > "$RUNDIR/osmocon.log" 2>&1 &
    echo $! > "$RUNDIR/osmocon.pid"
    if ! attendre 30 grep -aq "your code is running now" "$RUNDIR/osmocon.log"; then
        rater "osmocon n'a pas fini le romload (voir $RUNDIR/osmocon.log). Un osmocon tue a mi-bloc laisse
       le stub romload de l'UART en attente : ./run.sh --stop puis ./run.sh"
    fi
    dire "3. osmocon  pid $(pid_de osmocon)  L1CTL sur $L2_SOCK"
}

# BANC_VOCODEUR=rom: the ROM vocodes (modelled voice port, CALYPSO_VSP=1).
# The mobile must then be in `io-handler l1phy` -- the only mode where the firmware
# lifts the vocoder mutes (prim_tch.c:101-111) -- and WITHOUT io-tch-format or ALSA in
# the tch-voice block: under l1phy the mobile refuses io-tch-format and drops the WHOLE
# file. The variant is therefore generated here from MOBILE_CFG, never written by hand.
config_vocodeur_rom() {
    local src="$1" dst="$RUNDIR/mobile_pont_rom.cfg"
    awk '
        /^ tch-voice/ { voix = 1; print; next }
        /^ [^ ]/      { voix = 0 }
        voix && /^  io-handler gapk/ { print "  io-handler l1phy"; next }
        voix && /^  (io-tch-format|alsa-output-dev|alsa-input-dev)/ { next }
        { print }' "$src" > "$dst" || rater "variante l1phy impossible : $dst"
    grep -q "^  io-handler l1phy" "$dst" || rater "variante l1phy : pas de bloc tch-voice gapk dans $src"
    echo "$dst"
}

etape4_trx() {   # OSMO_TRX=1: the host app transceiver, instead of the mobile
    [ "$OSMO_TRX_PHONE" = 2 ] && { dire "4. (OSMO_TRX_PHONE=2 : telephone 2 du transceiver de l'instance 1, TS1 ; pas de transceiver ici)"; return; }
    vivant transceiver && { dire "4. transceiver deja lance (pid $(pid_de transceiver))"; return; }
    [ -S "$L2_SOCK" ] || rater "pas de socket L1CTL $L2_SOCK (etape 3 d'abord)"
    local args_ts1=""
    if [ "$OSMO_TRX_TS1" = 1 ]; then
        local l2_3="/tmp/osmocom_l2_$OSMO_TRX_TS1_INSTANCE"
        if [ ! -S "$l2_3" ]; then
            dire "4a. telephone 2 (TS1) : instance $OSMO_TRX_TS1_INSTANCE (c54x_exe + QEMU trx + osmocon)"
            INSTANCE="$OSMO_TRX_TS1_INSTANCE" OSMO_TRX=1 OSMO_TRX_PHONE=2 AIR=0 PONT=0 bash "$0" || rater "l'instance $OSMO_TRX_TS1_INSTANCE (telephone 2) n'a pas demarre"
        fi
        attendre 10 test -S "$l2_3" || rater "pas de socket L1CTL $l2_3 pour le telephone 2"
        args_ts1="-2 -L $l2_3"
    fi
    [ -x "$TRANSCEIVER" ] || rater "transceiver absent : $TRANSCEIVER (make -C /opt/GSM/osmocom-bb-transceiver/src/host/layer23/src/transceiver)"
    local p
    # LOCAL column of ss ($4; $5 is the peer), all states: with "-uan | grep" the peer column would match
    # too (osmo-bts-trx 5800 -> 127.0.0.1:5700 would pass for a holder), and "-uln" does not see the
    # transceiver's sockets, which are CONNECTED (ESTAB) to the BTS, not listening.
    for p in 5700 5701 5702; do
        ss -uanH 2>/dev/null | awk -v p="$p" '$4 ~ ("[:.]" p "$") {found=1} END {exit !found}' \
            && rater "port UDP $p deja pris par $(ss -uanpH 2>/dev/null | awk -v p="$p" '$4 ~ ("[:.]" p "$")' | grep -o 'users:.*' | head -1) (un pont.py qui traine ? ./run.sh --stop)"
    done
    vitrine transceiver
    # -l: osmocon's L1CTL socket; -m: RX timeslot mask (a single one to begin with); the TX mask
    # keeps its default 0xe3 (OSMO_TRX_TX_MASK to force it: -M), see the header
    ${PRIO[@]+"${PRIO[@]}"} stdbuf -oL -eL "$TRANSCEIVER" -l "$L2_SOCK" -m "$OSMO_TRX_TS_MASK" ${OSMO_TRX_TX_MASK:+-M "$OSMO_TRX_TX_MASK"} ${OSMO_TRX_BSIC:+-b "$OSMO_TRX_BSIC"} $args_ts1 $OSMO_TRX_ARGS > "$RUNDIR/transceiver.log" 2>&1 &
    echo $! > "$RUNDIR/transceiver.pid"
    attendre 10 sh -c "ss -uanH 2>/dev/null | awk '\$4 ~ /[:.]5702\$/ {f=1} END {exit !f}'" || rater "transceiver n'ecoute pas en 5700-5702 (voir $RUNDIR/transceiver.log)"
    vivant transceiver || rater "transceiver s'est arrete : $(tail -1 "$RUNDIR/transceiver.log")"
    # osmo-bts-trx (BTS#0) EXITS when the transceiver disappears ("No more clock from
    # transceiver", bts_shutdown exit 1): after a --stop the BTS is gone, and the fork's plan
    # (qosmo/run.sh --only bts) refuses to restart it alone (bsc dependency). It is restarted here
    # like the 60-bts.sh module, in the panel's log. OSMO_TRX_BTS_RELANCE=0 to do nothing.
    if [ "${OSMO_TRX_BTS_RELANCE:-1}" = 1 ]; then
        local bts_cfg="${BTS_CFG:-/etc/osmocom/osmo-bts-trx.cfg}" bts_bin bts_log bts_pid _i
        # The previous BTS may still be shutting down (bts_shutdown: up to 70 s measured):
        # wait until it is gone, or until it has switched the firmware on (POWERON -> "BTS MODE: 7 514").
        for _i in $(seq 1 "${OSMO_TRX_BTS_ATTENTE:-90}"); do
            pgrep -f -- "osmo-bts-trx -c $bts_cfg\$" >/dev/null 2>&1 || break
            grep -aq 'BTS MODE: [0-9]* [1-9]' "$RUNDIR/osmocon.log" 2>/dev/null && break
            [ "$_i" = 1 ] && dire "   BTS#0 presente mais muette : on attend qu'elle allume le firmware ou qu'elle finisse de s'eteindre (${OSMO_TRX_BTS_ATTENTE:-90} s max)"
            sleep 1
        done
        if ! pgrep -f -- "osmo-bts-trx -c $bts_cfg\$" >/dev/null 2>&1; then
            bts_bin="${OSMO_BTS_TRX_BIN:-$(command -v osmo-bts-trx || echo /usr/local/bin/osmo-bts-trx)}"
            bts_log="$(ls -d /run/user/*/osmo-nitb/logs 2>/dev/null | head -1)"; bts_log="${bts_log:-$RUNDIR}/bts.log"
            if [ -x "$bts_bin" ] && [ -r "$bts_cfg" ]; then
                setsid stdbuf -oL -eL $(chrt -r 1 true 2>/dev/null && echo "chrt -r 1") "$bts_bin" -c "$bts_cfg" >> "$bts_log" 2>&1 </dev/null &
                bts_pid=$!
                for bts_piddir in /run/user/*/osmo-nitb; do [ -d "$bts_piddir" ] && echo "$bts_pid" > "$bts_piddir/bts.pid"; done
                dire "4b. osmo-bts-trx relancee (BTS#0 absente : elle sort quand le transceiver disparait)  pid $bts_pid  $bts_cfg  journal $bts_log"
            else
                dire "4b. BTS#0 absente et non relancable (binaire $bts_bin ou config $bts_cfg) : ./start-native.sh --osmo-trx"
            fi
        fi
    fi
    dire "4. transceiver  pid $(pid_de transceiver)  L1CTL $L2_SOCK  TRXD 5700-5702 <- osmo-bts-trx  timeslots RX $OSMO_TRX_TS_MASK TX ${OSMO_TRX_TX_MASK:-0xe3}  bsic ${OSMO_TRX_BSIC:-?}  ($OSMO_TRX_ARGS)${args_ts1:+  + telephone 2 TS1 : $args_ts1}"
}

etape4() {   # the mobile
    [ -n "$FIRMWARE_FLASH" ] && { dire "4. (FIRMWARE_FLASH : pile L23 et UI dans le firmware Magnetite, pas de mobile ; ecran C139 : tools/osmo-c123-gtk.py --modele c139 --instance $INSTANCE)"; return; }
    [ -n "$FIRMWARE_APP" ] && { dire "4. (FIRMWARE_APP=$FIRMWARE_APP : pas de mobile ; ecran /dev/shm/calypso_lcd, touches /tmp/calypso_keys.sock, fenetre : tools/osmo-c123-gtk.py)"; return; }
    [ "$OSMO_TRX" = 1 ] && { etape4_trx; return; }
    vivant mobile && { dire "4. mobile deja lance (pid $(pid_de mobile))"; return; }
    if [ "${BANC_VOCODEUR:-hote}" = rom ]; then
        MOBILE_CFG="$(config_vocodeur_rom "$MOBILE_CFG")"
        dire "4. vocodeur ROM (BANC_VOCODEUR=rom) : tch-voice en l1phy -> $MOBILE_CFG"
    fi
    [ -S "$L2_SOCK" ] || rater "pas de socket L1CTL $L2_SOCK (etape 3 d'abord)"
    if [ "$INSTANCE" != 1 ]; then
        # instance-specific sockets and VTY (layer2/sap/ms_data suffixed _N, VTY +10 per instance)
        local _src="$MOBILE_CFG" _dst="$RUNDIR/mobile_instance$INSTANCE.cfg"
        # the VTY port is shifted by 100 per instance (4257 = osmo-ggsn; awk adds, sed does not; sub() keeps
        # the indentation: "$3 = ..." would rebuild the line without the leading space and the VTY would refuse it)
        awk -v inc=$((100 * (INSTANCE - 1))) '/^[[:space:]]*bind 127\.0\.0\.1 [0-9]+/ { n = $3 + inc; sub(/[0-9]+$/, n) } { print }' "$_src" \
            | sed -e "s#/tmp/osmocom_l2\b#$L2_SOCK#" -e "s#/tmp/osmocom_sap\b#/tmp/osmocom_sap_$INSTANCE#" -e "s#/tmp/ms_data\b#/tmp/ms_data_$INSTANCE#" > "$_dst"
        # "stick" follows the ARFCN of the radio plan (BTS#0) when it has changed (template: 514)
        local _arfcn; _arfcn="$( . /etc/osmocom/radio-plan.env 2>/dev/null; echo "${PLAN_ARFCN:-}")"
        [ -n "$_arfcn" ] && sed -i "s|^\([[:space:]]*\)stick [0-9]*\$|\1stick $_arfcn|" "$_dst"
        MOBILE_CFG="$_dst"
        dire "4. instance $INSTANCE : config mobile derivee -> $MOBILE_CFG"
    fi
    [ -x "$MOBILE" ] || rater "mobile absent : $MOBILE"
    [ -r "$MOBILE_CFG" ] || rater "config mobile absente : $MOBILE_CFG"
    local vty; vty="$(sed -n 's/^ *bind 127.0.0.1 \([0-9]*\).*/\1/p' "$MOBILE_CFG" | head -1)"
    if [ -n "$vty" ] && ss -ltn 2>/dev/null | grep -q ":$vty "; then
        rater "le port VTY $vty de $MOBILE_CFG est deja pris par : $(ss -ltnp 2>/dev/null | grep ":$vty " | grep -o 'users:.*' | head -1)
       changez la ligne « bind 127.0.0.1 $vty » de la config (les 42xx sont ceux du reseau du banc)"
    fi
    vitrine mobile
    # Resync on the chosen cell: 8 attempts instead of 1 (gsm322.c,
    # sync_retries_selection). The DSP only passes the SB once in three to
    # five, and two failures are enough for the "no service" loop.
    # grgsm setup: binary default, unchanged.
    local retries=""
    [ "$MODE" = dsp ] && retries="${L23_SYNC_RETRIES_SELECTION:-8}"
    L23_SYNC_RETRIES_SELECTION="$retries" \
    ${PRIO[@]+"${PRIO[@]}"} stdbuf -oL "$MOBILE" -c "$MOBILE_CFG" > "$RUNDIR/mobile.log" 2>&1 &
    echo $! > "$RUNDIR/mobile.pid"
    sleep 2
    vivant mobile || rater "mobile s'est arrete : $(sed 's/\x1b\[[0-9;]*m//g' "$RUNDIR/mobile.log" | grep -iE 'cannot|error|unable' | tail -1)"
    dire "4. mobile  pid $(pid_de mobile)  config $MOBILE_CFG  vty telnet 127.0.0.1 ${vty:-?}"
}

etape5_air() {   # AIR=1: the air between the two Calypsos (pont/air.py), instead of the bridge
    if vivant air; then dire "5. air deja lance (pid $(pid_de air))"; etape5_vsp; return; fi
    [ -r "$AIR_PY" ] || rater "air.py absent : $AIR_PY"
    local autre_shm="$HOTE_SHM" autre_port=6702
    [ "$INSTANCE" = 1 ] && { autre_shm=/dev/shm/calypso2; autre_port=6712; }
    vitrine air
    # A = the other instance (its TX ring, its BSP port), B = this one
    # phone 2 of the TRX (TS1, instance 3: /dev/shm/calypso3, BSP udp 6722) if it runs, and the TCH
    # announcement to the mobile (calypso_tch_cfg of THIS /dev/shm) when the air decodes the ASSIGNMENT COMMAND
    local args_c=""
    if [ -S "/tmp/osmocom_l2_${OSMO_TRX_TS1_INSTANCE}" ] && [ -f "/tmp/c54x-pont${OSMO_TRX_TS1_INSTANCE}/dsp.pid" ] \
       && kill -0 "$(cat "/tmp/c54x-pont${OSMO_TRX_TS1_INSTANCE}/dsp.pid" 2>/dev/null)" 2>/dev/null; then
        args_c="--c-ring $HOTE_SHM/calypso${OSMO_TRX_TS1_INSTANCE}/calypso_tx_rom --c-port $((6702 + 10 * (OSMO_TRX_TS1_INSTANCE - 1) + (OSMO_TRX_TS1_INSTANCE >= 3 ? 30 : 0)))"
    fi
    ${PRIO[@]+"${PRIO[@]}"} python3 -u "$AIR_PY" --a-ring "$autre_shm/calypso_tx_rom" --a-port "$autre_port" \
        --b-ring /dev/shm/calypso_tx_rom --b-port "${CALYPSO_BSP_PORT:-6702}" $args_c --tch-cfg /dev/shm/calypso_tch_cfg > "$RUNDIR/air.log" 2>&1 &
    echo $! > "$RUNDIR/air.pid"
    attendre 5 grep -aq "air :" "$RUNDIR/air.log" || rater "air.py ne s'est pas annonce (voir $RUNDIR/air.log)"
    dire "5. air  pid $(pid_de air)  $autre_shm/calypso_tx_rom -> udp $CALYPSO_BSP_PORT (ici) ; /dev/shm/calypso_tx_rom (ici) -> udp $autre_port (l'autre)${args_c:+ ; TS1 : $args_c}"
    etape5_vsp
}

# the voice port (CALYPSO_VSP=1): PCM rings of the DSP <-> PulseAudio, also in air mode (the call's
# sound otherwise goes through the bridge; without it the call is established but silent). Launched in
# the instance's namespace: /dev/shm/calypso_pcm_ul and _dl are those of THIS mobile.
etape5_vsp() {
    [ "${CALYPSO_VSP:-0}" = 1 ] || return 0
    vivant vsp && { dire "5b. pont audio VSP deja lance (pid $(pid_de vsp))"; return 0; }
    ( cd /opt/GSM/osmo-operator && exec python3 -m pont.vsp_audio ) > "$RUNDIR/vsp.log" 2>&1 &
    echo $! > "$RUNDIR/vsp.pid"
    dire "5b. pont audio VSP  pid $(pid_de vsp)  gsm_mic.monitor -> calypso_pcm_ul, calypso_pcm_dl -> gsm_audio  (journal $RUNDIR/vsp.log)"
}

etape5() {   # the TRX bridge (PONT=1): BTS bursts to layer 1
    [ "$AIR" = 1 ] && { etape5_air; return; }
    [ "$OSMO_TRX" = 1 ] && { dire "5. (OSMO_TRX=1 : pas de pont, transceiver tient 5700-5702)"; return; }
    [ "$OSMO_TRX_PHONE" = 2 ] && { dire "5. (telephone 2 : rien)"; return; }
    [ -n "$FIRMWARE_APP" ] && { dire "5. (FIRMWARE_APP=$FIRMWARE_APP : pas de pont)"; return; }
    [ "$PONT" = 1 ] || { dire "5. (PONT=0 : pas de pont.py, aucun burst du BTS n'arrive)"; return; }
    vivant pont && { dire "5. pont.py deja lance (pid $(pid_de pont))"; return; }
    [ -r "$PONT_PY" ] || rater "pont.py absent : $PONT_PY"
    local extra=""; [ "$MODE" = dsp ] && extra="--dsp-port 6702"
    [ "$PONT_AIRREC" = 0 ] && extra="$extra --no-record"
    vitrine pont
    # UL_RAW=1 (default): the bridge sends the ROM's final uplink bursts (calypso_tx_rom,
    # src/tsp_tx.c) as they are, at their frame number -- RACH, SDCCH, SACCH, FACCH, speech, A5 included.
    # UL_RAW=0: coding and A5 on the host side (PONT_UL_ROM remains the xCCH path).
    # ARM asm trace: the ARM+DSP pair can run at 14 times real time (exec,nochain).
    # The bridge's slaved clock (pont/trx.py) only goes down to 4 times below
    # real time by default; it is allowed 16 times (measured: 10 to 14 times;
    # beyond 16 the pause requested from osmo-bts-trx between two IND CLOCK
    # approaches its 50-frame limit, see trx.py run_clock). PONT_HORLOGE_MAX
    # already set in the environment is honoured.
    if [ "${ASSEMBLY_LOGS:-0}" = 1 ]; then export PONT_HORLOGE_MAX="${PONT_HORLOGE_MAX:-16}"; fi
    ( cd "$(dirname "$PONT_PY")/.." && bruit_env_pont && export PONT_UL_RAW="${UL_RAW:-1}" && exec ${PRIO[@]+"${PRIO[@]}"} python3 "$PONT_PY" $extra ) > "$RUNDIR/pont.log" 2>&1 &
    echo $! > "$RUNDIR/pont.pid"
    attendre 10 grep -aq "pont TRX : ports" "$RUNDIR/pont.log" || rater "pont.py ne s'est pas annonce (voir $RUNDIR/pont.log)"
    dire "5. pont.py  pid $(pid_de pont)  TRXD 5700-5702 <- BTS ; vers $([ "$MODE" = dsp ] && echo "le DSP (udp 6702)" || echo "la L1 gr-gsm (udp 4730/4731)")"
    [ -n "${PONT_HORLOGE_MAX:-}" ] && dire "   horloge asservie : ralentissement jusqu'a ${PONT_HORLOGE_MAX} x le temps reel (PONT_HORLOGE_MAX)"
    # voice port (CALYPSO_VSP=1): PCM rings of the DSP <-> PulseAudio
    if [ "${CALYPSO_VSP:-0}" = 1 ] && ! vivant vsp; then
        ( cd /opt/GSM/osmo-operator && exec python3 -m pont.vsp_audio ) > "$RUNDIR/vsp.log" 2>&1 &
        echo $! > "$RUNDIR/vsp.pid"
        dire "5b. pont audio VSP  pid $(pid_de vsp)  gsm_mic.monitor -> calypso_pcm_ul, calypso_pcm_dl -> gsm_audio"
    fi
}

arreter() {
    local n
    for n in vsp pont air bruit mobile transceiver osmocon rvinterf gdb qemu dsp asmcat; do
        if vivant "$n"; then kill "$(pid_de "$n")" 2>/dev/null; dire "arret $n (pid $(pid_de "$n"))"; fi
        rm -f "$RUNDIR/$n.pid"
    done; bruit_arreter
    sleep 1
    # FULL STOP OF AN INSTANCE N != 1 (banc-rssi --stop): processes without a pid file or whose pid was
    # lost (relaunch over a live instance) would keep the BSP port (udp 6762 held by a ghost c54x_exe, the
    # new one receiving nothing). Everything holding the instance's own resources is swept.
    if [ "$INSTANCE" != 1 ]; then
        # Not "$RUNDIR/": a mere tail -F of its logs, or the shell that launched --stop, would match.
        local motif="$DSP_SOCK|$MONITOR|$L2_SOCK|--stub $GDB_STUB |--port $GDB_TELNET "
        local p=$$ ancetres=" $$ "
        while p="$(awk '/^PPid:/ {print $2}' "/proc/$p/status" 2>/dev/null)" && [ -n "$p" ] && [ "$p" -gt 1 ]; do
            ancetres="$ancetres$p "
        done
        local restes="" r
        for r in $(pgrep -f -- "$motif"); do
            case "$ancetres" in *" $r "*) ;; *) restes="$restes$r " ;; esac
        done
        if [ -n "${restes// /}" ]; then
            dire "instance $INSTANCE : restes arretes (pid $restes)"
            kill $restes 2>/dev/null; sleep 1; kill -9 $restes 2>/dev/null
        fi
    fi
    rm -f "$DSP_SHM" "$DSP_SOCK" "$L2_SOCK" "$MONITOR" "$RUNDIR/modem.pty" /dev/shm/qemu-asm.fifo
    # Mobile sockets: they do not disappear with it. A leftover "sap" or
    # "ms_data" (tch-data, mobile_pont.cfg) file makes the bind of the next
    # run fail with EADDRINUSE, and the mobile starts without its data path
    # without saying so.
    # ONLY for instance 1: these sockets are in the SHARED /tmp and belong to the mobile of
    # instance 1; banc-rssi --stop (instance 4) must not erase them under the running mobile (SAP, tch-data).
    [ "$INSTANCE" = 1 ] && rm -f /tmp/osmocom_sap /tmp/ms_data
    # Bench clock (calypso_bsp.c -> pont/trx.py): left in place, pont.py
    # would slave its first second to the frame of a dead session and the BTS
    # would stay frozen waiting for a DSP that no longer exists.
    rm -f /dev/shm/calypso_horloge
    # Uplink side-bands (src/montant.c): otherwise pont.py re-reads the last
    # record of a previous session at startup.
    rm -f /dev/shm/calypso_rach /dev/shm/calypso_sdcch_ul \
          /dev/shm/calypso_tch_facch_ul /dev/shm/calypso_tch_sacch_ul \
          /dev/shm/calypso_tch_ul
    # TCH announcement by the DSP bridge (pont/dsp/tch.py -> montant.c
    # scruter_tch). dsp setup only: in grgsm this file belongs to QEMU's
    # gr-gsm L1, it is left alone.
    [ "$MODE" = dsp ] && rm -f /dev/shm/calypso_tch_cfg
}

statut() {
    local n
    for n in dsp qemu osmocon rvinterf mobile transceiver pont air; do
        if vivant "$n"; then printf '  %-8s pid %-7s  %s\n' "$n" "$(pid_de "$n")" "$RUNDIR/$n.log"
        else printf '  %-8s arrete\n' "$n"; fi
    done; bruit_statut
    [ -f "$RUNDIR/dsp.log" ] && grep -a "fn=" "$RUNDIR/dsp.log" | tail -1
    [ -f "$RUNDIR/osmocon.log" ] && grep -a "FB0\|FB1\|SB\|BSIC" "$RUNDIR/osmocon.log" | tail -1
}

# KEEP THE DSP AND BRIDGE LOGS. dsp.log is overwritten at every launch and
# pont.log is emptied by vitrine; neither is in the panel's archives
# (osmo-nitb/archives). The previous session is stored in
# $RUNDIR/archives/<date of dsp.log>/ and JOURNAUX_GARDES (10) of them are
# kept. JOURNAUX_GARDES=0 keeps nothing.
JOURNAUX_GARDES="${JOURNAUX_GARDES:-10}"
garder_journaux() {
    [ "$JOURNAUX_GARDES" -gt 0 ] 2>/dev/null || return 0
    [ -s "$RUNDIR/dsp.log" ] || [ -s /dev/shm/pont.log ] || return 0
    local d="$RUNDIR/archives/$(date -r "$RUNDIR/dsp.log" +%Y%m%d-%H%M%S 2>/dev/null || date +%Y%m%d-%H%M%S)"
    [ -d "$d" ] && return 0                      # already stored (--stop then relaunch)
    mkdir -p "$d" || return 0
    local n
    for n in dsp pont mobile qemu osmocon bruit; do
        [ -s "$RUNDIR/$n.log" ] && cp -L "$RUNDIR/$n.log" "$d/" 2>/dev/null
    done
    # state of the BSP's dedicated store (stored/played/missed/fallbacks/lost)
    [ -s /dev/shm/calypso_bsp_dedie ] && cp /dev/shm/calypso_bsp_dedie "$d/bsp_dedie.txt" 2>/dev/null
    ls -1dt "$RUNDIR"/archives/*/ 2>/dev/null | tail -n +$((JOURNAUX_GARDES + 1)) | xargs -r rm -rf
    dire "journaux de la session precedente ranges dans $d"
}

# NOISE INJECTOR, INACTIVE BY DEFAULT (BRUIT_MODE and BRUIT_SNR_DB empty: nothing
# changes, not one more line on screen). Placed here, at the end of the file, so that
# the line numbers cited by the wiki (Lancer-a-la-main.md...) stay right; the steps
# above only call bruit_env_dsp, bruit_dire_dsp, bruit_env_pont, bruit_arreter and
# bruit_statut, without effect by default.
# Single source: $QOSMO/tools/injecteur_bruit.py, fallback on tools/injecteur_bruit.py here
# (a link to it).
#   BRUIT_MODE     ber | souple | iq (EXPERIMENTAL) | relais: step b (--step b), between the
#                  DSP and QEMU in the sequence, between the bridge and the L1 in the flow.
#     dsp   : c54x_exe listens on BRUIT_PORT_DSP (16702, CALYPSO_BSP_PORT), the injector takes
#             127.0.0.1:6702 in its place; the bridge does not change (--dsp-port 6702). The BSP
#             learns its uplink peer from the sender of the downlink: its uplink goes through
#             the injector, which hands it unchanged to the bridge (5702) from 6702.
#     grgsm : the injector listens on BRUIT_PORT_GRGSM (14730) and relays to QEMU 4730; the bridge
#             sends there (PONT_GSMTAP_PORT, set at step 5). ber | relais only: QEMU
#             receives already decoded L2 blocks.
#     iq    : refused if CALYPSO_BSP_STREAM=1 (default of start-native.sh --dsp).
#   BRUIT_BER (0..1), BRUIT_RAFALES (L bits), BRUIT_SNR_DB (dB), BRUIT_PERTE (0..1),
#   BRUIT_TN (0 or 1,2...), BRUIT_GRAINE, BRUIT_STATS (10 s), BRUIT_OPTS (extra options),
#   BRUIT_PORT_DSP, BRUIT_PORT_GRGSM, BRUIT_PY.
#   BRUIT_SNR_DB WITHOUT BRUIT_MODE (dsp): no injector, AWGN of the C54x core -- c54x_exe
#   receives CALYPSO_BSP_SNR_DB (and CALYPSO_BSP_BRUIT_GRAINE <- BRUIT_GRAINE), calypso_c54x.c.
# A refused configuration (--verifier) stops run.sh BEFORE any launch.
# Log: $RUNDIR/bruit.log. Details: /opt/GSM/osmo-operator/wiki/Injecteur-bruit.md.
BRUIT_MODE="${BRUIT_MODE:-}"
BRUIT_SNR_DB="${BRUIT_SNR_DB:-}"
BRUIT_PORT_DSP="${BRUIT_PORT_DSP:-16702}"
BRUIT_PORT_GRGSM="${BRUIT_PORT_GRGSM:-14730}"
if [ -z "${BRUIT_PY:-}" ]; then
    BRUIT_PY="$QOSMO/tools/injecteur_bruit.py"
    [ -r "$BRUIT_PY" ] || BRUIT_PY="$HERE/tools/injecteur_bruit.py"
fi
BRUIT_ACTIF=0      # 1: the injector is inserted for this launch (bruit_preparer)
BRUIT_ARGS=()

bruit_args() {   # fills BRUIT_ARGS
    BRUIT_ARGS=(--mode "$BRUIT_MODE" --etiquette c54x --stats "${BRUIT_STATS:-10}")
    if [ "$MODE" = dsp ]; then
        BRUIT_ARGS+=(--cible dsp --ecoute 127.0.0.1:6702 --vers "127.0.0.1:$BRUIT_PORT_DSP")
    else
        BRUIT_ARGS+=(--cible grgsm --ecoute "127.0.0.1:$BRUIT_PORT_GRGSM" --vers 127.0.0.1:4730)
    fi
    [ -n "${BRUIT_BER:-}" ]     && BRUIT_ARGS+=(--ber "$BRUIT_BER")
    [ -n "${BRUIT_RAFALES:-}" ] && BRUIT_ARGS+=(--rafales "$BRUIT_RAFALES")
    [ -n "$BRUIT_SNR_DB" ]      && BRUIT_ARGS+=(--snr-db "$BRUIT_SNR_DB")
    [ -n "${BRUIT_PERTE:-}" ]   && BRUIT_ARGS+=(--perte "$BRUIT_PERTE")
    [ -n "${BRUIT_TN:-}" ]      && BRUIT_ARGS+=(--tn "$BRUIT_TN")
    [ -n "${BRUIT_GRAINE:-}" ]  && BRUIT_ARGS+=(--graine "$BRUIT_GRAINE")
    [ -n "${BRUIT_OPTS:-}" ]    && BRUIT_ARGS+=($BRUIT_OPTS)
    return 0
}

bruit_preparer() {   # before step 1: decides BRUIT_ACTIF, checks the configuration
    [ -n "$BRUIT_MODE" ] || return 0
    if vivant bruit; then BRUIT_ACTIF=1; return 0; fi
    if [ "$MODE" = dsp ] && vivant dsp; then
        # An already running c54x_exe can only be intercepted if it already listens on the moved port.
        if ! tr '\0' '\n' < "/proc/$(pid_de dsp)/environ" 2>/dev/null | grep -qx "CALYPSO_BSP_PORT=$BRUIT_PORT_DSP"; then
            dire "b. BRUIT_MODE=$BRUIT_MODE ignore : c54x_exe tourne deja sur 6702, sans injecteur (./run.sh --stop puis ./run.sh)"
            return 0
        fi
    fi
    [ -r "$BRUIT_PY" ] || rater "injecteur de bruit absent : $BRUIT_PY (posez BRUIT_PY)"
    bruit_args
    local msg
    msg="$(python3 "$BRUIT_PY" "${BRUIT_ARGS[@]}" --verifier 2>&1)" || rater "injecteur de bruit : $(printf '%s\n' "$msg" | tail -1)"
    BRUIT_ACTIF=1
}

bruit_env_dsp() {   # in the c54x_exe subshell; sets nothing by default
    [ "$BRUIT_ACTIF" = 1 ] && export CALYPSO_BSP_PORT="$BRUIT_PORT_DSP"
    if [ -z "$BRUIT_MODE" ] && [ -n "$BRUIT_SNR_DB" ]; then
        export CALYPSO_BSP_SNR_DB="${CALYPSO_BSP_SNR_DB:-$BRUIT_SNR_DB}"
        [ -n "${BRUIT_GRAINE:-}" ] && export CALYPSO_BSP_BRUIT_GRAINE="${CALYPSO_BSP_BRUIT_GRAINE:-$BRUIT_GRAINE}"
    fi
    return 0
}

bruit_dire_dsp() {   # after step 1; silent by default
    [ "$BRUIT_ACTIF" = 1 ] && dire "   BSP deplace sur udp/$BRUIT_PORT_DSP (CALYPSO_BSP_PORT) : 6702 est a l'injecteur de bruit"
    [ -z "$BRUIT_MODE" ] && [ -n "$BRUIT_SNR_DB" ] && dire "   bruit du coeur : CALYPSO_BSP_SNR_DB=${CALYPSO_BSP_SNR_DB:-$BRUIT_SNR_DB} dB${BRUIT_GRAINE:+, graine $BRUIT_GRAINE}"
    return 0
}

bruit_env_pont() {   # in the bridge subshell; sets nothing by default
    [ "$MODE" = grgsm ] && [ -n "$BRUIT_MODE" ] || return 0
    # pont/pont.py would target BRUIT_PORT_GRGSM by itself: it is stated explicitly, and
    # 4730 (QEMU) if the injector is not running, so as not to send into the void.
    if vivant bruit; then export PONT_GSMTAP_PORT="$BRUIT_PORT_GRGSM"; else export PONT_GSMTAP_PORT=4730; fi
    return 0
}

etapeb() {   # the noise injector (BRUIT_MODE), between the bridge and layer 1
    [ "$BRUIT_ACTIF" = 1 ] || return 0
    vivant bruit && { dire "b. injecteur de bruit deja lance (pid $(pid_de bruit))"; return; }
    bruit_args
    : > "$RUNDIR/bruit.log"
    python3 -u "$BRUIT_PY" "${BRUIT_ARGS[@]}" >> "$RUNDIR/bruit.log" 2>&1 &
    echo $! > "$RUNDIR/bruit.pid"
    attendre 5 grep -aq "PRET ecoute=" "$RUNDIR/bruit.log" || rater "l'injecteur de bruit ne s'est pas annonce (voir $RUNDIR/bruit.log)"
    dire "b. injecteur de bruit  pid $(pid_de bruit)  mode $BRUIT_MODE  $(sed -n 's/.*PRET \(ecoute=[^ ]*\) \(vers=[^ ]*\).*/\1 \2/p' "$RUNDIR/bruit.log" | head -1)  journal $RUNDIR/bruit.log"
}

bruit_arreter() {   # --stop: the injector without pid (launched by hand with --etiquette c54x)
    pkill -f -- "injecteur_bruit\.py .*--etiquette c54x" 2>/dev/null
    return 0
}

bruit_statut() {   # --status: silent by default
    if vivant bruit; then
        printf '  %-8s pid %-7s  %s\n' bruit "$(pid_de bruit)" "$RUNDIR/bruit.log"
        grep -a "STATS\|BILAN" "$RUNDIR/bruit.log" 2>/dev/null | tail -1
    elif [ -n "$BRUIT_MODE" ]; then
        printf '  %-8s arrete\n' bruit
    fi
    return 0
}

mkdir -p "$RUNDIR"
case "${1:-}" in
    --stop)   arreter; garder_journaux
              # phone 2 of the TRX (instance 3) goes down with instance 1
              if [ "$INSTANCE" = 1 ] && [ -d "/tmp/c54x-pont$OSMO_TRX_TS1_INSTANCE" ] && [ -f "/tmp/c54x-pont$OSMO_TRX_TS1_INSTANCE/dsp.pid" ]; then
                  INSTANCE="$OSMO_TRX_TS1_INSTANCE" OSMO_TRX=1 OSMO_TRX_PHONE=2 AIR=0 bash "$0" --stop
              fi
              # the mobile over the air (instance 2) too: left alive across start-native.sh --bts --ms
              # relaunches, the air would keep the OLD TX ring of instance 1 open (file deleted): A->B
              # frozen, no more downlink for the mobile, hence no more LU.
              if [ "$INSTANCE" = 1 ] && [ -d /tmp/c54x-pont2 ] && ls /tmp/c54x-pont2/*.pid >/dev/null 2>&1; then
                  INSTANCE=2 AIR=1 OSMO_TRX=0 bash "$0" --stop
              fi
              # emptied once stored: the next launch does not store them twice
              [ -f "$RUNDIR/dsp.log" ] && : > "$RUNDIR/dsp.log"
              [ -f /dev/shm/pont.log ] && : > /dev/shm/pont.log ;;
    --status) statut ;;
    --charge) charge ;;
    --logs)   # coloured on display (qosmo/tools/qosmo-couleurs.py, QOSMO_COULEURS=0 turns it off)
              _coul="$QOSMO/tools/qosmo-couleurs.py"
              if [ "${QOSMO_COULEURS:-1}" != 0 ] && [ -f "$_coul" ] && [ -t 1 ]; then
                  exec tail -n 5 -F "$RUNDIR"/dsp.log "$RUNDIR"/qemu.log "$RUNDIR"/osmocon.log "$RUNDIR"/mobile.log "$RUNDIR"/transceiver.log "$RUNDIR"/pont.log "$RUNDIR"/air.log $([ -n "$BRUIT_MODE" ] || [ -f "$RUNDIR/bruit.pid" ] && echo "$RUNDIR/bruit.log") 2>/dev/null | python3 -u "$_coul" --force
              fi
              exec tail -n 5 -F "$RUNDIR"/dsp.log "$RUNDIR"/qemu.log "$RUNDIR"/osmocon.log "$RUNDIR"/mobile.log "$RUNDIR"/transceiver.log "$RUNDIR"/pont.log "$RUNDIR"/air.log $([ -n "$BRUIT_MODE" ] || [ -f "$RUNDIR/bruit.pid" ] && echo "$RUNDIR/bruit.log") 2>/dev/null ;;
    --step)   case "${2:-}" in 1) bruit_preparer; etape1;; b) bruit_preparer; etapeb;; 2) etape2;; 3) etape3;; 4) etape4;; 5) etape5;; a) etape5_air;; *) rater "--step 1|b|2|3|4|5|a";; esac ;;
    -h|--help) sed -n '2,22p' "$0" ;;
    "")       garder_journaux; bruit_preparer; etape1; etapeb; etape2; etape3; etape4; etape5
              dire "tout tourne. Journaux : $RUNDIR/*.log   suivre : ./run.sh --logs   arreter : ./run.sh --stop" ;;
    *)        rater "option inconnue : $1 (voir --help)" ;;
esac
