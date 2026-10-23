#!/usr/bin/env bash
# run.sh - le cote MOBILE du banc Calypso, processus par processus, sans le reseau.
#
# Deux montages (MODE) :
#   dsp    (defaut)  1. c54x_exe --arm   le DSP TMS320C54x, mask-ROM TI, hors QEMU
#                    2. qosmo            l'ARM + layer1 osmocom-bb (CALYPSO_DSP_EXTERN=1)
#                    3. osmocon          romload sur le pty serial0, relais L1CTL
#                    4. mobile           couche 2/3 osmocom-bb
#                    5. pont_dsp.py      (PONT=1) bursts du BTS -> DSP (--dsp-port 6702) ;
#                                        le pont DSP (pont/dsp/), bascule TCH suivie par le firmware
#   grgsm            2. qosmo            l'ARM + layer1 avec la couche 1 gr-gsm (shunt)
#                    3. osmocon  4. mobile  5. pont.py (PONT=1) bursts du BTS -> L1 gr-gsm
# Le pont a besoin du BTS (osmo-bts-trx, TRXD 5700) : sans reseau il demarre et attend.
#   OSMO_TRX=1       [2026-10-19] LE TELEPHONE EST LE TRX DE LA BTS#0 (start-native.sh --osmo-trx) :
#                    2. qosmo sur trx.highram.elf (osmocom-bb-transceiver, appli « trx » de Sylvain Munaut)
#                    4. transceiver (hote, osmocom-bb-transceiver, patches/osmocom-bb-transceiver-qemu-bench)
#                       A LA PLACE de mobile : L1CTL sur $L2_SOCK, TRXD 5700-5702 face a osmo-bts-trx,
#                       --no-sync (aucune cellule a ecouter), un seul timeslot RX (OSMO_TRX_TS_MASK=0x01) ;
#                       le masque TX reste 0xe3 (TS 5,6,7,0,1) : prim_bts.c ne sert le TS0 BCCH que si
#                       tx_start == 5 (fenetre TX decalee de 3 slots). [2026-10-20] -m 0x01 posait aussi
#                       TX=0x01 -> tx_start 0 -> dummy a chaque trame (DUMMY 18000, FB 0, SB 0).
#                    5. pas de pont : il tiendrait les memes ports 5700-5702 (PONT force a 0)
#                    OSMO_TRX_ARGS (defaut « --no-sync -e 3 », + « -r $OSMO_TRX_RT » SCHED_RR 20), TRANSCEIVER (chemin du binaire).
#   INSTANCE=2       [2026-10-19] UNE DEUXIEME CHAINE COMPLETE a cote de la premiere (banc-mobile pendant
#                    que banc-trx tourne) : run.sh se relance dans un espace de montage prive ou
#                    /dev/shm EST /dev/shm/calypso2 (vu de l'hote), et le vrai /dev/shm de l'hote est
#                    visible en /run/calypso-shm-hote ; aucun chemin /dev/shm/... du code ne change.
#                    RUNDIR /tmp/c54x-pont2, L1CTL /tmp/osmocom_l2_2, socket DSP /tmp/calypso_dsp2.sock,
#                    moniteur, gdb 1235/44445, clavier /tmp/calypso_keys2.sock, BSP udp 6712, VTY mobile
#                    +100, 4347 (copie de mobile_pont.cfg ; 4257 est a osmo-ggsn), PANNEAU_LOGS=none. AIR=1 (defaut en instance 2) :
#                    etape 5 = pont/air.py, l'air entre les deux Calypsos (bursts TX de l'un vers le BSP
#                    de l'autre) a la place du pont.
#   FIRMWARE_APP=rssi  [2026-10-19] une appli firmware SEULE sur l'ARM + le DSP (rssi, menu,
#                    hello_world, snake_game... : board/compal_e88/<app>.highram.elf) : etapes 1-3,
#                    ni mobile ni pont. L'ecran et le clavier du C123 sont emules (qosmo :
#                    calypso_st7558.c -> /dev/shm/calypso_lcd, calypso_armio.c <- /tmp/calypso_keys.sock) ;
#                    la fenetre : /usr/bin/python3 /opt/GSM/osmo-operator/tools/osmo-c123-gtk.py
#
#   ./run.sh                 tout lancer            ./run.sh --status    qui tourne
#   MODE=grgsm ./run.sh      montage gr-gsm         ./run.sh --logs      suivre les journaux
#   PONT=1 ./run.sh          avec le pont           ./run.sh --stop      tout arreter, nettoyer
#   ./run.sh --step N        une seule etape (les precedentes doivent tourner)
# Variables : MODE, PONT, LOCKSTEP (1 : QEMU attend le DSP a chaque trame), INSNS (80000),
#   VERB (-v), IQ (none|fcch|cell|...), AMP (30000),
#   QOSMO, FIRMWARE_ELF, FIRMWARE_BIN, OSMOCON, MOBILE, MOBILE_CFG, PONT_PY, RUNDIR, L2_SOCK.
#   UL_RAW (1 : le pont emet les bursts finaux de la ROM tels quels ; 0 : codage hote), MONTANT_EFFACE_TASK_RA.
#   Temps reel : RT_NICE (-10 par defaut, vide = sans), RT_FIFO (SCHED_FIFO, ex. 20, root), RT_CPUS (taskset,
#   ex. 1-3), RT_RENICE_AUTRES (ex. 15 : renice des outils du panneau) ; ./run.sh --charge  qui consomme quoi.
# Details, attendus et verifications : LAUNCH.md a cote. Bruit (BRUIT_MODE, inactif par defaut) : bloc BRUIT_ en fin.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${MODE:-dsp}"
PONT="${PONT:-0}"
# [2026-10-19] INSTANCE=N (N != 1) : /dev/shm prive = /dev/shm/calypsoN de l'hote ; le vrai /dev/shm
# reste lisible en /run/calypso-shm-hote (l'air y lit l'anneau TX de l'instance 1). Les processus
# lances ici heritent de l'espace ; --stop/--status relances avec INSTANCE=N y entrent aussi.
INSTANCE="${INSTANCE:-1}"
# [2026-10-21] REGLAGES PAR INSTANCE : instances/instance-$INSTANCE.env (ecrit par la page web tools/osmo-reglages.py,
# patron instances/instance.env.example). Lignes `: "${VAR:=valeur}"` : l'environnement (start-native.sh, ligne de
# commande) garde la priorite, puis le fichier, puis les defauts ci-dessous. Exporte (set -a) pour c54x_exe/QEMU/pont.
INSTANCE_ENV="${INSTANCE_ENV:-/opt/GSM/osmo-operator/instances/instance-$INSTANCE.env}"
if [ -r "$INSTANCE_ENV" ]; then
    set -a; . "$INSTANCE_ENV"; set +a
    printf '\033[1m[run dsp]\033[0m instance %s : reglages de %s\n' "$INSTANCE" "$INSTANCE_ENV"
fi
HOTE_SHM=/run/calypso-shm-hote
AIR_DEMANDE="${AIR:-}"   # [2026-10-23] AIR pose par l appelant (sinon defaut par instance / appli, plus bas)
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
    # [2026-10-21] instance >= 3 : +30, car 6722 (= 6702 + 20) est le port TRXD « MS » de fake_trx.py (-p 6720, sidecar de
    # la BTS 1) : il se liait sur 127.0.0.1:6722 et volait les bursts montants du telephone 2 (instance 3 : 6752).
    export CALYPSO_BSP_PORT="${CALYPSO_BSP_PORT:-$((6702 + 10 * (INSTANCE - 1) + (INSTANCE >= 3 ? 30 : 0)))}"
    export CALYPSO_KEYS_SOCK="${CALYPSO_KEYS_SOCK:-/tmp/calypso_keys$INSTANCE.sock}"
    AIR="${AIR:-1}"
fi
AIR="${AIR:-0}"
AIR_PY="${AIR_PY:-/opt/GSM/osmo-operator/pont/air.py}"
QOSMO="${QOSMO:-/opt/GSM/qosmo}"
QEMU="${QEMU:-$QOSMO/build/qemu-system-arm}"
# Firmware : layer1.highram d'origine, non modifie. [2026-10-04] La variante layer1.highram_patched
# (trames B_BFI du TCH descendant passees a charge nulle, masquees par l'ECU de GAPK ; be10de1) est
# retiree : le B_BFI de la ROM est juste depuis qosmo a19dde59, et le but est le firmware tel quel.
# FIRMWARE_ELF=... pour un autre binaire.
_FW=/opt/GSM/firmware/board/compal_e88
# [2026-10-19] OSMO_TRX=1 : firmware trx.highram (le Calypso emet la BTS), appli hote transceiver.
OSMO_TRX="${OSMO_TRX:-0}"
FIRMWARE_APP="${FIRMWARE_APP:-}"
if [ -n "$FIRMWARE_APP" ]; then
    FIRMWARE_ELF="$_FW/$FIRMWARE_APP.highram.elf"   # s'impose aussi face a /etc/default/osmo-banc
    FIRMWARE_BIN="${FIRMWARE_ELF%.elf}.bin"
    PONT=0
fi
# [2026-10-23] RSSI : l'air realiste par defaut (calypso_bsp.c CALYPSO_AIR_NIVEAUX : la cellule seulement sur son
# ARFCN, niveaux DL/UL, bruit ailleurs) et, en BTS classique (pont_dsp.py vivant), pas d'air.py : le descendant arrive
# par la copie du pont (PONT_DSP_COPIES, udp 6762). Ici et non dans banc-rssi : un terminal ouvert avant la mise a
# jour des alias, ou un autre lanceur, relancait rssi avec l'ancien comportement.
if [ "$FIRMWARE_APP" = rssi ]; then
    export CALYPSO_AIR_NIVEAUX="${CALYPSO_AIR_NIVEAUX:-1}"
    export CALYPSO_FAUX_AIR="${CALYPSO_FAUX_AIR:-$( . /etc/osmocom/radio-plan.env 2>/dev/null; echo "${PLAN_ARFCN:-}")}"
    [ -z "$AIR_DEMANDE" ] && pgrep -f 'pont/pont_dsp\.py' >/dev/null && AIR=0
fi
# [2026-10-20] OSMO_TRX_TS1=1 (instance 1, avec OSMO_TRX=1) : un DEUXIEME Calypso TRX pour le TS1 (TCH/F, les appels) :
# l'instance OSMO_TRX_TS1_INSTANCE (3) est lancee avant le transceiver (etapes 1-3 seules, OSMO_TRX_PHONE=2 : firmware trx,
# L1CTL /tmp/osmocom_l2_3, pas de transceiver, pas d'air) et le transceiver prend « -2 -L /tmp/osmocom_l2_3 » (telephone 2 =
# TS1 TX+RX, prim_bts.c SLOT 0 / tn 1). Son BSP (udp 6752 depuis le 21/10, 6722 avant) n'a pas de SB pour se caler : CALYPSO_BSP_TRX_OFFSET=-1 (fn du
# firmware = tick - 1, mesure sur le telephone 1). L'air (instance 2) relie aussi cet anneau (--c-ring/--c-port).
# [2026-10-20 23:10] -2 par defaut : OSMO_TRX_TS1=1 (start-native.sh --ts 1 ou OSMO_TRX_TS1=0 pour un seul telephone)
# [2026-10-21] retour a OSMO_TRX_TS1=0 par defaut : le 2e telephone a un fn independant (TCH descendant a ~80 bits
# faux pour le mobile, ASSIGNMENT FAILURE), seul le montage UN Calypso TS0+TS1 (OSMO_TRX_TCH=1) passe un appel.
# start-native.sh --ts 2 (ou OSMO_TRX_TS1=1) pour le montage a deux telephones.
OSMO_TRX_TS1="${OSMO_TRX_TS1:-0}"
OSMO_TRX_TS1_INSTANCE="${OSMO_TRX_TS1_INSTANCE:-3}"
OSMO_TRX_PHONE="${OSMO_TRX_PHONE:-1}"
# [2026-10-20] Le mobile par l'air (INSTANCE=N, AIR=1) n'est jamais le TRX : start-native.sh --bts exporte OSMO_TRX=1
# et l'instance 2 en heritait (firmware trx + second transceiver -> « port UDP 5700 deja pris »).
if [ "$OSMO_TRX" = 1 ] && [ "$AIR" = 1 ] && [ "$INSTANCE" != 1 ] && [ "$OSMO_TRX_PHONE" != 2 ]; then
    printf '\033[1m[run dsp]\033[0m INSTANCE=%s AIR=1 : OSMO_TRX=1 herite ignore, ce Calypso est le mobile (le TRX est l instance 1)\n' "$INSTANCE"
    OSMO_TRX=0
fi
if [ "$OSMO_TRX" = 1 ]; then
    # Le firmware trx S'IMPOSE : /etc/default/osmo-banc (profil GPRS) pose FIRMWARE_ELF=layer1.highram.gprs
    # dans l'environnement, et un « ${FIRMWARE_ELF:-trx} » le laissait passer : le transceiver parlait
    # BTS_MODE (0x1f) a un layer1 qui ne le connait pas. OSMO_TRX_FIRMWARE=<elf> pour un autre trx.
    [ -n "${FIRMWARE_ELF:-}" ] && [ "${FIRMWARE_ELF##*/}" != "${OSMO_TRX_FIRMWARE:-trx.highram.elf}" ] \
        && printf '\033[1m[run dsp]\033[0m OSMO_TRX=1 : firmware %s ignore, trx.highram.elf a la place\n' "$FIRMWARE_ELF"
    FIRMWARE_ELF="${OSMO_TRX_FIRMWARE:-$_FW/trx.highram.elf}"
    FIRMWARE_BIN="${FIRMWARE_ELF%.elf}.bin"
    # BSIC de la cellule : osmo-bts-trx n'envoie que SETTSC (BCC), le NCC vient du bts 0 du BSC
    OSMO_TRX_BSIC="${OSMO_TRX_BSIC:-$(awk '/^ *bts 0$/{f=1} f&&/base_station_id_code/{print $2; exit}' /etc/osmocom/osmo-bsc.cfg 2>/dev/null)}"
    TRANSCEIVER="${TRANSCEIVER:-/opt/GSM/osmocom-bb-transceiver/src/host/layer23/src/transceiver/transceiver}"
    [ -x "$TRANSCEIVER" ] || TRANSCEIVER="$(command -v transceiver || echo "$TRANSCEIVER")"
    # [2026-10-20] OSMO_TRX_TCH=1 (defaut) : le telephone sert TS0 ET TS1 (TCH/F, les appels) : masques 0x03,
    # le firmware patche emet les deux slots de sa fenetre TX (prim_bts.c) et remonte le TS1 depuis rx[2] ;
    # c54x_exe publie les deux bursts depuis la page (CALYPSO_TRX_TX_SLOTS=2) et ecrit le montant TS1 dans
    # la page. OSMO_TRX_TCH=0 : TS0 seul (0x01).
    # [2026-10-20 23:10] avec le telephone 2 (OSMO_TRX_TS1=1, defaut) le telephone 1 garde le TS0 seul : OSMO_TRX_TCH=0
    OSMO_TRX_TCH="${OSMO_TRX_TCH:-$([ "$OSMO_TRX_TS1" = 1 ] && echo 0 || echo 1)}"
    if [ "$OSMO_TRX_TCH" = 1 ] && [ "$OSMO_TRX_PHONE" != 2 ]; then
        OSMO_TRX_TS_MASK="${OSMO_TRX_TS_MASK:-0x03}"
        OSMO_TRX_TX_MASK="${OSMO_TRX_TX_MASK:-0x03}"
        export CALYPSO_TRX_TX_SLOTS="${CALYPSO_TRX_TX_SLOTS:-2}"
    fi
    OSMO_TRX_TS_MASK="${OSMO_TRX_TS_MASK:-0x01}"
    # [2026-10-20] masque TX 0x01 : UNE fenetre TX (tx_start 0, slot TPU 5 = TS0 descendant), celle que la
    # ROM joue comme un RACH et que c54x_exe capture (script TSP 0x3cbb). Le 0xe3 d'origine (fenetre
    # multiple 5,6,7,0,1) n'est pas servi par le banc : seul le premier burst (dummy) sortait. Le firmware
    # trx corrige (prim_bts.c, SLOT/tn) vise le TS0 quel que soit tx_start.
    OSMO_TRX_TX_MASK="${OSMO_TRX_TX_MASK:-0x01}"
    # [2026-10-20 22:25] -r PRIO (SCHED_RR, main.c rt_prio ; OSMO_TRX_RT= vide pour s'en passer) ; -e 1 pour le debug (23:10 : -e 3 par defaut)
    OSMO_TRX_RT="${OSMO_TRX_RT-20}"
    OSMO_TRX_ARGS="${OSMO_TRX_ARGS:---no-sync -e 3}${OSMO_TRX_RT:+ -r $OSMO_TRX_RT}"
    PONT=0
    # la BTS repete ses bursts (dummy, FB) : c54x_exe publie chaque trame avec tache TX, pas que les nouveaux
    export MONTANT_ROM_TX_CHAQUE="${MONTANT_ROM_TX_CHAQUE:-1}"
    # [2026-10-20] le BSP joue les bursts montants (air.py, fn GSM du mobile) a la trame du firmware trx,
    # calee sur le fn du SB emis (pont.c -> calypso_bsp_trx_calage) ; CALYPSO_BSP_TRX_DELTA ajuste (defaut -2 avec le
    # firmware patche BTS_RX_LAG 3 : le burst montant fn X est joue une trame apres la fenetre native, marge pour l'air)
    export CALYPSO_BSP_TRX="${CALYPSO_BSP_TRX:-1}"
    # [2026-10-21] firmware trx BTS_RX_LAG 12 (prim_bts.c) : le burst montant fn X est joue 11 trames apres sa fenetre
    # native ; marge pour la latence aller-retour de l'air + le retard du mobile emule (RACH du Magnetite arrives
    # 27 trames trop tard avec LAG 4 / delta -3)
    export CALYPSO_BSP_TRX_DELTA="${CALYPSO_BSP_TRX_DELTA:--11}"
    # telephone 2 (TS1) : pas de SB emis, offset fn/tick fixe (voir OSMO_TRX_TS1)
    [ "$OSMO_TRX_PHONE" = 2 ] && export CALYPSO_BSP_TRX_OFFSET="${CALYPSO_BSP_TRX_OFFSET:--1}"
fi
FIRMWARE_ELF="${FIRMWARE_ELF:-$_FW/layer1.highram.elf}"
FIRMWARE_BIN="${FIRMWARE_BIN:-${FIRMWARE_ELF%.elf}.bin}"
# [2026-10-21] FIRMWARE_FLASH=<image 4 MiB> : un VRAI firmware en flash (FreeCalypso Magnetite,
# cible c139 : bootloader Compal en secteur 0 + fwimage.bin a 0x10000, voir
# osmo-operator/scripts/magnetite-c139.sh et doc/magnetite.md) a la place du layer1 osmocom-bb.
# QEMU demarre a 0 dans la flash (pas de -kernel, CPU ti925t = ARMv4T comme l'ARM7TDMI),
# l'ecran est celui du C139 (CALYPSO_LCD=c139, uWire), la pile L23 et l'UI sont DANS le
# firmware : pas d'osmocon (rvinterf decode les traces RVTMUX sur le pty modem, socket
# $RUNDIR/rvinterf.sock pour fc-shell / fc-fsio), pas de mobile. Le DSP (etape 1) et l'air
# ou le pont (etape 5) sont inchanges. start-native.sh --ms-magnetite le pose.
FIRMWARE_FLASH="${FIRMWARE_FLASH:-}"
# [2026-10-21 11:10] le TRX de la BTS (OSMO_TRX=1) ou le telephone 2 du TRX ne sont JAMAIS Magnetite
if [ -n "$FIRMWARE_FLASH" ] && { [ "$OSMO_TRX" = 1 ] || [ "$OSMO_TRX_PHONE" = 2 ]; }; then
    printf '\033[1m[run dsp]\033[0m OSMO_TRX=1 : FIRMWARE_FLASH=%s ignore, cette instance est le TRX de la BTS (firmware trx)\n' "$FIRMWARE_FLASH"
    FIRMWARE_FLASH=""
fi
# [2026-10-21] Magnetite (vrai firmware TI en flash) : pas de rattrapage du pacer. Avec le rattrapage (20),
# l ARM enchaine les trames en retard, la L1 sature et le chien de garde reset le firmware (gel).
if [ -n "$FIRMWARE_FLASH" ]; then
    export CALYPSO_PACER_RATTRAPAGE="${CALYPSO_PACER_RATTRAPAGE:-0}"
    export CALYPSO_PONT_ARM_LOCK="${CALYPSO_PONT_ARM_LOCK:-1}"
    # [2026-10-21] L1 TI : fenetre SB ouverte en fin de trame precedente (sinon BCCH decale d'une trame,
    # CRC KO sur chaque bloc) et TOA NB vise a 3 (marge 1) ; voir calypso_bsp.c sb_fin_trame()
    export CALYPSO_BSP_SB_FIN_TRAME="${CALYPSO_BSP_SB_FIN_TRAME:-1}"
    export CALYPSO_BSP_NB_MARGE="${CALYPSO_BSP_NB_MARGE:-1}"
    # TOA NB canne a la cible TI (pont.c). [21/10 17:05] coupe par defaut tant que c54x_exe n'est pas
    # recompile avec la garde init_done : sans elle le cannage ecrase le code DSP pendant le boot.
    export PONT_CAN_NB_TOA="${PONT_CAN_NB_TOA:-}"
    # [21/10] faux air : bruit hors de l'ARFCN de la cellule (calypso_bsp.c faux_air_*), ARFCN du plan radio
    # [21/10 19:50] SB module comme FB/NB : les reglages SB du 30/09 (instant 0.35, elargissement 1.0, bruit 3000,
    # faits pour la layer1 osmocom qui ignore l'angle du SB) biaisaient l'angle mesure (+166 a 0 Hz) ; la L1 TI
    # s'en sert pour l'AFC (-700 -> -673, ~224 Hz de vrai decalage) et perdait SB puis BCCH
    export CALYPSO_BSP_SB_DEC="${CALYPSO_BSP_SB_DEC:-0.5}" CALYPSO_BSP_SB_SYM="${CALYPSO_BSP_SB_SYM:-0.3}" CALYPSO_BSP_SB_NOISE="${CALYPSO_BSP_SB_NOISE:-0}"
    export CALYPSO_BSP_BRUIT_FOND="${CALYPSO_BSP_BRUIT_FOND:-1}"   # bruit au lieu de zeros (PM = 0 -> recovery L1)
    # rattraper le direct du BTS (calypso_bsp.c) : PAR L'AIR SEULEMENT (AIR=1, la BTS a sa propre horloge). [21/10 21:37]
    # en BTS classique pont_dsp.py cadence osmo-bts sur les trames livrees par le DSP : chaque saut de +33 accelerait la
    # BTS (pont a ~350 trames/s), d'ou plus de retard et plus de sauts ; RA_C 1708/1933/2165 contre RACH du pont a
    # 1737/2059/2457, IMM ASS jamais reconnues.
    export CALYPSO_BSP_RATTRAPE="${CALYPSO_BSP_RATTRAPE:-$AIR}"
    export CALYPSO_RACH_VERS_AIR="${CALYPSO_RACH_VERS_AIR:-1}"   # RACH du firmware TI publie vers l'air (pont.c)
    # [21/10 21:55] BTS classique : les bursts SDCCH/SACCH du firmware TI sont bien dans calypso_tx_rom (TSC 7, fn de
    # la ROM), mais pont_dsp les jetait faute de canal dedie annonce (calypso_dcch_cfg vient du tap L1CTL de QEMU,
    # absent sans layer1 osmocom) : « aucun canal dedie connu apres 300 ms, perdu », « DL: >N200 of SABM ».
    # PONT_UL_TN_DEFAUT=0 : TS0 (SDCCH/4 du banc) faute d'annonce. PONT_UL_RAW_DCCH_HOTE=1 (codage hote) reste dispo.
    export PONT_UL_TN_DEFAUT="${PONT_UL_TN_DEFAUT:-0}"
    # [21/10 23:05] TCH montant par l'HOTE (FACCH, SACCH/T, parole codees par le pont depuis a_fu/a_du) : le script TSP
    # de la ROM ne sort que 2 bursts « inconnus » sur le TCH du firmware TI, le SABM FACCH n'atteignait jamais la BTS
    # (BSC : rll_ready=no, ASSIGNMENT FAILURE, « >N200 of SABM »). 14-demarrage.sh exporte PONT_UL_RAW_TCH=1 pour le
    # vocodeur ROM du mobile osmocom : on force 0 ici (PONT_UL_RAW_TCH_MAGNETITE pour revenir dessus).
    export PONT_UL_RAW_TCH="${PONT_UL_RAW_TCH_MAGNETITE:-0}"
    # [21/10 21:35] PAS de correction de fn ni de RA (AIR_B_FN_DELTA, MONTANT_RACH_DIFFERE : 0). Les ecarts vus a 21:05
    # et 21:25 comparaient au RA_R de la L1 TI, qui trace la DEMANDE (MPHC_RA_REQ) ; le RACH part apres le tirage
    # aleatoire, juste avant RA_C (RA_R 9394 ra 12 -> RA_C 9403 ; c54x_exe publie ra 12 a 9403) : fn et RA etaient justes.
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
# [2026-09-23] Deux points d'entree pour le meme paquet pont/ : pont_dsp.py pour le
# montage dsp (--dsp-port 6702), pont.py pour le montage grgsm, qui
# garde ses defauts d'avant le decoupage. PONT_PY force l'un ou l'autre.
if [ "$MODE" = grgsm ]; then
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont.py}"
else
    PONT_PY="${PONT_PY:-/opt/GSM/osmo-operator/pont/pont_dsp.py}"
fi
RUNDIR="${RUNDIR:-/tmp/c54x-pont}"
# [2026-10-04] TEMPS REEL. Sous charge (campagne de tests, panneau, ffmpeg) la chaine c54x_exe <-> QEMU
# <-> osmocon/mobile <-> pont rate des ticks et le son se coupe : la trame fait 4,615 ms, le travail
# mesure ~1,2 ms ([chrono] 2026-09-23 : A 0.33 + go 0.40 + B 0.16 + qemu <0.32), et l'ordonnanceur CFS
# met volontiers un processus de cote 5 a 10 ms quand les coeurs sont pris. Les cinq processus de la
# chaine sont donc lances avec une priorite : RT_NICE=-10 par defaut (RT_NICE= vide pour ne rien faire),
# RT_FIFO=<prio> pour SCHED_FIFO (chrt -f, root ; rester sous 50, les threads noyau sont au-dessus),
# RT_CPUS=<liste> pour les epingler (taskset -c, ex. 1-3 : le coeur 0 reste au reste du systeme).
# RT_RENICE_AUTRES=<n> abaisse en plus les outils du panneau (osmo-fft-snap, osmo-panel, osmo-topzone,
# osmo-ts-probe, conky) a nice n. ./run.sh --charge montre qui consomme quoi.
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
GDB="${GDB:-1}"                              # gdbstub QEMU + console telnet (etape 2)
GDB_STUB="${GDB_STUB:-1234}"
GDB_TELNET="${GDB_TELNET:-44444}"
GDB_TELNET_PY="${GDB_TELNET_PY:-$QOSMO/tools/gdb-telnet.py}"   # [2026-10-03] etait /opt/GSM/qosmo-dsp/... (depot disparu)
# [2026-10-21] 120000 par defaut : c est le budget du DSP du TRX (instance 1, boote au premier IDLE) ;
# 16000 laissait le DSP de l instance Magnetite en « occupe boot » pour toujours.
INSNS="${INSNS:-120000}"   # [2026-09-23] 60000 debordait en TCH (jusqu a 87000 insn/trame), voir start-native.sh
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
DSP_SOCK="${DSP_SOCK:-/tmp/calypso_dsp.sock}"
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
    renice_autres
    [ "$MODE" = dsp ] || { dire "1. (montage grgsm : pas de c54x_exe, la couche 1 est dans QEMU)"; return; }
    vivant dsp && { dire "1. c54x_exe deja lance (pid $(pid_de dsp))"; return; }
    [ -x "$HERE/c54x_exe" ] || make -C "$HERE" >/dev/null || rater "make c54x_exe"
    rm -f "$DSP_SHM" "$DSP_SOCK"
    # [2026-09-23] L'annonce TCH d'une session morte (pont/dsp/tch.py) ne doit
    # pas etre relue par montant.c au demarrage.
    rm -f /dev/shm/calypso_tch_cfg
    # [2026-09-23] Attendre (au plus 40 ms) une trame que la BTS livre en
    # retard plutot que la jouer en effacement (calypso_bsp.c, bsp_attendre_trame).
    ( cd "$HERE" && bruit_env_dsp && CALYPSO_IQDUMP_FCCH=1 CALYPSO_BSP_ATTENTE_MS="${CALYPSO_BSP_ATTENTE_MS:-40}" exec ${PRIO[@]+"${PRIO[@]}"} "${C54X_BIN:-./c54x_exe}" --arm "$DSP_SOCK" --insns "$INSNS" --iq "$IQ" --amp "$AMP" $VERB ) > "$RUNDIR/dsp.log" 2>&1 &
    echo $! > "$RUNDIR/dsp.pid"
    attendre 5 test -S "$DSP_SOCK" || rater "c54x_exe n'a pas ouvert $DSP_SOCK (voir $RUNDIR/dsp.log)"
    # [2026-10-06] shm en 0666 : le umask rabaisse la demande 0666 de c54x_exe a 0644, ce qui empeche
    # un harvard.sh lance sous le compte utilisateur d'ecrire l'API RAM (d@!, demos sb/si3). On le force.
    attendre 3 test -e "$DSP_SHM" && chmod 0666 "$DSP_SHM" 2>/dev/null || true
    dire "1. c54x_exe --arm  pid $(pid_de dsp)  ($INSNS insn/trame, iq=$IQ, $DSP_SHM, $DSP_SOCK)"; bruit_dire_dsp
}

etape2() {   # l'ARM
    vivant qemu && { dire "2. QEMU deja lance (pid $(pid_de qemu))"; return; }
    [ -x "$QEMU" ] || rater "QEMU absent : $QEMU (ninja -C $QOSMO/build)"
    [ -r "$FIRMWARE_ELF" ] || rater "firmware absent : $FIRMWARE_ELF"
    local extern=""
    if [ "$MODE" = dsp ]; then [ -S "$DSP_SOCK" ] || rater "le DSP ne tourne pas (etape 1 d'abord)"; extern="$DSP_SOCK"; fi
    rm -f "$MONITOR"
    vitrine qemu
    # [2026-09-23] gdb sur l'ARM : gdbstub QEMU en tcp::$GDB_STUB, et la console
    # telnet de qosmo-dsp (tools/gdb-telnet.py) sur le port $GDB_TELNET :
    #     telnet 0 44444      (Ctrl-C arrete l'ARM, « continue & » le relance)
    # Tant que personne n'est connecte, gdb ne tourne pas et l'ARM n'est pas
    # touche. GDB=0 : ni stub ni console.
    local gdb_opt=()
    [ "$GDB" = 1 ] && gdb_opt=(-gdb "tcp:127.0.0.1:$GDB_STUB")
    # [2026-09-23] ASSEMBLY_LOGS=1 (start-native.sh --assembly-logs) : trace asm
    # de l'ARM dans qemu-asm.log, chaque bloc une fois, a sa traduction.
    # [2026-10-19] in_asm seul par defaut. ASSEMBLY_LOGS_FLAGS=in_asm,exec,nochain
    # journalise aussi chaque bloc execute (nochain sinon les blocs chaines ne
    # sont vus qu'une fois) : l'ARM passait de 0,4 a 62 ms par trame ([chrono]
    # go), le DSP en pas-a-pas suivait, mais l'horloge asservie du pont
    # (osmo-operator/pont/trx.py, plafond 4 x FRAME_DUR) prenait +1159 a +3897
    # trames d'avance, le BSP ne trouvait plus les bursts (« le burst a existe
    # puis a disparu ») et mobile ne voyait aucune cellule (FBSB result=255).
    # [2026-10-19] LA TRACE VA SUR LE DISQUE, PAS SUR LE TMPFS. $RUNDIR est dans
    # /tmp (tmpfs de 4,7 Go) : avec exec,nochain la trace a rempli /tmp en cinq
    # minutes (3,8 Go), et un rm ne libere rien tant que QEMU garde le fichier
    # ouvert (il faut tronquer /proc/<pid>/fd/N). On ecrit dans
    # ASSEMBLY_LOGS_DIR (/var/tmp/c54x-pont, sur le disque) et $RUNDIR/qemu-asm.log
    # devient un lien ; les archives n'y touchent pas (garder_journaux).
    if [ "${ASSEMBLY_LOGS:-0}" = 1 ]; then
        local asm_dir="${ASSEMBLY_LOGS_DIR:-/var/tmp/c54x-pont}"
        mkdir -p "$asm_dir" 2>/dev/null || asm_dir="$RUNDIR"
        rm -f "$RUNDIR/qemu-asm.log" "$asm_dir/qemu-asm.log"
        [ "$asm_dir" != "$RUNDIR" ] && ln -s "$asm_dir/qemu-asm.log" "$RUNDIR/qemu-asm.log"
        local asm_cible="$asm_dir/qemu-asm.log"
        # [2026-10-19] ASSEMBLY_LOGS_FIFO=1 : QEMU ecrit dans un FIFO sur /dev/shm et un
        # cat draine vers le fichier sur le disque. qemu_log_unlock() fait un fflush() a
        # CHAQUE ligne (util/log.c) : avec exec,nochain c'est un write(2) par bloc
        # execute (~830 par trame), une dizaine de us sur le disque contre 1 a 2 us
        # sur un pipe. Le lecteur est lance AVANT QEMU : son fopen(fifo, "w") bloque
        # tant que personne ne lit.
        if [ "${ASSEMBLY_LOGS_FIFO:-0}" = 1 ]; then
            local asm_fifo="/dev/shm/qemu-asm.fifo"
            rm -f "$asm_fifo"; mkfifo "$asm_fifo"
            ${PRIO[@]+"${PRIO[@]}"} cat "$asm_fifo" > "$asm_cible" 2>/dev/null &
            echo $! > "$RUNDIR/asmcat.pid"
            asm_cible="$asm_fifo"
        fi
        gdb_opt+=(-d "${ASSEMBLY_LOGS_FLAGS:-in_asm}" -D "$asm_cible")
        [ -n "${ASSEMBLY_LOGS_FILTRE:-}" ] && gdb_opt+=(-dfilter "$ASSEMBLY_LOGS_FILTRE")
        # [2026-10-19] QEMU_LOG_BUFFERED (util/log.c de qosmo) : fichier -D pleinement
        # bufferise, plus de fflush() par ligne. 4 Mio par defaut quand on trace.
        export QEMU_LOG_BUFFERED="${QEMU_LOG_BUFFERED:-4194304}"
        # [2026-10-19] ANNEAU DES BLOCS EXECUTES, PAR DEFAUT AVEC ASSEMBLY_LOGS=1. Le plugin
        # TCG tbring (qosmo/tools/qosmo-tbring) pousse le PC de chaque bloc EXECUTE, blocs
        # chaines compris, dans un anneau mmap : 8 octets par bloc, pas de texte dans la
        # boucle, pas de nochain, l'ARM tourne comme en production (-d exec,nochain : 150 Mo/s
        # de texte, 20 Go en 130 s). Anneau de 64 Gio = 2^33 blocs, soit ~12 h d'ARM a
        # 180 000 blocs/s ; fichier CREUX sur le disque ($asm_dir, pas le tmpfs), il ne pese
        # que ce qui a ete ecrit. Relecture a chaud, a tout moment, sans arreter QEMU :
        #     tbring-dump -n 500 -f $asm_dir/qemu-tbring -e $FIRMWARE_ELF -a $asm_dir/qemu-asm.log
        # (la trace in_asm, un bloc par traduction, sert de table de desassemblage).
        # ASSEMBLY_LOGS_RING=0 coupe ; ASSEMBLY_LOGS_RING_ENTRIES (puissance de 2) et
        # ASSEMBLY_LOGS_RING_FILE changent la taille et l'emplacement.
        if [ "${ASSEMBLY_LOGS_RING:-1}" = 1 ]; then
            local tbring="${TBRING_SO:-$QOSMO/tools/qosmo-tbring/libtbring.so}"
            [ -f "$tbring" ] || tbring=/usr/local/lib/qosmo/libtbring.so
            if [ -f "$tbring" ]; then
                local ring="${ASSEMBLY_LOGS_RING_FILE:-$asm_dir/qemu-tbring}"
                rm -f "$ring" "$RUNDIR/qemu-tbring"
                [ "$(dirname "$ring")" != "$RUNDIR" ] && ln -s "$ring" "$RUNDIR/qemu-tbring"
                # anneau= et pas file= : file= est le mot-cle de QEMU pour le chemin du plugin
                gdb_opt+=(-plugin "$tbring,anneau=$ring,entries=${ASSEMBLY_LOGS_RING_ENTRIES:-8589934592}")
            else
                dire "   anneau tbring ignore : libtbring.so introuvable (make -C $QOSMO/tools/qosmo-tbring)"
            fi
        fi
    fi
    # [2026-10-19] QEMU_DEBUG_FLAGS=int,guest_errors : journal des exceptions de l'ARM (PC du
    # Data Abort, acces a une adresse non mappee) dans $RUNDIR/qemu-debug.log. Sans
    # ASSEMBLY_LOGS seulement (un seul -d/-D par ligne QEMU).
    if [ -n "${QEMU_DEBUG_FLAGS:-}" ] && [ "${ASSEMBLY_LOGS:-0}" != 1 ]; then
        gdb_opt+=(-d "$QEMU_DEBUG_FLAGS" -D "$RUNDIR/qemu-debug.log")
    fi
    # [2026-10-19] QEMU_ICOUNT=auto|shift=N,... (start-native.sh --icount) : -icount.
    case "${QEMU_ICOUNT:-}" in ''|0|off) ;; *) gdb_opt+=(-icount "$QEMU_ICOUNT") ;; esac
    # [2026-09-23] Relance de QEMU vers le DSP toutes les trame/64 (0,07 ms) au
    # lieu de trame/16 (0,29 ms) : la partie en serie d'une trame (DONE de la
    # phase A, GO) payait deux fois cette latence -- mesure [chrono] en TCH.
    CALYPSO_PONT_RETRY_DIV="${CALYPSO_PONT_RETRY_DIV:-64}" \
    # [2026-10-21] FIRMWARE_FLASH : la flash (copie de travail dans $RUNDIR, gardee entre deux
    # lancements pour le FFS ; FLASH_RESET=1 ou une image plus recente la remplacent) remplace -kernel
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
    # [2026-10-19] moniteur mailbox (CALYPSO_MAILBOX=1) : deux fichiers, un par cote
    if [ -n "${CALYPSO_MAILBOX:-}" ] && [ "${CALYPSO_MAILBOX}" != 0 ]; then
        local _mb="${CALYPSO_MAILBOX_FILE:-${LOG_DIR:-/tmp/calypso/logs}/mailbox.log}"
        dire "   mailbox ARM<->DSP : ${_mb} (cote DSP, c54x_exe) et ${_mb%.log}-arm.log (cote ARM, QEMU) ; CALYPSO_MAILBOX=$CALYPSO_MAILBOX"
    fi
    [ "${ASSEMBLY_LOGS:-0}" = 1 ] && dire "   trace asm ARM : $(readlink -f "$RUNDIR/qemu-asm.log") (-d ${ASSEMBLY_LOGS_FLAGS:-in_asm}${ASSEMBLY_LOGS_FILTRE:+, -dfilter $ASSEMBLY_LOGS_FILTRE}${ASSEMBLY_LOGS_FIFO:+, via FIFO}, QEMU_LOG_BUFFERED=$QEMU_LOG_BUFFERED)$([ "${ASSEMBLY_LOGS_RING:-1}" = 1 ] && echo " ; anneau des blocs executes : $(readlink -f "$RUNDIR/qemu-tbring" 2>/dev/null) (tbring-dump)")"
}

etape3_rvinterf() {   # FIRMWARE_FLASH : rvinterf (FreeCalypso) a la place d'osmocon
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

# [2026-10-15] BANC_VOCODEUR=rom : la ROM vocode (port voix modelise, CALYPSO_VSP=1).
# Le mobile doit alors etre en `io-handler l1phy` -- seul mode ou le firmware leve les
# mutes du vocodeur (prim_tch.c:101-111) -- et SANS io-tch-format ni ALSA dans le bloc
# tch-voice : sous l1phy le mobile refuse io-tch-format et abandonne TOUT le fichier.
# La variante est donc generee ici depuis MOBILE_CFG, jamais ecrite a la main.
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

etape4_trx() {   # OSMO_TRX=1 : l'appli hote transceiver, a la place du mobile
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
    # [2026-10-19] Colonne LOCALE de ss ($4 ; $5 est le peer), tous etats : avec « -uan | grep » la colonne peer matchait
    # aussi (osmo-bts-trx 5800 -> 127.0.0.1:5700 passait pour un tenant), et « -uln » ne voit pas
    # les sockets du transceiver, qui sont CONNECTEES (ESTAB) vers la BTS, pas en ecoute.
    for p in 5700 5701 5702; do
        ss -uanH 2>/dev/null | awk -v p="$p" '$4 ~ ("[:.]" p "$") {found=1} END {exit !found}' \
            && rater "port UDP $p deja pris par $(ss -uanpH 2>/dev/null | awk -v p="$p" '$4 ~ ("[:.]" p "$")' | grep -o 'users:.*' | head -1) (un pont.py qui traine ? ./run.sh --stop)"
    done
    vitrine transceiver
    # -l : socket L1CTL d'osmocon ; -m : masque de timeslots RX (un seul pour commencer) ; le masque TX
    # garde son defaut 0xe3 (OSMO_TRX_TX_MASK pour le forcer : -M), voir l'en-tete
    ${PRIO[@]+"${PRIO[@]}"} stdbuf -oL -eL "$TRANSCEIVER" -l "$L2_SOCK" -m "$OSMO_TRX_TS_MASK" ${OSMO_TRX_TX_MASK:+-M "$OSMO_TRX_TX_MASK"} ${OSMO_TRX_BSIC:+-b "$OSMO_TRX_BSIC"} $args_ts1 $OSMO_TRX_ARGS > "$RUNDIR/transceiver.log" 2>&1 &
    echo $! > "$RUNDIR/transceiver.pid"
    attendre 10 sh -c "ss -uanH 2>/dev/null | awk '\$4 ~ /[:.]5702\$/ {f=1} END {exit !f}'" || rater "transceiver n'ecoute pas en 5700-5702 (voir $RUNDIR/transceiver.log)"
    vivant transceiver || rater "transceiver s'est arrete : $(tail -1 "$RUNDIR/transceiver.log")"
    # [2026-10-20] osmo-bts-trx (BTS#0) SORT quand le transceiver disparait (« No more clock from
    # transceiver », bts_shutdown exit 1) : apres un --stop la BTS n'est plus la, et le plan du fork
    # (qosmo/run.sh --only bts) refuse de la relancer seule (dependance bsc). On la relance ici comme
    # le module 60-bts.sh, dans le journal du panneau. OSMO_TRX_BTS_RELANCE=0 pour ne rien faire.
    if [ "${OSMO_TRX_BTS_RELANCE:-1}" = 1 ]; then
        local bts_cfg="${BTS_CFG:-/etc/osmocom/osmo-bts-trx.cfg}" bts_bin bts_log bts_pid _i
        # La BTS d'avant peut etre encore en train de s'eteindre (bts_shutdown : jusqu'a 70 s mesures) :
        # on attend qu'elle soit partie, ou qu'elle ait allume le firmware (POWERON -> « BTS MODE: 7 514 »).
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

etape4() {   # le mobile
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
        # [2026-10-19] sockets et VTY propres a l'instance (layer2/sap/ms_data suffixes _N, VTY +10 par instance)
        local _src="$MOBILE_CFG" _dst="$RUNDIR/mobile_instance$INSTANCE.cfg"
        # le port VTY est decale de 100 par instance (4257 = osmo-ggsn ; awk additionne, sed non ; sub() garde
        # l'indentation : « $3 = ... » reconstruisait la ligne sans l'espace de tete et le VTY la refusait)
        awk -v inc=$((100 * (INSTANCE - 1))) '/^[[:space:]]*bind 127\.0\.0\.1 [0-9]+/ { n = $3 + inc; sub(/[0-9]+$/, n) } { print }' "$_src" \
            | sed -e "s#/tmp/osmocom_l2\b#$L2_SOCK#" -e "s#/tmp/osmocom_sap\b#/tmp/osmocom_sap_$INSTANCE#" -e "s#/tmp/ms_data\b#/tmp/ms_data_$INSTANCE#" > "$_dst"
        # [2026-10-21] « stick » suit l'ARFCN du plan radio (BTS#0) quand il a change (gabarit : 514)
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
    # [2026-09-23] Resynchro sur la cellule choisie : 8 essais au lieu de 1
    # (gsm322.c, sync_retries_selection). Le DSP ne passe le SB qu'une fois sur
    # trois a cinq, et deux echecs suffisaient a la boucle « no service ».
    # Montage grgsm : defaut du binaire, inchange.
    local retries=""
    [ "$MODE" = dsp ] && retries="${L23_SYNC_RETRIES_SELECTION:-8}"
    L23_SYNC_RETRIES_SELECTION="$retries" \
    ${PRIO[@]+"${PRIO[@]}"} stdbuf -oL "$MOBILE" -c "$MOBILE_CFG" > "$RUNDIR/mobile.log" 2>&1 &
    echo $! > "$RUNDIR/mobile.pid"
    sleep 2
    vivant mobile || rater "mobile s'est arrete : $(sed 's/\x1b\[[0-9;]*m//g' "$RUNDIR/mobile.log" | grep -iE 'cannot|error|unable' | tail -1)"
    dire "4. mobile  pid $(pid_de mobile)  config $MOBILE_CFG  vty telnet 127.0.0.1 ${vty:-?}"
}

etape5_air() {   # AIR=1 : l'air entre les deux Calypsos (pont/air.py), a la place du pont
    if vivant air; then dire "5. air deja lance (pid $(pid_de air))"; etape5_vsp; return; fi
    [ -r "$AIR_PY" ] || rater "air.py absent : $AIR_PY"
    local autre_shm="$HOTE_SHM" autre_port=6702
    [ "$INSTANCE" = 1 ] && { autre_shm=/dev/shm/calypso2; autre_port=6712; }
    vitrine air
    # A = l'autre instance (son anneau TX, son port BSP), B = celle-ci
    # [2026-10-20] le telephone 2 du TRX (TS1, instance 3 : /dev/shm/calypso3, BSP udp 6722) s'il tourne, et l'annonce du
    # TCH au mobile (calypso_tch_cfg de CE /dev/shm) quand l'air decode l'ASSIGNMENT COMMAND
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

# [2026-10-20] le port voix (CALYPSO_VSP=1) : anneaux PCM du DSP <-> PulseAudio, aussi en mode air (le son
# de l'appel passait par le pont ; sans lui, appel etabli mais muet). Lance dans l'espace de l'instance :
# /dev/shm/calypso_pcm_ul et _dl sont ceux de CE mobile.
etape5_vsp() {
    [ "${CALYPSO_VSP:-0}" = 1 ] || return 0
    vivant vsp && { dire "5b. pont audio VSP deja lance (pid $(pid_de vsp))"; return 0; }
    ( cd /opt/GSM/osmo-operator && exec python3 -m pont.vsp_audio ) > "$RUNDIR/vsp.log" 2>&1 &
    echo $! > "$RUNDIR/vsp.pid"
    dire "5b. pont audio VSP  pid $(pid_de vsp)  gsm_mic.monitor -> calypso_pcm_ul, calypso_pcm_dl -> gsm_audio  (journal $RUNDIR/vsp.log)"
}

etape5() {   # le pont TRX (PONT=1) : bursts du BTS vers la couche 1
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
    # [2026-10-04] UL_RAW=1 (defaut) : le pont emet les bursts montants finaux de la ROM (calypso_tx_rom,
    # src/tsp_tx.c) tels quels, a son numero de trame -- RACH, SDCCH, SACCH, FACCH, parole, A5 compris.
    # UL_RAW=0 : codage et A5 cote hote comme avant (PONT_UL_ROM reste le chemin xCCH de 2026-10-03).
    # [2026-10-19] Trace asm de l'ARM : le couple ARM+DSP peut tourner a 14 fois le
    # temps reel (exec,nochain). L'horloge asservie du pont (pont/trx.py) ne
    # descend que 4 fois sous le temps reel par defaut ; on lui permet 16 fois
    # (mesure : 10 a 14 fois ; au-dela de 16 la pause demandee a osmo-bts-trx
    # entre deux IND CLOCK approche sa limite de 50 trames, voir trx.py
    # run_clock). PONT_HORLOGE_MAX deja pose dans l'environnement : on le respecte.
    if [ "${ASSEMBLY_LOGS:-0}" = 1 ]; then export PONT_HORLOGE_MAX="${PONT_HORLOGE_MAX:-16}"; fi
    ( cd "$(dirname "$PONT_PY")/.." && bruit_env_pont && export PONT_UL_RAW="${UL_RAW:-1}" && exec ${PRIO[@]+"${PRIO[@]}"} python3 "$PONT_PY" $extra ) > "$RUNDIR/pont.log" 2>&1 &
    echo $! > "$RUNDIR/pont.pid"
    attendre 10 grep -aq "pont TRX : ports" "$RUNDIR/pont.log" || rater "pont.py ne s'est pas annonce (voir $RUNDIR/pont.log)"
    dire "5. pont.py  pid $(pid_de pont)  TRXD 5700-5702 <- BTS ; vers $([ "$MODE" = dsp ] && echo "le DSP (udp 6702)" || echo "la L1 gr-gsm (udp 4730/4731)")"
    [ -n "${PONT_HORLOGE_MAX:-}" ] && dire "   horloge asservie : ralentissement jusqu'a ${PONT_HORLOGE_MAX} x le temps reel (PONT_HORLOGE_MAX)"
    # [2026-10-15] port voix (CALYPSO_VSP=1) : anneaux PCM du DSP <-> PulseAudio
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
    # [2026-10-23] ARRET COMPLET D'UNE INSTANCE N != 1 (banc-rssi --stop) : les processus sans fichier pid ou dont
    # le pid a ete perdu (relance par-dessus une instance vivante) gardaient le port BSP (udp 6762 tenu par un
    # c54x_exe fantome, le nouveau ne recevait rien). On balaie ce qui porte les ressources propres a l'instance.
    if [ "$INSTANCE" != 1 ]; then
        # Pas « $RUNDIR/ » : un simple tail -F de ses journaux, ou le shell qui a lance --stop, y passaient.
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
    # [2026-09-22] Sockets du mobile : elles ne disparaissent pas avec lui. Le
    # « sap » etait deja oublie, et « ms_data » (tch-data, mobile_pont.cfg)
    # arrive avec la meme faiblesse : un fichier reste fait echouer le bind du
    # run suivant sur EADDRINUSE, et le mobile demarre sans sa voie donnees
    # sans le dire.
    # [2026-10-23] SEULEMENT pour l'instance 1 : ces sockets sont dans le /tmp COMMUN et appartiennent au mobile de
    # l'instance 1 ; banc-rssi --stop (instance 4) les effacait sous le mobile en marche (SAP, tch-data).
    [ "$INSTANCE" = 1 ] && rm -f /tmp/osmocom_sap /tmp/ms_data
    # [2026-09-22] Horloge du banc (calypso_bsp.c -> pont/trx.py) : laissee en
    # place, pont.py asservirait sa premiere seconde sur la trame d'une session
    # morte et la BTS resterait figee en attendant un DSP qui n'existe plus.
    rm -f /dev/shm/calypso_horloge
    # Side-bands du lien montant (src/montant.c) : sinon pont.py relit le
    # dernier enregistrement d'une session precedente au demarrage.
    rm -f /dev/shm/calypso_rach /dev/shm/calypso_sdcch_ul \
          /dev/shm/calypso_tch_facch_ul /dev/shm/calypso_tch_sacch_ul \
          /dev/shm/calypso_tch_ul
    # [2026-09-23] Annonce du TCH par le pont DSP (pont/dsp/tch.py -> montant.c
    # scruter_tch). Montage dsp seulement : en grgsm ce fichier est celui de la
    # L1 gr-gsm de QEMU, on n'y touche pas.
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

# [2026-09-23] GARDER LES JOURNAUX DU DSP ET DU PONT. dsp.log est ecrase a
# chaque lancement et pont.log est vide par vitrine ; aucun des deux n'est dans
# les archives du panneau (osmo-nitb/archives). Deux appels de suite ont ete
# perdus ainsi (13:11 et 13:21 : SP-CORRUPT et FACCH ko sans trace). On range
# la session precedente dans $RUNDIR/archives/<date de dsp.log>/ et on en garde
# JOURNAUX_GARDES (10). JOURNAUX_GARDES=0 ne garde rien.
JOURNAUX_GARDES="${JOURNAUX_GARDES:-10}"
garder_journaux() {
    [ "$JOURNAUX_GARDES" -gt 0 ] 2>/dev/null || return 0
    [ -s "$RUNDIR/dsp.log" ] || [ -s /dev/shm/pont.log ] || return 0
    local d="$RUNDIR/archives/$(date -r "$RUNDIR/dsp.log" +%Y%m%d-%H%M%S 2>/dev/null || date +%Y%m%d-%H%M%S)"
    [ -d "$d" ] && return 0                      # deja range (--stop puis relance)
    mkdir -p "$d" || return 0
    local n
    for n in dsp pont mobile qemu osmocon bruit; do
        [ -s "$RUNDIR/$n.log" ] && cp -L "$RUNDIR/$n.log" "$d/" 2>/dev/null
    done
    # etat du magasin dedie du BSP (stockes/joues/manques/replis/perdues)
    [ -s /dev/shm/calypso_bsp_dedie ] && cp /dev/shm/calypso_bsp_dedie "$d/bsp_dedie.txt" 2>/dev/null
    ls -1dt "$RUNDIR"/archives/*/ 2>/dev/null | tail -n +$((JOURNAUX_GARDES + 1)) | xargs -r rm -rf
    dire "journaux de la session precedente ranges dans $d"
}

# [2026-10-03] INJECTEUR DE BRUIT, INACTIF PAR DEFAUT (BRUIT_MODE et BRUIT_SNR_DB vides :
# rien ne change, aucune ligne de plus a l'ecran). Place ici, en fin de fichier, pour que
# les numeros de ligne cites par le wiki (Lancer-a-la-main.md...) restent justes ; les
# etapes ci-dessus n'appellent que bruit_env_dsp, bruit_dire_dsp, bruit_env_pont,
# bruit_arreter et bruit_statut, sans effet par defaut.
# Source unique : $QOSMO/tools/injecteur_bruit.py, repli sur tools/injecteur_bruit.py d'ici
# (un lien vers elle).
#   BRUIT_MODE     ber | souple | iq (EXPERIMENTAL) | relais : etape b (--step b), entre le
#                  DSP et QEMU dans la sequence, entre le pont et la L1 dans le flux.
#     dsp   : c54x_exe ecoute BRUIT_PORT_DSP (16702, CALYPSO_BSP_PORT), l'injecteur prend
#             127.0.0.1:6702 a sa place ; le pont ne change pas (--dsp-port 6702). Le BSP
#             apprend son pair montant sur l'emetteur du descendant : son montant passe par
#             l'injecteur, qui le rend tel quel au pont (5702) depuis 6702.
#     grgsm : l'injecteur ecoute BRUIT_PORT_GRGSM (14730) et relaie vers QEMU 4730 ; le pont
#             y envoie (PONT_GSMTAP_PORT, pose a l'etape 5). ber | relais seulement : QEMU
#             recoit des blocs L2 deja decodes.
#     iq    : refuse si CALYPSO_BSP_STREAM=1 (defaut de start-native.sh --dsp).
#   BRUIT_BER (0..1), BRUIT_RAFALES (L bits), BRUIT_SNR_DB (dB), BRUIT_PERTE (0..1),
#   BRUIT_TN (0 ou 1,2...), BRUIT_GRAINE, BRUIT_STATS (10 s), BRUIT_OPTS (options en plus),
#   BRUIT_PORT_DSP, BRUIT_PORT_GRGSM, BRUIT_PY.
#   BRUIT_SNR_DB SANS BRUIT_MODE (dsp) : pas d'injecteur, AWGN du coeur C54x -- c54x_exe
#   recoit CALYPSO_BSP_SNR_DB (et CALYPSO_BSP_BRUIT_GRAINE <- BRUIT_GRAINE), calypso_c54x.c.
# Une configuration refusee (--verifier) arrete run.sh AVANT tout lancement.
# Journal : $RUNDIR/bruit.log. Detail : /opt/GSM/osmo-operator/wiki/Injecteur-bruit.md.
BRUIT_MODE="${BRUIT_MODE:-}"
BRUIT_SNR_DB="${BRUIT_SNR_DB:-}"
BRUIT_PORT_DSP="${BRUIT_PORT_DSP:-16702}"
BRUIT_PORT_GRGSM="${BRUIT_PORT_GRGSM:-14730}"
if [ -z "${BRUIT_PY:-}" ]; then
    BRUIT_PY="$QOSMO/tools/injecteur_bruit.py"
    [ -r "$BRUIT_PY" ] || BRUIT_PY="$HERE/tools/injecteur_bruit.py"
fi
BRUIT_ACTIF=0      # 1 : l'injecteur est intercale pour ce lancement (bruit_preparer)
BRUIT_ARGS=()

bruit_args() {   # remplit BRUIT_ARGS
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

bruit_preparer() {   # avant l'etape 1 : decide BRUIT_ACTIF, verifie la configuration
    [ -n "$BRUIT_MODE" ] || return 0
    if vivant bruit; then BRUIT_ACTIF=1; return 0; fi
    if [ "$MODE" = dsp ] && vivant dsp; then
        # Un c54x_exe deja lance n'est intercalable que s'il ecoute deja le port deplace.
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

bruit_env_dsp() {   # dans le sous-shell de c54x_exe ; ne pose rien par defaut
    [ "$BRUIT_ACTIF" = 1 ] && export CALYPSO_BSP_PORT="$BRUIT_PORT_DSP"
    if [ -z "$BRUIT_MODE" ] && [ -n "$BRUIT_SNR_DB" ]; then
        export CALYPSO_BSP_SNR_DB="${CALYPSO_BSP_SNR_DB:-$BRUIT_SNR_DB}"
        [ -n "${BRUIT_GRAINE:-}" ] && export CALYPSO_BSP_BRUIT_GRAINE="${CALYPSO_BSP_BRUIT_GRAINE:-$BRUIT_GRAINE}"
    fi
    return 0
}

bruit_dire_dsp() {   # apres l'etape 1 ; muet par defaut
    [ "$BRUIT_ACTIF" = 1 ] && dire "   BSP deplace sur udp/$BRUIT_PORT_DSP (CALYPSO_BSP_PORT) : 6702 est a l'injecteur de bruit"
    [ -z "$BRUIT_MODE" ] && [ -n "$BRUIT_SNR_DB" ] && dire "   bruit du coeur : CALYPSO_BSP_SNR_DB=${CALYPSO_BSP_SNR_DB:-$BRUIT_SNR_DB} dB${BRUIT_GRAINE:+, graine $BRUIT_GRAINE}"
    return 0
}

bruit_env_pont() {   # dans le sous-shell du pont ; ne pose rien par defaut
    [ "$MODE" = grgsm ] && [ -n "$BRUIT_MODE" ] || return 0
    # pont/pont.py viserait de lui-meme BRUIT_PORT_GRGSM : on le dit explicitement, et
    # 4730 (QEMU) si l'injecteur ne tourne pas, pour ne pas envoyer dans le vide.
    if vivant bruit; then export PONT_GSMTAP_PORT="$BRUIT_PORT_GRGSM"; else export PONT_GSMTAP_PORT=4730; fi
    return 0
}

etapeb() {   # l'injecteur de bruit (BRUIT_MODE), entre le pont et la couche 1
    [ "$BRUIT_ACTIF" = 1 ] || return 0
    vivant bruit && { dire "b. injecteur de bruit deja lance (pid $(pid_de bruit))"; return; }
    bruit_args
    : > "$RUNDIR/bruit.log"
    python3 -u "$BRUIT_PY" "${BRUIT_ARGS[@]}" >> "$RUNDIR/bruit.log" 2>&1 &
    echo $! > "$RUNDIR/bruit.pid"
    attendre 5 grep -aq "PRET ecoute=" "$RUNDIR/bruit.log" || rater "l'injecteur de bruit ne s'est pas annonce (voir $RUNDIR/bruit.log)"
    dire "b. injecteur de bruit  pid $(pid_de bruit)  mode $BRUIT_MODE  $(sed -n 's/.*PRET \(ecoute=[^ ]*\) \(vers=[^ ]*\).*/\1 \2/p' "$RUNDIR/bruit.log" | head -1)  journal $RUNDIR/bruit.log"
}

bruit_arreter() {   # --stop : l'injecteur sans pid (lance a la main avec --etiquette c54x)
    pkill -f -- "injecteur_bruit\.py .*--etiquette c54x" 2>/dev/null
    return 0
}

bruit_statut() {   # --status : muet par defaut
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
              # [2026-10-20] le telephone 2 du TRX (instance 3) tombe avec l'instance 1
              if [ "$INSTANCE" = 1 ] && [ -d "/tmp/c54x-pont$OSMO_TRX_TS1_INSTANCE" ] && [ -f "/tmp/c54x-pont$OSMO_TRX_TS1_INSTANCE/dsp.pid" ]; then
                  INSTANCE="$OSMO_TRX_TS1_INSTANCE" OSMO_TRX=1 OSMO_TRX_PHONE=2 AIR=0 bash "$0" --stop
              fi
              # [2026-10-20 22:30] le mobile par l'air (instance 2) aussi : il survivait aux relances de
              # start-native.sh --bts --ms (c54x_exe + air.py de 22:12 encore la a 22:28), et l'air gardait
              # ouvert l'ANCIEN anneau TX de l'instance 1 (fichier supprime) : A->B fige, plus de descendant
              # pour le mobile, donc plus de LU.
              if [ "$INSTANCE" = 1 ] && [ -d /tmp/c54x-pont2 ] && ls /tmp/c54x-pont2/*.pid >/dev/null 2>&1; then
                  INSTANCE=2 AIR=1 OSMO_TRX=0 bash "$0" --stop
              fi
              # vides une fois ranges : le lancement suivant ne les range pas deux fois
              [ -f "$RUNDIR/dsp.log" ] && : > "$RUNDIR/dsp.log"
              [ -f /dev/shm/pont.log ] && : > /dev/shm/pont.log ;;
    --status) statut ;;
    --charge) charge ;;
    --logs)   # [2026-10-19] en couleur a l'affichage (qosmo/tools/qosmo-couleurs.py, QOSMO_COULEURS=0 coupe)
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
