#!/usr/bin/env bash
# run_real.sh - the REAL bench: virtual BTS (osmo-bts-trx + osmocom core) ->
# pont.py (TRX) -> DSP c54x_exe (--iq none, UDP 6702 bursts) -> QEMU/ARM ->
# osmocon -> mobile. Same executables as run.sh, without any crutch
# (no CAN_TOA / CAN_SB / forced AFC): native FB, SB and BCCH.
#
#   ./run_real.sh              start everything, observe 120 s, SI / LAI verdict
#   ./run_real.sh --secondes N observation time
#   ./run_real.sh --logs       follow the logs         ./run_real.sh --stop  stop everything
#
# Settings: INSNS=60000 (DSP pace close to the BTS real time),
# CALYPSO_BSP_STREAM=1 (one TS0 burst per frame, FN order),
# CALYPSO_RHEA_DMA_XFER=1 CALYPSO_BSP_DIRECT_FEED=1 (DMA/RIF delivery),
# CALYPSO_BSP_NB_SYM=0.3 (normal burst widening, calypso_bsp.c).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNDIR=/tmp/c54x-pont
SECS=120
case "${1:-}" in
  --stop)  "$HERE/run.sh" --stop; systemctl stop osmo-bts-trx 2>/dev/null; exit 0 ;;
  --logs)  exec "$HERE/run.sh" --logs ;;
  --secondes) SECS="$2" ;;
  -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
esac
E_MCC="$(grep -m1 -oE 'network country code [0-9]+' /etc/osmocom/osmo-msc.cfg | grep -oE '[0-9]+$')"
E_MNC="$(grep -m1 -oE 'mobile network code [0-9]+'  /etc/osmocom/osmo-msc.cfg | grep -oE '[0-9]+$')"
E_LAC="$(grep -m1 -oE 'location_area_code 0x[0-9a-fA-F]+' /etc/osmocom/osmo-bsc.cfg | grep -oE '0x[0-9a-fA-F]+')"
E_LAC=$((E_LAC)); E_MCC=$((10#$E_MCC)); E_MNC=$((10#$E_MNC))
BSIC="$(grep -m1 -oE 'base_station_id_code [0-9]+' /etc/osmocom/osmo-bsc.cfg | grep -oE '[0-9]+$')"
echo "== run_real: expected LAI MCC=$E_MCC MNC=$E_MNC LAC=$E_LAC (BSIC=${BSIC:-?}), no crutch"
"$HERE/run.sh" --stop >/dev/null 2>&1; sleep 1
for u in osmo-hlr osmo-stp osmo-msc osmo-mgw osmo-bsc; do systemctl is-active --quiet "$u" || systemctl start "$u"; done
systemctl restart osmo-bts-trx; sleep 3
mkdir -p "$RUNDIR"; : > "$RUNDIR/mobile.log"; : > "$RUNDIR/osmocon.log"; : > "$RUNDIR/pont.log"
export MODE=dsp PONT=1 IQ=none INSNS="${INSNS:-60000}" LOCKSTEP="${LOCKSTEP:-1}"
export CALYPSO_RHEA_DMA_XFER=1 CALYPSO_BSP_DIRECT_FEED=1 CALYPSO_BSP_STREAM=1
export PONT_NB_DEBUG="${PONT_NB_DEBUG:-1}"
"$HERE/run.sh" || { echo "== run.sh failed"; exit 1; }
echo "== observation ${SECS}s ..."
sleep "$SECS"
M="$RUNDIR/mobile.log"; O="$RUNDIR/osmocon.log"
echo "== SB decoded (osmocon): $(sed 's/\x1b\[[0-9;]*m//g' "$O" | grep -ac '=> SB')   FBSB without SB: $(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -ac 'result=255')"
echo "== BCCH blocks dropped (fire): $(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -ac 'Dropping frame')"
echo "== System Information seen:"
sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -aoE "New SYSTEM INFORMATION [0-9a-z]+" | sort | uniq -c || echo "  (none)"
echo "== uplink: RACH published by the DSP: $(sed 's/\x1b\[[0-9;]*m//g' "$RUNDIR/dsp.log" | grep -ac '\[montant\] RACH')   bridge counters: $(grep -aoE 'UL bursts=[0-9]* (tard|late)=[0-9]* rach=[0-9]*' "$RUNDIR/pont.log" | tail -1)"
echo "== IMM ASS: $(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -ac 'IMMEDIATE ASSIGNMENT')   LU ACCEPT: $(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -ac 'LOCATION UPDATING ACCEPT')"
LAI="$(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -aoE "lai=[0-9]+-[0-9]+-[0-9]+" | tail -1)"
echo "== VERDICT:"
if [ -z "$LAI" ]; then echo "  FAILED: no SI read"; else
  D_MCC="$(echo "$LAI" | cut -d= -f2 | cut -d- -f1)"; D_MNC="$(echo "$LAI" | cut -d- -f2)"; D_LAC="$(echo "$LAI" | cut -d- -f3)"
  if [ "$((10#$D_MCC))" = "$E_MCC" ] && [ "$((10#$D_MNC))" = "$E_MNC" ] && [ "$D_LAC" = "$E_LAC" ]; then
    echo "  TRUE: $LAI = the network config, the mobile read the BTS BCCH"
  else echo "  FALSE POSITIVE: $LAI does not match MCC=$E_MCC MNC=$E_MNC LAC=$E_LAC"; fi
  sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -aq "camping normally" && echo "  and it camps (MM IDLE)"
fi
echo "== stop: ./run_real.sh --stop"
