#!/usr/bin/env bash
# run_si.sh - bring the mobile (c54x_exe DSP stack) up to decoding the BCCH of
# the REAL network and reading a System Information, THEN check that the
# decoded LAI matches the network config. Otherwise = FALSE POSITIVE, said so.
#
# The chain runs NATIVE: no result scaffolding (no CAN_TOA / CAN_SB / forced
# AFC / RHEA_DMA); the ROM's FB correlator, AFC and DMA do the work. Only the
# pacing and burst feeding are set, and they fabricate no result:
#   - CALYPSO_BSP_DIRECT_FEED=1  bursts delivered straight to the DSP buffers
#   - CALYPSO_PONT_LOCKSTEP=1    QEMU frame paced on the DSP
# A SI read here is an integration test of the whole chain
# (sync -> BCCH -> L2/L3) against the network's own identity.
#
#   ./run_si.sh              # real chain (BTS + bridge)
#   ./run_si.sh --secondes N # observation time (default 90)
#   ./run_si.sh --stop       # stop everything
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNDIR=/tmp/c54x-pont
SECS=90
BSIC="$(grep -m1 -oE 'base_station_id_code [0-9]+' /etc/osmocom/osmo-bsc.cfg 2>/dev/null | grep -oE '[0-9]+$')"; BSIC="${BSIC:-7}"
# expected LAI, read from the config (MCC/MNC/LAC)
E_MCC="$(grep -m1 -oE 'network country code [0-9]+' /etc/osmocom/osmo-msc.cfg | grep -oE '[0-9]+$')"
E_MNC="$(grep -m1 -oE 'mobile network code [0-9]+'  /etc/osmocom/osmo-msc.cfg | grep -oE '[0-9]+$')"
E_LAC="$(grep -m1 -oE 'location_area_code 0x[0-9a-fA-F]+' /etc/osmocom/osmo-bsc.cfg | grep -oE '0x[0-9a-fA-F]+')"
E_LAC=$((E_LAC)); E_MCC=$((10#$E_MCC)); E_MNC=$((10#$E_MNC))

case "${1:-}" in
  --stop) "$HERE/run.sh" --stop; systemctl stop osmo-bts-trx 2>/dev/null; exit 0 ;;
  --secondes) SECS="$2" ;;
  -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
esac

echo "== run_si : cible LAI attendu MCC=$E_MCC MNC=$E_MNC LAC=$E_LAC (BSIC=$BSIC, ARFCN 514)"
"$HERE/run.sh" --stop >/dev/null 2>&1; sleep 1
systemctl start osmo-bts-trx 2>/dev/null; sleep 3
: > "$RUNDIR/mobile.log" 2>/dev/null; : > "$RUNDIR/osmocon.log" 2>/dev/null

# real chain, NATIVE: no result scaffolding (CAN_TOA/CAN_SB/AFC_HZ/RHEA_DMA);
# the ROM's FB correlator, AFC and DMA do the work. Only the pacing and the
# burst feeding are set, and they fabricate no result.
export MODE=dsp PONT=1 IQ=none
export CALYPSO_BSP_DIRECT_FEED=1 CALYPSO_PONT_LOCKSTEP=1   # pacing / feeding (not cheats)
echo "== NATIF : aucun echafaudage (CAN_TOA/CAN_SB/AFC_HZ/RHEA_DMA retires) ; LOCKSTEP + DIRECT_FEED"
"$HERE/run.sh" >/dev/null 2>&1 &
sleep 12
grep -aq "code is running" "$RUNDIR/osmocon.log" && echo "== firmware lancé" || echo "== firmware PAS lancé (voir $RUNDIR/osmocon.log)"
echo "== observation ${SECS}s ..."
sleep "$SECS"

M="$RUNDIR/mobile.log"
echo "== System Information vus (BCCH) :"
sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -aoE "New SYSTEM INFORMATION [0-9a-z]+" | sort | uniq -c || echo "  (aucun)"
LAI="$(sed 's/\x1b\[[0-9;]*m//g' "$M" | grep -aoE "lai=[0-9]+-[0-9]+-[0-9]+" | tail -1)"
echo "== [can-sb] premières lignes :"; grep -a "\[can-sb\]" "$RUNDIR/dsp.log" 2>/dev/null | head -3 | cut -c1-120

echo "== VERDICT :"
if [ -z "$LAI" ]; then
  echo "  ÉCHEC : aucun SI lu -> pas de camp. (attendu tant que le démod NB/BCCH natif n'est pas bon)"
else
  D_MCC="$(echo "$LAI" | cut -d= -f2 | cut -d- -f1)"
  D_MNC="$(echo "$LAI" | cut -d- -f2)"
  D_LAC="$(echo "$LAI" | cut -d- -f3)"
  echo "  LAI décodé : MCC=$D_MCC MNC=$D_MNC LAC=$D_LAC"
  if [ "$D_MCC" = "$E_MCC" ] && [ "$D_MNC" = "$E_MNC" ] && [ "$D_LAC" = "$E_LAC" ]; then
    echo "  ✓ VRAI : le LAI correspond à la config -> le mobile a réellement lu la BCCH du réseau."
  else
    echo "  ✗ FAUX POSITIF : le LAI ne correspond pas (config MCC=$E_MCC MNC=$E_MNC LAC=$E_LAC)."
    echo "    Un bloc décodé qui ne porte pas l'identité du réseau n'est pas un camp, c'est du bruit qui a passé un CRC."
  fi
fi
echo "== arrêt : ./run_si.sh --stop"
