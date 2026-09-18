#!/usr/bin/env bash
# run_si.sh — amener le mobile (pile DSP c54x_exe) jusqu'à décoder la BCCH du
# VRAI réseau et lire un System Information, PUIS vérifier que le LAI décodé
# correspond à la config du réseau. Sinon = FAUX POSITIF, dit tel quel.
#
# C'est un banc SOUS ÉCHAFAUDAGE (« canning »), PAS un fonctionnement natif :
#   - PONT_CAN_TOA=23      TOA FB figée (le corrélateur natif ne verrouille pas)
#   - PONT_CAN_SB=<bsic>   résultat SB fabriqué (BSIC + FN du BTS), comme qemu-src
#   - CALYPSO_TWL3025_AFC_HZ  rotation AFC forcée (annule l'offset de fréquence)
#   - CALYPSO_PONT_LOCKSTEP=1  trame QEMU cadencée sur le DSP
# Chaque hack actif est imprimé. Un SI lu avec des hacks n'est pas une preuve du
# DSP natif ; c'est un test d'intégration de l'aval (sync -> BCCH -> L2/L3).
#
#   ./run_si.sh              # chaîne réelle (BTS + pont), échafaudage complet
#   ./run_si.sh --secondes N # durée d'observation (défaut 90)
#   ./run_si.sh --stop       # tout arrêter
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
RUNDIR=/tmp/c54x-pont
SECS=90
BSIC="$(grep -m1 -oE 'base_station_id_code [0-9]+' /etc/osmocom/osmo-bsc.cfg 2>/dev/null | grep -oE '[0-9]+$')"; BSIC="${BSIC:-7}"
# LAI attendu, lu depuis la config (MCC/MNC/LAC)
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

# chaîne réelle + échafaudage complet
export MODE=dsp PONT=1 IQ=none
export CALYPSO_RHEA_DMA_XFER=1 CALYPSO_BSP_DIRECT_FEED=1 CALYPSO_PONT_LOCKSTEP=1
export CALYPSO_TWL3025_AFC_HZ=1927
export PONT_CAN_TOA=23 PONT_CAN_SB="$BSIC"
echo "== hacks actifs : CAN_TOA=23, CAN_SB=$BSIC, AFC_HZ=1927, LOCKSTEP, DIRECT_FEED, RHEA_DMA"
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
