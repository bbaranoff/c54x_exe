#!/usr/bin/env bash
# balayage.sh - sweep the replay parameters that are not known for sure, and
# rank the combinations by the only criterion that matters: the number of TRUE
# SBs (CRC OK AND BSIC = the injected one AND valid T3 AND FN = the frame).
#
# Do not rank on crc_ok. The SCH CRC is 10 bits: random input would pass
# ~0.1 % of the time, while a degenerate (near-constant) input makes the
# Viterbi converge to the same word every time and passes far more often.
# Ranking on crc_ok would therefore reward a frozen output. The secondary
# criterion is the number of DISTINCT decoded words: it measures whether the
# output depends on the input.
#
# Several bench knobs are unverified hypotheses: the sampling phase (0.5 =
# symbol centre), the head margin, the alignment of the SB command (the DSP
# demodulates the burst of the LAST command frame), the number of commands,
# the RX order, the DMA window length. Tuning them one at a time misses the
# interactions, hence a full grid.
#
#   ./balayage.sh              full sweep, results in balayage.tsv
#   ./balayage.sh --rapide     reduced grid
#   TRAMES=1200 ./balayage.sh  more frames per trial (default 600)
#   sort -t$'\t' -k6,6nr -k5,5nr balayage.tsv | head   re-read the ranking
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXE="$HERE/c54x_exe"
OUT="${OUT:-$HERE/balayage.tsv}"
TRAMES="${TRAMES:-600}"
[ -x "$EXE" ] || { echo "binaire absent : $EXE" >&2; exit 1; }

if [ "${1:-}" = "--rapide" ]; then
    DECALAGES="-1 0";           UNIQUES="0 1"
    PHASES="0.0 0.25 0.5 0.75"; MARGES="0 21 41"
    RXAVANT="0";                LENS="380"
else
    DECALAGES="-5 -4 -3 -2 -1 0 1";   UNIQUES="0 1"
    PHASES="0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9"
    MARGES="0 10 21 30 41";     RXAVANT="0 1"
    LENS="296 380"
fi

total=0
for a in $DECALAGES; do for b in $UNIQUES; do for c in $PHASES; do
for d in $MARGES; do for e in $RXAVANT; do for f in $LENS; do
    total=$((total+1)); done; done; done; done; done; done

printf 'decalage\tunique\tphase\tmarge\trx_avant\tdaram_len\tvraies\tmots_distincts\tcrc_ok\tsb_tentees\n' > "$OUT"
echo "[balayage] $total combinaisons, $TRAMES trames chacune -> $OUT"
n=0; t0=$(date +%s)
for a in $DECALAGES; do
 for b in $UNIQUES; do
  for c in $PHASES; do
   for d in $MARGES; do
    for e in $RXAVANT; do
     for f in $LENS; do
      n=$((n+1))
      res=$(REJEU_SB_FORCE=1 \
            REJEU_SB_DECALAGE="$a" REJEU_SB_UNIQUE="$b" \
            REJEU_DECALAGE_SYMB="$c" REJEU_MARGE="$d" \
            REJEU_RX_AVANT="$e" CALYPSO_BSP_DARAM_LEN="$f" \
            timeout 180 "$EXE" --rejouer --verbeux --trames "$TRAMES" 2>/dev/null)
      vraies=$(printf '%s' "$res" | sed -n 's/.*VRAIES ([^)]*): \([0-9]*\).*/\1/p' | head -1)
      crc=$(   printf '%s' "$res" | sed -n 's/.*CRC OK: \([0-9]*\).*/\1/p' | head -1)
      tent=$(  printf '%s' "$res" | sed -n 's#.*SB tentees / CRC KO: \([0-9]*\) /.*#\1#p' | head -1)
      mots=$(  printf '%s' "$res" | grep -oE '^  SB[0-9] fn=[0-9]+ : sb=0x[0-9a-f]+' | grep -oE '0x[0-9a-f]+' | sort -u | wc -l)
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
             "$a" "$b" "$c" "$d" "$e" "$f" "${vraies:-0}" "${mots:-0}" "${crc:-0}" "${tent:-0}" >> "$OUT"
      if [ $((n % 25)) -eq 0 ]; then
          dt=$(( $(date +%s) - t0 ))
          echo "[balayage] $n/$total  ${dt}s  meilleur vraies=$(tail -n +2 "$OUT" | cut -f7 | sort -nr | head -1) mots_distincts=$(tail -n +2 "$OUT" | cut -f8 | sort -nr | head -1)"
      fi
     done; done; done; done; done; done
echo "[balayage] termine, $n essais."
echo "--- 15 meilleures combinaisons (vraies, puis mots distincts) ---"
{ head -1 "$OUT"; tail -n +2 "$OUT" | sort -t$'\t' -k7,7nr -k8,8nr | head -15; } | column -t -s$'\t'
