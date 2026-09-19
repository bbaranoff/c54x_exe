#!/usr/bin/env bash
# balayage.sh - explore les parametres du rejeu dont on n'est PAS sur, et classe
# les combinaisons par le seul critere qui compte : le nombre de SB VRAIES
# (CRC OK ET BSIC = celui injecte ET T3 valide ET FN = la trame).
#
# [2026-09-18] NE PAS CLASSER SUR crc_ok. Le CRC de la SCH fait 10 bits : une
# entree aleatoire passerait ~0,1 % du temps, or on mesurait 20 %. Un tel taux
# ne vient pas du hasard mais d'une entree DEGENERee (quasi constante) sur
# laquelle le Viterbi converge toujours vers le meme mot. Classer sur crc_ok
# recompense donc le figement : c'est ainsi qu'une phase de 0,75 est sortie en
# tete alors qu'elle produisait 12 CRC OK pour UN SEUL mot distinct. Le critere
# secondaire est le nombre de mots DISTINCTS decodes : lui mesure si la sortie
# depend de l'entree.
#
# Pourquoi : plusieurs boutons du banc sont des hypotheses non verifiees --
# la phase d'echantillonnage (0,5 = centre du symbole), la marge de tete, le
# calage de la commande SB (le DSP demodule le burst de la DERNIERE trame de
# commande, mesure du 18/09), le nombre de commandes, l'ordre RX, la longueur
# de fenetre DMA. Les regler un par un fait rater les interactions.
#
#   ./balayage.sh              balayage complet, resultats dans balayage.tsv
#   ./balayage.sh --rapide     grille reduite
#   TRAMES=1200 ./balayage.sh  plus de trames par essai (defaut 600)
#   sort -t$'\t' -k6,6nr -k5,5nr balayage.tsv | head   relire le classement
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
