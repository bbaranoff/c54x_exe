#!/bin/bash
# fetch-rom.sh [--dest DIR]... [--version 3606|3311] [--force]
#
# Recupere la ROM du DSP TI Calypso chez FreeCalypso et la convertit en
# calypso_dsp.{DROM,PDROM,PROM0..3}.bin, ce que lisent c54x_exe (--rom-dir) et
# QEMU. Le depot ne contient PAS la ROM : elle est telechargee a la demande, et
# la conversion (tools/dsp_txt2bin.py) est verifiee contre SHA256SUMS.<version>.
#
#   --dest DIR   ou ecrire les .bin (repetable ; defaut : ce dossier rom/)
#   --version V  3606 (defaut, celle de ce depot) ou 3311 (D-Sample ; pas de somme connue)
#   --force      retelecharger meme si les .bin sont deja la et corrects
#
# Registers n'est pas de la ROM : calypso_dsp.Registers.bin (instantane de
# registres, etat de reset du coeur) reste dans le depot et est copie tel quel
# vers chaque --dest.
set -u
ICI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION=3606; FORCE=0; DESTS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --dest)    DESTS+=("${2:?}"); shift ;;
        --version) VERSION="${2:?}"; shift ;;
        --force)   FORCE=1 ;;
        -h|--help) sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "option inconnue : $1" >&2; exit 2 ;;
    esac
    shift
done
[ ${#DESTS[@]} -gt 0 ] || DESTS=("$ICI")
URL="${FREECALYPSO_URL:-ftp://ftp.freecalypso.org/pub/GSM/Calypso}/dsp-rom-${VERSION}-dump.txt"
SOMMES="$ICI/SHA256SUMS.$VERSION"
ROMS="DROM PDROM PROM0 PROM1 PROM2 PROM3"

# Deja en place et correct dans TOUTES les destinations ? Rien a faire.
bon() {   # $1 = dossier
    [ -f "$SOMMES" ] || return 1
    local r; for r in $ROMS; do [ -f "$1/calypso_dsp.$r.bin" ] || return 1; done
    ( cd "$1" && sha256sum -c --quiet "$SOMMES" ) >/dev/null 2>&1
}
if [ "$FORCE" = 0 ]; then
    tout=1; for d in "${DESTS[@]}"; do bon "$d" || tout=0; done
    [ "$tout" = 1 ] && { echo "ROM DSP $VERSION deja en place et verifiee"; exit 0; }
fi

TMP="$(mktemp -d)"; mkdir -p "$TMP/bin"; trap 'rm -rf "$TMP"' EXIT
echo "telechargement : $URL"
if   command -v curl >/dev/null 2>&1; then curl -fsS --retry 2 -o "$TMP/dump.txt" "$URL"
elif command -v wget >/dev/null 2>&1; then wget -q -O "$TMP/dump.txt" "$URL"
else echo "ni curl ni wget" >&2; exit 1; fi || { echo "telechargement impossible : $URL" >&2; exit 1; }
# tools/dsp_txt2bin.py : une sortie par section (calypso_dsp.<SECTION>.bin) ; seules les six
# sections de ROM sont gardees plus bas, pas « Registers » (instantane de registres).
python3 "$ICI/../tools/dsp_txt2bin.py" "$TMP/dump.txt" "$TMP/bin/calypso_dsp.bin" >/dev/null \
    || { echo "conversion du dump impossible" >&2; exit 1; }
if [ -f "$SOMMES" ]; then
    ( cd "$TMP/bin" && sha256sum -c --quiet "$SOMMES" ) \
        || { echo "ROM $VERSION : somme de controle differente de SHA256SUMS.$VERSION - abandon" >&2; exit 1; }
    echo "somme de controle verifiee (SHA256SUMS.$VERSION)"
else
    echo "attention : pas de SHA256SUMS.$VERSION, conversion non verifiee" >&2
fi
for d in "${DESTS[@]}"; do
    mkdir -p "$d"
    for r in $ROMS; do install -m644 "$TMP/bin/calypso_dsp.$r.bin" "$d/calypso_dsp.$r.bin"; done
    [ -f "$ICI/calypso_dsp.Registers.bin" ] && [ "$d" != "$ICI" ] && install -m644 "$ICI/calypso_dsp.Registers.bin" "$d/"
    echo "ROM DSP $VERSION ecrite dans $d"
done
