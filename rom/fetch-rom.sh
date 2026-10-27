#!/bin/bash
# fetch-rom.sh [--dest DIR]... [--version 3606|3311] [--force]
#
# Fetches the TI Calypso DSP ROM from FreeCalypso and converts it into
# calypso_dsp.{DROM,PDROM,PROM0..3}.bin, which c54x_exe (--rom-dir) and QEMU
# read. The repository does NOT contain the ROM: it is downloaded on demand,
# and the conversion (tools/dsp_txt2bin.py) is checked against SHA256SUMS.<version>.
#
#   --dest DIR   where to write the .bin files (repeatable; default: this rom/ dir)
#   --version V  3606 (default, the one of this repository) or 3311 (D-Sample)
#   --force      download again even if the .bin files are present and correct
#
# Registers is not ROM: calypso_dsp.Registers.bin (register snapshot, reset
# state of the core) stays in the repository and is copied as is to each
# --dest.
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
URL="${FREECALYPSO_URL:-https://raw.githubusercontent.com/bbaranoff/fc-magnetite/main/freecalypso-env}/dsp-rom-${VERSION}-dump.txt"
SOMMES="$ICI/SHA256SUMS.$VERSION"
ROMS="DROM PDROM PROM0 PROM1 PROM2 PROM3"

# Already in place and correct in ALL destinations? Nothing to do.
bon() {   # $1 = directory
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
# tools/dsp_txt2bin.py: one output per section (calypso_dsp.<SECTION>.bin); only the six
# ROM sections are kept below, not "Registers" (register snapshot).
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
