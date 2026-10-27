#!/usr/bin/env bash
# sign.sh - SHA-256 digest and detached GPG signature of a file (default proof_toa_23.md).
#
#   ./sign.sh                      sign proof_toa_23.md
#   ./sign.sh FILE                 sign FILE
#   ./sign.sh --verify [FILE]      verify the digest and the signature
#
# Key: if no GPG secret key matches SIGN_UID, an ed25519 key (signing only, no expiration)
# is generated. Variables: SIGN_UID (identity), SIGN_PASSPHRASE (empty by default),
# GNUPGHOME (keyring, default ~/.gnupg). Outputs, next to the file:
#   FILE.sha256     digest (sha256sum format)
#   FILE.asc        ASCII detached signature
#   FILE.pubkey.asc public key to distribute with the document
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIGN_UID="${SIGN_UID:-proof_toa_23 <bastienbaranoff@gmail.com>}"
SIGN_PASSPHRASE="${SIGN_PASSPHRASE:-}"
export GNUPGHOME="${GNUPGHOME:-$HOME/.gnupg}"
mkdir -p "$GNUPGHOME"; chmod 700 "$GNUPGHOME"
GPG=(gpg --batch --yes --pinentry-mode loopback --passphrase "$SIGN_PASSPHRASE")

verify=0
[ "${1:-}" = "--verify" ] && { verify=1; shift; }
F="${1:-$HERE/proof_toa_23.md}"
[ -r "$F" ] || { echo "fichier illisible : $F" >&2; exit 2; }
D="$(cd "$(dirname "$F")" && pwd)"; B="$(basename "$F")"

if [ "$verify" = 1 ]; then
    (cd "$D" && sha256sum -c "$B.sha256")
    [ -r "$F.pubkey.asc" ] && gpg --batch --quiet --import "$F.pubkey.asc" 2>/dev/null || true
    gpg --batch --verify "$F.asc" "$F"
    exit $?
fi

# 1. digest
(cd "$D" && sha256sum "$B" > "$B.sha256")
cat "$F.sha256"

# 2. key (generated once)
if ! gpg --batch --list-secret-keys "=$SIGN_UID" >/dev/null 2>&1; then
    echo "aucune cle secrete pour « $SIGN_UID » : generation d'une cle ed25519 (signature, sans expiration)"
    "${GPG[@]}" --quick-gen-key "$SIGN_UID" ed25519 sign 0
fi
FPR="$(gpg --batch --with-colons --list-secret-keys "=$SIGN_UID" | awk -F: '$1=="fpr"{print $10; exit}')"
echo "cle : $FPR  ($SIGN_UID)"

# 3. detached signature + public key
"${GPG[@]}" --armor --detach-sign --local-user "$FPR" --output "$F.asc" "$F"
gpg --batch --armor --export "$FPR" > "$F.pubkey.asc"
echo "signature : $F.asc"
echo "cle publique : $F.pubkey.asc"
echo "verification : $0 --verify $F"
