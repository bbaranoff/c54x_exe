#!/usr/bin/env bash
# sign.sh - empreinte SHA-256 et signature GPG detachee d'un fichier (par defaut proof_toa_23.md).
#
#   ./sign.sh                      signe proof_toa_23.md
#   ./sign.sh FICHIER              signe FICHIER
#   ./sign.sh --verify [FICHIER]   verifie l'empreinte et la signature
#
# Cle : si aucune cle secrete GPG ne correspond a SIGN_UID, une cle ed25519 (signature seule, sans
# expiration) est generee. Variables : SIGN_UID (identite), SIGN_PASSPHRASE (vide par defaut),
# GNUPGHOME (trousseau, defaut ~/.gnupg). Produits, a cote du fichier :
#   FICHIER.sha256     empreinte (format sha256sum)
#   FICHIER.asc        signature detachee ASCII
#   FICHIER.pubkey.asc cle publique a diffuser avec le document
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

# 1. empreinte
(cd "$D" && sha256sum "$B" > "$B.sha256")
cat "$F.sha256"

# 2. cle (generee une seule fois)
if ! gpg --batch --list-secret-keys "=$SIGN_UID" >/dev/null 2>&1; then
    echo "aucune cle secrete pour « $SIGN_UID » : generation d'une cle ed25519 (signature, sans expiration)"
    "${GPG[@]}" --quick-gen-key "$SIGN_UID" ed25519 sign 0
fi
FPR="$(gpg --batch --with-colons --list-secret-keys "=$SIGN_UID" | awk -F: '$1=="fpr"{print $10; exit}')"
echo "cle : $FPR  ($SIGN_UID)"

# 3. signature detachee + cle publique
"${GPG[@]}" --armor --detach-sign --local-user "$FPR" --output "$F.asc" "$F"
gpg --batch --armor --export "$FPR" > "$F.pubkey.asc"
echo "signature : $F.asc"
echo "cle publique : $F.pubkey.asc"
echo "verification : $0 --verify $F"
