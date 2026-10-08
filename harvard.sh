#!/bin/sh
# harvard.sh - shell « Harvard » pour le DSP TMS320C54x du Calypso (sh POSIX, dash suffit).
#
# Le C54x est une machine Harvard : l'espace P (programme, les opcodes) et l'espace D (donnees,
# registres, API RAM) sont separes. Ce shell garde les deux de front :
#   - cote P : on tape une instruction (mnemonique assemble par gas tic54x, ou des mots hex),
#              on desassemble la ROM TI (PROM0 a 0x7000, PDROM a 0xE000, pages 1..3 via XPC) ;
#   - cote D : un etat (registres + mots de memoire donnees) qu'on lit, qu'on pose, et que
#              chaque instruction executee fait avancer ; la DROM (0x9000) et la PDROM se lisent.
# L'execution d'une instruction passe par ./isa_test (le vrai coeur C54x de qosmo, pcb-minimal) :
# on lui donne l'etat « avant » et les mots, il rend l'etat « apres ». Rien n'est simule ici.
# Les commandes « run » lancent les outils du banc tels quels (c54x_exe, dsp_tester, isa_test, run.sh).
#
#   ./harvard.sh                menu interactif
#   ./harvard.sh repl           invite de commandes directe
#   ./harvard.sh pas 'LD #0x1234, A'    une commande puis sortie (toutes celles du REPL)
#   ./harvard.sh aide
# Variables : HARVARD_DIR (etat, defaut ~/.local/state/harvard-c54x), ROM_DIR (defaut ./rom),
#             ISA_TEST, OBJDUMP, GAS (chemins des outils).

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=${HARVARD_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/harvard-c54x}
ROM_DIR=${ROM_DIR:-$HERE/rom}
ISA_TEST=${ISA_TEST:-$HERE/isa_test}
OBJDUMP=${OBJDUMP:-$HERE/build-tic54x/binutils/objdump}
GAS=${GAS:-$HERE/build-tic54x/gas/as-new}
ETAT=$DIR/etat.txt          # une ligne par element : "REG hex" | "M addr hex" | "P addr hex"
JOURNAL=$DIR/journal.txt
LOG=$DIR/coeur.log          # stderr du coeur (avertissements [c54x] ...)
REGS="A B T SP PC ST0 ST1 PMST AR0 AR1 AR2 AR3 AR4 AR5 AR6 AR7 BK BRC RSA REA TRN IMR IFR XPC"
SENT_REG=deadbeef1          # sentinelle 36 bits : jamais egale a un registre -> isa_test affiche « obtenu »
SENT_MEM=1ffff              # sentinelle 17 bits : jamais egale a un mot de 16 bits
mkdir -p "$DIR"; : >> "$ETAT"; : >> "$JOURNAL"

# ---------------------------------------------------------------- utilitaires
rouge() { if [ -t 1 ]; then printf '\033[31m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
vert()  { if [ -t 1 ]; then printf '\033[32m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
gras()  { if [ -t 1 ]; then printf '\033[1m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
hex() { # hex <texte> : accepte 0x1234, 1234, 1:8000 (page:adresse) ; rend l'entier
    case $1 in
        *:*) printf '%d' $(( ( ${1%%:*} << 16 ) | 0x${1#*:} )) ;;
        0x*|0X*) printf '%d' $(( $1 )) ;;
        *) printf '%d' $(( 0x$1 )) ;;
    esac 2>/dev/null
}
est_hex_mots() { # vrai si tous les arguments sont des mots hex de 1 a 4 chiffres
    for m in "$@"; do case $m in ''|*[!0-9a-fA-F]*|?????*) return 1;; esac; done; return 0
}
verifier_outils() {
    [ -x "$ISA_TEST" ] || rouge "isa_test absent ($ISA_TEST) : make isa_test"
    [ -x "$OBJDUMP" ]  || rouge "objdump tic54x absent ($OBJDUMP) : pas de desassemblage"
    [ -x "$GAS" ]      || rouge "gas tic54x absent ($GAS) : seuls les mots hex sont acceptes"
}

# ---------------------------------------------------------------- etat (cote D)
etat_reg() { awk -v r="$1" '$1==r {v=$2} END {print v}' "$ETAT"; }
etat_fixe() { # etat_fixe <cle...> <val> : remplace ou ajoute la ligne "<cles> <val>"
    cle=$1; shift
    [ $# -gt 1 ] && { cle="$cle $1"; shift; }
    { grep -v "^$cle " "$ETAT"; echo "$cle $1"; } > "$ETAT.new" && mv "$ETAT.new" "$ETAT"
}
etat_mem() { awk -v a="$1" '$1=="M" && $2==a {v=$3} END {print v}' "$ETAT"; }
drapeaux() { # drapeaux <st0> <st1> : noms des bits a 1
    st0=$(( 0x${1:-0} )); st1=$(( 0x${2:-0} )); d=""
    [ $(( st0 >> 12 & 1 )) = 1 ] && d="$d TC";  [ $(( st0 >> 11 & 1 )) = 1 ] && d="$d C"
    [ $(( st0 >> 10 & 1 )) = 1 ] && d="$d OVA"; [ $(( st0 >> 9 & 1 )) = 1 ]  && d="$d OVB"
    [ $(( st1 >> 15 & 1 )) = 1 ] && d="$d BRAF"; [ $(( st1 >> 14 & 1 )) = 1 ] && d="$d CPL"
    [ $(( st1 >> 13 & 1 )) = 1 ] && d="$d XF";  [ $(( st1 >> 12 & 1 )) = 1 ] && d="$d HM"
    [ $(( st1 >> 11 & 1 )) = 1 ] && d="$d INTM"; [ $(( st1 >> 9 & 1 )) = 1 ] && d="$d OVM"
    [ $(( st1 >> 8 & 1 )) = 1 ]  && d="$d SXM"; [ $(( st1 >> 7 & 1 )) = 1 ]  && d="$d C16"
    [ $(( st1 >> 6 & 1 )) = 1 ]  && d="$d FRCT"; [ $(( st1 >> 5 & 1 )) = 1 ] && d="$d CMPT"
    printf 'ARP=%d DP=%03x ASM=%02x flags:%s' $(( st0 >> 13 )) $(( st0 & 0x1ff )) $(( st1 & 0x1f )) "${d:- (aucun)}"
}
montrer_regs() {
    [ -s "$ETAT" ] || { echo "(etat vide : les registres ont les valeurs d'init du coeur ; faites un pas)"; return; }
    printf '  A  =%12s   B  =%12s   T  =%6s   PC =%6s   SP =%6s   XPC=%4s\n' \
        "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(etat_reg PC)" "$(etat_reg SP)" "$(etat_reg XPC)"
    printf '  AR0=%6s AR1=%6s AR2=%6s AR3=%6s AR4=%6s AR5=%6s AR6=%6s AR7=%6s\n' \
        "$(etat_reg AR0)" "$(etat_reg AR1)" "$(etat_reg AR2)" "$(etat_reg AR3)" "$(etat_reg AR4)" "$(etat_reg AR5)" "$(etat_reg AR6)" "$(etat_reg AR7)"
    printf '  ST0=%6s ST1=%6s PMST=%5s BK=%6s BRC=%6s RSA=%6s REA=%6s TRN=%6s IMR=%5s IFR=%5s\n' \
        "$(etat_reg ST0)" "$(etat_reg ST1)" "$(etat_reg PMST)" "$(etat_reg BK)" "$(etat_reg BRC)" "$(etat_reg RSA)" "$(etat_reg REA)" "$(etat_reg TRN)" "$(etat_reg IMR)" "$(etat_reg IFR)"
    printf '  %s\n' "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}
montrer_mem() {
    n=$(grep -c '^M ' "$ETAT"); [ "$n" = 0 ] && { echo "  D : aucun mot pose"; return; }
    echo "  D (mots poses / lus, $n) :"; awk '$1=="M" {printf "   [%04x]=%04x", strtonum("0x"$2), strtonum("0x"$3); if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT" 2>/dev/null \
      || awk '$1=="M" {printf "   [%s]=%s", $2, $3; if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT"
    n=$(grep -c '^P ' "$ETAT"); [ "$n" != 0 ] && { echo "  P (mots poses, $n) :"; awk '$1=="P" {printf "   [%s]=%s", $2, $3; if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT"; }
}
cmd_etat() { gras "--- registres (cote D)"; montrer_regs; montrer_mem; }
cmd_raz() { : > "$ETAT"; echo "etat remis a zero (valeurs d'init du coeur au prochain pas)"; }
cmd_reg() {
    [ $# -lt 2 ] && { montrer_regs; return; }
    r=$(printf '%s' "$1" | tr a-z A-Z); v=$(hex "$2") || { rouge "valeur invalide"; return 1; }
    etat_fixe "$r" "$(printf '%x' "$v")"; echo "$r <- $(printf '%x' "$v")"
}

# ---------------------------------------------------------------- assemblage / desassemblage (cote P)
asm() { # asm <instruction> -> mots hex sur stdout, 1 si echec
    if est_hex_mots $1; then printf '%s\n' "$*" | tr A-F a-f; return 0; fi
    [ -x "$GAS" ] || { rouge "pas d'assembleur : donnez les mots hex"; return 1; }
    printf '\t.text\n\t%s\n' "$*" > "$DIR/a.s"
    if ! "$GAS" -o "$DIR/a.o" "$DIR/a.s" 2> "$DIR/a.err"; then
        rouge "assemblage refuse :"; sed 's/^/    /' "$DIR/a.err"; return 1
    fi
    "$OBJDUMP" -d "$DIR/a.o" | sed -n 's/^ *[0-9a-f]*:[[:space:]]*\([0-9a-f]\{4\}\).*/\1/p' | tr '\n' ' ' | sed 's/ $/\n/'
}
dis_mots() { # dis_mots <adresse> <mots hex...> : desassemble des mots
    a=$1; shift; : > "$DIR/d.bin"
    for m in "$@"; do v=$(( 0x$m )); printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" >> "$DIR/d.bin"; done
    "$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma="0x${a:-9000}" "$DIR/d.bin" | sed -n 's/^ *\([0-9a-f]*:.*\)/    \1/p'
}
rom_fichier() { # rom_fichier <adr lineaire P ou D> <P|D> -> "fichier base_mots"
    a=$1
    if [ "$2" = P ]; then
        page=$(( a >> 16 )); off=$(( a & 0xffff ))
        case $page in
            0) if [ $off -ge $(( 0xe000 )) ]; then echo "$ROM_DIR/calypso_dsp.PDROM.bin $(( 0xe000 ))"
               elif [ $off -ge $(( 0x7000 )) ]; then echo "$ROM_DIR/calypso_dsp.PROM0.bin $(( 0x7000 ))"; else return 1; fi ;;
            1|2|3) [ $off -ge $(( 0x8000 )) ] || return 1; echo "$ROM_DIR/calypso_dsp.PROM$page.bin $(( (page << 16) | 0x8000 ))" ;;
            *) return 1 ;;
        esac
    else
        if [ $a -ge $(( 0xe000 )) ]; then echo "$ROM_DIR/calypso_dsp.PDROM.bin $(( 0xe000 ))"
        elif [ $a -ge $(( 0x9000 )) ]; then echo "$ROM_DIR/calypso_dsp.DROM.bin $(( 0x9000 ))"; else return 1; fi
    fi
}
cmd_p() { # p <adresse> [n mots] : desassembler la ROM programme
    [ $# -ge 1 ] || { echo "usage : p <adresse> [n]   (0x7000.., 0xe000.., 1:8000 = page 1)"; return 1; }
    a=$(hex "$1") || { rouge "adresse invalide"; return 1; }; n=${2:-16}
    set -- $(rom_fichier "$a" P) || { rouge "$(printf '%x' "$a") : hors ROM programme (PROM0 7000-dfff, PDROM e000-ffff, pages 1-3 : 8000-ffff)"; return 1; }
    f=$1; base=$2; [ -r "$f" ] || { rouge "ROM absente : $f (rom/fetch-rom.sh)"; return 1; }
    page=$(( a >> 16 )); off=$(( a & 0xffff )); vma=$(( base & 0xffff ))
    [ $page -gt 0 ] && gras "  page $page (XPC=$page), fichier $(basename "$f")"
    "$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma=$vma --start-address=$off --stop-address=$(( off + n )) "$f" \
        | sed -n 's/^ *\([0-9a-f]*:.*\)/    \1/p'
}
cmd_drom() { # drom <adresse> [n] : lire la ROM donnees
    [ $# -ge 1 ] || { echo "usage : drom <adresse> [n]   (DROM 9000-dfff, PDROM e000-ffff)"; return 1; }
    a=$(hex "$1") || { rouge "adresse invalide"; return 1; }; n=${2:-16}
    set -- $(rom_fichier "$a" D) || { rouge "$(printf '%x' "$a") : hors ROM donnees"; return 1; }
    f=$1; base=$2; [ -r "$f" ] || { rouge "ROM absente : $f"; return 1; }
    od -An -v -tx2 -w16 -j $(( (a - base) * 2 )) -N $(( n * 2 )) "$f" | awk -v a=$a '{printf "    [%04x]", a; print; a+=8}'
}
cmd_asm() { m=$(asm "$@") || return 1; printf '    mots : %s\n' "$m"; dis_mots "$(etat_reg PC)" $m; }
cmd_dis() { est_hex_mots "$@" || { echo "usage : dis <mots hex>"; return 1; }; dis_mots "$(etat_reg PC)" "$@"; }

# ---------------------------------------------------------------- execution d'une instruction (P -> D)
surveiller() { # surveiller <mot1> [mot2] : adresses D a lire a chaque pas (celles qu'une instruction isolee peut toucher)
    w1=$(( 0x$1 )); st0=$(etat_reg ST0); sp=$(etat_reg SP)
    {
        for rg in AR0 AR1 AR2 AR3 AR4 AR5 AR6 AR7; do v=$(etat_reg $rg); [ -n "$v" ] || continue
            v=$(( 0x$v )); printf '%04x\n%04x\n%04x\n' $(( (v - 1) & 0xffff )) $v $(( (v + 1) & 0xffff )); done
        [ -n "$sp" ] && { v=$(( 0x$sp )); printf '%04x\n%04x\n%04x\n' $(( (v - 1) & 0xffff )) $v $(( (v + 1) & 0xffff )); }
        printf '%04x\n' $(( ((0x${st0:-0} & 0x1ff) << 7) | (w1 & 0x7f) ))      # adressage direct DP:dma
        [ $# -ge 2 ] && printf '%04x\n' $(( 0x$2 & 0xffff ))                     # adresse longue lk
    } | sort -u > "$DIR/cand.txt"; awk '$1=="M" {print $2}' "$ETAT" > "$DIR/connu.txt"; grep -v -F -x -f "$DIR/connu.txt" "$DIR/cand.txt" 2>/dev/null || cat "$DIR/cand.txt"
}
executer() { # executer <garder 0|1> <titre> <mots hex...> [adresses D a lire...]  ->  $DIR/pas.apres : lignes "NOM valeur"
    garder=$1; titre=$2; mots=$3; shift 3
    t=$DIR/pas.txt; nm=$(grep -c '^M ' "$ETAT"); place=$(( 28 - nm )); [ $place -lt 0 ] && place=0
    {
        echo "T 0 $titre"; echo "W $mots"
        sed -n 's/^\([A-Z][A-Z0-9]*\) \([0-9a-fA-F]*\)$/B \1 \2/p; s/^\([MP]\) \([0-9a-fA-F]*\) \([0-9a-fA-F]*\)$/B \1 \2 \3/p' "$ETAT"
        for rg in $REGS; do echo "A $rg $SENT_REG"; done
        awk '$1=="M" {print "A M " $2 " '"$SENT_MEM"'"}' "$ETAT"
        { for adr in "$@"; do echo "$adr"; done; [ $# -eq 0 ] && surveiller $mots; } | head -n $place | sed "s/^/A M /; s/\$/ $SENT_MEM/"
        echo "E"
    } > "$t"
    "$ISA_TEST" "$t" 2>> "$LOG" > "$DIR/pas.out"
    grep -q '^FAIL' "$DIR/pas.out" || { rouge "isa_test n'a rien rendu :"; cat "$DIR/pas.out"; return 1; }
    grep -q '\[[0-9a-f ]*?\]' "$DIR/pas.out" && rouge "  (un registre de l'etat est inconnu d'isa_test, ignore)"
    # "  A: attendu deadbeef1, obtenu 12  data[0100]: attendu 1ffff, obtenu 0000" -> "A 12" / "M 0100 0000"
    sed -n 's/^FAIL[^]]*\]//p' "$DIR/pas.out" | sed 's/  /\n/g' | sed -n \
        's/^data\[\([0-9a-f]*\)\]: attendu [0-9a-f]*, obtenu \([0-9a-f]*\)$/M \1 \2/p;
         s/^prog\[\([0-9a-f]*\)\]: attendu [0-9a-f]*, obtenu \([0-9a-f]*\)$/P \1 \2/p;
         s/^\([A-Z][A-Z0-9]*\): attendu [0-9a-f]*, obtenu \([0-9a-f]*\)$/\1 \2/p' > "$DIR/pas.apres"
    if [ "$garder" = 1 ]; then
        # on garde : les registres, les mots P poses, les mots D deja dans l'etat, et tout mot D devenu non nul
        { grep '^P ' "$ETAT"
          awk 'FILENAME==ARGV[1] { if ($1=="M") connu[$2]=1; next }
               $1=="P" { next } $1!="M" { print; next } ($2 in connu) || ($3 != "0000" && $2 >= "0060") { print }' "$ETAT" "$DIR/pas.apres"
        } > "$ETAT.new" && mv "$ETAT.new" "$ETAT"
        printf '%s  %s  | %s\n' "$(date +%H:%M:%S)" "$mots" "$titre" >> "$JOURNAL"
    fi
}
diff_etat() { # diff_etat <avant> <apres> : ce qui a change
    awk 'FILENAME==ARGV[1] { if ($1=="M"||$1=="P") av[$1" "$2]=$3; else av[$1]=$2; next }
         { if ($1=="M"||$1=="P") { k=$1" "$2; v=$3 } else { k=$1; v=$2 }
           if (!(k in av)) { if (k !~ /^M |^P /) nouveaux=nouveaux" "k"="v; else if (v != "0000") printf "    %-10s %12s -> %s\n", k, "(nouveau)", v }
           else if (av[k] != v) printf "    %-10s %12s -> %s\n", k, av[k], v }
         END { if (nouveaux != "") print "    (premiere lecture :" nouveaux ")" }' "$1" "$2"
}
cmd_pas() { # pas <instruction | mots hex>
    [ $# -ge 1 ] || { echo "usage : pas <instruction>   ex : pas LD #0x1234, A   |   pas f020 1234"; return 1; }
    mots=$(asm "$@") || return 1
    pc=$(etat_reg PC); printf '  P[%s] %s   <- %s\n' "${pc:-9000}" "$mots" "$*"
    cp "$ETAT" "$DIR/avant.txt"
    executer 1 "$*" "$mots" || return 1
    gras "  changements :"; diff_etat "$DIR/avant.txt" "$ETAT" | grep . || echo "    (aucun registre ni mot pose n'a change)"
    printf '  PC=%s  A=%s  B=%s  T=%s  %s\n' "$(etat_reg PC)" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}
cmd_essai() { # essai <instruction> : executer sans garder l'etat
    [ $# -ge 1 ] || { echo "usage : essai <instruction>"; return 1; }
    mots=$(asm "$@") || return 1
    executer 0 "$*" "$mots" || return 1
    gras "  resultat (etat NON garde) :"; diff_etat "$ETAT" "$DIR/pas.apres" | grep . || echo "    (rien ne change)"
}
cmd_d() { # d <adresse> [n] : lire la memoire donnees du coeur (via un NOP non garde)
    [ $# -ge 1 ] || { echo "usage : d <adresse> [n<=24]"; return 1; }
    a=$(hex "$1") || { rouge "adresse invalide"; return 1; }; n=${2:-8}; [ $n -gt 24 ] && n=24
    liste=""; i=0; while [ $i -lt $n ]; do liste="$liste $(printf '%04x' $(( a + i )))"; i=$(( i + 1 )); done
    executer 0 "lecture D" f495 $liste || return 1
    awk -v a=$a 'BEGIN{k=0} $1=="M" { lu[$2]=$3 }
         END { for (i=0;i<'"$n"';i++) { k=sprintf("%04x", a+i); printf "    [%s]=%s", k, (k in lu)?lu[k]:"????"; if ((i+1)%8==0) print "" } if (i%8) print "" }' "$DIR/pas.apres"
    st0=$(etat_reg ST0); echo "    (lecture par le coeur avec l'etat courant ; DP=$(printf '%03x' $(( 0x${st0:-0} & 0x1ff ))) ; les mots non poses valent 0 hors registres mappes 0000-005f)"
}
cmd_d_pose() { # d! <adresse> <mots...> : poser des mots en D
    [ $# -ge 2 ] || { echo "usage : d! <adresse> <mot> [mot...]"; return 1; }
    a=$(hex "$1") || { rouge "adresse invalide"; return 1; }; shift
    for m in "$@"; do v=$(hex "$m") || { rouge "mot invalide : $m"; return 1; }
        etat_fixe M "$(printf '%04x' $a)" "$(printf '%04x' $(( v & 0xffff )))"; printf '    D[%04x] <- %04x\n' $a $(( v & 0xffff )); a=$(( a + 1 )); done
}
cmd_p_pose() { # p! <adresse> <instruction | mots> : poser des mots en P (precharges a chaque pas)
    [ $# -ge 2 ] || { echo "usage : p! <adresse> <instruction|mots hex>   (ex. une sous-routine appelee par CALL)"; return 1; }
    a=$(hex "$1") || { rouge "adresse invalide"; return 1; }; shift
    mots=$(asm "$@") || return 1
    for m in $mots; do etat_fixe P "$(printf '%04x' $a)" "$m"; printf '    P[%04x] <- %s\n' $a "$m"; a=$(( a + 1 )); done
}
cmd_journal() { gras "--- journal des pas"; tail -n ${1:-30} "$JOURNAL"; }
cmd_log() { gras "--- avertissements du coeur (dernieres lignes)"; tail -n ${1:-20} "$LOG"; }
cmd_sauve() { [ $# -ge 1 ] || { echo "usage : sauve <fichier>"; return 1; }; cp "$ETAT" "$1" && echo "etat -> $1"; }
cmd_charge() { [ $# -ge 1 ] && [ -r "$1" ] || { echo "usage : charge <fichier>"; return 1; }; cp "$1" "$ETAT" && echo "etat <- $1" && montrer_regs; }

# ---------------------------------------------------------------- cours / arret : courir dans la ROM (etat du shell)
# « arret » pose des points d'arret (adresses P lineaires, 1:8000 = page 1) ; « cours » part du PC courant
# (ff80 = vecteur de reset si l'etat est vide), lit chaque instruction dans la ROM (ou dans les mots P
# poses, qui ont priorite), l'execute par isa_test comme « pas », et s'arrete sur : un point d'arret,
# IDLE, une adresse hors ROM (RAM sans mot pose), un PC qui ne bouge pas, ou le budget de pas.
# Tout ceci agit sur l'ETAT DU SHELL, jamais sur le c54x_exe en marche (pour lui : stop / go / dsp).
# Limites (le coeur repart d'un etat « registres + mots » a chaque pas) : le creux d'un branchement
# retarde (BD, CALLD, RETD...) et le compteur d'un RPT ne survivent pas d'un pas a l'autre.
ARRETS=$DIR/arrets.txt; : >> "$ARRETS"
cmd_arret() { # arret [adr...] | arret - : poser / lister / effacer les points d'arret
    if [ $# -eq 0 ]; then
        [ -s "$ARRETS" ] && { gras "--- points d'arret (adresses P)"; sed 's/^/    /' "$ARRETS"; } || echo "  aucun point d'arret (arret <adr> [adr...])"; return
    fi
    [ "$1" = - ] && { : > "$ARRETS"; echo "  points d'arret effaces"; return; }
    for s in "$@"; do a=$(hex "$s") || { rouge "adresse invalide : $s"; return 1; }
        a=$(printf '%x:%04x' $(( a >> 16 )) $(( a & 0xffff ))); grep -q -x -F "$a" "$ARRETS" || echo "$a" >> "$ARRETS"; echo "    arret a $a"; done
}
rom_insn() { # rom_insn <lineaire P> -> "mots hex|mnemonique" ; priorite aux mots P poses ; 1 si hors ROM
    a=$1; page=$(( a >> 16 )); off=$(( a & 0xffff )); : > "$DIR/i.bin"
    i=0; while [ $i -lt 3 ]; do
        lin=$(( (page << 16) | ((off + i) & 0xffff) )); ad=$(printf '%04x' $(( lin & 0xffff )))
        m=$(awk -v a="$ad" '$1=="P" && $2==a {v=$3} END {print v}' "$ETAT")
        if [ -z "$m" ]; then
            if fb=$(rom_fichier $lin P); then set -- $fb; m=$(od -An -v -tx2 -j $(( (lin - $2) * 2 )) -N 2 "$1" | tr -d ' ')
            elif [ $i -eq 0 ]; then return 1; else m=f495; fi
        fi
        v=$(( 0x$m )); printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" >> "$DIR/i.bin"; i=$(( i + 1 ))
    done
    "$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma="0x$(printf '%04x' $off)" "$DIR/i.bin" | sed -n 's/^ *[0-9a-f]*:[[:space:]]*//p' \
      | awk 'NR==1 { w=$1; $1=""; sub(/^ +/,""); m=$0; next } NF>1 { exit } { w=w" "$1 } END { print w "|" m }'
}
cmd_cours() { # cours [N pas] : courir depuis le PC courant jusqu'a un arret
    n=${1:-200}; case $n in ''|*[!0-9]*) echo "usage : cours [nombre de pas max]"; return 1;; esac
    pc=$(etat_reg PC); xpc=$(etat_reg XPC)
    [ -n "$pc" ] || { etat_fixe PC ff80; etat_fixe XPC 0; pc=ff80; xpc=0; echo "  (etat vide : depart au vecteur de reset ff80)"; }
    gras "  cours depuis $(printf '%x:%04x' $(( 0x${xpc:-0} )) $(( 0x$pc )))  (budget $n pas ; arrets : $(tr '\n' ' ' < "$ARRETS"))"
    : > "$DIR/cours_adr.txt"; k=0; motif=""
    while :; do
        pc=$(etat_reg PC); xpc=$(etat_reg XPC); lin=$(( (0x${xpc:-0} << 16) | 0x$pc )); cle=$(printf '%x:%04x' $(( lin >> 16 )) $(( lin & 0xffff )))
        if [ $k -gt 0 ] && grep -q -x -F "$cle" "$ARRETS"; then motif="point d'arret $cle"; break; fi
        [ $k -lt $n ] || { motif="budget de $n pas epuise"; break; }
        r=$(rom_insn $lin) || { motif="$cle hors ROM programme (RAM sans mot pose)"; break; }
        mots=${r%%|*}; mn=${r#*|}
        printf '%s\n' "$cle" >> "$DIR/cours_adr.txt"
        case $mn in idle*) printf '    %-7s %-14s %s\n' "$cle" "$mots" "$mn"; motif="IDLE a $cle"; k=$(( k + 1 )); break ;; esac
        executer 1 "cours $cle $mn" "$mots" || { motif="isa_test en echec a $cle"; break; }
        k=$(( k + 1 )); pc2=$(etat_reg PC); xpc2=$(etat_reg XPC)
        printf '    %-7s %-14s %-28s -> %x:%04x  A=%s B=%s\n' "$cle" "$mots" "$mn" $(( 0x${xpc2:-0} )) $(( 0x${pc2:-0} )) "$(etat_reg A)" "$(etat_reg B)"
        [ "$pc2" = "$pc" ] && [ "${xpc2:-0}" = "${xpc:-0}" ] && { motif="PC immobile a $cle (boucle sur place / instruction non executee)"; break; }
    done
    gras "  arret : $motif  ($k pas)"
    gras "  adresses parcourues ($(sort -u "$DIR/cours_adr.txt" | wc -l) distinctes, dans l'ordre) :"
    awk '!(seen[$0]++)' "$DIR/cours_adr.txt" | tr '\n' ' ' | fold -s -w 96 | sed 's/^/    /'; echo
    printf '  PC=%s  A=%s  B=%s  T=%s  %s\n' "$(etat_reg PC)" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}

# ---------------------------------------------------------------- DSP EN MARCHE : stop / go / dsp sur le c54x_exe --arm
# Ici on agit sur le PROCESSUS VIVANT (celui que run.sh a lance), jamais sur l'etat du shell :
#   stop   pause du DSP. Binaire recent (src/pont.c SIGUSR1) : pause cooperative a la frontiere de trame
#          suivante (DONE rendu, DSP en IDLE, l'ARM attend son DONE suivant) ; binaire ancien : SIGSTOP.
#          Dans les deux cas rien n'est ecrit dans l'API RAM ni dans le coeur : le firmware ne voit qu'une
#          trame plus longue. Les registres sont lus dans la memoire du processus (/proc/PID/mem, decalages
#          de C54xState donnes par gdb sur l'executable), et on affiche LES ADRESSES : PC (ou le DSP s'est
#          arrete, IDLE) et les vecteurs armes (ou il repartira : IPTR de PMST, IT trame vec 28 = bit 12).
#   go     reprise (SIGUSR2 ou SIGCONT), puis un instantane 0,3 s plus tard pour voir qu'il a repris.
#   dsp    instantane sans arreter (lecture pendant qu'il tourne : coherente a une instruction pres).
PAUSE_FICHIER=${PAUSE_FICHIER:-/dev/shm/calypso_dsp_pause}
VIVANT=$DIR/vivant.txt
IRQ_NOMS="12=IT_trame_TPU(vec28) 14=DMA_INT10n(vec30)"   # c54x_irq.c:44-60 ; les autres bits : non nommes ici
vivant_pid() { # le c54x_exe --arm : comm == c54x_exe (lisible par tous, meme si le process est a root) + cmdline --arm
    for p in $(pgrep -x c54x_exe 2>/dev/null); do
        [ "$(cat /proc/$p/comm 2>/dev/null)" = c54x_exe ] || continue
        grep -qa -- --arm /proc/$p/cmdline 2>/dev/null && { echo $p; return 0; }
    done; return 1
}
vivant_accessible() { # vivant_accessible <pid> : ce compte peut-il piloter ce process (signal + /proc/mem) ?
    kill -0 "$1" 2>/dev/null && [ -r "/proc/$1/mem" ]
}
vivant_diag() { # message clair quand le banc tourne sous un autre compte
    pid=$1; u=$(stat -c %U /proc/$pid 2>/dev/null); moi=$(id -un)
    rouge "le banc c54x_exe (pid $pid) tourne sous « ${u:-?} », toi tu es « $moi »."
    rouge "  -> stop/go/dsp exigent le meme compte. Lance harvard ainsi :  sudo ./harvard.sh"
    rouge "     (ou relance le banc sous ton compte). Lecture seule (d@) possible, ecriture (d@!) non."
}
vivant_coop() { grep -q -a calypso_dsp_pause /proc/$1/exe 2>/dev/null; }
vivant_decalages() { # vivant_decalages <pid> -> fichier "champ decalage" (gdb sur l'executable, une fois par pid)
    f=$DIR/decalages.$1.txt
    if ! [ -s "$f" ]; then
        command -v gdb > /dev/null || { rouge "gdb absent : impossible de situer C54xState dans le processus" >&2; return 1; }
        { for c in data prog pc xpc idle running a b ar t sp st0 st1 pmst imr ifr bk brc rsa rea trn insn_count last_exec_pc last_exec_op; do
              printf 'printf "%s %%ld\\n", (long)&((C54xState*)0)->%s\n' "$c" "$c"; done
          printf 'printf "FN %%ld\\n", (long)&g_c54x_exe_fn\n'; } > "$DIR/decal.gdb"
        gdb -batch -q -nx -ex 'set debuginfod enabled off' -x "$DIR/decal.gdb" /proc/$1/exe 2>/dev/null | grep -E '^[a-zA-Z_0-9]+ [0-9]+$' > "$f"
        [ "$(wc -l < "$f")" -ge 25 ] || { rm -f "$f"; rouge "gdb n'a pas rendu les decalages (executable sans -g ?)" >&2; return 1; }
    fi
    echo "$f"
}
vivant_lire() { # vivant_lire <pid> <etiquette> : registres du coeur du processus -> $VIVANT (lignes "CLE val")
    pid=$1; d=$(vivant_decalages $pid) || return 1
    shm=$(awk '/calypso_api_ram/ {print $1; exit}' /proc/$pid/maps); [ -n "$shm" ] || { rouge "pas de fenetre API RAM dans le processus $pid"; return 1; }
    base=$(( 0x${shm%%-*} - 0x1000 - $(awk '$1=="data" {print $2}' "$d") ))      # data[0x0800] est mappe sur /dev/shm
    exe=$(awk '$6 ~ /c54x_exe/ && $3=="00000000" {print $1; exit}' /proc/$pid/maps); exe=$(( 0x${exe%%-*} ))
    dec() { awk -v k="$1" '$1==k {print $2}' "$d"; }
    lire() { dd if=/proc/$pid/mem bs=1 skip=$1 count=$2 2>/dev/null | od -An -v -tx$2 | tr -d ' \n'; }   # petit-boutiste -> hex
    echo "$pid $base $(dec prog) $(dec data) $(( 0x$(lire $(( base + $(dec pmst) )) 2) ))" > "$DIR/vivant_prog.txt"   # pour vivant_prog
    pc=$(lire $(( base + $(dec pc) )) 4) || return 1
    [ -n "$pc" ] || { rouge "lecture de /proc/$pid/mem refusee (ptrace)"; return 1; }
    {
        echo "ETAT $2"; echo "FN $(( 0x$(lire $(( exe + $(dec FN) )) 4) ))"
        printf 'PC %04x\nXPC %x\nIDLE %d\nRUNNING %d\nINSN %d\nLAST_PC %04x\nLAST_OP %04x\n' $(( 0x$pc & 0xffff )) 0x$(lire $(( base + $(dec xpc) )) 2) \
            0x$(lire $(( base + $(dec idle) )) 1) 0x$(lire $(( base + $(dec running) )) 1) 0x$(lire $(( base + $(dec insn_count) )) 4) \
            0x$(lire $(( base + $(dec last_exec_pc) )) 2) 0x$(lire $(( base + $(dec last_exec_op) )) 2)
        printf 'A %x\nB %x\n' $(( 0x$(lire $(( base + $(dec a) )) 8) & 0xFFFFFFFFFF )) $(( 0x$(lire $(( base + $(dec b) )) 8) & 0xFFFFFFFFFF ))
        for r in t sp st0 st1 pmst imr ifr bk brc rsa rea trn; do printf '%s %x\n' "$(printf '%s' $r | tr a-z A-Z)" 0x$(lire $(( base + $(dec $r) )) 2); done
        i=0; for v in $(dd if=/proc/$pid/mem bs=1 skip=$(( base + $(dec ar) )) count=16 2>/dev/null | od -An -v -tx2); do printf 'AR%d %x\n' $i 0x$v; i=$(( i + 1 )); done
    } > "$VIVANT"
}
vivant_prog() { # vivant_prog <adr> <n> : n mots de la memoire programme du processus vivant, comme prog_fetch
    set -- $(cat "$DIR/vivant_prog.txt") "$1" "$2"      # pid base prog data pmst adr n
    # c54x_mem.c prog_fetch : PMST.OVLY (bit 5) et 0x60 <= adr < 0x2800 -> data[adr] (DARAM recouverte), sinon prog[adr]
    if [ $(( $5 & 0x20 )) != 0 ] && [ $6 -ge 96 ] && [ $6 -lt 10240 ]; then o=$4; else o=$3; fi
    dd if=/proc/$1/mem bs=1 skip=$(( $2 + o + $6 * 2 )) count=$(( $7 * 2 )) 2>/dev/null | od -An -v -tx2 | tr -s ' \n' ' '
}
montrer_vivant() { # montrer_vivant <pid> : $VIVANT a l'ecran, avec les adresses du stop / go
    v() { awk -v k="$1" '$1==k {print substr($0, length($1) + 2)}' "$VIVANT"; }
    pc=$(v PC); xpc=$(v XPC); idle=$(v IDLE); pmst=$(v PMST); imr=$(v IMR); ifr=$(v IFR)
    gras "  DSP vivant (pid $1, $(vivant_coop $1 && echo 'pause cooperative SIGUSR1 disponible' || echo 'binaire ancien : SIGSTOP/SIGCONT')) : $(v ETAT)"
    printf '  fn=%s  PC=%x:%s  idle=%s running=%s  insn=%s  derniere instruction executee : P[%s]=%s\n' "$(v FN)" $(( 0x$xpc )) "$pc" "$idle" "$(v RUNNING)" "$(v INSN)" "$(v LAST_PC)" "$(v LAST_OP)"
    ( ETAT=$VIVANT; montrer_regs )
    iptr=$(( 0x$pmst >> 7 )); vbase=$(( iptr << 7 ))
    gras "  adresses de ce stop / go :"
    if [ "$idle" = 1 ]; then printf '    stop : PC=%x:%s  IDLE, le DSP attend une interruption (INTM=%d)\n' $(( 0x$xpc )) "$pc" $(( 0x${v_st1:-$(v ST1)} >> 11 & 1 ))
    else printf '    stop : PC=%x:%s  (en cours d execution, pas en IDLE)\n' $(( 0x$xpc )) "$pc"; fi
    printf '    go   : vecteurs armes (IMR=%04x IFR=%04x, IPTR=%03x -> table a %04x) :\n' $(( 0x$imr )) $(( 0x$ifr )) $iptr $vbase
    b=0; while [ $b -lt 16 ]; do
        if [ $(( 0x$imr >> b & 1 )) = 1 ]; then
            vec=$(( b + 16 )); adr=$(( vbase + vec * 4 )); nom=$(printf '%s\n' $IRQ_NOMS | awk -F= -v k=$b '$1==k {print $2}')
            cible=$(dis_mots "$(printf '%04x' $adr)" $(vivant_prog $adr 2) | head -1 | sed 's/^ *[0-9a-f]*:[[:space:]]*//; s/[[:space:]]\+/ /g')
            printf '           bit %2d vec %2d -> %04x %-22s %s%s\n' $b $vec $adr "${nom:-?}" "${cible:+: $cible}" "$([ $(( 0x$ifr >> b & 1 )) = 1 ] && echo '   <- EN ATTENTE (IFR)')"
        fi; b=$(( b + 1 )); done
    lin=$(( (0x$xpc << 16) | 0x$pc ))
    if rom_fichier $lin P > /dev/null 2>&1; then gras "  ROM a PC :"; cmd_p "$(printf '%x:%04x' $(( lin >> 16 )) $(( lin & 0xffff )))" 4; fi
}
cmd_dsp() { # dsp : instantane du DSP en marche, sans l'arreter
    pid=$(vivant_pid) || { rouge "aucun c54x_exe --arm en marche (./run.sh)"; return 1; }
    vivant_accessible "$pid" || { vivant_diag "$pid"; return 1; }
    vivant_lire $pid "instantane sans arret$([ -f "$DIR/pause.$pid" ] && echo ' (en pause : go pour reprendre)')" || return 1; montrer_vivant $pid
}
cmd_stop() { # stop : pause du DSP en marche
    pid=$(vivant_pid) || { rouge "aucun c54x_exe --arm en marche (./run.sh)"; return 1; }
    vivant_accessible "$pid" || { vivant_diag "$pid"; return 1; }
    [ -f "$DIR/pause.$pid" ] && { echo "  deja en pause ($(cat "$DIR/pause.$pid")) : go pour reprendre"; cmd_dsp; return; }
    if vivant_coop $pid; then
        rm -f "$PAUSE_FICHIER"; kill -USR1 $pid || return 1; i=0
        while [ $i -lt 50 ] && ! grep -q '^ETAT pause' "$PAUSE_FICHIER" 2>/dev/null; do sleep 0.1; i=$(( i + 1 )); done
        grep -q '^ETAT pause' "$PAUSE_FICHIER" 2>/dev/null || { rouge "pas de pause apres 5 s (le DSP attend un GO de l'ARM ?) ; kill -USR2 $pid pour annuler"; return 1; }
        mode="SIGUSR1 (frontiere de trame, fn=$(awk '$1=="FN"{print $2}' "$PAUSE_FICHIER"))"
    else
        kill -STOP $pid || return 1; mode="SIGSTOP"
    fi
    echo "$mode" > "$DIR/pause.$pid"
    vivant_lire $pid "EN PAUSE par $mode" || return 1; montrer_vivant $pid
    echo "  (l'ARM/QEMU attend le DONE du DSP : toute la chaine est suspendue ; « go » pour reprendre)"
}
cmd_go() { # go : reprise du DSP en marche
    pid=$(vivant_pid) || { rouge "aucun c54x_exe --arm en marche"; rm -f "$DIR"/pause.*; return 1; }
    [ -f "$DIR/pause.$pid" ] || { echo "  pas de pause en cours (stop d'abord)"; return 1; }
    mode=$(cat "$DIR/pause.$pid"); rm -f "$DIR/pause.$pid"
    case $mode in SIGUSR1*) kill -USR2 $pid; i=0; while [ $i -lt 30 ] && ! grep -q '^ETAT reprise' "$PAUSE_FICHIER" 2>/dev/null; do sleep 0.1; i=$(( i + 1 )); done ;;
                  *) kill -CONT $pid ;; esac
    avant_insn=$(awk '$1=="INSN"{print $2}' "$VIVANT" 2>/dev/null); avant_fn=$(awk '$1=="FN"{print $2}' "$VIVANT" 2>/dev/null)
    echo "  go : reprise ($mode)"; sleep 0.3
    vivant_lire $pid "0,3 s apres la reprise" || return 1
    v() { awk -v k="$1" '$1==k {print $2}' "$VIVANT"; }
    printf '  0,3 s apres : fn=%s (+%d trames)  PC=%x:%s idle=%s  insn=%s (+%d)\n' "$(v FN)" $(( $(v FN) - ${avant_fn:-0} )) $(( 0x$(v XPC) )) "$(v PC)" "$(v IDLE)" "$(v INSN)" $(( $(v INSN) - ${avant_insn:-0} ))
}

# ---------------------------------------------------------------- ARM : le firmware layer1 sous gdb (telnet 44444)
# run.sh lance QEMU avec -gdb tcp::1234 et la console tools/gdb-telnet.py sur le port 44444 : un
# gdb-multiarch attache au firmware (symboles layer1.highram.elf), panneau cmd.gdb + « demo ».
# Une seule session a la fois (le gdbstub n'accepte qu'un client) : si un telnet est deja ouvert,
# on ne force rien, on dit quoi y taper.
GDB_TELNET=${GDB_TELNET:-44444}
telnet_occupe() { ss -Htn state established "( sport = :$GDB_TELNET )" 2>/dev/null | grep -q . ; }
cmd_gdb() { # gdb <commande...> : envoyer une commande au gdb de l'ARM via le telnet, montrer la sortie
    [ $# -ge 1 ] || { echo "usage : gdb <commande du panneau osmocom>   ex : gdb fn | gdb l1 | gdb help_osmo | gdb demo etat"; return 1; }
    ss -Hltn "( sport = :$GDB_TELNET )" 2>/dev/null | grep -q . || { rouge "rien n'ecoute sur le port $GDB_TELNET (run.sh avec GDB=1)"; return 1; }
    if telnet_occupe; then rouge "une session telnet est deja ouverte sur le port $GDB_TELNET (une seule a la fois) : tape dans ce telnet :   $*"; return 1; fi
    gras "  telnet 127.0.0.1:$GDB_TELNET  (gdb) $*"
    python3 - "$GDB_TELNET" "${GDB_ATTENTE:-90}" "$*" <<'PY'
import re, socket, sys, time
port, attente, cmd = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
s = socket.create_connection(("127.0.0.1", port), timeout=5)
def lire(jusqua, t):
    buf = b""; fin = time.time() + t
    while time.time() < fin:
        s.settimeout(max(0.1, fin - time.time()))
        try: d = s.recv(4096)
        except socket.timeout: break
        if not d: break
        buf += d
        if jusqua in buf: break
    return buf
lire(b"(gdb) ", 20)                                   # gdb attache, cible relancee (continue &)
s.sendall(b"\x03"); lire(b"(gdb) ", 5); time.sleep(0.5)   # Ctrl-C : la cible s'arrete, le prompt revient
s.sendall(cmd.encode() + b"\necho __FIN__\\n\n")
out = lire(b"__FIN__", attente)
s.sendall(b"quit\n"); time.sleep(0.3); s.close()      # a la fermeture, la console detache gdb : l'ARM repart
txt = re.sub(rb"\xff[\xfb-\xfe].|\xff\xfa.*?\xff\xf0", b"", out).decode("utf-8", "replace").replace("\r", "")
txt = txt.split("__FIN__")[0]
print("\n".join("    " + l for l in txt.split("\n") if not l.startswith("echo __FIN__") and l.strip() != "(gdb)"))
PY
    echo "  (session fermee : gdb detache, l'ARM tourne)"
}
# ---------------------------------------------------------------- demo : la main sur le DSP vivant, puis sur l'ARM
DEMO_PAUSE=${DEMO_PAUSE:-4}    # secondes entre deux etapes de la demo, le temps de lire (0 = enchainer)
DEMO_STOP=${DEMO_STOP:-1.5}    # duree de la pause reelle du DSP : assez pour que la voix se coupe a l'oreille, pas plus
demo_pause() { [ "$DEMO_PAUSE" != 0 ] && { echo "    ... (suite dans $DEMO_PAUSE s)"; sleep "$DEMO_PAUSE"; }; }
A_CD=09d2       # a_cd[0] (flag FIRE) ; a_cd[3]=0x09d5 debut du bloc L2 de 23 octets (dsp_memcpy_from_api be=0)
# Dans le bloc descendant CCCH/BCCH : octet2 (type) = 0x09d6 bas ; cell_identity = 0x09d6 haut (MSB) | 0x09d7 bas (LSB).
shm_lire() { od -An -v -tx2 -j $(( (0x$1 - API_BASE) * 2 )) -N 2 "$API_SHM" 2>/dev/null | tr -d ' \n'; }  # 1 mot hex
shm_ecrire() { local v=$(( 0x$2 & 0xffff )); printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" \
    | dd of="$API_SHM" bs=1 seek=$(( (0x$1 - API_BASE) * 2 )) conv=notrunc status=none 2>/dev/null; }
api_mot() { cmd_d_chaud "$1" 1 2>/dev/null | sed -n 's/.*\]=\([0-9a-fA-F]\{4\}\).*/\1/p' | head -1; }  # EXACTEMENT 4 hex, sinon vide
demo_si3() { # cell_id -> CELL_ID (defaut 777) dans SI3 : boucle vive, on reecrit a chaque bloc SI3 qui passe
    cible=${1:-777}
    [ -r "$API_SHM" ] || { rouge "    SI3 : pas de segment API RAM ; ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    SI3 : segment en lecture seule ; chmod 0666 $API_SHM (ou sudo)"; return 1; }
    gras "    SI3 : relabellise le cell_id en $cible (0x$(printf %04x $cible)) a chaque bloc SI3 (type L2 0x1b) qui passe dans a_cd"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-1500} )); n=0; av=""
    while [ $(date +%s%3N) -lt $fin ]; do
        w4=$(shm_lire 09d6); case $w4 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ $(( 0x$w4 & 0xff )) = $(( 0x1b )) ] || continue             # pas un SI3 a cet instant
        w5=$(shm_lire 09d7); case $w5 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ -z "$av" ] && { av=$(( (0x$w4 >> 8 & 0xff) << 8 | (0x$w5 & 0xff) )); echo "       1er SI3 vu : cell_id = $av (0x$(printf %04x $av))"; }
        shm_ecrire 09d6 $(printf '%04x' $(( (0x$w4 & 0x00ff) | ((cible >> 8 & 0xff) << 8) )))   # garde le type 0x1b
        shm_ecrire 09d7 $(printf '%04x' $(( (0x$w5 & 0xff00) | (cible & 0xff) )))               # garde l'octet LAI
        n=$(( n + 1 ))
    done
    if [ "$n" = 0 ]; then echo "       (aucun SI3 vu en ${DEMO_HOLD_MS:-1500} ms ; le mobile doit lire la BCCH/CCCH -- reessaie quand il campe)"; return 1; fi
    vert "       $n blocs SI3 relabellises en $cible pendant ${DEMO_HOLD_MS:-1500} ms (le flag FIRE reste valide, pas de CRC L3)"
    echo "       -> a chaque SI3 que le firmware lit maintenant, le cell_id vaut $cible"
}
LAC_W=09d9      # a_cd[7] : les 2 octets du LAC du SI3 (octet-echanges : a_cd[7] = LAC_LSB<<8 | LAC_MSB)
demo_lac() { # lac [valeur] : forcer le LAC du SI3 a valeur (defaut 777) EN VIF via le shm, sans relance ni rebuild
    cible=${1:-777}
    [ -r "$API_SHM" ] || { rouge "    LAC : pas de segment API RAM ; ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    LAC : segment en lecture seule ; chmod 0666 $API_SHM (ou sudo)"; return 1; }
    sw=$(printf '%04x' $(( ((cible & 0xff) << 8) | ((cible >> 8) & 0xff) )))   # octet-echange pour a_cd[7]
    gras "    LAC : force a $cible (0x$(printf %04x $cible)) dans a_cd[7]=0x$sw, EN VIF, a chaque bloc SI3 (sans relance)"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-3000} )); n=0; av=""
    while [ $(date +%s%3N) -lt $fin ]; do
        w4=$(shm_lire 09d6); case $w4 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ $(( 0x$w4 & 0xff )) = $(( 0x1b )) ] || continue              # SI3 seulement
        w7=$(shm_lire $LAC_W); case $w7 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ -z "$av" ] && { av=$(( ((0x$w7 & 0xff) << 8) | ((0x$w7 >> 8) & 0xff) )); echo "       1er SI3 vu : LAC = $av (0x$(printf %04x $av))"; }
        shm_ecrire $LAC_W $sw; n=$(( n + 1 ))
    done
    [ "$n" = 0 ] && { echo "       (aucun SI3 vu en ${DEMO_HOLD_MS:-3000} ms ; le mobile doit lire la BCCH)"; return 1; }
    vert "       $n ecritures LAC=$cible pendant ${DEMO_HOLD_MS:-3000} ms, en vif (course avec le DSP : le mobile le voit si on gagne)"
    echo "       -> verifie : grep -ao 'CGI=[0-9-]*' /tmp/c54x-pont/mobile.log | tail  (champ LAC = 3e groupe)"
}
demo_sb() { # DEAD BEEF dans a_sch (le SB decode que l'ARM lit), en vif : on reecrit en boucle courte, on relit
    [ -r "$API_SHM" ] || { rouge "    SB : pas de segment API RAM ($API_SHM) ; ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    SB : segment en lecture seule pour ce compte ; chmod 0666 $API_SHM (ou sudo)"; return 1; }
    a=0837   # a_sch[0], page R0 (voir d@ noms a_sch)
    gras "    SB : a_sch a $a (le SCH decode que le firmware lit en 0xffd00000), ecriture vive"
    echo "       avant : a_sch[0..1] = $(shm_lire $a) $(shm_lire 0838)"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-1200} ))
    while [ $(date +%s%3N) -lt $fin ]; do shm_ecrire $a 40c3; shm_ecrire 0838 2026; done
    echo "       pendant : a_sch[0..1] = $(shm_lire $a) $(shm_lire 0838)   <- 40C3 2026 (CCC 2026) impose au SB que lit le firmware"
    vert "       (reecrit en boucle ${DEMO_HOLD_MS:-1200} ms ; le DSP reprend la main a la prochaine tache SB)"
}
demo_si3_preuve() { # attend le camping et montre le CGI vu par le mobile (doit finir par <cible>)
    cible=$1; log=${RUNDIR_OMBRE:-/tmp/c54x-pont}/mobile.log; dlog=${RUNDIR_OMBRE:-/tmp/c54x-pont}/dsp.log
    gras "    SI3 : preuve cote mobile -- cell_id force a $cible a la source (ombre descendante dans c54x_exe)"
    echo "       injections SI3 (DSP) : $(grep -ac 'ombre-dl.*SI3 cell_id' "$dlog" 2>/dev/null)   (0 si le mobile ne lit pas encore la BCCH)"
    i=0; cgi=""
    while [ $i -lt ${DEMO_CGI_ESSAIS:-40} ]; do
        cgi=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1)
        case $cgi in *-$cible) break ;; esac
        sleep 1; i=$(( i + 1 ))
    done
    case $cgi in
        *-$cible) vert "       mobile.log : $cgi   <- le telephone a decode le cell_id $cible (ombre reussie)" ;;
        '') echo "       (mobile pas encore campe apres ${DEMO_CGI_ESSAIS:-40} s ; grep CGI $log plus tard)" ;;
        *) echo "       mobile.log : $cgi (pas encore $cible ; injections=$(grep -ac 'ombre-dl.*SI3 cell_id' "$dlog" 2>/dev/null), laisse camper puis grep CGI $log)" ;;
    esac
}
demo_dsp() { # DSP vivant, SANS relance : force le LAC du SI3 en vif (shm), preuve CGI, stop/go
    pid=$(vivant_pid) || { rouge "demo dsp : aucun c54x_exe --arm en marche (./run.sh)"; return 1; }
    gras "===== DEMO DSP (c54x_exe --arm pid $pid, le vrai coeur C54x qui sert l'ARM) ====="
    log=${RUNDIR_OMBRE:-/tmp/c54x-pont}/mobile.log
    cible=${DEMO_CELL_ID:-777}
    gras "--- 1. LAC du SI3 force a $cible EN VIF (shm, SANS relance ; le banc reste campe)"
    av=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1); echo "       CGI avant : ${av:-<pas encore de camp>}"
    DEMO_HOLD_MS=${DEMO_LAC_MS:-5000} demo_lac "$cible"
    demo_pause
    gras "--- 2. preuve cote mobile : le LAC (3e groupe du CGI) doit passer a $cible"
    i=0; cgi=""
    while [ $i -lt ${DEMO_CGI_ESSAIS:-20} ]; do
        cgi=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1)
        case $cgi in CGI=*-*-$cible-*) break ;; esac
        DEMO_HOLD_MS=1500 demo_lac "$cible" >/dev/null 2>&1   # re-hammer pendant que le mobile relit la BCCH
        sleep 1; i=$(( i + 1 ))
    done
    case $cgi in
        CGI=*-*-$cible-*) vert "       mobile.log : $cgi   <- LAC = $cible, le telephone l'a decode (en vif, sans relance)" ;;
        '') echo "       (pas de CGI : le mobile n'est pas campe ; il faut qu'il lise la BCCH)" ;;
        *) echo "       mobile.log : $cgi (pas encore $cible au 3e groupe ; course avec le DSP, relance 'lac $cible' ou laisse camper)" ;;
    esac
    demo_pause
    if vivant_accessible "$pid"; then
        gras "--- 3. stop : pause du DSP, registres et LES ADRESSES"; cmd_stop || return 1; demo_pause
        gras "--- 4. go : reprise (fn, instructions)"; cmd_go
        gras "===== FIN DEMO DSP : LAC du SI3 force a $cible en vif, DSP relance, rien de casse, pas de relance du banc ====="
    else
        echo; vivant_diag "$pid"
        gras "===== FIN DEMO DSP : LAC force en vif. stop/go : harvard sous le compte du banc ====="
    fi
}
cmd_demo() { # demo [tout|arm [etat|trame|hello|go]|dsp] : d'abord l'ARM sous gdb (hello world from ram), puis le DSP vivant
    quoi=${1:-tout}; [ $# -gt 0 ] && shift
    case $quoi in
        dsp) demo_dsp ;;
        arm) gras "===== DEMO ARM (firmware layer1 dans QEMU, gdb par le telnet $GDB_TELNET) ====="; cmd_gdb demo "$@" ;;
        tout) gras "===== DEMO ARM D'ABORD (firmware layer1 dans QEMU, gdb par le telnet $GDB_TELNET : hello world from ram) ====="
              cmd_gdb demo "$@"; demo_pause; echo
              demo_dsp
              echo; gras "===== FIN DE LA DEMO : ARM en marche, DSP relance, rien de casse ====="; demo_pause ;;
        *) echo "usage : demo [tout | dsp | arm [etat|trame|hello|go]]"; return 1 ;;
    esac
}

# ---------------------------------------------------------------- API RAM a chaud (le DSP en marche, --arm)
API_SHM=${API_SHM:-/dev/shm/calypso_api_ram}   # = data[0x0800..0x27ff] du c54x_exe --arm, partage avec l'ARM (MAP_SHARED)
API_BASE=$(( 0x0800 )); API_MOTS=8192
CARTE=${CARTE:-$HERE/docs/carte-memoire.md}    # table « API RAM — tous les champs » (generee depuis l'ELF : scalaires seulement)
# Pages doubles (osmocom-bb include/calypso/dsp_api.h:18-23) : mot DSP = 0x0800 + (adresse ARM - 0xFFD00000)/2
#   W0 0x0800..0x0813  W1 0x0814..0x0827  (T_DB_MCU_TO_DSP, 17 mots utiles + 3 de bourrage, dsp_api.h:32-80)
#   R0 0x0828..0x083b  R1 0x083c..0x084f  (T_DB_DSP_TO_MCU, 20 mots, dsp_api.h:82-107)   NDB 0x08d4.. (268)  PARAM 0x0c31.. (57)
api_page() { # api_page <mot DSP> -> W0|W1|R0|R1|NDB|PARAM|API
    a=$1; if [ $a -lt $(( 0x0814 )) ]; then echo W0; elif [ $a -lt $(( 0x0828 )) ]; then echo W1
    elif [ $a -lt $(( 0x083c )) ]; then echo R0; elif [ $a -lt $(( 0x0850 )) ]; then echo R1
    elif [ $a -ge $(( 0x08d4 )) ] && [ $a -lt $(( 0x08d4 + 268 )) ]; then echo NDB
    elif [ $a -ge $(( 0x0c31 )) ] && [ $a -lt $(( 0x0c31 + 57 )) ]; then echo PARAM; else echo API; fi
}
api_table() { # "nom 0xADDR page" : la carte generee + les tableaux que l'ELF ne nomme pas (dsp_api.h, API 36xx)
    grep '^| [^|]* | [a-z][a-z0-9_]* | 0x[0-9a-fA-F]* | 0xFFD[0-9A-F]* | 0x[0-9a-f]*' "$CARTE" 2>/dev/null | awk -F'|' '{gsub(/ /,"",$2); gsub(/ /,"",$3); gsub(/ /,"",$6); print $3, $6, $2}'
    for pp in 0x0800:W0 0x0814:W1; do pb=$(( ${pp%%:*} )); ppg=${pp#*:}                                  # dsp_api.h:67-70, bourrage 17..19
        printf 'a_a5fn[0] 0x%04x %s\na_a5fn[1] 0x%04x %s\n' $(( pb + 12 )) "$ppg" $(( pb + 13 )) "$ppg"
        printf '(bourrage) 0x%04x %s\n(bourrage) 0x%04x %s\n(bourrage) 0x%04x %s\n' $(( pb + 17 )) "$ppg" $(( pb + 18 )) "$ppg" $(( pb + 19 )) "$ppg"; done
    for pp in 0x0828:R0 0x083c:R1; do pb=$(( ${pp%%:*} )); ppg=${pp#*:}; pi=0                           # dsp_api.h:97-99 (DSP 33..36)
        for nm in 'a_serv_demod[0]=D_TOA' 'a_serv_demod[1]=D_PM' 'a_serv_demod[2]=D_ANGLE' 'a_serv_demod[3]=D_SNR' 'a_pm[0]' 'a_pm[1]' 'a_pm[2]' 'a_sch[0]' 'a_sch[1]' 'a_sch[2]' 'a_sch[3]' 'a_sch[4]'; do
            printf '%s 0x%04x %s\n' "$nm" $(( pb + 8 + pi )) "$ppg"; pi=$(( pi + 1 )); done; done
}
api_adr() { # api_adr <nom|adresse> -> entier (mot DSP), 1 si inconnu / hors fenetre
    case $1 in
        [a-z]*) v=$(api_table | awk -v n="$1" '$1==n || (n !~ /\[/ && $1 ~ ("^" n "(\\[0\\])?(=|$)")) {print $2; exit}'); [ -n "$v" ] || { rouge "champ inconnu ($CARTE + dsp_api.h) : $1   -> d@ noms <motif>" >&2; return 1; }; v=$(( v )) ;;
        *) v=$(hex "$1") || { rouge "adresse invalide : $1" >&2; return 1; } ;;
    esac
    [ $v -ge $API_BASE ] && [ $v -lt $(( API_BASE + API_MOTS )) ] || { rouge "$(printf '%04x' $v) : hors fenetre API (0800-27ff)" >&2; return 1; }
    printf '%d' $v
}
api_nom() { api_table | awk -v a="$(printf '0x%04x' $1)" '$2==a {print $3 " " $1; exit}'; }
api_vivant() { [ -r "$API_SHM" ] || { rouge "pas de segment $API_SHM (lancer ./run.sh ou ./c54x_exe --arm)"; return 1; }
    pgrep -f 'c54x_exe.*--arm' > /dev/null || echo "  (aucun c54x_exe --arm en marche : contenu fige du dernier run)"; }
# Decodage des champs (sources : dsp_api.h:46-79 pour les bits, l1_environment.h:36-80 pour les taches,
# :339-353 et :370-386 pour d_ctrl_tch, :355-367 pour d_ctrl_abb / d_ctrl_system). Rien n'est devine.
TACHES="0=NO 1=IDLE1 5=FB 6=SB 8=TCH_FB 9=TCH_SB 10=RACH 11=AUL 12=DUL 13=TCHT 14=TCHA 15=TCH_DTX_UL 17=NBN 18=EBN 19=NBS 20=EBS 21=NP 22=EP 24=ALLC 25=CB 26=DDL 27=ADL 28=TCHD 30=PNP 31=PEP 32=PALLC 33=CHECKSUM/PBS 35=TST_NDB 36=TST_DB 37=INIT_VEGA 38=DSP_LOOP_C"
MODES="0=SIG_ONLY 1=TCH_FS 2=TCH_HS 3=TCH_96 4=TCH_48F 5=TCH_48H 6=TCH_24F 7=TCH_24H 8=TCH_EFR 9=TCH_144"
TYPES="0=INVALID 1=TCH_F 2=TCH_H 3=SDCCH_4 4=SDCCH_8"
decoder() { # decoder <nom> <hex16> -> texte (vide si rien a dire)
    n=${1%%=*}; v=$(( 0x$2 ))
    case $n in
        d_task_d|d_task_u|d_task_md|d_task_ra) t=$(printf '%s\n' $TACHES | awk -F= -v k=$v '$1==k {print $2}'); printf 'tache %d%s' $v "${t:+ = $t}" ;;
        d_fn) printf 'fn_report=%d (FN dans la periode de rapport)  fn_sid=%d (FN%%104)' $(( v & 0xff )) $(( v >> 8 )) ;;
        d_ctrl_tch) m=$(printf '%s\n' $MODES | awk -F= -v k=$(( v & 0xf )) '$1==k {print $2}'); ty=$(printf '%s\n' $TYPES | awk -F= -v k=$(( v >> 4 & 3 )) '$1==k {print $2}')
            printf 'chan_mode=%d%s chan_type=%d%s' $(( v & 0xf )) "${m:+($m)}" $(( v >> 4 & 3 )) "${ty:+($ty)}"
            [ $(( v >> 6 & 1 )) = 1 ] && printf ' RESET_SACCH'; [ $(( v >> 7 & 1 )) = 1 ] && printf ' VOCODER_ON'
            [ $(( v >> 8 & 1 )) = 1 ] && printf ' SYNC_TCH_UL'; [ $(( v >> 9 & 1 )) = 1 ] && printf ' SYNC_TCH_DL'
            [ $(( v >> 10 & 1 )) = 1 ] && printf ' STOP_TCH_UL'; [ $(( v >> 11 & 1 )) = 1 ] && printf ' STOP_TCH_DL'
            [ $(( v >> 12 & 3 )) != 0 ] && printf ' tch_loop=%d' $(( v >> 12 & 3 )); [ $(( v >> 15 & 1 )) = 1 ] && printf ' SUBCHANNEL' ;;
        d_ctrl_abb) printf 'bits:'; [ $(( v & 1 )) = 1 ] && printf ' b_ramp(a_ramp dans la NDB)'; [ $(( v >> 3 & 1 )) = 1 ] && printf ' b_apcdel(delais dans la NDB)'
            [ $(( v >> 4 & 1 )) = 1 ] && printf ' b_afc(d_afc valide)'; [ $(( v & ~0x19 )) != 0 ] && printf ' +inconnus(%04x)' $(( v & ~0x19 & 0xffff )) ;;
        'a_a5fn[0]') printf 'T2=%d T3=%d (chiffrement A5)' $(( v & 0x1f )) $(( v >> 5 & 0x3f )) ;;
        'a_a5fn[1]') printf 'T1=%d (chiffrement A5)' $(( v & 0xfff )) ;;
        d_ctrl_system) printf 'b_tsq=%d (sequence d apprentissage)' $(( v & 7 )); [ $(( v >> 3 & 1 )) = 1 ] && printf ' BCCH_FREQ_IND'; [ $(( v >> 15 & 1 )) = 1 ] && printf ' TASK_ABORT' ;;
        d_afc) [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'signe %d' $v ;;
        'a_serv_demod[0]') [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'TOA %d' $v ;;
        'a_serv_demod[2]') [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'angle %d (signe)' $v ;;
    esac
}
cmd_d_chaud() { # d@ <adr|nom> [n] : lire l'API RAM du DSP en marche, etiquetee par adresse absolue et decodee
    [ $# -ge 1 ] || { echo "usage : d@ <adresse|nom> [n]   ex : d@ 0800 20 | d@ d_task_d | d@ W1 | d@ noms d_task"; return 1; }
    case $1 in
        noms) api_table | grep "${2:-.}" | awk '{printf "    %-5s %s  %s\n", $3, $2, $1}'; return ;;
        W0) set -- 0800 17 ;; W1) set -- 0814 17 ;; R0) set -- 0828 20 ;; R1) set -- 083c 20 ;;
    esac
    api_vivant || return 1; a=$(api_adr "$1") || return 1; n=${2:-8}
    api_table > "$DIR/table.txt"
    od -An -v -tx2 -w2 -j $(( (a - API_BASE) * 2 )) -N $(( n * 2 )) "$API_SHM" > "$DIR/mots.txt"
    while read -r mot; do
        [ -n "$mot" ] || continue
        set -- $(awk -v ad="$(printf '0x%04x' $a)" '$2==ad {print $3, $1; exit}' "$DIR/table.txt")
        pg=${1:-$(api_page $a)}; nom=${2:-}
        printf '    %-5s [%04x]=%s  %-18s %s\n' "$pg" $a "$mot" "$nom" "$([ -n "$nom" ] && decoder "$nom" "$mot")"
        a=$(( a + 1 ))
    done < "$DIR/mots.txt"
}
cmd_d_chaud_ecrire() { # d@! <adr|nom> <mots...> : ecrire dans l'API RAM du DSP en marche (vu par le DSP et par l'ARM)
    [ $# -ge 2 ] || { echo "usage : d@! <adresse|nom> <mot> [mot...]"; return 1; }
    api_vivant || return 1; a=$(api_adr "$1") || return 1; shift
    [ -w "$API_SHM" ] || { rouge "segment en lecture seule pour cet utilisateur"; return 1; }
    for m in "$@"; do v=$(hex "$m") || { rouge "mot invalide : $m"; return 1; }; v=$(( v & 0xffff ))
        [ $a -lt $(( API_BASE + API_MOTS )) ] || { rouge "debordement de la fenetre"; return 1; }
        printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" \
            | dd of="$API_SHM" bs=1 seek=$(( (a - API_BASE) * 2 )) conv=notrunc status=none || { rouge "ecriture refusee"; return 1; }
        printf '    [%04x] <- %04x  %s\n' $a $v "$(api_nom $a)"; a=$(( a + 1 )); done
    echo "  (ecrit dans le DSP en marche ; l'ARM voit le meme segment)"
}
# ---------------------------------------------------------------- « run » : les outils du banc tels quels
lancer() { # lancer <cmd...> : execute, journalise, montre la fin
    gras "  \$ $*"; "$@" > "$DIR/dernier_run.log" 2>&1; rc=$?
    n=$(wc -l < "$DIR/dernier_run.log")
    [ $n -gt 60 ] && echo "  ... ($n lignes, tout dans $DIR/dernier_run.log)"
    tail -n 60 "$DIR/dernier_run.log"; echo "  (code de retour $rc)"; return $rc
}
cmd_rejeu() { # rejeu [trames] [-v..] : c54x_exe --rejouer, acquisition FB/SB deterministe
    t=${1:-200}; shift 2>/dev/null; (cd "$HERE" && lancer ./c54x_exe --rejouer --trames "$t" --verbeux "$@"); }
cmd_banc() { # banc [test|--all|--list] [options dsp_tester]
    [ $# -ge 1 ] || set -- --list; (cd "$HERE" && lancer ./dsp_tester "$@"); }
cmd_isa() { # isa [-k MOT] : les exemples SPRU172C contre le coeur
    (cd "$HERE" && lancer ./isa_test tools/isa_tests.txt "$@"); }
RUNDIR_OMBRE=${RUNDIR_OMBRE:-/tmp/c54x-pont}
cmd_ombre() { # ombre [cell_id|off] : relance la chaine COMPLETE (run_si.sh : BTS + echafaudage + pont) avec l'ombre SI3
    n=${1:-${DEMO_CELL_ID:-777}}
    case $n in off|OFF) sv=""; lib="desactivee" ;; *[!0-9]*) rouge "usage : ombre [cell_id|off]  (defaut 777)"; return 1 ;; *) sv="$n"; lib="LAC SI3 = $n" ;; esac
    # run_si.sh amene le mobile a decoder la BCCH (corrrelateur natif non verrouille -> echafaudage CAN_TOA/CAN_SB/AFC/LOCKSTEP).
    # run.sh nu (PONT=0) ne fait arriver AUCUN burst du BTS : pas de camp, pas de SI3, l'ombre ne se declenche jamais.
    lanceur="$HERE/run_si.sh"; mode_full=1
    [ -x "$lanceur" ] || { lanceur="$HERE/run.sh"; mode_full=0; rouge "  run_si.sh absent : run.sh nu (sans pont, le mobile ne campera pas)"; }
    gras "  ombre : relance de la chaine complete ($(basename "$lanceur")) avec CALYPSO_SHADOW_LAC=${sv:-<off>} ($lib)"
    logdir=${RUNDIR_OMBRE:-/tmp/c54x-pont}
    : > "$DIR/run_ombre.log"
    # [2026-10-06] eviter l'empilement : tuer tout lanceur fantome (run_si.sh/run.sh) AVANT de relancer,
    # sinon plusieurs chaines se disputent le socket DSP/le shm/le BTS et le mobile n'acquiert jamais
    # (FBSB result=255). On ne tue QUE les lanceurs, pas ce shell.
    for q in $(pgrep -f 'run_si\.sh|c54x_exe/run\.sh' 2>/dev/null); do [ "$q" = "$$" ] || kill "$q" 2>/dev/null; done
    ( cd "$HERE" && ./run.sh --stop ) >/dev/null 2>&1; sleep 1
    if [ "$mode_full" = 1 ]; then
        ( cd "$HERE" && CALYPSO_SHADOW_LAC="$sv" CALYPSO_SHADOW_SB="${OMBRE_SB:-}" setsid "$lanceur" --secondes "${OMBRE_SECS:-300}" ) > "$DIR/run_ombre.log" 2>&1 &
    else
        ( cd "$HERE" && ./run.sh --stop ) >/dev/null 2>&1
        ( cd "$HERE" && env MODE=dsp PONT=1 IQ=none CALYPSO_BSP_DIRECT_FEED=1 CALYPSO_PONT_LOCKSTEP=1 \
            CALYPSO_SHADOW_LAC="$sv" CALYPSO_SHADOW_SB="${OMBRE_SB:-}" setsid ./run.sh ) > "$DIR/run_ombre.log" 2>&1 &
    fi
    echo "  banc en relance (BTS + echafaudage + pont ; journal $DIR/run_ombre.log)..."
    i=0; annonce=""
    while [ $i -lt ${OMBRE_ATTENTE:-50} ]; do
        [ -z "$sv" ] && { [ -S /tmp/calypso_dsp.sock ] && { echo "  banc relance (ombre off)"; return 0; }; sleep 1; i=$((i+1)); continue; }
        annonce=$(grep -a "ombre-dl.*LAC SI3 force a $sv" "$logdir/dsp.log" 2>/dev/null | tail -1)
        [ -n "$annonce" ] && break
        sleep 1; i=$((i+1))
    done
    [ -n "$annonce" ] && vert "  ombre active : ${annonce#  }" || { echo "  (ombre pas encore annoncee apres ${OMBRE_ATTENTE:-50}s ; voir $DIR/run_ombre.log et $logdir/dsp.log)"; return 1; }
}
cmd_statut() { (cd "$HERE" && lancer ./run.sh --status); }

# ---------------------------------------------------------------- REPL
aide() {
    cat <<'FIN'
  cote P (programme)                          cote D (donnees)
    pas <insn|mots>   executer, garder l'etat     reg [REG val]   voir / fixer un registre
    essai <insn>      executer sans garder        d <adr> [n]     lire la memoire donnees
    p <adr> [n]       desassembler la ROM         d! <adr> <mots> poser des mots en D
    p! <adr> <insn>   poser des mots en P         drom <adr> [n]  lire la ROM donnees
    asm <insn>        assembler seulement         etat | raz      tout afficher | etat a zero
    dis <mots hex>    desassembler des mots
    cours [N]         courir dans la ROM jusqu'a arret / IDLE / N pas (etat du shell)
    arret [adr..|-]   points d'arret de « cours »
  DSP EN MARCHE (c54x_exe --arm)              ARM (firmware layer1 sous gdb, telnet 44444)
    stop              pause du DSP vivant + adresses   demo [tout|arm|dsp]  la main sur l'ARM (gdb) puis le DSP
    go                reprise                          gdb <cmd>   une commande du panneau (fn, l1, help_osmo)
    dsp               instantane sans arreter                      (demo arm etat|trame|hello|go : etape par etape)
    sb                DEAD BEEF dans a_sch (SB)        si3 [id]    relabellise le cell_id de SI3 (defaut 777)
    lac [id]          force le LAC du SI3 EN VIF (shm, sans relance ; course avec le DSP)
    ombre [id|off]    relance le banc, LAC du SI3 force a la SOURCE (deterministe ; mobile.log CGI 3e groupe)
    d@ <adr|nom|W0..R1> [n]  API RAM decodee     d@! <adr|nom> <mots>  y ecrire     d@ noms [motif]
  run                                            divers
    rejeu [trames]    c54x_exe --rejouer           journal [n]     derniers pas
    banc [test|--all] dsp_tester                   log [n]         avertissements du coeur
    isa [-k MOT]      isa_test SPRU172C            sauve|charge <f> etat sur disque
    statut            run.sh --status              menu | aide | q
  adresses : 1234, 0x1234, 1:8000 (page 1).  insn : mnemonique TI (LD #0x1234, A) ou mots hex (f020 1234).
FIN
}
dispatch() { # dispatch <ligne de commande>
    c=$1; shift
    case $c in
        pas) cmd_pas "$@" ;;      essai) cmd_essai "$@" ;;
        p) cmd_p "$@" ;;          'p!') cmd_p_pose "$@" ;;
        d) cmd_d "$@" ;;          'd!') cmd_d_pose "$@" ;;
        'd@') cmd_d_chaud "$@" ;;  'd@!') cmd_d_chaud_ecrire "$@" ;;
        drom) cmd_drom "$@" ;;    asm) cmd_asm "$@" ;;      dis) cmd_dis "$@" ;;
        reg) cmd_reg "$@" ;;      etat) cmd_etat ;;         raz) cmd_raz ;;
        cours) cmd_cours "$@" ;;  arret) cmd_arret "$@" ;;
        stop) cmd_stop ;;         go) cmd_go ;;             dsp) cmd_dsp ;;
        demo) cmd_demo "$@" ;;    gdb) cmd_gdb "$@" ;;
        sb|demo_sb) demo_sb ;;   si3|demo_si3) demo_si3 "$@" ;;   lac|demo_lac) demo_lac "$@" ;;
        demo_dsp) demo_dsp ;;    demo_arm) cmd_gdb demo "$@" ;;
        journal) cmd_journal "$@" ;; log) cmd_log "$@" ;;
        sauve) cmd_sauve "$@" ;;  charge) cmd_charge "$@" ;;
        rejeu) cmd_rejeu "$@" ;;  banc) cmd_banc "$@" ;;    isa) cmd_isa "$@" ;;  statut) cmd_statut ;;
        ombre) cmd_ombre "$@" ;;
        aide|'?'|help) aide ;;
        *) rouge "commande inconnue : $c (aide)"; return 1 ;;
    esac
}
repl() {
    verifier_outils; gras "shell Harvard c54x - « aide » pour les commandes, « menu » pour le menu, « q » pour sortir"
    while :; do
        pc=$(etat_reg PC); if [ -t 1 ]; then printf '\033[1mP:%s D>\033[0m ' "${pc:-9000}"; else printf 'P:%s D> ' "${pc:-9000}"; fi
        read -r ligne || { echo; return 0; }
        set -- $ligne; [ $# -eq 0 ] && continue
        case $1 in q|quit|exit) return 0 ;; menu) return 2 ;; esac
        dispatch "$@"
    done
}

# ---------------------------------------------------------------- menu interactif
pause() { printf '\n  [Entree pour continuer] '; read -r _; }
demander() { # demander <invite> [defaut] -> REP
    printf '  %s' "$1"; [ -n "${2:-}" ] && printf ' [%s]' "$2"; printf ' : '
    read -r REP; [ -z "$REP" ] && REP=${2:-}
}
menu() {
    verifier_outils
    while :; do
        clear 2>/dev/null
        pc=$(etat_reg PC)
        gras "================  SHELL HARVARD - DSP TMS320C54x (Calypso)  ================"
        printf '  PC=%s  A=%s  B=%s  SP=%s   %s\n' "${pc:-9000 (init)}" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg SP)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
        cat <<'FIN'

  ESPACE P (programme)                     ESPACE D (donnees)
   1) executer une instruction (un pas)      5) voir les registres et les mots poses
   2) essayer une instruction (sans garder)  6) fixer un registre
   3) desassembler la ROM programme          7) lire la memoire donnees du coeur
   4) poser des mots en P (sous-routine)     8) poser des mots en D
                                             9) lire la ROM donnees (DROM / PDROM)
  COURIR DANS LA ROM (etat du shell)        DSP EN MARCHE (c54x_exe --arm, /dev/shm)
  10) cours : jusqu'a arret / IDLE / N pas  12) stop : pause du DSP vivant, registres, adresses
  11) arret : points d'arret                13) go : reprise          14) dsp : instantane
                                            15) lire l'API RAM a chaud (adresse ou nom)
                                            16) ecrire dans l'API RAM a chaud
  ARM (firmware layer1 sous gdb, telnet 44444)
  17) demo : la main sur l'ARM puis le DSP  18) gdb : une commande du panneau osmocom
  RUN (les outils du banc, tels quels)      DIVERS
  19) c54x_exe --rejouer (acquisition FB/SB) 23) journal des pas / avertissements du coeur
  20) dsp_tester (liste, un test, --all)     24) etat a zero / sauver / charger
  21) isa_test (exemples SPRU172C, filtre)   r) invite de commandes (REPL)
  22) run.sh --status                        q) quitter
FIN
        demander "choix"
        case $REP in
            1) demander "instruction (mnemonique TI ou mots hex)" "NOP"; cmd_pas $REP; pause ;;
            2) demander "instruction" "NOP"; cmd_essai $REP; pause ;;
            3) demander "adresse (7000.., e000.., 1:8000)" "7000"; a=$REP; demander "nombre de mots" 24; cmd_p "$a" "$REP"; pause ;;
            4) demander "adresse P" "9100"; a=$REP; demander "instruction ou mots"; cmd_p_pose "$a" $REP; pause ;;
            5) cmd_etat; pause ;;
            6) demander "registre" "AR3"; r=$REP; demander "valeur hex" "0"; cmd_reg "$r" "$REP"; pause ;;
            7) demander "adresse D" "0100"; a=$REP; demander "nombre de mots (<=24)" 8; cmd_d "$a" "$REP"; pause ;;
            8) demander "adresse D" "0100"; a=$REP; demander "mots hex (separes par des espaces)"; cmd_d_pose "$a" $REP; pause ;;
            9) demander "adresse (9000.., e000..)" "9000"; a=$REP; demander "nombre de mots" 32; cmd_drom "$a" "$REP"; pause ;;
            10) cmd_arret; demander "nombre de pas max" 200; cmd_cours "$REP"; pause ;;
            11) cmd_arret; demander "arret <adr..> | - (effacer) | vide" ""; [ -n "$REP" ] && cmd_arret $REP; pause ;;
            12) cmd_stop; pause ;;
            13) cmd_go; pause ;;
            14) cmd_dsp; pause ;;
            15) demander "adresse, nom, page W0|W1|R0|R1, ou noms <motif>" "W0"; a=$REP; demander "nombre de mots" 8; case $a in noms*) cmd_d_chaud $a ;; *) cmd_d_chaud "$a" "$REP" ;; esac; pause ;;
            16) demander "adresse ou nom" "d_task_d"; a=$REP; demander "mots hex (separes par des espaces)"; cmd_d_chaud_ecrire "$a" $REP; pause ;;
            17) demander "demo tout | dsp | arm [etat|trame|hello|go]" "tout"; cmd_demo $REP; pause ;;
            18) demander "commande gdb (fn, l1, dsp, help_osmo...)" "help_osmo"; cmd_gdb $REP; pause ;;
            19) demander "trames" 200; t=$REP; demander "verbosite (-v .. -vvvvvv, vide = aucune)" ""; cmd_rejeu "$t" $REP; pause ;;
            20) cmd_banc --list; demander "test (nom, --all, ou vide)" ""; [ -n "$REP" ] && { demander "options (--trace N, -v, --insns N...)" ""; o=$REP; cmd_banc $REP $o; }; pause ;;
            21) demander "filtre -k (vide = tout)" ""; [ -n "$REP" ] && cmd_isa -k "$REP" || cmd_isa; pause ;;
            22) cmd_statut; pause ;;
            23) cmd_journal 20; cmd_log 10; pause ;;
            24) demander "raz | sauve <fichier> | charge <fichier>" "raz"; dispatch $REP; pause ;;
            r|R) repl; [ $? = 2 ] || return 0 ;;
            q|Q) return 0 ;;
            *) ;;
        esac
    done
}

# ---------------------------------------------------------------- point d'entree
case ${1:-menu} in
    menu) menu ;;
    repl) repl ;;
    aide|-h|--help) sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; aide ;;
    *) dispatch "$@" ;;
esac
