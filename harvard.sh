#!/bin/sh
# harvard.sh - "Harvard" shell for the Calypso's TMS320C54x DSP (POSIX sh, dash is enough).
#
# The C54x is a Harvard machine: the P space (program, the opcodes) and the D space (data,
# registers, API RAM) are separate. This shell keeps both side by side:
#   - P side: type an instruction (mnemonic assembled by gas tic54x, or hex words),
#             disassemble the TI ROM (PROM0 at 0x7000, PDROM at 0xE000, pages 1..3 via XPC);
#   - D side: a state (registers + data memory words) that is read, set, and that every
#             executed instruction advances; the DROM (0x9000) and the PDROM can be read.
# Executing an instruction goes through ./isa_test (the real qosmo C54x core, pcb-minimal):
# it is given the "before" state and the words, it returns the "after" state. Nothing is
# simulated here. The "run" commands launch the bench tools as they are (c54x_exe,
# dsp_tester, isa_test, run.sh).
#
#   ./harvard.sh                interactive menu
#   ./harvard.sh repl           direct command prompt
#   ./harvard.sh pas 'LD #0x1234, A'    one command then exit (any REPL command)
#   ./harvard.sh aide
# Variables: HARVARD_DIR (state, default ~/.local/state/harvard-c54x), ROM_DIR (default ./rom),
#            ISA_TEST, OBJDUMP, GAS (tool paths).

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=${HARVARD_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/harvard-c54x}
ROM_DIR=${ROM_DIR:-$HERE/rom}
ISA_TEST=${ISA_TEST:-$HERE/isa_test}
OBJDUMP=${OBJDUMP:-$HERE/build-tic54x/binutils/objdump}
GAS=${GAS:-$HERE/build-tic54x/gas/as-new}
ETAT=$DIR/etat.txt          # one line per item: "REG hex" | "M addr hex" | "P addr hex"
JOURNAL=$DIR/journal.txt
LOG=$DIR/coeur.log          # stderr of the core ([c54x] warnings ...)
REGS="A B T SP PC ST0 ST1 PMST AR0 AR1 AR2 AR3 AR4 AR5 AR6 AR7 BK BRC RSA REA TRN IMR IFR XPC"
SENT_REG=deadbeef1          # 36-bit sentinel: never equal to a register -> isa_test always prints the value got
SENT_MEM=1ffff              # 17-bit sentinel: never equal to a 16-bit word
mkdir -p "$DIR"; : >> "$ETAT"; : >> "$JOURNAL"

# ---------------------------------------------------------------- helpers
rouge() { if [ -t 1 ]; then printf '\033[31m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
vert()  { if [ -t 1 ]; then printf '\033[32m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
gras()  { if [ -t 1 ]; then printf '\033[1m%s\033[0m\n' "$*"; else printf '%s\n' "$*"; fi; }
hex() { # hex <text>: accepts 0x1234, 1234, 1:8000 (page:address); returns the integer
    case $1 in
        *:*) printf '%d' $(( ( ${1%%:*} << 16 ) | 0x${1#*:} )) ;;
        0x*|0X*) printf '%d' $(( $1 )) ;;
        *) printf '%d' $(( 0x$1 )) ;;
    esac 2>/dev/null
}
est_hex_mots() { # true if every argument is a hex word of 1 to 4 digits
    for m in "$@"; do case $m in ''|*[!0-9a-fA-F]*|?????*) return 1;; esac; done; return 0
}
verifier_outils() {
    [ -x "$ISA_TEST" ] || rouge "isa_test missing ($ISA_TEST): make isa_test"
    [ -x "$OBJDUMP" ]  || rouge "objdump tic54x missing ($OBJDUMP): no disassembly"
    [ -x "$GAS" ]      || rouge "gas tic54x missing ($GAS): only hex words are accepted"
}

# ---------------------------------------------------------------- state (D side)
etat_reg() { awk -v r="$1" '$1==r {v=$2} END {print v}' "$ETAT"; }
etat_fixe() { # etat_fixe <key...> <val>: replaces or adds the line "<keys> <val>"
    cle=$1; shift
    [ $# -gt 1 ] && { cle="$cle $1"; shift; }
    { grep -v "^$cle " "$ETAT"; echo "$cle $1"; } > "$ETAT.new" && mv "$ETAT.new" "$ETAT"
}
etat_mem() { awk -v a="$1" '$1=="M" && $2==a {v=$3} END {print v}' "$ETAT"; }
drapeaux() { # drapeaux <st0> <st1>: names of the bits set to 1
    st0=$(( 0x${1:-0} )); st1=$(( 0x${2:-0} )); d=""
    [ $(( st0 >> 12 & 1 )) = 1 ] && d="$d TC";  [ $(( st0 >> 11 & 1 )) = 1 ] && d="$d C"
    [ $(( st0 >> 10 & 1 )) = 1 ] && d="$d OVA"; [ $(( st0 >> 9 & 1 )) = 1 ]  && d="$d OVB"
    [ $(( st1 >> 15 & 1 )) = 1 ] && d="$d BRAF"; [ $(( st1 >> 14 & 1 )) = 1 ] && d="$d CPL"
    [ $(( st1 >> 13 & 1 )) = 1 ] && d="$d XF";  [ $(( st1 >> 12 & 1 )) = 1 ] && d="$d HM"
    [ $(( st1 >> 11 & 1 )) = 1 ] && d="$d INTM"; [ $(( st1 >> 9 & 1 )) = 1 ] && d="$d OVM"
    [ $(( st1 >> 8 & 1 )) = 1 ]  && d="$d SXM"; [ $(( st1 >> 7 & 1 )) = 1 ]  && d="$d C16"
    [ $(( st1 >> 6 & 1 )) = 1 ]  && d="$d FRCT"; [ $(( st1 >> 5 & 1 )) = 1 ] && d="$d CMPT"
    printf 'ARP=%d DP=%03x ASM=%02x flags:%s' $(( st0 >> 13 )) $(( st0 & 0x1ff )) $(( st1 & 0x1f )) "${d:- (none)}"
}
montrer_regs() {
    [ -s "$ETAT" ] || { echo "(empty state: registers hold the core's init values; run a step)"; return; }
    printf '  A  =%12s   B  =%12s   T  =%6s   PC =%6s   SP =%6s   XPC=%4s\n' \
        "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(etat_reg PC)" "$(etat_reg SP)" "$(etat_reg XPC)"
    printf '  AR0=%6s AR1=%6s AR2=%6s AR3=%6s AR4=%6s AR5=%6s AR6=%6s AR7=%6s\n' \
        "$(etat_reg AR0)" "$(etat_reg AR1)" "$(etat_reg AR2)" "$(etat_reg AR3)" "$(etat_reg AR4)" "$(etat_reg AR5)" "$(etat_reg AR6)" "$(etat_reg AR7)"
    printf '  ST0=%6s ST1=%6s PMST=%5s BK=%6s BRC=%6s RSA=%6s REA=%6s TRN=%6s IMR=%5s IFR=%5s\n' \
        "$(etat_reg ST0)" "$(etat_reg ST1)" "$(etat_reg PMST)" "$(etat_reg BK)" "$(etat_reg BRC)" "$(etat_reg RSA)" "$(etat_reg REA)" "$(etat_reg TRN)" "$(etat_reg IMR)" "$(etat_reg IFR)"
    printf '  %s\n' "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}
montrer_mem() {
    n=$(grep -c '^M ' "$ETAT"); [ "$n" = 0 ] && { echo "  D: no word set"; return; }
    echo "  D (words set / read, $n):"; awk '$1=="M" {printf "   [%04x]=%04x", strtonum("0x"$2), strtonum("0x"$3); if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT" 2>/dev/null \
      || awk '$1=="M" {printf "   [%s]=%s", $2, $3; if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT"
    n=$(grep -c '^P ' "$ETAT"); [ "$n" != 0 ] && { echo "  P (words set, $n):"; awk '$1=="P" {printf "   [%s]=%s", $2, $3; if (++k%6==0) print ""} END {if (k%6) print ""}' "$ETAT"; }
}
cmd_etat() { gras "--- registers (D side)"; montrer_regs; montrer_mem; }
cmd_raz() { : > "$ETAT"; echo "state reset (core init values at the next step)"; }
cmd_reg() {
    [ $# -lt 2 ] && { montrer_regs; return; }
    r=$(printf '%s' "$1" | tr a-z A-Z); v=$(hex "$2") || { rouge "invalid value"; return 1; }
    etat_fixe "$r" "$(printf '%x' "$v")"; echo "$r <- $(printf '%x' "$v")"
}

# ---------------------------------------------------------------- assembly / disassembly (P side)
asm() { # asm <instruction> -> hex words on stdout, 1 on failure
    if est_hex_mots $1; then printf '%s\n' "$*" | tr A-F a-f; return 0; fi
    [ -x "$GAS" ] || { rouge "no assembler: give the hex words"; return 1; }
    printf '\t.text\n\t%s\n' "$*" > "$DIR/a.s"
    if ! "$GAS" -o "$DIR/a.o" "$DIR/a.s" 2> "$DIR/a.err"; then
        rouge "assembly rejected:"; sed 's/^/    /' "$DIR/a.err"; return 1
    fi
    "$OBJDUMP" -d "$DIR/a.o" | sed -n 's/^ *[0-9a-f]*:[[:space:]]*\([0-9a-f]\{4\}\).*/\1/p' | tr '\n' ' ' | sed 's/ $/\n/'
}
dis_mots() { # dis_mots <address> <hex words...>: disassembles words
    a=$1; shift; : > "$DIR/d.bin"
    for m in "$@"; do v=$(( 0x$m )); printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" >> "$DIR/d.bin"; done
    "$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma="0x${a:-9000}" "$DIR/d.bin" | sed -n 's/^ *\([0-9a-f]*:.*\)/    \1/p'
}
rom_fichier() { # rom_fichier <linear P or D addr> <P|D> -> "file base_words"
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
cmd_p() { # p <address> [n words]: disassemble the program ROM
    [ $# -ge 1 ] || { echo "usage: p <address> [n]   (0x7000.., 0xe000.., 1:8000 = page 1)"; return 1; }
    a=$(hex "$1") || { rouge "invalid address"; return 1; }; n=${2:-16}
    set -- $(rom_fichier "$a" P) || { rouge "$(printf '%x' "$a"): outside program ROM (PROM0 7000-dfff, PDROM e000-ffff, pages 1-3: 8000-ffff)"; return 1; }
    f=$1; base=$2; [ -r "$f" ] || { rouge "ROM missing: $f (rom/fetch-rom.sh)"; return 1; }
    page=$(( a >> 16 )); off=$(( a & 0xffff )); vma=$(( base & 0xffff ))
    [ $page -gt 0 ] && gras "  page $page (XPC=$page), file $(basename "$f")"
    "$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma=$vma --start-address=$off --stop-address=$(( off + n )) "$f" \
        | sed -n 's/^ *\([0-9a-f]*:.*\)/    \1/p'
}
cmd_drom() { # drom <address> [n]: read the data ROM
    [ $# -ge 1 ] || { echo "usage: drom <address> [n]   (DROM 9000-dfff, PDROM e000-ffff)"; return 1; }
    a=$(hex "$1") || { rouge "invalid address"; return 1; }; n=${2:-16}
    set -- $(rom_fichier "$a" D) || { rouge "$(printf '%x' "$a"): outside data ROM"; return 1; }
    f=$1; base=$2; [ -r "$f" ] || { rouge "ROM missing: $f"; return 1; }
    od -An -v -tx2 -w16 -j $(( (a - base) * 2 )) -N $(( n * 2 )) "$f" | awk -v a=$a '{printf "    [%04x]", a; print; a+=8}'
}
cmd_asm() { m=$(asm "$@") || return 1; printf '    words: %s\n' "$m"; dis_mots "$(etat_reg PC)" $m; }
cmd_dis() { est_hex_mots "$@" || { echo "usage: dis <hex words>"; return 1; }; dis_mots "$(etat_reg PC)" "$@"; }

# ---------------------------------------------------------------- execution of one instruction (P -> D)
surveiller() { # surveiller <word1> [word2]: D addresses to read at each step (those a single instruction can touch)
    w1=$(( 0x$1 )); st0=$(etat_reg ST0); sp=$(etat_reg SP)
    {
        for rg in AR0 AR1 AR2 AR3 AR4 AR5 AR6 AR7; do v=$(etat_reg $rg); [ -n "$v" ] || continue
            v=$(( 0x$v )); printf '%04x\n%04x\n%04x\n' $(( (v - 1) & 0xffff )) $v $(( (v + 1) & 0xffff )); done
        [ -n "$sp" ] && { v=$(( 0x$sp )); printf '%04x\n%04x\n%04x\n' $(( (v - 1) & 0xffff )) $v $(( (v + 1) & 0xffff )); }
        printf '%04x\n' $(( ((0x${st0:-0} & 0x1ff) << 7) | (w1 & 0x7f) ))      # direct addressing DP:dma
        [ $# -ge 2 ] && printf '%04x\n' $(( 0x$2 & 0xffff ))                     # long address lk
    } | sort -u > "$DIR/cand.txt"; awk '$1=="M" {print $2}' "$ETAT" > "$DIR/connu.txt"; grep -v -F -x -f "$DIR/connu.txt" "$DIR/cand.txt" 2>/dev/null || cat "$DIR/cand.txt"
}
executer() { # executer <keep 0|1> <title> <hex words...> [D addresses to read...]  ->  $DIR/pas.apres: lines "NAME value"
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
    grep -q '^FAIL' "$DIR/pas.out" || { rouge "isa_test returned nothing:"; cat "$DIR/pas.out"; return 1; }
    grep -q '\[[0-9a-f ]*?\]' "$DIR/pas.out" && rouge "  (a register of the state is unknown to isa_test, ignored)"
    # "  A: attendu deadbeef1, obtenu 12  data[0100]: attendu 1ffff, obtenu 0000" -> "A 12" / "M 0100 0000"
    # (the two words are matched as [a-z]*: works with isa_test's French "attendu/obtenu" or English "expected/got")
    sed -n 's/^FAIL[^]]*\]//p' "$DIR/pas.out" | sed 's/  /\n/g' | sed -n \
        's/^data\[\([0-9a-f]*\)\]: [a-z]* [0-9a-f]*, [a-z]* \([0-9a-f]*\)$/M \1 \2/p;
         s/^prog\[\([0-9a-f]*\)\]: [a-z]* [0-9a-f]*, [a-z]* \([0-9a-f]*\)$/P \1 \2/p;
         s/^\([A-Z][A-Z0-9]*\): [a-z]* [0-9a-f]*, [a-z]* \([0-9a-f]*\)$/\1 \2/p' > "$DIR/pas.apres"
    if [ "$garder" = 1 ]; then
        # kept: the registers, the P words set, the D words already in the state, and any D word that became non-zero
        { grep '^P ' "$ETAT"
          awk 'FILENAME==ARGV[1] { if ($1=="M") connu[$2]=1; next }
               $1=="P" { next } $1!="M" { print; next } ($2 in connu) || ($3 != "0000" && $2 >= "0060") { print }' "$ETAT" "$DIR/pas.apres"
        } > "$ETAT.new" && mv "$ETAT.new" "$ETAT"
        printf '%s  %s  | %s\n' "$(date +%H:%M:%S)" "$mots" "$titre" >> "$JOURNAL"
    fi
}
diff_etat() { # diff_etat <before> <after>: what changed
    awk 'FILENAME==ARGV[1] { if ($1=="M"||$1=="P") av[$1" "$2]=$3; else av[$1]=$2; next }
         { if ($1=="M"||$1=="P") { k=$1" "$2; v=$3 } else { k=$1; v=$2 }
           if (!(k in av)) { if (k !~ /^M |^P /) nouveaux=nouveaux" "k"="v; else if (v != "0000") printf "    %-10s %12s -> %s\n", k, "(new)", v }
           else if (av[k] != v) printf "    %-10s %12s -> %s\n", k, av[k], v }
         END { if (nouveaux != "") print "    (first read:" nouveaux ")" }' "$1" "$2"
}
cmd_pas() { # pas <instruction | hex words>
    [ $# -ge 1 ] || { echo "usage: pas <instruction>   e.g. pas LD #0x1234, A   |   pas f020 1234"; return 1; }
    mots=$(asm "$@") || return 1
    pc=$(etat_reg PC); printf '  P[%s] %s   <- %s\n' "${pc:-9000}" "$mots" "$*"
    cp "$ETAT" "$DIR/avant.txt"
    executer 1 "$*" "$mots" || return 1
    gras "  changes:"; diff_etat "$DIR/avant.txt" "$ETAT" | grep . || echo "    (no register or word set changed)"
    printf '  PC=%s  A=%s  B=%s  T=%s  %s\n' "$(etat_reg PC)" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}
cmd_essai() { # essai <instruction>: execute without keeping the state
    [ $# -ge 1 ] || { echo "usage: essai <instruction>"; return 1; }
    mots=$(asm "$@") || return 1
    executer 0 "$*" "$mots" || return 1
    gras "  result (state NOT kept):"; diff_etat "$ETAT" "$DIR/pas.apres" | grep . || echo "    (nothing changes)"
}
cmd_d() { # d <address> [n]: read the core's data memory (through a NOP that is not kept)
    [ $# -ge 1 ] || { echo "usage: d <address> [n<=24]"; return 1; }
    a=$(hex "$1") || { rouge "invalid address"; return 1; }; n=${2:-8}; [ $n -gt 24 ] && n=24
    liste=""; i=0; while [ $i -lt $n ]; do liste="$liste $(printf '%04x' $(( a + i )))"; i=$(( i + 1 )); done
    executer 0 "D read" f495 $liste || return 1
    awk -v a=$a 'BEGIN{k=0} $1=="M" { lu[$2]=$3 }
         END { for (i=0;i<'"$n"';i++) { k=sprintf("%04x", a+i); printf "    [%s]=%s", k, (k in lu)?lu[k]:"????"; if ((i+1)%8==0) print "" } if (i%8) print "" }' "$DIR/pas.apres"
    st0=$(etat_reg ST0); echo "    (read by the core with the current state; DP=$(printf '%03x' $(( 0x${st0:-0} & 0x1ff ))); words not set read 0 except mapped registers 0000-005f)"
}
cmd_d_pose() { # d! <address> <words...>: set words in D
    [ $# -ge 2 ] || { echo "usage: d! <address> <word> [word...]"; return 1; }
    a=$(hex "$1") || { rouge "invalid address"; return 1; }; shift
    for m in "$@"; do v=$(hex "$m") || { rouge "invalid word: $m"; return 1; }
        etat_fixe M "$(printf '%04x' $a)" "$(printf '%04x' $(( v & 0xffff )))"; printf '    D[%04x] <- %04x\n' $a $(( v & 0xffff )); a=$(( a + 1 )); done
}
cmd_p_pose() { # p! <address> <instruction | words>: set words in P (preloaded at each step)
    [ $# -ge 2 ] || { echo "usage: p! <address> <instruction|hex words>   (e.g. a subroutine called by CALL)"; return 1; }
    a=$(hex "$1") || { rouge "invalid address"; return 1; }; shift
    mots=$(asm "$@") || return 1
    for m in $mots; do etat_fixe P "$(printf '%04x' $a)" "$m"; printf '    P[%04x] <- %s\n' $a "$m"; a=$(( a + 1 )); done
}
cmd_journal() { gras "--- step log"; tail -n ${1:-30} "$JOURNAL"; }
cmd_log() { gras "--- core warnings (last lines)"; tail -n ${1:-20} "$LOG"; }
cmd_sauve() { [ $# -ge 1 ] || { echo "usage: sauve <file>"; return 1; }; cp "$ETAT" "$1" && echo "state -> $1"; }
cmd_charge() { [ $# -ge 1 ] && [ -r "$1" ] || { echo "usage: charge <file>"; return 1; }; cp "$1" "$ETAT" && echo "state <- $1" && montrer_regs; }

# ---------------------------------------------------------------- cours / arret: run inside the ROM (shell state)
# "arret" sets breakpoints (linear P addresses, 1:8000 = page 1); "cours" starts from the current PC
# (ff80 = reset vector if the state is empty), reads each instruction from the ROM (or from the P words
# set, which take priority), executes it through isa_test like "pas", and stops on: a breakpoint,
# IDLE, an address outside the ROM (RAM without a word set), a PC that does not move, or the step budget.
# All of this acts on the SHELL STATE, never on the running c54x_exe (for that: stop / go / dsp).
# Limits (the core restarts from a "registers + words" state at each step): the delay slot of a
# delayed branch (BD, CALLD, RETD...) and the counter of an RPT do not survive from one step to the next.
ARRETS=$DIR/arrets.txt; : >> "$ARRETS"
cmd_arret() { # arret [addr...] | arret -: set / list / clear the breakpoints
    if [ $# -eq 0 ]; then
        [ -s "$ARRETS" ] && { gras "--- breakpoints (P addresses)"; sed 's/^/    /' "$ARRETS"; } || echo "  no breakpoint (arret <addr> [addr...])"; return
    fi
    [ "$1" = - ] && { : > "$ARRETS"; echo "  breakpoints cleared"; return; }
    for s in "$@"; do a=$(hex "$s") || { rouge "invalid address: $s"; return 1; }
        a=$(printf '%x:%04x' $(( a >> 16 )) $(( a & 0xffff ))); grep -q -x -F "$a" "$ARRETS" || echo "$a" >> "$ARRETS"; echo "    breakpoint at $a"; done
}
rom_insn() { # rom_insn <linear P> -> "hex words|mnemonic"; P words set take priority; 1 if outside the ROM
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
cmd_cours() { # cours [N steps]: run from the current PC until a stop
    n=${1:-200}; case $n in ''|*[!0-9]*) echo "usage: cours [max number of steps]"; return 1;; esac
    pc=$(etat_reg PC); xpc=$(etat_reg XPC)
    [ -n "$pc" ] || { etat_fixe PC ff80; etat_fixe XPC 0; pc=ff80; xpc=0; echo "  (empty state: start at reset vector ff80)"; }
    gras "  run from $(printf '%x:%04x' $(( 0x${xpc:-0} )) $(( 0x$pc )))  (budget $n steps; breakpoints: $(tr '\n' ' ' < "$ARRETS"))"
    : > "$DIR/cours_adr.txt"; k=0; motif=""
    while :; do
        pc=$(etat_reg PC); xpc=$(etat_reg XPC); lin=$(( (0x${xpc:-0} << 16) | 0x$pc )); cle=$(printf '%x:%04x' $(( lin >> 16 )) $(( lin & 0xffff )))
        if [ $k -gt 0 ] && grep -q -x -F "$cle" "$ARRETS"; then motif="breakpoint $cle"; break; fi
        [ $k -lt $n ] || { motif="budget of $n steps used up"; break; }
        r=$(rom_insn $lin) || { motif="$cle outside program ROM (RAM with no word set)"; break; }
        mots=${r%%|*}; mn=${r#*|}
        printf '%s\n' "$cle" >> "$DIR/cours_adr.txt"
        case $mn in idle*) printf '    %-7s %-14s %s\n' "$cle" "$mots" "$mn"; motif="IDLE at $cle"; k=$(( k + 1 )); break ;; esac
        executer 1 "cours $cle $mn" "$mots" || { motif="isa_test failed at $cle"; break; }
        k=$(( k + 1 )); pc2=$(etat_reg PC); xpc2=$(etat_reg XPC)
        printf '    %-7s %-14s %-28s -> %x:%04x  A=%s B=%s\n' "$cle" "$mots" "$mn" $(( 0x${xpc2:-0} )) $(( 0x${pc2:-0} )) "$(etat_reg A)" "$(etat_reg B)"
        [ "$pc2" = "$pc" ] && [ "${xpc2:-0}" = "${xpc:-0}" ] && { motif="PC stuck at $cle (loop in place / instruction not executed)"; break; }
    done
    gras "  stop: $motif  ($k steps)"
    gras "  addresses visited ($(sort -u "$DIR/cours_adr.txt" | wc -l) distinct, in order):"
    awk '!(seen[$0]++)' "$DIR/cours_adr.txt" | tr '\n' ' ' | fold -s -w 96 | sed 's/^/    /'; echo
    printf '  PC=%s  A=%s  B=%s  T=%s  %s\n' "$(etat_reg PC)" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg T)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
}

# ---------------------------------------------------------------- RUNNING DSP: stop / go / dsp on the c54x_exe --arm
# Here the LIVE PROCESS (the one run.sh launched) is acted upon, never the shell state:
#   stop   pauses the DSP. Binary with src/pont.c SIGUSR1 support: cooperative pause at the next frame
#          boundary (DONE returned, DSP in IDLE, the ARM waits for its next DONE); otherwise: SIGSTOP.
#          In both cases nothing is written into the API RAM or the core: the firmware only sees a
#          longer frame. The registers are read from the process memory (/proc/PID/mem, C54xState
#          offsets given by gdb on the executable), and THE ADDRESSES are shown: PC (where the DSP
#          stopped, IDLE) and the armed vectors (where it will resume: IPTR of PMST, frame IT vec 28 = bit 12).
#   go     resumes (SIGUSR2 or SIGCONT), then a snapshot 0.3 s later to see that it resumed.
#   dsp    snapshot without stopping (read while it runs: consistent to within one instruction).
PAUSE_FICHIER=${PAUSE_FICHIER:-/dev/shm/calypso_dsp_pause}
VIVANT=$DIR/vivant.txt
IRQ_NOMS="12=IT_trame_TPU(vec28) 14=DMA_INT10n(vec30)"   # c54x_irq.c:44-60; the other bits: not named here
vivant_pid() { # the c54x_exe --arm: comm == c54x_exe (readable by all, even if the process is root's) + cmdline --arm
    for p in $(pgrep -x c54x_exe 2>/dev/null); do
        [ "$(cat /proc/$p/comm 2>/dev/null)" = c54x_exe ] || continue
        grep -qa -- --arm /proc/$p/cmdline 2>/dev/null && { echo $p; return 0; }
    done; return 1
}
vivant_accessible() { # vivant_accessible <pid>: can this account drive this process (signal + /proc/mem)?
    kill -0 "$1" 2>/dev/null && [ -r "/proc/$1/mem" ]
}
vivant_diag() { # clear message when the bench runs under another account
    pid=$1; u=$(stat -c %U /proc/$pid 2>/dev/null); moi=$(id -un)
    rouge "the c54x_exe bench (pid $pid) runs as '${u:-?}', you are '$moi'."
    rouge "  -> stop/go/dsp need the same account. Start harvard like this:  sudo ./harvard.sh"
    rouge "     (or restart the bench under your account). Read only (d@) works, write (d@!) does not."
}
vivant_coop() { grep -q -a calypso_dsp_pause /proc/$1/exe 2>/dev/null; }
vivant_decalages() { # vivant_decalages <pid> -> file "field offset" (gdb on the executable, once per pid)
    f=$DIR/decalages.$1.txt
    if ! [ -s "$f" ]; then
        command -v gdb > /dev/null || { rouge "gdb missing: cannot locate C54xState in the process" >&2; return 1; }
        { for c in data prog pc xpc idle running a b ar t sp st0 st1 pmst imr ifr bk brc rsa rea trn insn_count last_exec_pc last_exec_op; do
              printf 'printf "%s %%ld\\n", (long)&((C54xState*)0)->%s\n' "$c" "$c"; done
          printf 'printf "FN %%ld\\n", (long)&g_c54x_exe_fn\n'; } > "$DIR/decal.gdb"
        gdb -batch -q -nx -ex 'set debuginfod enabled off' -x "$DIR/decal.gdb" /proc/$1/exe 2>/dev/null | grep -E '^[a-zA-Z_0-9]+ [0-9]+$' > "$f"
        [ "$(wc -l < "$f")" -ge 25 ] || { rm -f "$f"; rouge "gdb returned no offsets (executable built without -g?)" >&2; return 1; }
    fi
    echo "$f"
}
vivant_lire() { # vivant_lire <pid> <label>: core registers of the process -> $VIVANT (lines "KEY val")
    pid=$1; d=$(vivant_decalages $pid) || return 1
    shm=$(awk '/calypso_api_ram/ {print $1; exit}' /proc/$pid/maps); [ -n "$shm" ] || { rouge "no API RAM window in process $pid"; return 1; }
    base=$(( 0x${shm%%-*} - 0x1000 - $(awk '$1=="data" {print $2}' "$d") ))      # data[0x0800] is mapped on /dev/shm
    exe=$(awk '$6 ~ /c54x_exe/ && $3=="00000000" {print $1; exit}' /proc/$pid/maps); exe=$(( 0x${exe%%-*} ))
    dec() { awk -v k="$1" '$1==k {print $2}' "$d"; }
    lire() { dd if=/proc/$pid/mem bs=1 skip=$1 count=$2 2>/dev/null | od -An -v -tx$2 | tr -d ' \n'; }   # little-endian -> hex
    echo "$pid $base $(dec prog) $(dec data) $(( 0x$(lire $(( base + $(dec pmst) )) 2) ))" > "$DIR/vivant_prog.txt"   # for vivant_prog
    pc=$(lire $(( base + $(dec pc) )) 4) || return 1
    [ -n "$pc" ] || { rouge "read of /proc/$pid/mem denied (ptrace)"; return 1; }
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
vivant_prog() { # vivant_prog <addr> <n>: n words of the live process's program memory, like prog_fetch
    set -- $(cat "$DIR/vivant_prog.txt") "$1" "$2"      # pid base prog data pmst adr n
    # c54x_mem.c prog_fetch: PMST.OVLY (bit 5) and 0x60 <= adr < 0x2800 -> data[adr] (overlaid DARAM), else prog[adr]
    if [ $(( $5 & 0x20 )) != 0 ] && [ $6 -ge 96 ] && [ $6 -lt 10240 ]; then o=$4; else o=$3; fi
    dd if=/proc/$1/mem bs=1 skip=$(( $2 + o + $6 * 2 )) count=$(( $7 * 2 )) 2>/dev/null | od -An -v -tx2 | tr -s ' \n' ' '
}
montrer_vivant() { # montrer_vivant <pid>: $VIVANT on screen, with the stop / go addresses
    v() { awk -v k="$1" '$1==k {print substr($0, length($1) + 2)}' "$VIVANT"; }
    pc=$(v PC); xpc=$(v XPC); idle=$(v IDLE); pmst=$(v PMST); imr=$(v IMR); ifr=$(v IFR)
    gras "  live DSP (pid $1, $(vivant_coop $1 && echo 'cooperative SIGUSR1 pause available' || echo 'old binary: SIGSTOP/SIGCONT')): $(v ETAT)"
    printf '  fn=%s  PC=%x:%s  idle=%s running=%s  insn=%s  last executed instruction: P[%s]=%s\n' "$(v FN)" $(( 0x$xpc )) "$pc" "$idle" "$(v RUNNING)" "$(v INSN)" "$(v LAST_PC)" "$(v LAST_OP)"
    ( ETAT=$VIVANT; montrer_regs )
    iptr=$(( 0x$pmst >> 7 )); vbase=$(( iptr << 7 ))
    gras "  addresses of this stop / go:"
    if [ "$idle" = 1 ]; then printf '    stop: PC=%x:%s  IDLE, the DSP waits for an interrupt (INTM=%d)\n' $(( 0x$xpc )) "$pc" $(( 0x${v_st1:-$(v ST1)} >> 11 & 1 ))
    else printf '    stop: PC=%x:%s  (executing, not in IDLE)\n' $(( 0x$xpc )) "$pc"; fi
    printf '    go  : armed vectors (IMR=%04x IFR=%04x, IPTR=%03x -> table at %04x):\n' $(( 0x$imr )) $(( 0x$ifr )) $iptr $vbase
    b=0; while [ $b -lt 16 ]; do
        if [ $(( 0x$imr >> b & 1 )) = 1 ]; then
            vec=$(( b + 16 )); adr=$(( vbase + vec * 4 )); nom=$(printf '%s\n' $IRQ_NOMS | awk -F= -v k=$b '$1==k {print $2}')
            cible=$(dis_mots "$(printf '%04x' $adr)" $(vivant_prog $adr 2) | head -1 | sed 's/^ *[0-9a-f]*:[[:space:]]*//; s/[[:space:]]\+/ /g')
            printf '           bit %2d vec %2d -> %04x %-22s %s%s\n' $b $vec $adr "${nom:-?}" "${cible:+: $cible}" "$([ $(( 0x$ifr >> b & 1 )) = 1 ] && echo '   <- PENDING (IFR)')"
        fi; b=$(( b + 1 )); done
    lin=$(( (0x$xpc << 16) | 0x$pc ))
    if rom_fichier $lin P > /dev/null 2>&1; then gras "  ROM at PC:"; cmd_p "$(printf '%x:%04x' $(( lin >> 16 )) $(( lin & 0xffff )))" 4; fi
}
cmd_dsp() { # dsp: snapshot of the running DSP, without stopping it
    pid=$(vivant_pid) || { rouge "no c54x_exe --arm running (./run.sh)"; return 1; }
    vivant_accessible "$pid" || { vivant_diag "$pid"; return 1; }
    vivant_lire $pid "snapshot without stopping$([ -f "$DIR/pause.$pid" ] && echo ' (paused: go to resume)')" || return 1; montrer_vivant $pid
}
cmd_stop() { # stop: pause the running DSP
    pid=$(vivant_pid) || { rouge "no c54x_exe --arm running (./run.sh)"; return 1; }
    vivant_accessible "$pid" || { vivant_diag "$pid"; return 1; }
    [ -f "$DIR/pause.$pid" ] && { echo "  already paused ($(cat "$DIR/pause.$pid")): go to resume"; cmd_dsp; return; }
    if vivant_coop $pid; then
        rm -f "$PAUSE_FICHIER"; kill -USR1 $pid || return 1; i=0
        while [ $i -lt 50 ] && ! grep -q '^ETAT pause' "$PAUSE_FICHIER" 2>/dev/null; do sleep 0.1; i=$(( i + 1 )); done
        grep -q '^ETAT pause' "$PAUSE_FICHIER" 2>/dev/null || { rouge "no pause after 5 s (is the DSP waiting for a GO from the ARM?); kill -USR2 $pid to cancel"; return 1; }
        mode="SIGUSR1 (frame boundary, fn=$(awk '$1=="FN"{print $2}' "$PAUSE_FICHIER"))"
    else
        kill -STOP $pid || return 1; mode="SIGSTOP"
    fi
    echo "$mode" > "$DIR/pause.$pid"
    vivant_lire $pid "PAUSED by $mode" || return 1; montrer_vivant $pid
    echo "  (the ARM/QEMU waits for the DSP's DONE: the whole chain is suspended; 'go' to resume)"
}
cmd_go() { # go: resume the running DSP
    pid=$(vivant_pid) || { rouge "no c54x_exe --arm running"; rm -f "$DIR"/pause.*; return 1; }
    [ -f "$DIR/pause.$pid" ] || { echo "  no pause in progress (stop first)"; return 1; }
    mode=$(cat "$DIR/pause.$pid"); rm -f "$DIR/pause.$pid"
    case $mode in SIGUSR1*) kill -USR2 $pid; i=0; while [ $i -lt 30 ] && ! grep -q '^ETAT reprise' "$PAUSE_FICHIER" 2>/dev/null; do sleep 0.1; i=$(( i + 1 )); done ;;
                  *) kill -CONT $pid ;; esac
    avant_insn=$(awk '$1=="INSN"{print $2}' "$VIVANT" 2>/dev/null); avant_fn=$(awk '$1=="FN"{print $2}' "$VIVANT" 2>/dev/null)
    case $mode in SIGUSR1*) reprise="SIGUSR2" ;; *) reprise="SIGCONT" ;; esac
    echo "  go: resumed with $reprise (the pause was $mode)"; sleep 0.3
    vivant_lire $pid "0.3 s after resuming" || return 1
    v() { awk -v k="$1" '$1==k {print $2}' "$VIVANT"; }
    printf '  0.3 s later: fn=%s (+%d frames)  PC=%x:%s idle=%s  insn=%s (+%d)\n' "$(v FN)" $(( $(v FN) - ${avant_fn:-0} )) $(( 0x$(v XPC) )) "$(v PC)" "$(v IDLE)" "$(v INSN)" $(( $(v INSN) - ${avant_insn:-0} ))
}

# ---------------------------------------------------------------- ARM: the layer1 firmware under gdb (telnet 44444)
# run.sh launches QEMU with -gdb tcp::1234 and the console tools/gdb-telnet.py on port 44444: a
# gdb-multiarch attached to the firmware (layer1.highram.elf symbols), cmd.gdb panel + "demo".
# One session at a time (the gdbstub only accepts one client): if a telnet is already open,
# nothing is forced, the command to type there is shown.
GDB_TELNET=${GDB_TELNET:-44444}
telnet_occupe() { ss -Htn state established "( sport = :$GDB_TELNET )" 2>/dev/null | grep -q . ; }
cmd_gdb() { # gdb <command...>: send a command to the ARM's gdb through the telnet, show the output
    [ $# -ge 1 ] || { echo "usage: gdb <osmocom panel command>   e.g. gdb fn | gdb l1 | gdb help_osmo | gdb demo etat"; return 1; }
    ss -Hltn "( sport = :$GDB_TELNET )" 2>/dev/null | grep -q . || { rouge "nothing listens on port $GDB_TELNET (run.sh with GDB=1)"; return 1; }
    if telnet_occupe; then rouge "a telnet session is already open on port $GDB_TELNET (one at a time): type in that telnet:   $*"; return 1; fi
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
lire(b"(gdb) ", 20)                                   # gdb attached, target resumed (continue &)
s.sendall(b"\x03"); lire(b"(gdb) ", 5); time.sleep(0.5)   # Ctrl-C: the target stops, the prompt comes back
s.sendall(cmd.encode() + b"\necho __FIN__\\n\n")
out = lire(b"__FIN__", attente)
s.sendall(b"quit\n"); time.sleep(0.3); s.close()      # on close, the console detaches gdb: the ARM resumes
txt = re.sub(rb"\xff[\xfb-\xfe].|\xff\xfa.*?\xff\xf0", b"", out).decode("utf-8", "replace").replace("\r", "")
txt = txt.split("__FIN__")[0]
print("\n".join("    " + l for l in txt.split("\n") if not l.startswith("echo __FIN__") and l.strip() != "(gdb)"))
PY
    echo "  (session closed: gdb detached, the ARM runs)"
}
# ---------------------------------------------------------------- demo: hands on the live DSP, then on the ARM
DEMO_PAUSE=${DEMO_PAUSE:-4}    # seconds between two demo steps, time to read (0 = chain)
DEMO_STOP=${DEMO_STOP:-1.5}    # duration of the real DSP pause: enough for the voice to cut out audibly, no more
demo_pause() { [ "$DEMO_PAUSE" != 0 ] && { echo "    ... (next in $DEMO_PAUSE s)"; sleep "$DEMO_PAUSE"; }; }
A_CD=09d2       # a_cd[0] (FIRE flag); a_cd[3]=0x09d5 start of the 23-byte L2 block (dsp_memcpy_from_api be=0)
# In the downlink CCCH/BCCH block: byte2 (type) = 0x09d6 low; cell_identity = 0x09d6 high (MSB) | 0x09d7 low (LSB).
shm_lire() { od -An -v -tx2 -j $(( (0x$1 - API_BASE) * 2 )) -N 2 "$API_SHM" 2>/dev/null | tr -d ' \n'; }  # 1 hex word
shm_ecrire() { local v=$(( 0x$2 & 0xffff )); printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" \
    | dd of="$API_SHM" bs=1 seek=$(( (0x$1 - API_BASE) * 2 )) conv=notrunc status=none 2>/dev/null; }
api_mot() { cmd_d_chaud "$1" 1 2>/dev/null | sed -n 's/.*\]=\([0-9a-fA-F]\{4\}\).*/\1/p' | head -1; }  # EXACTLY 4 hex digits, else empty
demo_si3() { # cell_id -> CELL_ID (default 777) in SI3: live loop, rewritten on every SI3 block that passes
    cible=${1:-777}
    [ -r "$API_SHM" ] || { rouge "    SI3: no API RAM segment; ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    SI3: segment is read-only; chmod 0666 $API_SHM (or sudo)"; return 1; }
    gras "    SI3: relabel the cell_id to $cible (0x$(printf %04x $cible)) on every SI3 block (L2 type 0x1b) passing through a_cd"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-1500} )); n=0; av=""
    while [ $(date +%s%3N) -lt $fin ]; do
        w4=$(shm_lire 09d6); case $w4 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ $(( 0x$w4 & 0xff )) = $(( 0x1b )) ] || continue             # not an SI3 at this instant
        w5=$(shm_lire 09d7); case $w5 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ -z "$av" ] && { av=$(( (0x$w4 >> 8 & 0xff) << 8 | (0x$w5 & 0xff) )); echo "       1st SI3 seen: cell_id = $av (0x$(printf %04x $av))"; }
        shm_ecrire 09d6 $(printf '%04x' $(( (0x$w4 & 0x00ff) | ((cible >> 8 & 0xff) << 8) )))   # keeps the 0x1b type
        shm_ecrire 09d7 $(printf '%04x' $(( (0x$w5 & 0xff00) | (cible & 0xff) )))               # keeps the LAI byte
        n=$(( n + 1 ))
    done
    if [ "$n" = 0 ]; then echo "       (no SI3 seen in ${DEMO_HOLD_MS:-1500} ms; the mobile must read the BCCH/CCCH -- retry once it camps)"; return 1; fi
    vert "       $n SI3 blocks relabelled to $cible during ${DEMO_HOLD_MS:-1500} ms (the FIRE flag stays valid, no L3 CRC)"
    echo "       -> every SI3 the firmware reads now carries cell_id $cible"
}
LAC_W=09d9      # a_cd[7]: the 2 LAC bytes of the SI3 (byte-swapped: a_cd[7] = LAC_LSB<<8 | LAC_MSB)
demo_lac() { # lac [value]: force the SI3 LAC to value (default 777) LIVE through the shm, without restart or rebuild
    cible=${1:-777}
    [ -r "$API_SHM" ] || { rouge "    LAC: no API RAM segment; ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    LAC: segment is read-only; chmod 0666 $API_SHM (or sudo)"; return 1; }
    sw=$(printf '%04x' $(( ((cible & 0xff) << 8) | ((cible >> 8) & 0xff) )))   # byte-swapped for a_cd[7]
    gras "    LAC: forced to $cible (0x$(printf %04x $cible)) in a_cd[7]=0x$sw, LIVE, on every SI3 block (no restart)"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-3000} )); n=0; av=""
    while [ $(date +%s%3N) -lt $fin ]; do
        w4=$(shm_lire 09d6); case $w4 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ $(( 0x$w4 & 0xff )) = $(( 0x1b )) ] || continue              # SI3 only
        w7=$(shm_lire $LAC_W); case $w7 in ''|*[!0-9a-fA-F]*) continue ;; esac
        [ -z "$av" ] && { av=$(( ((0x$w7 & 0xff) << 8) | ((0x$w7 >> 8) & 0xff) )); echo "       1st SI3 seen: LAC = $av (0x$(printf %04x $av))"; }
        shm_ecrire $LAC_W $sw; n=$(( n + 1 ))
    done
    [ "$n" = 0 ] && { echo "       (no SI3 seen in ${DEMO_HOLD_MS:-3000} ms; the mobile must read the BCCH)"; return 1; }
    vert "       $n writes LAC=$cible during ${DEMO_HOLD_MS:-3000} ms, live (race with the DSP: the mobile sees it if we win)"
    echo "       -> check: grep -ao 'CGI=[0-9-]*' /tmp/c54x-pont/mobile.log | tail  (LAC field = 3rd group)"
}
demo_sb() { # DEAD BEEF into a_sch (the decoded SB the ARM reads), live: rewritten in a short loop, read back
    [ -r "$API_SHM" ] || { rouge "    SB: no API RAM segment ($API_SHM); ./run.sh"; return 1; }
    [ -w "$API_SHM" ] || { rouge "    SB: segment is read-only for this account; chmod 0666 $API_SHM (or sudo)"; return 1; }
    a=0837   # a_sch[0], page R0 (see d@ noms a_sch)
    gras "    SB: a_sch at $a (the decoded SCH the firmware reads at 0xffd00000), live write"
    echo "       before: a_sch[0..1] = $(shm_lire $a) $(shm_lire 0838)"
    fin=$(( $(date +%s%3N) + ${DEMO_HOLD_MS:-1200} ))
    while [ $(date +%s%3N) -lt $fin ]; do shm_ecrire $a 40c3; shm_ecrire 0838 2026; done
    echo "       during: a_sch[0..1] = $(shm_lire $a) $(shm_lire 0838)   <- 40C3 2026 (CCC 2026) forced into the SB the firmware reads"
    vert "       (rewritten in a loop for ${DEMO_HOLD_MS:-1200} ms; the DSP takes over again at the next SB task)"
}
demo_si3_preuve() { # waits for camping and shows the CGI seen by the mobile (must end with <cible>)
    cible=$1; log=${RUNDIR_OMBRE:-/tmp/c54x-pont}/mobile.log; dlog=${RUNDIR_OMBRE:-/tmp/c54x-pont}/dsp.log
    gras "    SI3: proof on the mobile side -- cell_id forced to $cible at the source (downlink shadow in c54x_exe)"
    echo "       SI3 injections (DSP): $(grep -ac 'ombre-dl.*SI3 cell_id' "$dlog" 2>/dev/null)   (0 if the mobile does not read the BCCH yet)"
    i=0; cgi=""
    while [ $i -lt ${DEMO_CGI_ESSAIS:-40} ]; do
        cgi=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1)
        case $cgi in *-$cible) break ;; esac
        sleep 1; i=$(( i + 1 ))
    done
    case $cgi in
        *-$cible) vert "       mobile.log: $cgi   <- the phone decoded cell_id $cible (shadow succeeded)" ;;
        '') echo "       (mobile not camped yet after ${DEMO_CGI_ESSAIS:-40} s; grep CGI $log later)" ;;
        *) echo "       mobile.log: $cgi (not $cible yet; injections=$(grep -ac 'ombre-dl.*SI3 cell_id' "$dlog" 2>/dev/null), let it camp then grep CGI $log)" ;;
    esac
}
demo_dsp() { # live DSP, WITHOUT restart: forces the SI3 LAC live (shm), CGI proof, stop/go
    pid=$(vivant_pid) || { rouge "demo dsp: no c54x_exe --arm running (./run.sh)"; return 1; }
    gras "===== DSP DEMO (c54x_exe --arm pid $pid, the real C54x core serving the ARM) ====="
    log=${RUNDIR_OMBRE:-/tmp/c54x-pont}/mobile.log
    cible=${DEMO_CELL_ID:-777}
    gras "--- 1. SI3 LAC forced to $cible LIVE (shm, NO restart; the bench stays camped)"
    av=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1); echo "       CGI before: ${av:-<not camped yet>}"
    DEMO_HOLD_MS=${DEMO_LAC_MS:-5000} demo_lac "$cible"
    demo_pause
    gras "--- 2. proof on the mobile side: the LAC (3rd group of the CGI) must become $cible"
    i=0; cgi=""
    while [ $i -lt ${DEMO_CGI_ESSAIS:-20} ]; do
        cgi=$(grep -ao 'CGI=[0-9-]*' "$log" 2>/dev/null | tail -1)
        case $cgi in CGI=*-*-$cible-*) break ;; esac
        DEMO_HOLD_MS=1500 demo_lac "$cible" >/dev/null 2>&1   # hammer again while the mobile re-reads the BCCH
        sleep 1; i=$(( i + 1 ))
    done
    case $cgi in
        CGI=*-*-$cible-*) vert "       mobile.log: $cgi   <- LAC = $cible, the phone decoded it (live, no restart)" ;;
        '') echo "       (no CGI: the mobile is not camped; it must read the BCCH)" ;;
        *) echo "       mobile.log: $cgi (not $cible in the 3rd group yet; race with the DSP, run 'lac $cible' again or let it camp)" ;;
    esac
    demo_pause
    if vivant_accessible "$pid"; then
        gras "--- 3. stop: DSP paused, registers and THE ADDRESSES"; cmd_stop || return 1; demo_pause
        gras "--- 4. go: resume (fn, instructions)"; cmd_go
        gras "===== END OF DSP DEMO: SI3 LAC forced to $cible live, DSP resumed, nothing broken, no bench restart ====="
    else
        echo; vivant_diag "$pid"
        gras "===== END OF DSP DEMO: LAC forced live. stop/go: run harvard under the bench account ====="
    fi
}
arm_tourne() { # after the gdb session: does the ARM write its pages for the DSP again? (measured, not assumed)
    # gdb only lets the ARM go once the command returns: measured here, outside gdb, in the shared API RAM.
    # W pages = ARM 0xffd00000..0xffd0004f (BASE_API_W_PAGE_0/1, 2 x 20 words) = DSP 0x0800..0x0827.
    [ -r "$API_SHM" ] || { rouge "  cannot measure: $API_SHM unreadable"; return 1; }
    # In idle mode the ARM writes its W pages only on some frames: one 0.5 s window can be quiet (measured on
    # 2026-11-08: 9, 13, 1, 0, 2 words per window). Up to 6 windows (3 s): running as soon as one shows a change.
    sleep 0.5
    t=$(mktemp); n=0; k=0
    while [ "$k" -lt 6 ] && [ "$n" -eq 0 ]; do
        od -An -v -tx2 -N80 "$API_SHM" | tr -s ' ' '\n' | grep . > "$t.a"; sleep 0.5
        od -An -v -tx2 -N80 "$API_SHM" | tr -s ' ' '\n' | grep . > "$t.b"
        n=$(paste -d' ' "$t.a" "$t.b" | awk '$1 != $2' | wc -l); k=$((k + 1))
    done; rm -f "$t" "$t.a" "$t.b"
    if [ "$n" -gt 0 ]; then echo "  measured after gdb released: $n words of the ARM->DSP pages (0xffd00000, 40 words) changed in 0.5 s (window $k of 6): the ARM is running"
    else rouge "  measured after gdb released: no word of the ARM->DSP pages changed in 6 windows of 0.5 s: the ARM no longer writes"; fi
}
cmd_demo() { # demo [tout|arm [etat|trame|hello|go]|dsp]: first the ARM under gdb (hello world from ram), then the live DSP
    quoi=${1:-tout}; [ $# -gt 0 ] && shift
    case $quoi in
        dsp) demo_dsp ;;
        arm) gras "===== ARM DEMO (layer1 firmware in QEMU, gdb through telnet $GDB_TELNET) ====="; cmd_gdb demo "$@"; arm_tourne ;;
        tout) gras "===== ARM DEMO FIRST (layer1 firmware in QEMU, gdb through telnet $GDB_TELNET: hello world from ram) ====="
              cmd_gdb demo "$@"; arm_tourne; demo_pause; echo
              demo_dsp
              echo; gras "===== END OF DEMO: ARM running, DSP resumed, nothing broken ====="; demo_pause ;;
        *) echo "usage: demo [tout | dsp | arm [etat|trame|hello|go|dit <text>]]"; return 1 ;;
    esac
}

# ---------------------------------------------------------------- live API RAM (the running DSP, --arm)
API_SHM=${API_SHM:-/dev/shm/calypso_api_ram}   # = data[0x0800..0x27ff] of the c54x_exe --arm, shared with the ARM (MAP_SHARED)
API_BASE=$(( 0x0800 )); API_MOTS=8192
CARTE=${CARTE:-$HERE/docs/carte-memoire.md}    # the "API RAM - all fields" table (generated from the ELF: scalars only)
# Double pages (osmocom-bb include/calypso/dsp_api.h:18-23): DSP word = 0x0800 + (ARM address - 0xFFD00000)/2
#   W0 0x0800..0x0813  W1 0x0814..0x0827  (T_DB_MCU_TO_DSP, 17 useful words + 3 padding, dsp_api.h:32-80)
#   R0 0x0828..0x083b  R1 0x083c..0x084f  (T_DB_DSP_TO_MCU, 20 words, dsp_api.h:82-107)   NDB 0x08d4.. (268)  PARAM 0x0c31.. (57)
api_page() { # api_page <DSP word> -> W0|W1|R0|R1|NDB|PARAM|API
    a=$1; if [ $a -lt $(( 0x0814 )) ]; then echo W0; elif [ $a -lt $(( 0x0828 )) ]; then echo W1
    elif [ $a -lt $(( 0x083c )) ]; then echo R0; elif [ $a -lt $(( 0x0850 )) ]; then echo R1
    elif [ $a -ge $(( 0x08d4 )) ] && [ $a -lt $(( 0x08d4 + 268 )) ]; then echo NDB
    elif [ $a -ge $(( 0x0c31 )) ] && [ $a -lt $(( 0x0c31 + 57 )) ]; then echo PARAM; else echo API; fi
}
api_table() { # "name 0xADDR page": the generated map + the arrays the ELF does not name (dsp_api.h, API 36xx)
    grep '^| [^|]* | [a-z][a-z0-9_]* | 0x[0-9a-fA-F]* | 0xFFD[0-9A-F]* | 0x[0-9a-f]*' "$CARTE" 2>/dev/null | awk -F'|' '{gsub(/ /,"",$2); gsub(/ /,"",$3); gsub(/ /,"",$6); print $3, $6, $2}'
    for pp in 0x0800:W0 0x0814:W1; do pb=$(( ${pp%%:*} )); ppg=${pp#*:}                                  # dsp_api.h:67-70, padding 17..19
        printf 'a_a5fn[0] 0x%04x %s\na_a5fn[1] 0x%04x %s\n' $(( pb + 12 )) "$ppg" $(( pb + 13 )) "$ppg"
        printf '(padding) 0x%04x %s\n(padding) 0x%04x %s\n(padding) 0x%04x %s\n' $(( pb + 17 )) "$ppg" $(( pb + 18 )) "$ppg" $(( pb + 19 )) "$ppg"; done
    for pp in 0x0828:R0 0x083c:R1; do pb=$(( ${pp%%:*} )); ppg=${pp#*:}; pi=0                           # dsp_api.h:97-99 (DSP 33..36)
        for nm in 'a_serv_demod[0]=D_TOA' 'a_serv_demod[1]=D_PM' 'a_serv_demod[2]=D_ANGLE' 'a_serv_demod[3]=D_SNR' 'a_pm[0]' 'a_pm[1]' 'a_pm[2]' 'a_sch[0]' 'a_sch[1]' 'a_sch[2]' 'a_sch[3]' 'a_sch[4]'; do
            printf '%s 0x%04x %s\n' "$nm" $(( pb + 8 + pi )) "$ppg"; pi=$(( pi + 1 )); done; done
}
api_adr() { # api_adr <name|address> -> integer (DSP word), 1 if unknown / outside the window
    case $1 in
        [a-z]*) v=$(api_table | awk -v n="$1" '$1==n || (n !~ /\[/ && $1 ~ ("^" n "(\\[0\\])?(=|$)")) {print $2; exit}'); [ -n "$v" ] || { rouge "unknown field ($CARTE + dsp_api.h): $1   -> d@ noms <pattern>" >&2; return 1; }; v=$(( v )) ;;
        *) v=$(hex "$1") || { rouge "invalid address: $1" >&2; return 1; } ;;
    esac
    [ $v -ge $API_BASE ] && [ $v -lt $(( API_BASE + API_MOTS )) ] || { rouge "$(printf '%04x' $v): outside the API window (0800-27ff)" >&2; return 1; }
    printf '%d' $v
}
api_nom() { api_table | awk -v a="$(printf '0x%04x' $1)" '$2==a {print $3 " " $1; exit}'; }
api_vivant() { [ -r "$API_SHM" ] || { rouge "no segment $API_SHM (start ./run.sh or ./c54x_exe --arm)"; return 1; }
    pgrep -f 'c54x_exe.*--arm' > /dev/null || echo "  (no c54x_exe --arm running: frozen content of the last run)"; }
# Field decoding (sources: dsp_api.h:46-79 for the bits, l1_environment.h:36-80 for the tasks,
# :339-353 and :370-386 for d_ctrl_tch, :355-367 for d_ctrl_abb / d_ctrl_system). Nothing is guessed.
TACHES="0=NO 1=IDLE1 5=FB 6=SB 8=TCH_FB 9=TCH_SB 10=RACH 11=AUL 12=DUL 13=TCHT 14=TCHA 15=TCH_DTX_UL 17=NBN 18=EBN 19=NBS 20=EBS 21=NP 22=EP 24=ALLC 25=CB 26=DDL 27=ADL 28=TCHD 30=PNP 31=PEP 32=PALLC 33=CHECKSUM/PBS 35=TST_NDB 36=TST_DB 37=INIT_VEGA 38=DSP_LOOP_C"
MODES="0=SIG_ONLY 1=TCH_FS 2=TCH_HS 3=TCH_96 4=TCH_48F 5=TCH_48H 6=TCH_24F 7=TCH_24H 8=TCH_EFR 9=TCH_144"
TYPES="0=INVALID 1=TCH_F 2=TCH_H 3=SDCCH_4 4=SDCCH_8"
decoder() { # decoder <name> <hex16> -> text (empty if nothing to say)
    n=${1%%=*}; v=$(( 0x$2 ))
    case $n in
        d_task_d|d_task_u|d_task_md|d_task_ra) t=$(printf '%s\n' $TACHES | awk -F= -v k=$v '$1==k {print $2}'); printf 'task %d%s' $v "${t:+ = $t}" ;;
        d_fn) printf 'fn_report=%d (FN within the report period)  fn_sid=%d (FN%%104)' $(( v & 0xff )) $(( v >> 8 )) ;;
        d_ctrl_tch) m=$(printf '%s\n' $MODES | awk -F= -v k=$(( v & 0xf )) '$1==k {print $2}'); ty=$(printf '%s\n' $TYPES | awk -F= -v k=$(( v >> 4 & 3 )) '$1==k {print $2}')
            printf 'chan_mode=%d%s chan_type=%d%s' $(( v & 0xf )) "${m:+($m)}" $(( v >> 4 & 3 )) "${ty:+($ty)}"
            [ $(( v >> 6 & 1 )) = 1 ] && printf ' RESET_SACCH'; [ $(( v >> 7 & 1 )) = 1 ] && printf ' VOCODER_ON'
            [ $(( v >> 8 & 1 )) = 1 ] && printf ' SYNC_TCH_UL'; [ $(( v >> 9 & 1 )) = 1 ] && printf ' SYNC_TCH_DL'
            [ $(( v >> 10 & 1 )) = 1 ] && printf ' STOP_TCH_UL'; [ $(( v >> 11 & 1 )) = 1 ] && printf ' STOP_TCH_DL'
            [ $(( v >> 12 & 3 )) != 0 ] && printf ' tch_loop=%d' $(( v >> 12 & 3 )); [ $(( v >> 15 & 1 )) = 1 ] && printf ' SUBCHANNEL' ;;
        d_ctrl_abb) printf 'bits:'; [ $(( v & 1 )) = 1 ] && printf ' b_ramp(a_ramp in the NDB)'; [ $(( v >> 3 & 1 )) = 1 ] && printf ' b_apcdel(delays in the NDB)'
            [ $(( v >> 4 & 1 )) = 1 ] && printf ' b_afc(d_afc valid)'; [ $(( v & ~0x19 )) != 0 ] && printf ' +unknown(%04x)' $(( v & ~0x19 & 0xffff )) ;;
        'a_a5fn[0]') printf 'T2=%d T3=%d (A5 ciphering)' $(( v & 0x1f )) $(( v >> 5 & 0x3f )) ;;
        'a_a5fn[1]') printf 'T1=%d (A5 ciphering)' $(( v & 0xfff )) ;;
        d_ctrl_system) printf 'b_tsq=%d (training sequence)' $(( v & 7 )); [ $(( v >> 3 & 1 )) = 1 ] && printf ' BCCH_FREQ_IND'; [ $(( v >> 15 & 1 )) = 1 ] && printf ' TASK_ABORT' ;;
        d_afc) [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'signed %d' $v ;;
        'a_serv_demod[0]') [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'TOA %d' $v ;;
        'a_serv_demod[2]') [ $v -ge 32768 ] && v=$(( v - 65536 )); printf 'angle %d (signed)' $v ;;
    esac
}
cmd_d_chaud() { # d@ <addr|name> [n]: read the API RAM of the running DSP, labelled by absolute address and decoded
    [ $# -ge 1 ] || { echo "usage: d@ <address|name> [n]   e.g. d@ 0800 20 | d@ d_task_d | d@ W1 | d@ noms d_task"; return 1; }
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
cmd_d_chaud_ecrire() { # d@! <addr|name> <words...>: write into the API RAM of the running DSP (seen by the DSP and the ARM)
    [ $# -ge 2 ] || { echo "usage: d@! <address|name> <word> [word...]"; return 1; }
    api_vivant || return 1; a=$(api_adr "$1") || return 1; shift
    [ -w "$API_SHM" ] || { rouge "segment is read-only for this user"; return 1; }
    for m in "$@"; do v=$(hex "$m") || { rouge "invalid word: $m"; return 1; }; v=$(( v & 0xffff ))
        [ $a -lt $(( API_BASE + API_MOTS )) ] || { rouge "window overflow"; return 1; }
        printf "\\$(printf '%03o' $(( v & 255 )))\\$(printf '%03o' $(( v >> 8 )))" \
            | dd of="$API_SHM" bs=1 seek=$(( (a - API_BASE) * 2 )) conv=notrunc status=none || { rouge "write refused"; return 1; }
        printf '    [%04x] <- %04x  %s\n' $a $v "$(api_nom $a)"; a=$(( a + 1 )); done
    echo "  (written into the running DSP; the ARM sees the same segment)"
}
# ---------------------------------------------------------------- "run": the bench tools as they are
lancer() { # lancer <cmd...>: run, log, show the tail
    gras "  \$ $*"; "$@" > "$DIR/dernier_run.log" 2>&1; rc=$?
    n=$(wc -l < "$DIR/dernier_run.log")
    [ $n -gt 60 ] && echo "  ... ($n lines, all in $DIR/dernier_run.log)"
    tail -n 60 "$DIR/dernier_run.log"; echo "  (exit code $rc)"; return $rc
}
cmd_rejeu() { # rejeu [frames] [-v..]: c54x_exe --rejouer, deterministic FB/SB acquisition
    t=${1:-200}; shift 2>/dev/null; (cd "$HERE" && lancer ./c54x_exe --rejouer --trames "$t" --verbeux "$@"); }
cmd_banc() { # banc [test|--all|--list] [options dsp_tester]
    [ $# -ge 1 ] || set -- --list; (cd "$HERE" && lancer ./dsp_tester "$@"); }
cmd_isa() { # isa [-k WORD]: the SPRU172C examples against the core
    (cd "$HERE" && lancer ./isa_test tools/isa_tests.txt "$@"); }
RUNDIR_OMBRE=${RUNDIR_OMBRE:-/tmp/c54x-pont}
cmd_ombre() { # ombre [cell_id|off]: restart the WHOLE chain (run_si.sh: BTS + scaffolding + bridge) with the SI3 shadow
    n=${1:-${DEMO_CELL_ID:-777}}
    case $n in off|OFF) sv=""; lib="disabled" ;; *[!0-9]*) rouge "usage: ombre [cell_id|off]  (default 777)"; return 1 ;; *) sv="$n"; lib="LAC SI3 = $n" ;; esac
    # run_si.sh brings the mobile to decode the BCCH (native correlator not locked -> CAN_TOA/CAN_SB/AFC/LOCKSTEP scaffolding).
    # bare run.sh (PONT=0) delivers NO burst from the BTS: no camping, no SI3, the shadow never triggers.
    lanceur="$HERE/run_si.sh"; mode_full=1
    [ -x "$lanceur" ] || { lanceur="$HERE/run.sh"; mode_full=0; rouge "  run_si.sh missing: bare run.sh (no bridge, the mobile will not camp)"; }
    gras "  ombre: restarting the full chain ($(basename "$lanceur")) with CALYPSO_SHADOW_LAC=${sv:-<off>} ($lib)"
    logdir=${RUNDIR_OMBRE:-/tmp/c54x-pont}
    : > "$DIR/run_ombre.log"
    # avoid stacking: kill any ghost launcher (run_si.sh/run.sh) BEFORE restarting, otherwise
    # several chains fight over the DSP socket/the shm/the BTS and the mobile never acquires
    # (FBSB result=255). ONLY the launchers are killed, not this shell.
    for q in $(pgrep -f 'run_si\.sh|c54x_exe/run\.sh' 2>/dev/null); do [ "$q" = "$$" ] || kill "$q" 2>/dev/null; done
    ( cd "$HERE" && ./run.sh --stop ) >/dev/null 2>&1; sleep 1
    if [ "$mode_full" = 1 ]; then
        ( cd "$HERE" && CALYPSO_SHADOW_LAC="$sv" CALYPSO_SHADOW_SB="${OMBRE_SB:-}" setsid "$lanceur" --secondes "${OMBRE_SECS:-300}" ) > "$DIR/run_ombre.log" 2>&1 &
    else
        ( cd "$HERE" && ./run.sh --stop ) >/dev/null 2>&1
        ( cd "$HERE" && env MODE=dsp PONT=1 IQ=none CALYPSO_BSP_DIRECT_FEED=1 CALYPSO_PONT_LOCKSTEP=1 \
            CALYPSO_SHADOW_LAC="$sv" CALYPSO_SHADOW_SB="${OMBRE_SB:-}" setsid ./run.sh ) > "$DIR/run_ombre.log" 2>&1 &
    fi
    echo "  bench restarting (BTS + scaffolding + bridge; log $DIR/run_ombre.log)..."
    i=0; annonce=""
    while [ $i -lt ${OMBRE_ATTENTE:-50} ]; do
        [ -z "$sv" ] && { [ -S /tmp/calypso_dsp.sock ] && { echo "  bench restarted (shadow off)"; return 0; }; sleep 1; i=$((i+1)); continue; }
        annonce=$(grep -aE "ombre-dl.*LAC SI3 (force a|forced to) $sv" "$logdir/dsp.log" 2>/dev/null | tail -1)
        [ -n "$annonce" ] && break
        sleep 1; i=$((i+1))
    done
    [ -n "$annonce" ] && vert "  shadow active: ${annonce#  }" || { echo "  (shadow not announced yet after ${OMBRE_ATTENTE:-50}s; see $DIR/run_ombre.log and $logdir/dsp.log)"; return 1; }
}
cmd_statut() { (cd "$HERE" && lancer ./run.sh --status); }

# ---------------------------------------------------------------- REPL
aide() {
    cat <<'FIN'
  P side (program)                            D side (data)
    pas <insn|words>  execute, keep the state     reg [REG val]   show / set a register
    essai <insn>      execute without keeping     d <addr> [n]    read data memory
    p <addr> [n]      disassemble the ROM         d! <addr> <words> set words in D
    p! <addr> <insn>  set words in P              drom <addr> [n] read the data ROM
    asm <insn>        assemble only               etat | raz      show all | reset state
    dis <hex words>   disassemble words
    cours [N]         run in the ROM until breakpoint / IDLE / N steps (shell state)
    arret [addr..|-]  breakpoints for "cours"
  RUNNING DSP (c54x_exe --arm)                ARM (layer1 firmware under gdb, telnet 44444)
    stop              pause the live DSP + addresses   demo [tout|arm|dsp]  hands on the ARM (gdb) then the DSP
    go                resume                           gdb <cmd>   one panel command (fn, l1, help_osmo)
    dsp               snapshot without stopping                    (demo arm etat|trame|hello|go: step by step)
                                                   dit <text>  the firmware prints it (puts -> osmocon.log)
    sb                DEAD BEEF into a_sch (SB)        si3 [id]    relabel the SI3 cell_id (default 777)
    lac [id]          force the SI3 LAC LIVE (shm, no restart; race with the DSP)
    ombre [id|off]    restart the bench, SI3 LAC forced at the SOURCE (deterministic; mobile.log CGI 3rd group)
    d@ <addr|name|W0..R1> [n]  decoded API RAM   d@! <addr|name> <words>  write it   d@ noms [pattern]
  run                                            misc
    rejeu [frames]    c54x_exe --rejouer           journal [n]     last steps
    banc [test|--all] dsp_tester                   log [n]         core warnings
    isa [-k WORD]     isa_test SPRU172C            sauve|charge <f> state on disk
    statut            run.sh --status              menu | aide | q
  addresses: 1234, 0x1234, 1:8000 (page 1).  insn: TI mnemonic (LD #0x1234, A) or hex words (f020 1234).
FIN
}
dispatch() { # dispatch <command line>
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
        dit) [ $# -ge 1 ] || { echo "usage: dit <text>   (e.g. dit hello 40c3)"; return 1; }; cmd_demo arm dit "$@" ;;
        sb|demo_sb) demo_sb ;;   si3|demo_si3) demo_si3 "$@" ;;   lac|demo_lac) demo_lac "$@" ;;
        demo_dsp) demo_dsp ;;    demo_arm) cmd_gdb demo "$@" ;;
        journal) cmd_journal "$@" ;; log) cmd_log "$@" ;;
        sauve) cmd_sauve "$@" ;;  charge) cmd_charge "$@" ;;
        rejeu) cmd_rejeu "$@" ;;  banc) cmd_banc "$@" ;;    isa) cmd_isa "$@" ;;  statut) cmd_statut ;;
        ombre) cmd_ombre "$@" ;;
        aide|'?'|help) aide ;;
        *) rouge "unknown command: $c (aide)"; return 1 ;;
    esac
}
repl() {
    verifier_outils; gras "c54x Harvard shell - 'aide' for the commands, 'menu' for the menu, 'q' to quit"
    while :; do
        pc=$(etat_reg PC); if [ -t 1 ]; then printf '\033[1mP:%s D>\033[0m ' "${pc:-9000}"; else printf 'P:%s D> ' "${pc:-9000}"; fi
        read -r ligne || { echo; return 0; }
        set -- $ligne; [ $# -eq 0 ] && continue
        case $1 in q|quit|exit) return 0 ;; menu) return 2 ;; esac
        dispatch "$@"
    done
}

# ---------------------------------------------------------------- interactive menu
pause() { printf '\n  [Enter to continue] '; read -r _; }
demander() { # demander <prompt> [default] -> REP
    printf '  %s' "$1"; [ -n "${2:-}" ] && printf ' [%s]' "$2"; printf ' : '
    read -r REP; [ -z "$REP" ] && REP=${2:-}
}
menu() {
    verifier_outils
    while :; do
        clear 2>/dev/null
        pc=$(etat_reg PC)
        gras "================  HARVARD SHELL - TMS320C54x DSP (Calypso)  ================"
        printf '  PC=%s  A=%s  B=%s  SP=%s   %s\n' "${pc:-9000 (init)}" "$(etat_reg A)" "$(etat_reg B)" "$(etat_reg SP)" "$(drapeaux "$(etat_reg ST0)" "$(etat_reg ST1)")"
        cat <<'FIN'

  P SPACE (program)                        D SPACE (data)
   1) execute one instruction (a step)      5) show registers and words set
   2) try an instruction (not kept)         6) set a register
   3) disassemble the program ROM           7) read the core's data memory
   4) set words in P (subroutine)           8) set words in D
                                            9) read the data ROM (DROM / PDROM)
  RUN IN THE ROM (shell state)             RUNNING DSP (c54x_exe --arm, /dev/shm)
  10) cours: until breakpoint / IDLE / N   12) stop: pause the live DSP, registers, addresses
  11) arret: breakpoints                   13) go: resume            14) dsp: snapshot
                                           15) read the live API RAM (address or name)
                                           16) write into the live API RAM
  ARM (layer1 firmware under gdb, telnet 44444)
  17) demo: hands on the ARM then the DSP  18) gdb: one osmocom panel command
  RUN (the bench tools, as they are)       MISC
  19) c54x_exe --rejouer (FB/SB acquisition) 23) step log / core warnings
  20) dsp_tester (list, one test, --all)     24) reset state / save / load
  21) isa_test (SPRU172C examples, filter)   r) command prompt (REPL)
  22) run.sh --status                        q) quit
FIN
        demander "choice"
        case $REP in
            1) demander "instruction (TI mnemonic or hex words)" "NOP"; cmd_pas $REP; pause ;;
            2) demander "instruction" "NOP"; cmd_essai $REP; pause ;;
            3) demander "address (7000.., e000.., 1:8000)" "7000"; a=$REP; demander "number of words" 24; cmd_p "$a" "$REP"; pause ;;
            4) demander "P address" "9100"; a=$REP; demander "instruction or words"; cmd_p_pose "$a" $REP; pause ;;
            5) cmd_etat; pause ;;
            6) demander "register" "AR3"; r=$REP; demander "hex value" "0"; cmd_reg "$r" "$REP"; pause ;;
            7) demander "D address" "0100"; a=$REP; demander "number of words (<=24)" 8; cmd_d "$a" "$REP"; pause ;;
            8) demander "D address" "0100"; a=$REP; demander "hex words (space separated)"; cmd_d_pose "$a" $REP; pause ;;
            9) demander "address (9000.., e000..)" "9000"; a=$REP; demander "number of words" 32; cmd_drom "$a" "$REP"; pause ;;
            10) cmd_arret; demander "max number of steps" 200; cmd_cours "$REP"; pause ;;
            11) cmd_arret; demander "arret <addr..> | - (clear) | empty" ""; [ -n "$REP" ] && cmd_arret $REP; pause ;;
            12) cmd_stop; pause ;;
            13) cmd_go; pause ;;
            14) cmd_dsp; pause ;;
            15) demander "address, name, page W0|W1|R0|R1, or noms <pattern>" "W0"; a=$REP; demander "number of words" 8; case $a in noms*) cmd_d_chaud $a ;; *) cmd_d_chaud "$a" "$REP" ;; esac; pause ;;
            16) demander "address or name" "d_task_d"; a=$REP; demander "hex words (space separated)"; cmd_d_chaud_ecrire "$a" $REP; pause ;;
            17) demander "demo tout | dsp | arm [etat|trame|hello|go|dit <text>]" "tout"; cmd_demo $REP; pause ;;
            18) demander "gdb command (fn, l1, dsp, help_osmo...)" "help_osmo"; cmd_gdb $REP; pause ;;
            19) demander "frames" 200; t=$REP; demander "verbosity (-v .. -vvvvvv, empty = none)" ""; cmd_rejeu "$t" $REP; pause ;;
            20) cmd_banc --list; demander "test (name, --all, or empty)" ""; [ -n "$REP" ] && { demander "options (--trace N, -v, --insns N...)" ""; o=$REP; cmd_banc $REP $o; }; pause ;;
            21) demander "filter -k (empty = all)" ""; [ -n "$REP" ] && cmd_isa -k "$REP" || cmd_isa; pause ;;
            22) cmd_statut; pause ;;
            23) cmd_journal 20; cmd_log 10; pause ;;
            24) demander "raz | sauve <file> | charge <file>" "raz"; dispatch $REP; pause ;;
            r|R) repl; [ $? = 2 ] || return 0 ;;
            q|Q) return 0 ;;
            *) ;;
        esac
    done
}

# ---------------------------------------------------------------- entry point
case ${1:-menu} in
    menu) menu ;;
    repl) repl ;;
    aide|-h|--help) sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; aide ;;
    *) dispatch "$@" ;;
esac
