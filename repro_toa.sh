#!/bin/bash
# Reproduces the "magic 23" evidence and writes everything to toa.log.
# Idempotent: skips downloads and builds already present.
# Needs: curl, python3, gcc, make; osmocom-bb is cloned if absent.
set -u
LOG=toa.log
: > "$LOG"
say() { printf '%s\n' "$*" | tee -a "$LOG"; }
run() { say "\$ $*"; "$@" 2>&1 | tee -a "$LOG"; say ""; }

say "# magic 23 reproduction — $(date -u +%Y-%m-%dT%H:%MZ) — $(uname -srm)"
say ""

# ---------------------------------------------------------------- 1. ROM
say "## 1. ROM dump -> PROM0.bin"
[ -f dsp-rom-3606-dump.txt ] || run curl -sO ftp://ftp.freecalypso.org/pub/GSM/Calypso/dsp-rom-3606-dump.txt
run sha256sum dsp-rom-3606-dump.txt
python3 - <<'E' 2>&1 | tee -a "$LOG"
import re,struct
sec=None;w={}
for l in open("dsp-rom-3606-dump.txt",errors="replace"):
    m=re.match(r"DSP dump: (\w+) \[",l)
    if m: sec=m.group(1); continue
    m=re.match(r"([0-9a-f]{5}) : ((?:[0-9a-f]{4} ?)+)",l)
    if m and sec=="PROM0":
        a=int(m.group(1),16)
        for i,x in enumerate(m.group(2).split()): w[a+i]=int(x,16)
open("PROM0.bin","wb").write(b"".join(struct.pack("<H",w[a]) for a in range(0x7000,0xe000)))
print("PROM0.bin: %d words (0x7000-0xdfff)" % len(range(0x7000,0xe000)))
for a in (0x7cbd,0x84a4,0xb21d,0xb2b1,0xb32d):
    print("  %04x: %04x %04x" % (a,w[a],w[a+1]))
E
run sha256sum PROM0.bin
say ""

# ---------------------------------------------------------- 2. disassembler
say "## 2. objdump tic54x (binutils 2.21.1)"
OBJDUMP=build-tic54x/binutils/objdump
if [ ! -x "$OBJDUMP" ]; then
    [ -f binutils-2.21.1.tar.bz2 ] || run curl -sO https://ftp.gnu.org/gnu/binutils/binutils-2.21.1.tar.bz2
    [ -d binutils-2.21.1 ] || tar xjf binutils-2.21.1.tar.bz2
    mkdir -p build-tic54x && ( cd build-tic54x && \
      ../binutils-2.21.1/configure --target=tic54x-coff --disable-werror --disable-nls --disable-gdb \
        --disable-gas --disable-ld --disable-gold --disable-gprof --disable-sim MAKEINFO=true \
        CFLAGS='-O1 -w -std=gnu89' >/dev/null && make -j8 all-binutils MAKEINFO=true >/dev/null ) \
      && say "built $OBJDUMP" || say "BUILD FAILED"
fi
run "$OBJDUMP" --version
"$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma=0x7000 PROM0.bin > PROM0.dis 2>&1
say "full disassembly: PROM0.dis ($(wc -l < PROM0.dis) lines)"
say ""
say "### expected: 7cbd sub #11236,a | 84a4 stm #10791,ar3 | b21d stm #191,ar3 | b2b1 stm #151,ar3 | b32d stm #48,ar3"
grep -E '^ *(7cbd|84a4|b21d|b2b1|b32d):' PROM0.dis | tee -a "$LOG"
say ""
say "### SB TOA chain"
for a in 7c3d 81c8 7c49 7a1c 84a1 84ca 84dd 7c5f 7ca6 7cbc 7e8a b1e7; do
    grep -E "^ *$a:" PROM0.dis | tee -a "$LOG"
done
say ""
say "### FB TOA: 0x793e-0x795a"
awk '/^ *793e:/,/^ *795a:/' PROM0.dis | tee -a "$LOG"
say ""
say "### DMA programming routine: 0xa5cd-0xa5f6"
awk '/^ *a5cd:/,/^ *a5f6:/' PROM0.dis | tee -a "$LOG"
say ""

# ---------------------------------------------------------------- 3. git
say "## 3. layer1 git history"
[ -d osmocom-bb ] || run git clone -q https://gitea.osmocom.org/phone-side/osmocom-bb
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -S"toa -= 23" -- src/target/firmware/layer1
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -S"magic 23" -- src/target/firmware/layer1
say "### cb71b972: toa -= 23 / fbinfo2cellinfo lines (expect added fbinfo2cellinfo, no removed toa -= 23)"
git -C osmocom-bb show cb71b972 -- src/target/firmware/layer1/prim_fbsb.c | grep -nE '^[-+].*(toa -= 23|fbinfo2cellinfo)|^@@' | tee -a "$LOG"
say ""
say "### current constants"
grep -nE 'L1_(SB|NB)_MARGIN_Q|L1_TAIL_DURATION_Q|L1_(SB|NB)_DURATION_Q' osmocom-bb/src/target/firmware/layer1/tpu_window.c | tee -a "$LOG"
grep -nE 'toa -= 23|magic 23' osmocom-bb/src/target/firmware/layer1/prim_fbsb.c | tee -a "$LOG"
grep -nE 'tpu_shift \+= 75|SB TOA of 4' osmocom-bb/src/target/firmware/layer1/sync.c | tee -a "$LOG"
say ""

# ---------------------------------------------------------------- 4. logs
say "## 4. hardware logs: qbits check ((TOA-46) mod 1250)*4"
say "sources: http://lists.osmocom.org/pipermail/baseband-devel/2011-September/002526.html"
say "         https://www.mail-archive.com/baseband-devel@lists.osmocom.org/msg01046.html"
python3 - <<'E' 2>&1 | tee -a "$LOG"
fb1 = [(9651,3420),(8751,4820),(8755,4836),(10003,4828),(10007,4844),(8755,4836)]
sb  = [(29,4932),(29,4932),(27,4924),(27,4924),(24,4912),(26,4920)]
ok = 0
for toa,q in fb1+sb:
    p46 = ((toa-46)%1250)*4; p23 = ((toa-23)%1250)*4
    ok += (p46==q)
    print("  TOA=%5d printed=%4d  -46:%4d %s  -23:%4d" % (toa,q,p46,"OK " if p46==q else "NO ",p23))
print("  %d/%d lines match the -46 rule" % (ok,len(fb1)+len(sb)))
fb0=[7200,2544,8784,10032,48,1296,8784]; m1=[9651,8751,8755,10003,10007,12507,12507,1259,10007,8755]
print("  FB mode 0 multiples of 48: %d/%d" % (sum(t%48==0 for t in fb0),len(fb0)))
print("  FB mode 1 == 3 mod 4:      %d/%d" % (sum(t%4==3 for t in m1),len(m1)))
d=[t-27.25 for t,_ in sb]
print("  first SB after FB sync: %s  mean delta %.2f" % ([t for t,_ in sb], sum(d)/len(d)))
print("  148 + 2*23 - 3 = %d ; 148 + 2*3 - 3 = %d ; (191-148+3)/2 = %g" % (148+2*23-3,148+2*3-3,(191-148+3)/2))
E
say ""
say "# done — $(wc -l < "$LOG") lines in $LOG"
