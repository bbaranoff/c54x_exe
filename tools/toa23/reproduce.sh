#!/usr/bin/env bash
# reproduce.sh: reproduces the "magic 23" evidence of proof_toa_23.md and writes it to toa.log.
#
#   tools/toa23/reproduce.sh [WORKDIR]      (default WORKDIR: tools/toa23/work)
#
# Idempotent: downloads, the binutils build and the osmocom-bb clone are reused when present.
# Needs: bash, curl, ca-certificates, python3, build-essential (gcc, make, libc6-dev), bzip2, git.
# Sources, all public and fetched here: the FreeCalypso dump of the TI DSP ROM 3606 (not
# redistributed by this repository), GNU binutils 2.21.1, osmocom-bb, two baseband-devel posts.
# Exit status 0 only if every check below holds.
set -u -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${1:-$HERE/work}"; mkdir -p "$WORK"; cd "$WORK" || exit 2
export LC_ALL=C

OBB_URL=https://gitea.osmocom.org/phone-side/osmocom-bb
OBB_REV=168b8c5fe49e84a8a8142e64519085b363b4982a       # HEAD verified on 2026-11-07
L1=src/target/firmware/layer1
DUMP_URL=ftp://ftp.freecalypso.org/pub/GSM/Calypso/dsp-rom-3606-dump.txt
DUMP_SHA=e35798f50fd0a29aa61cd53e417c010d7f172582c7bc055211daab8af1e05470
PROM0_SHA=75ce5375a07eac85ee42ba00af7ca82089d15b5df517710a756e6c26497021d7
BU_URL=https://ftp.gnu.org/gnu/binutils/binutils-2.21.1.tar.bz2
BU_SHA=cdecfa69f02aa7b05fbcdf678e33137151f361313b2f3e48aba925f64eabf654
HW1=https://lists.osmocom.org/pipermail/baseband-devel/2011-September/002526.html
HW2=https://www.mail-archive.com/baseband-devel@lists.osmocom.org/msg01046.html

LOG=toa.log; RES=results.txt
: > "$LOG"; : > "$RES"
say() { printf '%s\n' "$*" | tee -a "$LOG"; }
run() { say "\$ $*"; "$@" 2>&1 | tee -a "$LOG"; say ""; }
res() { printf '%s=%s\n' "$1" "$2" >> "$RES"; }
fetch() { [ -s "$2" ] || curl -fsSL --retry 3 -o "$2" "$1" || { say "DOWNLOAD FAILED: $1"; rm -f "$2"; }; }
shaok() { echo "$2  $1" | sha256sum -c --quiet - >/dev/null 2>&1; }

say "# magic 23 reproduction — $(date -u +%Y-%m-%dT%H:%MZ) — $(uname -srm)"
say "# workdir: $WORK"
say ""

# ---------------------------------------------------------------- 1. ROM
say "## 1. ROM dump -> PROM0.bin"
fetch "$DUMP_URL" dsp-rom-3606-dump.txt
run sha256sum dsp-rom-3606-dump.txt
shaok dsp-rom-3606-dump.txt "$DUMP_SHA" && res dump_sha ok || { say "SHA-256 MISMATCH (expected $DUMP_SHA)"; res dump_sha bad; }
python3 -I - <<'E' 2>&1 | tee -a "$LOG"
import re, struct
sec = None; w = {}
for l in open("dsp-rom-3606-dump.txt", errors="replace"):
    m = re.match(r"DSP dump: (\w+) \[", l)
    if m: sec = m.group(1); continue
    m = re.match(r"([0-9a-f]{5}) : ((?:[0-9a-f]{4} ?)+)", l)
    if m and sec == "PROM0":
        a = int(m.group(1), 16)
        for i, x in enumerate(m.group(2).split()): w[a + i] = int(x, 16)
open("PROM0.bin", "wb").write(b"".join(struct.pack("<H", w[a]) for a in range(0x7000, 0xe000)))
print("PROM0.bin: %d words (0x7000-0xdfff)" % len(range(0x7000, 0xe000)))
for a in (0x7cbd, 0x84a4, 0xb21d, 0xb2b1, 0xb32d):
    print("  %04x: %04x %04x" % (a, w[a], w[a + 1]))
E
run sha256sum PROM0.bin
shaok PROM0.bin "$PROM0_SHA" && res prom0_sha ok || { say "SHA-256 MISMATCH (expected $PROM0_SHA)"; res prom0_sha bad; }

say "### 0x7a1c-0x7a5b as data: the 64 correlator coefficients (read by 0x7c49-0x7c58, §2.2)"
say "### expected: sign bit = SB extended training sequence, GSM 05.02 §5.2.5 (1 -> -512, 0 -> +512)"
python3 -I - <<'E' 2>&1 | tee -a "$LOG"
import struct
rom = open("PROM0.bin", "rb").read()
word = lambda a: struct.unpack_from("<H", rom, 2 * (a - 0x7000))[0]
c = [word(0x7a1c + i) for i in range(64)]
for r in range(4):
    print("  %04x: %s" % (0x7a1c + 16 * r, " ".join("%04x" % x for x in c[16 * r:16 * r + 16])))
s = [x - 0x10000 if x & 0x8000 else x for x in c]
# GSM 05.02 (3GPP TS 45.002) §5.2.5, synchronisation burst, extended training sequence, 64 bits
ets = "1011100101100010000001000000111100101101010001010111011000011011"
got = "".join("1" if v < 0 else "0" for v in s)
print("  values         %s" % sorted(set(s)))
print("  ETS 05.02      %s" % ets)
print("  coeff < 0      %s" % got)
n = sum(a == b for a, b in zip(ets, got)) if set(s) == {-512, 512} else 0
print("  %d/64 bits match" % n)
open("results.txt", "a").write("ets=%d/64\n" % n)
E
say ""

# ---------------------------------------------------------- 2. disassembler
say "## 2. objdump tic54x (binutils 2.21.1)"
OBJDUMP=build-tic54x/binutils/objdump
if [ ! -x "$OBJDUMP" ]; then
    fetch "$BU_URL" binutils-2.21.1.tar.bz2
    shaok binutils-2.21.1.tar.bz2 "$BU_SHA" || say "SHA-256 MISMATCH binutils-2.21.1.tar.bz2 (expected $BU_SHA)"
    [ -d binutils-2.21.1 ] || tar xjf binutils-2.21.1.tar.bz2
    say "building objdump for tic54x-coff (a few minutes)"
    mkdir -p build-tic54x && ( cd build-tic54x && \
      ../binutils-2.21.1/configure --target=tic54x-coff --disable-werror --disable-nls --disable-gdb \
        --disable-gas --disable-ld --disable-gold --disable-gprof --disable-sim MAKEINFO=true \
        CFLAGS='-O1 -w -std=gnu89' >build.log 2>&1 && make -j"$(nproc)" all-binutils MAKEINFO=true >>build.log 2>&1 ) \
      && say "built $OBJDUMP" || { say "BUILD FAILED, see $WORK/build-tic54x/build.log"; tail -20 build-tic54x/build.log | tee -a "$LOG"; }
fi
run "$OBJDUMP" --version
"$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma=0x7000 PROM0.bin > PROM0.dis 2>&1
say "full disassembly: PROM0.dis ($(wc -l < PROM0.dis) lines)"
say ""
say "### expected: 7cbd sub #11236,a | 84a4 stm #10791,ar3 | b21d stm #191,ar3 | b2b1 stm #151,ar3 | b32d stm #48,ar3"
grep -E '^ *(7cbd|84a4|b21d|b2b1|b32d):' PROM0.dis | tee -a "$LOG"
grep -qE '^ *b21d:.*stm +#191,ar3' PROM0.dis && res rom_191 ok || res rom_191 bad
grep -qE '^ *b2b1:.*stm +#151,ar3' PROM0.dis && res rom_151 ok || res rom_151 bad
say ""
say "### SB TOA chain (0x7a1c is data, dumped in section 1: its disassembly is meaningless)"
for a in 7c3d 81c8 7c49 84a1 84ca 84dd 7c5f 7ca6 7cbc 7e8a b1e7; do
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
say "## 3. layer1 source and git history, at $OBB_REV"
[ -d osmocom-bb/.git ] || run git clone -q "$OBB_URL" osmocom-bb
git -C osmocom-bb cat-file -e "$OBB_REV^{commit}" 2>/dev/null || run git -C osmocom-bb fetch -q origin
G() { git -C osmocom-bb "$@"; }
run git -C osmocom-bb log -1 --format='%H %ad %s' --date=short "$OBB_REV"
say "### commits that add or remove a 'toa -= 23' line (-G; -S misses cb71b972, which adds one and removes one)"
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -G'toa -= 23' "$OBB_REV" -- $L1
case "$(G log --format='%h' -G'toa -= 23' "$OBB_REV" -- $L1)" in *cb71b972*) res git_cb71 ok;; *) res git_cb71 bad;; esac
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -G'magic 23' "$OBB_REV" -- $L1
say "### cb71b972: expect: toa -= 23 moved from l1a_fb_compl into fbinfo2cellinfo (one + and one -), while the mode-1 toa -= 23 in l1s_fbdet_resp stays"
G show cb71b972 -- $L1/prim_fbsb.c | grep -nE '^[-+].*(toa -= 23|fbinfo2cellinfo)|^@@' | tee -a "$LOG"
say ""
say "### the FIXME lines and their origin (git blame at $OBB_REV)"
G blame -L333,334 -L469,470 --date=short "$OBB_REV" -- $L1/prim_fbsb.c | tee -a "$LOG"
say ""
say "### constants and call sites at $OBB_REV (file:line)"
G show "$OBB_REV:$L1/tpu_window.c" | grep -nE 'L1_(SB|NB)_MARGIN_Q|L1_TAIL_DURATION_Q|L1_(SB|NB)_DURATION_Q' | sed 's/^/tpu_window.c:/' | tee -a "$LOG"
G show "$OBB_REV:$L1/prim_fbsb.c" | grep -nE 'toa -= 23|magic 23|fbinfo2cellinfo\(|l1a_fb_compl' | sed 's/^/prim_fbsb.c:/' | tee -a "$LOG"
G show "$OBB_REV:$L1/sync.c" | grep -nE 'tpu_shift \+= 75|SB TOA of 4' | sed 's/^/sync.c:/' | tee -a "$LOG"
TPU="$(G show "$OBB_REV:$L1/tpu_window.c")"
grep -qE 'define L1_SB_MARGIN_Q[[:space:]]+\(23 \* 4\)' <<< "$TPU" && res fw_sb ok || res fw_sb bad
grep -qE 'define L1_NB_MARGIN_Q[[:space:]]+\(3 \* 4\)' <<< "$TPU" && res fw_nb ok || res fw_nb bad
say ""

# ---------------------------------------------------------------- 4. hardware logs
say "## 4. hardware logs, parsed from the list archives"
say "[1] $HW1"
say "[2] $HW2"
fetch "$HW1" hw1.html
fetch "$HW2" hw2.html
python3 -I - <<'E' 2>&1 | tee -a "$LOG"
import html, re, os
def lines(f):
    if not os.path.exists(f): return []
    t = html.unescape(re.sub(r"<[^>]+>", "", open(f, errors="replace").read()))
    return [l.strip() for l in t.splitlines() if re.search(r"TOA=|qbits=|TOA AVG", l)]
fb0, fb1, pairs, sbs = [], [], [], []
for f in ("hw1.html", "hw2.html"):
    last = None   # ("FB1"|"SB", toa) waiting for its "=>FB @ ... qbits=" line
    print("  --- %s" % f)
    for l in lines(f):
        print("  " + re.sub(r",.*", "", l) if "TOA=" in l else "  " + l)
        m = re.search(r"(FB0|FB1|SB1) \(\d+:\d+\): TOA= *(\d+)", l)
        if m:
            k, v = m.group(1), int(m.group(2))
            if k == "FB0": fb0.append(v); last = None
            elif k == "FB1": fb1.append(v); last = ("FB1", v)
            else: sbs.append(v); last = ("SB", v)
            continue
        m = re.search(r"=>FB @ .*qbits=(\d+)", l)
        if m and last:
            pairs.append((last[0], last[1], int(m.group(1)))); last = None
print()
print("  qbits printed by fbinfo2cellinfo vs ((TOA-46) mod 1250)*4 and ((TOA-23) mod 1250)*4")
ok = 0
for k, toa, q in pairs:
    p46 = ((toa - 46) % 1250) * 4; p23 = ((toa - 23) % 1250) * 4
    ok += p46 == q
    print("  %-3s TOA=%5d printed=%4d  -46:%4d %s  -23:%4d" % (k, toa, q, p46, "OK " if p46 == q else "NO ", p23))
print("  %d/%d lines match the -46 rule" % (ok, len(pairs)))
m1 = sum(t % 4 == 3 for t in fb1); m0 = sum(t % 48 == 0 for t in fb0)
print("  FB mode 1 TOAs %s: %d/%d == 3 mod 4" % (fb1, m1, len(fb1)))
print("  FB mode 0 TOAs %s: %d/%d multiples of 48" % (fb0, m0, len(fb0)))
mean = sum(t - 27.25 for t in sbs) / len(sbs) if sbs else float("nan")
print("  first SB after FB sync %s: mean delta vs 27.25 = %+.2f bit" % (sbs, mean))
with open("results.txt", "a") as r:
    r.write("qbits=%d/%d\nm1=%d/%d\nm0=%d/%d\nsb_n=%d\nsb_mean=%.2f\n" % (ok, len(pairs), m1, len(fb1), m0, len(fb0), len(sbs), mean))
E
say ""

# ---------------------------------------------------------------- summary
say "# full log: $WORK/$LOG"
say "## summary"
python3 -I - <<'E' 2>&1 | tee summary.txt | tee -a "$LOG"
r = dict(l.strip().split("=", 1) for l in open("results.txt") if "=" in l)
ok = lambda *k: all(r.get(x) == "ok" for x in k)
frac = lambda k: r.get(k, "0/1").split("/")
full = lambda k: frac(k)[0] == frac(k)[1] and frac(k)[1] != "0"
bad = []
if ok("rom_191", "fw_sb") and 148 + 2 * 23 - 3 == 191: print("191 = 148 + 2*23 - 3   (ROM 0xb21d: stm #191,ar3)")
else: bad.append("SB window 191")
if ok("rom_151", "fw_nb") and 148 + 2 * 3 - 3 == 151: print("151 = 148 + 2*3  - 3   (ROM 0xb2b1: stm #151,ar3)")
else: bad.append("NB window 151")
print("qbits rule ((TOA-46) mod 1250)*4: %s" % r.get("qbits"))
print("FB mode 1 = 3 mod 4: %s ; FB mode 0 multiple of 48: %s" % (r.get("m1"), r.get("m0")))
print("first SB after FB sync: mean delta %s bit (n=%s)" % (r.get("sb_mean"), r.get("sb_n")))
for k in ("qbits", "m1", "m0", "ets"):
    if not full(k): bad.append(k)
for k in ("dump_sha", "prom0_sha", "git_cb71"):
    if r.get(k) != "ok": bad.append(k)
if bad: print("FAILED: " + ", ".join(bad))
open("status", "w").write("FAILED\n" if bad else "OK\n")
E
[ "$(cat status)" = OK ]
