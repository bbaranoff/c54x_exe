#!/usr/bin/env python3
# =====================================================================
# GSM 05.03 xCCH block interleaving — REFERENCE
# Applies to BCCH / CCCH (PCH/AGCH) / SDCCH / SACCH.
# One 456-bit coded block -> 4 normal bursts of 114 data bits each,
# NO cross-block overlap (block-rectangular, unlike the 8-burst TCH).
#
# Byte-for-byte identical to libosmocore:
#     gsm0503_xcch_interleave() / gsm0503_xcch_deinterleave()
#         B = k & 3
#         j = 2*((49*k) % 57) + ((k % 8) >> 2)
#         iB[B*114 + j] = cB[k]        (deinterleave: cB[k] = iB[B*114+j])
#
# Burst data layout for j:
#     j =  0..56  -> first  57-bit half  (before the 26-bit midamble)
#     j = 57..113 -> second 57-bit half  (after  the midamble)
#   The 2 stealing flags are NOT part of these 114 bits.
# =====================================================================
import csv
import random
from collections import Counter

N = 456

def fwd_map():
    """k -> (k, B, j, iB_index)."""
    rows = []
    for k in range(N):
        B = k & 3                                   # burst 0..3  (k mod 4)
        j = 2 * ((49 * k) % 57) + ((k % 8) >> 2)    # position 0..113
        rows.append((k, B, j, B * 114 + j))
    return rows

# ---- verify the map is a real interleaver (bijection + full coverage) ----
def verify(rows):
    idx = [r[3] for r in rows]
    assert sorted(idx) == list(range(N)), "iB index is NOT a bijection"
    pairs = Counter((r[1], r[2]) for r in rows)
    assert len(pairs) == N and all(v == 1 for v in pairs.values()), "duplicate (B,j)"
    for B in range(4):
        js = sorted(r[2] for r in rows if r[1] == B)
        assert js == list(range(114)), f"burst {B}: j not full 0..113"
    return True

# ---- rate-1/2 K=5 convolutional code (GSM 05.03 4.1.3) --------------------
# G0 = 1 + D^3 + D^4   (0b10011 = 0x13)  -> even coded index  c(2k)
# G1 = 1 + D + D^3 + D^4 (0b11011 = 0x1B) -> odd  coded index  c(2k+1)
def conv_encode(u):
    assert len(u) == 228                 # 184 info + 40 Fire parity + 4 tail(=0)
    U = lambda i: u[i] if 0 <= i < 228 else 0
    c = [0] * N
    for k in range(228):
        c[2*k]   = U(k) ^ U(k-3) ^ U(k-4)
        c[2*k+1] = U(k) ^ U(k-1) ^ U(k-3) ^ U(k-4)
    return c

def interleave(c):
    iB = [0] * N
    for (k, B, j, idx) in fwd_map():
        iB[idx] = c[k]
    return iB

def deinterleave(iB):
    c = [0] * N
    for (k, B, j, idx) in fwd_map():
        c[k] = iB[idx]
    return c

# =========================================================================
if __name__ == "__main__":
    rows = fwd_map()
    verify(rows)

    # round-trip proof: u -> encode -> interleave -> deinterleave == c
    random.seed(0)
    u = [random.randint(0, 1) for _ in range(224)] + [0, 0, 0, 0]
    c = conv_encode(u)
    assert deinterleave(interleave(c)) == c
    print("OK  interleaver is a bijection over 0..455")
    print("OK  per-burst j fully covers 0..113")
    print("OK  conv-encode -> interleave -> deinterleave round-trip exact\n")

    # ---- main table: k -> (burst, pos), plus the two bug variants ----
    with open("cch_interleave_ref.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "lane_G(0=G0/c0,1=G1/c1)", "burst_B", "pos_j",
                    "iB_index", "j_if_LSB_dropped", "j_if_LSB_inverted",
                    "iB_index_if_LSB_inverted"])
        for (k, B, j, idx) in rows:
            base = 2 * ((49 * k) % 57)
            j_drop = base
            j_inv = base + (1 - ((k % 8) >> 2))
            w.writerow([k, k & 1, B, j, idx, j_drop, j_inv, B * 114 + j_inv])

    # ---- inverse table: what a deinterleaver actually indexes ----
    inv = [0] * N
    for (k, B, j, idx) in rows:
        inv[idx] = k
    with open("cch_deinterleave_inverse_ref.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["iB_index", "burst_B", "pos_j", "source_coded_k", "lane_G"])
        for idx in range(N):
            k = inv[idx]
            w.writerow([idx, idx // 114, idx % 114, k, k & 1])

    # ---- deinterleave with a deliberate bug, for stage isolation ----
    def deinterleave_mode(iB, mode):
        c = [0] * N
        for k in range(N):
            B = k & 3
            base = 2 * ((49 * k) % 57)
            if mode == "correct":
                j = base + ((k % 8) >> 2)
            elif mode == "lsb_dropped":     # ((k%8)>>2) forced to 0
                j = base
            elif mode == "lsb_inverted":    # 0<->1 on the LSB term
                j = base + (1 - ((k % 8) >> 2))
            c[k] = iB[B * 114 + j]
        return c

    # Known test vector. u is pseudo-random (dense in 1s) on purpose:
    # any interleaver error then shows up massively at the cB level.
    # Fire is NOT needed here - this isolates interleaver + conv-decoder only.
    random.seed(1)
    u_tv = [random.randint(0, 1) for _ in range(224)] + [0, 0, 0, 0]
    c_tv = conv_encode(u_tv)                 # 456 coded bits, correct order
    iB_tv = interleave(c_tv)                 # 4 x 114, from the CORRECT interleaver
    cB_correct = deinterleave_mode(iB_tv, "correct")      # == c_tv (Viterbi(cB)->u)
    cB_drop    = deinterleave_mode(iB_tv, "lsb_dropped")
    cB_inv     = deinterleave_mode(iB_tv, "lsb_inverted")
    assert cB_correct == c_tv

    s = lambda bits: "".join(map(str, bits))
    with open("cch_testvectors.txt", "w") as f:
        f.write("# GSM 05.03 xCCH stage-isolation test vector\n")
        f.write("# Fixed pseudo-random 228-bit u (u[224:228]=tail=0).\n")
        f.write("# TEST A (interleaver): feed burst0..3 into YOUR deinterleaver,\n")
        f.write("#   compare the 456-bit result against the three candidates below:\n")
        f.write("#     == cB_correct       -> interleaver OK, bug is downstream\n")
        f.write("#     == cB_lsb_dropped   -> you never apply the ((k%8)>>2) term\n")
        f.write("#     == cB_lsb_inverted  -> that term is inverted / off-by-one\n")
        f.write("#     == none of them     -> different bug (burst order? half swap?)\n")
        f.write("# TEST B (Viterbi/lane): feed cB_correct into YOUR Viterbi,\n")
        f.write("#   compare the 228-bit result against u. Mismatch with cB correct\n")
        f.write("#   => c0/c1 swap, wrong polynomials, or tail handling.\n\n")
        f.write("u_228           = " + s(u_tv) + "\n\n")
        f.write("burst0_114      = " + s(iB_tv[0:114]) + "\n")
        f.write("burst1_114      = " + s(iB_tv[114:228]) + "\n")
        f.write("burst2_114      = " + s(iB_tv[228:342]) + "\n")
        f.write("burst3_114      = " + s(iB_tv[342:456]) + "\n\n")
        f.write("cB_correct      = " + s(cB_correct) + "\n\n")
        f.write("cB_lsb_dropped  = " + s(cB_drop) + "\n\n")
        f.write("cB_lsb_inverted = " + s(cB_inv) + "\n")

    dd = sum(a != b for a, b in zip(cB_correct, cB_drop))
    di = sum(a != b for a, b in zip(cB_correct, cB_inv))
    print(f"test vector written: cB differs from correct in "
          f"{dd}/456 (dropped) and {di}/456 (inverted) positions\n")

    # ---- how big / what shape is the LSB bug ----
    diff = [k for (k, B, j, idx) in rows
            if (B * 114 + j) != (B * 114 + 2 * ((49 * k) % 57) + (1 - ((k % 8) >> 2)))]
    print(f"LSB-inverted bug moves {len(diff)}/456 coded bits to a wrong burst slot")
    print("  -> every k is displaced by exactly +/-1 in j (even<->odd swap),")
    print("     within its own burst, so the burst assignment (k&3) stays right.\n")

    print("spot check (k : burst, pos):")
    for k in [0, 1, 2, 3, 4, 5, 6, 7, 8, 57, 114, 228, 455]:
        B = k & 3
        j = 2 * ((49 * k) % 57) + ((k % 8) >> 2)
        print(f"  k={k:3d} -> burst {B}, pos {j:3d}   (lane {'G1/c1' if k & 1 else 'G0/c0'})")
