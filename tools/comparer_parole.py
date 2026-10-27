#!/usr/bin/env python3
# comparer_parole.py - downlink speech and FACCH: ROM against reference.
#
# Decides between "the bursts delivered to the DSP are bad" and "the DSP
# decodes badly" by decoding the same bursts with libosmocoding and comparing
# with what the ROM returned:
#
#   /dev/shm/calypso_tch_dl.bin   TCH bursts the BSP delivered to the DSP
#                                 (tick BE32, BTS fn BE32, 148 bits 0/1)
#   /dev/shm/calypso_add_dl.bin   frames the ROM returned (montant.c,
#                                 noter_dl: speech converted TI -> FR, FACCH)
#
# The bursts are deciphered with the session Kc ([a5] lines of dsp.log) and
# the COUNT of their own fn, then decoded by gsm0503_tch_fr_decode, as the
# bridge does. Each block is paired with the frame the ROM delivered right
# after it; for speech the wrong bits are counted per class (Ia 50, Ib 132,
# II 78), for the FACCH the 23 bytes are compared.
#
# Usage: tools/comparer_parole.py [--dsp-log /tmp/c54x-pont/dsp.log] [--kc HEX]
import argparse
import collections
import ctypes
import re
import struct
import sys

sys.path.insert(0, "/opt/GSM/osmo-operator")
from pont import gsm  # noqa: E402

FR = gsm.FR_BYTES
MAC = gsm.MACBLOCK_LEN


def lire_bursts(path):
    raw = open(path, "rb").read()
    out = []
    for o in range(0, len(raw) - 155, 156):
        tick, fn = struct.unpack(">II", raw[o:o + 8])
        out.append((tick, fn, bytes(raw[o + 8:o + 156])))
    return out


def lire_rom(path):
    raw = open(path, "rb").read()
    out = []
    for o in range(0, len(raw) - 47, 48):
        fn, typ, n, etat, err = struct.unpack_from("<IBBHH", raw, o)
        out.append((fn, typ, etat, err, bytes(raw[o + 12:o + 12 + n])))
    return out


def lire_kc(dsp_log):
    """(fn, Kc) of each new Kc seen by the A5 coprocessor."""
    kcs = []
    for line in open(dsp_log, errors="replace"):
        m = re.search(r"\[a5\] #\d+ A5/\d fn=(\d+) .*Kc=([0-9a-f]{16})", line)
        if m:
            fn, kc = int(m.group(1)), bytes.fromhex(m.group(2))
            if not kcs or kcs[-1][1] != kc:
                kcs.append((fn, kc))
    return kcs


def kc_pour(kcs, fn):
    best = None
    for f, kc in kcs:
        if f <= fn + 2:
            best = kc
    return best


def classes_fr():
    """class (0 Ia, 1 Ib, 2 II) of each bit of the standard FR format (260 bits
    after the 0xd header), from gsm610_bitorder: d[k] = s[bitorder[k]]."""
    lib = ctypes.CDLL("libosmocodec.so")
    order = (ctypes.c_uint16 * 260).in_dll(lib, "gsm610_bitorder")
    cl = [0] * 260
    for k in range(260):
        cl[order[k]] = 0 if k < 50 else (1 if k < 182 else 2)
    return cl


def bits260(fr):
    v = int.from_bytes(fr[:FR], "big")
    s = bin(v)[2:].zfill(8 * FR)
    return s[4:4 + 260]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dsp-log", default="/tmp/c54x-pont/dsp.log")
    ap.add_argument("--bursts", default="/dev/shm/calypso_tch_dl.bin")
    ap.add_argument("--rom", default="/dev/shm/calypso_add_dl.bin")
    ap.add_argument("--kc", help="Kc en hexa (sinon lu dans dsp.log)")
    ap.add_argument("-v", action="store_true", help="une ligne par trame")
    a = ap.parse_args()

    bursts = lire_bursts(a.bursts)
    rom = lire_rom(a.rom)
    if not bursts or not rom:
        sys.exit("rien a comparer : %d bursts, %d trames ROM" % (len(bursts), len(rom)))
    kcs = [(0, bytes.fromhex(a.kc))] if a.kc else lire_kc(a.dsp_log)
    offs = collections.Counter(t - f for t, f, _ in bursts)
    off = offs.most_common(1)[0][0]
    print("bursts : %d (fn BTS %d..%d, tick - fn = %s)" % (
        len(bursts), bursts[0][1], bursts[-1][1], dict(offs.most_common(3))))
    print("trames ROM : %d (parole %d, FACCH %d) ; Kc connus : %s" % (
        len(rom), sum(1 for r in rom if r[1] == 0), sum(1 for r in rom if r[1] == 1),
        [k.hex() for _, k in kcs]))

    # Reference decoding: a block ends on the 4th burst of a half-block.
    par_fn = {fn: b for _, fn, b in bursts}
    blocs = {}   # end fn -> (rc, data, errors)
    for fn in sorted(par_fn):
        if not gsm.is_tch_carrier(fn) or gsm.tch_burst_index(fn) % 4 != 3:
            continue
        seq, f = [fn], fn
        while len(seq) < 8:
            f -= 1
            while not gsm.is_tch_carrier(f):
                f -= 1
            seq.append(f)
        seq.reverse()
        if any(x not in par_fn for x in seq):
            continue
        kc = kc_pour(kcs, fn)
        coded = []
        for x in seq:
            b = par_fn[x]
            if kc:
                ks, _ = gsm.a5_keystream(1, kc, x)
                b = gsm.a5_xor(b, ks)
            coded.append(gsm.coded_from_burst(b))
        buf = (gsm.sbit * (8 * 116)).from_buffer_copy(gsm._soft(coded))
        out = (ctypes.c_uint8 * FR)()
        ne, nb = ctypes.c_int(), ctypes.c_int()
        rc = gsm._cod.gsm0503_tch_fr_decode(out, buf, 1, 0, ctypes.byref(ne), ctypes.byref(nb))
        blocs[fn] = (rc, bytes(out), ne.value)
    n_fr = sum(1 for r in blocs.values() if r[0] == FR)
    n_fa = sum(1 for r in blocs.values() if r[0] == MAC)
    print("reference : %d blocs, %d parole, %d FACCH, %d en echec" % (
        len(blocs), n_fr, n_fa, len(blocs) - n_fr - n_fa))

    cl = classes_fr()
    fins = sorted(blocs)
    stats = collections.defaultdict(list)
    facch = collections.Counter()
    for fn_rom, typ, etat, err, data in rom:
        cible = fn_rom - off
        cand = [f for f in fins if cible - 8 <= f <= cible]
        if not cand:
            stats["sans_reference"].append(0)
            continue
        f = cand[-1]
        rc, ref, ne = blocs[f]
        if typ == 0:
            if rc != FR:
                stats["parole_ref_pas_parole"].append(rc)
                continue
            b1, b2 = bits260(ref), bits260(data)
            diff = [i for i in range(260) if b1[i] != b2[i]]
            par = [sum(1 for i in diff if cl[i] == c) for c in range(3)]
            stats["parole"].append((len(diff), par, err, ne, etat))
            if a.v:
                print("  parole fn=%d ref_fin=%d etat=%04x err_rom=%d err_ref=%d faux=%d (Ia %d, Ib %d, II %d)"
                      % (fn_rom, f, etat, err, ne, len(diff), *par))
        else:
            fire = bool(etat & 0x0040)
            if rc == MAC:
                facch["ref FACCH, ROM %s" % ("FIRE KO" if fire else
                      ("identique" if data[:MAC] == ref[:MAC] else "DIFFERENTE"))] += 1
            else:
                facch["ref pas FACCH (rc=%d), ROM %s" % (rc, "FIRE KO" if fire else "ok")] += 1
            if a.v:
                print("  facch  fn=%d ref_fin=%d etat=%04x rc_ref=%d ROM=%s REF=%s" % (
                    fn_rom, f, etat, rc, data[:6].hex(" "), ref[:6].hex(" ")))

    p = stats["parole"]
    if p:
        tot = [x[0] for x in p]
        print("\nPAROLE : %d trames appariees" % len(p))
        print("  identiques aux bits pres : %d" % sum(1 for x in tot if x == 0))
        print("  bits faux par trame : moyenne %.1f, max %d (sur 260)" % (sum(tot) / len(tot), max(tot)))
        for c, nom in enumerate(("Ia (50)", "Ib (132)", "II (78)")):
            v = [x[1][c] for x in p]
            print("  classe %-8s : moyenne %.2f, trames touchees %d" % (nom, sum(v) / len(v), sum(1 for x in v if x)))
        print("  erreurs canal : ROM (a_dd[2]) moy %.1f | reference (Viterbi) moy %.1f" % (
            sum(x[2] for x in p) / len(p), sum(x[3] for x in p) / len(p)))
    for k in ("parole_ref_pas_parole", "sans_reference"):
        if stats[k]:
            print("  %s : %d" % (k, len(stats[k])))
    if facch:
        print("\nFACCH (trames a_fd vues par la ROM) :")
        for k, v in facch.most_common():
            print("  %-40s %d" % (k, v))


if __name__ == "__main__":
    main()
