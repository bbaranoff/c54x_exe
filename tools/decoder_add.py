#!/usr/bin/env python3
# decoder_add.py - decode avec libgsm les trames FR notees par montant.c et cherche un ton.
#
#   /dev/shm/calypso_add_dl.bin   parole descendante rendue par la ROM (a_dd), convertie TI -> FR
#   /dev/shm/calypso_add_ul.bin   parole montante (a_du) : type 1 = brute TI, type 0 = convertie FR
#
# Enregistrements de 48 octets : fn u32, type u8, n u8, etat u16, err u16, [12..12+n] donnees.
# Le descendant a_dd est bit-exact avec ce que la BTS emet (comparer_parole.py) : s'il ne
# porte pas le ton injecte au micro, c'est le montant (ou l'echo) qui l'a perdu.
#
# Usage : tools/decoder_add.py [dl|ul] [--ton 1000] [--out fichier.raw]
import argparse, ctypes, math, struct, sys
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("sens", nargs="?", default="dl", choices=["dl", "ul"])
ap.add_argument("--ton", type=float, default=1000.0, help="frequence cherchee (Hz)")
ap.add_argument("--out", help="PCM s16le 8 kHz decode")
a = ap.parse_args()
path = "/dev/shm/calypso_add_%s.bin" % a.sens
raw = open(path, "rb").read()
recs = []
for o in range(0, len(raw) - 47, 48):
    fn, typ, n, etat, err = struct.unpack_from("<IBBHH", raw, o)
    recs.append((fn, typ, n, etat, err, raw[o + 12:o + 12 + n]))
fr = [r for r in recs if r[1] == 0 and r[2] == 33]
print("%s : %d enregistrements, %d trames FR (fn %d..%d), nibble de tete %s" % (
    path, len(recs), len(fr), fr[0][0] if fr else 0, fr[-1][0] if fr else 0,
    sorted({hex(x[5][0] >> 4) for x in fr})))
g = ctypes.CDLL("libgsm.so.1"); g.gsm_create.restype = ctypes.c_void_p
g.gsm_decode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_short)]
h = g.gsm_create()
pcm = []
for _, _, _, _, _, d in fr:
    out = (ctypes.c_short * 160)()
    g.gsm_decode(h, (ctypes.c_ubyte * 33)(*d), out)
    pcm.extend(out)
x = np.array(pcm, dtype=float); sr = 8000
if a.out:
    open(a.out, "wb").write(np.array(pcm, dtype="<i2").tobytes())
print(" sec    rms  raie(Hz)  %%dans %.0f+-50  fn" % a.ton)
for s in range(int(len(x) // sr)):
    v = x[s * sr:(s + 1) * sr]
    rms = math.sqrt(np.mean(v * v)) + 1e-9
    f = np.fft.rfftfreq(len(v), 1 / sr); sp = np.abs(np.fft.rfft(v * np.hanning(len(v)))) ** 2
    k = int(np.argmax(sp[1:])) + 1; band = (f >= a.ton - 50) & (f <= a.ton + 50)
    print("%4d %6.0f %9.0f %12.0f  %d" % (s, rms, f[k], 100 * sp[band].sum() / (sp[1:].sum() + 1e-9), fr[min(s * 50, len(fr) - 1)][0]))
