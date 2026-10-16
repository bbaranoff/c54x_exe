"""cmp_codeur.py <trace> : codeur FR de la ROM contre libgsm, sur les memes 160 echantillons."""
import ctypes, sys, numpy as np
gsm = ctypes.CDLL("libgsm.so.1"); oc = ctypes.CDLL("/usr/local/lib/libosmocodec.so.4")
gsm.gsm_create.restype = ctypes.c_void_p
order = (ctypes.c_uint16 * 260).in_dll(oc, "gsm610_bitorder")
def bit(b, i): return (b[i >> 3] >> (7 - (i & 7))) & 1
def ti_vers_fr(words):
    ti = bytearray()
    for w in words[3:20]: ti += bytes([w >> 8, w & 0xff])
    canon = bytearray(33)
    for i in range(260):
        si = i + 4 if i >= 182 else i
        if bit(ti, si): canon[order[i] >> 3] |= 0x80 >> (order[i] & 7)
    fr = bytearray(33); fr[0] = 0xd0 | (canon[0] >> 4)
    for i in range(1, 33): fr[i] = ((canon[i-1] << 4) | (canon[i] >> 4)) & 0xff
    return bytes(fr)
def explode(g, fr):
    p = (ctypes.c_short * 76)(); gsm.gsm_explode(ctypes.c_void_p(g), fr, p); return list(p)
NOMS = ['LAR1','LAR2','LAR3','LAR4','LAR5','LAR6','LAR7','LAR8'] + [f'{n}{k}' for k in range(4) for n in ['Nc','bc','Mc','xmax'] + [f'x{j}' for j in range(13)]]
so, paires = None, []
for l in open(sys.argv[1]):
    if l.startswith('SO'): so = [int(v) for v in l.split()[1:]]
    elif l.startswith('ENC') and so is not None:
        a = l.split(' A ')[1].split(' B ')[0].split()
        paires.append((so, [int(w, 16) for w in a])); so = None
genc = gsm.gsm_create(); gdec = gsm.gsm_create(); gx = gsm.gsm_create()
diff = np.zeros(76); n = 0; sortie_rom, sortie_ref, entree = [], [], []
for so, w in paires:
    x = (ctypes.c_short * 160)(*[max(-32768, min(32767, v * 2)) for v in so])
    ref = (ctypes.c_ubyte * 33)(); gsm.gsm_encode(ctypes.c_void_p(genc), x, ref)
    fr = ti_vers_fr(w)
    pr, pf = explode(gx, bytes(ref)), explode(gx, fr)
    if max(abs(v) for v in so) > 200:
        diff += np.array(pr) != np.array(pf); n += 1
    o = (ctypes.c_short * 160)(); gsm.gsm_decode(ctypes.c_void_p(gdec), (ctypes.c_ubyte * 33)(*fr), o); sortie_rom += list(o)
    entree += [v * 2 for v in so]
print(f"{len(paires)} trames, {n} avec du signal ; fraction de trames ou le parametre ROM != libgsm :")
print('  ' + '  '.join(f"{NOMS[i]}={diff[i]/max(n,1):.2f}" for i in range(76) if i < 12 or i % 17 in (8, 9, 10, 11)))
e = np.array(entree, float); r = np.array(sortie_rom, float)
def spec(x):
    S = np.zeros(513)
    for i in range(0, len(x) - 1024, 1024):
        s = x[i:i+1024]
        if np.sqrt((s**2).mean()) > 300: S += abs(np.fft.rfft(s * np.hanning(1024)))**2
    f = np.arange(513) * 8000 / 1024; t = S.sum() or 1
    return [round(100 * S[(f >= a) & (f < a + 1000)].sum() / t, 1) for a in (0, 1000, 2000, 3000)]
print('entree micro  energie % [0-1k,1-2k,2-3k,3-4k]', spec(e))
print('codeur ROM decode par libgsm               ', spec(r))
np.array(r, '<i2').tofile(sys.argv[1] + '.ul_rom_decode.raw')
