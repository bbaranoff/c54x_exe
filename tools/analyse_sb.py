#!/usr/bin/env python3
# analyse_sb.py - resume des tentatives SB d un rejeu verbeux (c54x_exe --rejouer --verbeux) :
#   SCH presents (burst dans la fenetre), taux CRC OK, angle moyen OK/KO, TOA.
# Usage : tools/analyse_sb.py rejeu.log "etiquette"
import re,sys,collections
L=open(sys.argv[1],errors='replace').read().split('\n')
att=[]
for l in L:
    m=re.search(r'SBresp att=(\d) fn=(\d+) p51=(\d+) crc=(\w+) a_sch=(\w+) (\w+) (\w+) (\w+) (\w+) toa=(-?\d+) pm=(-?\d+) angle=(-?\d+) snr=(-?\d+) sb_cmd_fn=(\d+)',l)
    if m: att.append(dict(att=int(m.group(1)),crc=m.group(4),toa=int(m.group(10)),angle=int(m.group(12)),snr=int(m.group(13))))
pres=[a for a in att if a['snr']>1000]           # burst present dans la fenetre
ok=[a for a in pres if a['crc']=='OK']; ko=[a for a in pres if a['crc']!='OK']
ma=lambda g: (sum(a['angle'] for a in g)/len(g)) if g else float('nan')
print(f"{sys.argv[2]:28s} SCH presents={len(pres):3d} OK={len(ok):3d} ({100*len(ok)/max(1,len(pres)):3.0f}%)  angle OK={ma(ok):5.0f} KO={ma(ko):5.0f}  toa OK={collections.Counter(a['toa'] for a in ok).most_common(2)}")
