#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""layer1_tester.py - l'interface layer1 (ARM) <-> ROM du DSP Calypso, champ par champ.

Ce que l'outil fait (sorties PASS / FAIL / ECART / NON-EXERCE / INFO) :

 1. EXTRAIRE. Les structures de l'API RAM telles que COMPILEES pour le firmware :
    - DWARF de layer1.highram.elf (gdb-multiarch « ptype /o ») : offset et taille de
      chaque champ de T_DB_MCU_TO_DSP, T_DB_DSP_TO_MCU, T_NDB_MCU_DSP, T_PARAM_MCU_DSP ;
    - offsetof() sur dsp_api.h + l1_environment.h (memes #define : DSP=36, CHIPSET=12,
      ANLG_FAM=2) compile sur l'hote : les deux doivent coincider champ par champ ;
    - BASE_API_* (en-tete) contre l'initialiseur dsp_api de l'ELF, dsp_params (PARAM),
      macros B_*, *_DSP_TASK, BL_* (calypso/dsp.c).
    Chaque champ -> adresse ARM, offset octets dans l'API RAM, adresse mot DSP
    (0x0800 + octets/2).
 2. RECOUPER avec les constantes de qosmo (calypso_api.h, calypso_fbsb.h,
    calypso_arm2dsp.c, calypso_c54x.h, calypso_bsp.c...) et de c54x_exe (pont.c,
    montant.c, rejouer.c, tools/rejeu_banc.c...) : macros nommees (table d'unites
    ci-dessous, valeurs jamais recopiees) + balayage des lignes qui citent un champ
    a cote d'une adresse en dur (SUSPECT si l'adresse tombe sur un autre champ).
 3. FONCTIONS. Balayage statique des fonctions de la layer1 (layer1/*.c,
    calypso/dsp.c) : quels champs chacune ecrit / lit. Avec un rejeu (--rejeu ou
    --mesure), pour chaque champ et chaque fonction : l'ARM y ecrit-il (enregistrements
    'A'), des valeurs coherentes (codes de tache, burst_d/u, d_ctrl_system, d_ctrl_tch,
    d_fn, a_a5fn, d_dsp_page, B_BLUD...), la ROM le lit-elle (crochet de
    tools/layer1_tester.c), la ROM produit-elle ce que la layer1 lit ; page W
    ecrite = page annoncee par d_dsp_page ; echo page W -> page R ; PARAM et
    initialisations NDB de l'etat 'S' = valeurs du firmware.
 4. CARTE (--md, alias « api_carte ») : docs/carte-memoire.md, tables generees +
    significations de tools/layer1_tester.notes.tsv et adresses internes de
    tools/layer1_tester.internes.tsv (editables a la main, une source par ligne).

    make layer1_tester
    python3 tools/layer1_tester.py                          # 1-3 (statique)
    python3 tools/layer1_tester.py --rejeu FICHIER          # + mesure (≈ 1 min)
    python3 tools/layer1_tester.py --rejeu FICHIER --md docs/carte-memoire.md
    python3 tools/layer1_tester.py --mesure m.tsv --journal j.tsv --enreg FICHIER --md ...
    python3 tools/layer1_tester.py api_carte --rejeu FICHIER   # = --md docs/carte-memoire.md

Code de sortie 1 s'il y a au moins un FAIL. Lecture seule partout (firmware, qosmo,
enregistrement) ; n'ecrit que --md, --cache et les fichiers temporaires.
"""
import argparse
import collections
import datetime
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile

ICI = os.path.dirname(os.path.abspath(__file__))
RACINE = os.path.dirname(ICI)                      # c54x_exe
GSM = os.path.dirname(RACINE)                      # /opt/GSM
API_ARM = 0xFFD00000
API_DSP = 0x0800
API_MOTS = 0x2000

# ------------------------------------------------------------------ resultats
class Resultats:
    ORDRE = ['FAIL', 'SUSPECT', 'ECART', 'NON-EXERCE', 'PASS', 'INFO']

    def __init__(self, bavard=False):
        self.lignes = []
        self.bavard = bavard

    def __call__(self, statut, cat, sujet, detail='', source=''):
        self.lignes.append((statut, cat, sujet, detail, source))
        if self.bavard or statut in ('FAIL', 'SUSPECT', 'ECART'):
            print('%-10s %-10s %-34s %s%s' % (statut, cat, sujet, detail, ('  [%s]' % source) if source else ''))

    def compte(self, cat=None):
        c = collections.Counter(l[0] for l in self.lignes if cat is None or l[1] == cat)
        return c

    def de(self, cat):
        return [l for l in self.lignes if l[1] == cat]


def rel(p):
    """Chemin relatif a /opt/GSM pour les sources citees."""
    p = os.path.abspath(p)
    return os.path.relpath(p, GSM) if p.startswith(GSM + os.sep) else p


def lancer(cmd, **kw):
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, **kw)
    if r.returncode != 0:
        raise RuntimeError('%s : %s' % (' '.join(cmd[:3]), r.stderr[-2000:]))
    return r.stdout


# ------------------------------------------------------- evaluation C minimale
_SUFFIXE = re.compile(r'\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b')
_CAST = re.compile(r'\(\s*(?:const\s+)?(?:volatile\s+)?(?:unsigned\s+|signed\s+)?'
                   r'(?:u?int(?:8|16|32|64)_t|int|long|short|char|API|API_SIGNED)\s*\*?\s*\)')


def c_vers_py(expr):
    e = _CAST.sub('', expr)
    e = _SUFFIXE.sub(r'\1', e)
    e = e.replace('&&', ' and ').replace('||', ' or ').replace('!', ' not ').replace(' not =', '!=')
    # ternaire a ? b : c (un seul niveau)
    m = re.match(r'^\s*\(?\s*(.+?)\s*\?\s*(.+?)\s*:\s*(.+?)\s*\)?\s*$', e)
    if m and '?' in e:
        e = '((%s) if (%s) else (%s))' % (m.group(2), m.group(1), m.group(3))
    e = re.sub(r'(?<![\w])/(?![/=])', '//', e)
    return e


def evaluer(expr, env, prof=0):
    """Evalue une expression constante C ; env : nom -> (texte, params) ou entier."""
    if prof > 20:
        raise ValueError('recursion')
    expr = re.sub(r'/\*.*?\*/', '', expr)
    expr = re.sub(r'//.*', '', expr).strip()
    if not expr:
        raise ValueError('vide')

    def sub(m):
        n = m.group(0)
        if n in env:
            v = env[n]
            if isinstance(v, int):
                return str(v)
            texte, params = v
            if params is not None:
                raise ValueError('macro a parametres ' + n)
            return '(%d)' % evaluer(texte, env, prof + 1)
        if re.match(r'^(0[xX][0-9a-fA-F]+|\d+)$', n):
            return n
        if n in ('and', 'or', 'not', 'if', 'else'):
            return n
        raise ValueError('inconnu ' + n)
    e = c_vers_py(expr)
    e = re.sub(r'\b[A-Za-z_]\w*\b|\b0[xX][0-9a-fA-F]+\b|\b\d+\b', sub, e)
    if re.search(r'[^\s\d()+\-*/%<>&|^~xXa-fA-F,.]|\b(?!(if|else|and|or|not)\b)[g-wyzG-WYZ_]\w*', e):
        pass
    v = eval(e, {'__builtins__': {}}, {})
    if isinstance(v, bool):
        v = int(v)
    if not isinstance(v, int):
        raise ValueError('pas entier')
    return v


_DEFINE = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)(\(([^)]*)\))?(?:[ \t]+(.*?))?\s*$')


def lire_defines(chemin):
    """nom -> (texte, params|None, ligne) ; joint les lignes continuees par \\."""
    res = {}
    try:
        lignes = open(chemin, errors='replace').read().split('\n')
    except OSError:
        return res
    i = 0
    while i < len(lignes):
        l = lignes[i]
        n0 = i + 1
        while l.endswith('\\') and i + 1 < len(lignes):
            i += 1
            l = l[:-1] + ' ' + lignes[i]
        m = _DEFINE.match(l)
        if m:
            texte = (m.group(4) or '').strip()
            texte = re.sub(r'/\*.*?\*/', '', texte)
            texte = re.sub(r'//.*', '', texte).strip()
            params = [p.strip() for p in m.group(3).split(',')] if m.group(2) else None
            res.setdefault(m.group(1), (texte, params, n0))
        i += 1
    return res


def valeur_macro(defs, nom, args=None):
    texte, params, _ = defs[nom]
    env = {k: (v[0], v[1]) for k, v in defs.items()}
    if params is not None:
        for p, a in zip(params, args or []):
            texte = re.sub(r'\b%s\b' % re.escape(p), '(%d)' % a, texte)
    return evaluer(texte, env)


# ------------------------------------------------------------------ extraction
class Interface:
    """Les structures API de la layer1, telles que compilees."""
    TYPES = ['T_DB_MCU_TO_DSP', 'T_DB_DSP_TO_MCU', 'T_NDB_MCU_DSP', 'T_PARAM_MCU_DSP']
    ZONES = [('W0', 'T_DB_MCU_TO_DSP', 'BASE_API_W_PAGE_0'), ('W1', 'T_DB_MCU_TO_DSP', 'BASE_API_W_PAGE_1'),
             ('R0', 'T_DB_DSP_TO_MCU', 'BASE_API_R_PAGE_0'), ('R1', 'T_DB_DSP_TO_MCU', 'BASE_API_R_PAGE_1'),
             ('NDB', 'T_NDB_MCU_DSP', 'BASE_API_NDB'), ('PARAM', 'T_PARAM_MCU_DSP', 'BASE_API_PARAM')]

    def __init__(self, fw, elf, res, tmp):
        self.fw, self.elf, self.res, self.tmp = fw, elf, res, tmp
        self.inc = os.path.join(fw, 'include')
        self.hdr = os.path.join(self.inc, 'calypso', 'dsp_api.h')
        self.env_h = os.path.join(self.inc, 'calypso', 'l1_environment.h')
        self.dsp_c = os.path.join(fw, 'calypso', 'dsp.c')
        self.disp = {}          # type -> [(nom, off, taille, n, signe)]
        self.taille_elf = {}
        self.taille_hote = {}
        self.l1 = {}            # macro layer1 -> valeur (compilee)
        self.l1_src = {}        # macro -> fichier:ligne
        self.dspc = {}          # macros de calypso/dsp.c
        self.params = {}        # dsp_params (ELF)
        self.dsp_api_init = {}
        self.commentaires = {}  # (type, champ) -> texte
        self.champs = []        # dicts
        self.par_mot = collections.defaultdict(list)

    # -- DWARF
    def _gdb(self):
        cmds = []
        for t in self.TYPES:
            cmds += ['-ex', 'echo @@%s\\n' % t, '-ex', 'ptype /o %s' % t, '-ex', 'print sizeof(%s)' % t]
        cmds += ['-ex', 'echo @@params\\n', '-ex', 'print/x dsp_params', '-ex', 'echo @@api\\n', '-ex', 'print/x dsp_api']
        gdb = shutil.which('gdb-multiarch') or shutil.which('gdb')
        out = lancer([gdb, '-batch', '-nx'] + cmds + [self.elf])
        blocs = re.split(r'^@@(\w+)$', out, flags=re.M)
        for i in range(1, len(blocs) - 1, 2):
            nom, txt = blocs[i], blocs[i + 1]
            if nom in self.TYPES:
                ch = []
                for m in re.finditer(r'/\*\s*(\d+)\s*\|\s*(\d+)\s*\*/\s*(API_SIGNED|API)\s+(\w+)(?:\[(\d+)\])?;', txt):
                    ch.append((m.group(4), int(m.group(1)), int(m.group(2)), int(m.group(5) or 1), m.group(3) == 'API_SIGNED'))
                self.disp[nom] = ch
                m = re.search(r'^\$\d+ = (\d+)', txt, re.M)
                self.taille_elf[nom] = int(m.group(1)) if m else None
            elif nom == 'params':
                self.params = self._gdb_struct(txt)
            elif nom == 'api':
                self.dsp_api_init = {k: v[0] for k, v in self._gdb_struct(txt).items()}

    @staticmethod
    def _gdb_struct(txt):
        txt = txt[txt.find('{') + 1: txt.rfind('}')]
        res = {}
        for m in re.finditer(r'(\w+) = (\{[^}]*\}|-?0x[0-9a-fA-F]+|-?\d+)', txt):
            v = m.group(2)
            if v.startswith('{'):
                vals = []
                for tok in v[1:-1].split(','):
                    tok = tok.strip()
                    r = re.match(r'(-?0x[0-9a-fA-F]+|-?\d+)(?:\s*<repeats (\d+) times>)?', tok)
                    if r:
                        vals += [int(r.group(1), 0)] * int(r.group(2) or 1)
                res[m.group(1)] = vals
            else:
                res[m.group(1)] = [int(v, 0)]
        return res

    # -- offsetof hote + macros
    def _hote(self):
        defs = {}
        for f in (self.env_h, self.hdr):
            for n, (t, p, l) in lire_defines(f).items():
                if p is None and t and not n.startswith('_'):
                    defs.setdefault(n, '%s:%d' % (rel(f), l))
        src = ['#include <stdio.h>', '#include <stddef.h>', '#include "dsp_api.h"', 'int main(void) {']
        for t in self.TYPES:
            for (nom, off, taille, n, sg) in self.disp[t]:
                src.append('printf("F %s %s %%zu %%zu\\n", offsetof(%s, %s), sizeof(((%s *)0)->%s));' % (t, nom, t, nom, t, nom))
            src.append('printf("S %s %%zu\\n", sizeof(%s));' % (t, t))
        for n in sorted(defs):
            src.append('#ifdef %s\nprintf("M %s %%lld\\n", (long long)(%s));\n#endif' % (n, n, n))
        src.append('return 0; }')
        c = os.path.join(self.tmp, 'offsets.c')
        open(c, 'w').write('\n'.join(src) + '\n')
        exe = os.path.join(self.tmp, 'offsets')
        r = subprocess.run(['gcc', '-w', '-I', os.path.join(self.inc, 'calypso'), '-I', self.inc, '-o', exe, c],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        if r.returncode != 0:
            # une macro non entiere casse la compilation : on retire les M fautives
            mauvaises = set(re.findall(r"error: [^\n]*?'(\w+)'", r.stderr))
            src = [s for s in src if not any(('M %s ' % m) in s for m in mauvaises)]
            open(c, 'w').write('\n'.join(src) + '\n')
            lancer(['gcc', '-w', '-I', os.path.join(self.inc, 'calypso'), '-I', self.inc, '-o', exe, c])
        hote = {}
        for l in lancer([exe]).split('\n'):
            p = l.split()
            if not p:
                continue
            if p[0] == 'F':
                hote[(p[1], p[2])] = (int(p[3]), int(p[4]))
            elif p[0] == 'S':
                self.taille_hote[p[1]] = int(p[2])
            elif p[0] == 'M':
                self.l1[p[1]] = int(p[2])
                self.l1_src[p[1]] = defs[p[1]]
        return hote

    def _commentaires(self):
        """Commentaires de chaque champ, depuis l'en-tete PREPROCESSE (branche active)."""
        c = os.path.join(self.tmp, 'pp.c')
        open(c, 'w').write('#include "dsp_api.h"\n')
        pp = lancer(['gcc', '-E', '-C', '-P', '-I', os.path.join(self.inc, 'calypso'), '-I', self.inc, c])
        for m in re.finditer(r'typedef\s+struct\s*\{(.*?)\}\s*(\w+)\s*;', pp, re.S):
            corps, t = m.group(1), m.group(2)
            dernier = None
            for l in corps.split('\n'):
                f = re.search(r'\b(API_SIGNED|API)\s+(\w+)\s*(\[[^\]]*\])?\s*;(.*)$', l)
                if f:
                    dernier = f.group(2)
                    com = f.group(4).strip()
                    com = re.sub(r'^//\s*|^/\*\s*|\s*\*/$', '', com)
                    self.commentaires[(t, dernier)] = com
                elif dernier and re.match(r'^\s*//', l):
                    self.commentaires[(t, dernier)] += ' ' + re.sub(r'^\s*//\s*', '', l).strip()
                elif re.match(r'^\s*$', l) or re.match(r'^\s*//', l):
                    continue
                else:
                    dernier = None

    def _dsp_c(self):
        d = lire_defines(self.dsp_c)
        for n in d:
            try:
                self.dspc[n] = valeur_macro(d, n)
            except Exception:
                pass
        self.dspc_src = {n: '%s:%d' % (rel(self.dsp_c), d[n][2]) for n in d}

    def extraire(self):
        r = self.res
        self._gdb()
        hote = self._hote()
        self._commentaires()
        self._dsp_c()
        # 1. offsets ELF == offsetof hote
        nb = collections.Counter()
        for t in self.TYPES:
            for (nom, off, taille, n, sg) in self.disp[t]:
                h = hote.get((t, nom))
                if h == (off, taille):
                    nb[t] += 1
                    if r.bavard:
                        r('PASS', 'extraction', '%s.%s' % (t, nom), 'offset %d taille %d (ELF = offsetof)' % (off, taille))
                else:
                    r('FAIL', 'extraction', '%s.%s' % (t, nom), 'ELF offset %d taille %d, offsetof hote %s' % (off, taille, h))
            r('PASS', 'extraction', t, '%d champs : offsets DWARF de l\'ELF = offsetof(dsp_api.h, DSP=%s)' % (nb[t], self.l1.get('DSP')),
              rel(self.elf))
            te, th = self.taille_elf.get(t), self.taille_hote.get(t)
            if te != th:
                dernier = self.disp[t][-1]
                r('ECART', 'extraction', 'sizeof(%s)' % t,
                  'ELF %s octets, hote %s : bourrage de fin (dernier champ %s finit a %d) ; offsets inchanges' %
                  (te, th, dernier[0], dernier[1] + dernier[2]), rel(self.elf))
        # 2. bases : en-tete == initialiseur dsp_api de l'ELF
        for z, t, b in self.ZONES:
            if b not in self.l1:
                r('FAIL', 'extraction', b, 'absent de dsp_api.h')
        for cle, base in (('ndb', 'BASE_API_NDB'), ('db_r', 'BASE_API_R_PAGE_0'), ('db_w', 'BASE_API_W_PAGE_0'), ('param', 'BASE_API_PARAM')):
            v = self.dsp_api_init.get(cle)
            st = 'PASS' if v == self.l1.get(base) else 'FAIL'
            r(st, 'extraction', 'dsp_api.%s' % cle, 'ELF %s, %s = 0x%08x' % (hex(v) if v is not None else None, base, self.l1.get(base, 0)),
              '%s ; %s' % (rel(self.dsp_c), self.l1_src.get(base, '')))
        # 3. table des champs
        for z, t, b in self.ZONES:
            base = self.l1[b]
            for (nom, off, taille, n, sg) in self.disp[t]:
                arm = base + off
                octet = arm - API_ARM
                mot = API_DSP + octet // 2
                c = dict(zone=z, type=t, nom=nom, off=off, taille=taille, n=n, signe=sg, arm=arm, octet=octet,
                         mot=mot, mots=list(range(mot, mot + taille // 2)),
                         commentaire=self.commentaires.get((t, nom), ''), trou='hole' in nom)
                self.champs.append(c)
                for i, w in enumerate(c['mots']):
                    self.par_mot[w].append((c, i))
        # 4. geometrie : pages disjointes, PARAM dans un trou du NDB, tout dans la fenetre DSP
        zones = {z: (self.l1[b] - API_ARM, self.l1[b] - API_ARM + self.taille_hote[t]) for z, t, b in self.ZONES}
        for a, b in (('W0', 'W1'), ('W1', 'R0'), ('R0', 'R1'), ('R1', 'NDB')):
            ok = zones[a][1] <= zones[b][0]
            r('PASS' if ok else 'FAIL', 'extraction', 'pages %s/%s' % (a, b),
              '%s 0x%03x..0x%03x, %s commence en 0x%03x' % (a, zones[a][0], zones[a][1] - 1, b, zones[b][0]))
        ndb0, ndb1 = zones['NDB']
        p0, p1 = zones['PARAM']
        trou = [c for c in self.champs if c['zone'] == 'NDB' and c['octet'] <= p0 < c['octet'] + c['taille']]
        if trou:
            tc = trou[0]
            utile = self.disp['T_PARAM_MCU_DSP'][-1]
            fin_utile = p0 + utile[1] + utile[2]
            ok = tc['trou'] and fin_utile <= tc['octet'] + tc['taille']
            r('PASS' if ok else 'FAIL', 'extraction', 'PARAM dans le NDB',
              'PARAM (0x%03x..0x%03x utiles) recouvre NDB.%s (0x%03x..0x%03x)%s' %
              (p0, fin_utile - 1, tc['nom'], tc['octet'], tc['octet'] + tc['taille'] - 1,
               '' if ok else ' : le recouvrement deborde sur un champ utile'))
        fin = max(v[1] for v in zones.values())
        r('PASS' if fin <= API_MOTS * 2 else 'FAIL', 'extraction', 'fenetre DSP',
          'structures 0x000..0x%03x octets = mots DSP 0x0800..0x%04x (fenetre 0x0800..0x%04x)' %
          (fin - 1, API_DSP + (fin - 1) // 2, API_DSP + API_MOTS - 1))
        # 5. chargeur (calypso/dsp.c)
        base_ram = self.dspc.get('BASE_API_RAM')
        r('PASS' if base_ram == API_ARM else 'FAIL', 'extraction', 'BASE_API_RAM', '0x%08x' % (base_ram or 0), self.dspc_src.get('BASE_API_RAM', ''))
        api_size = self.dspc.get('API_SIZE')
        r('PASS' if api_size == API_MOTS else 'FAIL', 'extraction', 'API_SIZE', '0x%x mots' % (api_size or 0), self.dspc_src.get('API_SIZE', ''))
        self.bl = {}
        for n in ('BL_CMD_STATUS', 'BL_ADDR_LO', 'BL_ADDR_HI', 'BL_SIZE'):
            if n in self.dspc:
                a = self.dspc[n]
                self.bl[n] = API_DSP + (a - API_ARM) // 2
                dessous = [c['nom'] for c, i in self.par_mot.get(self.bl[n], [])]
                r('INFO', 'extraction', n, 'ARM 0x%08x = mot DSP 0x%04x (recouvre NDB %s)' % (a, self.bl[n], ','.join(dessous) or '-'),
                  self.dspc_src[n])
        # 6. adresse en dur de la layer1 elle-meme (dsp.c SC_CHKSUM_VER)
        m = re.search(r'#define\s+SC_CHKSUM_VER\s+\(BASE_API_W_PAGE_0\s*\+\s*\(2\s*\*\s*\((0x[0-9A-Fa-f]+)\s*-\s*0x800\)\)\)',
                      open(self.dsp_c).read())
        if m:
            w = int(m.group(1), 16)
            noms = [c['nom'] for c, i in self.par_mot.get(w, []) if c['zone'] == 'NDB']
            r('PASS' if noms == ['d_version_number2'] else 'FAIL', 'extraction', 'SC_CHKSUM_VER (dsp.c)',
              'mot DSP 0x%04x = NDB.%s (« dsp patch version »)' % (w, ','.join(noms)), rel(self.dsp_c))
        return self

    # -- recherche
    def champ(self, zone, nom):
        for c in self.champs:
            if c['nom'] == nom and (c['zone'] == zone or (zone in ('W', 'R') and c['zone'].startswith(zone))):
                return c
        return None

    def zone_base(self, z):
        for zz, t, b in self.ZONES:
            if zz == z:
                return self.l1[b]
        raise KeyError(z)

    def noms(self):
        return set(c['nom'] for c in self.champs)

    def au_mot(self, w):
        """Champ(s) au mot DSP w, PARAM avant les trous du NDB."""
        l = self.par_mot.get(w, [])
        return sorted(l, key=lambda ci: (ci[0]['trou'], ci[0]['zone'] == 'NDB'))
