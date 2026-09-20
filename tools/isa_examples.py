#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""isa_examples.py - turn the SPRU172C worked examples into ISA test vectors.

WHY. The C54x core (qosmo l1-dsp) has been fixed one instruction at a time,
each time a probe on the ROM's FB/SB path pointed at one. The manual carries
about 240 "Before Instruction / After Instruction" examples, one oracle per
instruction. This script reads them out of hw/arm/calypso/doc/spru172c.md,
assembles each example with the binutils opcode table (the same table the
disassembler uses, so encoding and decoding agree) and writes a flat test file
that tools/isa_test.c replays against the core.

    tools/isa_examples.py > tools/isa_tests.txt
    make isa_test && ./isa_test tools/isa_tests.txt

Test file format (one record per example):
    T <n> <section> | <instruction as printed>
    W <word> [<word> [<word>]]         assembled instruction
    B <REG> <hex>       B M <addr> <hex>    state before (registers, data memory)
    A <REG> <hex>       A M <addr> <hex>    state after
    E
Examples that cannot be assembled (labels, far calls, ports) are written as
    S <n> <section> | <instruction> | <reason>
so the count of what is NOT covered stays visible.
"""
import io, os, re, sys

DOC   = "/opt/GSM/qosmo-dsp/hw/arm/calypso/doc/spru172c.md"
TABLE = "/opt/GSM/qosmo-dsp/hw/arm/calypso/doc/opcodes/tic54x-opc.c"

# ---------------------------------------------------------------- the table --
ENTREE = re.compile(
    r'\{\s*"([^"]+)"\s*,\s*(\d+)\s*,[^,]*,[^,]*,\s*(0x[0-9A-Fa-f]+)\s*,\s*(0x[0-9A-Fa-f]+)\s*,\s*\{([^}]*)\}')

def charger_table(chemin):
    src = io.open(chemin, encoding="utf-8", errors="replace").read()
    fin = src.find("tic54x_paroptab")
    if fin > 0:
        src = src[:fin]
    tab = []
    for m in ENTREE.finditer(src):
        nom, mots, op, masque, ops = m.group(1), int(m.group(2)), \
            int(m.group(3), 16), int(m.group(4), 16), m.group(5).strip()
        if nom == "???":
            continue
        ops = [o.strip() for o in ops.split(",") if o.strip()]
        tab.append((nom, mots, op, masque, ops))
    return tab

# ---------------------------------------------------------------- operands ---
MMR = {"IMR": 0, "IFR": 1, "ST0": 6, "ST1": 7, "AL": 8, "AH": 9, "AG": 10, "BL": 11,
       "BH": 12, "BG": 13, "T": 14, "TREG": 14, "TRN": 15, "SP": 0x18, "BK": 0x19,
       "BRC": 0x1A, "RSA": 0x1B, "REA": 0x1C, "PMST": 0x1D, "XPC": 0x1E}
for _i in range(8):
    MMR["AR%d" % _i] = 0x10 + _i

COND = {"UNC": 0x00, "AEQ": 0x45, "ANEQ": 0x44, "ALT": 0x43, "ALEQ": 0x47, "AGT": 0x46,
        "AGEQ": 0x42, "BEQ": 0x4D, "BNEQ": 0x4C, "BLT": 0x4B, "BLEQ": 0x4F, "BGT": 0x4E,
        "BGEQ": 0x4A, "AOV": 0x70, "ANOV": 0x60, "BOV": 0x78, "BNOV": 0x68,
        "TC": 0x30, "NTC": 0x20, "C": 0x0C, "NC": 0x08, "BIO": 0x03, "NBIO": 0x02}

SBITS = {"BRAF": (1, 15), "CPL": (1, 14), "XF": (1, 13), "HM": (1, 12), "INTM": (1, 11),
         "OVM": (1, 9), "SXM": (1, 8), "C16": (1, 7), "FRCT": (1, 6), "CMPT": (1, 5),
         "TC": (0, 12), "C": (0, 11), "OVA": (0, 10), "OVB": (0, 9)}

# indirect Smem modifiers -> MOD nibble (SPRU172C table 3-?; binutils tic54x-dis.c)
SMOD = {"": 0x0, "-": 0x1, "+": 0x2, "-0B": 0x4, "-0": 0x5, "+0": 0x6, "+0B": 0x7,
        "-%": 0x8, "-0%": 0x9, "+%": 0xA, "+0%": 0xB}
XMOD = {"": 0, "-": 1, "+": 2, "+0%": 3}

def norm(t):
    return t.replace("–", "-").replace("−", "-").replace("—", "-").strip()

def nombre(t):
    """TI numbers: 0FFFEh, 1234h, 10h, 248, -8, +3, #..., 15-12 (bit expression)."""
    t = norm(t).lstrip("#").strip()
    if re.fullmatch(r'[+-]?[0-9A-Fa-f]+[hH]', t):
        return int(t[:-1], 16)
    if re.fullmatch(r'[+-]?\d+', t):
        return int(t, 10)
    if re.fullmatch(r'0[xX][0-9A-Fa-f]+', t):
        return int(t, 16)
    m = re.fullmatch(r'(\d+)\s*-\s*(\d+)', t)
    if m:
        return int(m.group(1)) - int(m.group(2))
    raise ValueError("nombre: %r" % t)

class Smem:
    """Parsed single data-memory operand."""
    def __init__(self, txt):
        t = norm(txt)
        self.lk = None
        if t.startswith("*"):
            m = re.fullmatch(r'\*\(\s*([^)]+)\)', t)
            if m:                                  # *(lk) absolute
                self.code = 0x80 | (0xF << 3); self.lk = nombre(m.group(1)); return
            m = re.fullmatch(r'\*\+AR(\d)\(([^)]+)\)(%?)', t)
            if m:                                  # *+ARn(lk) [%]
                self.code = 0x80 | ((0xE if m.group(3) else 0xD) << 3) | int(m.group(1))
                self.lk = nombre(m.group(2)); return
            m = re.fullmatch(r'\*AR(\d)\(([^)]+)\)', t)
            if m:                                  # *ARn(lk)
                self.code = 0x80 | (0xC << 3) | int(m.group(1)); self.lk = nombre(m.group(2)); return
            m = re.fullmatch(r'\*\+AR(\d)', t)
            if m:
                self.code = 0x80 | (0x3 << 3) | int(m.group(1)); return
            m = re.fullmatch(r'\*AR(\d)(.*)', t)
            if m and m.group(2) in SMOD:
                self.code = 0x80 | (SMOD[m.group(2)] << 3) | int(m.group(1)); return
            raise ValueError("Smem: %r" % txt)
        if t.startswith("@"):
            t = t[1:]
        if t.upper() in MMR:                       # MMR by name = direct address
            self.code = MMR[t.upper()]; return
        v = nombre(t)
        if not 0 <= v <= 0x7F:
            raise ValueError("Smem direct hors 0..7F: %r" % txt)
        self.code = v

def xmem(txt):
    t = norm(txt)
    m = re.fullmatch(r'\*AR(\d)(.*)', t)
    if not m or m.group(2) not in XMOD:
        raise ValueError("Xmem: %r" % txt)
    n = int(m.group(1))
    if not 2 <= n <= 5:
        raise ValueError("Xmem AR%d hors AR2..AR5" % n)
    return (XMOD[m.group(2)] << 2) | (n - 2)

def conds(tokens):
    v = 0
    for c in tokens:
        c = norm(c).upper()
        if c not in COND:
            raise ValueError("cond: %r" % c)
        v |= COND[c]
    return v

# --------------------------------------------------------------- assembler ---
def split_operands(s):
    out, cur, depth = [], "", 0
    for ch in s:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip()); cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out

def est_acc(t):
    return norm(t).upper() in ("A", "B")

def assembler(tab, texte):
    """Returns list of words or raises ValueError."""
    texte = norm(texte.split(";")[0])
    m = re.match(r'([A-Za-z][A-Za-z0-9]*)\s*(.*)$', texte)
    if not m:
        raise ValueError("syntaxe")
    nom, reste = m.group(1).lower(), m.group(2)
    ops = split_operands(reste) if reste else []
    erreurs = []
    for (tnom, mots, opc, masque, types) in tab:
        if tnom != nom:
            continue
        try:
            return encoder(tnom, mots, opc, masque, types, list(ops))
        except ValueError as e:
            erreurs.append("%s%s: %s" % (tnom, types, e))
    raise ValueError("; ".join(erreurs) if erreurs else "mnemonique inconnu %s" % nom)

def encoder(nom, mots, opc, masque, types, ops):
    word = opc
    extra = []
    src = dst = None
    types = list(types)
    # optional operands are flagged OPT| in binutils; strip the flag but remember
    optional = [t.startswith("OPT|") for t in types]
    types = [t.replace("OPT|", "") for t in types]
    # implicit-operand types consume nothing from the text
    IMPLICIT = {"OP_None"}
    ti = 0
    for k, ty in enumerate(types):
        if ty in IMPLICIT:
            continue
        if ti >= len(ops):
            if optional[k]:
                continue
            raise ValueError("operande manquant pour %s" % ty)
        o = ops[ti]
        if ty in ("OP_Smem", "OP_Sind", "OP_Lmem", "OP_MMR"):
            if ty == "OP_MMR" and norm(o).upper() in MMR:
                word |= MMR[norm(o).upper()]
            else:
                sm = Smem(o)
                word |= sm.code
                if sm.lk is not None:
                    extra.append(sm.lk & 0xFFFF)
        elif ty == "OP_Xmem":
            word |= xmem(o) << 4
        elif ty == "OP_Ymem":
            word |= xmem(o)
        elif ty == "OP_SRC":
            if not est_acc(o): raise ValueError("src attendu, %r" % o)
            src = 1 if norm(o).upper() == "B" else 0
            word |= src << 9
        elif ty == "OP_SRC1":
            if not est_acc(o): raise ValueError("src attendu, %r" % o)
            word |= (1 if norm(o).upper() == "B" else 0) << 8
        elif ty == "OP_DST":
            if not est_acc(o): raise ValueError("dst attendu, %r" % o)
            dst = 1 if norm(o).upper() == "B" else 0
            word |= dst << 8
        elif ty in ("OP_lk", "OP_lku", "OP_pmad", "OP_dmad", "OP_PA"):
            extra.append(nombre(o) & 0xFFFF)
        elif ty == "OP_xpmad":
            v = nombre(o)
            word |= (v >> 16) & 0x7F
            extra.append(v & 0xFFFF)
        elif ty in ("OP_k8u", "OP_k8"):
            word |= nombre(o) & 0xFF
        elif ty == "OP_k9":
            word |= nombre(o) & 0x1FF
        elif ty == "OP_k5":
            word |= nombre(o) & 0x1F
        elif ty == "OP_k3":
            word |= nombre(o) & 0x7
        elif ty == "OP_SHIFT":
            v = nombre(o)
            if not -16 <= v <= 15: raise ValueError("SHIFT hors -16..15")
            word |= v & 0x1F
        elif ty == "OP_SHFT":
            v = nombre(o)
            if not 0 <= v <= 15: raise ValueError("SHFT hors 0..15")
            word |= v & 0xF
        elif ty == "OP_031":
            v = nombre(o)
            if not 0 <= v <= 31: raise ValueError("0..31")
            word |= v & 0x1F
        elif ty == "OP_16":
            if norm(o) != "16": raise ValueError("16 attendu")
        elif ty in ("OP_T", "OP_TS", "OP_ASM", "OP_DP", "OP_ARP", "OP_TRN", "OP_A", "OP_B"):
            attendu = {"OP_T": "T", "OP_TS": "TS", "OP_ASM": "ASM", "OP_DP": "DP", "OP_ARP": "ARP",
                       "OP_TRN": "TRN", "OP_A": "A", "OP_B": "B"}[ty]
            if norm(o).upper() != attendu: raise ValueError("%s attendu" % attendu)
        elif ty == "OP_RND":
            raise ValueError("OP_RND non gere")
        elif ty in ("OP_CC", "OP_CC3"):
            # remaining operands are all conditions
            word |= conds(ops[ti:])
            ti = len(ops)
            continue
        elif ty == "OP_CC2":
            u = norm(o).upper()
            if nom in ("saccd", "srccd", "strcd"):  # 4-bit accumulator condition, bits 3-0
                if u not in COND or not (COND[u] & 0x40): raise ValueError("cond acc attendue")
                word |= COND[u] & 0x0F
            else:                                    # CMPR CC, ARx : cc in bits 9-8
                v = nombre(o) if not u.isalpha() else {"EQ": 0, "LT": 1, "GT": 2, "NEQ": 3}[u]
                word |= (v & 3) << 8
        elif ty == "OP_ARX":
            m = re.fullmatch(r'AR(\d)', norm(o).upper())
            if not m: raise ValueError("ARx attendu")
            word |= int(m.group(1)) & 7
        elif ty in ("OP_MMRX", "OP_MMRY"):       # AR0..AR7 = 0..7, SP = 8 ; X bits 7-4, Y bits 3-0
            u = norm(o).upper()
            m = re.fullmatch(r'AR(\d)', u)
            v = int(m.group(1)) if m else (8 if u == "SP" else None)
            if v is None: raise ValueError("ARx/SP attendu")
            word |= v << (4 if ty == "OP_MMRX" else 0)
        elif ty == "OP_N":
            u = norm(o).upper()
            if u in SBITS:                         # RSBX SXM : N and SBIT from the name
                n, b = SBITS[u]; word |= (n << 9) | b
                ti += 1
                if ti < len(ops): raise ValueError("SBIT apres nom")
                return [word] + extra
            word |= (nombre(o) & 1) << 9
        elif ty == "OP_SBIT":
            u = norm(o).upper()
            word |= (SBITS[u][1] if u in SBITS else nombre(o)) & 0xF
        elif ty == "OP_BITC":
            word |= nombre(o) & 0xF
        elif ty == "OP_123":
            word |= ((nombre(o) - 1) & 3) << 8
        elif ty == "OP_12":
            word |= ((nombre(o) - 1) & 1) << 9
        else:
            raise ValueError("type %s non gere" % ty)
        ti += 1
    if ti != len(ops):
        raise ValueError("operandes en trop: %r" % ops[ti:])
    # optional dst omitted: dst = src
    if "OP_DST" in types and dst is None and src is not None:
        word |= src << 8
    if len(extra) + 1 != mots and not (len(extra) + 1 == mots + 1):
        # the table's word count does not include the Smem lk word
        raise ValueError("longueur %d != %d" % (len(extra) + 1, mots))
    return [word & 0xFFFF] + extra

# -------------------------------------------------------- example parsing ----
REG_OK = {"A", "B", "T", "TRN", "ASM", "C", "TC", "OVA", "OVB", "OVM", "SXM", "FRCT", "C16",
          "CMPT", "DP", "SP", "ARP", "BK", "BRC", "RSA", "REA", "PC", "XPC", "PMST", "ST0",
          "ST1", "IMR", "IFR", "INTM", "BRAF", "CPL", "XF", "HM", "AR0", "AR1", "AR2", "AR3",
          "AR4", "AR5", "AR6", "AR7", "TS"}
HEX = re.compile(r'^[0-9A-Fa-f]+$')

def valeur(tokens):
    """Value from the tokens following a register name. Returns (int, nbits)."""
    toks = [norm(t) for t in tokens]
    if len(toks) >= 3 and all(HEX.match(t) for t in toks[:3]) and len(toks[0]) == 2 \
            and len(toks[1]) == 4 and len(toks[2]) == 4:
        return int("".join(toks[:3]), 16), 40
    if toks and HEX.match(toks[0]) and len(toks[0]) == 4:
        return int(toks[0], 16), 16
    if toks and toks[0] in ("0", "1"):
        return int(toks[0]), 1
    if toks and toks[0].lower() == "x":
        return None, 1
    if toks and HEX.match(toks[0]) and len(toks[0]) in (1, 2, 3):
        return int(toks[0], 16), 16
    raise ValueError("valeur: %r" % toks)

def nettoyer_nom(t):
    return re.sub(r'[†‡†‡*]+$', '', norm(t)).upper()

def parser(doc):
    L = io.open(doc, encoding="utf-8", errors="replace").read().split("\n")
    tests, section = [], "?"
    i = 0
    while i < len(L):
        l = L[i]
        m = re.match(r'^Syntax\s+(?:\d+:\s*)?(\S.*)$', l)
        if m:
            section = norm(m.group(1)); i += 1; continue
        m = re.match(r'^\s*(?:\d+:\s*)?Syntax\s+(\S.*)$', l)
        if m:
            section = norm(m.group(1)); i += 1; continue
        m = re.match(r'^Example(?:\s+\d+)?\s+([A-Z][A-Za-z0-9]*(?:\s.*)?)$', l)
        if not m:
            i += 1; continue
        instr = m.group(1).strip()
        t = {"section": section, "instr": instr, "before": {}, "after": {},
             "mem_b": {}, "mem_a": {}, "pmem_b": {}, "pmem_a": {}, "warn": []}
        zone = "reg"
        j = i + 1
        while j < len(L):
            lj = L[j]
            if re.match(r'^Example', lj) or re.match(r'^\s*(?:\d+:\s*)?Syntax\s', lj) \
                    or re.match(r'^Syntax', lj) or re.match(r'^[A-Z][a-z]+\s{2,}', lj) and "Instruction" not in lj:
                break
            if "Data Memory" in lj:
                zone = "mem"; j += 1; continue
            if "Program Memory" in lj:
                zone = "pmem"; j += 1; continue
            if "Before Instruction" in lj:
                j += 1; continue
            toks = lj.split()
            if not toks:
                j += 1; continue
            if zone in ("mem", "pmem"):
                mm = re.findall(r'([0-9A-Fa-f]+)h\s+([0-9A-Fa-f]{4})', lj)
                if len(mm) >= 1:
                    tgt_b = t["mem_b"] if zone == "mem" else t["pmem_b"]
                    tgt_a = t["mem_a"] if zone == "mem" else t["pmem_a"]
                    tgt_b[int(mm[0][0], 16)] = int(mm[0][1], 16)
                    if len(mm) >= 2:
                        tgt_a[int(mm[1][0], 16)] = int(mm[1][1], 16)
                    j += 1; continue
                # a register row can follow memory rows on some pages
            name = nettoyer_nom(toks[0])
            if name in REG_OK:
                # split at the second occurrence of the name
                idx = [k for k, tk in enumerate(toks) if nettoyer_nom(tk) == name]
                try:
                    if len(idx) >= 2:
                        vb, _ = valeur(toks[idx[0] + 1:idx[1]])
                        va, _ = valeur(toks[idx[1] + 1:])
                        if vb is not None: t["before"][name] = vb
                        if va is not None: t["after"][name] = va
                    else:
                        t["warn"].append("ligne registre incomplete: %s" % lj.strip())
                except ValueError as e:
                    t["warn"].append(str(e))
            j += 1
        tests.append(t)
        i = j
    return tests

def main():
    tab = charger_table(TABLE)
    tests = parser(DOC)
    n_ok = n_skip = 0
    for k, t in enumerate(tests):
        instr = t["instr"]
        if not t["before"] and not t["after"] and not t["mem_a"]:
            continue                                # prose caught by the regex
        # symbolic pmad/dmad (COEFFS, DAT127...) : take the first Program Memory address
        if t["pmem_b"]:
            instr = re.sub(r'\b(COEFFS|DAT\d+)\b', "%04Xh" % min(t["pmem_b"]), instr)
        try:
            words = assembler(tab, instr)
        except ValueError as e:
            # dual-operand examples written with AR6/AR7: only AR2..AR5 can be encoded.
            # Rename AR6->AR4 and AR7->AR5 in the instruction AND the register rows.
            ren = {}
            if "AR6" in instr and "AR4" not in instr and "AR4" not in t["before"]: ren["AR6"] = "AR4"
            if "AR7" in instr and "AR5" not in instr and "AR5" not in t["before"]: ren["AR7"] = "AR5"
            if ren:
                instr2 = instr
                for a, b in ren.items(): instr2 = instr2.replace(a, b)
                try:
                    words = assembler(tab, instr2)
                    for a, b in ren.items():
                        for d in (t["before"], t["after"]):
                            if a in d: d[b] = d.pop(a)
                    instr = instr2 + " ;(" + ",".join("%s->%s" % kv for kv in ren.items()) + ")"
                    e = None
                except ValueError as e2:
                    e = e2
            if e is not None:
                print("S %d %s | %s | %s" % (k, t["section"], instr, str(e)[:120]))
                n_skip += 1
                continue
        print("T %d %s | %s" % (k, t["section"], instr))
        print("W " + " ".join("%04x" % w for w in words))
        for r, v in sorted(t["before"].items()):
            print("B %s %x" % (r, v))
        for a, v in sorted(t["mem_b"].items()):
            print("B M %04x %04x" % (a, v))
        for a, v in sorted(t["pmem_b"].items()):
            print("B P %04x %04x" % (a, v))
        for r, v in sorted(t["after"].items()):
            print("A %s %x" % (r, v))
        for a, v in sorted(t["mem_a"].items()):
            print("A M %04x %04x" % (a, v))
        for w in t["warn"]:
            print("# %s" % w)
        print("E")
        n_ok += 1
    sys.stderr.write("%d exemples assembles, %d non assembles\n" % (n_ok, n_skip))

if __name__ == "__main__":
    main()
