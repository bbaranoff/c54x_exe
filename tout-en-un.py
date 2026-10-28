#!/usr/bin/env python3
"""tout-en-un.py - gathers every text file (sources, scripts, configs, docs,
logs) of one or more directories into ONE Markdown file, shortened WITHOUT LOSS:

  * identical files (same content): a single copy, the others refer to the
    first one;
  * identical consecutive lines: "line  ×N";
  * consecutive lines that differ only by numbers (timestamp, counter, fn...):
    a template where the constant numbers stay in place and the variable ones
    become ⟨1⟩ ⟨2⟩..., followed by the ordered list of values. Each line is
    rebuilt by putting the values back in order.
  * ANSI colour codes and carriage returns removed (the only, deliberate, loss).

    ./tout-en-un.py [dir...]          default: /opt/GSM/c54x_exe and the latest /root/c54x_exe-*
    SORTIE=/path.md  EXT="sh py c"    (restrict to extensions; default: every text file)
    SEUIL=3 (minimum group size)

  Skipped: .git, hidden and cache directories, binaries (ELF, images, .pyc, archives),
  backups (.bak*, ~, .orig, .avant-*), build-tic54x and binutils-*, ROM disassembly (.dis)
  and dumps (-dump.txt), signatures (.asc, .sha256), LICENSE/COPYING and previous
  outputs of this script.
"""
import glob
import hashlib
import os
import re
import subprocess
import sys
import time

EXT = os.environ.get("EXT", "").split()             # empty: every text file
SEUIL = int(os.environ.get("SEUIL", "3"))
FORMAT = os.environ.get("FORMAT", "qmd")           # qmd (Quarto) or md
SORTIE = os.environ.get("SORTIE") or "/root/c54x_exe-%s.%s" % (time.strftime("%Y%m%d-%H%M%S"), FORMAT)
AUTEUR = os.environ.get("AUTEUR", "Banc GSM émulé — banc-max")
IGNORES = {".git", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", "node_modules", ".venv", "venv",
           "rom", "roms",                              # ROM dumps (Calypso DSP): not ours, not in the bundle
           "osmo-operator-qmd",                        # the repository compiled by another tool: all duplicated
           "build-tic54x"}                             # binutils build tree (config.log, Makefile...)
IGNORES_PREFIXES = (".", "binutils-")                  # hidden directories (.avant-isa-*...), binutils sources
# Useless text files: DSP ROM disassembly and dumps, signatures/digests, .avant-* backups
INUTILES = re.compile(r"(\.dis|-dump\.txt|\.asc|\.sha256|\.avant-[\w-]+)$")
BINAIRES = set("png jpg jpeg gif bmp ico webp pdf zip gz tgz bz2 xz 7z tar bin elf o a so pyc pyo pyd "
               "wav mp3 ogg mp4 sqlite db img iso woff woff2 ttf otf".split())
DSP_DUMP = re.compile(r"(^calypso_dsp.*\.txt|^dsp-rom-.*\.txt|\.dis)$")   # DSP ROM dumps (not ours, megabytes of hex)
SAUVEGARDES = re.compile(r"(~|\.orig|\.rej|\.swp|\.bak(-\w+)?)$")
TITRE_SORTIE = (b"generator: tout-en-un.py",                         # a previous output of this script
                "title: \"Banc GSM émulé — dossier de run\"".encode())
HTML_SANS_COULEUR = 2_500_000                      # bytes: larger bundles get no syntax highlighting in html
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
TELNET = re.compile(rb"\xff[\xfb-\xfe].|\xff.")           # telnet negotiation (IAC) of VTY captures
CONTROLE = re.compile("[\x00-\x08\x0b\x0c\x0e-\x1f\x7f\ufffd]")  # control characters, undecodable bytes
NUM = re.compile(r"\d+")
# LaTeX preamble of the pdf format (xelatex). Without it: "Dimension too large" (framed
# measures the whole code block before splitting it), unwrapped code lines,
# Mermaid diagrams wider than the page, missing glyphs.
PDF_PREAMBULE = "".join("        %s\n" % l for l in r"""
% Code : police reduite, retour a la ligne (y compris dans les mots longs)
\usepackage{fvextra}
\fvset{fontsize=\small,breaklines,breakanywhere}
\RecustomVerbatimEnvironment{verbatim}{Verbatim}{fontsize=\small,breaklines,breakanywhere}
% Images (diagrammes Mermaid) : jamais plus larges que la ligne ni plus hautes que la page
\usepackage{adjustbox}
\usepackage{letltxmacro}
\LetLtxMacro\ORIGincludegraphics\includegraphics
\renewcommand{\includegraphics}[2][]{\adjustimage{#1,max width=\linewidth,max totalheight=.85\textheight}{#2}}
% Sans cadre : framed/snugshade mesure tout le bloc et plante ("Dimension too large")
\renewenvironment{Shaded}{\medskip}{\medskip}
% Glyphes absents de Latin Modern Roman -> DejaVu Sans
\usepackage{newunicodechar}
\newfontfamily\fallbackfont{DejaVu Sans}
\newunicodechar{↔}{{\fallbackfont ↔}}
\newunicodechar{⟨}{{\fallbackfont ⟨}}
\newunicodechar{⟩}{{\fallbackfont ⟩}}
\newunicodechar{≠}{{\fallbackfont ≠}}
\newunicodechar{∈}{{\fallbackfont ∈}}
\newunicodechar{ᵉ}{{\fallbackfont ᵉ}}
\newunicodechar{✓}{{\fallbackfont ✓}}
\newunicodechar{✗}{{\fallbackfont ✗}}
\newunicodechar{⚠}{{\fallbackfont ⚠}}
\newunicodechar{📱}{{\fallbackfont ☎}}
\newunicodechar{↩}{{\fallbackfont ↩}}
""".strip("\n").split("\n"))

LANG = {"sh": "bash", "bash": "bash", "py": "python", "md": "markdown", "qmd": "markdown", "mmd": "mermaid",
        "txt": "text", "log": "text", "c": "c", "h": "c", "cpp": "cpp", "cc": "cpp", "hpp": "cpp",
        "ini": "ini", "env": "ini", "service": "ini", "desktop": "ini", "toml": "toml", "yml": "yaml",
        "yaml": "yaml", "json": "json", "patch": "diff", "diff": "diff", "mak": "makefile", "mk": "makefile",
        "html": "html", "xml": "xml", "svg": "xml", "js": "javascript", "css": "css", "sql": "sql", "rs": "rust"}
NOMS_LANG = {"Makefile": "makefile", "GNUmakefile": "makefile", "Dockerfile": "dockerfile"}


def extension(nom):
    return nom.rsplit(".", 1)[-1].lower() if "." in nom[1:] else ""


def langage(chemin):
    """Highlighting language: extension, then file name, then shebang."""
    nom = os.path.basename(chemin)
    e = extension(nom)
    if e in LANG:
        return LANG[e]
    if nom in NOMS_LANG:
        return NOMS_LANG[nom]
    try:
        premiere = open(chemin, "rb").readline().decode("utf-8", "replace")
    except OSError:
        premiere = ""
    if premiere.startswith("#!"):
        if "python" in premiere:
            return "python"
        if re.search(r"\b(ba|z|da|k)?sh\b", premiere):
            return "bash"
    return "text"


def texte_legitime(chemin, nom):
    """Text file of ours: not a binary, a backup, a licence nor an output of this script."""
    if extension(nom) in BINAIRES or SAUVEGARDES.search(nom) or DSP_DUMP.search(nom) or INUTILES.search(nom) or nom in ("LICENSE", "COPYING"):
        return False
    if nom.startswith("tout-en-un-"):
        return False
    try:
        with open(chemin, "rb") as f:
            debut = f.read(8192)
    except OSError:
        return False
    if b"\0" in debut:                        # ELF, images, .pyc, archives...
        return False
    if any(t in debut for t in TITRE_SORTIE):
        return False
    if b'<meta name="generator" content="quarto-' in debut:   # page rendered by Quarto
        return False
    return True


def dossiers_par_defaut():
    runs = sorted(glob.glob("/root/c54x_exe-*"), key=os.path.getmtime)
    runs = [r for r in runs if os.path.isdir(r)]
    return ["/opt/GSM/c54x_exe"] + ([runs[-1]] if runs else [])




def rendu_quarto(racine, d):
    """d is the resource directory of a page rendered by Quarto (<name>_files next to <name>.html)."""
    return d.endswith("_files") and os.path.exists(os.path.join(racine, d[:-len("_files")] + ".html"))


def fichiers(dossier):
    for racine, dirs, noms in os.walk(dossier):
        dirs[:] = sorted(d for d in dirs if d not in IGNORES and not rendu_quarto(racine, d) and not d.startswith(IGNORES_PREFIXES))
        for n in sorted(noms):
            chemin = os.path.join(racine, n)
            if EXT and extension(n) not in EXT:
                continue
            if texte_legitime(chemin, n):
                yield chemin


LIGNE_MAX = int(os.environ.get("LIGNE_MAX", "1000"))


def couper(l):
    if len(l) <= LIGNE_MAX:
        return [l]
    return [l[i:i + LIGNE_MAX] + ("↩" if i + LIGNE_MAX < len(l) else "") for i in range(0, len(l), LIGNE_MAX)]


def compacter(lignes):
    """Returns (output lines, number of compacted groups)."""
    out, groupes, i, n = [], 0, 0, len(lignes)
    while i < n:
        cle = NUM.sub("\0", lignes[i])
        j = i + 1
        while j < n and NUM.sub("\0", lignes[j]) == cle:
            j += 1
        taille = j - i
        if taille < SEUIL:
            out.extend(lignes[i:j])
            i = j
            continue
        groupes += 1
        if "\0" not in cle:                      # exact repetition
            out.append("%s  ×%d" % (lignes[i], taille))
            i = j
            continue
        valeurs = [NUM.findall(l) for l in lignes[i:j]]
        nb = len(valeurs[0])
        varie = [len({v[k] for v in valeurs}) > 1 for k in range(nb)]
        # template: constant numbers put back in place, variable ones numbered ⟨k⟩
        morceaux, k, idx = cle.split("\0"), 0, 0
        gab = morceaux[0]
        for m in morceaux[1:]:
            if varie[k]:
                idx += 1
                gab += "⟨%d⟩" % idx
            else:
                gab += valeurs[0][k]
            gab += m
            k += 1
        out.append("%s  ×%d" % (gab, taille))
        if idx:
            tuples = [",".join(v[k] for k in range(nb) if varie[k]) for v in valeurs]
            # value lines of ~100 columns
            ligne, courant = [], "    ⟨⟩ ="
            for t in tuples:
                if len(courant) + len(t) + 3 > 110:
                    ligne.append(courant)
                    courant = "       "
                courant += " (" + t + ")"
            ligne.append(courant)
            out.extend(ligne)
        i = j
    return out, groupes


def synthese(tous):
    """Run, scale, failures, coverage summary: read from verdict.txt / couverture.txt."""
    lignes, vus = [], set()
    for f in tous:
        nom = os.path.basename(f)
        if nom not in ("verdict.txt", "couverture.txt"):
            continue
        run = os.path.basename(os.path.dirname(f))
        if (run, nom) in vus:                 # same run copied in two places
            continue
        vus.add((run, nom))
        try:
            txt = ANSI.sub("", open(f, "rb").read().decode("utf-8", "replace"))
        except OSError:
            continue
        for l in txt.splitlines():
            l = re.sub(r"\s+", " ", l.strip())
            if l.startswith("ELEMENT MAX"):
                lignes.append((0, "| %s | échelle | %s |" % (run, l)))
            elif l.startswith("bilan :"):
                lignes.append((2, "| %s | couverture | %s |" % (run, l[8:])))
            elif l.startswith("couverture couche 1"):
                lignes.append((1, "| %s | mode, date | %s |" % (run, l.split("—", 1)[-1].strip())))
            else:
                m = re.match(r"(\d+) (\S+) (ECHEC|SAUTE) ?(.*)", l)
                if m:
                    lignes.append((3, "| %s | barreau %s %s | %s %s |" % (run, m.group(1), m.group(2), m.group(3), m.group(4))))
    return [l for _, l in sorted(lignes, key=lambda x: x[0])]


MERMAID_OUVRE = "```{mermaid}" if FORMAT == "qmd" else "```mermaid"


def rendre_mmd(lignes):
    return "%s\n%s\n```" % (MERMAID_OUVRE, "\n".join(lignes))


_DEPOTS = {}


def depot_git(chemin):
    """(root, GitHub base URL, branch) of the git repository holding chemin, or None."""
    d = os.path.abspath(chemin)
    if not os.path.isdir(d):
        d = os.path.dirname(d)
    while not os.path.exists(os.path.join(d, ".git")):
        if d == "/":
            return None
        d = os.path.dirname(d)
    if d not in _DEPOTS:
        def git(*a):
            r = subprocess.run(["git", "-C", d] + list(a), capture_output=True, text=True)
            return r.stdout.strip() if r.returncode == 0 else ""
        url = git("config", "--get", "remote.origin.url")
        url = re.sub(r"^git@([^:]+):", r"https://\1/", url)
        url = re.sub(r"\.git$", "", url).rstrip("/")
        branche = git("rev-parse", "--abbrev-ref", "HEAD")
        _DEPOTS[d] = (d, url, branche if branche and branche != "HEAD" else "main") if url else None
    return _DEPOTS[d]


def adresse(chemin):
    """GitHub address of chemin (blob/<branch>/<path>), or the local path outside a repository."""
    g = depot_git(chemin)
    if not g:
        return chemin
    rel = os.path.relpath(os.path.abspath(chemin), g[0])
    return "%s/blob/%s/%s" % (g[1], g[2], "" if rel == "." else rel)


def reference(chemin):
    """File reference used in titles: a GitHub link when the file lives in a repository."""
    a = adresse(chemin)
    if not a.startswith("http"):
        return a
    # Quarto opens external links in a new tab (link-external-newwindow);
    # plain Markdown needs an HTML anchor for that
    return "<%s>" % a if FORMAT == "qmd" else '<a href="%s" target="_blank">%s</a>' % (a, a)


def nom_depot():
    """Name of the repository this script belongs to (title of the bundle)."""
    ici = os.path.dirname(os.path.realpath(__file__))
    g = depot_git(ici)
    return os.path.basename(g[0]) if g else os.path.basename(ici)


LIEN = re.compile(r"(?<!!)(\[[^\]]*\]\()([^)\s]+)((?:\s+\"[^\"]*\")?\))")
LIEN_REF = re.compile(r"^(\s*\[[^\]]+\]:\s*)(\S+)(.*)$")


def lien_github(cible, md):
    """Rewrite a link target of the .md file md to its GitHub address; others stay as they are."""
    if re.match(r"^([a-zA-Z][\w+.-]*:|#|<)", cible):
        return cible
    chemin, _, ancre = cible.partition("#")
    if not chemin:
        return cible
    if not chemin.startswith("/"):
        chemin = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(md)), chemin))
    a = adresse(chemin)
    if not a.startswith("http"):
        return cible
    return a + ("#" + ancre if ancre else "")


def lien_md(m, md):
    cible = lien_github(m.group(2), md)
    if FORMAT != "qmd" and cible.startswith("http"):  # plain Markdown: new tab via an HTML anchor
        return '<a href="%s" target="_blank">%s</a>' % (cible, m.group(1)[1:-2])
    return m.group(1) + cible + m.group(3)


IMAGE = re.compile(r"(!\[[^\]]*\]\()([^)\s]+)((?:\s+\"[^\"]*\")?\))")


def image_locale(cible, md):
    """Relative image of the .md file md: path relative to the output file, so the page
    embeds it offline (Quarto reads absolute paths as project-relative)."""
    if re.match(r"^([a-zA-Z][\w+.-]*:|/|#)", cible):
        return cible
    chemin = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(md)), cible))
    return os.path.relpath(chemin, os.path.dirname(os.path.abspath(SORTIE))) if os.path.exists(chemin) else cible


def liens_github(l, md):
    l = IMAGE.sub(lambda m: m.group(1) + image_locale(m.group(2), md) + m.group(3), l)
    l = LIEN.sub(lambda m: lien_md(m, md), l)
    return LIEN_REF.sub(lambda m: m.group(1) + lien_github(m.group(2), md) + m.group(3), l)


def rendre_md(lignes, md):
    out, dans_bloc, cloture = [], False, ""
    i = 0
    if lignes and lignes[0].strip() == "---":           # YAML front matter: shown as a block, not interpreted
        j = next((k for k in range(1, len(lignes)) if lignes[k].strip() in ("---", "...")), None)
        if j:
            out.append("```yaml"); out.extend(lignes[1:j]); out.append("```")
            i = j + 1
    while i < len(lignes):
        l = lignes[i]; i += 1
        m = re.match(r"^(\s*)(`{3,}|~{3,})(.*)$", l)
        if m and not dans_bloc:
            dans_bloc, cloture = True, m.group(2)
            info = m.group(3).strip()
            if info.startswith("{mermaid}") or info.startswith("mermaid"):
                l = m.group(1) + MERMAID_OUVRE
            elif info.startswith("{"):                     # ```{r ...} or {.bash filename=...}: language only,
                l = m.group(1) + m.group(2) + info.strip("{}").split()[0].split(",")[0].lstrip(".")  # no execution nor float
            out.append(l); continue
        if m and dans_bloc and m.group(2)[0] == cloture[0] and len(m.group(2)) >= len(cloture) and not m.group(3).strip():
            dans_bloc = False; out.append(l); continue
        if not dans_bloc:
            l = liens_github(l, md)
            h = re.match(r"^(#{1,6})\s", l)
            if h:
                l = "#" * min(6, len(h.group(1)) + 3) + l[len(h.group(1)):]
            elif l.strip() in ("---", "..."):             # no YAML block nor rule in the middle of the bundle
                l = "* * *"
        out.append(l)
    if dans_bloc:
        out.append(cloture)
    return "\n".join(out)


def main():
    dossiers = sys.argv[1:] or dossiers_par_defaut()
    vus, sections, table = {}, [], []
    total_in = total_out = 0
    # Order: test results (.txt .md .mmd: verdicts, coverage, reports, grafcets) first,
    # the bench files (sources, scripts, configs...) next, the .log files last
    CATEGORIES = (("Verdict et couverture", ()), ("Resultats des tests", ("txt", "md", "mmd", "tsv", "csv")),
                  ("Fichiers", None), ("Logs", ("log",)))          # Fichiers: everything else
    EN_TETE = ("verdict.txt", "couverture.txt")     # at the very top, in this order
    tous = []
    for d in dossiers:
        if not os.path.isdir(d):
            table.append("| (absent) %s | | | | |" % d)
            continue
        tous.extend(fichiers(d))
    def rang(f):
        if os.path.basename(f) in EN_TETE:
            return 0
        e = extension(os.path.basename(f))
        for i, (_, exts) in enumerate(CATEGORIES):
            if exts and e in exts:
                return i
        return 2                                  # Fichiers
    categorie_courante = None
    def cle(f):
        r = rang(f)
        return (r, EN_TETE.index(os.path.basename(f)) if r == 0 else 0, tous.index(f))
    for f in sorted(tous, key=cle):
        if rang(f) != categorie_courante:
            categorie_courante = rang(f)
            nom = CATEGORIES[categorie_courante][0] if categorie_courante < len(CATEGORIES) else "Autres"
            sections.append("## %s\n" % nom)
        if True:
            try:
                brut = open(f, "rb").read()
            except OSError as e:
                table.append("| %s | | | | illisible : %s |" % (f, e))
                continue
            texte = ANSI.sub("", TELNET.sub(b"", brut).decode("utf-8", "replace")).replace("\r", "")
            texte = CONTROLE.sub("", texte)
            lignes = texte.split("\n")
            if lignes and lignes[-1] == "":
                lignes.pop()
            # oversized lines (base64 blob, minified...): cut into LIGNE_MAX pieces ended
            # by ↩; otherwise xelatex measures the whole line and fails (Dimension too large)
            lignes = [m for l in lignes for m in couper(l)]
            h = hashlib.sha1(texte.encode()).hexdigest()
            total_in += len(brut)
            if h in vus:
                sections.append("### %s\n\nidentique à %s\n" % (reference(f), reference(vus[h])))
                continue
            vus[h] = f
            ext = extension(os.path.basename(f))
            if ext in ("md", "qmd", "mmd"):
                # Rendered, not quoted: .mmd files become Mermaid diagrams, .md files
                # are included as they are (headings demoted under the file title,
                # mermaid blocks rendered, {r}/{python} blocks neutralised). No compaction.
                total_out += len(texte) + 1
                sections.append("### %s\n\n%d octets, %d lignes\n\n%s\n"
                                % (reference(f), len(brut), len(lignes), rendre_md(lignes, f) if ext in ("md", "qmd") else rendre_mmd(lignes)))
                continue
            compact, groupes = compacter(lignes)
            total_out += sum(len(l) + 1 for l in compact)
            note = "%d groupes compactés" % groupes if groupes else ""
            # fence longer than any run of ` in the content: a ``` inside a .py
            # or a .md cannot close the block (pandoc would read what follows
            # as Markdown: "Could not fetch resource")
            plus_long = max((len(m) for m in re.findall(r"`+", "\n".join(compact))), default=0)
            cloture = "`" * max(3, plus_long + 1)
            sections.append("### %s\n\n%d octets, %d lignes → %d lignes%s\n\n%s%s\n%s\n%s\n"
                            % (reference(f), len(brut), len(lignes), len(compact), (" (%s)" % note) if note else "",
                               cloture, langage(f), "\n".join(compact), cloture))
    runs = sorted({os.path.basename(os.path.dirname(f)) for f in tous if os.path.basename(f) == "verdict.txt"})
    with open(SORTIE, "w") as o:
        o.write("---\n")
        o.write("title: \"%s\"\n" % nom_depot())
        o.write("generator: tout-en-un.py\n")
        o.write("subtitle: \"%s\"\n" % (", ".join(runs) if runs else ", ".join(dossiers)))
        o.write("author: \"%s\"\n" % AUTEUR)
        o.write("date: \"%s\"\n" % time.strftime("%Y-%m-%d %H:%M"))
        o.write("lang: fr\n")
        if FORMAT == "qmd":
            o.write("engine: markdown\n")
            o.write("code-annotations: false\n")  # a "<100>" in a comment is not an annotation     # pandoc only: knitr stops on ```{r} in included reports
        if FORMAT == "qmd":
            o.write("format:\n  html:\n    toc: true\n    toc-depth: 3\n    toc-location: left\n"
                    "    link-external-newwindow: true\n"
                    "    number-sections: true\n    embed-resources: true\n    theme: cosmo\n"
                    "    code-overflow: wrap\n    fontsize: 0.9em\n")
            # Highlighting wraps every token in a <span>; past ~2.5 MB the DOM is
            # too large for Quarto's HTML post-processing (RangeError: Maximum call
            # stack size exceeded), so large bundles are rendered without it
            if total_out > HTML_SANS_COULEUR:
                o.write("    highlight-style: none\n")
            o.write("  pdf:\n    toc: true\n    toc-depth: 3\n    number-sections: true\n"
                    "    shift-heading-level-by: -1\n"      # ## Synthese = section 1, not 0.1
                    "    papersize: a4\n    geometry: margin=2cm\n    fontsize: 10pt\n"
                    "    monofont: DejaVu Sans Mono\n"      # box drawing, ✓ ✗ ⟨⟩ ⚠ missing from Latin Modern Mono
                    "    include-in-header:\n      text: |\n" + PDF_PREAMBULE)
        o.write("---\n\n")
        o.write("## Synthèse\n\n")
        o.write("| run | élément | valeur |\n|---|---|---|\n")
        lignes_syn = synthese(tous)
        o.write("\n".join(lignes_syn) + "\n\n" if lignes_syn else "| (pas de verdict.txt) | | |\n\n")
        o.write("| dossier de run | contenu | |\n|---|---|---|\n")
        o.write("| Verdict et couverture | échelle des barreaux et tableau de couverture couche 1 | |\n")
        o.write("| Résultats des tests | captures VTY, diagnostics, rapports pytest, grafcets (.txt .md .mmd) | |\n")
        o.write("| Fichiers | sources, scripts, configs et docs du banc (tout fichier texte) | |\n")
        o.write("| Logs | journaux (.log), en dernier | |\n\n")
        o.write("::: {.callout-note collapse=\"true\"}\n## Méthode\n\n" if FORMAT == "qmd" else "### Méthode\n\n")
        o.write("Dossiers : %s. Extensions : %s. Entrée %d Ko, sortie %d Ko.\n\n"
                % (", ".join(dossiers), " ".join(EXT) if EXT else "tous les fichiers texte (hors .git, caches, "
                   "binaires, sauvegardes, LICENSE et sorties précédentes)", total_in // 1024, total_out // 1024))
        o.write("Compactage sans perte : fichiers identiques cités une fois ; lignes consécutives identiques "
                "« ×N » ; lignes ne différant que par des nombres : gabarit avec ⟨k⟩ puis la liste ordonnée "
                "des valeurs (k-uplets), chaque ligne se reconstruit en remettant les valeurs dans l'ordre. "
                "Seuls les codes couleur, la négociation telnet des captures VTY et les caractères de commande "
                "sont retirés ; les lignes de plus de %d caractères sont coupées en morceaux terminés par ↩. Les .md sont inclus tels quels (titres rétrogradés) et les .mmd rendus en "
                "diagrammes Mermaid, sans compactage.\n" % LIGNE_MAX)
        o.write(":::\n\n" if FORMAT == "qmd" else "\n")
        for t in table:                      # only missing directories / unreadable files remain
            o.write(t.strip("| ").split(" |")[0] + "\n")
        o.write("\n")
        o.write("\n".join(sections))
    print("%s : %d fichiers (%d uniques), %d Ko -> %d Ko" % (SORTIE, len(tous), len(vus), total_in // 1024,
                                                            os.path.getsize(SORTIE) // 1024))


if __name__ == "__main__":
    main()
