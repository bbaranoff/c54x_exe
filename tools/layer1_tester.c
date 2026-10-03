/* layer1_tester.c - qui lit, qui ecrit chaque mot de la memoire de donnees du DSP
 * (partie mesure de tools/layer1_tester.py ; « api_carte » en est l'alias).
 *
 * Rejoue un enregistrement de canal dedie (/dev/shm/calypso_rejeu_tch.bin,
 * format de tools/rejeu_banc.c : 'S' 'D' 'T' 'A' 'B') sur le coeur C54x de
 * qosmo et la ROM TI, dans l'ordre de pont.c, et COMPTE pour chacun des 65536
 * mots de donnees :
 *   - les lectures et les ecritures faites par une instruction de la ROM
 *     (premier PC, premier tick, jusqu'a 6 PC distincts, valeurs typiques) ;
 *   - les ecritures de l'ARM (enregistrements 'A' : mots modifies cote QEMU entre
 *     deux trames, donc des MODIFICATIONS, pas toutes les ecritures) et la
 *     valeur de l'etat de depart ('S' pour l'API RAM, 'D' pour le reste) ;
 *   - les ecritures qui ne passent pas par une instruction : DMA, livraison
 *     d'I/Q du BSP, empilement d'interruption, bequilles du coeur qui ecrivent
 *     s->data[] en direct (difference d'instantanes avant/apres chaque etape).
 *
 * LE CROCHET. qosmo n'est pas modifie : data_read()/data_write() de c54x_mem.c
 * appellent calypso_mbx() (moniteur de la boite aux lettres) a chaque acces
 * quand c54x_rapide == 0, avec l'adresse, la valeur et le PC. On lie le coeur
 * SANS calypso_mailbox.c et on fournit ici calypso_mbx_actif/_init/_evt : chaque
 * acces DSP de la memoire de donnees arrive dans compter(). Les acces faits hors
 * d'une instruction (DMA, BSP...) sont separes par g_dans_dsp. Une ecriture
 * directe s->data[] (bequille, DMA) echappe au crochet : la difference
 * d'instantanes la rattrape (colonne hc_*). Une LECTURE directe s->data[] du
 * coeur echappe aux deux : limite connue.
 *
 * PORTS I/O. PORTR/PORTW (c54x_exec.c) appellent d'abord calypso_a5_portr/_portw
 * pour TOUT port : enveloppes (--wrap) pour compter port, sens, PC, valeur
 * ecrite (lignes « io:xxxx » de la sortie). Seules les instructions 0x74xx/0x75xx
 * passent par la ; l'autre encodage PORTW (0x9Fxx) n'est que journalise par le
 * coeur et n'est pas compte.
 *
 * PC ACTIFS (-p). Nombre d'acces memoire par PC (xpc:pc) : dit si une routine
 * connue de la ROM a tourne pendant le rejeu (une routine sans acces memoire
 * n'y apparait pas).
 *
 * SANS EFFET DE BORD. Les sources du coeur ouvrent des fichiers /dev/shm et une
 * socket UDP (calypso_bsp.c) : l'edition de liens enveloppe fopen/open/socket
 * (-Wl,--wrap=...) ; toute ouverture en ecriture part sur /dev/null, socket()
 * echoue. Seul le fichier de sortie (-o) est ecrit. Rien n'est lance, aucun
 * processus du banc n'est touche.
 *
 * BOUCLE DE BUDGET. c54x_run rend la main de lui-meme toutes les
 * CALYPSO_DSP_YIELD instructions (32768) : courir() boucle jusqu'au budget,
 * comme pont.c et rejeu_banc.c (2026-10-03), sinon le DSP ne finit pas sa trame.
 *
 * CONSOMMATION. Pour chaque mot de l'API RAM, une modification ARM ('A') est
 * « en attente » jusqu'a ce qu'une instruction de la ROM lise le mot (consommee,
 * latence en ticks) ou que l'ARM le modifie de nouveau (ecrasee sans lecture).
 *
 *   make layer1_tester          (ou make api_carte, alias)
 *   tools/layer1_tester [-o sortie.tsv] [-j journal.tsv] [-n ticks_max] [fichier]
 *   python3 tools/layer1_tester.py --rejeu fichier   (lance ce binaire, juge, fait la carte)
 *
 * Sortie (-o) : TSV, une ligne par mot touche (voir l'en-tete), lignes '#' = bilan.
 * Journal (-j) : par tick, chaque mot de l'API RAM lu ou ecrit par la ROM :
 *   tick, mot (adresse DSP), lectures, derniere valeur lue, ecritures,
 *   derniere valeur ecrite, PC de la derniere ecriture.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include "qemu/thread.h"
#include "calypso_c54x.h"
#include "calypso_dma.h"
#include "calypso_bsp.h"
#include "calypso_rhea_dma.h"
#include "calypso_mailbox.h"
#include "hw/arm/calypso/calypso_api.h"

/* ce que main.c fournit d'habitude (meme jeu que tools/rejeu_banc.c) */
uint32_t g_c54x_exe_fn;
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }
void calypso_inth_arm_ack(void) { }
QemuMutex calypso_pcb_daram_lock;
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{ (void)addr; if (!wr) memset(buf, 0, len); }

/* ---- garde-fous : aucune ecriture de fichier, aucune socket -------------- */
FILE *__real_fopen(const char *path, const char *mode);
int __real_open(const char *path, int flags, ...);
static int g_fichiers_detournes;
FILE *__wrap_fopen(const char *path, const char *mode)
{
    if (strpbrk(mode, "wa+")) { g_fichiers_detournes++; return __real_fopen("/dev/null", "w"); }
    return __real_fopen(path, mode);
}
int __wrap_open(const char *path, int flags, ...)
{
    if (flags & (O_WRONLY | O_RDWR | O_CREAT | O_TRUNC | O_APPEND)) {
        g_fichiers_detournes++;
        return __real_open("/dev/null", O_WRONLY);
    }
    return __real_open(path, flags);
}
int __wrap_socket(int domain, int type, int protocol)
{ (void)domain; (void)type; (void)protocol; errno = EACCES; return -1; }
/* ports I/O : voir plus bas, io_noter() */
static void io_noter(int ecrit, uint16_t pa, uint16_t val);
bool __real_calypso_a5_portw(C54xState *s, uint16_t pa, uint16_t val);
bool __real_calypso_a5_portr(C54xState *s, uint16_t pa, uint16_t *val);
bool __wrap_calypso_a5_portw(C54xState *s, uint16_t pa, uint16_t val)
{ io_noter(1, pa, val); return __real_calypso_a5_portw(s, pa, val); }
bool __wrap_calypso_a5_portr(C54xState *s, uint16_t pa, uint16_t *val)
{
    bool r = __real_calypso_a5_portr(s, pa, val);
    io_noter(0, pa, r ? *val : 0);
    return r;
}
ssize_t __wrap_sendto(int fd, const void *b, size_t n, int fl, const struct sockaddr *a, socklen_t l)
{ (void)fd; (void)b; (void)fl; (void)a; (void)l; return (ssize_t)n; }

/* ---- etat ------------------------------------------------------------------ */
#define API_WORDS 0x2000u                  /* fenetre API vue du DSP : data 0x0800..0x27ff */
#define ENREG_API_WORDS CALYPSO_API_WORDS  /* taille de l'API RAM enregistree par pont.c ('S') */
#define NDB 0xD4u
#define BL_ADDR_HI_W 0x7FCu
#define BL_SIZE_W 0x7FDu
#define BL_ADDR_LO_W 0x7FEu
#define BL_STATUS_W 0x7FFu
#define NPC 6                              /* PC distincts retenus par mot et par sens */
#define NVAL 4                             /* Misra-Gries : exact si <= 4 valeurs distinctes */

typedef struct {
    int64_t a, b; uint16_t ar[8], t, trn, sp, bk, brc, rsa, rea, st0, st1, pmst, imr, ifr, xpc;
    uint32_t pc; uint8_t idle, running;
} EnregRegs;                               /* = pont.c enreg_base() */

typedef struct { uint16_t v[NVAL]; uint32_t n[NVAL]; uint8_t plus; } Valeurs;
typedef struct {
    uint64_t n;                            /* acces */
    uint32_t pc1, tick1;                   /* premier acces */
    uint32_t pcs[NPC]; uint8_t npc, pcplus;
    Valeurs val;
    uint16_t dernier;
} Sens;

enum { SEG_BOOT = 0, SEG_DSP, SEG_IRQ, SEG_DMA, SEG_BSP, SEG_POMPE, SEG_N };
static const char *SEG_NOM[SEG_N] = { "boot", "dsp", "irq", "dma", "bsp", "pompe" };

static C54xState *dsp;
static uint16_t *api;
static uint8_t *fichier;
static size_t taille;
static int g_dans_dsp;                     /* 1 pendant c54x_run */
static int g_seg = SEG_BOOT;
static int g_phase_rejeu;                  /* 0 = boot local, 1 = rejeu */

static Sens *s_rd, *s_wr, *s_arm;          /* [65536] */
static uint64_t *boot_rd, *boot_wr;        /* acces pendant le boot local */
static uint64_t *hors_wr;                  /* ecritures par le crochet hors instruction */
static uint8_t *hors_src;
static uint64_t *hc_n;                     /* ecritures hors crochet (instantanes) */
static uint32_t *hc_tick1; static uint8_t *hc_src; static uint16_t *hc_der;
static uint16_t *val_s, *val_d; static uint8_t *a_s, *a_d;
static uint16_t *snap; static uint32_t *gen; static uint32_t g_gen = 1;
static uint64_t g_n_evt, g_n_rd, g_n_wr;
static Sens *io_rd, *io_wr;                /* [65536] ports */
static uint32_t *pc_acc;                   /* [0x40000] acces memoire par xpc:pc */
/* consommation des modifications ARM (mots de l'API RAM) */
static uint8_t *arm_pend; static uint32_t *arm_pend_tick;
static uint64_t *conso_n, *conso_lat, *ecrase_n;
/* journal par tick (mots de l'API RAM) */
static FILE *g_journal;
static uint32_t *j_r, *j_w; static uint16_t *j_vr, *j_vw; static uint32_t *j_pcw;
static uint16_t j_liste[0x2000]; static unsigned j_n;
static inline void j_toucher(uint16_t mot)
{
    unsigned i = mot - 0x800u;
    if (!j_r[i] && !j_w[i]) j_liste[j_n++] = (uint16_t)i;
}

static inline uint32_t pc_complet(void)
{
    uint32_t pc = dsp->pc & 0xffff;
    if (pc >= 0x8000 && dsp->xpc) pc |= (uint32_t)dsp->xpc << 16;
    return pc;
}

static inline void valeur(Valeurs *v, uint16_t x)
{
    for (int i = 0; i < NVAL; i++) if (v->n[i] && v->v[i] == x) { v->n[i]++; return; }
    for (int i = 0; i < NVAL; i++) if (!v->n[i]) { v->v[i] = x; v->n[i] = 1; return; }
    v->plus = 1;
    for (int i = 0; i < NVAL; i++) v->n[i]--;
}

static inline void noter(Sens *s, uint32_t pc, uint16_t v)
{
    if (!s->n) { s->pc1 = pc; s->tick1 = g_c54x_exe_fn; }
    s->n++;
    s->dernier = v;
    valeur(&s->val, v);
    if (pc != 0xffffffffu) {
        for (int i = 0; i < s->npc; i++) if (s->pcs[i] == pc) goto vu;
        if (s->npc < NPC) s->pcs[s->npc++] = pc; else s->pcplus = 1;
    vu:;
    }
}

static void io_noter(int ecrit, uint16_t pa, uint16_t val)
{
    if (!g_phase_rejeu || !io_rd) return;
    noter(ecrit ? &io_wr[pa] : &io_rd[pa], pc_complet(), val);
}

/* Le crochet : remplace calypso_mailbox.c (seuls symboles exportes). */
int calypso_mbx_actif = 1;
void calypso_mbx_init(void) { }
void calypso_mbx_evt(CalypsoMbxSens sens, uint16_t mot, uint16_t val, uint16_t avant,
                     uint32_t ctx, uint32_t fn, uint32_t insn)
{
    (void)avant; (void)ctx; (void)fn; (void)insn;
    g_n_evt++;
    if (g_phase_rejeu && g_dans_dsp) pc_acc[pc_complet() & 0x3ffff]++;
    if (sens == MBX_DSP_RD) {
        g_n_rd++;
        if (!g_phase_rejeu) { boot_rd[mot]++; return; }
        if (g_dans_dsp) {
            noter(&s_rd[mot], pc_complet(), val);
            if (mot >= 0x800 && mot < 0x2800) {
                if (arm_pend[mot]) { arm_pend[mot] = 0; conso_n[mot]++; conso_lat[mot] += g_c54x_exe_fn - arm_pend_tick[mot]; }
                if (g_journal) { j_toucher(mot); j_r[mot - 0x800]++; j_vr[mot - 0x800] = val; }
            }
        }
    } else if (sens == MBX_DSP_WR) {
        g_n_wr++;
        gen[mot] = g_gen;
        if (!g_phase_rejeu) { boot_wr[mot]++; return; }
        if (g_dans_dsp) {
            noter(&s_wr[mot], pc_complet(), val);
            if (g_journal && mot >= 0x800 && mot < 0x2800) {
                j_toucher(mot); j_w[mot - 0x800]++; j_vw[mot - 0x800] = val; j_pcw[mot - 0x800] = pc_complet(); }
        }
        else { hors_wr[mot]++; hors_src[mot] |= (uint8_t)(1u << g_seg); }
    }
}

/* Ecritures qui n'ont pas pris le crochet depuis le dernier instantane. */
static void instantane(int seg)
{
    const uint64_t *a = (const uint64_t *)dsp->data, *b = (const uint64_t *)snap;
    for (unsigned q = 0; q < C54X_DATA_SIZE / 4; q++) {
        if (a[q] == b[q]) continue;
        for (unsigned k = 0; k < 4; k++) {
            unsigned m = q * 4 + k;
            if (dsp->data[m] == snap[m]) continue;
            if (m >= 0x60 && gen[m] != g_gen && g_phase_rejeu) {   /* MMR/peripheriques exclus */
                if (!hc_n[m]) hc_tick1[m] = g_c54x_exe_fn;
                hc_n[m]++; hc_src[m] |= (uint8_t)(1u << seg); hc_der[m] = dsp->data[m];
            }
            snap[m] = dsp->data[m];
        }
    }
    g_gen++;
}

static void synchro_instantane(void) { memcpy(snap, dsp->data, C54X_DATA_SIZE * 2); g_gen++; }

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

static long courir(long n)
{
    long k = 0;
    g_dans_dsp = 1;
    while (k < n && dsp->running && !dsp->idle) {
        int r = c54x_run(dsp, (int)(n - k));
        if (r <= 0) break;
        k += r;
    }
    g_dans_dsp = 0;
    instantane(SEG_DSP);
    return k;
}

static void reveil(void)
{
    g_seg = SEG_IRQ;
    if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) && !(dsp->ifr & (1u << 14)))
        c54x_interrupt_ex(dsp, 30, 14);
    if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
    instantane(SEG_IRQ);
}

static void boot(void)
{
    memset(api, 0, API_WORDS * 2);
    long b = 0;
    g_dans_dsp = 1;
    while (api[BL_STATUS_W] != 1 && b < 8000000) { int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex; }
    api[BL_ADDR_HI_W] = 0; api[BL_ADDR_LO_W] = 0x7000; api[BL_SIZE_W] = 0; api[BL_STATUS_W] = 2;
    while (b < 12000000 && dsp->running) { int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex; if (dsp->idle) break; }
    g_dans_dsp = 0;
    printf("# boot local : %ld insn, idle=%d\n", b, dsp->idle);
}

static void ecrire_arm(unsigned a, uint16_t v)
{
    if (a >= API_WORDS) return;
    api[a] = v;
    snap[0x800 + a] = v;
    noter(&s_arm[0x800 + a], 0xffffffffu, v);
    if (arm_pend[0x800 + a]) ecrase_n[0x800 + a]++;
    arm_pend[0x800 + a] = 1; arm_pend_tick[0x800 + a] = g_c54x_exe_fn;
}

static void pc_txt(char *o, uint32_t pc)
{
    if (pc > 0xffff) sprintf(o, "%x:%04x", pc >> 16, pc & 0xffff); else sprintf(o, "%04x", pc);
}

static void sens_txt(FILE *f, const Sens *s)
{
    char b[16];
    fprintf(f, "\t%llu", (unsigned long long)s->n);
    if (!s->n) { fprintf(f, "\t\t\t\t\t"); return; }
    if (s->pc1 != 0xffffffffu) { pc_txt(b, s->pc1); fprintf(f, "\t%s", b); } else fprintf(f, "\t");
    fprintf(f, "\t%u\t", s->tick1);
    for (int i = 0; i < s->npc; i++) { pc_txt(b, s->pcs[i]); fprintf(f, "%s%s", i ? "," : "", b); }
    if (s->pcplus) fprintf(f, ",+");
    fprintf(f, "\t");
    /* valeurs, de la plus frequente a la moins frequente */
    int ord[NVAL]; for (int i = 0; i < NVAL; i++) ord[i] = i;
    for (int i = 0; i < NVAL; i++) for (int j = i + 1; j < NVAL; j++)
        if (s->val.n[ord[j]] > s->val.n[ord[i]]) { int t = ord[i]; ord[i] = ord[j]; ord[j] = t; }
    int prem = 1;
    for (int i = 0; i < NVAL; i++) if (s->val.n[ord[i]]) {
        fprintf(f, "%s%04x:%u", prem ? "" : ",", s->val.v[ord[i]], s->val.n[ord[i]]); prem = 0; }
    if (s->val.plus) fprintf(f, ",+");
    fprintf(f, "\t%04x", s->dernier);
}

static void src_txt(FILE *f, uint8_t m)
{
    int prem = 1;
    for (int i = 0; i < SEG_N; i++) if (m & (1u << i)) { fprintf(f, "%s%s", prem ? "" : ",", SEG_NOM[i]); prem = 0; }
}

typedef struct { size_t debut, fin; uint32_t tick; uint8_t drap; long budget; } Tick;

static void *alloue(size_t n) { void *p = calloc(1, n); if (!p) { perror("calloc"); exit(1); } return p; }

int main(int argc, char **argv)
{
    const char *nom = "/dev/shm/calypso_rejeu_tch.bin", *sortie = NULL, *f_pcs = NULL;
    long max_ticks = 1000000;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) sortie = argv[++i];
        else if (!strcmp(argv[i], "-p") && i + 1 < argc) f_pcs = argv[++i];
        else if (!strcmp(argv[i], "-j") && i + 1 < argc) {
            g_journal = __real_fopen(argv[++i], "w");
            if (!g_journal) { perror(argv[i]); return 1; }
        }
        else if (!strcmp(argv[i], "-n") && i + 1 < argc) max_ticks = atol(argv[++i]);
        else if (argv[i][0] == '-') { fprintf(stderr, "usage : %s [-o sortie.tsv] [-j journal.tsv] [-p pcs.tsv] [-n ticks] [fichier]\n", argv[0]); return 2; }
        else nom = argv[i];
    }
    FILE *f = __real_fopen(nom, "rb");
    if (!f) { perror(nom); return 1; }
    fseek(f, 0, SEEK_END); taille = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    fichier = alloue(taille);
    if (fread(fichier, 1, taille, f) != taille) { fprintf(stderr, "lecture courte\n"); return 1; }
    fclose(f);
    FILE *o = sortie ? __real_fopen(sortie, "w") : stdout;
    if (!o) { perror(sortie); return 1; }

    s_rd = alloue(65536 * sizeof(Sens)); s_wr = alloue(65536 * sizeof(Sens)); s_arm = alloue(65536 * sizeof(Sens));
    boot_rd = alloue(65536 * 8); boot_wr = alloue(65536 * 8); hors_wr = alloue(65536 * 8); hors_src = alloue(65536);
    hc_n = alloue(65536 * 8); hc_tick1 = alloue(65536 * 4); hc_src = alloue(65536); hc_der = alloue(65536 * 2);
    val_s = alloue(65536 * 2); val_d = alloue(65536 * 2); a_s = alloue(65536); a_d = alloue(65536);
    snap = alloue(65536 * 2); gen = alloue(65536 * 4);
    io_rd = alloue(65536 * sizeof(Sens)); io_wr = alloue(65536 * sizeof(Sens)); pc_acc = alloue(0x40000 * 4);
    arm_pend = alloue(65536); arm_pend_tick = alloue(65536 * 4);
    conso_n = alloue(65536 * 8); conso_lat = alloue(65536 * 8); ecrase_n = alloue(65536 * 8);
    j_r = alloue(0x2000 * 4); j_w = alloue(0x2000 * 4); j_vr = alloue(0x2000 * 2); j_vw = alloue(0x2000 * 2);
    j_pcw = alloue(0x2000 * 4);
    if (g_journal) fprintf(g_journal, "tick\tmot\tr\tval_r\tw\tval_w\tpc_w\n");

    /* meme environnement que tools/rejeu_banc.c ; les ecritures /dev/shm et la
     * socket sont de toute facon neutralisees par les enveloppes ci-dessus */
    setenv("CALYPSO_BSP_PORT", "16703", 1);
    setenv("CALYPSO_BSP_BIND_ADDR", "127.0.0.1", 1);
    setenv("CALYPSO_BSP_HORLOGE", "0", 1);
    setenv("CALYPSO_C54X_IRQ_LEVEL", "1", 0);
    setenv("CALYPSO_BSP_STREAM", "1", 0);
    setenv("CALYPSO_RHEA_DMA_XFER", "1", 0);
    setenv("CALYPSO_REJEU_ENREG", "0", 1);
    qemu_mutex_init(&calypso_pcb_daram_lock);
    dsp = c54x_init();
    api = &dsp->data[0x800];
    c54x_set_api_ram(dsp, api);
    static const struct { const char *s; uint32_t a; bool p; } R[] = {
        { "PROM0", 0x07000, true }, { "PROM1", 0x18000, true }, { "PROM2", 0x28000, true },
        { "PROM3", 0x38000, true }, { "DROM", 0x09000, false }, { "PDROM", 0x0E000, false }, { "PDROM", 0x0E000, true } };
    const char *rom = getenv("API_CARTE_ROM") ? getenv("API_CARTE_ROM") : "/opt/GSM";
    for (unsigned i = 0; i < 7; i++) {
        char c[512]; snprintf(c, sizeof c, "%s/calypso_dsp.%s.bin", rom, R[i].s);
        if (c54x_load_section(dsp, c, R[i].a, R[i].p) < 0) { fprintf(stderr, "ROM %s\n", c); return 1; }
    }
    { char c[512]; snprintf(c, sizeof c, "%s/calypso_dsp.Registers.bin", rom); c54x_load_registers(dsp, c); }
    c54x_reset(dsp);
    calypso_dma_init();
    calypso_bsp_init(dsp);
    struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0);
    boot();
    g_phase_rejeu = 1;

    /* Premiere passe : etat de depart et decoupage en ticks. */
    Tick *ticks = alloue(sizeof(Tick) * 200000); int nt = 0;
    size_t p = 0;
    while (p < taille) {
        uint8_t k = fichier[p];
        size_t l;
        switch (k) {
        case 'S': l = 5 + (size_t)ENREG_API_WORDS * 2; break;
        case 'D': l = 5 + sizeof(EnregRegs) + C54X_DATA_SIZE * 2; break;
        case 'T': l = 10; break;
        case 'A': l = 8 + 4 * (size_t)be16(fichier + p + 6); break;
        case 'B': l = 15 + 2 * (size_t)be16(fichier + p + 13); break;
        default: fprintf(stderr, "enregistrement inconnu 0x%02x a %zu\n", k, p); goto fini;
        }
        if (p + l > taille) break;
        if (k == 'S') {
            memcpy(api, fichier + p + 5, API_WORDS * 2);
            for (unsigned i = 0; i < API_WORDS; i++) { val_s[0x800 + i] = api[i]; a_s[0x800 + i] = 1; }
            printf("# etat S pose (tick %u)\n", be32(fichier + p + 1));
        } else if (k == 'D') {
            EnregRegs r; memcpy(&r, fichier + p + 5, sizeof r);
            uint16_t *sauve = alloue(API_WORDS * 2); memcpy(sauve, api, API_WORDS * 2);
            memcpy(dsp->data, fichier + p + 5 + sizeof r, C54X_DATA_SIZE * 2);
            for (unsigned i = 0; i < C54X_DATA_SIZE; i++) { val_d[i] = dsp->data[i]; a_d[i] = 1; }
            memcpy(api, sauve, API_WORDS * 2); free(sauve);   /* 'S' fait foi pour l'API RAM */
            dsp->a = r.a; dsp->b = r.b; memcpy(dsp->ar, r.ar, sizeof r.ar); dsp->t = r.t; dsp->trn = r.trn;
            dsp->sp = r.sp; dsp->bk = r.bk; dsp->brc = r.brc; dsp->rsa = r.rsa; dsp->rea = r.rea;
            dsp->st0 = r.st0; dsp->st1 = r.st1; dsp->pmst = r.pmst; dsp->imr = r.imr; dsp->ifr = r.ifr;
            dsp->xpc = r.xpc; dsp->pc = r.pc; dsp->idle = r.idle; dsp->running = r.running;
            printf("# etat D pose : pc=%04x sp=%04x idle=%d imr=%04x\n", r.pc & 0xffff, r.sp, r.idle, r.imr);
        } else if (k == 'T') {
            if (nt > 0) ticks[nt - 1].fin = p;
            if (nt < 200000) {
                ticks[nt].debut = p; ticks[nt].tick = be32(fichier + p + 1);
                ticks[nt].drap = fichier[p + 5]; ticks[nt].budget = (long)be32(fichier + p + 6);
                nt++;
            }
        }
        p += l;
    }
fini:
    if (nt > 0) ticks[nt - 1].fin = p;
    printf("# %d ticks enregistres (%u..%u)\n", nt, nt ? ticks[0].tick : 0, nt ? ticks[nt - 1].tick : 0);
    synchro_instantane();

    uint16_t prec_cd = 0xffff, prec_fd = 0xffff, prec_dd = 0;
    int sacch_ok = 0, sacch_ko = 0, facch_ok = 0, facch_ko = 0;
    unsigned long parole_n = 0, parole_bfi = 0, arm_hors_fenetre = 0;
    uint64_t insns = 0;
    int joues = 0;
    for (int it = 0; it < nt && it < max_ticks; it++, joues++) {
        Tick *t = &ticks[it];
        g_c54x_exe_fn = t->tick;
        long budget = t->budget;
        /* ecritures ARM, fenetre 0 (avant le TICK) */
        for (size_t q = t->debut; q < t->fin;) {
            uint8_t k = fichier[q];
            size_t l = k == 'T' ? 10 : k == 'A' ? 8 + 4 * (size_t)be16(fichier + q + 6)
                     : k == 'B' ? 15 + 2 * (size_t)be16(fichier + q + 13) : 0;
            if (!l) break;
            if (k == 'A' && fichier[q + 5] == 0)
                for (unsigned i = 0, n = be16(fichier + q + 6); i < n; i++) {
                    unsigned a = be16(fichier + q + 8 + 4 * i);
                    if (a >= API_WORDS) arm_hors_fenetre++;
                    ecrire_arm(a, be16(fichier + q + 10 + 4 * i));
                }
            q += l;
        }
        /* phase A */
        g_seg = SEG_DMA; calypso_dma_tick(dsp); instantane(SEG_DMA);
        reveil();
        g_seg = SEG_IRQ;
        if ((dsp->imr & (1u << C54X_IT_TPU_FRAME_BIT)) && (t->drap & 1))
            c54x_interrupt_ex(dsp, C54X_IT_TPU_FRAME_VEC, C54X_IT_TPU_FRAME_BIT);
        instantane(SEG_IRQ);
        g_seg = SEG_DSP;
        long fait = 0;
        if (!dsp->idle) fait = courir(budget / 8);
        while (!dsp->idle && fait < budget / 2) { long r = courir(256); if (r <= 0) break; fait += r; }
        /* ecritures ARM, fenetre 1, puis livraisons I/Q dans l'ordre */
        for (size_t q = t->debut; q < t->fin;) {
            uint8_t k = fichier[q];
            size_t l = k == 'T' ? 10 : k == 'A' ? 8 + 4 * (size_t)be16(fichier + q + 6)
                     : k == 'B' ? 15 + 2 * (size_t)be16(fichier + q + 13) : 0;
            if (!l) break;
            if (k == 'A' && fichier[q + 5] == 1)
                for (unsigned i = 0, n = be16(fichier + q + 6); i < n; i++) {
                    unsigned a = be16(fichier + q + 8 + 4 * i);
                    if (a >= API_WORDS) arm_hors_fenetre++;
                    ecrire_arm(a, be16(fichier + q + 10 + 4 * i));
                }
            if (k == 'B') {
                int n = be16(fichier + q + 13);
                static int16_t iq[1024];
                memcpy(iq, fichier + q + 15, 2 * (size_t)(n > 1024 ? 1024 : n));
                g_seg = SEG_BSP;
                calypso_bsp_rx_burst(fichier[q + 5], be32(fichier + q + 6), iq, n);
                instantane(SEG_BSP);
            }
            q += l;
        }
        /* phase B : reste du budget, puis la pompe DMA de pont.c */
        reveil();
        g_seg = SEG_DSP;
        if (!dsp->idle && budget - fait > 0) fait += courir(budget - fait);
        for (int k = 0; k < 40 && dsp->running; k++) {
            g_seg = SEG_POMPE;
            int pompe = calypso_rhea_dma_pump(dsp);
            instantane(SEG_POMPE);
            if (!pompe) break;
            if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
            g_seg = SEG_DSP;
            if (!dsp->idle) fait += courir(budget / 4);
        }
        insns += (uint64_t)fait;
        /* controle de fidelite : memes bilans que tools/rejeu_banc.c */
        uint16_t *cd = &api[NDB + 0x1FC / 2], *fd = &api[NDB + 0x21A / 2], *dd = &api[NDB + 0x238 / 2];
        if (cd[0] != prec_cd && (cd[0] & 0x8000)) { if (cd[0] & (1u << 6)) sacch_ko++; else sacch_ok++; }
        if (fd[0] != prec_fd && (fd[0] & 0x8000)) { if (fd[0] & (1u << 6)) facch_ko++; else facch_ok++; }
        if (dd[0] != prec_dd && (dd[0] & 0x8000)) { parole_n++; if (dd[0] & 0x4) parole_bfi++; }
        prec_cd = cd[0]; prec_fd = fd[0]; prec_dd = dd[0];
        if (g_journal) {
            for (unsigned q = 0; q < j_n; q++) {
                unsigned i = j_liste[q]; char b[16];
                pc_txt(b, j_pcw[i]);
                fprintf(g_journal, "%u\t%04x\t%u\t%04x\t%u\t%04x\t%s\n", t->tick, 0x800 + i, j_r[i], j_vr[i], j_w[i], j_vw[i],
                        j_w[i] ? b : "");
                j_r[i] = j_w[i] = 0;
            }
            j_n = 0;
        }
    }
    if (g_journal) fclose(g_journal);
    struct timespec t1; clock_gettime(CLOCK_MONOTONIC, &t1);
    double sec = (double)(t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;

    fprintf(o, "# layer1_tester : fichier=%s ticks_joues=%d/%d tick=%u..%u insn=%llu duree=%.1fs\n", nom, joues, nt,
            nt ? ticks[0].tick : 0, joues ? ticks[joues - 1].tick : 0, (unsigned long long)insns, sec);
    fprintf(o, "# controle : SACCH %d bonnes / %d Fire KO ; FACCH %d / %d KO ; PAROLE %lu trames, %lu BFI\n",
            sacch_ok, sacch_ko, facch_ok, facch_ko, parole_n, parole_bfi);
    fprintf(o, "# crochet : %llu evenements (%llu lectures, %llu ecritures DSP) ; ouvertures detournees vers /dev/null : %d ;"
            " mots ARM hors fenetre DSP : %lu\n", (unsigned long long)g_n_evt, (unsigned long long)g_n_rd,
            (unsigned long long)g_n_wr, g_fichiers_detournes, arm_hors_fenetre);
    fprintf(o, "adr\tdsp_r\tr_pc1\tr_tick1\tr_pcs\tr_vals\tr_der"
               "\tdsp_w\tw_pc1\tw_tick1\tw_pcs\tw_vals\tw_der"
               "\tarm_w\tarm_pc1\tarm_tick1\tarm_pcs\tarm_vals\tarm_der"
               "\thors_w\thors_src\thc_w\thc_src\thc_tick1\thc_der\tval_s\tval_d\tval_fin\tboot_r\tboot_w"
               "\tarm_conso\tarm_lat_moy\tarm_ecrase\n");
    for (unsigned a = 0; a < 65536; a++) {
        bool api_mot = a >= 0x800 && a < 0x800 + API_WORDS;
        if (!api_mot && !s_rd[a].n && !s_wr[a].n && !hors_wr[a] && !hc_n[a] && !boot_rd[a] && !boot_wr[a]) continue;
        fprintf(o, "%04x", a);
        sens_txt(o, &s_rd[a]); sens_txt(o, &s_wr[a]); sens_txt(o, &s_arm[a]);
        fprintf(o, "\t%llu\t", (unsigned long long)hors_wr[a]); src_txt(o, hors_src[a]);
        fprintf(o, "\t%llu\t", (unsigned long long)hc_n[a]); src_txt(o, hc_src[a]);
        if (hc_n[a]) fprintf(o, "\t%u\t%04x", hc_tick1[a], hc_der[a]); else fprintf(o, "\t\t");
        if (a_s[a]) fprintf(o, "\t%04x", val_s[a]); else fprintf(o, "\t");
        if (a_d[a]) fprintf(o, "\t%04x", val_d[a]); else fprintf(o, "\t");
        fprintf(o, "\t%04x\t%llu\t%llu", dsp->data[a], (unsigned long long)boot_rd[a], (unsigned long long)boot_wr[a]);
        if (conso_n[a]) fprintf(o, "\t%llu\t%.2f", (unsigned long long)conso_n[a], (double)conso_lat[a] / conso_n[a]);
        else fprintf(o, "\t0\t");
        fprintf(o, "\t%llu\n", (unsigned long long)ecrase_n[a]);
    }
    for (unsigned a = 0; a < 65536; a++) {
        if (!io_rd[a].n && !io_wr[a].n) continue;
        fprintf(o, "io:%04x", a);
        sens_txt(o, &io_rd[a]); sens_txt(o, &io_wr[a]);
        static const Sens zero;
        sens_txt(o, &zero);                  /* pas de colonnes ARM pour un port */
        for (int k = 0; k < 15; k++) fputc('\t', o);
        fputc('\n', o);
    }
    if (o != stdout) fclose(o);
    if (f_pcs) {
        FILE *fp = __real_fopen(f_pcs, "w");
        if (!fp) { perror(f_pcs); return 1; }
        fprintf(fp, "pc\tacces\n");
        for (unsigned a = 0; a < 0x40000; a++) if (pc_acc[a]) {
            char b[16]; pc_txt(b, a); fprintf(fp, "%s\t%u\n", b, pc_acc[a]); }
        fclose(fp);
    }
    printf("# fini : %d ticks, %llu insn, %.1f s ; SACCH %d/%d KO ; PAROLE %lu (%lu BFI)\n", joues,
           (unsigned long long)insns, sec, sacch_ok, sacch_ko, parole_n, parole_bfi);
    return 0;
}
