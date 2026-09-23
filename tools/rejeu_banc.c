/* rejeu_banc.c - rejoue hors banc un TCH enregistre par c54x_exe.
 *
 * Entree : /dev/shm/calypso_rejeu_tch.bin (calypso_bsp.c + pont.c,
 * CALYPSO_REJEU_ENREG). Enregistrements :
 *   'S' tick, API RAM complete        'D' tick, registres, memoire de donnees
 *   'T' tick, drapeaux, budget        'A' tick, fenetre, n x (mot, valeur)
 *   'B' tick, tn, fn, one_shot, nwin, n, I/Q
 * On boote le DSP comme tools/tch_rejeu, on pose l'etat 'S'/'D', puis chaque
 * tick dans l'ordre de pont.c : ecritures ARM d'avant le TICK, interruption
 * trame, phase A jusqu'a l'IDLE (TCH), ecritures ARM d'entre A et GO,
 * livraisons d'I/Q, reste du budget, pompe DMA. On imprime chaque resultat
 * SACCH (a_cd) et FACCH (a_fd).
 *
 * Si le rejeu reproduit le Fire KO du banc, on peut iterer ici sans relancer
 * le banc ; REJEU_SANS_D=1 garde l'etat du boot local au lieu de 'D'.
 *
 *   ./rejeu_banc [fichier] [ticks max]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "qemu/thread.h"
#include "calypso_c54x.h"
#include "calypso_dma.h"
#include "calypso_bsp.h"
#include "calypso_rhea_dma.h"
#include "hw/arm/calypso/calypso_api.h"
#include <math.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/a5.h>

uint32_t g_c54x_exe_fn;
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }
void calypso_inth_arm_ack(void) { }
QemuMutex calypso_pcb_daram_lock;
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{ (void)addr; if (!wr) memset(buf, 0, len); }

#define API_WORDS 0x2000u          /* fenetre API vue du DSP : data 0x0800..0x27ff */
#define ENREG_API_WORDS CALYPSO_API_WORDS   /* taille de l'API RAM enregistree par pont.c */
#define NDB 0xD4u
#define PARAM 0x431u
#define BL_ADDR_HI_W 0x7FCu
#define BL_SIZE_W 0x7FDu
#define BL_ADDR_LO_W 0x7FEu
#define BL_STATUS_W 0x7FFu

typedef struct {
    int64_t a, b; uint16_t ar[8], t, trn, sp, bk, brc, rsa, rea, st0, st1, pmst, imr, ifr, xpc;
    uint32_t pc; uint8_t idle, running;
} EnregRegs;

static C54xState *dsp;
static uint16_t *api;
static uint8_t *fichier;
static size_t taille;

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

static long pump(long max, int stop_idle)
{
    long b = 0;
    while (b < max && dsp->running) {
        int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex;
        if (stop_idle && dsp->idle) break;
    }
    return b;
}

static void reveil(void)
{
    if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) && !(dsp->ifr & (1u << 14)))
        c54x_interrupt_ex(dsp, 30, 14);
    if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
}

static void boot(void)
{
    memset(api, 0, API_WORDS * 2);
    long b = 0;
    while (api[BL_STATUS_W] != 1 && b < 8000000) { int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex; }
    api[BL_ADDR_HI_W] = 0; api[BL_ADDR_LO_W] = 0x7000; api[BL_SIZE_W] = 0; api[BL_STATUS_W] = 2;
    b = pump(4000000, 1);
    printf("boot local : %ld insn, idle=%d\n", b, dsp->idle);
}

/* SONDE (REJEU_SONDE=1) : ou la ROM range-t-elle les bursts SACCH ?
 * Pour chaque burst du canal (slot 0 livre par le BSP), on retrouve les bits
 * par demodulation GMSK differentielle (polarite calee sur la TSC7), on les
 * dechiffre (Kc et d_a5mode pris dans l'API RAM), puis au tick ou a_cd change
 * on cherche dans la memoire du DSP ou sont les 116 bits de chacun des quatre
 * bursts du bloc (bits souples : signe negatif ou positif = 1). */
static const uint8_t TSC7[26] = {1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0};
static struct { uint32_t fn; uint8_t b[148]; } memo[4096];
static int nmemo;
static void memo_burst(uint32_t fn, const int16_t *iq, int n, int marge)
{
    if (n / 2 < marge + 148) return;
    uint8_t d[148], b[2][148];
    for (int k = 0; k < 148; k++) {
        int i0 = 2 * (marge + k), i1 = i0 - 2;
        double ph1 = atan2(iq[i0 + 1], iq[i0]);
        double ph0 = k ? atan2(iq[i1 + 1], iq[i1]) : 0;
        double dp = ph1 - ph0;
        while (dp > M_PI) dp -= 2 * M_PI;
        while (dp < -M_PI) dp += 2 * M_PI;
        d[k] = dp > 0;
    }
    int best = -1, bs = -1;
    for (int pol = 0; pol < 2; pol++) {
        uint8_t prev = 0;
        for (int k = 0; k < 148; k++) { uint8_t dd = d[k] ^ pol; b[pol][k] = dd ^ prev; prev = b[pol][k]; }
        int sc = 0; for (int k = 0; k < 26; k++) sc += b[pol][61 + k] == TSC7[k];
        if (sc > bs) { bs = sc; best = pol; }
    }
    int slot = nmemo % 4096; nmemo++;
    memo[slot].fn = fn; memcpy(memo[slot].b, b[best], 148);
    static int nlog; if (nlog++ < 3) printf("  [sonde] burst fn=%u : TSC %d/26 (differentielle, pol=%d)\n", fn, bs, best);
}
static const uint8_t *memo_trouver(uint32_t fn)
{
    /* Bits exacts de la sonde du BSP (calypso_sacch_tf.bin), s'il y en a. */
    static uint8_t (*vrai)[156]; static int nvrai = -1;
    if (nvrai < 0) {
        nvrai = 0;
        const char *nom = getenv("REJEU_BITS");
        FILE *f = nom ? fopen(nom, "rb") : NULL;
        if (f) { vrai = malloc(4096 * 156); while (nvrai < 4096 && fread(vrai[nvrai], 1, 156, f) == 156) nvrai++; fclose(f); }
        if (nom) printf("  [sonde] %d bursts exacts lus dans %s\n", nvrai, nom);
    }
    for (int i = 0; i < nvrai; i++) if (be32(vrai[i] + 4) == fn) return vrai[i] + 8;
    if (getenv("REJEU_BITS")) return NULL;
    for (int i = 0; i < 4096 && i < nmemo; i++) if (memo[i].fn == fn) return memo[i].b;
    return NULL;
}
static void sonde_bloc(uint32_t fn_dernier)
{
    uint8_t kc[8]; uint16_t *akc = &api[NDB + 0x2CE / 2]; int algo = api[NDB + 0x1CE / 2];
    for (int i = 0; i < 4; i++) { kc[7 - 2 * i] = akc[i] & 0xff; kc[6 - 2 * i] = akc[i] >> 8; }
    for (int k = 0; k < 4; k++) {
        uint32_t fn = fn_dernier - 26 * (3 - k);
        const uint8_t *b0 = memo_trouver(fn);
        if (!b0) { printf("    burst %d fn=%u : non enregistre\n", k, fn); continue; }
        uint8_t b[148]; memcpy(b, b0, 148);
        if (algo) { ubit_t dl[114], ul[114]; osmo_a5(algo, kc, fn, dl, ul);
            for (int j = 0; j < 57; j++) { b[3 + j] ^= dl[j]; b[88 + j] ^= dl[57 + j]; } }
        uint8_t t[116]; memcpy(t, b + 3, 58); memcpy(t + 58, b + 87, 58);
        int best[2] = {0, 0}; unsigned ad[2] = {0, 0};
        for (unsigned a = 0x60; a + 116 < C54X_DATA_SIZE; a++) {
            int m0 = 0, m1 = 0;
            for (int i = 0; i < 116; i++) { int16_t v = (int16_t)dsp->data[a + i];
                if (v != 0) { if ((v < 0) == (t[i] != 0)) m0++; if ((v > 0) == (t[i] != 0)) m1++; } }
            if (m0 > best[0]) { best[0] = m0; ad[0] = a; }
            if (m1 > best[1]) { best[1] = m1; ad[1] = a; }
        }
        /* aussi en 57+57 sans les bits de vol (e[] seuls) */
        uint8_t e[114]; memcpy(e, b + 3, 57); memcpy(e + 57, b + 88, 57);
        int be = 0; unsigned ae = 0;
        for (unsigned a = 0x60; a + 114 < C54X_DATA_SIZE; a++) {
            int m = 0; for (int i = 0; i < 114; i++) { int16_t v = (int16_t)dsp->data[a + i]; if (v != 0 && (v < 0) == (e[i] != 0)) m++; }
            if (m > be) { be = m; ae = a; }
        }
        printf("    burst %d fn=%u (fn%%104=%u) : 116b neg %d@%04x pos %d@%04x | 114b neg %d@%04x\n",
               k, fn, fn % 104, best[0], ad[0], best[1], ad[1], be, ae);
    }
}

/* REJEU_TRACE=1 : pas a pas, anneau des dernieres instructions, arret des que
 * SP sort de la pile ou que PC tombe en DARAM basse (< 0x0800, hors OVLY). */
uint16_t prog_fetch(C54xState *s, uint16_t pc);
#define ANNEAU 4096
static struct { uint16_t pc, op, op2, sp, st0, st1, ar[8]; uint32_t tick; int64_t a, b; } anneau[ANNEAU];
static unsigned apos;
static int trace_on = -1, plante;
static long courir(long n)
{
    if (trace_on < 0) trace_on = getenv("REJEU_TRACE") != NULL;
    if (!trace_on) return c54x_run(dsp, (int)n);
    long k = 0;
    for (; k < n && dsp->running && !dsp->idle && !plante; k++) {
        unsigned i = apos++ & (ANNEAU - 1);
        uint16_t pc = dsp->pc & 0xffff;
        anneau[i].pc = pc; anneau[i].op = prog_fetch(dsp, pc); anneau[i].op2 = prog_fetch(dsp, pc + 1);
        anneau[i].sp = dsp->sp; anneau[i].st0 = dsp->st0; anneau[i].st1 = dsp->st1; anneau[i].tick = g_c54x_exe_fn;
        memcpy(anneau[i].ar, dsp->ar, sizeof dsp->ar); anneau[i].a = dsp->a; anneau[i].b = dsp->b;
        static int wa = -2; static uint16_t wv;
        if (wa == -2) { const char *e = getenv("REJEU_STOP_W"); wa = e ? (int)strtoul(e, NULL, 16) : -1; }
        if (wa >= 0) wv = dsp->data[wa];
        c54x_run(dsp, 1);
        uint16_t npc = dsp->pc & 0xffff;
        if (wa >= 0 && dsp->data[wa] != wv) {
            printf("ECRITURE data[%04x] %04x -> %04x par pc=%04x tick=%u : AR0=%04x AR2=%04x AR3=%04x AR4=%04x BK=%04x "
                   "[4bcc]=%04x d_task_md(W0/W1)=%04x/%04x d_task_d=%04x/%04x\n",
                   wa, wv, dsp->data[wa], pc, g_c54x_exe_fn, dsp->ar[0], dsp->ar[2], dsp->ar[3], dsp->ar[4], dsp->bk,
                   dsp->data[0x4bcc], api[4], api[0x14 + 4], api[0], api[0x14]);
            if (getenv("REJEU_ANNEAU_W")) {
                int m = atoi(getenv("REJEU_ANNEAU_W"));
                for (int j = m; j > 0; j--) {
                    unsigned q = (apos - j) & (ANNEAU - 1);
                    printf("  %4d pc=%04x op=%04x %04x ar0=%04x ar2=%04x ar3=%04x ar4=%04x ar5=%04x sp=%04x\n", -j, anneau[q].pc,
                           anneau[q].op, anneau[q].op2, anneau[q].ar[0], anneau[q].ar[2], anneau[q].ar[3], anneau[q].ar[4], anneau[q].ar[5], anneau[q].sp);
                }
                exit(4);
            }
            static int nw; if (++nw >= 6) exit(4);
        }
        if (dsp->sp < 0x5900 || dsp->sp > 0x5c00 || (getenv("REJEU_STOP_PC") && npc == (uint16_t)strtoul(getenv("REJEU_STOP_PC"), NULL, 16)) || (getenv("REJEU_STOP_DEBUG") && dsp->data[0x08dc] != 0x0074 && g_c54x_exe_fn > 6200)) {
            plante = 1;
            printf("PLANTAGE tick=%u pc=%04x sp=%04x xpc=%d\n", g_c54x_exe_fn, npc, dsp->sp, dsp->xpc);
            int m = getenv("REJEU_ANNEAU") ? atoi(getenv("REJEU_ANNEAU")) : 120;
            for (int j = m; j > 0; j--) {
                unsigned q = (apos - j) & (ANNEAU - 1);
                printf("  %4d t=%u pc=%04x op=%04x %04x sp=%04x st0=%04x st1=%04x ar=%04x %04x %04x %04x %04x %04x %04x %04x a=%010llx b=%010llx\n",
                       -j, anneau[q].tick, anneau[q].pc, anneau[q].op, anneau[q].op2, anneau[q].sp, anneau[q].st0, anneau[q].st1,
                       anneau[q].ar[0], anneau[q].ar[1], anneau[q].ar[2], anneau[q].ar[3], anneau[q].ar[4], anneau[q].ar[5],
                       anneau[q].ar[6], anneau[q].ar[7], (unsigned long long)(anneau[q].a & 0xFFFFFFFFFFULL),
                       (unsigned long long)(anneau[q].b & 0xFFFFFFFFFFULL));
            }
            exit(3);
        }
    }
    return k;
}

/* Un tick = les enregistrements entre deux 'T'. */
typedef struct { size_t debut, fin; uint32_t tick; uint8_t drap; long budget; } Tick;

int main(int argc, char **argv)
{
    const char *nom = argc > 1 ? argv[1] : "/dev/shm/calypso_rejeu_tch.bin";
    long max_ticks = argc > 2 ? atol(argv[2]) : 100000;
    FILE *f = fopen(nom, "rb");
    if (!f) { perror(nom); return 1; }
    fseek(f, 0, SEEK_END); taille = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    fichier = malloc(taille);
    if (fread(fichier, 1, taille, f) != taille) { printf("lecture courte\n"); return 1; }
    fclose(f);

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
    for (unsigned i = 0; i < 7; i++) {
        char c[256]; snprintf(c, sizeof c, "/opt/GSM/calypso_dsp.%s.bin", R[i].s);
        if (c54x_load_section(dsp, c, R[i].a, R[i].p) < 0) { printf("ROM %s\n", c); return 1; }
    }
    c54x_load_registers(dsp, "/opt/GSM/calypso_dsp.Registers.bin");
    c54x_reset(dsp);
    calypso_dma_init();
    calypso_bsp_init(dsp);
    boot();

    /* Premiere passe : l'etat de depart et le decoupage en ticks. */
    static Tick ticks[40000]; int nt = 0;
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
        default: printf("enregistrement inconnu 0x%02x a %zu\n", k, p); goto fini;
        }
        if (p + l > taille) break;
        if (k == 'S') {
            memcpy(api, fichier + p + 5, API_WORDS * 2);
            printf("etat 'S' pose (tick %u)\n", be32(fichier + p + 1));
        } else if (k == 'D' && !getenv("REJEU_SANS_D")) {
            EnregRegs r; memcpy(&r, fichier + p + 5, sizeof r);
            uint16_t sauve_api[API_WORDS]; memcpy(sauve_api, api, sizeof sauve_api);
            memcpy(dsp->data, fichier + p + 5 + sizeof r, C54X_DATA_SIZE * 2);
            memcpy(api, sauve_api, sizeof sauve_api);    /* 'S' fait foi pour l'API RAM */
            dsp->a = r.a; dsp->b = r.b; memcpy(dsp->ar, r.ar, sizeof r.ar); dsp->t = r.t; dsp->trn = r.trn;
            dsp->sp = r.sp; dsp->bk = r.bk; dsp->brc = r.brc; dsp->rsa = r.rsa; dsp->rea = r.rea;
            dsp->st0 = r.st0; dsp->st1 = r.st1; dsp->pmst = r.pmst; dsp->imr = r.imr; dsp->ifr = r.ifr;
            dsp->xpc = r.xpc; dsp->pc = r.pc; dsp->idle = r.idle; dsp->running = r.running;
            printf("etat 'D' pose : pc=%04x sp=%04x idle=%d imr=%04x\n", r.pc & 0xffff, r.sp, r.idle, r.imr);
            {   /* REJEU_POKE=adr=val[,adr=val...] : corriger l'etat de depart */
                const char *e = getenv("REJEU_POKE");
                while (e && *e) {
                    unsigned a, v; if (sscanf(e, "%x=%x", &a, &v) != 2) break;
                    printf("poke data[%04x] = %04x (etait %04x)\n", a, v, dsp->data[a & 0xffff]);
                    dsp->data[a & 0xffff] = (uint16_t)v;
                    e = strchr(e, ','); if (e) e++;
                }
            }
        } else if (k == 'T') {
            if (nt > 0) ticks[nt - 1].fin = p;
            if (nt < 40000) {
                ticks[nt].debut = p; ticks[nt].tick = be32(fichier + p + 1);
                ticks[nt].drap = fichier[p + 5]; ticks[nt].budget = (long)be32(fichier + p + 6);
                nt++;
            }
        }
        p += l;
    }
fini:
    if (nt > 0) ticks[nt - 1].fin = p;
    printf("%d ticks enregistres (%u..%u)\n", nt, nt ? ticks[0].tick : 0, nt ? ticks[nt - 1].tick : 0);

    uint16_t prec_cd = 0xffff, prec_fd = 0xffff;
    int sacch_ok = 0, sacch_ko = 0;
    for (int it = 0; it < nt && it < max_ticks; it++) {
        Tick *t = &ticks[it];
        g_c54x_exe_fn = t->tick;
        long budget = t->budget;
        uint32_t dernier_livre = 0xffffffffu;
        /* ecritures ARM, fenetre 0 */
        for (size_t q = t->debut; q < t->fin;) {
            uint8_t k = fichier[q];
            size_t l = k == 'T' ? 10 : k == 'A' ? 8 + 4 * (size_t)be16(fichier + q + 6)
                     : k == 'B' ? 15 + 2 * (size_t)be16(fichier + q + 13) : 0;
            if (!l) break;
            if (k == 'A' && fichier[q + 5] == 0)
                for (unsigned i = 0, n = be16(fichier + q + 6); i < n; i++) {
                    unsigned a = be16(fichier + q + 8 + 4 * i);
                    if (a < API_WORDS) api[a] = be16(fichier + q + 10 + 4 * i);
                }
            q += l;
        }
        /* phase A : comme jouer_trame(phase 1) */
        calypso_dma_tick(dsp);
        reveil();
        if ((dsp->imr & (1u << C54X_IT_TPU_FRAME_BIT)) && (t->drap & 1))
            c54x_interrupt_ex(dsp, C54X_IT_TPU_FRAME_VEC, C54X_IT_TPU_FRAME_BIT);
        long fait = 0;
        if (!dsp->idle) fait = courir(budget / 8);
        while (!dsp->idle && fait < budget / 2) fait += courir(256);
        /* ecritures ARM, fenetre 1, puis livraisons I/Q dans l'ordre */
        for (size_t q = t->debut; q < t->fin;) {
            uint8_t k = fichier[q];
            size_t l = k == 'T' ? 10 : k == 'A' ? 8 + 4 * (size_t)be16(fichier + q + 6)
                     : k == 'B' ? 15 + 2 * (size_t)be16(fichier + q + 13) : 0;
            if (!l) break;
            if (k == 'A' && fichier[q + 5] == 1)
                for (unsigned i = 0, n = be16(fichier + q + 6); i < n; i++) {
                    unsigned a = be16(fichier + q + 8 + 4 * i);
                    if (a < API_WORDS) api[a] = be16(fichier + q + 10 + 4 * i);
                }
            if (k == 'B') {
                int n = be16(fichier + q + 13);
                static int16_t iq[1024];
                memcpy(iq, fichier + q + 15, 2 * (size_t)(n > 1024 ? 1024 : n));
                calypso_bsp_rx_burst(fichier[q + 5], be32(fichier + q + 6), iq, n);
                if (fichier[q + 5] == 0) dernier_livre = be32(fichier + q + 6);
                if (fichier[q + 5] == 0 && getenv("REJEU_SONDE")) {
                    int nwin = be16(fichier + q + 11) / 2, marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
                    memo_burst(be32(fichier + q + 6), iq, n, marge);
                }
            }
            q += l;
        }
        /* phase B : reste du budget, puis la pompe DMA de pont.c */
        reveil();
        if (!dsp->idle && budget - fait > 0) courir(budget - fait);
        for (int k = 0; k < 40 && dsp->running; k++) {
            if (!calypso_rhea_dma_pump(dsp)) break;
            if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
            if (!dsp->idle) courir(budget / 4);
        }
        uint16_t *cd = &api[NDB + 0x1FC / 2], *fd = &api[NDB + 0x21A / 2];
        {   static int dit;
            if (!dsp->idle && dit < 5) { dit++; printf("tick=%u : DSP PAS A L'IDLE en fin de tick, pc=%04x sp=%04x\n", t->tick, dsp->pc & 0xffff, dsp->sp); } }
        {   /* REJEU_DUMP=t1-t2,dossier : memoire de donnees du DSP en fin de tick */
            const char *e = getenv("REJEU_DUMP");
            unsigned t1, t2; char dos[512];
            if (e && sscanf(e, "%u-%u,%511s", &t1, &t2, dos) == 3 && t->tick >= t1 && t->tick <= t2) {
                char nm[600]; snprintf(nm, sizeof nm, "%s/mem_%u.bin", dos, t->tick);
                FILE *fm = fopen(nm, "wb"); if (fm) { fwrite(dsp->data, 2, C54X_DATA_SIZE, fm); fclose(fm); }
            }
        }
        if (getenv("REJEU_SONDE") && dernier_livre != 0xffffffffu && dernier_livre % 104 == 12) {
            printf("tick=%u fn=%u (4e burst SACCH TS2) : a_cd0=%04x %s err=%u\n", t->tick, dernier_livre, cd[0],
                   (cd[0] & 0x8000) ? ((cd[0] & 0x40) ? "FIRE KO" : "ok") : "pas de BLUD", cd[2]);
            sonde_bloc(dernier_livre);
        }
        if (cd[0] != prec_cd && (cd[0] & 0x8000)) {
            bool ko = cd[0] & (1u << 6);
            ko ? sacch_ko++ : sacch_ok++;
            printf("tick=%u SACCH a_cd0=%04x %s err=%u L2=%02x %02x %02x %02x\n", t->tick, cd[0],
                   ko ? "FIRE KO" : "ok", cd[2], cd[3] & 0xff, cd[3] >> 8, cd[4] & 0xff, cd[4] >> 8);

        }
        if (fd[0] != prec_fd && (fd[0] & 0x8000))
            printf("tick=%u FACCH a_fd0=%04x %s L2=%02x %02x %02x\n", t->tick, fd[0],
                   (fd[0] & (1u << 6)) ? "FIRE KO" : "ok", fd[3] & 0xff, fd[3] >> 8, fd[4] & 0xff);
        prec_cd = cd[0]; prec_fd = fd[0];
    }
    printf("SACCH : %d bonnes, %d Fire KO\n", sacch_ok, sacch_ko);
    return 0;
}
