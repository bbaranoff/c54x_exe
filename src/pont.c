/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * pont.c - c54x_exe cote serveur du pont ARM/DSP.
 *
 * [2026-09-16] L'ARM (layer1 osmocom-bb) tourne sous QEMU (qosmo,
 * CALYPSO_DSP_EXTERN=1) ; le C54x tourne ici. Les deux se partagent l'API RAM
 * par /dev/shm/calypso_api_ram et se verrouillent trame par trame sur
 * /tmp/calypso_dsp.sock. La procedure par trame recopie section par section
 * ce que qosmo-dsp/hw/arm/calypso/calypso_trx.c:calypso_tdma_tick() fait avec
 * son DSP interne - si les deux divergent, c'est ici qu'il faut regarder.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <math.h>
#include <sys/stat.h>
#include "calypso_c54x.h"
#include "calypso_dma.h"
#include "calypso_bsp.h"
#include "calypso_rhea_dma.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_dsp_pont.h"
#include "pont.h"
#include "gmsk.h"
#include "cellule.h"

extern uint32_t g_c54x_exe_fn;          /* main.c : ce que calypso_trx_get_fn() rend */

static volatile sig_atomic_t g_stop;
static void sur_signal(int sig) { (void)sig; g_stop = 1; }

C54xState *pont_allouer_dsp(void)
{
    /* data[] est a offsetof(C54xState, data) dans la structure (202 + 512 Ko
     * de prog[]) : on decale le debut de la structure pour que data[] tombe
     * sur une page, et donc data[0x0800] aussi (0x1000 octets plus loin). */
    size_t off = offsetof(C54xState, data) % 4096;
    size_t decal = off ? 4096 - off : 0;
    void *brut = NULL;
    if (posix_memalign(&brut, 4096, sizeof(C54xState) + 4096) != 0) {
        fprintf(stderr, "pont : posix_memalign\n");
        return NULL;
    }
    memset(brut, 0, sizeof(C54xState) + 4096);
    C54xState *s = (C54xState *)((char *)brut + decal);
    uint16_t *fenetre = &s->data[C54X_API_BASE];
    if (((uintptr_t)fenetre & 4095) != 0) {
        fprintf(stderr, "pont : fenetre API non alignee (%p)\n", (void *)fenetre);
        return NULL;
    }

    int fd = shm_open(CALYPSO_PONT_SHM, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        fprintf(stderr, "pont : shm_open(%s) : %s\n", CALYPSO_PONT_SHM, strerror(errno));
        return NULL;
    }
    if (ftruncate(fd, CALYPSO_PONT_SHM_BYTES) < 0) {
        fprintf(stderr, "pont : ftruncate : %s\n", strerror(errno));
        close(fd);
        return NULL;
    }
    void *map = mmap(fenetre, CALYPSO_PONT_SHM_BYTES, PROT_READ | PROT_WRITE,
                     MAP_SHARED | MAP_FIXED, fd, 0);
    close(fd);
    if (map == MAP_FAILED || map != (void *)fenetre) {
        fprintf(stderr, "pont : mmap MAP_FIXED : %s\n", strerror(errno));
        return NULL;
    }
    memset(fenetre, 0, CALYPSO_PONT_SHM_BYTES);
    return s;
}

static bool envoyer(int fd, uint32_t type, uint32_t a, uint32_t b)
{
    CalypsoPontMsg m = { type, a, b };
    return send(fd, &m, sizeof(m), MSG_NOSIGNAL) == (ssize_t)sizeof(m);
}

/* Une trame TDMA, dans l'ordre de calypso_tdma_tick() de qosmo-dsp :
 *   1. DMA tick
 *   2. boot : c54x_run jusqu'au premier IDLE (dsp_init_done)
 *   5. IRQ TPU-frame si l'IMR l'arme, un budget de c54x_run, front -> IDLE
 * Renvoie les drapeaux de PONT_DONE, et le nombre d'instructions dans *insns. */
/* Profil de PC : c54x_run par tranches de 64 instructions, le PC releve entre
 * deux tranches tombe dans un seau de 64 mots. Exact (pas d'echantillonnage
 * par fil), et gratuit. Publie toutes les 217 trames : quelles routines de la
 * ROM tournent vraiment, avec et sans burst - c'est la question du README. */
#define PROFIL_SEAU 64
static unsigned long g_profil[0x10000 / PROFIL_SEAU];
static uint32_t g_profil_hw;     /* high-water : PC le plus haut vu (hors boot) */
/* Fenetre de trace : PONT_TRACE_FB=N -> a la premiere trame ou la page
 * d'ecriture porte d_task_md=5 (FB_DSP_TASK), les N instructions suivantes
 * sont journalisees (pc, opcode, A, AR3, DP, INTM, IMR/IFR) dans
 * /tmp/c54x-pont/trace-fb.txt. Pas a pas exact (c54x_run par 1). */
static long g_trace_reste = -1;
static FILE *g_trace_f;
static bool g_trace_ouverte;
static uint32_t g_trace_pc_lo, g_trace_pc_hi;   /* PONT_TRACE_PC=lo-hi : ouvrir quand PC entre dans [lo,hi] */
static void trace_armer(void)
{
    static bool fait;
    if (fait) return;
    fait = true;
    const char *e = getenv("PONT_TRACE_FB");
    const char *r = getenv("PONT_TRACE_PC");
    if (e && *e) { g_trace_reste = atol(e); g_trace_f = fopen("/tmp/c54x-pont/trace-fb.txt", "w"); }
    if (r && *r) {
        char *fin = NULL; g_trace_pc_lo = (uint32_t)strtoul(r, &fin, 0);
        g_trace_pc_hi = (fin && *fin == '-') ? (uint32_t)strtoul(fin + 1, NULL, 0) : g_trace_pc_lo;
        if (g_trace_reste <= 0) g_trace_reste = 4000;
        if (!g_trace_f) g_trace_f = fopen("/tmp/c54x-pont/trace-fb.txt", "w");
    }
}
/* Cellules surveillees pendant la trace : tout changement est journalise avec
 * le PC de l'instruction qui l'a fait (diff avant/apres chaque pas). */
static const uint16_t g_cellules[] = { 0x43d8, 0x43d5, 0x3f92, 0x098c, 0x098a, 0x435e, 0x435b, 0x4368,
    0x0810, 0x08d4, 0x0804, 0x0818, 0x058a, 0x3fb0, 0x3fc1, 0x3fde, 0x3fdc, 0x3fd3, 0x0c3f, 0x08f8, 0x08fa, 0x0906,
    0x0837, 0x0838, 0x083a, 0x083b, 0x084b, 0x084c, 0x084e, 0x084f,   /* a_sch[0,1,3,4] pages R0/R1 */
    0x0e4e, 0x0e4f, 0x0e60, 0x2a00, 0x2a01, 0x0000 };
#define N_CELLULES (sizeof(g_cellules) / sizeof(g_cellules[0]))
static uint16_t g_cell_prev[N_CELLULES];
static uint16_t g_trace_pc_prev;
static void trace_cellules(C54xState *dsp, bool init)
{
    for (unsigned i = 0; i < N_CELLULES; i++) {
        uint16_t v = dsp->data[g_cellules[i]];
        if (!init && v != g_cell_prev[i] && g_trace_f)
            fprintf(g_trace_f, "      W data[%04x] %04x -> %04x   (par pc=%04x)\n",
                    g_cellules[i], g_cell_prev[i], v, g_trace_pc_prev);
        g_cell_prev[i] = v;
    }
}

static inline void trace_pas(C54xState *dsp)
{
    if (g_trace_reste <= 0 || !g_trace_f || !g_trace_ouverte) return;
    trace_cellules(dsp, false);
    g_trace_pc_prev = dsp->pc & 0xffff;
    uint16_t op = dsp->prog[dsp->pc & 0x3ffff];
    uint16_t op2 = dsp->prog[(dsp->pc + 1) & 0x3ffff];
    fprintf(g_trace_f, "%04x  %04x %04x  A=%010llx B=%010llx AR1=%04x AR2=%04x AR3=%04x AR4=%04x DP=%03x INTM=%d IMR=%04x IFR=%04x SP=%04x T=%04x BRC=%04x RSA=%04x REA=%04x ST0=%04x ST1=%04x insn=%u\n",
            dsp->pc & 0xffff, op, op2,
            (unsigned long long)(dsp->a & 0xffffffffffULL), (unsigned long long)(dsp->b & 0xffffffffffULL),
            dsp->ar[1], dsp->ar[2], dsp->ar[3], dsp->ar[4], dsp->st0 & 0x1ff, !!(dsp->st1 & 0x800),
            dsp->imr, dsp->ifr, dsp->sp, dsp->t, dsp->brc, dsp->rsa, dsp->rea, dsp->st0, dsp->st1, dsp->insn_count);
    if (--g_trace_reste == 0) { fclose(g_trace_f); g_trace_f = NULL; printf("pont : trace FB terminee (/tmp/c54x-pont/trace-fb.txt)\n"); }
}

/* PONT_PC_COUNT=pc1,pc2,... : compteurs EXACTS de passage (execution pas a pas,
 * ~3x plus lent ; diagnostic). Publies avec le profil. */
static uint16_t g_pcc[16]; static unsigned long g_pcc_n[16]; static int g_pcc_k = -1;
static void pcc_armer(void)
{
    if (g_pcc_k >= 0) return;
    g_pcc_k = 0;
    const char *e = getenv("PONT_PC_COUNT");
    while (e && *e && g_pcc_k < 16) {
        char *fin = NULL; long v = strtol(e, &fin, 0);
        if (fin == e) break;
        g_pcc[g_pcc_k++] = (uint16_t)v;
        e = (*fin == ',') ? fin + 1 : fin;
    }
}
/* PONT_DUMP_DATA=addr:n,addr:n : mots de DARAM affiches avec le profil (vecteurs, tremplins). */
static void dump_publier(C54xState *dsp)
{
    const char *e = getenv("PONT_DUMP_DATA");
    while (e && *e) {
        char *fin = NULL; long a = strtol(e, &fin, 0); long n = 8;
        if (fin == e) break;
        if (*fin == ':') n = strtol(fin + 1, &fin, 0);
        printf("  data[%04lx..] :", a);
        for (long i = 0; i < n && i < 32; i++) printf(" %04x", dsp->data[(a + i) & 0xffff]);
        printf("\n");
        e = (*fin == ',') ? fin + 1 : fin;
    }
}
static void pcc_publier(void)
{
    if (g_pcc_k <= 0) return;
    printf("  passages exacts :");
    for (int i = 0; i < g_pcc_k; i++) { printf(" %04x:%lu", g_pcc[i], g_pcc_n[i]); g_pcc_n[i] = 0; }
    printf("\n");
}

static int c54x_run_profile(C54xState *dsp, int budget)
{
    int fait = 0;
    pcc_armer();
    while (fait < budget && !dsp->idle && dsp->running) {
        int n = budget - fait < PROFIL_SEAU ? budget - fait : PROFIL_SEAU;
        static long minfn = -1;
        if (minfn < 0) { const char *e = getenv("PONT_TRACE_MINFN"); minfn = (e && *e) ? atol(e) : 0; }
        if (g_trace_reste > 0 && !g_trace_ouverte && g_trace_pc_hi && (long)g_c54x_exe_fn >= minfn &&
            (dsp->pc & 0xffff) >= g_trace_pc_lo && (dsp->pc & 0xffff) <= g_trace_pc_hi && g_trace_f) {
            g_trace_ouverte = true;
            trace_cellules(dsp, true);
            fprintf(g_trace_f, "# PC=%04x dans [%04x,%04x] a fn=%u ; IMR=%04x IFR=%04x\n",
                    dsp->pc & 0xffff, g_trace_pc_lo, g_trace_pc_hi, g_c54x_exe_fn, dsp->imr, dsp->ifr);
            printf("pont : trace ouverte sur PC=%04x fn=%u\n", dsp->pc & 0xffff, g_c54x_exe_fn);
        }
        if (g_trace_reste > 0 && g_trace_ouverte) n = 1;
        if (g_pcc_k > 0) {
            n = 1;
            uint16_t pc = dsp->pc & 0xffff;
            for (int i = 0; i < g_pcc_k; i++) if (g_pcc[i] == pc) g_pcc_n[i]++;
        }
        if (n == 1) trace_pas(dsp);
        /* [2026-09-17] compter ce que c54x_run dit avoir execute, pas le delta
         * de insn_count : les repetitions d'un RPT n'incrementent pas
         * insn_count, et un pas (n=1) ou un seau entier tombe dans un
         * « rpt *(lk) ; nop » de plusieurs centaines de tours faisait croire a
         * un DSP bloque -> la trame s'arretait au milieu de l'ISR (le faux
         * blocage a 0xa4eb/0xa4f0 des traces). */
        int ex = c54x_run(dsp, n);
        fait += ex;
        g_profil[(dsp->pc & 0xffff) / PROFIL_SEAU]++;
        if ((dsp->pc & 0xffff) > g_profil_hw) g_profil_hw = dsp->pc & 0xffff;
        if (ex <= 0) break;
    }
    return fait;
}

static void profil_publier(void)
{
    unsigned long tot = 0; unsigned n = 0;
    for (unsigned i = 0; i < 0x10000 / PROFIL_SEAU; i++) tot += g_profil[i];
    if (!tot) return;
    printf("  profil PC (%lu tranches, high-water 0x%04x) :", tot, g_profil_hw);
    /* les 12 seaux les plus chauds, par ordre decroissant */
    for (int k = 0; k < 12; k++) {
        unsigned best = 0; unsigned long bv = 0;
        for (unsigned i = 0; i < 0x10000 / PROFIL_SEAU; i++)
            if (g_profil[i] > bv) { bv = g_profil[i]; best = i; }
        if (!bv) break;
        printf(" %04x:%lu%%", best * PROFIL_SEAU, bv * 100 / tot);
        g_profil[best] = 0; n++;
    }
    printf("\n");
    memset(g_profil, 0, sizeof(g_profil)); g_profil_hw = 0;
}

static uint32_t jouer_trame(C54xState *dsp, long budget, bool *init_done, uint32_t *insns)
{
    uint32_t drapeaux = 0;
    uint32_t avant = dsp->insn_count;

    calypso_dma_tick(dsp);

    if (dsp->running && !*init_done) {
        if (!dsp->idle) {
            c54x_run(dsp, (int)budget);
        }
        if (dsp->idle) {
            *init_done = true;
            drapeaux |= PONT_DONE_INIT;
        }
    }
    if (dsp->running) {
        bool etait_idle = dsp->idle;
        /* [2026-09-17] Reveil sur IT en niveau / en attente. Le coeur ne vectorise
         * une IT en attente (IFR&IMR, INTM=0) qu'a l'instruction suivante
         * (c54x_irq_level_check, CALYPSO_C54X_IRQ_LEVEL=1) et un DSP en IDLE
         * n'execute rien : sur silicium c'est l'IT qui le reveille (SPRU131).
         * INT10n (fin de DMA RHEA, bit 14) est une ligne : tant qu'un canal
         * garde IRQ_STATE, elle est tenue ; on la presente au reveil. */
        if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) &&
            !(dsp->ifr & (1u << 14)))
            c54x_interrupt_ex(dsp, 30, 14);
        if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800))
            dsp->idle = false;
        if (dsp->imr & (1u << C54X_IT_TPU_FRAME_BIT)) {
            c54x_interrupt_ex(dsp, C54X_IT_TPU_FRAME_VEC, C54X_IT_TPU_FRAME_BIT);
        }
        if (getenv("PONT_IRQ_DEBUG") && g_c54x_exe_fn > 5000 && g_c54x_exe_fn < 5012)
            printf("  [irq] fn=%u APRES vec28 : idle=%d pc=%04x INTM=%d IMR=%04x IFR=%04x\n",
                   g_c54x_exe_fn, dsp->idle, dsp->pc & 0xffff, !!(dsp->st1 & 0x800), dsp->imr, dsp->ifr);
        if (!dsp->idle) {
            c54x_run_profile(dsp, (int)budget);
        }
        if (!etait_idle && dsp->idle) {
            drapeaux |= PONT_DONE_API_IRQ;
        }
    }
    if (dsp->idle)    drapeaux |= PONT_DONE_IDLE;
    if (dsp->running) drapeaux |= PONT_DONE_RUNNING;
    if (*init_done)   drapeaux |= PONT_DONE_INIT;
    *insns = dsp->insn_count - avant;
    return drapeaux;
}

/* ── injection I/Q synthetique ──────────────────────────────────────────────
 * Ce que le DSP recoit sur silicium : 148 echantillons complexes int16 par
 * burst, 1 par symbole, deposes par le DMA BSP en DARAM 0x2a00 (296 mots),
 * puis l'IT RX. calypso_bsp_rx_burst() fait exactement ca (ecriture sous
 * verrou, vecteur TPU-frame, vecteur RX). Une FCCH = GMSK de 148 zeros = une
 * porteuse a +1625/24 kHz = +pi/2 de phase par symbole a 270,833 ksymb/s. */
#define IQ_N 148
static void iq_synthese(const char *mode, int amp, uint32_t fn, int16_t *iq)
{
    static unsigned seed = 12345;
    double dphi = 0;
    bool bruit = false;
    if (!strcmp(mode, "fcch")) {
        dphi = M_PI / 2;
    } else if (!strcmp(mode, "noise")) {
        bruit = true;
    } else if (!strncmp(mode, "tone:", 5)) {
        dphi = atof(mode + 5);
    }
    for (int k = 0; k < IQ_N; k++) {
        if (bruit) {
            seed = seed * 1103515245u + 12345u; int a = (int)((seed >> 16) % (2 * amp + 1)) - amp;
            seed = seed * 1103515245u + 12345u; int b = (int)((seed >> 16) % (2 * amp + 1)) - amp;
            iq[2 * k] = (int16_t)a; iq[2 * k + 1] = (int16_t)b;
        } else {
            double ph = dphi * k;
            iq[2 * k]     = (int16_t)lrint(amp * cos(ph));
            iq[2 * k + 1] = (int16_t)lrint(amp * sin(ph));
        }
    }
    (void)fn;
}

/* Injection d'un burst (cellule synthetique ou signal simple) dans le RIF/BSP. */
static void injecter_burst(C54xState *dsp, const char *iq_mode, int amp, uint32_t fn,
                           unsigned long *injectes)
{
        int16_t iq[2 * 256];
        int n_iq = 2 * IQ_N;
        if (!strncmp(iq_mode, "cell", 4)) {
            /* cell[:bsic[:decalage[:marge]]] : la cellule complete, FCCH/SCH/factice */
            static int bsic = -1; static double dec = 0.5; static int marge = 21;
            if (bsic < 0) { bsic = 42; const char *p = strchr(iq_mode, ':');
                if (p) { bsic = atoi(p + 1) & 0x3f; p = strchr(p + 1, ':');
                    if (p) { dec = atof(p + 1); p = strchr(p + 1, ':'); if (p) marge = atoi(p + 1); } }
                if (marge > 50) marge = 50;
                if (strstr(iq_mode, "all")) cellule_sch_partout = 1;
                printf("pont : cellule BSIC=%d (NCC=%d BCC=%d), echantillonnage a %.2f symbole, "
                       "SCH dans une fenetre de %d echantillons (DARAM len=%u mots)%s\n",
                       bsic, bsic >> 3, bsic & 7, dec, 148 + 2 * marge, calypso_bsp_get_daram_len(),
                       cellule_sch_partout ? ", SCH sur toutes les trames non-FCCH" : ""); }
            char t = cellule_burst(fn, (uint8_t)bsic, amp, dec, marge, iq, &n_iq);
            static unsigned nS, nF;
            if (t == 'S') nS++; else if (t == 'F') nF++;
            if ((nS + nF) && (nS + nF) % 500 == 1) printf("pont : bursts injectes FCCH=%u SCH=%u (fn=%u)\n", nF, nS, fn);
        } else {
            iq_synthese(iq_mode, amp, fn, iq);
        }
        static int irqdbg = -1; static unsigned irqdbg_n;
        if (irqdbg < 0) irqdbg = getenv("PONT_IRQ_DEBUG") ? 1 : 0;
        bool dbg = irqdbg && fn > 5000 && irqdbg_n < 12;
        if (dbg) printf("  [irq] fn=%u AVANT rx_burst : idle=%d pc=%04x INTM=%d IMR=%04x IFR=%04x PMST=%04x SP=%04x\n",
                        fn, dsp->idle, dsp->pc & 0xffff, !!(dsp->st1 & 0x800), dsp->imr, dsp->ifr, dsp->pmst, dsp->sp);
        calypso_bsp_rx_burst(0, fn, iq, n_iq);
        if (dbg) { printf("  [irq] fn=%u APRES rx_burst : idle=%d pc=%04x INTM=%d IMR=%04x IFR=%04x SP=%04x\n",
                          fn, dsp->idle, dsp->pc & 0xffff, !!(dsp->st1 & 0x800), dsp->imr, dsp->ifr, dsp->sp); irqdbg_n++; }
        (*injectes)++;
}

static void servir(int fd, C54xState *dsp, uint16_t *api_ram, long insns, bool verbeux,
                   const char *iq_mode, int amp)
{
    bool injecter = iq_mode && *iq_mode && strcmp(iq_mode, "none") != 0;
    unsigned long injectes = 0;
    const uint16_t *a_sync = &api_ram[(API_NDB + NDB_A_SYNC_DEMOD) / 2];
    bool init_done = false;
    unsigned long trames = 0, irqs = 0, resets = 0;
    uint64_t insns_total = 0;
    const uint16_t *d_fb_det = &api_ram[(API_NDB + NDB_D_FB_DET) / 2];
    const uint16_t *a_sch0   = &api_ram[(API_R_PAGE(0) + RP_A_SCH) / 2];

    CalypsoPontMsg m;
    if (recv(fd, &m, sizeof(m), 0) != (ssize_t)sizeof(m) || m.type != PONT_HELLO ||
        m.a != CALYPSO_API_WORDS || m.b != CALYPSO_PONT_MAGIC) {
        fprintf(stderr, "pont : poignee de main invalide (type=%u a=%u)\n", m.type, m.a);
        return;
    }
    envoyer(fd, PONT_HELLO_OK, CALYPSO_API_WORDS, CALYPSO_PONT_MAGIC);
    printf("pont : ARM connecte, API RAM %u mots, %ld insn/trame\n", CALYPSO_API_WORDS, insns);
    fflush(stdout);

    while (!g_stop) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        if (poll(&pfd, 1, 200) <= 0) {
            continue;
        }
        ssize_t n = recv(fd, &m, sizeof(m), 0);
        if (n != (ssize_t)sizeof(m)) {
            printf("pont : ARM deconnecte (%zd)\n", n);
            break;
        }
        switch (m.type) {
        case PONT_RESET:
            /* qosmo-dsp : c54x_reset, running, boot par les ticks, d_dsp_page=0 */
            c54x_reset(dsp);
            dsp->running = true;
            init_done = false;
            api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2] = 0;
            resets++;
            printf("pont : RESET #%lu (DL_STATUS=0x%04x) fn=%u pc=0x%04x\n",
                   resets, m.a, g_c54x_exe_fn, dsp->pc);
            break;
        case PONT_TICK: {
            g_c54x_exe_fn = m.a;
            calypso_bsp_set_tpu_offset((int)m.c);   /* [2026-09-17] fenetre RX du firmware */
            { static unsigned _to=0; if (getenv("PONT_TPU_DEBUG") && (_to<5 || _to%2000==0)) printf("  [tpu] fn=%u tpu_offset=%u\n", m.a, m.c); _to++; }
            trace_armer();
            if (g_trace_reste > 0 && g_trace_f && !g_trace_ouverte && !g_trace_pc_hi) {
                /* attendre la premiere tache FB posee par l'ARM (page W0 ou W1) */
                static int md_cible = -1;
                if (md_cible < 0) { const char *e = getenv("PONT_TRACE_MD"); md_cible = (e && *e) ? atoi(e) : 5; }
                if (api_ram[(API_W_PAGE(0) + WP_D_TASK_MD) / 2] == md_cible ||
                    api_ram[(API_W_PAGE(1) + WP_D_TASK_MD) / 2] == md_cible) {
                    g_trace_ouverte = true;
                    trace_cellules(dsp, true);
                    fprintf(g_trace_f, "# d_task_md=%d vu a fn=%u page=%u ; IMR=%04x IFR=%04x pc=%04x\n",
                            md_cible, m.a, m.b, dsp->imr, dsp->ifr, dsp->pc & 0xffff);
                    printf("pont : trace FB ouverte a fn=%u\n", m.a);
                }
            }
            /* [2026-09-17] PONT_RX_APRES (defaut 1) : le burst arrive APRES l'IT
             * trame, comme sur silicium (la fenetre RX est programmee par le TPU dans
             * la trame, le DSP arme DMA2 dans l'ISR trame, les echantillons tombent
             * ensuite). Avec l'injection avant l'IT, la requete RX du RIF trouvait
             * DMA2 ENABLE=0 et le job SB decodait la fenetre du tick precedent. */
            static int rx_apres = -1;
            if (rx_apres < 0) { const char *e = getenv("PONT_RX_APRES"); rx_apres = (e && *e == '0') ? 0 : 1; }
            if (!rx_apres && injecter && init_done) injecter_burst(dsp, iq_mode, amp, m.a, &injectes);
            /* [2026-09-17] Chaîne réelle : vider la socket UDP 6702 (bursts du
             * pont/BTS) et les livrer au DSP. Sans injection synthétique c'est la
             * seule source. Harmless si la socket est vide. */
            if (init_done) calypso_bsp_service(m.a);
            uint32_t ninsn = 0;
            bool init_avant = init_done;
            uint32_t drapeaux = jouer_trame(dsp, insns, &init_done, &ninsn);
            if (rx_apres && injecter && init_done && dsp->running) {
                injecter_burst(dsp, iq_mode, amp, m.a, &injectes);
                /* fin de DMA dans la meme trame : reveil sur INT10n tenue, puis on
                 * laisse le DSP finir (ISR de fin de DMA + fond) jusqu'a l'IDLE. */
                uint32_t avant = dsp->insn_count;
                if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) &&
                    !(dsp->ifr & (1u << 14)))
                    c54x_interrupt_ex(dsp, 30, 14);
                if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
                if (!dsp->idle) c54x_run_profile(dsp, (int)insns);
                ninsn += dsp->insn_count - avant;
                drapeaux = (drapeaux & ~PONT_DONE_IDLE) | (dsp->idle ? PONT_DONE_IDLE : 0);
            }
            /* [2026-09-17] CANNING PREMIERE APPROCHE (pas la fin — echafaudage
             * pour prouver le pipeline jusqu'au LU). PONT_CAN_TOA=23 : force le TOA
             * rapporte (a_sync_demod[D_TOA]) a la valeur « on-time » attendue par le
             * firmware (prim_fbsb.c: toa -= 23). A retirer quand le TOA natif du
             * correlateur est bon. -1 (defaut) = pas de canning. */
            {
                static int can_toa = -2;
                if (can_toa == -2) { const char *e = getenv("PONT_CAN_TOA"); can_toa = (e && *e) ? atoi(e) : -1; }
                if (can_toa >= 0 && *d_fb_det)
                    api_ram[(API_NDB + NDB_A_SYNC_DEMOD) / 2 + D_TOA] = (uint16_t)can_toa;
                /* SB : le firmware lit a_serv_demod[D_TOA] (read page) et veut ~4.
                 * PONT_CAN_SB_TOA=4 le cale sur les deux pages R (cadre le SCH). */
                static int can_sb = -2;
                if (can_sb == -2) { const char *e = getenv("PONT_CAN_SB_TOA"); can_sb = (e && *e) ? atoi(e) : -1; }
                if (can_sb >= 0) {
                    api_ram[(API_R_PAGE(0) + RP_A_SERV_DEMOD) / 2 + D_TOA] = (uint16_t)can_sb;
                    api_ram[(API_R_PAGE(1) + RP_A_SERV_DEMOD) / 2 + D_TOA] = (uint16_t)can_sb;
                }
            }
            if (*d_fb_det) calypso_bsp_toa_feedback((int)(int16_t)a_sync[0]);  /* verrou TOA natif */
            trames++;
            insns_total += ninsn;
            if (drapeaux & PONT_DONE_API_IRQ) irqs++;
            envoyer(fd, PONT_DONE, drapeaux, ninsn);
            if (!init_avant && init_done) {
                printf("pont : DSP boote (premier IDLE) fn=%u insn=%u\n", m.a, dsp->insn_count);
            }
            if ((trames % 217) == 0) {
                profil_publier();
                pcc_publier();
                dump_publier(dsp);
            }
            if (verbeux || (trames % 217) == 0) {
                printf("  fn=%-7u page=%u insn=%-7u %s%s%s  d_fb_det=%-5u a_sch=%04x %04x %04x %04x"
                       " sync=%04x %04x %04x  | trames=%lu irq=%lu iq=%lu\n",
                       m.a, m.b, ninsn,
                       (drapeaux & PONT_DONE_IDLE) ? "IDLE " : "occupe ",
                       (drapeaux & PONT_DONE_API_IRQ) ? "IRQ-API " : "",
                       (drapeaux & PONT_DONE_INIT) ? "" : "boot ",
                       *d_fb_det, a_sch0[0], a_sch0[1], a_sch0[2], a_sch0[3],
                       a_sync[0], a_sync[1], a_sync[2], trames, irqs, injectes);
            }
            fflush(stdout);
            break;
        }
        case PONT_BYE:
            printf("pont : BYE\n");
            return;
        default:
            fprintf(stderr, "pont : message inconnu type=%u\n", m.type);
            break;
        }
    }
    printf("pont : bilan de la session : %lu trames, %lu IRQ API, %lu reset, %llu insn\n",
           trames, irqs, resets, (unsigned long long)insns_total);
}

int pont_serveur(C54xState *dsp, uint16_t *api_ram, const char *socket_path,
                 long insns, bool verbeux, const char *iq_mode, int amp)
{
    signal(SIGINT, sur_signal);
    signal(SIGTERM, sur_signal);

    int srv = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (srv < 0) {
        fprintf(stderr, "pont : socket : %s\n", strerror(errno));
        return 1;
    }
    struct sockaddr_un sa = { .sun_family = AF_UNIX };
    snprintf(sa.sun_path, sizeof(sa.sun_path), "%s", socket_path);
    unlink(socket_path);
    if (bind(srv, (struct sockaddr *)&sa, sizeof(sa)) < 0 || listen(srv, 1) < 0) {
        fprintf(stderr, "pont : bind/listen(%s) : %s\n", socket_path, strerror(errno));
        close(srv);
        return 1;
    }
    chmod(socket_path, 0666);
    if (iq_mode && *iq_mode && strcmp(iq_mode, "none") != 0)
        printf("pont : injection I/Q « %s » amplitude %d a chaque trame (DARAM 0x%04x, %u mots)\n",
               iq_mode, amp, calypso_bsp_get_daram_addr(), calypso_bsp_get_daram_len());
    printf("pont : en attente de l'ARM sur %s (API RAM : /dev/shm%s)\n"
           "       cote QEMU : CALYPSO_DSP_EXTERN=1 qemu-system-arm -M calypso ...\n",
           socket_path, CALYPSO_PONT_SHM);
    fflush(stdout);

    while (!g_stop) {
        struct pollfd pfd = { .fd = srv, .events = POLLIN };
        if (poll(&pfd, 1, 200) <= 0) {
            continue;
        }
        int fd = accept(srv, NULL, NULL);
        if (fd < 0) {
            continue;
        }
        servir(fd, dsp, api_ram, insns, verbeux, iq_mode, amp);
        close(fd);
        fflush(stdout);
    }
    close(srv);
    unlink(socket_path);
    return 0;
}
