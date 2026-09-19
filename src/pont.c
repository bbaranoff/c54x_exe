/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * pont.c - server side of the ARM/DSP bridge.
 *
 * The ARM (osmocom-bb layer1) runs under QEMU (qosmo, CALYPSO_DSP_EXTERN=1);
 * the C54x runs in this process. Both share the API RAM through
 * /dev/shm/calypso_api_ram and lock step frame by frame over
 * /tmp/calypso_dsp.sock. The per-frame sequence below mirrors, section by
 * section, what qosmo-dsp/hw/arm/calypso/calypso_trx.c:calypso_tdma_tick()
 * does with its internal DSP; divergences between the two originate here.
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
#include "calypso_twl3025.h"
#include "calypso_rhea_dma.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_dsp_pont.h"
#include "pont.h"
#include "gmsk.h"
#include "cellule.h"

extern uint32_t g_c54x_exe_fn;          /* main.c: value returned by calypso_trx_get_fn() */

static volatile sig_atomic_t g_stop;
static void sur_signal(int sig) { (void)sig; g_stop = 1; }

C54xState *pont_allouer_dsp(void)
{
    /* Shift the start of the struct so that data[] lands on a page boundary,
     * and therefore data[C54X_API_BASE] too: the API window must be mmap-able
     * with MAP_FIXED. */
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

/* PC profile: c54x_run is driven in slices of 64 instructions and the PC
 * sampled between two slices is charged to a 64-word bucket. No sampling
 * thread, no cost. Published every 217 frames. */
#define PROFIL_SEAU 64
static unsigned long g_profil[0x10000 / PROFIL_SEAU];
static uint32_t g_profil_hw;     /* high-water mark: highest PC seen (boot excluded) */
/* Trace window: PONT_TRACE_FB=N logs the next N instructions (pc, opcode, A,
 * AR3, DP, INTM, IMR/IFR) to /tmp/c54x-pont/trace-fb.txt, starting at the
 * first frame whose write page carries d_task_md=5 (FB_DSP_TASK). Exact
 * single-stepping (c54x_run with n=1). */
static long g_trace_reste = -1;
static FILE *g_trace_f;
static bool g_trace_ouverte;
static uint32_t g_trace_pc_lo, g_trace_pc_hi;   /* PONT_TRACE_PC=lo-hi: open the trace when PC enters [lo,hi] */
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
/* DARAM cells watched during the trace: every change is logged with the PC of
 * the instruction that made it (before/after diff on each step). */
static const uint16_t g_cellules[] = { 0x43d8, 0x43d5, 0x3f92, 0x098c, 0x098a, 0x435e, 0x435b, 0x4368,
    0x0810, 0x08d4, 0x0804, 0x0818, 0x058a, 0x3fb0, 0x3fc1, 0x3fde, 0x3fdc, 0x3fd3, 0x0c3f, 0x08f8, 0x08fa, 0x0906,
    0x0837, 0x0838, 0x083a, 0x083b, 0x084b, 0x084c, 0x084e, 0x084f,   /* a_sch[0,1,3,4] on R pages 0 and 1 */
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

/* PONT_PC_COUNT=pc1,pc2,... : exact hit counters, published with the profile.
 * Forces single-stepping, about 3x slower; diagnostic use only. */
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
/* PONT_DUMP_DATA=addr:n,addr:n : DARAM words printed along with the profile. */
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
        /* Budget accounting uses the count c54x_run returns, not the insn_count
         * delta: RPT iterations do not bump insn_count, so a step or a whole
         * bucket landing inside a several-hundred-iteration "rpt *(lk); nop"
         * reads as a stalled DSP and aborts the frame mid-ISR. */
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
    /* the 12 hottest buckets, in decreasing order */
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

/* One TDMA frame, in the order calypso_tdma_tick() uses: DMA tick, then boot
 * (run to the first IDLE) while init is pending, then the TPU-frame interrupt
 * if IMR arms it followed by one run budget. Returns the PONT_DONE flags and
 * the executed instruction count in *insns. */
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
        /* Wake on a level-held or pending interrupt. The core only vectors a
         * pending interrupt (IFR&IMR, INTM=0) on the next instruction
         * (c54x_irq_level_check, CALYPSO_C54X_IRQ_LEVEL=1), and an idle DSP
         * executes none; on silicon the interrupt itself wakes it (SPRU131).
         * INT10n (RHEA DMA completion, bit 14) is a level line: it stays
         * asserted while a channel holds IRQ_STATE, so present it on wake. */
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

/* Synthetic I/Q injection.
 *
 * On silicon the DSP gets 148 complex int16 samples per burst, one per symbol,
 * written to DARAM 0x2a00 (296 words) by the BSP DMA and followed by the RX
 * interrupt; calypso_bsp_rx_burst() reproduces that sequence. An FCCH burst is
 * GMSK over 148 zero bits, i.e. a tone at +1625/24 kHz, which is +pi/2 of
 * phase per symbol at 270.833 ksym/s (GSM 45.004). */
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

/* Inject one burst (synthetic cell or plain signal) into the RIF/BSP path. */
static void injecter_burst(C54xState *dsp, const char *iq_mode, int amp, uint32_t fn,
                           unsigned long *injectes)
{
        int16_t iq[2 * 256];
        int n_iq = 2 * IQ_N;
        if (!strncmp(iq_mode, "cell", 4)) {
            /* cell[:bsic[:offset[:margin]]] : full cell, FCCH/SCH/dummy bursts */
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


/* Register of active hacks. Every departure from native behaviour (canned TOA,
 * forced AFC rotation, direct feed, locks, ...) is listed by hacks_actifs() and
 * printed on every milestone line: a milestone reached with a non-empty list is
 * not a native milestone. The stimulus (synthetic cell) is reported separately,
 * being test input rather than a crutch. */

/* SB encoder: exact inverse of l1s_decode_sb (prim_fbsb.c), identical to
 * shunt_encode_sb in qemu-calypso. Maps {bsic, T1, T2, T3} to the 25-bit sb
 * word; the firmware reads sb = a_sch[3] | a_sch[4]<<16, then bsic = (sb>>2)
 * & 0x3f, and so on (GSM 45.002 SCH layout). */
static uint32_t pont_encode_sb(uint8_t bsic, uint16_t t1, uint8_t t2, uint8_t t3)
{
    uint8_t t3p = (t3 == 0) ? 0 : (uint8_t)((t3 - 1) / 10);
    uint32_t sb = 0;
    sb |= ((uint32_t)(bsic & 0x3f)) << 2;
    sb |= ((uint32_t)(t1 & 0x001)) << 23;
    sb |= ((uint32_t)(t1 & 0x1fe)) << 7;
    sb |= ((uint32_t)(t1 & 0x600)) >> 9;
    sb |= ((uint32_t)(t2 & 0x1f))  << 18;
    sb |= ((uint32_t)(t3p & 1))    << 24;
    sb |= ((uint32_t)(t3p & 6))    << 15;
    return sb;
}

static const char *hacks_actifs(void)
{
    static char buf[512]; buf[0] = 0;
    struct { const char *env, *tag; int mode; } t[] = {   /* mode 0: set and non-empty; 1: =="1"; 2: =="0"; 3: integer != 0; 4: integer >= 0 */
        {"PONT_CAN_TOA","CAN_TOA",4}, {"PONT_CAN_SB_TOA","CAN_SB_TOA",4}, {"PONT_CAN_SB","CAN_SB_FULL",0},
        {"CALYPSO_TWL3025_AFC_HZ","AFC_HZ",3}, {"CALYPSO_TWL3025_AFC","AFC_OFF",2},
        {"CALYPSO_TWL3025_AFC_SIGN_OLD","AFC_SIGN_OLD",0},
        {"CALYPSO_BSP_VEC30","VEC30",1}, {"CALYPSO_BSP_DIRECT_FEED","DIRECT_FEED",1},
        {"CALYPSO_BSP_RX_LEAD","RX_LEAD",3}, {"CALYPSO_BSP_TPU_TRACK","TPU_TRACK",1},
        {"CALYPSO_BSP_TOA_LOCK","TOA_LOCK",1}, {"CALYPSO_BSP_STREAM","STREAM",1},
        {"CALYPSO_PONT_LOCKSTEP","LOCKSTEP",1}, {"CALYPSO_BSP_IQ_PASSTHROUGH","IQ_SYNTH",2},
        {"CALYPSO_RHEA_DMA_XFER","RHEA_DMA",1}, {"CALYPSO_BSP_RX_VEC","RX_VEC",0},
        {"CELLULE_SCH_ONLY","SCH_ONLY",1}, {"CELLULE_SCH_AMPDIV","SCH_AMPDIV",3},
        {"CALYPSO_FIXES","FIXES",0},
    };
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
        const char *v = getenv(t[i].env); if (!v || !*v) continue;
        int on = 0; long n = atol(v);
        switch (t[i].mode) { case 0: on = 1; break; case 1: on = (*v=='1'); break; case 2: on = (*v=='0'); break;
                             case 3: on = (n != 0); break; case 4: on = (n >= 0); break; }
        if (!on) continue;
        size_t l = strlen(buf);
        if (t[i].mode == 3 || t[i].mode == 4) snprintf(buf + l, sizeof buf - l, "%s%s=%ld", l ? "," : "", t[i].tag, n);
        else snprintf(buf + l, sizeof buf - l, "%s%s", l ? "," : "", t[i].tag);
    }
    /* a decoder fix turned off is a departure from native too */
    static const char *fx[] = {"NORM_SD","F7_DELAYED","MPY_MAC_LK","MACP_MACD","PAR_ST_DSTBAR","STL_STH_SHFT","XCCD","ADDSUB_XSHFT","FIRS_RPT","RPT_COUNT"};
    for (unsigned i = 0; i < sizeof fx / sizeof fx[0]; i++) {
        char e[64]; snprintf(e, sizeof e, "CALYPSO_FIX_%s", fx[i]); const char *v = getenv(e);
        if (v && *v == '0') { size_t l = strlen(buf); snprintf(buf + l, sizeof buf - l, "%sFIX_%s=0", l ? "," : "", fx[i]); }
    }
    return buf[0] ? buf : "aucun";
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
            /* Same as qosmo-dsp: c54x_reset, running, boot driven by the ticks, d_dsp_page=0 */
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
            calypso_bsp_set_tpu_offset((int)m.c);   /* firmware RX window */
            /* AFC relay, closing the loop. The ARM writes d_afc (word 15 of the W
             * page) into the shared API RAM; on silicon the DSP serialises it to
             * the TWL3025 over the TSP. Without this relay the sample rotation
             * stays frozen and the frequency error never converges below the SB
             * threshold. m.b carries d_dsp_page, bit 0 selecting the W page. */
            { unsigned wp = m.b & 1u;
              int16_t dac = (int16_t)api_ram[(wp ? 0x14u : 0x00u) + 15u];
              static int16_t prev; static int first = 1;
              if (first || dac != prev) { calypso_twl3025_set_afc_dac(dac); prev = dac; first = 0; } }
            { static unsigned _to=0; if (getenv("PONT_TPU_DEBUG") && (_to<5 || _to%2000==0)) printf("  [tpu] fn=%u tpu_offset=%u\n", m.a, m.c); _to++; }
            trace_armer();
            if (g_trace_reste > 0 && g_trace_f && !g_trace_ouverte && !g_trace_pc_hi) {
                /* wait for the first FB task posted by the ARM (W page 0 or 1) */
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
            /* PONT_RX_APRES (default 1): deliver the burst AFTER the frame
             * interrupt, as on silicon, where the TPU programs the RX window
             * within the frame, the DSP arms DMA2 in the frame ISR, and the
             * samples land afterwards. Injecting before the interrupt instead
             * leaves DMA2 with ENABLE=0 when the RIF issues its RX request, and
             * the SB job then decodes the previous tick's window. */
            static int rx_apres = -1;
            if (rx_apres < 0) { const char *e = getenv("PONT_RX_APRES"); rx_apres = (e && *e == '0') ? 0 : 1; }
            if (!rx_apres && injecter && init_done) injecter_burst(dsp, iq_mode, amp, m.a, &injectes);
            /* Real chain: drain UDP socket 6702 (bursts from the bridge/BTS) and
             * hand them to the DSP. This is the only source when synthetic
             * injection is off, and a no-op when the socket is empty. */
            if (init_done) calypso_bsp_service(m.a);
            uint32_t ninsn = 0;
            bool init_avant = init_done;
            uint32_t drapeaux = jouer_trame(dsp, insns, &init_done, &ninsn);
            if (rx_apres && injecter && init_done && dsp->running) {
                injecter_burst(dsp, iq_mode, amp, m.a, &injectes);
                /* DMA completion inside the same frame: wake on the held INT10n
                 * line, then let the DSP run the completion ISR and background
                 * work through to IDLE. */
                uint32_t avant = dsp->insn_count;
                if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) &&
                    !(dsp->ifr & (1u << 14)))
                    c54x_interrupt_ex(dsp, 30, 14);
                if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
                if (!dsp->idle) c54x_run_profile(dsp, (int)insns);
                ninsn += dsp->insn_count - avant;
                drapeaux = (drapeaux & ~PONT_DONE_IDLE) | (dsp->idle ? PONT_DONE_IDLE : 0);
            }
            /* Canned results: scaffolding to prove the pipeline through to the
             * LU, not an end state. PONT_CAN_TOA=23 forces the reported TOA
             * (a_sync_demod[D_TOA]) to the on-time value the firmware expects
             * (prim_fbsb.c subtracts 23). Drop it once the correlator's native
             * TOA is correct. -1, the default, disables canning. */
            {
                static int can_toa = -2;
                if (can_toa == -2) { const char *e = getenv("PONT_CAN_TOA"); can_toa = (e && *e) ? atoi(e) : -1; }
                if (can_toa >= 0 && *d_fb_det)
                    api_ram[(API_NDB + NDB_A_SYNC_DEMOD) / 2 + D_TOA] = (uint16_t)can_toa;
                /* SB: the firmware reads a_serv_demod[D_TOA] from the read page
                 * and expects about 4. PONT_CAN_SB_TOA=4 pins it on both R
                 * pages, which frames the SCH correctly. */
                static int can_sb = -2;
                if (can_sb == -2) { const char *e = getenv("PONT_CAN_SB_TOA"); can_sb = (e && *e) ? atoi(e) : -1; }
                if (can_sb >= 0) {
                    api_ram[(API_R_PAGE(0) + RP_A_SERV_DEMOD) / 2 + D_TOA] = (uint16_t)can_sb;
                    api_ram[(API_R_PAGE(1) + RP_A_SERV_DEMOD) / 2 + D_TOA] = (uint16_t)can_sb;
                }
                /* Full SB canning (PONT_CAN_SB=<bsic>): write the SB result
                 * (a_sch) with CRC OK, the BSIC and the real frame number, the
                 * way qemu-calypso's shunt_encode_sb does. The frame number comes
                 * from the last delivered burst (real BTS, via
                 * calypso_bsp_get_last_fn) or from m.a. Applies only when the ARM
                 * asks for SB (d_task_md=6 on a W page) and on an SCH frame,
                 * fn%51 in {1,11,21,31,41}. */
                static int can_sb_full = -2, can_sb_bsic = 7;
                if (can_sb_full == -2) { const char *e = getenv("PONT_CAN_SB"); can_sb_full = (e && *e) ? 1 : 0; if (e && *e) can_sb_bsic = atoi(e) & 0x3f; }
                if (can_sb_full) {
                    int md0 = api_ram[4] & 0xff, md1 = api_ram[0x18] & 0xff;   /* d_task_md on W pages 0 and 1 */
                    if (md0 == 6 || md1 == 6) {
                        uint32_t bfn = calypso_bsp_get_last_fn(); if (bfn == 0) bfn = m.a;
                        uint32_t p51 = bfn % 51;
                        if (!(p51 % 10 == 1 && p51 <= 41)) bfn += (51 + 1 - (int)p51) % 51;  /* snap to an SCH frame */
                        uint32_t sb = pont_encode_sb((uint8_t)can_sb_bsic, bfn / 1326, bfn % 26, bfn % 51);
                        for (int pg = 0; pg < 2; pg++) {
                            uint16_t *a = &api_ram[(API_R_PAGE(pg) + RP_A_SCH) / 2];
                            a[0] = 0x8000;                 /* B_SCH_CRC clear = CRC OK */
                            a[1] = 0x2034;                 /* echo value, as the DSP writes it */
                            a[3] = (uint16_t)(sb & 0xffff);
                            a[4] = (uint16_t)(sb >> 16);
                        }
                        static int nlog = 0;
                        if (nlog < 8) { printf("  [can-sb] fn=%u -> sb=0x%08x BSIC=%d (T1=%u T2=%u T3=%u)  hacks=%s\n",
                                                bfn, sb, can_sb_bsic, bfn/1326, bfn%26, bfn%51, hacks_actifs()); nlog++; }
                    }
                }
            }
            if (*d_fb_det) calypso_bsp_toa_feedback((int)(int16_t)a_sync[0]);  /* native TOA tracking loop */
            trames++;
            insns_total += ninsn;
            if (drapeaux & PONT_DONE_API_IRQ) irqs++;
            envoyer(fd, PONT_DONE, drapeaux, ninsn);
            if (!init_avant && init_done) {
                printf("pont : DSP boote (premier IDLE) fn=%u insn=%u\n", m.a, dsp->insn_count);
            }
            /* frame at which the ARM posts the FB/SB task (W0 word 4, W1 word 0x18) */
            { static int prev5 = -1, prev6 = -1, ncmd = 0;
              int md0 = api_ram[4] & 0xff, md1 = api_ram[0x18] & 0xff;
              int has5 = (md0 == 5 || md1 == 5), has6 = (md0 == 6 || md1 == 6);
              if (has5 && prev5 != 1 && ncmd < 24) { printf("  [cmd] fn=%u tache FB postee par l'ARM\n", m.a); ncmd++; }
              if (has6 && prev6 != 1 && ncmd < 24) { printf("  [cmd] fn=%u tache SB postee par l'ARM\n", m.a); ncmd++; }
              prev5 = has5; prev6 = has6; }
            { static int fb_prev = 0; if (*d_fb_det && !fb_prev) printf("  [jalon] fn=%u d_fb_det=1  hacks=%s\n", m.a, hacks_actifs()); fb_prev = *d_fb_det != 0; }
            { static int crc_prev = 1; int crc_ok = (a_sch0[0] & 0x8100) == 0x8000;
              if (crc_ok && !crc_prev) printf("  [jalon] fn=%u SB CRC OK a_sch=%04x %04x %04x %04x  hacks=%s\n", m.a, a_sch0[0], a_sch0[1], a_sch0[3], a_sch0[4], hacks_actifs());
              crc_prev = crc_ok; }
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
                if ((trames % (217*8)) == 0) printf("  hacks actifs : %s ; stimulus : %s\n", hacks_actifs(), iq_mode ? iq_mode : "none");
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
