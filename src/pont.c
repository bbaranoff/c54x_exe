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
#include "calypso_rif.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_dsp_pont.h"
#include "pont.h"
#include "calypso_gmsk.h"
#include "cellule.h"
#include "hw/arm/calypso/calypso_debug.h"

extern int g_toa_grille, g_toa_valeur;   /* c54x_mem.c : provenance du TOA */
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
    const char *e = calypso_getenv("PONT_TRACE_FB");
    const char *r = calypso_getenv("PONT_TRACE_PC");
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
/* Cells watched during the trace. Rebuilt [2026-09-19]: the previous list had
 * accreted 13 addresses (0x3fc1 0x3fde 0x3fd3 0x0c3f 0x0906 0x058a 0x0e60
 * 0x435b 0x0e4f 0x2a01 ...) that appear NOWHERE else in the tree and whose
 * introducing commit (812c343) says only "commit". An address whose meaning
 * cannot be restated is not evidence: when it moves, nothing follows. Every
 * entry below carries where its meaning comes from.
 *
 * API window cells are derived from calypso_api.h rather than written raw, so
 * they cannot drift from the map: API base = C54X_API_BASE = 0x0800 words, and
 * the API_* offsets are in BYTES (hence the /2). */
#define CEL_W(p, off)   (uint16_t)(C54X_API_BASE + (API_W_PAGE(p) + (off)) / 2)
#define CEL_R(p, off)   (uint16_t)(C54X_API_BASE + (API_R_PAGE(p) + (off)) / 2)
#define CEL_NDB(off)    (uint16_t)(C54X_API_BASE + (API_NDB + (off)) / 2)

static const uint16_t g_cellules[] = {
    /* --- ARM -> DSP pages (calypso_api.h) ------------------------------- */
    CEL_W(0, WP_D_TASK_MD), CEL_W(1, WP_D_TASK_MD),   /* 0x0804/0x0818 the posted task */
    CEL_W(0, WP_D_TASK_D),  CEL_W(1, WP_D_TASK_D),    /* 0x0800/0x0814 */
    CEL_W(0, WP_D_FN),      CEL_W(1, WP_D_FN),        /* 0x0808/0x081c */

    /* --- NDB (calypso_api.h + calypso_fbsb.h) --------------------------- */
    CEL_NDB(NDB_D_DSP_PAGE),                          /* 0x08d4 page toggle */
    CEL_NDB(NDB_D_FB_DET),                            /* 0x08f8 FB found flag */
    CEL_NDB(NDB_A_SYNC_DEMOD + 2 * D_TOA),            /* 0x08fa */
    CEL_NDB(NDB_A_SYNC_DEMOD + 2 * D_PM),             /* 0x08fb */
    CEL_NDB(NDB_A_SYNC_DEMOD + 2 * D_ANGLE),          /* 0x08fc  <- was MISSING */
    CEL_NDB(NDB_A_SYNC_DEMOD + 2 * D_SNR),            /* 0x08fd  <- was MISSING */

    /* --- DSP -> ARM pages: a_sch[0,1,3,4], BOTH pages -------------------- */
    CEL_R(0, RP_A_SCH + 0), CEL_R(0, RP_A_SCH + 2),
    CEL_R(0, RP_A_SCH + 6), CEL_R(0, RP_A_SCH + 8),   /* 0x0837 38 3a 3b */
    CEL_R(1, RP_A_SCH + 0), CEL_R(1, RP_A_SCH + 2),
    CEL_R(1, RP_A_SCH + 6), CEL_R(1, RP_A_SCH + 8),   /* 0x084b 4c 4e 4f */

    /* --- FB correlator input, read out of the mask ROM [2026-09-19] ------ */
    0x3fb5,   /* pointer to the input buffer; PROM0 0xb2c4 ST #0x0cce,*(0x3fb5)
               *                              and 0xb2c9 ST #0x0d2e,*(0x3fb5) */
    0x0cce,   /* buffer A, same two instructions; also the AAD the DMA uses */
    0x0d2e,   /* buffer B, idem */
    0x0e4e,   /* second DARAM target seen in the rhea-dma RX transfers */
    0x2a00,   /* CALYPSO_BSP_DARAM_ADDR env default, before AAD_FOLLOW */

    /* --- cells whose meaning is stated elsewhere in the tree ------------- */
    0x3f92,   /* calypso_dma.h:9   source of d_error_status (0x08d5) */
    0x43d8,   /* calypso_bsp.c:1567  poked per burst on the FB/SB mission */
    0x43d5,   /* c54x_mem.c:2229   PROM 0xb4be stm #0x43d5 ; reada *AR1+ */
    0x098c,   /* calypso_mailbox.c:46  mailbox poll (0xde86 ld *(0x098c)) */
    0x435e,   /* calypso_c54x.c:4574  bit13 = DMA config lock */
    0x4368,   /* c54x_mem.c:1835   DISPATCH-CELL-RESEED */
    0x3fb0,   /* c54x_probes.c:505  BSP read window 0x3fb0..0x3fbf */
    0x0000 };
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
    const char *e = calypso_getenv("PONT_PC_COUNT");
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
    const char *e = calypso_getenv("PONT_DUMP_DATA");
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
        if (minfn < 0) { const char *e = calypso_getenv("PONT_TRACE_MINFN"); minfn = (e && *e) ? atol(e) : 0; }
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
/* [2026-09-20] MID-FRAME INJECTION (PONT_RX_MODE=milieu, the default). The ROM
 * arms DMA2 in its frame ISR and the receiver only keeps samples while a
 * window is open (calypso_rif: no window, no sample). Delivering the burst
 * AFTER the frame's run (the old PONT_RX_APRES=1) found the channel closed on
 * most frames: measured 13 transfers on one frame in six, the FB block counter
 * starved, TOA = 1251 for an FCCH four frames away. The replay bench delivers
 * after a short slice of the frame, once the ISR has armed; the bridge now does
 * the same: budget/8, inject (synthetic cell and UDP bursts), then the rest. */
static bool g_tick_irq_trame = true;   /* TICK.b bit 16 : l'ARM a arme l'interruption trame du DSP */
static struct { bool actif; const char *iq_mode; int amp; uint32_t fn; unsigned long *injectes; bool udp; char dernier_type; int dernier_n_iq; } g_inj;
static void injecter_burst(C54xState *dsp, const char *iq_mode, int amp, uint32_t fn, unsigned long *injectes);

uint16_t prog_fetch(C54xState *s, uint16_t pc);
static unsigned g_flags_entree; static uint16_t g_data_avant_b[C54X_DATA_SIZE];
static uint16_t g_snap_2be2[148]; static int g_snap_ok;
static uint16_t g_snap_2a00[456]; static int g_snap_2a00_ok;
#define s_or_dsp_st0(d) ((d)->st0)
static int16_t g_dernier_iq[2 * 256]; static int g_dernier_n_iq;
static uint16_t g_daram_apres_dma[384]; static uint16_t g_daram_aad;
/* phase : 0 = whole frame ; 1 = frame ISR only, up to the arming of the RX
 * window (then DONE|PHASE_A and wait for PONT_GO) ; 2 = burst delivery and
 * the rest of the frame. See CALYPSO_PONT_TICK_DEUX_PHASES. */
static uint32_t jouer_trame(C54xState *dsp, long budget, bool *init_done, uint32_t *insns, int phase)
{
    uint32_t drapeaux = 0;
    uint32_t avant = dsp->insn_count;
    static int fait_a;          /* phase A instructions, charged to phase B's budget */
    static bool etait_idle_a;   /* idle state at the frame start, for PONT_DONE_API_IRQ */

    if (phase == 2) goto phase_b;
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
        etait_idle_a = dsp->idle;
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
        /* [2026-09-20] The DSP frame interrupt is TPU_CTRL_DSP_EN, a bit the
         * firmware sets in dsp_end_scenario() on EVERY scenario and that the
         * TPU consumes: the ROM gets a frame interrupt only on frames where
         * the ARM handed it a page. Raising it on every tick made the ROM
         * re-read the same page for 10+ frames (the firmware only flips the
         * write page on frames with a DSP item and the ROM never clears
         * d_task_md): the FB task restarted at each FCCH and the SB job never
         * ran (0xaba4 dispatched every frame, no 764-byte window armed).
         * QEMU now says in TICK.b bit 16 whether the ARM armed it.
         * PONT_IRQ_TRAME=1 restores the interrupt on every tick (A/B). */
        static int irq_chaque = -1;
        if (irq_chaque < 0) { const char *e = calypso_getenv("PONT_IRQ_TRAME"); irq_chaque = (e && *e == '1') ? 1 : 0; }
        if ((dsp->imr & (1u << C54X_IT_TPU_FRAME_BIT)) && (irq_chaque || g_tick_irq_trame)) {
            c54x_interrupt_ex(dsp, C54X_IT_TPU_FRAME_VEC, C54X_IT_TPU_FRAME_BIT);
        }
        if (calypso_getenv("PONT_IRQ_DEBUG") && g_c54x_exe_fn > 5000 && g_c54x_exe_fn < 5012)
            printf("  [irq] fn=%u APRES vec28 : idle=%d pc=%04x INTM=%d IMR=%04x IFR=%04x\n",
                   g_c54x_exe_fn, dsp->idle, dsp->pc & 0xffff, !!(dsp->st1 & 0x800), dsp->imr, dsp->ifr);
        if (g_inj.actif || phase == 1) {
            int fait = 0;
            if (!dsp->idle) fait = c54x_run_profile(dsp, (int)budget / 8);   /* the ISR arms DMA2 */
            /* [2026-09-20] Deliver only once the receive window is armed. On a
             * frame where the ROM first finishes the previous burst's demod
             * (20-30k instructions of Viterbi) before it programs the one-shot
             * NB window, the fixed budget/8 slice delivered the frame BEFORE the
             * arming, the RIF discarded it (no window) and the result page stayed
             * empty: measured on the BTS bench, one normal-burst result in four
             * missing (b0 absent in 23 of 97 BCCH blocks), read by the firmware
             * as EMPTY / BURST ID n!=m. Keep running, in slices, until DMA2 is
             * armed or the DSP idles, half the budget at most. */
            while (!dsp->idle && !calypso_rhea_dma_rx_armed() && fait < (int)budget / 2)
                fait += c54x_run_profile(dsp, 256);
            fait_a = fait;
            if (phase == 1) {
                /* ISR played, R page written, window armed: the ARM may run
                 * l1_sync now. The burst comes with PONT_GO. */
                if (dsp->idle)    drapeaux |= PONT_DONE_IDLE;
                if (dsp->running) drapeaux |= PONT_DONE_RUNNING;
                if (*init_done)   drapeaux |= PONT_DONE_INIT;
                *insns = dsp->insn_count - avant;
                return drapeaux | PONT_DONE_PHASE_A;
            }
phase_b:
            fait = fait_a;
            /* PONT_NB_DEBUG: flags at the burst's entry, and a snapshot of the
             * data memory to list what the demod writes (regions), see
             * sonde_bits(). */
            { static int on = -1; if (on < 0) on = calypso_getenv("PONT_NB_DEBUG") ? 1 : 0;
              if (on) { g_flags_entree = ((s_or_dsp_st0(dsp) & ST0_OVA) ? 1 : 0) | ((dsp->st0 & ST0_OVB) ? 2 : 0) | ((dsp->st0 & ST0_C) ? 4 : 0) | ((dsp->st0 & ST0_TC) ? 8 : 0) | ((dsp->st1 & ST1_OVM) ? 16 : 0) | ((dsp->st1 & ST1_FRCT) ? 32 : 0) | ((dsp->st1 & ST1_SXM) ? 64 : 0);
                        memcpy(g_data_avant_b, dsp->data, sizeof g_data_avant_b); } }
            if (g_inj.udp) calypso_bsp_service(g_inj.fn);
            if (g_inj.iq_mode) injecter_burst(dsp, g_inj.iq_mode, g_inj.amp, g_inj.fn, g_inj.injectes);
            { uint16_t aad = calypso_rhea_dma_get_daram();
              memcpy(g_daram_apres_dma, &dsp->data[aad], sizeof g_daram_apres_dma); g_daram_aad = aad; }
            if (dsp->idle && calypso_rhea_dma_irq_level() && (dsp->imr & (1u << 14)) && !(dsp->ifr & (1u << 14)))
                c54x_interrupt_ex(dsp, 30, 14);
            if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
            /* PONT_NB_HIST=<dir>: opcode histogram of phase B (the burst's demod)
             * for the first 12 normal bursts, one file per frame, to name the
             * instructions the NB path leans on and cross them with the ISA
             * scorecard (isa_test). Single-stepped: prog_fetch() before each. */
            static const char *hist_dir = NULL; static int hist_init = 0; static unsigned hist_n;
            if (!hist_init) { hist_init = 1; hist_dir = calypso_getenv("PONT_NB_HIST"); }
            if (hist_dir && hist_n < 12 && g_inj.dernier_type == 'B' && !dsp->idle && dsp->api_ram &&
                (dsp->api_ram[API_R_PAGE(0) / 2] == 24 || dsp->api_ram[API_R_PAGE(1) / 2] == 24)) {
                static unsigned hist[65536]; memset(hist, 0, sizeof hist);
                int reste = (int)budget - fait, k = 0;
                /* data watch: every write into the demod's working cells, with
                 * the PC of the instruction (taps 0x2cbb.., tracker 0x5aaa..,
                 * result cells 0x3fa4.., reference 0x2b28..) */
                static const struct { uint16_t lo, hi; } W[] = { {0x2cbb, 0x2d04}, {0x5aaa, 0x5ac8}, {0x3fa4, 0x3fa8}, {0x2b28, 0x2b58}, {0x2f00, 0x2f2c}, {0x2a00, 0x2bc8} };
                #define NW 6
                static uint16_t prev[0x600]; int nw = 0;
                for (unsigned r = 0; r < NW; r++) for (unsigned a = W[r].lo; a < W[r].hi; a++) prev[nw++] = dsp->data[a];
                char nomw[256]; snprintf(nomw, sizeof nomw, "%s/watch_%u.txt", hist_dir, g_inj.fn);
                FILE *fw = fopen(nomw, "w");
                char nomt[256]; snprintf(nomt, sizeof nomt, "%s/trace_%u.txt", hist_dir, g_inj.fn);
                FILE *ft = fopen(nomt, "w");
                while (!dsp->idle && k < reste) {
                    uint16_t w = prog_fetch(dsp, (uint16_t)dsp->pc);
                    uint16_t pc0 = (uint16_t)dsp->pc; uint16_t ar2 = dsp->ar[2], ar3 = dsp->ar[3];
                    hist[w]++;
                    if (k == 12800) { memcpy(g_snap_2be2, &dsp->data[0x2be2], sizeof g_snap_2be2); g_snap_ok = 1; }
                    if (pc0 == 0x9a78 && !g_snap_2a00_ok) { memcpy(g_snap_2a00, &dsp->data[0x2a00], sizeof g_snap_2a00); g_snap_2a00_ok = 1;
                        char nm[256]; snprintf(nm, sizeof nm, "%s/mem_%u_9a78.bin", hist_dir, g_inj.fn); FILE *fm = fopen(nm, "wb"); if (fm) { fwrite(dsp->data, 2, C54X_DATA_SIZE, fm); fclose(fm); } }
                    if (ft) fprintf(ft, "%04x %04x %010llx %010llx %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n", pc0, w,
                                    (unsigned long long)(dsp->a & 0xFFFFFFFFFFULL), (unsigned long long)(dsp->b & 0xFFFFFFFFFFULL),
                                    dsp->t, dsp->st0, dsp->st1, dsp->ar[1], ar2, ar3, dsp->ar[4], dsp->ar[5], dsp->ar[0],
                                    dsp->data[ar2], dsp->data[ar3], dsp->data[dsp->ar[4]], dsp->data[dsp->ar[5]], dsp->ar[6]);
                    c54x_run(dsp, 1); k++;
                    if (fw) { int i = 0;
                        for (unsigned r = 0; r < NW; r++) for (unsigned a = W[r].lo; a < W[r].hi; a++, i++)
                            if (dsp->data[a] != prev[i]) { fprintf(fw, "%d pc=%04x op=%04x %04x: %04x -> %04x (%d) AR2=%04x AR3=%04x\n", k, pc0, w, a, prev[i], dsp->data[a], (int16_t)dsp->data[a], ar2, ar3); prev[i] = dsp->data[a]; } }
                }
                if (fw) fclose(fw);
                if (ft) fclose(ft);
                { char nm[256]; snprintf(nm, sizeof nm, "%s/mem_%u_fin.bin", hist_dir, g_inj.fn); FILE *fm = fopen(nm, "wb"); if (fm) { fwrite(dsp->data, 2, C54X_DATA_SIZE, fm); fclose(fm); } }
                char nom[256]; snprintf(nom, sizeof nom, "%s/hist_%u.txt", hist_dir, g_inj.fn);
                FILE *f = fopen(nom, "w");
                if (f) { for (unsigned w = 0; w < 65536; w++) if (hist[w]) fprintf(f, "%04x %u\n", w, hist[w]); fclose(f); }
                hist_n++;
            }
            if (!dsp->idle) c54x_run_profile(dsp, (int)budget - fait);
        } else if (!dsp->idle) {
            c54x_run_profile(dsp, (int)budget);
        }
        if (!etait_idle_a && dsp->idle) {
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

/* Recorded-cell replay, paced by the DSP frame clock.
 *
 * /opt/GSM/cellule_reelle.bin holds the TS0 of 311 CONSECUTIVE frames captured
 * off the air at 4 samples/symbol, each tagged with its real frame number, so
 * the 51-multiframe is intact: FCCH at fn%51 in {0,10,20,30,40}, SCH at
 * {1,11,21,31,41}.
 *
 * Why it is fed from here and not over UDP: the source and the DSP have
 * independent clocks. Measured on this bench, the DSP runs between 72 and 160
 * frames/s depending on load while rejouer_cellule.py streams at a rate of its
 * own, so the DSP swallowed about four cell frames per frame of its own. The
 * FB task survives that — it only ever correlates ONE window — but the SB task
 * needs the burst of the frame IMMEDIATELY AFTER the FCCH, and that frame was
 * never the SCH. Pulled from here, one burst per frame, the multiframe reaches
 * the DSP intact whatever the emulated clock does. */
typedef struct { uint32_t fn; int16_t *iq; int fcch; } ReelBurst;
static ReelBurst *g_reel;
static unsigned   g_reel_n, g_reel_util;
static int        g_reel_nsym;
static unsigned   g_reel_base;

/* Decimate a stored burst to the 1 sample/symbol the correlator works at. The
 * same decimation the UDP path applies through CALYPSO_BSP_IQ_DECIM, and the
 * same one reelle_injecter delivers: what is measured here is what the DSP
 * gets. Returns the number of int16 written. */
static int reelle_decimer(const ReelBurst *b, int16_t *iq, int max_i16)
{
    int decim = g_reel_nsym / 148;
    if (decim < 1) decim = 1;
    int n = 0;
    for (int k = 0; k * decim < g_reel_nsym && n <= max_i16 - 2; k++) {
        iq[n++] = b->iq[2 * (k * decim)];
        iq[n++] = b->iq[2 * (k * decim) + 1];
    }
    return n;
}

/* Is this burst an FCCH? At 1 sample/symbol an all-zeros GMSK burst is a pure
 * tone rotating by exactly +pi/2 per sample — the criterion calypso_bsp.c's
 * FCCH-PROBE already uses (coh ~ 1, dphi ~ +1.571). Nothing else on TS0 comes
 * close, so this labels the multiframe without decoding anything. */
static int reelle_est_fcch(const ReelBurst *b)
{
    int16_t iq[2 * 256];
    int n = reelle_decimer(b, iq, (int)(sizeof(iq) / sizeof(iq[0])));
    int ns = n / 2;
    if (ns < 32) return 0;
    double accr = 0, acci = 0, den = 0;
    for (int k = 1; k < ns; k++) {
        double i0 = iq[2*(k-1)], q0 = iq[2*(k-1)+1];
        double i1 = iq[2*k],     q1 = iq[2*k+1];
        accr += i1*i0 + q1*q0;
        acci += q1*i0 - i1*q0;
        den  += sqrt((i0*i0 + q0*q0) * (i1*i1 + q1*q1));
    }
    if (den <= 0) return 0;
    double coh  = sqrt(accr*accr + acci*acci) / den;
    double dphi = atan2(acci, accr);
    return coh > 0.90 && fabs(dphi - M_PI_2) < 0.30;
}

/* Where does the recording sit in the 51-multiframe?
 *
 * The frame numbers stored in the file cannot be trusted for this: measured on
 * cellule_reelle.bin, the FCCH bursts carry tags whose residues are
 * {0,10,20,31,41} where GSM 05.02 puts the FCCH at fn%51 in {0,10,20,30,40} —
 * a constant offset of 31. So the phase is taken from the SIGNAL instead: the
 * FCCH bursts are located by reelle_est_fcch(), and the one followed by a gap
 * of 11 frames is the last of its multiframe, i.e. true phase 40 (the sequence
 * of gaps is 10,10,10,10,11). g_reel_base then satisfies
 *
 *     entry_index  ==  (arm_fn - g_reel_base)  (mod 51)
 *
 * which is what reelle_injecter uses to pick a burst BY FRAME NUMBER. */
static void reelle_caler(void)
{
    unsigned fcch[64], nf = 0;
    for (unsigned i = 0; i < g_reel_n && nf < 64; i++)
        if (g_reel[i].fcch) fcch[nf++] = i;

    unsigned i40 = 0; int trouve = 0;
    for (unsigned k = 0; k + 1 < nf; k++)
        if (fcch[k + 1] - fcch[k] == 11) { i40 = fcch[k]; trouve = 1; break; }

    if (!trouve) {
        g_reel_base = 0;
        printf("pont : ATTENTION — phase de multitrame indeterminee (%u FCCH reperees, "
               "aucun ecart de 11) ; calage a 0, la lecture reste cadencee par fn\n", nf);
        return;
    }
    g_reel_base = (40u + 51u - (i40 % 51u)) % 51u;   /* i40 == 40 - base  (mod 51) */
    unsigned ecart_tag = (g_reel[0].fn + 51u - g_reel_base) % 51u;
    printf("pont : %u FCCH reperees dans la capture (ecarts 10/11), phase calee sur le signal : "
           "base=%u ; les tags fn du fichier sont decales de +%u mod 51\n",
           nf, g_reel_base, ecart_tag);
}

/* Recorded-cell replay, addressed by the ARM frame number.
 *
 * /opt/GSM/cellule_reelle.bin holds the TS0 of 311 CONSECUTIVE frames captured
 * off the air at 4 samples/symbol, each tagged with its real frame number.
 *
 * Why it is fed from here and not over UDP: the source and the DSP have
 * independent clocks. Measured on this bench, the DSP runs between 72 and 160
 * frames/s depending on load while rejouer_cellule.py streams at a rate of its
 * own, so the DSP swallowed about four cell frames per frame of its own. The
 * FB task survives that — it only ever correlates ONE window — but the SB task
 * needs the burst of the frame IMMEDIATELY AFTER the FCCH, and that frame was
 * never the SCH.
 *
 * [2026-09-19] Why it is addressed by fn and no longer by a running index:
 * calypso_trx.c:pont_echange() SKIPS a tick whenever the DSP has not returned
 * its DONE — about 3% of frames once the traces are off, 85% with -vvv on a
 * terminal. A running index does not advance on a skipped tick while the ARM
 * clock does, so every skip shifted the recording against the ARM by one frame,
 * FOR GOOD: measured 4202 frames of accumulated drift over a 5-minute run, i.e.
 * a multiframe phase wandering without bound. The ARM then armed its SB one
 * frame after an FCCH and got a burst from somewhere else entirely (11 SB CRC
 * OK in 68000 frames), and the deinterleaver assembled BCCH blocks out of
 * unrelated bursts (205-221 bit errors per block, every block dropped).
 * Indexing by fn makes a skipped tick skip a RECORDING burst too: the phase is
 * held whatever the emulated clock does. */
static void reelle_injecter(C54xState *dsp, uint32_t fn, unsigned long *injectes)
{
    if (!g_reel_util) return;
    unsigned idx = (fn + g_reel_util - (g_reel_base % g_reel_util)) % g_reel_util;
    const ReelBurst *b = &g_reel[idx];

    int16_t iq[2 * 256];
    int n_iq = reelle_decimer(b, iq, (int)(sizeof(iq) / sizeof(iq[0])));

    /* The burst keeps its OWN frame number: it is the provenance of the samples,
     * and calypso_bsp_rx_burst only ever logs it. */
    calypso_bsp_rx_burst(0, b->fn, iq, n_iq);
    (*injectes)++;
}

static int reelle_charger(const char *chemin)
{
    FILE *f = fopen(chemin, "rb");
    if (!f) { fprintf(stderr, "pont : cellule reelle introuvable : %s\n", chemin); return 0; }
    uint32_t hdr[4];
    if (fread(hdr, sizeof(hdr), 1, f) != 1) { fclose(f); return 0; }
    unsigned n = hdr[0], nsym = hdr[1], bsic = hdr[2];
    if (!n || !nsym || nsym > 4096) { fclose(f); return 0; }
    g_reel = calloc(n, sizeof(*g_reel));
    if (!g_reel) { fclose(f); return 0; }
    for (unsigned i = 0; i < n; i++) {
        uint32_t e[2];
        if (fread(e, sizeof(e), 1, f) != 1) { n = i; break; }
        int16_t *iq = malloc((size_t)nsym * 2 * sizeof(int16_t));
        if (!iq || fread(iq, sizeof(int16_t) * 2, nsym, f) != nsym) { free(iq); n = i; break; }
        g_reel[i].fn = e[0];
        g_reel[i].iq = iq;
    }
    fclose(f);
    g_reel_n = n; g_reel_nsym = (int)nsym;

    /* Only a WHOLE number of 51-multiframes may be looped: 311 = 6*51 + 5, so
     * wrapping on all 311 shifted the phase by 5 at every turn. */
    g_reel_util = (n / 51u) * 51u;

    for (unsigned i = 0; i < g_reel_n; i++)
        g_reel[i].fcch = reelle_est_fcch(&g_reel[i]);

    printf("pont : cellule REELLE %s — %u trames consecutives a %d ech/symbole, BSIC=%u, "
           "fn %u..%u, %u trames jouees (%u multitrames de 51, %u ecartees), adressage par fn\n",
           chemin, n, nsym / 148, bsic, n ? g_reel[0].fn : 0, n ? g_reel[n - 1].fn : 0,
           g_reel_util, g_reel_util / 51u, n - g_reel_util);
    reelle_caler();
    return (int)g_reel_util;
}

/* Inject one burst (synthetic cell or plain signal) into the RIF/BSP path. */
static int g_verif_sonde = -1;

static void injecter_burst(C54xState *dsp, const char *iq_mode, int amp, uint32_t fn,
                           unsigned long *injectes)
{
        int16_t iq[2 * 256];
        int n_iq = 2 * IQ_N;
        char t_dbg = '?';
        if (!strncmp(iq_mode, "reelle", 6)) {
            static int charge = 0;
            if (!charge) { const char *p = strchr(iq_mode, ':');
                charge = reelle_charger(p ? p + 1 : "/opt/GSM/cellule_reelle.bin"); if (!charge) charge = -1; }
            if (charge > 0) reelle_injecter(dsp, fn, injectes);
            return;
        }
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
            /* the SCH burst is delivered as the 190-sample window block only when
             * the DSP has its one-shot SB window armed; otherwise it is a frame of
             * the FB stream like any other (same rule as rejouer.c) */
            int marge_eff = calypso_rhea_dma_one_shot() ? marge : 0;
            /* [2026-09-20] BCCH/CCCH normal bursts: framed like the BSP stream
             * assembler (bsp_livrer_trame), 3 silent samples ahead when the
             * one-shot window is the 151-sample NB one (tpu_window.c
             * L1_NB_MARGIN_Q), bare 148 samples in the continuous FB stream. */
            { int nwin = calypso_rhea_dma_one_shot() ? calypso_rhea_dma_get_len_words() / 2 : 0;
              /* PONT_NB_MARGE=<n> overrides the 3-sample head margin of a normal
               * burst; "auto" sweeps 0..7 by 51-multiframe (the [scan] probe
               * prints it), one run to find where the ROM's TSC search lands. */
              static int mnb = -2; static int mauto = 0;
              if (mnb == -2) { const char *e = calypso_getenv("PONT_NB_MARGE"); mnb = 3;
                               if (e && !strcmp(e, "auto")) mauto = 1; else if (e && *e) mnb = atoi(e); }
              int m_nb = mauto ? (int)((fn / 51u) % 8u) : mnb;
              /* CELLULE_TSC=<k>|auto : training sequence written in the bursts
               * (the ARM still tells the ROM tsc = BCC); auto sweeps 0..7 */
              static int tsc = -2; static int tauto = 0;
              if (tsc == -2) { const char *e = calypso_getenv("CELLULE_TSC"); tsc = -1;
                               if (e && !strcmp(e, "auto")) tauto = 1; else if (e && *e) tsc = atoi(e) & 7; }
              cellule_tsc_force = tauto ? (int)((fn / 51u) % 8u) : tsc;
              cellule_marge_nb = (nwin >= 150 && nwin < 190) ? m_nb : (nwin >= 190 ? marge : -1);
              cellule_fenetre_nb = nwin; }
            char t = cellule_burst(fn, (uint8_t)bsic, amp, dec, marge_eff, iq, &n_iq);
            t_dbg = t; g_inj.dernier_type = t; g_inj.dernier_n_iq = n_iq;
            static unsigned nS, nF, nB, nC, tot;
            if (t == 'S') nS++; else if (t == 'F') nF++; else if (t == 'B') nB++; else if (t == 'C') nC++;
            if (++tot % 5000 == 1) printf("pont : bursts injectes FCCH=%u SCH=%u BCCH=%u CCCH=%u (fn=%u)\n", nF, nS, nB, nC, fn);
        } else {
            iq_synthese(iq_mode, amp, fn, iq);
        }
        static int irqdbg = -1; static unsigned irqdbg_n;
        if (irqdbg < 0) irqdbg = calypso_getenv("PONT_IRQ_DEBUG") ? 1 : 0;
        bool dbg = irqdbg && fn > 5000 && irqdbg_n < 12;
        if (dbg) printf("  [irq] fn=%u AVANT rx_burst : idle=%d pc=%04x INTM=%d IMR=%04x IFR=%04x PMST=%04x SP=%04x\n",
                        fn, dsp->idle, dsp->pc & 0xffff, !!(dsp->st1 & 0x800), dsp->imr, dsp->ifr, dsp->pmst, dsp->sp);
        { static int dj = -1; if (dj < 0) dj = calypso_getenv("PONT_DEBUG_INJ") ? 1 : 0;
          int md0 = dsp->api_ram ? (dsp->api_ram[4] & 0xff) : 0, md1 = dsp->api_ram ? (dsp->api_ram[0x18] & 0xff) : 0;
          if (dj && (calypso_rhea_dma_one_shot() || md0 == 6 || md1 == 6 || (fn >= 300 && fn <= 312)))
              printf("  [inj] fn=%u p51=%u AVANT : type=%c n_iq=%d dma armee=%d one_shot=%d task_md=%d/%d rif=%d mots idle=%d pc=%04x"
                     " | W0=%04x %04x %04x %04x %04x ..%04x %04x  W1=%04x %04x %04x %04x %04x ..%04x %04x  NDB page=%04x fb_mode=%04x fb_det=%04x\n",
                     fn, fn % 51u, t_dbg, n_iq, calypso_rhea_dma_rx_armed(), calypso_rhea_dma_one_shot(), md0, md1, calypso_rif_level(), dsp->idle, dsp->pc & 0xffff,
                     dsp->api_ram[0], dsp->api_ram[1], dsp->api_ram[2], dsp->api_ram[3], dsp->api_ram[4], dsp->api_ram[15], dsp->api_ram[16],
                     dsp->api_ram[0x14], dsp->api_ram[0x15], dsp->api_ram[0x16], dsp->api_ram[0x17], dsp->api_ram[0x18], dsp->api_ram[0x14+15], dsp->api_ram[0x14+16],
                     dsp->api_ram[0xd4], dsp->api_ram[0xd4+37], dsp->api_ram[0xd4+36]); }
        memcpy(g_dernier_iq, iq, (size_t)n_iq * sizeof(int16_t)); g_dernier_n_iq = n_iq;
        calypso_bsp_rx_burst(0, fn, iq, n_iq);
        { static int dj2 = -1; if (dj2 < 0) dj2 = calypso_getenv("PONT_DEBUG_INJ") ? 1 : 0;
          if (dj2 && fn >= 300 && fn <= 312)
              printf("  [inj] fn=%u APRES : rif=%d mots idle=%d IFR=%04x\n", fn, calypso_rif_level(), dsp->idle, dsp->ifr); }
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
        {"CALYPSO_BSP_VEC30","VEC30",1},
        {"CALYPSO_BSP_RX_LEAD","RX_LEAD",3}, {"CALYPSO_BSP_TPU_TRACK","TPU_TRACK",1},
        {"CALYPSO_BSP_TOA_LOCK","TOA_LOCK",1}, {"CALYPSO_BSP_STREAM","STREAM",1},
        {"CALYPSO_PONT_LOCKSTEP","LOCKSTEP",1}, {"CALYPSO_BSP_IQ_PASSTHROUGH","IQ_SYNTH",2},
        {"CALYPSO_RHEA_DMA_XFER","RHEA_DMA",1}, {"CALYPSO_BSP_RX_VEC","RX_VEC",0},
        {"CELLULE_SCH_ONLY","SCH_ONLY",1}, {"CELLULE_SCH_AMPDIV","SCH_AMPDIV",3},
        {"CALYPSO_FIXES","FIXES",0},
    };
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
        const char *v = calypso_getenv(t[i].env); if (!v || !*v) continue;
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
        char e[64]; snprintf(e, sizeof e, "CALYPSO_FIX_%s", fx[i]); const char *v = calypso_getenv(e);
        if (v && *v == '0') { size_t l = strlen(buf); snprintf(buf + l, sizeof buf - l, "%sFIX_%s=0", l ? "," : "", fx[i]); }
    }
    return buf[0] ? buf : "aucun";
}

/* PONT_NB_DEBUG=1: pages as seen at one instant of the frame (A = end of the
 * ROM's frame ISR, G = PONT_GO received i.e. after the ARM's l1_sync, B = end
 * of the frame). Who reads which W page and who writes which R page, when. */
static void sonde_pages(const char *quand, uint32_t fn, C54xState *dsp, const uint16_t *api_ram)
{
    static int on = -1; static unsigned n;
    if (on < 0) on = calypso_getenv("PONT_NB_DEBUG") ? 1 : 0;
    if (!on || n >= 240) return;
    const uint16_t *w0 = &api_ram[API_W_PAGE(0) / 2], *w1 = &api_ram[API_W_PAGE(1) / 2];
    const uint16_t *r0 = &api_ram[API_R_PAGE(0) / 2], *r1 = &api_ram[API_R_PAGE(1) / 2];
    if (!(w0[0] == 24 || w1[0] == 24 || r0[0] == 24 || r1[0] == 24)) return;
    n++;
    printf("  [pg%s] fn=%u p51=%u W0=%u/%u W1=%u/%u R0=%u/%u R1=%u/%u dsp_page=%04x idle=%d pc=%04x IFR=%04x IMR=%04x INTM=%d irq_trame=%d dma_armee=%d\n",
           quand, fn, fn % 51u, w0[0], w0[1], w1[0], w1[1], r0[0], r0[1], r1[0], r1[1],
           api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2], dsp->idle, dsp->pc & 0xffff, dsp->ifr, dsp->imr,
           !!(dsp->st1 & 0x800), g_tick_irq_trame, calypso_rhea_dma_rx_armed());
}

/* PONT_NB_DEBUG=1, after a normal burst: look for the demodulated bits in the
 * DSP data memory. Two templates, the 116 data bits (57+hl, hu+57) and the
 * whole 148-bit burst, matched against the sign of each int16 word (soft bits,
 * both polarities) and against hard 0/1 words. Reports the best run. */
static uint32_t g_insn_a, g_insn_b;
static void sonde_bits(uint32_t fn, C54xState *dsp, uint8_t bsic)
{
    static int on = -1; static unsigned n;
    if (on < 0) on = calypso_getenv("PONT_NB_DEBUG") ? 1 : 0;
    if (!on || n >= 40) return;
    uint8_t bits[148];
    if (cellule_bits_attendus(fn, bsic, bits) < 0) return;
    uint8_t t116[116]; memcpy(t116, bits + 3, 58); memcpy(t116 + 58, bits + 87, 58);
    const struct { const uint8_t *t; int len; const char *nom; } tpl[2] = { { t116, 116, "116 data" }, { bits, 148, "148 burst" } };
    n++;
    printf("  [scan] fn=%u p51=%u burst %d :", fn, fn % 51u, (int)((fn % 51u % 10 - 2) & 3));
    for (int k = 0; k < 2; k++) {
        int best[3] = {0,0,0}; unsigned bad[3] = {0,0,0};
        for (unsigned a = 0x60; a + tpl[k].len < C54X_DATA_SIZE; a++) {
            int m0 = 0, m1 = 0, m2 = 0;
            for (int i = 0; i < tpl[k].len; i++) {
                int16_t v = (int16_t)dsp->data[a + i];
                int neg = v < 0, one = (v == 1), zero = (v == 0);
                if ((v < 0) == (tpl[k].t[i] != 0)) m0++;
                if ((v > 0) == (tpl[k].t[i] != 0)) m1++;
                if ((one && tpl[k].t[i]) || (zero && !tpl[k].t[i])) m2++;
            }
            if (m0 > best[0]) { best[0] = m0; bad[0] = a; }
            if (m1 > best[1]) { best[1] = m1; bad[1] = a; }
            if (m2 > best[2]) { best[2] = m2; bad[2] = a; }
        }
        printf("  %s: neg=1 %d/%d @%04x  pos=1 %d/%d @%04x  hard %d/%d @%04x |", tpl[k].nom,
               best[0], tpl[k].len, bad[0], best[1], tpl[k].len, bad[1], best[2], tpl[k].len, bad[2]);
    }
    /* the ROM's burst buffer at 0x2be2 (found by this scan: 146/148 on good
     * bursts): mismatch map, one char per bit, '.' ok, 'x' wrong, '|' at the
     * data/TSC boundaries */
    { char map[160]; int k = 0, err = 0;
      for (int i = 0; i < 148; i++) {
          if (i == 3 || i == 61 || i == 87 || i == 145) map[k++] = '|';
          int16_t v = (int16_t)dsp->data[0x2be2 + i];
          int ok = ((v > 0) == (bits[i] != 0)); if (!ok) err++;
          map[k++] = ok ? '.' : 'x';
      }
      map[k] = 0;
      /* the same buffer against the two previous bursts: a demod deferred to
       * the next frame's ISR would leave burst N-1 here at the end of frame N */
      int e1 = -1, e2 = -1; uint8_t pb[148];
      if (cellule_bits_attendus(fn - 1, bsic, pb) == 0) { e1 = 0; for (int i = 0; i < 148; i++) if ((((int16_t)dsp->data[0x2be2 + i]) > 0) != (pb[i] != 0)) e1++; }
      if (cellule_bits_attendus(fn - 2, bsic, pb) == 0) { e2 = 0; for (int i = 0; i < 148; i++) if ((((int16_t)dsp->data[0x2be2 + i]) > 0) != (pb[i] != 0)) e2++; }
      /* regions of the data memory the demod wrote during phase B (beyond the
       * DARAM burst buffer and the API pages), with a few values each */
      { char z[900]; int k = 0; unsigned a = 0x60;
        while (a < C54X_DATA_SIZE && k < 800) {
            if (dsp->data[a] != g_data_avant_b[a]) {
                unsigned b = a; while (b < C54X_DATA_SIZE && b - a < 4096 && (dsp->data[b] != g_data_avant_b[b] || (b + 1 < C54X_DATA_SIZE && dsp->data[b+1] != g_data_avant_b[b+1]))) b++;
                k += snprintf(z + k, sizeof z - k, " %04x+%u", a, b - a); a = b;
            } else a++;
        }
        { const char *d = calypso_getenv("PONT_NB_HIST"); static unsigned nz;
          if (d && nz < 12) { char nom[256]; snprintf(nom, sizeof nom, "%s/zones_%u.txt", d, fn); FILE *f = fopen(nom, "w");
              if (f) { for (unsigned q = 0x60; q < C54X_DATA_SIZE; q++) if (dsp->data[q] != g_data_avant_b[q]) fprintf(f, "%04x %04x %04x\n", q, g_data_avant_b[q], dsp->data[q]); fclose(f); nz++; } } }
        printf("  [zones] fn=%u flags(OVA=%d OVB=%d C=%d TC=%d OVM=%d FRCT=%d SXM=%d) ecrites:%s\n", fn,
               g_flags_entree & 1, !!(g_flags_entree & 2), !!(g_flags_entree & 4), !!(g_flags_entree & 8), !!(g_flags_entree & 16), !!(g_flags_entree & 32), !!(g_flags_entree & 64), z); }
      if (g_snap_ok) { int es = 0; for (int i = 0; i < 148; i++) if ((((int16_t)g_snap_2be2[i]) > 0) != (bits[i] != 0)) es++;
                       printf("  [scan] fn=%u 2be2 au pas 12800 (avant le decodeur) : erreurs=%d\n", fn, es); g_snap_ok = 0; }
      printf("  [scan] fn=%u marge=%d tsc=%d dec=%.2f phase=%.1f rif=%d 2be2 erreurs=%d (vs N-1: %d, N-2: %d) insnA=%u insnB=%u %s  v[0..3]=%d %d %d %d\n", fn, cellule_marge_nb, cellule_tsc_force, cellule_dec_nb, cellule_phase_nb, calypso_rif_level(), err, e1, e2, g_insn_a, g_insn_b, map,
             (int16_t)dsp->data[0x2be2], (int16_t)dsp->data[0x2be3], (int16_t)dsp->data[0x2be4], (int16_t)dsp->data[0x2be5]); }
    /* after burst 3: the decoder's vectors. Search the whole data memory for
     * the 456 coded bits in deinterleaved order and for the 184 information
     * bits, as signs of int16 words and as hard 0/1 words. */
    { uint8_t code[456], info[184];
      if (((fn % 51u) % 10u - 2) % 4 == 3 && cellule_bloc_attendu(fn, bsic, code, info) == 0) {
          const struct { const uint8_t *t; int len; const char *nom; } tp[2] = { { code, 456, "456 codes" }, { info, 184, "184 info" } };
          printf("  [bloc] fn=%u :", fn);
          for (int k = 0; k < 2; k++) {
              int best[3] = {0,0,0}; unsigned bad[3] = {0,0,0};
              for (unsigned a = 0x60; a + tp[k].len < C54X_DATA_SIZE; a++) {
                  int m0 = 0, m1 = 0, m2 = 0;
                  for (int i = 0; i < tp[k].len; i++) {
                      int16_t v = (int16_t)dsp->data[a + i];
                      if ((v < 0) == (tp[k].t[i] != 0)) m0++;
                      if ((v > 0) == (tp[k].t[i] != 0)) m1++;
                      if ((v == 1 && tp[k].t[i]) || (v == 0 && !tp[k].t[i])) m2++;
                  }
                  if (m0 > best[0]) { best[0] = m0; bad[0] = a; }
                  if (m1 > best[1]) { best[1] = m1; bad[1] = a; }
                  if (m2 > best[2]) { best[2] = m2; bad[2] = a; }
              }
              printf("  %s: neg=1 %d/%d @%04x  pos=1 %d/%d @%04x  hard %d/%d @%04x |", tp[k].nom,
                     best[0], tp[k].len, bad[0], best[1], tp[k].len, bad[1], best[2], tp[k].len, bad[2]);
          }
          /* also the coded bits packed 16 per word (MSB first) */
          { int bestp = 0; unsigned badp = 0;
            for (unsigned a = 0x60; a + 29 < C54X_DATA_SIZE; a++) {
                int m = 0;
                for (int i = 0; i < 456; i++) { int bit = (dsp->data[a + i / 16] >> (15 - i % 16)) & 1; if (bit == code[i]) m++; }
                if (m > bestp) { bestp = m; badp = a; } }
            printf("  packed %d/456 @%04x", bestp, badp);
            int pb[4] = {0,0,0,0};
            for (int i = 0; i < 456; i++) { int bit = (dsp->data[badp + i / 16] >> (15 - i % 16)) & 1; if (bit == code[i]) pb[i & 3]++; }
            printf(" par burst %d %d %d %d /114", pb[0], pb[1], pb[2], pb[3]); }
          printf("\n");
          /* the decoder input at 0x2a00 (the trellis loop reads pairs from AR1 = 0x2a00) */
          if (g_snap_2a00_ok) { const char *d = calypso_getenv("PONT_NB_HIST"); if (d) { char nom[256]; snprintf(nom, sizeof nom, "%s/entree_%u.txt", d, fn); FILE *f = fopen(nom, "w"); if (f) { for (int i = 0; i < 456; i++) fprintf(f, "%d\n", (int16_t)g_snap_2a00[i]); fclose(f); } } }
          if (g_snap_2a00_ok) { int mp = 0, mn = 0, m1 = 0, nz = 0; g_snap_2a00_ok = 0;
            for (int i = 0; i < 456; i++) { int16_t v = (int16_t)g_snap_2a00[i]; if (v) nz++;
                if ((v > 0) == (code[i] != 0)) mp++; if ((v < 0) == (code[i] != 0)) mn++; if ((v == 1) == (code[i] != 0)) m1++; }
            printf("  [entree] fn=%u 0x2a00..+456 : nonzero=%d  pos=1 %d/456  neg=1 %d/456  one=1 %d/456 | premiers:", fn, nz, mp, mn, m1);
            for (int i = 0; i < 48; i++) printf(" %d", (int16_t)g_snap_2a00[i]);
            printf("\n  [entree] attendu :"); for (int i = 0; i < 48; i++) printf(" %d", code[i]); printf("\n");
            /* per-burst view: bits k with k%4==b */
            for (int b = 0; b < 4; b++) { int m = 0, n = 0; for (int i = b; i < 456; i += 4) { int16_t v = (int16_t)g_snap_2a00[i]; if (v) { n++; if ((v < 0) == (code[i] != 0)) m++; } }
                printf("  [entree] burst %d : neg=1 %d/%d\n", b, m, n); } }
          /* the 228 decoder output bits: best match over memory, hard words and LSB */
          { uint8_t u[228];
            if (cellule_u228_attendu(fn, bsic, u) == 0) {
                int best[2] = {0,0}; unsigned bad[2] = {0,0};
                for (unsigned a = 0x60; a + 228 < C54X_DATA_SIZE; a++) {
                    int m0 = 0, m1 = 0;
                    for (int i = 0; i < 228; i++) { uint16_t v = dsp->data[a + i];
                        if ((v == 1 && u[i]) || (v == 0 && !u[i])) m0++; if ((v & 1) == u[i]) m1++; }
                    if (m0 > best[0]) { best[0] = m0; bad[0] = a; } if (m1 > best[1]) { best[1] = m1; bad[1] = a; } }
                printf("  [u228] fn=%u hard %d/228 @%04x  lsb %d/228 @%04x | ", fn, best[0], bad[0], best[1], bad[1]);
                /* mismatch map at 0x2d66 (hard) */
                unsigned a = 0x2d66; int e = 0; char map[240]; int k = 0;
                for (int i = 0; i < 228; i++) { uint16_t v = dsp->data[a + i]; int ok = (v == u[i]); if (!ok) e++; if (i == 184 || i == 224) map[k++] = '|'; map[k++] = ok ? '.' : (v > 1 ? '?' : 'x'); }
                map[k] = 0; printf("2d66: %d faux %s\n", e, map);
                /* packed forms of the 228 bits anywhere in memory: 16 per word MSB
                 * first, LSB first, and the same with each word bit-reversed */
                { const char *nm[4] = { "msb", "lsb", "msb-rev", "lsb-rev" }; printf("  [u228] fn=%u packed:", fn);
                  for (int f = 0; f < 4; f++) { int best = 0; unsigned bad = 0;
                    for (unsigned a = 0x60; a + 15 < C54X_DATA_SIZE; a++) { int m = 0;
                        for (int i = 0; i < 228; i++) { uint16_t w = dsp->data[a + i / 16]; int bi = i % 16;
                            int bit = (f == 0) ? (w >> (15 - bi)) & 1 : (f == 1) ? (w >> bi) & 1 : (f == 2) ? (w >> bi) & 1 : (w >> (15 - bi)) & 1;
                            if (bit == u[i]) m++; }
                        if (m > best) { best = m; bad = a; } }
                    printf(" %s %d/228 @%04x", nm[f], best, bad); }
                  printf(" | 2c3c:"); for (int i = 0; i < 16; i++) printf(" %04x", dsp->data[0x2c3c + i]); printf("\n"); } } }
          } }
    /* where did the delivered samples land in DARAM (AAD)? offset in words at
     * which the ROM's buffer equals our frame, and how many words match */
    { uint16_t aad = calypso_rhea_dma_get_daram(); int best = 0, boff = 0;
      for (int off = -8; off <= 8; off++) {
          int m = 0;
          for (int i = 0; i < g_dernier_n_iq; i++) {
              int a = (int)aad + off + i; if (a < 0 || a >= C54X_DATA_SIZE) continue;
              if ((int16_t)dsp->data[a] == g_dernier_iq[i]) m++;
          }
          if (m > best) { best = m; boff = off; }
      }
      int p0 = -1; for (int i = 0; i < 40; i++) if ((int16_t)dsp->data[aad + i] != 0) { p0 = i; break; }
      { /* runs of words that differ from what was delivered, at offset 0 */
        char runs[256]; int k = 0, i = 0;
        while (i < g_dernier_n_iq && k < 200) {
            if ((int16_t)dsp->data[aad + i] != g_dernier_iq[i]) {
                int j = i; while (j < g_dernier_n_iq && (int16_t)dsp->data[aad + j] != g_dernier_iq[j]) j++;
                k += snprintf(runs + k, sizeof runs - k, " %d+%d(%d)", i, j - i, (int16_t)dsp->data[aad + i]); i = j;
            } else i++;
        }
        runs[k] = 0;
        int d_dma = 0; for (int q = 0; q < g_dernier_n_iq && q < 384; q++) if ((int16_t)g_daram_apres_dma[q] != g_dernier_iq[q]) d_dma++;
        printf("  [daram] fn=%u differences apres la trame (mot+longueur(valeur)):%s | juste apres le DMA, avant la ROM : %d mots differents (aad=%04x)\n", fn, runs, d_dma, g_daram_aad); }
      printf("  [daram] fn=%u aad=%04x livres=%d mots, identiques=%d a l'offset %d, premier mot non nul a +%d, mots 0..7: %d %d %d %d %d %d %d %d\n",
             fn, aad, g_dernier_n_iq, best, boff, p0,
             (int16_t)dsp->data[aad], (int16_t)dsp->data[aad+1], (int16_t)dsp->data[aad+2], (int16_t)dsp->data[aad+3],
             (int16_t)dsp->data[aad+4], (int16_t)dsp->data[aad+5], (int16_t)dsp->data[aad+6], (int16_t)dsp->data[aad+7]); }
    printf("\n");
}

static void servir(int fd, C54xState *dsp, uint16_t *api_ram, long insns, bool verbeux,
                   const char *iq_mode, int amp)
{
    bool injecter = iq_mode && *iq_mode && strcmp(iq_mode, "none") != 0;
    unsigned long injectes = 0;
    const uint16_t *a_sync = &api_ram[(API_NDB + NDB_A_SYNC_DEMOD) / 2];
    bool init_done = false;
    if (g_verif_sonde < 0) g_verif_sonde = calypso_getenv("CALYPSO_BSP_VERIF") ? 1 : 0;
    unsigned long trames = 0, irqs = 0, resets = 0;
    uint64_t insns_total = 0;
    const uint16_t *d_fb_det = &api_ram[(API_NDB + NDB_D_FB_DET) / 2];
    /* [2026-09-19] a_sch was read from R page 0 ONLY, hardcoded, while the DSP
     * writes its result to the page d_dsp_page designates, alternating. Stale
     * page-0 content read as a result is exactly what produces SB decodes that
     * are wrong yet REPEATABLE (measured: BSIC 44 six times, 22 five times out
     * of 17, never the 32 the capture carries). The PONT_CAN_SB hack in this
     * same file already writes BOTH pages, so the page was known to matter.
     * Both are kept here and the live one is picked per frame. */
    const uint16_t *a_sch_pg[2] = {
        &api_ram[(API_R_PAGE(0) + RP_A_SCH) / 2],
        &api_ram[(API_R_PAGE(1) + RP_A_SCH) / 2] };
    const uint16_t *a_sch0   = a_sch_pg[0];

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
            g_tick_irq_trame = (m.b & CALYPSO_PONT_TICK_IRQ_TRAME) != 0;
            bool deux_phases = (m.b & CALYPSO_PONT_TICK_DEUX_PHASES) != 0;
            m.b &= 1u;
            calypso_bsp_set_tpu_offset((int)m.c);   /* firmware RX window */
            /* AFC relay, closing the loop. The ARM writes d_afc (word 15 of the W
             * page) into the shared API RAM; on silicon the DSP serialises it to
             * the TWL3025 over the TSP. Without this relay the sample rotation
             * stays frozen and the frequency error never converges below the SB
             * threshold. m.b carries d_dsp_page, bit 0 selecting the W page. */
            { unsigned wp = m.b & 1u;
              int16_t dac = (int16_t)api_ram[(wp ? 0x14u : 0x00u) + 15u];
              static int16_t prev; static int first = 1;
              /* [2026-09-19] Measurement before any fix: the AFC DAC never settles,
               * every correction is followed by a write of exactly -700
               * (afc_initial_dac_value). The suspicion is the dual-page write the
               * set_afc_dac filter already documents — page A holding the inherited
               * init while page B carries the correction — with the filter guarding
               * only against 0, not against -700. Print BOTH pages so the claim can
               * be checked instead of assumed. */
              { static unsigned na;
                int16_t d0 = (int16_t)api_ram[0x00u + 15u], d1 = (int16_t)api_ram[0x14u + 15u];
                static int16_t p0 = 0x7fff, p1 = 0x7fff;
                if ((d0 != p0 || d1 != p1) && na++ < 40)
                    printf("  [afc] fn=%u w_page=%u relaye=%d | page0=%d page1=%d\n",
                           m.a, wp, dac, d0, d1);
                p0 = d0; p1 = d1; }
              if (first || dac != prev) { calypso_twl3025_set_afc_dac(dac); prev = dac; first = 0; } }
            { static unsigned _to=0; if (calypso_getenv("PONT_TPU_DEBUG") && (_to<5 || _to%2000==0)) printf("  [tpu] fn=%u tpu_offset=%u\n", m.a, m.c); _to++; }
            trace_armer();
            if (g_trace_reste > 0 && g_trace_f && !g_trace_ouverte && !g_trace_pc_hi) {
                /* wait for the first FB task posted by the ARM (W page 0 or 1) */
                static int md_cible = -1;
                if (md_cible < 0) { const char *e = calypso_getenv("PONT_TRACE_MD"); md_cible = (e && *e) ? atoi(e) : 5; }
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
            /* PONT_RX_MODE : milieu (default, see jouer_trame) | apres | avant.
             * PONT_RX_APRES=0/1 is still honoured as avant/apres. */
            static int rx_mode = -1;   /* 0 = milieu, 1 = apres, 2 = avant */
            if (rx_mode < 0) { const char *e = calypso_getenv("PONT_RX_MODE"); const char *a = calypso_getenv("PONT_RX_APRES");
                               rx_mode = (e && !strcmp(e, "apres")) ? 1 : (e && !strcmp(e, "avant")) ? 2
                                       : (a && *a == '1') ? 1 : (a && *a == '0') ? 2 : 0; }
            bool rx_apres = (rx_mode == 1);
            if (rx_mode == 2 && injecter && init_done) injecter_burst(dsp, iq_mode, amp, m.a, &injectes);
            /* Real chain: drain UDP socket 6702 (bursts from the bridge/BTS) and
             * hand them to the DSP. This is the only source when synthetic
             * injection is off, and a no-op when the socket is empty. */
            if (init_done && rx_mode != 0) calypso_bsp_service(m.a);
            g_inj.actif = (rx_mode == 0 && init_done);
            g_inj.iq_mode = (rx_mode == 0 && injecter) ? iq_mode : NULL;
            g_inj.amp = amp; g_inj.fn = m.a; g_inj.injectes = &injectes; g_inj.udp = (rx_mode == 0 && init_done);
            uint32_t ninsn = 0;
            bool init_avant = init_done;
            uint32_t drapeaux = jouer_trame(dsp, insns, &init_done, &ninsn, deux_phases ? 1 : 0);
            if (deux_phases) {
                /* [2026-09-21] Phase A done: the ROM's frame ISR has written the
                 * R page (previous burst) and armed the window. Tell QEMU, which
                 * raises the ARM frame IRQ, waits for the end of l1_sync() and
                 * sends PONT_GO; only then is the burst of this frame delivered.
                 * That is the silicon order, and what keeps "BURST ID n!=m" and
                 * "EMPTY" (prim_rx_nb.c) away. */
                sonde_pages("A", m.a, dsp, api_ram);
                envoyer(fd, PONT_DONE, drapeaux & ~PONT_DONE_API_IRQ, ninsn);
                bool go = false;
                while (!g_stop && !go) {
                    struct pollfd pg = { .fd = fd, .events = POLLIN };
                    if (poll(&pg, 1, 2000) <= 0) { static unsigned nt; if (nt++ < 3) printf("pont : PONT_GO attendu (fn=%u)\n", m.a); continue; }
                    CalypsoPontMsg g;
                    ssize_t ng = recv(fd, &g, sizeof(g), 0);
                    if (ng != (ssize_t)sizeof(g)) { printf("pont : ARM deconnecte en attente de GO (%zd)\n", ng); g_stop = 1; break; }
                    if (g.type == PONT_GO) go = true;
                    else { static unsigned nx; if (nx++ < 3) printf("pont : message %u recu en attente de GO, ignore\n", g.type); }
                }
                if (!go) break;
                sonde_pages("G", m.a, dsp, api_ram);
                if (calypso_getenv("PONT_NB_DEBUG") && ((m.a % 51u) % 10u - 2) % 4 == 3 && m.a % 51u <= 5 &&
                    (api_ram[API_R_PAGE(0) / 2] == 24 || api_ram[API_R_PAGE(1) / 2] == 24)) {
                    static unsigned nq;
                    if (nq++ < 8) {
                        printf("  [garde] fn=%u avant le burst 3, bursts precedents en memoire :", m.a);
                        for (int b = 1; b <= 3; b++) {
                            uint8_t bits[148]; if (cellule_bits_attendus(m.a - b, 42, bits) < 0) continue;
                            uint8_t t116[116]; memcpy(t116, bits + 3, 58); memcpy(t116 + 58, bits + 87, 58);
                            int best[3] = {0,0,0}; unsigned bad[3] = {0,0,0};
                            for (unsigned a = 0x60; a + 116 < C54X_DATA_SIZE; a++) {
                                int m0 = 0, m1 = 0, m2 = 0;
                                for (int i = 0; i < 116; i++) { int16_t v = (int16_t)dsp->data[a + i];
                                    if ((v < 0) == (t116[i] != 0)) m0++; if ((v > 0) == (t116[i] != 0)) m1++;
                                    if ((v == 1 && t116[i]) || (v == 0 && !t116[i])) m2++; }
                                if (m0 > best[0]) { best[0] = m0; bad[0] = a; } if (m1 > best[1]) { best[1] = m1; bad[1] = a; } if (m2 > best[2]) { best[2] = m2; bad[2] = a; }
                            }
                            /* packed 16 per word too */
                            int bestp = 0; unsigned badp = 0;
                            for (unsigned a = 0x60; a + 8 < C54X_DATA_SIZE; a++) { int mm = 0;
                                for (int i = 0; i < 116; i++) { int bit = (dsp->data[a + i / 16] >> (15 - i % 16)) & 1; if (bit == t116[i]) mm++; }
                                if (mm > bestp) { bestp = mm; badp = a; } }
                            printf(" b%d(fn %u): neg %d@%04x pos %d@%04x hard %d@%04x packed %d@%04x |", 3 - b, m.a - b, best[0], bad[0], best[1], bad[1], best[2], bad[2], bestp, badp);
                        }
                        printf("\n");
                    }
                }
                uint32_t n2 = 0;
                g_insn_a = ninsn;
                drapeaux = jouer_trame(dsp, insns, &init_done, &n2, 2);
                ninsn += n2; g_insn_b = n2;
            }
            g_inj.actif = false;
            sonde_pages("B", m.a, dsp, api_ram);
            if (g_inj.dernier_type == 'B' && (api_ram[API_R_PAGE(0) / 2] == 24 || api_ram[API_R_PAGE(1) / 2] == 24)) sonde_bits(m.a, dsp, 42);
            /* Reference probe (CALYPSO_BSP_VERIF=1): compare DARAM against the
             * burst the BSP was handed, AFTER the DSP has run — the samples
             * only reach DARAM through the DSP's own DMA draining the RIF, so
             * there is nothing to compare before jouer_trame. */
            if (g_verif_sonde) {
                uint32_t vfn = 0; uint16_t vad = 0; int vn = 0, vage = 0;
                int ident = calypso_bsp_verif_compare(&vfn, &vad, &vn, &vage);
                if (ident >= 0 && vn > 0) {
                    static unsigned nv;
                    if (nv++ < 4000) {
                        uint16_t vpg = calypso_bsp_verif_last_page();
                        printf("  [verif] fn=%u p51=%u age=%d page=0x%04x w_page=%d : "
                               "%d/%d en 0x%04x %s\n",
                               vfn, vfn % 51u, vage, vpg, (int)(vpg & 1u),
                               ident, vn, vad, ident == vn ? "VALIDE" : "partiel");
                    }
                }
            }
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
            }
            /* [2026-09-20] STREAM PUMP (same as rejouer.c). DMA2 now fills its
             * double buffer with full pages and interrupts once per pair; the
             * ROM's ISR consumes both halves and the DSP idles. Then the next
             * pair is handed over, until the receiver holds less than a pair. A
             * 156.25-symbol frame is 3.25 pages, so this runs 1 or 2 times. */
            for (int k = 0; k < 40 && dsp->running; k++) {   /* 13 page pairs per 1250-symbol frame */
                if (!calypso_rhea_dma_pump(dsp)) break;
                if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
                uint32_t av2 = dsp->insn_count;
                if (!dsp->idle) c54x_run_profile(dsp, (int)insns / 4);
                ninsn += dsp->insn_count - av2;
                drapeaux = (drapeaux & ~PONT_DONE_IDLE) | (dsp->idle ? PONT_DONE_IDLE : 0);
            }
            drapeaux = (drapeaux & ~PONT_DONE_IDLE) | (dsp->idle ? PONT_DONE_IDLE : 0);
            /* [2026-09-20] NB probe (PONT_NB_DEBUG=1): what the ROM leaves in the
             * R pages for a normal-burst task (ALLC/BCCH), per burst: d_task_d,
             * d_burst_d, a_serv_demod (TOA, PM, ANGLE, SNR) and, on burst 3,
             * the a_cd header of the NDB (Fire/CRC word, bit errors) plus the
             * first decoded octets. Read alongside mobile's "Dropping frame
             * with N bit errors". */
            {
                static int nbdbg = -1; static unsigned nbn;
                if (nbdbg < 0) nbdbg = calypso_getenv("PONT_NB_DEBUG") ? 1 : 0;
                if (nbdbg && nbn < 400) {
                    static uint16_t prev[2][4];
                    for (int pg = 0; pg < 2; pg++) {
                        const uint16_t *r = &api_ram[API_R_PAGE(pg) / 2];
                        uint16_t cur[4] = { r[RP_D_TASK_D/2], r[RP_D_BURST_D/2], r[RP_A_SERV_DEMOD/2 + D_TOA], r[RP_A_SERV_DEMOD/2 + D_PM] };
                        if (r[RP_D_TASK_D/2] && memcmp(cur, prev[pg], sizeof cur)) {
                            const uint16_t *w0 = &api_ram[API_W_PAGE(0) / 2], *w1 = &api_ram[API_W_PAGE(1) / 2];
                            printf("  [nb] fn=%u p51=%u R%d task_d=%u burst_d=%u TOA=%d PM=%d ANGLE=%d SNR=%u | W0 task_d=%u burst=%u W1 task_d=%u burst=%u | livre=%c n_iq=%d one_shot=%d len=%u mots",
                                   m.a, m.a % 51u, pg, r[RP_D_TASK_D/2], r[RP_D_BURST_D/2],
                                   (int16_t)r[RP_A_SERV_DEMOD/2 + D_TOA], (int16_t)r[RP_A_SERV_DEMOD/2 + D_PM],
                                   (int16_t)r[RP_A_SERV_DEMOD/2 + D_ANGLE], r[RP_A_SERV_DEMOD/2 + D_SNR],
                                   w0[0], w0[1], w1[0], w1[1], g_inj.dernier_type, g_inj.dernier_n_iq,
                                   calypso_rhea_dma_one_shot(), calypso_rhea_dma_get_len_words());
                            if (r[RP_D_BURST_D/2] == 3) {
                                const uint16_t *cd = &api_ram[(API_NDB + NDB_A_CD) / 2];
                                printf(" | a_cd=%04x %04x %04x :", cd[0], cd[1], cd[2]);
                                for (int k = 3; k < 15; k++) printf(" %04x", cd[k]);
                            }
                            printf("\n"); nbn++;
                        }
                        memcpy(prev[pg], cur, sizeof cur);
                    }
                }
            }
            /* Canned results: scaffolding to prove the pipeline through to the
             * LU, not an end state. PONT_CAN_TOA=23 forces the reported TOA
             * (a_sync_demod[D_TOA]) to the on-time value the firmware expects
             * (prim_fbsb.c subtracts 23). Drop it once the correlator's native
             * TOA is correct. -1, the default, disables canning. */
            {
                static int can_toa = -2;
                if (can_toa == -2) { const char *e = calypso_getenv("PONT_CAN_TOA"); can_toa = (e && *e) ? atoi(e) : -1; }
                if (can_toa >= 0 && *d_fb_det)
                    api_ram[(API_NDB + NDB_A_SYNC_DEMOD) / 2 + D_TOA] = (uint16_t)can_toa;
                /* SB: the firmware reads a_serv_demod[D_TOA] from the read page
                 * and expects about 4. PONT_CAN_SB_TOA=4 pins it on both R
                 * pages, which frames the SCH correctly. */
                static int can_sb = -2;
                if (can_sb == -2) { const char *e = calypso_getenv("PONT_CAN_SB_TOA"); can_sb = (e && *e) ? atoi(e) : -1; }
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
                if (can_sb_full == -2) { const char *e = calypso_getenv("PONT_CAN_SB"); can_sb_full = (e && *e) ? 1 : 0; if (e && *e) can_sb_bsic = atoi(e) & 0x3f; }
                if (can_sb_full) {
                    int md0 = api_ram[4] & 0xff, md1 = api_ram[0x18] & 0xff;   /* d_task_md on W pages 0 and 1 */
                    if (md0 == 6 || md1 == 6) {
                        /* [2026-09-19] The frame MUST be the tick's own (m.a), not
                         * calypso_bsp_get_last_fn(): measured, the two diverged badly
                         * (the canned SB announced fn=5662 while the firmware sat at
                         * 11261), and Synchronize_TDMA then locked the ARM onto a frame
                         * number unrelated to what the injector delivers — every burst
                         * after the sync misaligned. A canned SB has to carry the clock
                         * the cell is actually generated from, or it proves nothing. */
                        uint32_t bfn = m.a;
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
            /* [2026-09-19] ONE LINE PER SB ATTEMPT. Everything upstream of the SB
             * task is now measured correct — right frame, right window, right
             * sample offset — yet the CRC passes in 0.4% of attempts. A decode
             * that were simply broken would give 0%, so something DISCRIMINATES
             * the rare successes. This logs, for every frame the ARM has an SB
             * task posted, the TOA the FB left behind (a_sync[0], the value the
             * firmware positions the window from) next to the CRC outcome, so
             * the two populations can be compared directly. */
            /* The a_sync cells are already cleared by the time the SB task is
             * posted (measured: toa=pm=ang=0 on every attempt), so the FB result
             * has to be LATCHED when d_fb_det rises and carried to the attempt. */
            static int lat_toa, lat_pm, lat_ang; static uint32_t lat_fn;
            { static int fbl = 0;
              if (*d_fb_det && !fbl) {
                  lat_toa = (int)(int16_t)a_sync[0]; lat_pm = a_sync[1];
                  lat_ang = (int)(int16_t)a_sync[2]; lat_fn = m.a;
              }
              fbl = *d_fb_det != 0; }
            { static int sb_prev = 0;
              int md_0 = api_ram[4] & 0xff, md_1 = api_ram[0x18] & 0xff;
              int sb_now = (md_0 == 6 || md_1 == 6);
              if (sb_now && !sb_prev) {
                  static unsigned nsb;
                  if (nsb++ < 4000)
                      printf("  [sb] fn=%u p51=%u | FB a fn=%u toa=%d pm=%u ang=%d "
                             "| TOA-ROM=%d src=%s "
                             "| a_sch=%04x %s\n",
                             m.a, m.a % 51u, lat_fn, lat_toa, lat_pm, lat_ang,
                             g_toa_valeur,
                             g_toa_grille < 0 ? "?" : (g_toa_grille ? "GRILLE(0x0cce)" : "fine"),
                             a_sch0[0],
                             ((a_sch0[0] & 0x8100) == 0x8000) ? "CRC_OK" : "crc_ko");
              }
              sb_prev = sb_now; }
            /* CRC watched on BOTH R pages, and the page reported: reading page 0
             * alone cannot tell a real decode from stale content. A genuine SCH
             * gives the SAME BSIC every time (32 for cellule_reelle.bin); a
             * value that changes at each hit is a 10-bit CRC passing by chance
             * (1/1024) or a stale cell. */
            /* [2026-09-19] Who writes a_sch at all? The DSP never does: the
             * A_SCH-WR probe in c54x_mem.c (live, its ANGLE-WR neighbour fires)
             * counted ZERO stores by the ROM to 0x0837..0x083b / 0x084b..0x084f.
             * The only host writer is the PONT_CAN_SB block below, which is off.
             * Yet the cells change. The remaining writer is the ARM, through the
             * shared mapping, which no DSP-side probe can see -- so watch the
             * VALUES from here and name the frame. */
            /* [2026-09-19] Who maintains d_dsp_page? calypso_api.h: an armed page
             * is B_GSM_TASK|page = 0x0002 or 0x0003; 0x0000 is the reset state
             * l1s_reset_hw() writes (sync.c:165), and it also puts the firmware
             * back on R page 0. Measured at 0x0000 on 39% of samples, because
             * the FBSB loop restarts ~12000 times. On silicon the DSP re-arms
             * the page itself; this names every transition and its writer. */
            { static uint16_t dpp; static int dfirst = 1; static unsigned ndp;
              uint16_t dp = api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2];
              if (!dfirst && dp != dpp && ndp < 40) {
                  printf("  [page] fn=%u d_dsp_page %04x -> %04x  (%s)\n", m.a, dpp, dp,
                         (dp & 0x0002) ? ((dp & 1) ? "arme page 1" : "arme page 0")
                                       : "NON ARME (etat de reset)");
                  ndp++;
              }
              dpp = dp; dfirst = 0; }
            { static uint16_t prev[2][5]; static int first = 1; static unsigned nch;
              for (int pg = 0; pg < 2; pg++) {
                  const uint16_t *a = a_sch_pg[pg];
                  if (!first && nch < 60 &&
                      (a[0] != prev[pg][0] || a[3] != prev[pg][3] || a[4] != prev[pg][4])) {
                      printf("  [a_sch] fn=%u page=%d : %04x %04x %04x %04x -> "
                             "%04x %04x %04x %04x  (d_dsp_page=%04x)\n",
                             m.a, pg, prev[pg][0], prev[pg][1], prev[pg][3], prev[pg][4],
                             a[0], a[1], a[3], a[4],
                             api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2]);
                      nch++;
                  }
                  prev[pg][0]=a[0]; prev[pg][1]=a[1]; prev[pg][3]=a[3]; prev[pg][4]=a[4];
              }
              first = 0; }
            { static int crc_prev[2] = {1, 1};
              for (int pg = 0; pg < 2; pg++) {
                  const uint16_t *a = a_sch_pg[pg];
                  /* [2026-09-19] The criterion (a[0] & 0x8100) == 0x8000 counts
                   * SATURATED ACCUMULATORS as CRC OK: a_sch[0] has been seen
                   * carrying plain numbers, and 0x8000 is exactly what a
                   * saturated accumulator stores (rejouer.c:923). Every "SB CRC
                   * OK" of this session rested on it, so it is replaced by a
                   * test the arithmetic cannot pass by accident: GSM 04.08
                   * bounds T2 <= 25 and T3' <= 4, so 6 of 32 T2 values and 3 of
                   * 8 T3' values are IMPOSSIBLE in a real SCH. */
                  uint32_t _sb = (uint32_t)a[3] | ((uint32_t)a[4] << 16);
                  unsigned _t2  = (_sb >> 18) & 0x1f;
                  unsigned _t3p = ((_sb >> 24) & 1) | ((_sb >> 15) & 6);
                  int _plausible = (_t2 <= 25) && (_t3p <= 4);
                  int crc_ok = ((a[0] & 0x8100) == 0x8000) && _plausible;
                  if (crc_ok && !crc_prev[pg]) {
                      uint32_t sb = (uint32_t)a[3] | ((uint32_t)a[4] << 16);
                      printf("  [jalon] fn=%u SB PLAUSIBLE page=%d BSIC=%u "
                             "a_sch=%04x %04x %04x %04x  hacks=%s\n",
                             m.a, pg, (unsigned)((sb >> 2) & 0x3f),
                             a[0], a[1], a[3], a[4], hacks_actifs());
                  }
                  crc_prev[pg] = crc_ok;
              } }
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
    { static int16_t rempl[2 * 148];         /* TS1..TS7 of the C0 carrier: dummy bursts */
      cellule_factice(amp, 0.5, rempl);
      calypso_bsp_set_remplissage(rempl, 2 * 148); }

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
