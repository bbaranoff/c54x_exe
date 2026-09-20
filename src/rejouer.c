/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * rejouer.c - deterministic replay of FB/SB acquisition on the real DSP.
 *
 * Two identical runs of the full bench (QEMU + osmocon + mobile) diverged: the
 * ARM (TCG) advances at host speed between frame interrupts, so the frame on
 * which it posts the FB task changes from run to run. Here the ARM is NOT
 * emulated: its layer 1 is replayed in C (l1_sync + fb_sched_set/sb_sched_set
 * from prim_fbsb.c/sync.c), injection is locked to the frame counter, and
 * nothing reads the host clock. Same input => same output, always.
 *
 * This is not an operating mode: it is a DSP measurement bench (FB then SB)
 * driven by the same command sequence as the real firmware.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "calypso_c54x.h"
#include "calypso_bsp.h"
#include "hw/arm/calypso/calypso_api.h"
#include "cellule.h"
#include "calypso_twl3025.h"
#include "calypso_rhea_dma.h"
#include "rejouer.h"
#include <osmocom/core/bits.h>
#include <osmocom/core/crcgen.h>
#include <osmocom/coding/gsm0503_parity.h>
#include "hw/arm/calypso/calypso_debug.h"

/* ---- API offsets in WORDS from the API base (= DSP 0x800) --------------- */
#define W_PAGE(p)     ((p) ? 0x14u : 0x00u)   /* T_DB_MCU_TO_DSP, 17 words */
#define R_PAGE(p)     ((p) ? 0x3Cu : 0x28u)   /* T_DB_DSP_TO_MCU, 20 words */
#define W_SIZE        17u
#define R_SIZE        20u
#define W_TASK_D      0u
#define W_TASK_MD     4u
#define W_CTRL_ABB    11u
#define W_AFC         15u
#define W_CTRL_SYS    16u
#define NDB           0xD4u
#define NDB_PAGE      (NDB + 0u)
#define NDB_ERRSTAT   (NDB + 1u)
#define NDB_FB_DET    (NDB + 36u)
#define NDB_FB_MODE   (NDB + 37u)
#define NDB_SYNC      (NDB + 38u)   /* a_sync_demod[TOA,PM,ANGLE,SNR] */
#define R_SERV        8u            /* a_serv_demod[4] */
/* [2026-09-18] SYMBOL SAMPLING OFFSET. Replay sampled the GMSK at 0.0, i.e. on
 * the symbol BOUNDARY, while the bridge uses 0.5, the CENTER. With the gaussian
 * pulse of gmsk.c (BT=0.3), the weight of the target symbol in the phase
 * difference between two consecutive samples is:
 *
 *   offset   target symbol   previous neighbour   next neighbour
 *    0.0         0.454            0.023               0.492
 *    0.25        0.590            0.069               0.333
 *    0.5         0.653            0.158               0.189
 *
 * At 0.0 the NEXT neighbour outweighs the symbol being read: the offset is not
 * noisy, it is UNDETERMINED, and the demodulator may latch onto either one. That
 * accounts for offset=22 at margin=21, midamble at 56/63, data at 71-73/78 with
 * otherwise perfect samples, and the correlation peak jumping between adjacent
 * lags. */
/* Tunable for sweeps: 0.5 (symbol center) is argued above, but it is a
 * hypothesis and the bench must be able to test it.
 * REJEU_DECALAGE_SYMB=<x>, default 0.5. */
static double decalage_symb(void)
{
    static double d = -1.0;
    if (d < 0) { const char *e = calypso_getenv("REJEU_DECALAGE_SYMB");
                 d = (e && *e) ? atof(e) : 0.5;
                 if (d < 0 || d >= 1.0) d = 0.5; }
    return d;
}
#define DECALAGE_SYMB decalage_symb()
/* Tunable head margin (REJEU_MARGE, default 21); the tail fills out to 190 complex samples */
static int marge_tete(void)
{
    static int m = -1;
    if (m < 0) { const char *e = calypso_getenv("REJEU_MARGE"); m = e ? atoi(e) : 21;
                 if (m < 0 || m > 41) m = 21;
                 cellule_marge_fin = 190 - 148 - m; }
    return m;
}

#define R_SCH         15u           /* a_sch[5] */
#define B_SCH_CRC     8
/* B_GSM_TASK comes from calypso_api.h (bit mask, (1u << 1)) */
#define B_AFC         4
#define FB_DSP_TASK   5
#define SB_DSP_TASK   6
#define BITS_PER_TDMA 1250   /* tpu.h: a TDMA frame is 8 x 156.25 = 1250 bit periods */

/* firmware thresholds (prim_fbsb.c: SNR not gating, #else FB*_SNR_THRESH=0) */
#define THRESH1       (11000 - 1000)
#define THRESH2       (1000 - 200)
#define AFC_RETRY_MAX 30
#define FB0_RETRY_MAX 3
/* sync.h: ANGLE_TO_FREQ(a) = a * BITFREQ_DIV_PI / ANG2FREQ_SCALING */
#ifndef BITFREQ_DIV_PI
#define BITFREQ_DIV_PI   86208   /* sync.h:203: 270kHz/pi */
#endif
#ifndef ANG2FREQ_SCALING
#define ANG2FREQ_SCALING (2<<15) /* sync.h:204: fx1.15 */
#endif
#define ANGLE2FREQ(a) ((int)(int16_t)(a) * BITFREQ_DIV_PI / ANG2FREQ_SCALING)

/* ---- replayed ARM state ------------------------------------------------- */
typedef void (*cb_t)(int attempt);
struct item { int frame; cb_t cb; int attempt; };

static uint16_t *api;
static C54xState *dsp;
static unsigned w_page, r_page, r_page_used;
static uint32_t fn_cur;              /* l1s.current_time.fn */
static unsigned long histo_pc[64];
static unsigned long insn_total;
static unsigned char *vu_sb, *vu_hors;
static int16_t g_livre_iq[2*256]; static int g_livre_niq;
static char g_livre_type; static uint32_t g_livre_fn; static int g_livre_n;
static int      afc_dac = -700;      /* afc_initial_dac_value (compal) */
static int      fb_mode, afc_retries, fb0_retries;
static struct   item sched[32]; static int n_sched;
static struct   { int toa, pm, angle, snr, freq_diff, attempt; uint32_t fnr; } fb;
static int      verdict;             /* 0 running, 1 SB OK, -1 gave up */
static uint32_t sb_word; static int sb_bsic; static uint32_t sb_fn;
static int      n_fb_ok, n_sb_try, n_sb_crcfail, n_crc_ok, n_sb_vraies;
static int      n_err_dsp, n_err8;
static long     n_cmps_sb, n_cmps_hors, n_e0_sb, n_sttrn_sb;
static long     n_e0x_sb[4];
static int      n_firs_vus;
static long     n_cmps_reg[64];
static long     n_xc_reg[64];
static int      n_xc_vus;
static int      n_bal;
static int      n_sat_vus;
static int      n_ecr;
static int      n_2a0;
static int      g_bsic_injecte;   /* BSIC actually transmitted by the cell */
static int      trace;
static unsigned long hit_7c31, hit_9841, hit_84a1, hit_770a, hit_b219, hit_7a16;
static unsigned long hit_8478, hit_8492, hit_8493, hit_8497;
static unsigned long hit_7d1c, hit_7d1d, hit_7d1e, hit_81e4;
/* [2026-09-19] SB PATH OPCODE INVENTORY. The fault is confined to the window
 * 0x84a1..0x8497, but which instruction is mis-emulated is unknown. Record the
 * distinct opcodes executed inside the SB demod with their pass count and one
 * witness PC: that gives a finite list to audit against SPRU172C instead of an
 * intuition. REJEU_OPCODES=1. */
/* Environment flags read ONCE in rejouer(): calypso_getenv() inside the per-instruction
 * loop was a linear scan of environ per emulated instruction. */
static int env_div, env_softs_continu, env_firs, env_probe_t, env_decodeur;
static uint16_t g_ad_avant;
static uint32_t g_vie_fn; static uint16_t g_vie_ad;
static int g_dans_sb;            /* true between 0x7c31 (demod) and 0x9841 (decoder) */
static unsigned long g_op_n[65536];
static unsigned long g_op_dec[65536];   /* DECODER opcodes, region 0x9800-0x9bff */
static unsigned long g_op_eq[65536];    /* EQUALIZER opcodes, region 0x8400-0x84ff */
static unsigned long g_n_dec;           /* number of decodes (passes at 0x9841) */
static uint16_t      g_op_pc[65536];
static unsigned long g_op_tous[65536];   /* every executed opcode word, both paths */

/* [2026-09-18] REAL SCH BURSTS. Until now the DSP only ever saw our own
 * fixture, a synthetic GMSK SCH at 1 sample/symbol, so "is the ROM sound, or is
 * the fixture too poor for a coherent equalizer?" stayed open. Inject SCH bursts
 * extracted from a REAL capture (ptrkrysik/test_data, ARFCN 725, USRP decimation
 * 174 -> 574712.6 Hz) whose answer is known: BSIC=32, Viterbi decode with 0
 * errors, valid CRC. REJEU_SCH_REEL=<file>. The framing is unchanged -- same
 * margins, same timing -- ONLY the burst content differs. */
static int16_t (*g_reels)[296];
static uint8_t (*g_reels_code)[78];      /* the 78 bits actually transmitted */
static uint32_t *g_reels_fn; static uint16_t *g_reels_bsic;
static unsigned g_n_reels, g_reel_i;
static int g_reel_pour_fn[64];           /* which frame received which burst */
static int reels_charger(const char *chemin)
{
    FILE *f = fopen(chemin, "rb");
    if (!f) { fprintf(stderr, "SCH reels : %s illisible\n", chemin); return -1; }
    uint32_t n = 0, nsym = 0, ncode = 0;
    if (fread(&n, 4, 1, f) != 1 || fread(&nsym, 4, 1, f) != 1 ||
        fread(&ncode, 4, 1, f) != 1 || nsym != 148 || ncode != 78 || !n) {
        fprintf(stderr, "SCH reels : entete invalide\n"); fclose(f); return -1; }
    g_reels = calloc(n, sizeof *g_reels);
    g_reels_code = calloc(n, sizeof *g_reels_code);
    g_reels_fn = calloc(n, sizeof *g_reels_fn);
    g_reels_bsic = calloc(n, sizeof *g_reels_bsic);
    if (!g_reels || !g_reels_code || !g_reels_fn || !g_reels_bsic) { fclose(f); return -1; }
    for (uint32_t i = 0; i < n; i++) {
        uint32_t fn; uint16_t bsic; uint8_t pad[2];
        if (fread(&fn,4,1,f)!=1 || fread(&bsic,2,1,f)!=1 || fread(pad,1,2,f)!=2 ||
            fread(g_reels[i], sizeof(int16_t), 296, f) != 296 ||
            fread(g_reels_code[i], 1, 78, f) != 78) { n = i; break; }
        g_reels_fn[i] = fn; g_reels_bsic[i] = bsic;
    }
    fclose(f); g_n_reels = n;
    for (int i = 0; i < 64; i++) g_reel_pour_fn[i] = -1;
    printf("SCH reels : %u bursts charges depuis %s (BSIC attendu %u)\n",
           n, chemin, n ? g_reels_bsic[0] : 0);
    return n ? 0 : -1;
}
/* Read PROGRAM space the way the core does: with OVLY set, DARAM 0x0060..0x27FF
 * is aliased into program space, so reading dsp->prog[] directly misses the
 * alias -- exactly the trap the FIRS probe must avoid. */
static uint16_t prog_ovly(C54xState *d, uint16_t a)
{
    if ((d->pmst & (1u << 5)) && a >= 0x0060 && a < 0x2800) return d->data[a];
    return d->prog[a];
}

static void plan(int delay, cb_t cb, int attempt)
{
    if (n_sched >= 32) return;
    sched[n_sched].frame = (int)fn_cur + delay;
    sched[n_sched].cb = cb; sched[n_sched].attempt = attempt; n_sched++;
}
static void sched_reset(void) { n_sched = 0; }

static uint16_t *dbw(void) { return &api[W_PAGE(w_page)]; }
static uint16_t *dbr(void) { return &api[R_PAGE(r_page)]; }

/* ---- callbacks: faithful copy of prim_fbsb.c ---------------------------- */
static void fbdet_cmd(int unused);
static void fbdet_resp(int attempt);
static void sbdet_cmd(int attempt);

/* calypso_getenv("X") alone is TRUE even for X=0, so a line written
 * `REJEU_SCH_PARTOUT=0 REJEU_SB_FORCE=0` enabled every hack instead of cutting
 * them. Here "0", "", "non", "no" and "off" are false. */
static int drapeau_env(const char *nom)
{
    const char *e = calypso_getenv(nom);
    if (!e || !*e) return 0;
    if (e[0] == '0' && e[1] == 0) return 0;
    if (!strcmp(e, "non") || !strcmp(e, "no") || !strcmp(e, "off")) return 0;
    return 1;
}

static void plan_sb_annule(void) { }

static void sbdet_resp(int attempt);

static void plan_fb_set(int delay, int mode)
{
    fb_mode = mode;
    plan(delay + 0, fbdet_cmd, 0);
    for (int a = 1; a <= 12; a++) plan(delay + 1 + a, fbdet_resp, a);
}
static int sb_uniq(void)
{
    static int u = -1;
    if (u < 0) u = drapeau_env("REJEU_SB_UNIQUE") ? 1 : 0;
    return u;
}
/* Last scheduled SB attempt: that one must restart an acquisition. Hardcoding
 * it to "attempt == 2" leaves the bench silent under a single command, where
 * that attempt does not exist. */
static int sb_dernier_essai(void) { return sb_uniq() ? 1 : 2; }

static void plan_sb_set(int delay)
{
/* [2026-09-18] The DSP demodulates the burst of the frame FOLLOWING the command
     * (measured by perturbation: command at fn=11 -> only a perturbation at fn=12
     * changes the softs). Posting on TWO consecutive frames therefore consumes f and
     * f+1; SCH frames are 10 apart, so the second attempt always reads a DUMMY
     * burst, which is half the measurements taken on a constant input.
     * REJEU_SB_UNIQUE=1 posts a single command so every attempt sees a real SCH. */
    plan(delay + 0, sbdet_cmd, 1);
    if (!sb_uniq()) plan(delay + 1, sbdet_cmd, 2);
    plan(delay + 3, sbdet_resp, 1);
    if (!sb_uniq()) plan(delay + 4, sbdet_resp, 2);
}

static uint32_t g_sb_cmd_fn;   /* frame of the LAST SB command = the one the DSP demodulates */
static uint32_t fb_cmd_fn;      /* frame on which the ARM posted the FB task */
static void fbdet_cmd(int unused)
{
    (void)unused;
    fb_cmd_fn = fn_cur;
    dbw()[W_TASK_MD]  = FB_DSP_TASK;
    api[NDB_FB_MODE]  = (uint16_t)fb_mode;
}

static void fbdet_resp(int attempt)
{
    if (!api[NDB_FB_DET]) {
        if (attempt < 12) return;
        sched_reset();
        if (fb0_retries < FB0_RETRY_MAX) { fb0_retries++; plan_fb_set(1, 0); }
        else verdict = -1;
        return;
    }
    /* read_fb_result */
    fb.toa   = (int16_t)api[NDB_SYNC + 0];
    fb.pm    = (int16_t)api[NDB_SYNC + 1] >> 3;
    fb.angle = (int16_t)api[NDB_SYNC + 2];
    fb.snr   = (int16_t)api[NDB_SYNC + 3];
    fb.freq_diff = ANGLE2FREQ(fb.angle);
    fb.fnr = fn_cur; fb.attempt = attempt;
    api[NDB_FB_DET] = 0; api[NDB_SYNC + 0] = 0;
    n_fb_ok++;
    if (trace) {
        /* real FCCH = first frame >= cmd with p51 in {0,10,20,30,40} */
        uint32_t f = fb_cmd_fn; while ((f % 51) % 10 != 0 || (f % 51) > 40) f++;
        int dframe = (int)(f - fb_cmd_fn);                 /* FCCH frame inside the window */
        int toa_attendu = dframe * BITS_PER_TDMA + 23;      /* what the firmware can read */
        int ntdma_lu = (fb.toa - 23) / BITS_PER_TDMA;
        printf("  FB%d att=%d fn=%u TOA=%d (df=%d) | cmd=%u FCCH reelle=%u (+%d trames) "
               "=> TOA attendu ~%d, ntdma lu=%d au lieu de %d\n",
               fb_mode, attempt, fn_cur, fb.toa, fb.freq_diff,
               fb_cmd_fn, f, dframe, toa_attendu, ntdma_lu, dframe);
    }
    /* afc_correct: delta = (norm * err)/slope; slope compal_e88 = 287 */
    afc_dac += (int)(((32768 / 947) * (long)fb.freq_diff) / 287);
    if (afc_dac > 4095) afc_dac = 4095; if (afc_dac < -4096) afc_dac = -4096;
    sched_reset();
    if (fb_mode == 0) {
        if (abs(fb.freq_diff) < THRESH1) plan_fb_set(1, 1);
        else if (afc_retries < AFC_RETRY_MAX) { afc_retries++; plan_fb_set(1, 0); }
        else verdict = -1;
    } else {
        int toa = fb.toa - 23, ntdma, qbits;
        if (toa < 0) { qbits = (toa + BITS_PER_TDMA) * 4; ntdma = -1; }
        else { ntdma = toa / BITS_PER_TDMA; qbits = (toa - ntdma * BITS_PER_TDMA) * 4; }
        int fn_offset = (int)fn_cur - attempt + ntdma;
        int delay = fn_offset + 11 - (int)fn_cur - 1;
        if (trace) printf("    -> toa-23=%d ntdma=%d qbits=%d delay=%d\n", toa, ntdma, qbits, delay);
        if (abs(fb.freq_diff) < THRESH2) {
            if (delay < 0) delay = 0; if (delay > 20) delay = 20;
            /* REJEU_SB_FORCE=1: aim at the NEXT SCH frame (p51 in {1,11,21,31,41})
             * instead of the computed delay. Separates the DEMODULATOR from the
             * TIMING: a CRC passing here means only the timing (ntdma) is at
             * fault; a failure means the SB demodulation really is wrong. */
            static int force = -1;
            if (force < 0) force = drapeau_env("REJEU_SB_FORCE") ? 1 : 0;
            if (force) {
                uint32_t f = fn_cur + 1;
                while (!((f % 51) % 10 == 1 && (f % 51) <= 41)) f++;
                delay = (int)(f - fn_cur);
                /* [2026-09-18] THE CONSUMED FRAME IS NOT THE FIRST COMMAND.
                 * plan_sb_set posts the task on TWO frames (delay+0 and delay+1) and the
                 * DSP demodulates the SECOND: measured by perturbing one raw sample,
                 * frame by frame, over fn 9..15 -- only fn=12 changes the soft bits read
                 * at fn=14/15, while [force] aimed at fn=11. The SCH was thus placed on
                 * frame 11 while the DSP demodulated frame 12, which carries a DUMMY
                 * burst (fixed pattern, 45.002 5.2.6), hence a CONSTANT input: frozen
                 * softs, identical range from frame to frame, and CRC OKs all returning
                 * the same word tens of frames apart. Stepping back one frame aligns the
                 * CONSUMED frame with the SCH. REJEU_SB_DECALAGE=<n> (default -1). */
                { static int d = -2; if (d == -2) { const char *e = calypso_getenv("REJEU_SB_DECALAGE");
                                                    d = e ? atoi(e) : -1; }
                  delay += d; if (delay < 0) delay = 0; }
                if (trace) printf("    [force] SB vise fn=%u (p51=%u), delay=%d\n", f, f % 51, delay);
            }
            plan_sb_set(delay);
        } else plan_fb_set(1, 1);
    }
}

static void sbdet_cmd(int attempt)
{
    g_sb_cmd_fn = fn_cur;
    if (trace) { unsigned p = fn_cur % 51;
        printf("  SBcmd att=%d fn=%u p51=%u %s\n", attempt, fn_cur, p,
               (p % 10 == 1 && p <= 41) ? "<- trame SCH (bon)" : "<- PAS une trame SCH"); }
    dbw()[W_TASK_MD] = SB_DSP_TASK;
    api[NDB_FB_MODE] = 0;
    if (trace) printf("  [SBcmd W] page=%u W=%04x %04x %04x %04x %04x ..%04x %04x  NDB page=%04x fb_mode=%04x fb_det=%04x\n",
                      w_page, dbw()[0], dbw()[1], dbw()[2], dbw()[3], dbw()[4], dbw()[15], dbw()[16], api[NDB_PAGE], api[NDB_FB_MODE], api[NDB_FB_DET]);
}

static void sbdet_resp(int attempt)
{
    n_sb_try++;
    r_page_used = 1;
    if (trace) {
        /* SB demodulator internals (RE 9.4 and 9.19):
         * 0x2f06 = correlation peak index
         * 0x2bf8 = internal CRC flag
         * 0x2c72 = THE 78 SOFT BITS. 0x2a00 is the 296-word FB correlator buffer, so
         * reading it there yields only 0x0000 and 0xffff; the real softs at 0x2c72
         * have varied magnitudes. Their range is printed too. */
        {
            int16_t mn = 32767, mx = -32768, nnul = 0;
            for (int k = 0; k < 78; k++) {
                int16_t v = (int16_t)dsp->data[0x2c72 + k];
                if (v) nnul++;
                if (v < mn) mn = v;
                if (v > mx) mx = v;
            }
            /* THE FEED. Does the buffer the DSP demodulates really hold the
             * delivered samples, and at which shift? Compare the BSP drop word by word
             * with the last delivered burst and look for the best alignment: a maximum
             * away from 0 means the burst is shifted; low everywhere means it is not
             * our burst at all. */
            if (calypso_getenv("REJEU_FEED")) {
                uint16_t ad = calypso_bsp_get_daram_addr();
                int nl = g_livre_niq;
                int best = 0, bestn = -1, n0 = 0;
                for (int sh = -80; sh <= 80; sh++) {
                    int m = 0;
                    for (int k = 0; k < nl; k++) {
                        int j = k + sh; if (j < 0 || j >= 2048) continue;
                        if ((int16_t)dsp->data[(ad + j) & 0x3fff] == g_livre_iq[k]) m++;
                    }
                    if (sh == 0) n0 = m;
                    if (m > bestn) { bestn = m; best = sh; }
                }
                printf("    [feed] fn=%u (livre fn=%u type=%c) addr=0x%04x len=%u nl=%d"
                       "  identiques a shift0=%d/%d  meilleur shift=%d avec %d/%d\n",
                       fn_cur, g_livre_fn, g_livre_type ? g_livre_type : '?', ad,
                       calypso_bsp_get_daram_len(), nl, n0, nl, best, bestn, nl);
            }
            /* [2026-09-18] PER-POSITION ERROR PROFILE. An aggregate percentage hides
             * what discriminates: WHERE the errors fall. The 78 coded bits occupy
             * symbols [3..41] and [106..144], the midamble sits at [42..105]. Errors at
             * the EDGES = channel estimate right at the midamble and drifting away from
             * it (phase ramp / too short an estimate). Uniform errors = wrong equalizer.
             * ~0 errors under one polarity = good softs and a bug downstream. Compared
             * against the frame ACTUALLY demodulated (last SB command), not the current
             * frame. */
            /* [2026-09-19] FIRS reads its input at 0x2a8e..0x2a9a (zone 0x2a00) while
             * the BSP drops the burst at 0x0cce. Is that intermediate zone filled, and
             * does it carry our burst? */
            if (calypso_getenv("REJEU_ZONE2A")) {
                uint16_t ad = calypso_bsp_get_daram_addr();
                int nz0 = 0, nz2 = 0;
                for (int k = 0; k < 296; k++) {
                    if (dsp->data[(ad + k) & 0x3fff]) nz0++;
                    if (dsp->data[(0x2a00 + k) & 0x3fff]) nz2++;
                }
                printf("    [zone] depot 0x%04x : %d/296 non nuls | zone 0x2a00 : %d/296 non nuls\n",
                       ad, nz0, nz2);
                printf("      0x%04x : %04x %04x %04x %04x %04x %04x\n", ad,
                       dsp->data[ad&0x3fff], dsp->data[(ad+1)&0x3fff], dsp->data[(ad+2)&0x3fff],
                       dsp->data[(ad+3)&0x3fff], dsp->data[(ad+4)&0x3fff], dsp->data[(ad+5)&0x3fff]);
                printf("      0x2a8e : %04x %04x %04x %04x %04x %04x   (entree lue par FIRS)\n",
                       dsp->data[0x2a8e], dsp->data[0x2a8f], dsp->data[0x2a90],
                       dsp->data[0x2a91], dsp->data[0x2a92], dsp->data[0x2a93]);
            }
            if (calypso_getenv("REJEU_PROFIL")) {
                unsigned char att78[78];
                int ri = g_n_reels ? g_reel_pour_fn[g_sb_cmd_fn & 63] : -1;
                if (ri >= 0) memcpy(att78, g_reels_code[ri], 78);   /* real ground truth */
                else cellule_code_attendu(g_sb_cmd_fn, (uint8_t)g_bsic_injecte, att78);
                int acc[2] = {0, 0};
                char map[2][79];
                for (int pol = 0; pol < 2; pol++) {
                    for (int k = 0; k < 78; k++) {
                        int16_t v = (int16_t)dsp->data[0x2c72 + k];
                        int bit = pol ? (v > 0) : (v < 0);   /* both sign conventions */
                        int ok = (bit == (att78[k] & 1));
                        map[pol][k] = ok ? '.' : 'X';
                        acc[pol] += ok;
                    }
                    map[pol][78] = 0;
                }
                /* [2026-09-18] ALIGNMENT SWEEP. The peak moves from frame to frame
                 * (23, 20, 19...): if it places the read window, comparing at the
                 * canonical alignment compares good softs against a wrong framing, and
                 * "uncorrelated" would be a false conclusion. Rebuild the full expected
                 * burst (3 tail + 39 + 64 TSC + 39 + 3 tail) and find the shift s that
                 * maximizes agreement. An s reaching ~78/78 means the softs are GOOD and
                 * only the framing is wrong. */
                {
                    unsigned char burst[148];
                    memset(burst, 0, 3);
                    for (int k = 0; k < 39; k++) burst[3 + k] = att78[k];
                    for (int k = 0; k < 64; k++) burst[42 + k] = (unsigned char)cellule_train_sb(k);
                    for (int k = 0; k < 39; k++) burst[106 + k] = att78[39 + k];
                    memset(burst + 145, 0, 3);
                    int bs = 0, bp = 0, bv = -1;
                    for (int sh = -40; sh <= 40; sh++) {
                        for (int pol = 0; pol < 2; pol++) {
                            int m = 0, n = 0;
                            for (int k = 0; k < 78; k++) {
                                int pos = (k < 39 ? 3 + k : 106 + (k - 39)) + sh;
                                if (pos < 0 || pos >= 148) continue;
                                int16_t v = (int16_t)dsp->data[0x2c72 + k];
                                int bit = pol ? (v > 0) : (v < 0);
                                if (bit == (burst[pos] & 1)) m++;
                                n++;
                            }
                            if (n >= 60 && m > bv) { bv = m; bs = sh; bp = pol; }
                        }
                    }
                    printf("    [align] meilleur decalage=%+d polarite=%d -> %d/78\n", bs, bp, bv);
                }
                int best = acc[1] > acc[0] ? 1 : 0;
                printf("    [profil] fn_demod=%u (p51=%u) pic=%u : concordance pol0=%d/78 pol1=%d/78\n"
                       "      premiers39 |%.39s|\n"
                       "      derniers39 |%s|\n",
                       g_sb_cmd_fn, g_sb_cmd_fn % 51, dsp->data[0x2f06],
                       acc[0], acc[1], map[best], map[best] + 39);
            }
            if (calypso_getenv("REJEU_DUMP_SOUPLES")) {
                printf("    [souples] fn=%u pic=%u :", fn_cur, dsp->data[0x2f06]);
                for (int k = 0; k < 78; k++) printf(" %04x", dsp->data[0x2c72 + k]);
                printf("\n");
            }
            printf("    [sb-int] pic(2f06)=%u  crc(2bf8)=%u  souples(2c72)[0..5]=%04x %04x %04x %04x %04x %04x"
                   "  non nuls=%d/78  etendue=[%d..%d]\n",
                   dsp->data[0x2f06], dsp->data[0x2bf8],
                   dsp->data[0x2c72], dsp->data[0x2c73], dsp->data[0x2c74],
                   dsp->data[0x2c75], dsp->data[0x2c76], dsp->data[0x2c77], nnul, mn, mx);
        }
    }
    if (trace) { unsigned p = fn_cur % 51;
        printf("  SBresp att=%d fn=%u p51=%u crc=%s\n", attempt, fn_cur, p,
               (dbr()[R_SCH + 0] & (1 << B_SCH_CRC)) ? "FAUX" : "OK"); }
    if (dbr()[R_SCH + 0] & (1 << B_SCH_CRC)) {
        n_sb_crcfail++;
        if (attempt == sb_dernier_essai()) { sched_reset(); plan_fb_set(1, 0); }
        return;
    }
    sb_word = dbr()[R_SCH + 3] | ((uint32_t)dbr()[R_SCH + 4] << 16);
    sb_bsic = (sb_word >> 2) & 0x3f;
    unsigned t1 = ((sb_word >> 23) & 1) | ((sb_word >> 7) & 0x1fe) | ((sb_word << 9) & 0x600);
    unsigned t2 = (sb_word >> 18) & 0x1f;
    unsigned t3p = ((sb_word >> 24) & 1) | ((sb_word >> 15) & 6);
    unsigned t3 = t3p * 10 + 1;
    sb_fn = 51u * ((t3 - t2 + 26u) % 26u) + t3 + 26u * 51u * t1;
    /* A CRC OK is a decode only if it yields the INJECTED BSIC, a valid T3 (T3 <= 50
     * by construction) and a reconstructed FN equal to the current frame. Each pass
     * is qualified and the run continues instead of stopping on the first.
     * REJEU_ARRET_1ER=1 restores stopping.
     *
     * REJEU_SCH_PARTOUT=1 CORRUPTS THE REFERENCE TRUTH: T3' has three bits, so
     * a SCH emitted on p51=45 encodes T3'=4, which the decoder turns back into
     * T3=41 and sb_fn != fn_cur even for a PERFECT decode. FN can only be
     * validated without that flag. */
    int t3_ok = (t3 <= 50), bsic_ok = (sb_bsic == (unsigned)g_bsic_injecte);
    /* [2026-09-20] The SB word carries the frame number of the BURST that was
     * demodulated, i.e. the frame of the last SB command (g_sb_cmd_fn), not the
     * frame on which the ARM reads the result (fn_cur, 2-3 frames later). The
     * first genuine decodes (BSIC 7, FN 31 read at fn 34; FN 62 read at fn 64)
     * were being counted as false positives by the old fn_cur comparison. */
    int fn_ok = (sb_fn == g_sb_cmd_fn);
    n_crc_ok++;
    if (bsic_ok && t3_ok && fn_ok) n_sb_vraies++;
    printf("  SB%d fn=%u : sb=0x%08x BSIC=%d (injecte %d) T1=%u T2=%u T3=%u -> FN=%u (burst demodule fn=%u)  %s%s\n",
           attempt, fn_cur, sb_word, sb_bsic, g_bsic_injecte, t1, t2, t3, sb_fn, g_sb_cmd_fn,
           (bsic_ok && t3_ok && fn_ok) ? "** VRAIE **"
           : !bsic_ok ? "FAUX POSITIF (BSIC ne colle pas)"
           : !t3_ok   ? "FAUX POSITIF (T3 > 50, impossible)"
           :            "FAUX POSITIF (FN != trame du burst demodule)",
           cellule_sch_partout ? "  [FN non qualifiable sous SCH_PARTOUT]" : "");
    if (drapeau_env("REJEU_ARRET_1ER") || ((bsic_ok && t3_ok && fn_ok) && !drapeau_env("REJEU_CONTINUER"))) { verdict = 1; return; }
    if (bsic_ok && t3_ok && fn_ok) { sched_reset(); plan_fb_set(1, 0); return; }   /* REJEU_CONTINUER: restart the acquisition */
    /* [2026-09-18] An unqualified CRC OK (false positive) must reschedule, or the
     * bench freezes and the freeze reads as a result: the item queue drains, no
     * command is posted again, and the replay crosses the remaining thousands of
     * frames asking the DSP nothing. The summary was then IDENTICAL at 1200 and at
     * 12000 frames -- not because the DSP stalled, but because the replayed ARM had
     * gone silent. The CRC failure branch already restarted (plan_fb_set on attempt
     * 2); only the "CRC OK but false positive" exit was a dead end. */
    sched_reset(); plan_fb_set(1, 0);
}


/* ---- ARM-side DSP init: what the firmware's dsp_power_on() does ---------
 * Without it the DSP runs on a blank API RAM: the FB detection thresholds
 * (d_fb_thr_det_iacq/track) and the margins (d_fb_margin_beg/end, which feed the
 * TOA formula at 0x794b/0x7956) are 0, and FB detection diverges from the first
 * frame. Values from dsp_params.c / dsp_ndb_init(). */
#define PARAM   0x431u          /* BASE_API_PARAM (API 0x862) in words */
#define A_SCH26 (NDB + 42u)
/* DSP bootloader: API offsets in WORDS (dsp.c: BASE_API_RAM + 0x0ff8..0x0ffe) */
#define BL_ADDR_HI_W  0x7FCu
#define BL_SIZE_W     0x7FDu
#define BL_ADDR_LO_W  0x7FEu
#define BL_STATUS_W   0x7FFu
#define DSP_START     0x7000u

static long pump(long max, int stop_on_idle)
{
    long b = 0;
    while (b < max && dsp->running) {
        int ex = c54x_run(dsp, 256);
        if (ex <= 0) break;
        b += ex;
        if (stop_on_idle && dsp->idle) break;
    }
    return b;
}

static void arm_dsp_init(void)
{
    memset(api, 0, 0x2000u * sizeof(uint16_t));          /* dsp_api_memset(API) */

    /* dsp_pre_boot(): the DSP has just been reset; wait for BL_STATUS_IDLE */
    long b = 0;
    while (api[BL_STATUS_W] != 1 && b < 8000000) {
        int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex;
    }
    printf("  bootloader DSP : BL_STATUS=%u apres %ld insn\n", api[BL_STATUS_W], b);

    /* dsp_set_params(): NDB first */
    api[NDB + 8]  = 0x0074;  api[NDB + 9]  = 0x0001;
    api[NDB + 10] = 0x0154;  api[NDB + 11] = 0x17ff;
    api[NDB + 12] = 7;       api[NDB + 13] = 0;
    api[NDB + 14] = 3;                                  /* d_dsp_state = IDLE3 */
    /* then the parameter table (dsp_params.c). The first four FB fields feed the
     * TOA formula (0x794b/0x7956) and the detection thresholds. */
    static const int16_t P[] = {
        0x6666, 15, 12, 5, 4, 0x7002, 1, 0xE, 0, 0, 0, 0,
        24, 22, 296, 30,                                 /* margin_beg/end, nsubb_idle/dedic */
        0x3333, (int16_t)0x28f6,                         /* fb_thr_det_iacq / _track */
        0x7fff, 17408, 26624, 20152,
        7872, -4, 7872, 5772, 7872, 53, -892, 208,
    };
    for (unsigned i = 0; i < sizeof P / sizeof P[0]; i++) api[PARAM + i] = (uint16_t)P[i];

    /* dsp_bl_start_at(DSP_START) */
    api[BL_ADDR_HI_W] = 0; api[BL_ADDR_LO_W] = DSP_START; api[BL_SIZE_W] = 0;
    api[BL_STATUS_W]  = 2;                               /* BL_CMD_COPY_BLOCK */
    b = pump(4000000, 1);
    printf("  DSP demarre : %ld insn, idle=%d, version=0x%04x%04x\n",
           b, dsp->idle, api[NDB + 6], api[NDB + 7]);

    /* dsp_ndb_init(): what matters for FB/SB */
    api[NDB + 2]  = 0x0179;                              /* d_spcx_rif */
    api[NDB + 3]  = 0x0800 | ((8 - 4) << 7);             /* d_tch_mode */
    api[NDB_FB_MODE] = 1;
    api[NDB_FB_DET]  = 0;
    api[A_SCH26]     = (1u << B_SCH_CRC);
    /* dsp_db_init() */
    memset(&api[W_PAGE(0)], 0, W_SIZE * sizeof(uint16_t));
    memset(&api[W_PAGE(1)], 0, W_SIZE * sizeof(uint16_t));
    memset(&api[R_PAGE(0)], 0, R_SIZE * sizeof(uint16_t));
    memset(&api[R_PAGE(1)], 0, R_SIZE * sizeof(uint16_t));
    /* [2026-09-20] REJEU_DUMP_PARAM=1: the first eight parameter words after
     * the DSP has started, to compare with the live bench's shared API RAM
     * (od on /dev/shm/calypso_api_ram at word 0x431). */
    if (drapeau_env("REJEU_DUMP_PARAM"))
        printf("  PARAM apres demarrage : %04x %04x %04x %04x %04x %04x %04x %04x  (attendu 6666 000f 000c 0005 0004 7002 0001 000e)\n",
               api[PARAM], api[PARAM+1], api[PARAM+2], api[PARAM+3], api[PARAM+4], api[PARAM+5], api[PARAM+6], api[PARAM+7]);
}

/* ---- one frame: l1_sync() then the DSP --------------------------------- */
static void l1_sync(void)
{
    r_page_used = 0;
    memset(dbw(), 0, W_SIZE * sizeof(uint16_t));          /* memset db_w */
    dbw()[W_AFC] = (uint16_t)(int16_t)afc_dac;            /* afc_load_dsp */
    dbw()[W_CTRL_ABB] |= (1u << B_AFC);
    /* [2026-09-17] CLOSED AFC LOOP. qemu-src relays the d_afc write to the TWL3025
     * model (calypso_trx.c, offsets 0x001E/0x0046); qosmo does it NOWHERE
     * (set_afc_dac is never called there), so the loop stayed open, sample rotation
     * never moved, and the frequency error never converged below the SB threshold
     * (800 Hz). */
    calypso_twl3025_set_afc_dac((int16_t)afc_dac);
    if (api[NDB_ERRSTAT]) {                                /* as in sync.c:249 */
        static int n; if (trace && n < 6) { printf("  DSP Error Status: %u\n", api[NDB_ERRSTAT]); n++; }
        api[NDB_ERRSTAT] = 0;
    }
    /* Run the items of THIS frame. A callback may schedule a new one for the
     * CURRENT frame (delay=0, which is what the firmware computes after FB1), so
     * rescan until exhaustion or that item is silently lost -- and it is precisely
     * the SB command aiming at the right SCH frame. */
    for (int tour = 0; tour < 8; tour++) {
        int fait = 0;
        for (int i = 0; i < n_sched; i++) {
            if (sched[i].frame == (int)fn_cur && sched[i].cb) {
                cb_t cb = sched[i].cb; int at = sched[i].attempt;
                sched[i].cb = NULL;
                cb(at); fait = 1;
                if (verdict) return;
            }
        }
        if (!fait) break;
    }
    if (r_page_used) {
        memset(dbr(), 0, R_SIZE * sizeof(uint16_t));
        dbr()[R_SCH + 0] = (1u << B_SCH_CRC);
        r_page ^= 1;
    }
    api[NDB_PAGE] = (uint16_t)(B_GSM_TASK | w_page);      /* dsp_end_scenario() */
    w_page ^= 1;
}

int rejouer(C54xState *d, uint16_t *api_ram, long trames, long insns,
            const char *iq_mode, int amp, int bsic, int verbeux)
{
    dsp = d; api = api_ram; trace = verbeux;
    env_div = calypso_getenv("REJEU_DIV") != NULL;
    env_softs_continu = calypso_getenv("REJEU_SOFTS_CONTINU") != NULL;
    env_firs = calypso_getenv("REJEU_FIRS") != NULL;
    env_probe_t = drapeau_env("REJEU_PROBE_T");
    env_decodeur = calypso_getenv("REJEU_DECODEUR") != NULL;
    w_page = r_page = r_page_used = 0; fn_cur = 0; afc_dac = -700;
    fb_mode = 0; afc_retries = fb0_retries = 0; n_sched = 0; verdict = 0;
    n_fb_ok = n_sb_try = n_sb_crcfail = n_crc_ok = n_sb_vraies = 0;
    n_err_dsp = n_err8 = 0;
    g_bsic_injecte = bsic;
    { const char *r = calypso_getenv("REJEU_SCH_REEL");
      if (r && *r && reels_charger(r) == 0) g_bsic_injecte = g_reels_bsic[0]; }
    memset(&fb, 0, sizeof fb);

    if (drapeau_env("REJEU_SCH_PARTOUT")) { cellule_sch_partout = 1; printf("  [stimulus] SCH sur toutes les trames non-FCCH (masque le cadencage)\n"
               "  [stimulus] ATTENTION : t3p = p51/10 n'est juste que sur p51 in {1,11,21,31,41},\n"
               "             donc la FN reconstruite ne peut PAS etre validee sous ce drapeau.\n"); }
    printf("rejeu deterministe : %ld trames max, %ld insn/trame, cellule BSIC=%d, iq=%s\n",
           trames, insns, bsic, iq_mode ? iq_mode : "cell");
    /* firmware dsp_power_on(): bootloader boot + parameters + NDB */
    arm_dsp_init();
    { static int16_t rempl[2 * 148];         /* TS1..TS7 of the C0 carrier: dummy bursts */
      cellule_factice(amp, DECALAGE_SYMB, rempl);
      calypso_bsp_set_remplissage(rempl, 2 * 148); }
    plan_fb_set(1, 0);                       /* first FBSB_REQ */

    int16_t iq[2 * 256]; int n_iq;
    for (long t = 0; t < trames && !verdict; t++) {
        fn_cur = (uint32_t)t;
        g_c54x_exe_fn = fn_cur;
        uint32_t insn_debut_trame = dsp->insn_count;
        l1_sync();
        /* [2026-09-18] The firmware (sync.c) reads d_error_status every frame, prints
         * it and clears it; replay ignored it entirely, hence its silence about the
         * "DSP Error Status: 8" seen on the firmware side. 8 = DSP_ERR_DMA_PROG:
         * overflow of the DMA job ring at 0x4330 (orm *(0x3f92),#8 at 0xaa83). */
        if (api[NDB_ERRSTAT]) {
            unsigned e = api[NDB_ERRSTAT];
            n_err_dsp++;
            if (e & 8) n_err8++;
            if (n_err_dsp <= 8)
                printf("  [erreur DSP] status=%u%s a fn=%u\n", e,
                       (e & 8) ? " (bit 3 = DSP_ERR_DMA_PROG, anneau DMA 0x4330 sature)" : "",
                       fn_cur);
            api[NDB_ERRSTAT] = 0;
        }
        if (verdict) break;
        /* [2026-09-17] HARDWARE ORDER. On silicon: the ARM posts the task -> the DSP
         * reads it on the frame interrupt and ARMS its RX window (DMA) -> the samples
         * arrive -> the DSP processes them. Injecting BEFORE the DSP has armed makes
         * the transfer use the PREVIOUS task's DMA programming (the FB one), so the SB
         * receives the wrong burst. REJEU_RX_AVANT=1 restores the old order for
         * comparison. */
        static int rx_avant = -1;
        if (rx_avant < 0) rx_avant = drapeau_env("REJEU_RX_AVANT") ? 1 : 0;
        n_iq = 2 * 148;
        int injecter = (!iq_mode || strcmp(iq_mode, "none") != 0);
        if (injecter && rx_avant) {
            g_livre_type = cellule_burst(fn_cur, (uint8_t)g_bsic_injecte, amp, DECALAGE_SYMB, marge_tete(), iq, &n_iq);
            g_livre_fn = fn_cur; g_livre_n = n_iq;
            g_ad_avant = calypso_bsp_get_daram_addr();
            { static int da = -1; static unsigned nd;
              if (da < 0) da = calypso_getenv("REJEU_ADR") ? 1 : 0;
              if (da && nd < 14) { nd++;
                  printf("  [adr] fn=%-4u type=%c n_iq=%-4d -> depot 0x%04x len=%u\n",
                         fn_cur, g_livre_type ? g_livre_type : '?', n_iq,
                         calypso_bsp_get_daram_addr(), calypso_bsp_get_daram_len()); } }
            calypso_bsp_rx_burst(0, fn_cur, iq, n_iq);
        }
        /* frame interrupt: the DSP reads the task and arms its RX window.
         * [2026-09-20] REJEU_IRQ_SCENARIO=1: raise it only on frames where the
         * replayed ARM ended a DSP scenario (a task on the page just handed
         * over), as dsp_end_scenario() does with tpu_dsp_frameirq_enable(),
         * a bit the firmware sets again on every scenario. Every other frame
         * the ROM gets no frame interrupt and does not re-read the page. */
        { static int irq_sc = -1;
          if (irq_sc < 0) irq_sc = drapeau_env("REJEU_IRQ_SCENARIO") ? 1 : 0;
          unsigned wp_donnee = (api[NDB_PAGE] & 1u);
          bool scenario = api[W_PAGE(wp_donnee) + W_TASK_MD] != 0 || api[W_PAGE(wp_donnee) + 0] != 0;
          if ((dsp->imr & (1u << 12)) && (!irq_sc || scenario)) c54x_interrupt_ex(dsp, 28, 12); }
        if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
        if (injecter && !rx_avant) {
            /* let the DSP arm (short budget), THEN deliver the samples */
            long arm = 0;
            while (arm < insns / 4 && dsp->running && !dsp->idle) {
                int ex = c54x_run(dsp, 64); if (ex <= 0) break; arm += ex;
            }
            g_livre_type = cellule_burst(fn_cur, (uint8_t)g_bsic_injecte, amp, DECALAGE_SYMB, marge_tete(), iq, &n_iq);
            /* [2026-09-20] FULL FRAME for the FB search. On silicon the FB task
             * streams 156.25 symbols per TDMA frame (148 burst + 8.25 guard); the
             * TOA it reports counts frames in those units and the firmware turns
             * it back into frames with BITS_PER_TDMA = 1250. Delivering 148
             * samples per frame made a 9-frame distance read as 5. Pad every
             * non-SCH burst with silence to 156 samples (157 on one frame in four,
             * so the average is 156.25). REJEU_TRAME_PLEINE=0 restores 148. */
            /* The SCH frame is delivered as the 190-sample WINDOW block (margins
             * 21/21) only while the DSP has its one-shot SB window armed; inside
             * an FB search (continuous DMA) it is a plain frame of the stream like
             * any other, or the stream would gain 34 symbols on every SCH frame.
             * The TOA origin (the firmware's "23") is not set here but by the DMA
             * model at arm time (CALYPSO_RHEA_DMA_ARM_SKIP): the burst position
             * inside the frame can only move within the 8.25 idle symbols. */
            { static int pleine = -1;
              if (pleine < 0) { const char *e = calypso_getenv("REJEU_TRAME_PLEINE"); pleine = (e && *e == '0') ? 0 : 1; }
              bool fenetre_sb = calypso_rhea_dma_one_shot();
              if (pleine && !(g_livre_type == 'S' && fenetre_sb)) {
                  if (g_livre_type == 'S') {          /* drop the window margins: burst only */
                      int m = marge_tete();
                      memmove(iq, iq + 2 * m, 2 * 148 * sizeof(int16_t));
                      n_iq = 2 * 148;
                  }
                  int cible = 156 + ((fn_cur & 3) == 3 ? 1 : 0);
                  if (n_iq < 2 * cible) { memset(iq + n_iq, 0, (size_t)(2 * cible - n_iq) * sizeof(int16_t)); n_iq = 2 * cible; }
              } }
            g_livre_fn = fn_cur; g_livre_n = n_iq;
            /* Replace the SCH CONTENT with a real burst, without touching the framing. */
            if (g_n_reels && g_livre_type == 'S') {
                int m = marge_tete();
                unsigned r = g_reel_i % g_n_reels;
                { static double gn = -1;
                  if (gn < 0) { const char *e = calypso_getenv("REJEU_REEL_GAIN"); gn = e ? atof(e) : 1.0;
                                if (gn <= 0) gn = 1.0; }
                  if (gn == 1.0) memcpy(iq + 2 * m, g_reels[r], 296 * sizeof(int16_t));
                  else for (int q = 0; q < 296; q++) {
                      long v = lrint(g_reels[r][q] * gn);
                      iq[2 * m + q] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v); } }
                /* Check: is the burst dropped really the REAL one? */
                static unsigned nv;
                if (nv < 4) { nv++;
                    printf("  [reel] trame %u <- burst #%u (fn reelle %u, BSIC %u) I/Q %d %d %d %d\n",
                           fn_cur, r, g_reels_fn[r], g_reels_bsic[r],
                           iq[2*m], iq[2*m+1], iq[2*m+2], iq[2*m+3]); }
                g_reel_pour_fn[fn_cur & 63] = (int)r;
                g_reel_i++;
            }
            /* [2026-09-18] Perturb ONE raw sample rather than a bit: this maps
             * influence index by index, with no confusion from burst packing nor from
             * the scheduling change a coded-bit flip causes. REJEU_PERTURBER_ECH=<n>
             * adds a delta to complex sample n of the delivered buffer. */
            { static int pe = -2; static long pf = -2;
              if (pe == -2) { const char *e = calypso_getenv("REJEU_PERTURBER_ECH"); pe = e ? atoi(e) : -1; }
              if (pf == -2) { const char *e = calypso_getenv("REJEU_PERTURBER_FN");  pf = e ? atol(e) : -1; }
              if (pe >= 0 && 2 * pe + 1 < n_iq && (pf < 0 || (long)fn_cur == pf)) {
                  iq[2 * pe]     = (int16_t)(iq[2 * pe]     + 3000);
                  iq[2 * pe + 1] = (int16_t)(iq[2 * pe + 1] - 3000);
              } }
            /* [2026-09-18] DELIVERED SAMPLE CONVENTION. Transport is exact ([feed]
             * probe: 380/380 identical at shift 0), so if the feed is at fault it is the
             * CONVENTION, not the routing. Four mutually exclusive hypotheses:
             *   derot-  : the DSP expects a signal DEROTATED by -pi/2 per symbol
             *   derot+  : ... by +pi/2
             *   swap    : I and Q swapped
             *   conj    : Q negated (conjugate)
             * REJEU_FEED_XFORM=derot-|derot+|swap|conj|none (default none). */
            { static const char *xf = NULL; static int init = 0;
              if (!init) { xf = calypso_getenv("REJEU_FEED_XFORM"); init = 1; }
              if (xf && *xf && strcmp(xf, "none")) {
                  int ns = n_iq / 2;
                  if (!strcmp(xf, "swap")) {
                      for (int k = 0; k < ns; k++) { int16_t t = iq[2*k]; iq[2*k] = iq[2*k+1]; iq[2*k+1] = t; }
                  } else if (!strcmp(xf, "conj")) {
                      for (int k = 0; k < ns; k++) iq[2*k+1] = (int16_t)(-iq[2*k+1]);
                  } else if (!strcmp(xf, "derot-") || !strcmp(xf, "derot+")) {
                      double sgn = (xf[5] == '-') ? -1.0 : 1.0;
                      for (int k = 0; k < ns; k++) {
                          double ph = sgn * (M_PI / 2.0) * k;
                          double c = cos(ph), sn = sin(ph);
                          double i0 = iq[2*k], q0 = iq[2*k+1];
                          iq[2*k]   = (int16_t)lrint(i0 * c - q0 * sn);
                          iq[2*k+1] = (int16_t)lrint(i0 * sn + q0 * c);
                      }
                  }
              } }
            memcpy(g_livre_iq, iq, (size_t)n_iq * sizeof(int16_t)); g_livre_niq = n_iq;
            { static int da = -1; static unsigned nd;
              if (da < 0) da = calypso_getenv("REJEU_ADR") ? 1 : 0;
              if (da && nd < 14) { nd++;
                  printf("  [adr] fn=%-4u type=%c n_iq=%-4d -> depot 0x%04x len=%u\n",
                         fn_cur, g_livre_type ? g_livre_type : '?', n_iq,
                         calypso_bsp_get_daram_addr(), calypso_bsp_get_daram_len()); } }
            /* [2026-09-20] PAGE-BY-PAGE DELIVERY (REJEU_PAGES=<words>, e.g. 96).
             * On silicon the RIF streams continuously and DMA2 completes one
             * 96-word page at a time, each completion interrupting the DSP; the
             * FB correlator runs per page. Handing the whole frame in one call
             * drains 3-4 pages in one pass with ONE interrupt, so the ROM
             * processes one page per frame and its TOA advanced by 96 per frame
             * instead of 156 symbols. Here the frame's samples are delivered in
             * chunks, the DSP running a slice after each, so every page gets its
             * own completion. */
            { static long pages_mots = -1;
              if (pages_mots < 0) { const char *e = calypso_getenv("REJEU_PAGES"); pages_mots = (e && *e) ? atol(e) : 0; }
              if (pages_mots > 0) {
                  int pos = 0, npage = 0;
                  while (pos < n_iq) {
                      int m = n_iq - pos < pages_mots ? n_iq - pos : (int)pages_mots;
                      uint16_t b34 = dsp->data[0x3fb4], b35 = dsp->data[0x3fb5];
                      calypso_bsp_rx_burst(0, fn_cur, iq + pos, m);
                      pos += m; npage++;
                      { static unsigned nb; if (nb < 40 && fn_cur >= 2 && fn_cur <= 14) { nb++;
                          printf("  [bloc] fn=%u page %d (%d mots) : 0x3fb4=%04x 0x3fb5=%04x avant", fn_cur, npage, m, b34, b35); } }
                      /* completion interrupt already pending: let the DSP serve it */
                      if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
                      long slice = 0;
                      while (slice < insns / 8 && dsp->running && !dsp->idle) {
                          int ex = c54x_run(dsp, 64); if (ex <= 0) break; slice += ex;
                      }
                      { static unsigned nb2; if (nb2 < 40 && fn_cur >= 2 && fn_cur <= 14) { nb2++;
                          printf(" -> apres 0x3fb4=%04x 0x3fb5=%04x (idle=%d, %ld insn)\n", dsp->data[0x3fb4], dsp->data[0x3fb5], dsp->idle, slice); } }
                  }
                  { static unsigned np; if (np < 3) { np++;
                      printf("  [pages] fn=%u : %d mots livres en %d pages de %ld\n", fn_cur, n_iq, npage, pages_mots); } }
              } else
                  calypso_bsp_rx_burst(0, fn_cur, iq, n_iq);
            }
            /* [2026-09-20] STREAM PUMP. The DMA now fills its double buffer with
             * full pages only and interrupts once per pair; the ROM's ISR consumes
             * both halves and the DSP goes idle. Then the next pair is handed
             * over, as the hardware would once the ISR is out of the way. A frame
             * carries 3.25 pages, so this runs 1 or 2 times per frame. */
            for (int k = 0; k < 40; k++) {          /* a 1250-symbol frame is 26 pages = 13 pairs */
                if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
                long sl = 0;
                while (sl < insns / 4 && dsp->running && !dsp->idle) {
                    int ex = c54x_run(dsp, 64); if (ex <= 0) break; sl += ex;
                }
                if (!calypso_rhea_dma_pump(dsp)) break;
            }
            if (calypso_getenv("CALYPSO_BSP_VERIF")) {
                static int dit;
                if (!dit) { dit = 1;
                    printf("  [mem] api est-il un alias de data[0x0800] ? %s\n",
                           (void *)api == (void *)&dsp->data[0x0800] ? "OUI" : "NON — deux memoires distinctes");
                    printf("        api[0x0cce-0x0800]=%04x   data[0x0cce]=%04x\n",
                           api[0x0cce - 0x0800], dsp->data[0x0cce]); }
                static unsigned nvr;
                uint32_t vfn; uint16_t vad; int vn, vage;
                int id = calypso_bsp_verif_compare(&vfn, &vad, &vn, &vage);
                if (id >= 0 && nvr < 8) { nvr++;
                    printf("  [ref] fn=%-4u type=%c age=%d : %d/%d identiques en 0x%04x%s\n",
                           vfn, g_livre_type ? g_livre_type : '?', vage, id, vn, vad,
                           id == vn ? "   VALIDE" : "   <<< ECRITURE FAUSSE"); }
            }
            { static int vv = -1; static unsigned nv;
              if (vv < 0) vv = calypso_getenv("REJEU_VIE") ? 1 : 0;
              if (vv && g_livre_type == 'S' && nv < 6) { nv++;
                  uint16_t ad = calypso_bsp_get_daram_addr();
                  printf("  [vie] fn=%-4u adresse APRES l'appel : 0x%04x (avant : 0x%04x)\n",
                         fn_cur, ad, g_ad_avant);
                  int ex = 0, best = 0, bex = -1;
                  for (int sh = -8; sh <= 44; sh++) {
                      int e = 0;
                      for (int k = 0; k < 280; k++) {
                          int j = k + sh; if (j < 0 || j >= n_iq) continue;
                          if ((int16_t)dsp->data[(ad + k) & 0x3fff] == iq[j]) e++;
                      }
                      if (e > bex) { bex = e; best = sh; }
                      if (sh == 0) ex = e;
                  }
                  printf("  [vie] fn=%-4u APRES depot en 0x%04x : shift0=%d/280  meilleur shift=%+d avec %d/280\n",
                         fn_cur, ad, ex, best, bex);
                  if (bex < 200) {   /* pas trouve la : ou est le burst ? */
                      int ba = -1, bn = 0;
                      for (unsigned a = 0; a + 280 < 0x4000; a++) {
                          int e = 0;
                          for (int k = 0; k < 64; k++)
                              if ((int16_t)dsp->data[a + k] == iq[k]) e++;
                          if (e > bn) { bn = e; ba = (int)a; }
                      }
                      if (bn >= 60) {
                          int tot = 0;
                          for (int k = 0; k < 280; k++)
                              if ((int16_t)dsp->data[(ba + k) & 0x3fff] == iq[k]) tot++;
                          printf("        -> burst TROUVE en 0x%04x : %d/280 identiques\n", ba, tot);
                      } else printf("        -> burst introuvable en DARAM (meilleur %d/64 en 0x%04x)\n", bn, ba);
                  }
                  g_vie_fn = fn_cur; g_vie_ad = ad; } }
            if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
        }
        { static int tf = -1; if (tf < 0) tf = calypso_getenv("REJEU_TRACE_FB") ? 1 : 0;
          if (tf && fn_cur < 60)
              printf("  [fb] fn=%-3u apres pompe : 0x3fb4=%04x 0x3fb3=%04x d_fb_det=%u fb_mode=%u task_md=%u/%u idle=%d insn_trame=%u  sync=%04x %04x %04x %04x\n",
                     fn_cur, dsp->data[0x3fb4], dsp->data[0x3fb3], api[NDB_FB_DET], api[NDB_FB_MODE],
                     api[W_PAGE(0) + W_TASK_MD], api[W_PAGE(1) + W_TASK_MD], dsp->idle, dsp->insn_count - insn_debut_trame,
                     api[NDB_SYNC], api[NDB_SYNC+1], api[NDB_SYNC+2], api[NDB_SYNC+3]); }
        long done = 0;
        static int probe = -1;
        if (probe < 0) probe = drapeau_env("REJEU_PROBE_TOA") ? 1 : 0;
        if (!probe) {
            /* Always count, step by step: running 256 instructions at a time without
             * looking at a single PC left hit_b219/7c31/84a1/9841/770a at zero while the
             * summary still printed them, so "job b219=0 ... corr FB 770a=0" only meant
             * that nobody was counting. The PC histogram says what the DSP really
             * executes. */
            while (done < insns && dsp->running && !dsp->idle) {
                uint16_t pc = dsp->pc & 0xffff;
                histo_pc[pc >> 10]++;
                switch (pc) {
                case 0x7c31: hit_7c31++; break;
                case 0x9841: hit_9841++; break;
                case 0x84a1: hit_84a1++; break;
                case 0x770a: hit_770a++; break;
                case 0xb219: hit_b219++; break;
                case 0x7a16: hit_7a16++; break;
                /* The two FIRS sites of the SB. The one at 0x8493 is followed by
                 * `sth *AR6+,B` at 0x8497: THAT is what writes the soft bits. If it does
                 * not run, the softs come out of a stale B. */
                case 0x8478: hit_8478++; break;
                case 0x8492: hit_8492++; break;
                case 0x8493: hit_8493++; break;
                case 0x8497: hit_8497++; break;
                /* [2026-09-19] THE 16-STEP DIVISION. 0x7d1c RPT #15; 0x7d1d SUBC
                 * *(0x0b),A; 0x7d1e STL A,*(0x0a) = the quotient. A null divisor gives a
                 * degenerate quotient and the whole cascade follows. */
                /* [2026-09-19] The dividend is born here. It should be 1 shifted
                 * (ld #1,A; sfta A,<n>) and it arrives NULL at the division. Trace A over
                 * the whole prologue to see which instruction zeroes it. */
                case 0x7d10: case 0x7d11: case 0x7d12: case 0x7d13:
                case 0x7d14: case 0x7d15: case 0x7d16: case 0x7d17:
                case 0x7d18: case 0x7d19: case 0x7d1a: case 0x7d1b:
                    if (env_div) {
                        static int dumpe;
                        if (!dumpe) { dumpe = 1;
                            printf("    [rom] 0x7d10-0x7d20 :");
                            for (unsigned a = 0x7d10; a <= 0x7d20; a++)
                                printf(" %04x", prog_ovly(dsp, (uint16_t)a));
                            printf("\n"); }
                        static unsigned np;
                        if (np < 26) { np++;
                            printf("    [pro] pc=%04x op=%04x  A=%010llx  T=%04x ST0=%04x ST1=%04x\n",
                                   pc, prog_ovly(dsp, pc),
                                   (unsigned long long)(dsp->a & 0xffffffffffULL),
                                   dsp->t, dsp->st0, dsp->st1); }
                    }
                    break;
                case 0x7d1c: hit_7d1c++;
                    if (env_div) {
                        unsigned dp = dsp->st0 & 0x1FF;
                        uint16_t dv = dsp->data[(uint16_t)((dp << 7) | 0x0B)];
                        static unsigned nd;
                        if (nd < 12) { nd++;
                            printf("    [div] avant : dividende A=%010llx  diviseur=0x%04x (%d)%s\n",
                                   (unsigned long long)(dsp->a & 0xffffffffffULL), dv, (int16_t)dv,
                                   dv == 0 ? "   <<< DIVISEUR NUL" : ""); }
                    }
                    break;
                case 0x7d1d: hit_7d1d++; break;
                case 0x7d1e: hit_7d1e++;
                    if (env_div) {
                        static unsigned nq;
                        if (nq < 12) { nq++;
                            printf("    [div] apres : quotient A=%010llx  (mot bas = %d)\n",
                                   (unsigned long long)(dsp->a & 0xffffffffffULL),
                                   (int16_t)(dsp->a & 0xffff)); }
                    }
                    break;
                case 0x81e4: hit_81e4++; break;
                default: break; }
                /* [2026-09-19] WHO WRITES a_sch? The status word a_sch[0] sometimes
                 * receives plain numbers (0x1111, 0x1388=5000, 0x142e) where only B_BLUD
                 * (bit15) and B_SCH_CRC (bit8) have a meaning -- and the 0x8000 read as
                 * "CRC OK" is more likely a saturated accumulator. a_sch[0..4] =
                 * R_PAGE+15.., i.e. data[0x0837..0x083b] (page 0) and
                 * data[0x084b..0x084f] (page 1). Record the PC of each write.
                 * REJEU_QUI_ASCH=1. */
                { static int qa = -1; static uint16_t sh[10]; static int ini; static unsigned nqa;
                  if (qa < 0) qa = calypso_getenv("REJEU_QUI_ASCH") ? 1 : 0;
                  if (qa) {
                      static const uint16_t adr[10] = {0x0837,0x0838,0x0839,0x083a,0x083b,
                                                       0x084b,0x084c,0x084d,0x084e,0x084f};
                      if (!ini) { for (int k=0;k<10;k++) sh[k]=dsp->data[adr[k]]; ini=1; }
                      for (int k=0;k<10;k++) {
                          uint16_t v = dsp->data[adr[k]];
                          if (v != sh[k]) {
                              /* On a CRC OK, print the FULL word: eight events
                               * carrying the same word are not eight independent
                               * draws but one attractor reached eight times, which
                               * changes the statistics completely. */
                              if (k % 5 == 0 && v == 0x8000) {
                                  const uint16_t *b = &dsp->data[adr[k]];
                                  printf("    [crcok] fn=%u  a_sch = %04x %04x %04x %04x %04x"
                                         "  -> mot 0x%04x%04x  crc_interne(2bf8)=%u\n",
                                         fn_cur, b[0], b[1], b[2], b[3], b[4],
                                         b[4], b[3], dsp->data[0x2bf8]);
                              }
                              if (nqa < 4000) { nqa++;
                                  int j = k % 5;
                                  printf("    [asch] a_sch[%d] (page %d, 0x%04x) : %04x -> %04x"
                                         "   ecrit juste avant pc=%04x  fn=%u%s\n",
                                         j, k/5, adr[k], sh[k], v, pc, fn_cur,
                                         (j==0 && v && v!=0x0100 && v!=0x8000 && v!=0x8100)
                                           ? "   <<< PAS UN DRAPEAU" : ""); }
                              sh[k] = v;
                          }
                      }
                  } }
                /* [2026-09-19] WHICH PC PINS A AT +-0x40000000 (A_high = +-16384)?
                 * Everything else follows from it: the clipping at 0x2ac0, B zeroed by
                 * the ADD at 0x8389, the degenerate softs. Record the PC of each
                 * transition to that value. REJEU_QUI_A=1. */
                { static int qA = -1; static int64_t aprec; static unsigned long parpc[0x10000];
                  static unsigned long tA; static int armA;
                  if (qA < 0) qA = calypso_getenv("REJEU_QUI_A") ? 1 : 0;
                  if (qA) {
                      int64_t a40 = dsp->a & 0xffffffffffLL;
                      int pin = (a40 == 0x0040000000LL || a40 == 0xffc0000000LL);
                      int pin0 = (aprec == 0x0040000000LL || aprec == 0xffc0000000LL);
                      static uint16_t pcp;
                      if (pin && !pin0 && armA) parpc[pcp]++;
                      aprec = a40; pcp = pc; armA = 1;
                      if (++tA % 400000 == 0) {
                          printf("  [quiA] PC qui amenent A a +-0x40000000 :\n");
                          for (int rang=0; rang<8; rang++) {
                              unsigned best=0; unsigned long bv=0;
                              for (unsigned i=0;i<0x10000;i++) if (parpc[i]>bv){bv=parpc[i];best=i;}
                              if (!bv) break;
                              printf("    pc=%04x op=%04x : %lu fois\n", best,
                                     prog_ovly(dsp,(uint16_t)best), bv);
                              parpc[best]=0;
                          }
                          tA = 1;
                      }
                  } }
                /* [2026-09-19] WHO WRITES THE +-16384 INTO 0x2ac0? Watch the buffer
                 * word by word and record the PC of each write, separating clipped
                 * values from the rest. REJEU_QUI2AC0=1. */
                { static int qc = -1; static uint16_t sh2[64]; static int ini3;
                  static unsigned long par_pc_butee[0x10000], par_pc_autre[0x10000];
                  static unsigned long tot2;
                  if (qc < 0) qc = calypso_getenv("REJEU_QUI2AC0") ? 1 : 0;
                  if (qc) {
                      if (!ini3) { for (int k=0;k<64;k++) sh2[k]=dsp->data[0x2ac0+k]; ini3=1; }
                      for (int k=0;k<64;k++) {
                          uint16_t v = dsp->data[0x2ac0+k];
                          if (v != sh2[k]) {
                              int16_t sv = (int16_t)v;
                              if (sv == 16384 || sv == -16384) par_pc_butee[pc]++;
                              else par_pc_autre[pc]++;
                              sh2[k] = v;
                          }
                      }
                      if (++tot2 % 300000 == 0) {
                          printf("  [2ac0] PC ecrivains (butee +-16384 | autres) :\n");
                          for (int rang=0; rang<8; rang++) {
                              unsigned best=0; unsigned long bv=0;
                              for (unsigned i=0;i<0x10000;i++)
                                  if (par_pc_butee[i]+par_pc_autre[i] > bv) { bv=par_pc_butee[i]+par_pc_autre[i]; best=i; }
                              if (!bv) break;
                              printf("    pc=%04x op=%04x : butee=%lu  autres=%lu\n",
                                     best, prog_ovly(dsp,(uint16_t)best),
                                     par_pc_butee[best], par_pc_autre[best]);
                              par_pc_butee[best]=0; par_pc_autre[best]=0;
                          }
                          tot2 = 1;
                      }
                  } }
                /* [2026-09-19] DOES THE WORKING BUFFER HOLD THE BURST? The injected
                 * samples are known (g_livre_iq); after the copy, 0x2a00 and 0x2ac0
                 * should carry the I and Q channels. Compare instead of assuming.
                 * REJEU_CMP2A=1: at the first pass at 0x84a0 (correlator entry, so the
                 * copy is done) on an SCH frame. */
                { static int cm = -1; static int fait2;
                  if (cm < 0) cm = calypso_getenv("REJEU_CMP2A") ? 1 : 0;
                  if (cm && fait2 < 3 && pc == 0x84a0 && g_n_reels &&
                      g_reel_pour_fn[g_sb_cmd_fn & 63] >= 0) {
                      fait2++;
                      int ri = g_reel_pour_fn[g_sb_cmd_fn & 63];
                      const int16_t *bi = g_reels[ri];
                      int m = 0;
                      printf("  [cmp] burst reel #%d (fn %u, BSIC %u) vs tampons de travail\n", ri, g_reels_fn[ri], g_reels_bsic[ri]);
                      printf("    I injecte : "); for (int k=0;k<10;k++) printf(" %6d", bi[2*(m+k)]);
                      printf("\n    0x2a00    : "); for (int k=0;k<10;k++) printf(" %6d", (int16_t)dsp->data[0x2a00+k]);
                      printf("\n    0x2ac0    : "); for (int k=0;k<10;k++) printf(" %6d", (int16_t)dsp->data[0x2ac0+k]);
                      printf("\n    Q injecte : "); for (int k=0;k<10;k++) printf(" %6d", bi[2*(m+k)+1]);
                      printf("\n");
                      /* normalized correlation between each buffer and each channel */
                      /* how far the constancy goes: how many words are identical? */
                      for (int buf=0; buf<2; buf++) {
                          uint16_t base = buf ? 0x2ac0 : 0x2a00;
                          int16_t v0 = (int16_t)dsp->data[base];
                          int same = 0, n = 0; int16_t mn=32767, mx=-32768;
                          for (int k=0;k<190;k++) { int16_t v=(int16_t)dsp->data[base+k];
                              if (v==v0) same++; if(v<mn)mn=v; if(v>mx)mx=v; n++; }
                          printf("    0x%04x : %d/%d mots egaux au premier (%d) ; etendue [%d..%d]\n",
                                 base, same, n, v0, mn, mx);
                      }
                      for (int buf=0; buf<2; buf++) {
                          uint16_t base = buf ? 0x2ac0 : 0x2a00;
                          for (int voie=0; voie<2; voie++) {
                              double sxy=0, sxx=0, syy=0;
                              for (int k=0;k<128;k++) {
                                  double x=(int16_t)dsp->data[base+k], y=bi[2*(m+k)+voie];
                                  sxy+=x*y; sxx+=x*x; syy+=y*y; }
                              printf("    correlation 0x%04x vs %c : %+.3f\n", base, voie?'Q':'I',
                                     (sxx>0&&syy>0)? sxy/sqrt(sxx*syy) : 0.0);
                          }
                      }
                  } }
                /* [2026-09-19] THE burst -> working buffer COPY, the last link never
                 * examined: the burst arrives exact at 0x0cce and buffer 0x2a80 is
                 * filled by 0x81d0..0x81da. What does that loop READ? */
                { static int rc = -1; static unsigned n;
                  if (rc < 0) rc = calypso_getenv("REJEU_RECOPIE") ? 1 : 0;
                  if (rc && pc >= 0x81c8 && pc <= 0x81e0) {
                      /* Does AR5 sweep a table, or stay on two cells? */
                      static unsigned lo = 0xffff, hi = 0, vus[64], nv;
                      if (dsp->ar[5] < lo) lo = dsp->ar[5];
                      if (dsp->ar[5] > hi) hi = dsp->ar[5];
                      { int trouve = 0; for (unsigned q = 0; q < nv; q++) if (vus[q] == dsp->ar[5]) trouve = 1;
                        if (!trouve && nv < 64) vus[nv++] = dsp->ar[5]; }
                      /* Does the multiplicand change? A phasor kept in place would
                       * change value at every step; a constant would not. */
                      { static unsigned long nval; static uint16_t vu_v[32]; static unsigned nvv;
                        uint16_t v = dsp->data[dsp->ar[5] & 0x3fff];
                        int t2 = 0; for (unsigned q = 0; q < nvv; q++) if (vu_v[q] == v) t2 = 1;
                        if (!t2 && nvv < 32) vu_v[nvv++] = v;
                        if (++nval % 4000 == 0) {
                            printf("    [mul] %u valeurs distinctes lues via AR5 :", nvv);
                            for (unsigned q = 0; q < nvv && q < 12; q++) printf(" %04x", vu_v[q]);
                            printf("\n"); } }
                      static unsigned long tot;
                      if (++tot % 4000 == 0)
                          printf("    [ar5] apres %lu pas : plage 0x%04x..0x%04x, %u cellules distinctes\n",
                                 tot, lo, hi, nv);
                  }
                  if (rc && n < 16 && pc >= 0x81c8 && pc <= 0x81e0) {
                      n++;
                      printf("    [cp] pc=%04x op=%04x  A=%010llx B=%010llx"
                             "  AR2=%04x->%04x AR3=%04x->%04x AR4=%04x->%04x AR5=%04x->%04x\n",
                             pc, prog_ovly(dsp, pc),
                             (unsigned long long)(dsp->a & 0xffffffffffULL),
                             (unsigned long long)(dsp->b & 0xffffffffffULL),
                             dsp->ar[2], dsp->data[dsp->ar[2] & 0x3fff],
                             dsp->ar[3], dsp->data[dsp->ar[3] & 0x3fff],
                             dsp->ar[4], dsp->data[dsp->ar[4] & 0x3fff],
                             dsp->ar[5], dsp->data[dsp->ar[5] & 0x3fff]);
                  } }
                /* [2026-09-19] THE BLOCK THAT OVERWRITES THE LOWER HALF OF THE FIRS
                 * WINDOW. 0x832b..0x8330 writes 0000 x4 then 4000 x2 there. Boundary
                 * conditions, or clobbering? Print the program words of the area and the
                 * registers on entry, once. REJEU_DUMP832=1. */
                { static int d8 = -1; static int fait;
                  if (d8 < 0) d8 = calypso_getenv("REJEU_DUMP832") ? 1 : 0;
                  if (d8 && !fait && pc == 0x8320) {
                      fait = 1;
                      printf("  [832] programme 0x8318-0x8340 (alias OVLY compris) :\n");
                      for (unsigned a = 0x8318; a <= 0x8340; a += 8) {
                          printf("    %04x:", a);
                          for (int k = 0; k < 8 && a + k <= 0x8340; k++)
                              printf(" %04x", prog_ovly(dsp, (uint16_t)(a + k)));
                          printf("\n");
                      }
                      printf("    registres : A=%010llx B=%010llx T=%04x\n",
                             (unsigned long long)(dsp->a & 0xffffffffffULL),
                             (unsigned long long)(dsp->b & 0xffffffffffULL), dsp->t);
                      printf("    AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x\n",
                             dsp->ar[0],dsp->ar[1],dsp->ar[2],dsp->ar[3],
                             dsp->ar[4],dsp->ar[5],dsp->ar[6],dsp->ar[7]);
                  } }
                /* [2026-09-19] THE CORRELATOR BLOCK, PC BY PC, OVER A SINGLE DECODE.
                 * 3100 MACs per decode reads either as "50 x 62" or "31 x 100"; only a
                 * per-pass count decides. Count each PC in 0x84a0..0x84d0 during the
                 * FIRST decode, then print. REJEU_BLOC=1. */
                { static int bl = -1; static unsigned long cnt[0x40]; static int fini, vu_dec;
                  if (bl < 0) bl = calypso_getenv("REJEU_BLOC") ? 1 : 0;
                  if (bl && !fini) {
                      if (pc >= 0x84a0 && pc <= 0x84df) cnt[pc - 0x84a0]++;
                      if (pc == 0x9841) {
                          if (!vu_dec) { vu_dec = 1; }
                          else {
                              fini = 1;
                              printf("  [bloc] correlateur 0x84a0-0x84df sur UN decodage :\n");
                              for (int k = 0; k < 0x40; k++)
                                  if (cnt[k]) printf("    pc=%04x  op=%04x  %6lu fois\n",
                                                     0x84a0 + k, prog_ovly(dsp, 0x84a0 + k), cnt[k]);
                          }
                      }
                  } }
                /* [2026-09-19] IMPULSE RESPONSE OF 0x2c72. Force ONE position to an
                 * extreme value, held over the whole window, and compare the output
                 * between +v and -v. How many decoded bits move:
                 *   1 position       -> a permutation, 0x2c72 really is one soft per bit
                 *   3 or 4 adjacent  -> normal ISI, the chain is sound here
                 *   all / none       -> these are not per-bit softs
                 * REJEU_IMPULSION=<k> REJEU_IMPULSION_VAL=<v>. */
                { static int ik = -2, iv;
                  if (ik == -2) { const char *e = calypso_getenv("REJEU_IMPULSION");
                                  ik = e ? atoi(e) : -1;
                                  const char *w = calypso_getenv("REJEU_IMPULSION_VAL");
                                  iv = w ? atoi(w) : 20000; }
                  if (ik >= 0 && ik < 78 && g_dans_sb)
                      dsp->data[0x2c72 + ik] = (uint16_t)(int16_t)iv; }
                /* [2026-09-19] IS THE PEAK USED AT ALL? The addresses FIRS reads do
                 * not move when the burst moves, while the peak does follow. Force the
                 * peak to an arbitrary value over the whole demodulation window: if
                 * nothing downstream changes, it is not consumed and the equalizer is
                 * aligned on nothing. REJEU_FORCER_PIC=<n>. */
                { static int fp = -2;
                  if (fp == -2) { const char *e = calypso_getenv("REJEU_FORCER_PIC"); fp = e ? atoi(e) : -1; }
                  if (fp >= 0 && g_dans_sb) dsp->data[0x2f06] = (uint16_t)fp; }
                /* [2026-09-19] PERFECT SOFTS. The fault lies somewhere between the
                 * correlator (sound) and the soft-bit write. Split the space in two:
                 * just before the decoder (0x9841) reads 0x2c72, write the ideal soft
                 * bits there ourselves, derived from the 78 bits the injected burst
                 * REALLY carries. If the DSP then returns BSIC=32, everything downstream
                 * (Viterbi, CRC, a_sch packing) is proven and the fault is strictly in
                 * soft PRODUCTION; otherwise it is downstream.
                 * REJEU_SOFTS_PARFAITS=<amplitude>, sign tested both ways via
                 * REJEU_SOFTS_POLARITE=0|1. REJEU_SOFTS_CONTINU=1 holds the ideal softs
                 * at EVERY step of the demodulation window, not only at the decoder
                 * entry -- otherwise a read before 0x9841 escapes the injection and "no
                 * effect" means nothing. */
                if (pc == 0x9841 || (env_softs_continu && g_dans_sb)) {
                    static int amp = -2, pol = -1;
                    if (amp == -2) { const char *e = calypso_getenv("REJEU_SOFTS_PARFAITS");
                                     amp = e ? atoi(e) : -1;
                                     const char *q = calypso_getenv("REJEU_SOFTS_POLARITE");
                                     pol = q ? atoi(q) : 0; }
                    if (amp > 0) {
                        int ri = g_n_reels ? g_reel_pour_fn[g_sb_cmd_fn & 63] : -1;
                        unsigned char att[78];
                        if (ri >= 0) memcpy(att, g_reels_code[ri], 78);
                        else cellule_code_attendu(g_sb_cmd_fn, (uint8_t)g_bsic_injecte, att);
                        /* REJEU_SOFTS_PERM: the order of the 78 values is a hypothesis.
                         * none = as is; swap = the two halves of 39 exchanged;
                         * rev = reversed order; entrelace = even/odd separated. */
                        static const char *perm; static int perm_lu;
                        if (!perm_lu) { perm = calypso_getenv("REJEU_SOFTS_PERM"); perm_lu = 1; }
                        /* [2026-09-20] The decoder reads its 78 softs at 0x2a00
                         * (ROM 0x984a `stm #0x2a00,AR1`; packing 0x7e65-0x7e7a),
                         * NOT at 0x2c72. Writing 0x2c72 tested nothing.
                         * REJEU_SOFTS_ADDR overrides (default 0x2a00). */
                        static long sa = -1;
                        if (sa < 0) { const char *e = calypso_getenv("REJEU_SOFTS_ADDR");
                                      sa = (e && *e) ? strtol(e, NULL, 0) : 0x2a00; }
                        for (int k = 0; k < 78; k++) {
                            int j = k;
                            if (perm && !strcmp(perm, "swap"))       j = (k < 39) ? k + 39 : k - 39;
                            else if (perm && !strcmp(perm, "rev"))   j = 77 - k;
                            else if (perm && !strcmp(perm, "entrelace")) j = (k < 39) ? 2*k : 2*(k-39)+1;
                            int bit = att[j] & 1;
                            int v = (pol ? bit : !bit) ? amp : -amp;
                            dsp->data[(uint16_t)(sa + k)] = (uint16_t)(int16_t)v;
                        }
                        static unsigned ns; if (ns < 3) { ns++;
                            printf("  [softs] fn_demod=%u : 78 souples IDEAUX ecrits en 0x%04lx "
                                   "(amp=%d pol=%d, source=%s)\n", g_sb_cmd_fn, sa, amp, pol,
                                   ri >= 0 ? "burst reel" : "fixture"); }
                    }
                }
                /* [2026-09-19] WHO WRITES THE EQUALIZER INPUT WINDOW? FIRS reads
                 * 0x2a8e..0x2a9a and its lower half is zero or 0x4000. Watch the area
                 * word by word and record the PC of each write: that says who fills it,
                 * in what order, and where the filling stops. REJEU_QUI2A=1. */
                { static int q2 = -1; static uint16_t shadow[0x30]; static int init2;
                  static unsigned nq;
                  if (q2 < 0) q2 = calypso_getenv("REJEU_QUI2A") ? 1 : 0;
                  if (q2) {
                      if (!init2) { for (int k = 0; k < 0x30; k++) shadow[k] = dsp->data[0x2a80 + k]; init2 = 1; }
                      for (int k = 0; k < 0x30; k++) {
                          uint16_t v = dsp->data[0x2a80 + k];
                          if (v != shadow[k]) {
                              if (nq < 4000) { nq++;
                                  printf("    [qui] 0x%04x : %04x -> %04x   ecrit juste avant pc=%04x (fn=%u)\n",
                                         0x2a80 + k, shadow[k], v, pc, fn_cur); }
                              shadow[k] = v;
                          }
                      }
                  } }
                /* [2026-09-19] WHO FILLS B? The correlator is sound (the peak follows
                 * the margin), FIRS carries nothing, and yet `sth *AR6+,B` at 0x8497 is
                 * what writes the soft bits. Trace A and B over the whole window
                 * 0x8470..0x84a0 to see where B takes its value. REJEU_TRACE_B=1. */
                /* 0x847c / 0x8498 : op=0x4485 = LD Smem,16,A per tic54x-opc.c
                 * (0x4400/0xFE00). No handler matches this mask in c54x_exec.c,
                 * yet A changes across it. Dump A before/after and every AR with
                 * the word it points at, to find where the value comes from. */
                /* Dump the DSP work buffers at the correlator entry, together with
                 * the real burst that was fed, so an external model can identify what
                 * each buffer holds instead of guessing. REJEU_DUMP_BUF=<path>. */
                /* Who writes the BSP deposit window 0x0cce, and when? At correlator
                 * entry the window holds none of the injected bursts and its content
                 * does not change between frames. Shadow the window, log each change
                 * with the PC that made it. REJEU_QUI_CCE=1. */
                /* Timeline of the deposit window on one SB frame: every change with
                 * the PC and the instruction count, plus the correlator entry. Tells
                 * whether the burst is destroyed before or after the SB reads it.
                 * REJEU_CHRONO=<fn>. */
                { static long cf = -2; static uint16_t sh2[380]; static int ini2; static unsigned nl;
                  if (cf == -2) { const char *e = calypso_getenv("REJEU_CHRONO"); cf = e ? atol(e) : -1; }
                  if (cf >= 0 && (long)fn_cur == cf) {
                      if (!ini2) { for (int k=0;k<380;k++) sh2[k]=dsp->data[0x0cce + k]; ini2=1;
                                   printf("  [chrono] trame %ld\n", cf); }
                      unsigned chg=0;
                      for (int k=0;k<380;k++) { uint16_t v=dsp->data[0x0cce + k];
                          if (v!=sh2[k]) { chg++; sh2[k]=v; } }
                      if (chg && nl < 24) { nl++;
                          printf("      insn=%-8u pc=%04x  %4u mots changes\n",
                                 dsp->insn_count, pc, chg); }
                      if (pc == 0x84a0 && nl < 30) { nl++;
                          printf("      insn=%-8u pc=84a0  <== ENTREE DU CORRELATEUR SB\n",
                                 dsp->insn_count); }
                      if (pc == 0x7c31 && nl < 30) { nl++;
                          printf("      insn=%-8u pc=7c31  <== entree du demodulateur SB\n",
                                 dsp->insn_count); }
                  } }
                { static int qc = -1; static uint16_t sh[380]; static int ini;
                  static unsigned long par_pc[0x10000], tot; static unsigned long nfr[64];
                  if (qc < 0) qc = calypso_getenv("REJEU_QUI_CCE") ? 1 : 0;
                  if (qc) {
                      if (!ini) { for (int k=0;k<380;k++) sh[k]=dsp->data[0x0cce + k]; ini=1; }
                      unsigned chg = 0;
                      for (int k=0;k<380;k++) {
                          uint16_t v = dsp->data[0x0cce + k];
                          if (v != sh[k]) { chg++; sh[k]=v; }
                      }
                      if (chg) { par_pc[pc] += chg; nfr[fn_cur & 63] += chg; }
                      if (++tot % 500000 == 0) {
                          printf("  [cce] ecrivains de 0x0cce (mots modifies) :\n");
                          for (int r=0;r<6;r++) {
                              unsigned best=0; unsigned long bv=0;
                              for (unsigned i=0;i<0x10000;i++) if (par_pc[i]>bv){bv=par_pc[i];best=i;}
                              if (!bv) break;
                              printf("      pc=%04x op=%04x : %lu mots\n", best,
                                     prog_ovly(dsp,(uint16_t)best), bv);
                              par_pc[best]=0;
                          }
                          tot=1;
                      }
                  } }
                { static int vv2 = -1; static unsigned nv2;
                  if (vv2 < 0) vv2 = calypso_getenv("REJEU_VIE") ? 1 : 0;
                  if (vv2 && pc == 0x84a0 && g_vie_ad && nv2 < 5) { nv2++;
                      int ex = 0;
                      for (int k = 0; k < 296; k++)
                          if ((int16_t)dsp->data[(g_vie_ad + k) & 0x3fff] == g_livre_iq[2*marge_tete() + k]) ex++;
                      printf("  [vie] fn=%-4u ENTREE CORRELATEUR (depot de fn=%u) : %d/296 identiques\n",
                             fn_cur, g_vie_fn, ex); } }
                { static int db = -1; static FILE *fb;
                  if (db < 0) { const char *e = calypso_getenv("REJEU_DUMP_BUF");
                                db = e ? 1 : 0; if (db) fb = fopen(e, "wb"); }
                  if (db && fb && pc == 0x84a0 && g_n_reels) {
                      int ri = g_reel_pour_fn[g_sb_cmd_fn & 63];
                      if (ri >= 0) {
                          static int nb;
                          if (nb < 8) { nb++;
                              uint32_t hdr[4] = { g_sb_cmd_fn, (uint32_t)ri,
                                                  g_reels_fn[ri], g_reels_bsic[ri] };
                              fwrite(hdr, 4, 4, fb);
                              fwrite(g_reels[ri], 2, 296, fb);          /* le burst injecte */
                              fwrite(&dsp->data[0x0cce], 2, 380, fb);   /* depot BSP */
                              fwrite(&dsp->data[0x2a00], 2, 256, fb);   /* tampon 1 */
                              fwrite(&dsp->data[0x2ac0], 2, 256, fb);   /* tampon 2 */
                              fwrite(&dsp->data[0x2c72], 2, 78, fb);    /* bits souples */
                              fwrite(g_reels_code[ri], 1, 78, fb);      /* bits emis */
                              fflush(fb);
                          }
                      }
                  } }
                { static int q4 = -1; static unsigned n4; static int64_t avant; static int arme;
                  if (q4 < 0) q4 = calypso_getenv("REJEU_Q4485") ? 1 : 0;
                  if (q4) {
                      if (arme) { arme = 0;
                          printf("        -> A apres = %010llx\n",
                                 (unsigned long long)(dsp->a & 0xffffffffffULL)); }
                      if ((pc == 0x847c || pc == 0x8498) && n4 < 6) {
                          n4++; avant = dsp->a; arme = 1;
                          printf("    [4485] pc=%04x op=%04x  A avant = %010llx\n",
                                 pc, prog_ovly(dsp, pc), (unsigned long long)(avant & 0xffffffffffULL));
                          for (int k = 0; k < 8; k++)
                              printf("        AR%d=%04x -> %04x\n", k, dsp->ar[k],
                                     dsp->data[dsp->ar[k] & 0x3fff]);
                      }
                  } }
                { static int tb = -1; static unsigned ntb;
                  if (tb < 0) tb = calypso_getenv("REJEU_TRACE_B") ? 1 : 0;
                  if (tb && ntb < 70 && pc >= 0x8470 && pc <= 0x84a0) {
                      ntb++;
                      printf("    [B] pc=%04x op=%04x A=%010llx B=%010llx AR2=%04x->%04x AR3=%04x->%04x\n",
                             pc, prog_ovly(dsp, pc),
                             (unsigned long long)(dsp->a & 0xffffffffffULL),
                             (unsigned long long)(dsp->b & 0xffffffffffULL),
                             dsp->ar[2], dsp->data[dsp->ar[2] & 0x3fff],
                             dsp->ar[3], dsp->data[dsp->ar[3] & 0x3fff]);
                  } }
                { uint16_t o = prog_ovly(dsp, pc); uint8_t h = o >> 8;
                  static int dans_sb2;
                  if (pc == 0x7c31) dans_sb2 = 1;
                  if (pc == 0x9841) dans_sb2 = 0;
                  g_dans_sb = dans_sb2;
                  /* [2026-09-18] WHAT FIRS MULTIPLIES. FIRS (0xE0, 2 words) takes its
                   * pmad from the following word and reads its coefficients in PROGRAM
                   * space. Print pmad, the coefficients as the core sees them (OVLY alias
                   * included) and data[] at the same address: null or constant
                   * coefficients mean the equalizer output CANNOT depend on its input,
                   * which would be the root cause. */
                  if (env_firs && dans_sb2 && h == 0xE0) {
                      static unsigned nf;
                      if (nf < 10) {
                          uint16_t pmad = prog_ovly(dsp, (uint16_t)(pc + 1));
                          printf("    [firs] #%u pc=%04x pmad=%04x  coef(prog+ovly)=", ++nf, pc, pmad);
                          for (int k = 0; k < 6; k++) printf(" %04x", prog_ovly(dsp, (uint16_t)(pmad + k)));
                          printf("   data[pmad..]=");
                          for (int k = 0; k < 6; k++) printf(" %04x", dsp->data[(pmad + k) & 0x3fff]);
                          printf("\n");
                      }
                  }
                  if (pc >= 0x9800 && pc <= 0x9bff) g_op_dec[o]++;
                  if (pc >= 0x8400 && pc <= 0x84ff) g_op_eq[o]++;
                  if (pc == 0x9841) g_n_dec++;
                  if (dans_sb2) {
                      if (g_op_n[o]++ == 0) g_op_pc[o] = pc;
                      if (h == 0x8E || h == 0x8F) n_cmps_sb++;
                      if (h >= 0xE0 && h <= 0xE3) { n_e0_sb++; n_e0x_sb[h - 0xE0]++; }
                      /* [2026-09-18] FIRS reads its coefficients at Pmem[pmad], with
                       * pmad ~ 0x0061. But 0x0060-0x007F is the C54x DARAM SCRATCH-PAD
                       * and the core's OVLY window only starts at 0x0080, so
                       * prog_read(0x61) falls back on prog[] where nothing is loaded
                       * below 0x7000. If the coefficients only live in data[], FIRS
                       * multiplies by nothing and its output cannot depend on its input.
                       * Read BOTH spaces at the FIRS. */
                      if (h == 0xE0 && n_firs_vus < 6) {
                          uint16_t pmad = dsp->prog[(pc + 1) & 0xffff];
                          int nzp = 0, nzd = 0;
                          for (int k = 0; k < 6; k++) {
                              if (dsp->prog[(pmad + k) & 0xffff]) nzp++;
                              if (dsp->data[(pmad + k) & 0x3fff]) nzd++;
                          }
                          /* [2026-09-18] Reading prog[] RAW is misleading: with
                           * PMST_OVLY set and the alias floor at 0x0060 (gate
                           * CALYPSO_OVLY_SCRATCH, default 1), a PROGRAM read in
                           * 0x0060-0x27FF is redirected to data[]. Reproduce the core's
                           * translation to know what FIRS REALLY reads, rather than what
                           * the prog[] array holds. */
                          int ovly = (dsp->pmst & 0x0020) != 0;
                          int alias = ovly && pmad >= 0x0060 && pmad < 0x2800;
                          printf("    [firs] pc=%04x pmad=%04x PMST=%04x OVLY=%d alias=%s"
                                 " -> FIRS lit", pc, pmad, dsp->pmst, ovly,
                                 alias ? "data[] (scratch-pad visible)" : "prog[] (PAS d'alias)");
                          for (int k = 0; k < 6; k++)
                              printf(" %04x", alias ? dsp->data[(pmad + k) & 0x3fff]
                                                    : dsp->prog[(pmad + k) & 0xffff]);
                          printf("\n");
                          printf("    [firs] pc=%04x pmad=%04x | prog[pmad..+5]=", pc, pmad);
                          for (int k = 0; k < 6; k++) printf(" %04x", dsp->prog[(pmad + k) & 0xffff]);
                          printf(" (%d non nuls)\n                      | data[pmad..+5]=", nzp);
                          for (int k = 0; k < 6; k++) printf(" %04x", dsp->data[(pmad + k) & 0x3fff]);
                          printf(" (%d non nuls)\n", nzd);
                          n_firs_vus++;
                      }
                  } else if (h == 0x8E || h == 0x8F) n_cmps_hors++;
                  /* [2026-09-18] The dans_sb window closes at 0x9841, so it EXCLUDES
                   * the SCH decoder and its Viterbi (0x9a78): "0 CMPS in the SB demod"
                   * said nothing about the Viterbi. CMPS is therefore also counted per PC
                   * region, with no window, which is unambiguous. */
                  if (h == 0x8E || h == 0x8F) n_cmps_reg[pc >> 10]++;
                  /* [2026-09-18] WHO WRITES 0x2a00? The decoder reads 78 words at
                   * 0x2a00 and finds them ALL NULL on entry: either the equalization
                   * stage never writes them, or something erases them first. Watch
                   * 0x2a00..0x2a8d over the WHOLE SB job (not just the demod), recording
                   * the PC of each change and its direction (to a value, or to zero). */
                /* [2026-09-18] 0x8389 = 0x4594 = `ADD *AR4+,16,A,B` (0x4400/0xFC00,
                 * bit9=src, bit8=dst). The core has NO handler for 0x4400-0x47FF: the
                 * only one in the area is (op & 0xFC00) == 0x4000, which covers SUB
                 * alone. Check that B really changes across that instruction. */
                { static int nav; static int64_t bavant;
                  if (pc == 0x8389) { bavant = dsp->b; nav = 1; }
                  else if (nav == 1 && pc == 0x838a) {
                      static int n=0;
                      if (n < 4) {
                          printf("    [add-4594] B avant=%010llx  B apres=%010llx  A=%010llx  %s\n",
                                 (unsigned long long)(bavant & 0xffffffffffULL),
                                 (unsigned long long)(dsp->b & 0xffffffffffULL),
                                 (unsigned long long)(dsp->a & 0xffffffffffULL),
                                 (dsp->b == bavant) ? "B INCHANGE => instruction NON EXECUTEE" : "B modifie");
                          n++;
                      }
                      nav = 0;
                  } }
                { static int nb838d;
                  if (pc == 0x838e && nb838d < 6) {
                      printf("    [B@838d] B=%010llx  A=%010llx  AR6=%04x  BRC=%u  ST1=%04x\n",
                             (unsigned long long)(dsp->b & 0xffffffffffULL),
                             (unsigned long long)(dsp->a & 0xffffffffffULL),
                             dsp->ar[6], dsp->brc, dsp->st1);
                      nb838d++;
                  } }
                { static uint16_t omb2[142]; static int arme6; static long ecr[64], vers0[64];
                  static uint16_t pmin[64], pmax[64];
                  static struct { uint16_t pc; long v, z; } parpc2[12];
                  if (pc == 0xb219) {
                      for (int k = 0; k < 142; k++) omb2[k] = dsp->data[0x2a00 + k];
                      arme6 = 1;
                      for (int b = 0; b < 64; b++) { ecr[b] = 0; vers0[b] = 0; }
                      for (int t = 0; t < 12; t++) { parpc2[t].pc = 0; parpc2[t].v = 0; parpc2[t].z = 0; }
                  }
                  if (arme6) {
                      for (int k = 0; k < 142; k++) {
                          if (dsp->data[0x2a00 + k] != omb2[k]) {
                              /* [2026-09-18] Bucket by EXACT PC, and separate value
                               * writes from zeroing: 284 changes of which 142 to zero
                               * means something writes the 142 soft bits then ERASES
                               * them all, and both instructions must be named. The PC
                               * printed is that of the FOLLOWING instruction. */
                              { int z = (dsp->data[0x2a00 + k] == 0);
                                for (int t = 0; t < 12; t++) {
                                    if (parpc2[t].pc == 0 || parpc2[t].pc == pc) {
                                        parpc2[t].pc = pc;
                                        if (z) parpc2[t].z++; else parpc2[t].v++;
                                        break;
                                    }
                                } }
                              int b = pc >> 10;
                              if (!ecr[b]) { pmin[b] = pc; pmax[b] = pc; }
                              if (pc < pmin[b]) pmin[b] = pc;
                              if (pc > pmax[b]) pmax[b] = pc;
                              ecr[b]++;
                              if (dsp->data[0x2a00 + k] == 0) vers0[b]++;
                              omb2[k] = dsp->data[0x2a00 + k];
                          }
                      }
                  }
                  if (pc == 0x9841 && arme6 && n_2a0 < 3) {
                      printf("    [qui-2a00] modifications de 0x2a00..0x2a8d pendant le job SB :\n");
                      long tot = 0;
                      for (int b = 0; b < 64; b++)
                          if (ecr[b]) {
                              printf("        region 0x%04x : %ld modifs (dont %ld vers zero), PC 0x%04x a 0x%04x\n",
                                     b << 10, ecr[b], vers0[b], pmin[b], pmax[b]);
                              tot += ecr[b];
                          }
                      if (!tot) printf("        AUCUNE : rien n'ecrit jamais dans 0x2a00\n");
                      printf("        par PC exact (PC imprime = instruction SUIVANTE) :\n");
                      for (int t = 0; t < 12; t++)
                          if (parpc2[t].pc)
                              printf("            pc=%04x (donc ecrivain %04x) : %ld valeurs, %ld mises a ZERO\n",
                                     parpc2[t].pc, parpc2[t].pc - 1, parpc2[t].v, parpc2[t].z);
                      n_2a0++;
                  } }
                /* [2026-09-18] WHO WRITES THE 78 SOFT BITS, AND AT WHICH ADDRESS?
                 * Sweeping the 78 coded bits shows the first half landing at position b+3
                 * and the second at TWO positions 51 apart (b-38 and b+13), i.e. the two
                 * data blocks OVERLAP instead of concatenating. Shadow the area and
                 * record the PC as soon as its content changes. */
                { static uint16_t ombre[78]; static int arme5; static long parpc[64];
                  static uint16_t pcmin[64], pcmax[64]; static long seq;
                  if (dans_sb2) {
                      if (!arme5) { for (int k = 0; k < 78; k++) ombre[k] = dsp->data[0x2c72 + k]; arme5 = 1; seq = 0; }
                      for (int k = 0; k < 78; k++) {
                          if (dsp->data[0x2c72 + k] != ombre[k]) {
                              int b = pc >> 10;
                              if (!parpc[b]) { pcmin[b] = pc; pcmax[b] = pc; }
                              if (pc < pcmin[b]) pcmin[b] = pc;
                              if (pc > pcmax[b]) pcmax[b] = pc;
                              parpc[b]++;
                              /* [2026-09-18] EXACT CHRONOLOGY: sequence number, PC,
                               * index touched, old and new value. Tells (a) in which
                               * ORDER the two passes write, hence which destroys which,
                               * and (b) whether two writes at the same index carry the
                               * SAME value (mis-addressed copy) or DIFFERENT ones (two
                               * halves of a sum that should have landed together). */
                              if (n_ecr == 0 && seq < 100)
                                  printf("    [w] %3ld pc=%04x k=%2d  %04x -> %04x  "
                                         "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x BRC=%u\n",
                                         seq, pc, k, ombre[k], dsp->data[0x2c72 + k],
                                         dsp->ar[0], dsp->ar[1], dsp->ar[2], dsp->ar[3],
                                         dsp->ar[4], dsp->ar[5], dsp->ar[6], dsp->ar[7], dsp->brc);
                              seq++;
                              ombre[k] = dsp->data[0x2c72 + k];
                          }
                      }
                  } else if (arme5 && n_ecr < 2) {
                      printf("    [ecrit-2c72] sites qui modifient les 78 bits souples :\n");
                      for (int b = 0; b < 64; b++)
                          if (parpc[b])
                              printf("        region 0x%04x : %ld ecritures, PC de 0x%04x a 0x%04x\n",
                                     b << 10, parpc[b], pcmin[b], pcmax[b]);
                      n_ecr++; arme5 = 0;
                      for (int b = 0; b < 64; b++) parpc[b] = 0;
                  } }
                /* [2026-09-18] WHY THE EDGES ARE LOST. The max|delta| profile shows
                 * three orders of magnitude between the middle of the burst (13452,
                 * 19534, extreme values REPEATED) and its edges (2 to 30). Identical
                 * extremes coming back means SATURATION. In a fixed-point MLSE the path
                 * metrics MUST be renormalized at every step; without that they grow,
                 * saturate, and the relative contribution of edge symbols collapses.
                 * Count accumulator saturations and flag state during the SB demod. */
                { static long nsat, nova, novb; static int ovm_vu;
                  if (dans_sb2) {
                      int64_t a = dsp->a, b = dsp->b;
                      if (a > 0x7FFFFFFFLL || a < -0x80000000LL) nsat++;
                      if (b > 0x7FFFFFFFLL || b < -0x80000000LL) nsat++;
                      if (dsp->st0 & 0x0400) nova++;      /* OVA */
                      if (dsp->st0 & 0x0200) novb++;      /* OVB */
                      if (dsp->st1 & 0x0200) ovm_vu = 1;  /* OVM: saturation mode */
                  } else if ((nsat || nova || novb) && n_sat_vus < 3) {
                      printf("    [saturation] demod SB : %ld depassements 32 bits,"
                             " OVA pose %ld fois, OVB %ld fois, mode OVM %s\n",
                             nsat, nova, novb, ovm_vu ? "ACTIF" : "inactif");
                      n_sat_vus++; nsat = nova = novb = 0;
                  } }
                /* [2026-09-18] WHERE DOES THE FIXED WINDOW ADDRESS COME FROM? Watch
                 * the address registers that SWEEP the input buffer 0x0cce during the SB
                 * demod and print, per register, the sample index range covered and the
                 * PC that set it. If that range does NOT move by 10 when the margin goes
                 * from 21 to 31, the address is computed once and reused as is. BK is
                 * printed too: a mis-emulated circular block size would loop the access
                 * over a fixed window of the right length with a correct base. */
                { static uint16_t amin[8], amax[8], apc[8]; static long an[8];
                  static int arme4;
                  if (dans_sb2) {
                      if (!arme4) { for (int k = 0; k < 8; k++) { amin[k] = 0xffff; amax[k] = 0; an[k] = 0; } arme4 = 1; }
                      for (int k = 0; k < 8; k++) {
                          uint16_t v = dsp->ar[k];
                          if (v >= 0x0cce && v < 0x0cce + 380) {
                              uint16_t idx = (uint16_t)((v - 0x0cce) / 2);
                              if (idx < amin[k]) { amin[k] = idx; apc[k] = pc; }
                              if (idx > amax[k]) amax[k] = idx;
                              an[k]++;
                          }
                      }
                  } else if (arme4 && n_bal < 3) {
                      printf("    [tampon-AR] parcours de 0x0cce pendant le demod SB (BK=%u) :\n", dsp->bk);
                      for (int k = 0; k < 8; k++)
                          if (an[k])
                              printf("        AR%d : echantillons %u..%u (%u larges), %ld acces, 1er a pc=%04x\n",
                                     k, amin[k], amax[k], amax[k] - amin[k] + 1, an[k], apc[k]);
                      n_bal++; arme4 = 0;
                  } }
                /* [2026-09-18] XC PIPELINE HAZARD. On the C54x the XC condition is
                   * sampled two cycles before execution, so an instruction setting the
                   * flag just before the XC is not yet visible; the core evaluates at
                   * execution time. The hazard therefore only bites if the ROM places the
                   * flag setter within two slots of the XC (code written for real silicon
                   * normally does not). Measure the real DISTANCE instead of assuming it:
                   * keep the last 4 PCs and the ST0 before each. */
                  { static uint16_t ring_pc[4], ring_op[4], ring_st0[4]; static int ri;
                    if ((h == 0xFD || h == 0xFF) && pc >= 0x8400 && pc < 0x8800 && n_xc_vus < 10) {
                        printf("    [xc] pc=%04x op=%04x cc=%02x | 3 precedentes :", pc, o, o & 0xff);
                        for (int k = 3; k >= 1; k--) {
                            int q = (ri - k) & 3;
                            printf("  %04x:%04x(ST0=%04x)", ring_pc[q], ring_op[q], ring_st0[q]);
                        }
                        printf(" | ST0 au XC=%04x\n", dsp->st0);
                        n_xc_vus++;
                    }
                    ring_pc[ri] = pc; ring_op[ri] = o; ring_st0[ri] = dsp->st0;
                    ri = (ri + 1) & 3;
                    if (h == 0xFD || h == 0xFF) n_xc_reg[pc >> 10]++; } }
                /* [2026-09-20] DECODER ORACLE (REJEU_DECODEUR=1). Ideal softs at
                 * 0x2a00 still give no CRC OK, so the fault is inside 0x9841..: ACS
                 * (0x9a78), traceback (0x9aaf) or CRC (0x9887). Dump each stage's
                 * output next to what libosmocoding says it should be: u[0..34] =
                 * 25 info bits of sb_info + 10 parity bits (gsm0503_sch_crc10). */
                if (env_decodeur) {
                    static unsigned nd;
                    { static unsigned nt;
                      if (pc == 0x9a7f && nt < 6) { nt++;
                          printf("  [dec] apres `sth @0x0e,B` (pc=9a7e) : T=%04x  B=%010llx (hi=%04x)  DP=%03x CPL=%d  ST0=%04x ST1=%04x  AR1=%04x\n",
                                 dsp->t, (unsigned long long)(dsp->b & 0xffffffffffULL), (unsigned)((dsp->b >> 16) & 0xffff),
                                 dsp->st0 & 0x1ff, !!(dsp->st1 & 0x4000), dsp->st0, dsp->st1, dsp->ar[1]); } }
                    { static int cpl_prev = -1; static uint16_t pc_prev; static unsigned ncpl;
                      int cpl = !!(dsp->st1 & 0x4000);
                      if (cpl_prev >= 0 && cpl != cpl_prev && ncpl < 16) { ncpl++;
                          printf("  [cpl] CPL %d -> %d apres l'instruction pc=%04x op=%04x  (fn=%u ST1=%04x SP=%04x)\n",
                                 cpl_prev, cpl, pc_prev, prog_ovly(dsp, pc_prev), fn_cur, dsp->st1, dsp->sp); }
                      cpl_prev = cpl; pc_prev = pc; }
                    if (pc == 0x9866 && nd < 3) {
                        printf("  [dec] fn_demod=%u ACS termine : TRN[0..38] @0x2c82 =", g_sb_cmd_fn);
                        for (int k = 0; k < 39; k++) printf(" %04x", dsp->data[0x2c82 + k]);
                        printf("\n        metriques @0x2c00..0x2c1f =");
                        for (int k = 0; k < 32; k++) printf(" %04x", dsp->data[0x2c00 + k]);
                        printf("\n        softs @0x2a00[0..7] = %04x %04x %04x %04x %04x %04x %04x %04x  BK=%04x AR0=%04x ST1=%04x\n",
                               dsp->data[0x2a00], dsp->data[0x2a01], dsp->data[0x2a02], dsp->data[0x2a03],
                               dsp->data[0x2a04], dsp->data[0x2a05], dsp->data[0x2a06], dsp->data[0x2a07],
                               dsp->bk, dsp->ar[0], dsp->st1);
                    }
                    if (pc == 0x987b && nd < 3) {
                        int ri = g_n_reels ? g_reel_pour_fn[g_sb_cmd_fn & 63] : -1;
                        unsigned char u[35];
                        if (ri >= 0) { /* decode the real codeword back to u: not available, print softs only */
                            memset(u, 9, sizeof u);
                        } else {
                            uint32_t fn = g_sb_cmd_fn;
                            uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = fn % 51, t3p = t3 ? (t3 - 1) / 10 : 0;
                            uint8_t sb_info[4] = {
                                (uint8_t)(((g_bsic_injecte & 0x3f) << 2) | ((t1 & 0x600) >> 9)),
                                (uint8_t)((t1 & 0x1fe) >> 1),
                                (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1)),
                                (uint8_t)(t3p & 0x1) };
                            ubit_t ub[35];
                            osmo_pbit2ubit_ext(ub, 0, sb_info, 0, 25, 1);
                            osmo_crc16gen_set_bits(&gsm0503_sch_crc10, ub, 25, ub + 25);
                            for (int k = 0; k < 35; k++) u[k] = ub[k];
                        }
                        unsigned long long um = 0, ul = 0;
                        for (int k = 0; k < 35; k++) { um = (um << 1) | u[k]; ul |= (unsigned long long)u[k] << k; }
                        printf("  [dec] traceback termine : mots @0x2c00 = %04x %04x %04x %04x | attendu u[0..34] MSB-first=0x%09llx LSB-first=0x%09llx\n",
                               dsp->data[0x2c00], dsp->data[0x2c01], dsp->data[0x2c02], dsp->data[0x2c03], um, ul);
                        printf("        u = "); for (int k = 0; k < 35; k++) printf("%d", u[k]); printf("\n");
                    }
                    if (pc == 0x98a2 && nd < 3) {
                        printf("  [dec] CRC : drapeau 0x2bf8=%u  A=%010llx  mots @0x2c00 = %04x %04x %04x\n",
                               dsp->data[0x2bf8], (unsigned long long)(dsp->a & 0xffffffffffULL),
                               dsp->data[0x2c00], dsp->data[0x2c01], dsp->data[0x2c02]);
                        nd++;
                    }
                }
                g_op_tous[prog_ovly(dsp, pc)]++;
                int ex = c54x_run(dsp, 1);
                if (ex <= 0) break;
                done += ex;
                insn_total++;
            }
        } else {
            /* single-step around the TOA computation (0x7940..0x795c): A, B, T, 0x3fb4 */
            static int nlog;
            while (done < insns && dsp->running && !dsp->idle) {
                uint16_t pc = dsp->pc & 0xffff;
                /* The SB demod writes 78 soft bits at 0x2a00 (repack at 0x7e94) and
                 * the decoder at 0x9841 reads them back. The FB correlator uses THE SAME
                 * buffer. Compare both instants to prove the overwrite. */
                { static uint16_t apres_demod[8]; static int arme, np;
                  if (pc == 0x7e94 && !arme) {
                      for (int k = 0; k < 8; k++) apres_demod[k] = dsp->data[0x2a00 + k];
                      arme = 1;
                  } else if (pc == 0x9841 && arme && np < 6) {
                      int diff = 0;
                      for (int k = 0; k < 8; k++) if (dsp->data[0x2a00 + k] != apres_demod[k]) diff = 1;
                      printf("    [2a00] apres demod: %04x %04x %04x %04x | a l'entree du decodeur: %04x %04x %04x %04x  => %s\n",
                             apres_demod[0], apres_demod[1], apres_demod[2], apres_demod[3],
                             dsp->data[0x2a00], dsp->data[0x2a01], dsp->data[0x2a02], dsp->data[0x2a03],
                             diff ? "ECRASE" : "intact");
                      np++; arme = 0;
                  } }
                /* [2026-09-17] Proof that the samples carry the message: demodulate
                 * the SAME buffer 0x0cce the DSP reads, with a reference demodulator
                 * written in C. If it recovers the code word and the DSP does not, the
                 * signal is good and the fault is entirely in the emulated DSP. */
                /* Does the buffer hold EXACTLY the samples delivered for this frame,
                 * or a mix of several bursts? The midamble is identical in every SCH: a
                 * mix would reinforce it while drowning the data, which is precisely the
                 * symptom observed. */
                { static int nx;
                  if (pc == 0x9841 && nx < 4 && g_livre_niq) {
                      int ident = 0, n = g_livre_niq < 380 ? g_livre_niq : 380;
                      int prem_diff = -1;
                      for (int k = 0; k < n; k++) {
                          if ((int16_t)dsp->data[0x0cce + k] == g_livre_iq[k]) ident++;
                          else if (prem_diff < 0) prem_diff = k;
                      }
                      printf("    [tampon] identique aux echantillons livres : %d/%d mots"
                             " (1re difference au mot %d)\n", ident, n, prem_diff);
                      nx++;
                  } }
                /* [2026-09-18] At the decoder entry, measure what it actually READS,
                 * job by job: the signs of the 78 words packed at 0x2a00 against the
                 * expected code word, with the best frame searched as in [ref] (under
                 * REJEU_SCH_PARTOUT the delivered FN is not reliable). Both polarities
                 * are printed: on differential GMSK the sign convention is not known a
                 * priori. Links "the stage rewriting 0x2a00 outputs values" to "the
                 * decoder has something to work on". */
                { static int nb2;
                  if (pc == 0x9841 && nb2 < 20) {
                      int16_t sb[78]; int nz = 0, mn = 32767, mx = -32768;
                      for (int k = 0; k < 78; k++) {
                          sb[k] = (int16_t)dsp->data[0x2a00 + k];
                          if (!sb[k]) nz++;
                          if (sb[k] < mn) mn = sb[k];
                          if (sb[k] > mx) mx = sb[k];
                      }
                      int meil_f = -1, meil_s = -1, s0 = -1;
                      for (int df = -12; df <= 3; df++) {
                          long f = (long)g_livre_fn + df; if (f < 0) continue;
                          unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)g_bsic_injecte, a2);
                          int ok = 0;
                          for (int k = 0; k < 78; k++) { int bit = sb[k] < 0 ? 1 : 0; if (bit == a2[k]) ok++; }
                          int meilleur = ok > 78 - ok ? ok : 78 - ok;
                          if (meilleur > meil_s) { meil_s = meilleur; meil_f = (int)f; }
                          if (df == 0) s0 = ok;
                      }
                      printf("    [2a00-lu] job %d : zeros=%d/78 etendue=[%d..%d] |"
                             " signes trame livree %d/78 | meilleure trame %d avec %d/78\n",
                             nb2 + 1, nz, mn, mx, s0, meil_f, meil_s);
                      nb2++;
                  } }
                { static int nr;
                  if (pc == 0x9841 && nr < 4) {
                      int16_t ech[2 * 190];
                      for (int k = 0; k < 380; k++) ech[k] = (int16_t)dsp->data[0x0cce + k];
                      unsigned char b148[148]; int off = 0;
                      unsigned char att[78];
                      cellule_code_attendu(g_livre_fn, (uint8_t)g_bsic_injecte, att);
                      if (cellule_demod_reference(ech, 190, b148, &off) == 0) {
                          /* sanity of the reference demod: the midamble must come back out */
                          int okm = 0;
                          for (int i = 0; i < 64; i++) if (b148[42 + i] == cellule_train_sb(i)) okm++;
                          /* raw differential bits: no error propagation */
                          unsigned char d148[148]; int okd = 0, okdd = 0;
                          if (cellule_demod_d(ech, 190, off, d148) == 0) {
                              for (int i = 1; i < 64; i++) {
                                  int att_d = cellule_train_sb(i) ^ cellule_train_sb(i - 1);
                                  if (d148[42 + i] == att_d) okd++;
                              }
                              /* same bits, opposite polarity */
                              for (int i = 1; i < 64; i++) {
                                  int att_d = cellule_train_sb(i) ^ cellule_train_sb(i - 1);
                                  if (d148[42 + i] != att_d) okdd++;
                              }
                          }
                          /* The DATA too, as raw differential bits, against the full
                           * expected burst (3 tail + 39 + midamble + 39 + 3 tail).
                           * Comparing on b148 is meaningless: differential decoding
                           * propagates any single error over the rest of the burst. */
                          int okdat = 0, ndat = 0, meil_f = -1, meil_s = -1;
                          for (int df = -12; df <= 3; df++) {
                              long f = (long)g_livre_fn + df; if (f < 0) continue;
                              unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)g_bsic_injecte, a2);
                              unsigned char burst[148];
                              memset(burst, 0, 3);
                              memcpy(burst + 3, a2, 39);
                              for (int i = 0; i < 64; i++) burst[42 + i] = (unsigned char)cellule_train_sb(i);
                              memcpy(burst + 106, a2 + 39, 39);
                              memset(burst + 145, 0, 3);
                              int ok2 = 0, n2 = 0, prev2 = 1;
                              for (int i = 0; i < 148; i++) {
                                  int att_d = burst[i] ^ prev2; prev2 = burst[i];
                                  int est_donnee = (i >= 3 && i < 42) || (i >= 106 && i < 145);
                                  if (!est_donnee) continue;
                                  n2++; if (d148[i] == att_d) ok2++;
                              }
                              if (ok2 > meil_s) { meil_s = ok2; meil_f = (int)f; }
                              if (df == 0) { okdat = ok2; ndat = n2; }
                          }
                          printf("    [d-brut] midambule %d/63 (inverse %d) | DONNEES trame livree %d/%d"
                                 " | meilleure trame %d avec %d/%d\n",
                                 okd, okdd, okdat, ndat, meil_f, meil_s, ndat);
                          /* the midamble is IDENTICAL for every SCH frame: if it locks
                           * but the data does not, the buffer holds the burst of ANOTHER
                           * frame. Find out which. */
                          int meilleure = -1, meilleur_score = -1;
                          for (int df = -12; df <= 3; df++) {
                              long f = (long)g_livre_fn + df; if (f < 0) continue;
                              unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)g_bsic_injecte, a2);
                              int ok2 = 0;
                              for (int i = 0; i < 39; i++) if (b148[3 + i] == a2[i]) ok2++;
                              for (int i = 0; i < 39; i++) if (b148[106 + i] == a2[39 + i]) ok2++;
                              if (ok2 > meilleur_score) { meilleur_score = ok2; meilleure = (int)f; }
                          }
                          int ok = 0;
                          for (int i = 0; i < 39; i++) if (b148[3 + i] == att[i]) ok++;
                          for (int i = 0; i < 39; i++) if (b148[106 + i] == att[39 + i]) ok++;
                          printf("    [ref] fn livree=%u offset=%d | midambule %d/64 |"
                                 " mot de code de la trame livree %d/78 |"
                                 " MEILLEURE trame = %d avec %d/78\n",
                                 g_livre_fn, off, okm, ok, meilleure, meilleur_score);
                      } else {
                          printf("    [ref] fn=%u  demod de reference : pas de pic exploitable\n", g_livre_fn);
                      }
                      nr++;
                  } }
                /* [2026-09-17] Hunting the next ISA bug. The FB path works, so every
                 * opcode it executes is valid by construction. List the opcodes only the
                 * SB demodulator executes: the one family never validated, hence the pool
                 * of suspects. */
                { static int dans_sb;
                  if (!vu_sb) { vu_sb = calloc(65536, 1); vu_hors = calloc(65536, 1); }
                  if (pc == 0x7c31) dans_sb = 1;
                  if (pc == 0x9841) dans_sb = 0;
                  /* [2026-09-18] Flipping ONE coded bit moves 78 positions out of 78
                   * in 0x2c72: the traceback is global. On the C54x that points at CMPS
                   * (0x8E/0x8F) and the TRN register. So count what really executes in
                   * the SB path: the true CMPS, or the 0xE0 family (FIRS/LMS/SQDST/ABDST)
                   * whose variant still carries pseudo-CMPS semantics in the core. */
                  { uint16_t o = dsp->prog[pc]; uint8_t h = o >> 8;
                    if (dans_sb) {
                        if (h == 0x8E || h == 0x8F) n_cmps_sb++;
                        if (h >= 0xE0 && h <= 0xE3) { n_e0_sb++; n_e0x_sb[h - 0xE0]++; }
                        if (o == 0x8D00 || (o & 0xFF00) == 0x8D00) n_sttrn_sb++;
                    } else {
                        if (h == 0x8E || h == 0x8F) n_cmps_hors++;
                    } }
                  { uint16_t o = dsp->prog[pc];
                    if (dans_sb) vu_sb[o] = 1; else vu_hors[o] = 1; }
                }
                /* [2026-09-17] The 78 soft bits are at 0x2c72 (NOT 0x2a00, which is
                 * the 296-word FB correlator buffer). Compare the SIGN of each with the
                 * bit actually transmitted: matching signs mean the demodulator is sound
                 * and the fault is in the decoder; matching with a shift means the
                 * correlator offset is wrong. */
                { static int nb;
                  if (pc == 0x9841 && nb < 400) {
                      unsigned char att[78];
                      cellule_code_attendu(g_livre_fn, (uint8_t)g_bsic_injecte, att);
                      /* Try the plausible conventions: polarity, halves swapped (the
                       * DSP may return 39+39 in the other order), and a shift. A
                       * combination clearly above chance means the demodulator is sound
                       * and it is a matter of convention. */
                          /* [2026-09-18] The family tested did NOT include even/odd
                           * de-interleaving. The SCH convolutional code emits its 78 bits
                           * in the order C(2k), C(2k+1) per trellis step, and the burst
                           * stores them as two halves of 39. If the DSP stores its softs
                           * in the INTERNAL order (all evens then all odds), no cyclic
                           * shift, polarity or half swap recovers it: every combination
                           * lands exactly at chance, which is precisely the plateau
                           * observed. Add the permutation and its inverse.
                           *   0 identity   1 halves swapped
                           *   2 evens first   3 odds first */
                          int best = -99, bestd = 0, bestv = 0;
                          for (int v = 0; v < 8; v++) {
                              int inv = v & 1, perm = v >> 1;
                              for (int d = -8; d <= 8; d++) {
                                  int ok = 0, tot = 0;
                                  for (int i = 0; i < 78; i++) {
                                      int src;
                                      switch (perm) {
                                      case 1:  src = (i + 39) % 78; break;
                                      case 2:  src = (i % 2 == 0) ? (i / 2) : (39 + (i - 1) / 2); break;
                                      case 3:  src = (i % 2 == 1) ? ((i - 1) / 2) : (39 + i / 2); break;
                                      default: src = i; break;
                                      }
                                  int j = src + d; if (j < 0 || j >= 78) continue;
                                  int16_t sv = (int16_t)dsp->data[0x2c72 + j];
                                  if (!sv) continue;
                                  tot++;
                                  int bit = att[i] != 0; if (inv) bit = !bit;
                                  if ((sv < 0) == bit) ok++;
                              }
                              if (tot >= 40) { int pc2 = ok * 100 / tot;
                                  if (pc2 > best) { best = pc2; bestd = d; bestv = v; } }
                          }
                      }
                      int nznb = 0; for (int i = 0; i < 78; i++) if (dsp->data[0x2c72 + i]) nznb++;
                          /* Is the DSP output a SLICE of the burst at some arbitrary
                           * offset? Correlate the 78 signs against the differential
                           * sequence of the 148 burst bits, at every shift, in both
                           * polarities. */
                          { unsigned char burst[148]; int prev3 = 1; unsigned char dref[148];
                            memset(burst, 0, 3); memcpy(burst + 3, att, 39);
                            for (int i = 0; i < 64; i++) burst[42 + i] = (unsigned char)cellule_train_sb(i);
                            memcpy(burst + 106, att + 39, 39); memset(burst + 145, 0, 3);
                            for (int i = 0; i < 148; i++) { dref[i] = burst[i] ^ prev3; prev3 = burst[i]; }
                            int bs = -1, bo = 0, bp = 0;
                            for (int o = -78; o <= 148; o++) {
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok = 0, tot = 0;
                                    for (int i = 0; i < 78; i++) {
                                        int j = o + i; if (j < 0 || j >= 148) continue;
                                        int16_t sv = (int16_t)dsp->data[0x2c72 + i];
                                        if (!sv) continue;
                                        tot++; int b = dref[j]; if (pol) b = !b;
                                        if ((sv < 0) == b) ok++;
                                    }
                                    if (tot >= 50 && ok * 100 / tot > bs) { bs = ok * 100 / tot; bo = o; bp = pol; }
                                }
                            }
                            printf("    [tranche] sortie DSP vs burst complet : max %d%% a l'offset %d (polarite %d)\n",
                                   bs, bo, bp); }
                      /* [2026-09-18] Peak and agreement on the SAME line, so they can
                       * be correlated frame by frame: if the frames whose peak is 21 (the
                       * real burst start) agree better, peak PRECISION is still a problem
                       * and the channel estimate sits beside the burst, which is enough
                       * to ruin an MLSE equalizer on GMSK whose ISI spans three symbols.
                       * A flat agreement whatever the peak clears the correlator. */
                      /* fingerprint of the 78 soft bits, to diff two runs */
                      { static int nemq; static long cible = -2;
                        if (cible == -2) { const char *e = calypso_getenv("REJEU_EMPREINTE_FN");
                                           cible = e ? atol(e) : -1; }
                        /* [2026-09-18] THE FRAME MUST BE LOCKED. Taking whichever
                         * occurrence comes first does not compare the same thing across
                         * runs: the frame the SB attempt lands on depends on FB
                         * scheduling, and under SCH_PARTOUT a different frame means a
                         * different T1/T2/T3, hence an entirely different code word --
                         * 190/380 samples differing and 38 signs out of 78 flipping,
                         * i.e. chance. REJEU_EMPREINTE_FN=<n> prints frame n only. */
                        if (nemq < 1 && (cible < 0 || (long)g_livre_fn == cible)) {
                            /* [2026-09-18] Check the MODULATOR before blaming the
                             * demodulator. Flipping coded bit 0 touches burst bit 3, so
                             * alpha_3 and alpha_4 both change sign and their SUM is
                             * preserved: the phase realigns after two symbols, and only
                             * ~6 complex samples around index 24 should differ out of
                             * 380. If all 380 differ, the modulator is at fault. */
                            printf("    [empreinte-iq] fn=%u :", g_livre_fn);
                            for (int k = 0; k < 380; k++) printf(" %04x", dsp->data[0x0cce + k]);
                            printf("\n"); nemq++;
                        } }
                      { static int nemp; static long cible2 = -2;
                        if (cible2 == -2) { const char *e = calypso_getenv("REJEU_EMPREINTE_FN");
                                            cible2 = e ? atol(e) : -1; }
                        if (nemp < 3 && (cible2 < 0 || (long)g_livre_fn == cible2)) {
                            printf("    [empreinte] fn=%u (trame courante %u%s) pic=%u :", g_livre_fn, fn_cur,
                                   g_livre_fn == fn_cur ? "" : " DESALIGNE", dsp->data[0x2f06]);
                            for (int k = 0; k < 78; k++) printf(" %04x", dsp->data[0x2c72 + k]);
                            printf("\n"); nemp++;
                        } }
                      /* [2026-09-18] Four independent decodings of the block
                       * 0x84a0-0x84d8 show it is NOT the soft-bit writer but the MIDAMBLE
                       * CORRELATOR: 50 lags, 64 taps of a fixed reference at 0x2cea
                       * (64 = midamble length), Re output at 0x2c56 and Im at 0x2c88,
                       * then |corr|^2 at 0x2be4 for the argmax. Its index is the LAG, not
                       * a coded-bit rank, so there is nothing to look for there. They
                       * point at 0x2a00 as the real buffer (loop 0x848d-0x849f, BRC=141
                       * hence 142 iterations, `sth B,*AR6+` at 0x8497). 142 is close to
                       * the 148 burst bits, hence the hypothesis "0x2a00 indexed by
                       * POSITION IN THE BURST", where coded bit b is at 3+b for b<39 and
                       * at 106+(b-39) beyond. */
                      /* [2026-09-18] SEARCH RATHER THAN GUESS. At the decoder entry,
                       * sweep ALL of data memory for a zone whose SIGNS match the
                       * transmitted code word, under two layouts: 78 contiguous words
                       * indexed by coded-bit rank, and indexing by burst position (3+b
                       * then 106+b-39). Print the best bases. If none stands clearly
                       * above chance, the soft bits are not in data memory in either
                       * form. */
                      /* [2026-09-18] INPUT / OUTPUT DISCRIMINANT for the 0x2ad5
                       * candidate. Agreement does not separate the two: the derotated
                       * input and the demodulator output are both indexed by burst
                       * position and would answer a bit flip alike. What separates them
                       * is CHANNEL dependence: a demodulator output depends on the
                       * channel estimate, hence on the peak; an input representation does
                       * not. So perturb the MIDAMBLE alone (which moves the peak without
                       * touching the data) and see whether 0x2ad5 moves AT THE DATA
                       * POSITIONS. Invariant => input. Moving => output, buffer found. */
                      /* [2026-09-18] THE REAL BUFFER, found in ROM: 0x2a00.
                       * Two successive stages, both at 0x2a00:
                       *   - equalizer output, 142 words, index = burst position - 3
                       *     (`sth B,*AR6+`, AR6 init 0x2a00, BRC=0x8d hence 142 turns);
                       *   - PACKED codeword, 78 contiguous words, index = coded-bit rank,
                       *     produced by 0x7e65-0x7e7a: 39 copies, then
                       *     `mar *+AR2(0x0040)` at 0x7e76 which SKIPS THE 64 MIDAMBLE
                       *     BITS, then 39 copies. The +64 is literal in ROM.
                       * The decoder confirms it: 0x984a `stm #0x2a00,AR1`, BRC=0x26, and
                       * the body at 0x9a78 advances AR1 by 2 per trellis step, 39 steps =
                       * 78 words.
                       *
                       * Zeros must NOT be discarded here: these soft bits take the values
                       * 0x0000 and 0xffff, so an `if (!v) continue;` plus a `n >= 60`
                       * requirement threw away half the samples and eliminated 0x2a00
                       * before it could be scored. */
                      { static int n2a0;
                        if (n2a0 < 4) {
                            int meil_c = -1, pol_c = 0, meil_b = -1, pol_b = 0;
                            for (int pol = 0; pol < 2; pol++) {
                                int ok = 0;
                                for (int b = 0; b < 78; b++) {
                                    int16_t v = (int16_t)dsp->data[0x2a00 + b];
                                    int bit = att[b] != 0; if (pol) bit = !bit;
                                    if ((v < 0) == bit) ok++;
                                }
                                if (ok * 100 / 78 > meil_c) { meil_c = ok * 100 / 78; pol_c = pol; }
                                ok = 0;
                                for (int b = 0; b < 78; b++) {
                                    int j = (b < 39) ? b : (64 + b);
                                    int16_t v = (int16_t)dsp->data[0x2a00 + j];
                                    int bit = att[b] != 0; if (pol) bit = !bit;
                                    if ((v < 0) == bit) ok++;
                                }
                                if (ok * 100 / 78 > meil_b) { meil_b = ok * 100 / 78; pol_b = pol; }
                            }
                            int nz = 0, nff = 0, naut = 0;
                            for (int k = 0; k < 142; k++) {
                                uint16_t v = dsp->data[0x2a00 + k];
                                if (v == 0) nz++; else if (v == 0xffff) nff++; else naut++;
                            }
                            printf("    [2a00] COMPACTE (0x2a00+b) : %d%% (polarite %d) | "
                                   "BRUT (saut de 64) : %d%% (polarite %d)\n",
                                   meil_c, pol_c, meil_b, pol_b);
                            printf("    [2a00] contenu sur 142 mots : %d nuls, %d a 0xffff, %d autres |",
                                   nz, nff, naut);
                            for (int k = 0; k < 10; k++) printf(" %04x", dsp->data[0x2a00 + k]);
                            printf("\n");
                            n2a0++;
                        } }
                      { static int n2d;
                        if (n2d < 1) {
                            printf("    [2ad5] pic=%u valeurs aux positions de burst 0..147 :\n      ",
                                   dsp->data[0x2f06]);
                            for (int k = 0; k < 148; k++) {
                                printf(" %04x", dsp->data[0x2ad5 + k]);
                                if (k % 12 == 11) printf("\n      ");
                            }
                            printf("\n");
                            n2d++;
                        } }
                      { static int nch;
                        if (nch < 1) {
                            struct { int sc, base, disp, pol; } top[6];
                            for (int t = 0; t < 6; t++) { top[t].sc = -1; top[t].base = 0; }
                            for (int base = 0; base < 0x3f00; base++)
                              for (int disp = 0; disp < 2; disp++)
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok = 0, n = 0;
                                    for (int b = 0; b < 78; b++) {
                                        int j = disp ? ((b < 39) ? (3 + b) : (106 + b - 39)) : b;
                                        int16_t v = (int16_t)dsp->data[(base + j) & 0x3fff];
                                        if (!v) continue;
                                        n++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok++;
                                    }
                                    if (n < 60) continue;
                                    int sc = ok * 100 / n;
                                    for (int t = 0; t < 6; t++)
                                        if (sc > top[t].sc) {
                                            for (int u = 5; u > t; u--) top[u] = top[u-1];
                                            top[t].sc = sc; top[t].base = base; top[t].disp = disp; top[t].pol = pol;
                                            break;
                                        }
                                }
                            printf("    [chasse] meilleure zone : base 0x%04x %s pol %d -> %d%%"
                                   "   (2e: 0x%04x %d%%, 3e: 0x%04x %d%%)\n",
                                   top[0].base, top[0].disp ? "burst" : "contigu", top[0].pol, top[0].sc,
                                   top[1].base, top[1].sc, top[2].base, top[2].sc);
                            nch++;
                        } }
                      { static int n2a;
                        if (n2a < 3) {
                            int meil = -1, mo = 0, mp = 0;
                            for (int o = -8; o <= 8; o++)
                              for (int pol = 0; pol < 2; pol++) {
                                  int ok = 0, n = 0;
                                  for (int b = 0; b < 78; b++) {
                                      int bp = (b < 39) ? (3 + b) : (106 + b - 39);
                                      int j = bp + o; if (j < 0 || j >= 148) continue;
                                      int16_t v = (int16_t)dsp->data[0x2a00 + j]; if (!v) continue;
                                      n++; int bit = att[b] != 0; if (pol) bit = !bit;
                                      if ((v < 0) == bit) ok++;
                                  }
                                  if (n >= 50) { int pc2 = ok * 100 / n;
                                      if (pc2 > meil) { meil = pc2; mo = o; mp = pol; } }
                              }
                            int nz = 0; for (int k = 0; k < 148; k++) if (dsp->data[0x2a00 + k]) nz++;
                            printf("    [2a00] indexe par position de burst : concordance max %d%%"
                                   " (decalage %d, polarite %d) | %d/148 mots non nuls\n",
                                   meil, mo, mp, nz);
                            printf("    [2a00] valeurs :");
                            for (int k = 0; k < 14; k++) printf(" %04x", dsp->data[0x2a00 + k]);
                            printf("\n");
                            n2a++;
                        } }
                      { static int nv;
                        if (nv < 3) {
                            int m1b = -1, m1o = 0, m1s = 0, m2b = -1, m2o = 0, m2s = 0;
                            for (int o = 0; o < 100; o++)
                              for (int sens = -1; sens <= 1; sens += 2)
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok1 = 0, n1 = 0, ok2 = 0, n2 = 0;
                                    for (int b = 0; b < 39; b++) {
                                        int j = o + sens * b; if (j < 0 || j >= 100) continue;
                                        int16_t v = (int16_t)dsp->data[0x2c56 + j]; if (!v) continue;
                                        n1++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok1++;
                                    }
                                    for (int b = 39; b < 78; b++) {
                                        int j = o + sens * (b - 39); if (j < 0 || j >= 100) continue;
                                        int16_t v = (int16_t)dsp->data[0x2c56 + j]; if (!v) continue;
                                        n2++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok2++;
                                    }
                                    if (n1 >= 30) { int pc1 = ok1 * 100 / n1;
                                        if (pc1 > m1b) { m1b = pc1; m1o = o; m1s = sens * (pol ? -2 : 1); } }
                                    if (n2 >= 30) { int pc2 = ok2 * 100 / n2;
                                        if (pc2 > m2b) { m2b = pc2; m2o = o; m2s = sens * (pol ? -2 : 1); } }
                                }
                            printf("    [2c56] moitie 1 : meilleur %d%% a l'offset %d (code %d) | "
                                   "moitie 2 : meilleur %d%% a l'offset %d (code %d)\n",
                                   m1b, m1o, m1s, m2b, m2o, m2s);
                            int nz = 0; for (int k = 0; k < 100; k++) if (dsp->data[0x2c56 + k]) nz++;
                            printf("    [2c56] %d/100 mots non nuls, carte des nuls (. = nul, X = non nul) :\n      ", nz);
                            for (int k = 0; k < 100; k++) {
                                printf("%c", dsp->data[0x2c56 + k] ? 'X' : '.');
                                if (k % 50 == 49) printf("\n      ");
                            }
                            printf("\n");
                            for (int lig = 0; lig < 10; lig++) {
                                printf("      +%2d :", lig * 10);
                                for (int k = lig * 10; k < lig * 10 + 10; k++) printf(" %04x", dsp->data[0x2c56 + k]);
                                printf("\n");
                            }
                            nv++;
                        } }
                      printf("    [pic-conc] pic=%u concordance=%d%%\n", dsp->data[0x2f06], best);
                      printf("    [bits] fn=%u  0x2c72 non-nuls=%d/78 : %04x %04x %04x %04x %04x %04x\n"
                             "           concordance max = %d%% (decalage %d, polarite %d, ordre %s)\n",
                             g_livre_fn, nznb, dsp->data[0x2c72], dsp->data[0x2c73], dsp->data[0x2c74],
                             dsp->data[0x2c75], dsp->data[0x2c76], dsp->data[0x2c77],
                             best, bestd, bestv & 1,
                             (bestv >> 1) == 1 ? "moities echangees"
                             : (bestv >> 1) == 2 ? "pairs d'abord"
                             : (bestv >> 1) == 3 ? "impairs d'abord" : "identite");
                      nb++;
                  } }
                /* [2026-09-18] The `sb` word returned is IDENTICAL for BSIC 7, 14 and
                 * 49, while the soft bits at 0x2c72 do follow the payload: the word the
                 * ARM reads back does NOT come from the decoder. Does the DSP ever write
                 * the a_sch words of the R page? API at 0x0800, R_PAGE0=0x28 and
                 * R_PAGE1=0x3C, a_sch at word +15. */
                { static uint16_t vu0[10], vu1[10]; static int arme3, nw;
                  const uint16_t A0 = 0x800 + 0x28 + 15, A1 = 0x800 + 0x3C + 15;
                  if (!arme3) {
                      for (int k = 0; k < 10; k++) { vu0[k] = dsp->data[A0 + k]; vu1[k] = dsp->data[A1 + k]; }
                      arme3 = 1;
                  } else if (nw < 12) {
                      for (int k = 0; k < 5; k++) {
                          if (dsp->data[A0 + k] != vu0[k]) {
                              printf("    [a_sch] page0 mot %d : %04x -> %04x  ecrit par pc=%04x\n",
                                     k, vu0[k], dsp->data[A0 + k], pc);
                              vu0[k] = dsp->data[A0 + k]; nw++;
                          }
                          if (dsp->data[A1 + k] != vu1[k]) {
                              printf("    [a_sch] page1 mot %d : %04x -> %04x  ecrit par pc=%04x\n",
                                     k, vu1[k], dsp->data[A1 + k], pc);
                              vu1[k] = dsp->data[A1 + k]; nw++;
                          }
                      }
                  } }
                /* [2026-09-18] The magnitudes are present and the argmax is still
                 * wrong. Next question: WHICH zone does it sweep? Follow the span of the
                 * address registers during peak selection (0x84c8..0x84ef) and compare it
                 * with the magnitude zones. */
                { static uint16_t armin[8], armax[8]; static int arme2;
                  if (pc >= 0x84c8 && pc <= 0x84ef) {
                      if (!arme2) { for (int k = 0; k < 8; k++) { armin[k] = 0xffff; armax[k] = 0; } arme2 = 1; }
                      for (int k = 0; k < 8; k++) {
                          uint16_t v = dsp->ar[k];
                          if (v < armin[k]) armin[k] = v;
                          if (v > armax[k]) armax[k] = v;
                      }
                  }
                { static int nq;
                  if (pc == 0x84ef && nq < 5) {
                      printf("    [balayage] registres d'adresse pendant 0x84c8-0x84ef :");
                      for (int k = 0; k < 8; k++)
                          if (armax[k] >= armin[k] && armax[k] > armin[k])
                              printf(" AR%d=[0x%04x..0x%04x]", k, armin[k], armax[k]);
                      printf("\n");
                      arme2 = 0;
                      printf("    [magn@84ef] plages non nulles entre 0x2b00 et 0x2d00 :\n");
                      int deb = -1, fin = -1;
                      for (int a = 0x2b00; a <= 0x2d00; a++) {
                          int nz = (a <= 0x2cff) && dsp->data[a] != 0;
                          if (nz && deb < 0) deb = a;
                          if (nz) fin = a;
                          if (!nz && deb >= 0) {
                              /* [2026-09-18] Compare as int16_t, not uint16_t: the
                               * reported "maxima" (65502, 65104, 65318) were in fact
                               * -34, -432, -218, so the probe reported the value closest
                               * to -1 and the rank with it was noise. The maximum in
                               * absolute value is printed too: if 0x2be4 carries a SIGNED
                               * correlation, |v| (or v*v) must decide, not v. The two
                               * ranks side by side say which. */
                              int n = fin - deb + 1;
                              int16_t vmax = -32768; int imax = -1;
                              int amax = -1, iamax = -1;
                              for (int k = deb; k <= fin; k++) {
                                  int16_t v = (int16_t)dsp->data[k];
                                  int a = v < 0 ? -(int)v : (int)v;
                                  if (v > vmax) { vmax = v; imax = k - deb; }
                                  if (a > amax) { amax = a; iamax = k - deb; }
                              }
                              printf("        0x%04x-0x%04x : %3d mots, max signe=%d au rang %d,"
                                     " max |v|=%d au rang %d\n",
                                     deb, fin, n, vmax, imax, amax, iamax);
                              deb = -1;
                          }
                      }
                      printf("        argmax DSP 0x2f06=%u\n", dsp->data[0x2f06]);
                      nq++;
                  } } }
                /* [2026-09-17] Where does the correlator ACTUALLY write? Snapshot
                 * data RAM on entry to 0x84a1 and list what changed on exit, rather than
                 * guessing the address of the energy buffer. */
                { static uint16_t *snap; static int armec, ns;
                  if (pc == 0x84a1 && !armec && ns < 3) {
                      if (!snap) snap = malloc(0x4000 * sizeof(uint16_t));
                      memcpy(snap, dsp->data, 0x4000 * sizeof(uint16_t));
                      armec = 1;
                  } else if (pc == 0x7e94 && armec) {
                      int deb = -1, fin = -1, nch = 0;
                      printf("    [ecrit] plages modifiees par le correlateur SB :\n");
                      for (int k = 0; k < 0x4000; k++) {
                          int d = (dsp->data[k] != snap[k]);
                          if (d && deb < 0) deb = k;
                          if (d) { fin = k; nch++; }
                          if (!d && deb >= 0 && k > fin + 8) {
                              printf("        0x%04x-0x%04x (%d mots)  ex: %04x->%04x\n",
                                     deb, fin, fin - deb + 1, snap[deb], dsp->data[deb]);
                              deb = -1;
                          }
                      }
                      if (deb >= 0) printf("        0x%04x-0x%04x\n", deb, fin);
                      printf("        total %d mots changes\n", nch);
                      armec = 0; ns++;
                  } }
                /* [2026-09-17] With a REAL SCH burst on the input, the soft bits at
                 * 0x2a00 come out null. Look at the middle link: the midamble
                 * correlation energy (0x2be4) and the argmax (0x2f06). */
                { static int nc;
                  if (pc == 0x84ef && nc < 8) {
                      /* same type fix as above: int16_t, plus |v| */
                      int nzc = 0, emax = -1, iemax = -1, samax = -32768, isamax = -1;
                      for (int k = 0; k < 64; k++) {
                          int16_t v = (int16_t)dsp->data[0x2be4 + k];
                          int a = v < 0 ? -(int)v : (int)v;
                          if (v) nzc++;
                          if (a > emax) { emax = a; iemax = k; }
                          if (v > samax) { samax = v; isamax = k; }
                      }
                      int nzin = 0; for (int k = 0; k < 380; k++) if (dsp->data[0x0cce + k]) nzin++;
                      printf("    [corr] entree 0x0cce non-nuls=%d/380 | 0x2be4 non-nuls=%d/64"
                             " | max |v|=%d au rang %d, max signe=%d au rang %d"
                             " | argmax 0x2f06=%u | 0x2be4: %04x %04x %04x %04x %04x %04x\n",
                             nzin, nzc, emax, iemax, samax, isamax, dsp->data[0x2f06],
                             dsp->data[0x2be4], dsp->data[0x2be5], dsp->data[0x2be6],
                             dsp->data[0x2be7], dsp->data[0x2be8], dsp->data[0x2be9]);
                      nc++;
                  } }
                /* SB demod input: the DMA must have dropped 190 complex samples at 0x0cce */
                { static int ne;
                  if (pc == 0x7c31 && ne < 6) {
                      int nz = 0; for (int k = 0; k < 380; k++) if (dsp->data[0x0cce + k]) nz++;
                      int z0 = 0; while (z0 < 380 && !dsp->data[0x0cce + z0]) z0++;
                      printf("    [in-sb] fn=%u p51=%u  DERNIER BURST LIVRE: type=%c fn=%u n=%d\n"
                             "            0x0cce non-nuls=%d/380 zeros_tete=%d : %04x %04x %04x %04x %04x %04x\n",
                             fn_cur, fn_cur % 51, g_livre_type ? g_livre_type : '?', g_livre_fn, g_livre_n,
                             nz, z0, dsp->data[0x0cce], dsp->data[0x0ccf], dsp->data[0x0cd0],
                             dsp->data[0x0cd1], dsp->data[0x0cd2], dsp->data[0x0cd3]);
                      ne++;
                  } }
                switch (pc) {                       /* SB chain (RE 9.4) */
                case 0x7c31: hit_7c31++; break;     /* SB demodulator */
                /* Counters must be fed in BOTH execution paths: one that stays at zero
                 * under REJEU_PROBE_TOA while the code runs reads as a measurement. */
                case 0x7d1c: hit_7d1c++; break;     /* rpt #15, the division */
                case 0x7d1d: hit_7d1d++; break;     /* subc */
                case 0x7d1e: hit_7d1e++; break;     /* the quotient */
                case 0x81e4: hit_81e4++; break;     /* the mpy that depends on T */
                case 0x8478: hit_8478++; break;     /* FIRS site 1 */
                case 0x8492: hit_8492++; break;     /* rpt of FIRS site 2 */
                case 0x8493: hit_8493++; break;     /* FIRS site 2 */
                case 0x8497: hit_8497++; break;     /* sth B -> soft bits */
                case 0x9841: hit_9841++; break;     /* SCH decoder */
                case 0x84a1: hit_84a1++; break;     /* midamble correlator */
                case 0x770a: hit_770a++; break;     /* FB correlator */
                case 0xb219: hit_b219++; break;     /* SB job (arms the DMA) */
                case 0x7a16: hit_7a16++; break;     /* fcall into the SB demod */
                default: break; }
                histo_pc[pc >> 10]++; insn_total++;
                { static uint16_t tprev, tpc, ppc; static int tn;
                  if (dsp->t != tprev) { tpc = ppc; tprev = dsp->t; }
                  ppc = pc;
                  if (pc == 0x7947 && tn < 10 && env_probe_t) {
                      printf("    [T] au mpya (0x7947) : T=%04x (%u), dernier ecrit par pc=%04x\n",
                             dsp->t, dsp->t, tpc); tn++; } }
                if (!env_probe_t && pc >= 0x7940 && pc <= 0x795c && nlog < 60) {
                    printf("    [toa] pc=%04x A=%010llx B=%010llx T=%04x 3fb4=%04x AR4=%04x\n",
                           pc, (unsigned long long)(dsp->a & 0xffffffffffULL),
                           (unsigned long long)(dsp->b & 0xffffffffffULL),
                           dsp->t, dsp->data[0x3fb4], dsp->ar[4]);
                    nlog++;
                }
                g_op_tous[prog_ovly(dsp, pc)]++;
                int ex = c54x_run(dsp, 1);
                if (ex <= 0) break;
                done += ex;
            }
        }
    }
    {   /* [2026-09-18] Raw dump of the program words around the soft-bit writer,
         * to read the ADDRESS COMPUTATION of both passes. */
        printf("  mots programme 0x8378-0x8396 (l'ecrivain 0x838d qui met 142 zeros) :\n");
        for (uint16_t a = 0x8378; a <= 0x8396; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x8378) % 8 == 7) ? "\n" : "");
        printf("\n  mots programme 0x81c8-0x81e0 (les ecrivains des 142 valeurs) :\n");
        for (uint16_t a = 0x81c8; a <= 0x81e0; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x81c8) % 8 == 7) ? "\n" : "");
        printf("\n");
        printf("  mots programme 0x84a0-0x84d8 (correlateur de midambule) :\n");
        for (uint16_t a = 0x84a0; a <= 0x84d8; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x84a0) % 8 == 7) ? "\n" : "");
        printf("\n  mots programme 0x7cb0-0x7cc0 (site qui remet a zero) :\n");
        for (uint16_t a = 0x7cb0; a <= 0x7cc0; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x7cb0) % 8 == 7) ? "\n" : "");
        printf("\n");
    }
    { const char *e = calypso_getenv("REJEU_OPCODES_TOUS");
      if (e && *e) { FILE *fo = fopen(e, "w");
          if (fo) { for (unsigned o = 0; o < 65536; o++) if (g_op_tous[o]) fprintf(fo, "%04x %lu\n", o, g_op_tous[o]);
                    fclose(fo); printf("  opcodes executes ecrits dans %s\n", e); } } }
    printf("\n─── bilan rejeu (deterministe) ───\n");
    {
        printf("  instructions DSP executees : %lu\n", insn_total);
        printf("  histogramme des PC (plages de 1024 mots, non vides) :\n");
        for (int b = 0; b < 64; b++)
            if (histo_pc[b])
                printf("      0x%04x-0x%04x : %10lu  (%4.1f%%)\n",
                       b << 10, ((b + 1) << 10) - 1, histo_pc[b],
                       insn_total ? 100.0 * histo_pc[b] / insn_total : 0.0);
    }
    if (vu_sb && vu_hors) {
        int n = 0;
        printf("  opcodes executes UNIQUEMENT par le demodulateur SB (suspects d'ISA,\n"
               "  jamais valides par le chemin FB qui, lui, fonctionne) :\n    ");
        int fam[256] = {0}, famt[256] = {0};
        for (int o = 0; o < 65536; o++) {
            if (vu_sb[o]) famt[o >> 8]++;
            if (vu_sb[o] && !vu_hors[o]) { fam[o >> 8]++; n++; }
        }
        printf("\n");
        for (int f = 0; f < 256; f++)
            if (fam[f]) printf("    famille %02x.. : %3d opcodes exclusifs SB / %3d executes en SB\n",
                               f, fam[f], famt[f]);
        printf("    (%d opcodes distincts exclusifs au demodulateur SB)\n", n);
    }
    /* [2026-09-19] WHERE DOES 0x01dbf46a COME FROM? The same word is returned by
   * the synthetic fixture, by the real capture in replay and by the bridge: three
   * inputs with nothing in common. A constant word looks like a CONSTANT, not a
   * computation. Sweep data space AND program space for the pattern. */
  if (calypso_getenv("REJEU_CHERCHER_MOT")) {
      unsigned long v = strtoul(calypso_getenv("REJEU_CHERCHER_MOT"), NULL, 0);
      uint16_t lo = (uint16_t)(v & 0xffff), hi = (uint16_t)(v >> 16);
      printf("  recherche de 0x%04x%04x (lo=%04x hi=%04x) :\n", hi, lo, lo, hi);
      int n = 0;
      for (unsigned a = 0; a + 1 < 0x4000; a++) {
          if (dsp->data[a] == lo && dsp->data[a+1] == hi && n < 12) {
              printf("    data[0x%04x] = %04x %04x   (lo puis hi)\n", a, lo, hi); n++; }
          if (dsp->data[a] == hi && dsp->data[a+1] == lo && n < 12) {
              printf("    data[0x%04x] = %04x %04x   (hi puis lo)\n", a, hi, lo); n++; }
      }
      for (unsigned a = 0; a + 1 < 0x10000; a++) {
          if (dsp->prog[a] == lo && dsp->prog[a+1] == hi && n < 24) {
              printf("    prog[0x%04x] = %04x %04x\n", a, lo, hi); n++; }
      }
      int nlo = 0, nhi = 0;
      for (unsigned a = 0; a < 0x4000; a++) { if (dsp->data[a]==lo) nlo++; if (dsp->data[a]==hi) nhi++; }
      printf("    occurrences isolees en data : lo(%04x) x%d, hi(%04x) x%d\n", lo, nlo, hi, nhi);
      if (!n) printf("    motif absent de la memoire : le mot est CALCULE, pas stocke\n");
  }
  if (calypso_getenv("REJEU_OPCODES")) {
      printf("  %lu decodages\n", g_n_dec);
      printf("  opcodes de l'EGALISEUR (0x8400-0x84ff), par decodage :\n");
      for (int rang = 0; rang < 16; rang++) {
          unsigned best = 0; unsigned long bv = 0;
          for (unsigned i = 0; i < 65536; i++) if (g_op_eq[i] > bv) { bv = g_op_eq[i]; best = i; }
          if (!bv) break;
          double q = g_n_dec ? (double)bv / g_n_dec : 0;
          printf("    op=%04x  %8lu   /dec = %8.2f %s\n", best, bv, q,
                 (g_n_dec && bv % g_n_dec == 0) ? "  (entier)" : "  <<< NON ENTIER");
          g_op_eq[best] = 0;
      }
      printf("\n");
      printf("  opcodes du DECODEUR (0x9800-0x9bff) :\n");
      { unsigned long st_trn = 0, cmps = 0;
        for (unsigned i = 0; i < 65536; i++) {
            if ((i & 0xFF00) == 0x8D00) st_trn += g_op_dec[i];
            if ((i >> 8) == 0x8E || (i >> 8) == 0x8F) cmps += g_op_dec[i];
        }
        printf("    CMPS (0x8E/0x8F) : %lu     ST TRN (0x8D00) : %lu\n", cmps, st_trn); }
      for (int rang = 0; rang < 14; rang++) {
          unsigned best = 0; unsigned long bv = 0;
          for (unsigned i = 0; i < 65536; i++) if (g_op_dec[i] > bv) { bv = g_op_dec[i]; best = i; }
          if (!bv) break;
          printf("    op=%04x  %8lu fois\n", best, bv);
          g_op_dec[best] = 0;
      }
      printf("  opcodes du demod SB, par nombre de passages :\n");
      for (int rang = 0; rang < 26; rang++) {
          unsigned best = 0; unsigned long bv = 0;
          for (unsigned i = 0; i < 65536; i++) if (g_op_n[i] > bv) { bv = g_op_n[i]; best = i; }
          if (!bv) break;
          printf("    op=%04x  %8lu fois   vu en pc=%04x\n", best, bv, g_op_pc[best]);
          g_op_n[best] = 0;
      }
  }
  printf("  division 16 pas : rpt(7d1c)=%lu subc(7d1d)=%lu quotient(7d1e)=%lu mpy(81e4)=%lu\n",
         hit_7d1c, hit_7d1d, hit_7d1e, hit_81e4);
  printf("  sites FIRS du SB : 0x8478=%lu  0x8492(rpt)=%lu  0x8493=%lu  0x8497(sth B->softs)=%lu\n",
         hit_8478, hit_8492, hit_8493, hit_8497);
  printf("  trames jouees      : %u\n", fn_cur + 1);
    printf("  chaine SB (pas-a-pas) : job b219=%lu  fcall 7a16=%lu  demod 7c31=%lu  corr 84a1=%lu  decodeur 9841=%lu  | corr FB 770a=%lu\n",
           hit_b219, hit_7a16, hit_7c31, hit_84a1, hit_9841, hit_770a);
    printf("  FB acceptes (ARM)  : %d\n", n_fb_ok);
    printf("  SB tentees / CRC KO: %d / %d\n", n_sb_try, n_sb_crcfail);
    printf("  CMPS (0x8E/0x8F) : %ld fois dans le demod SB, %ld hors | famille 0xE0-0xE3 en SB : %ld | ST TRN : %ld\n",
           n_cmps_sb, n_cmps_hors, n_e0_sb, n_sttrn_sb);
    {
        printf("  XC (0xFD/0xFF) par region de PC :");
        for (int b = 0; b < 64; b++) if (n_xc_reg[b]) printf(" 0x%04x=%ld", b << 10, n_xc_reg[b]);
        printf("\n");
        printf("  CMPS (0x8E/0x8F) par region de PC :");
        for (int b = 0; b < 64; b++)
            if (n_cmps_reg[b]) printf(" 0x%04x=%ld", b << 10, n_cmps_reg[b]);
        printf("\n");
    }
    printf("      detail : FIRS(E0)=%ld  LMS(E1)=%ld  SQDST(E2)=%ld  ABDST(E3)=%ld"
           "   <- inertes si CALYPSO_ISA_E0_FAM=1\n",
           n_e0x_sb[0], n_e0x_sb[1], n_e0x_sb[2], n_e0x_sb[3]);
    printf("  erreurs DSP signalees: %d, dont erreur 8 (anneau DMA): %d\n", n_err_dsp, n_err8);
    printf("  CRC OK: %d, dont VRAIES (BSIC injecte + T3 valide + FN = trame): %d%s\n",
           n_crc_ok, n_sb_vraies,
           cellule_sch_partout ? "   [FN non qualifiable : SCH_PARTOUT actif]" : "");
    if (verdict == 1) printf("  VERDICT : SB DECODEE  BSIC=%d FN=%u (sb=0x%08x)\n", sb_bsic, sb_fn, sb_word);
    else if (verdict == -1) printf("  VERDICT : ABANDON (le firmware a renonce)\n");
    else printf("  VERDICT : AUCUNE SB (ni decodee ni abandon) en %ld trames\n", trames);
    return verdict == 1 ? 0 : 1;
}
