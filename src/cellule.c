/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * cellule.c - cell synchronisation bursts for the DSP.
 *
 * FCCH = 148 zero bits (45.002 5.2.4). SCH = 3 tail + 39 + 64 extended training
 * sequence (45.002 5.2.5) + 39 + 3 tail; the 78 coded bits come from
 * gsm0503_sch_encode() of libosmocoding fed with sb_info (BSIC + T1'/T2/T3',
 * 44.018 9.1.30). Dummy burst: the fixed pattern of 45.002 5.2.6. Burst
 * assembly follows osmo-bts sched_lchan_fcch_sch.c and scheduler.c.
 */
#include <string.h>
#include <stdlib.h>
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>
#include "gmsk.h"
#include "cellule.h"

static const uint8_t train_sb[64] = {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};
static const uint8_t factice[148] = {
    0,0,0,
    1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
    0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
    0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
    0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
    0,0,1,0,1,1,1,1,1,0,1,0,1,0,
    0,0,0,
};

int cellule_sch_partout;      /* diagnostic: emit SCH on every non-FCCH frame */

/* sb_info of 44.018 9.1.30 for frame fn, byte layout of osmo-bts
 * sched_lchan_fcch_sch.c. T3' = (T3 - 1) / 10: the SCH sits on T3 in
 * {1,11,21,31,41}, so T3' in 0..4. Previously computed as T3 / 10 here and as
 * (T3 - 1) / 10 in pont.c: identical on real SCH frames, different at T3 = 50
 * under cellule_sch_partout. One encoder now, shared by cellule_burst() and
 * cellule_code_attendu(). */
static void sb_info_de_fn(uint32_t fn, uint8_t bsic, uint8_t sb_info[4])
{
    uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = fn % 51;
    uint32_t t3p = t3 ? (t3 - 1) / 10 : 0;
    sb_info[0] = (uint8_t)(((bsic & 0x3f) << 2) | ((t1 & 0x600) >> 9));
    sb_info[1] = (uint8_t)((t1 & 0x1fe) >> 1);
    sb_info[2] = (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1));
    sb_info[3] = (uint8_t)(t3p & 0x1);
}
/* Caller sets the head margin; the tail margin is trimmed to keep 190 complex
 * samples in total. Sliding the burst inside that fixed-size buffer separates an
 * influence window that is an ABSOLUTE buffer index (late bits lose influence,
 * early ones gain it) from one RELATIVE to the burst start (the profile moves
 * with the burst and keeps its shape). */
int cellule_marge_fin = -1;

char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq)
{
    uint8_t bits[148];
    uint32_t p51 = fn % 51;
    char type;
    if (p51 % 10 == 0 && p51 <= 40) {
        memset(bits, 0, sizeof(bits));
        type = 'F';
    } else if ((p51 % 10 == 1 && p51 <= 41) || cellule_sch_partout) {
        uint8_t sb_info[4];
        sb_info_de_fn(fn, bsic, sb_info);
        ubit_t code[78];
        gsm0503_sch_encode(code, sb_info);
        /* Demodulator impulse response: flip exactly ONE of the 78 coded bits and
         * watch which soft-bit positions at 0x2c72 move.
         *   1 position          -> pure permutation, 78 runs give the whole table
         *   3 or 4 around one   -> normal ISI, the chain is sound there
         *   all of them         -> global, wrong traceback (on C54x: CMPS/TRN) */
        {
            /* The flip must stay inside ONE frame: otherwise it perturbs the whole
             * run history (scheduling, AFC) and the differential measurement then
             * compares two histories instead of two bursts.
             * REJEU_INVERSER_FN=<f> restricts the flip to frame f. */
            static int inv = -2; static long invfn = -2;
            if (inv == -2) { const char *e = getenv("REJEU_INVERSER_BIT"); inv = e ? atoi(e) : -1; }
            if (invfn == -2) { const char *e = getenv("REJEU_INVERSER_FN"); invfn = e ? atol(e) : -1; }
            if (inv >= 0 && inv < 78 && (invfn < 0 || (long)fn == invfn)) code[inv] ^= 1;
        }
        memset(bits, 0, 3);
        memcpy(bits + 3, code, 39);
        memcpy(bits + 42, train_sb, 64);
        memcpy(bits + 106, code + 39, 39);
        memset(bits + 145, 0, 3);
        /* Midamble probe: the 78 coded bits cannot reach the training sequence, yet
         * that sequence is what channel estimation is supposed to use. If flipping
         * one midamble bit leaves the soft bits untouched, the demodulator ignores
         * the known sequence and equalises against an estimate built elsewhere. */
        {
            /* Same one-frame confinement as the coded-bit flip; without it every SCH
             * burst of the run is hit. REJEU_INVERSER_FN is shared with
             * REJEU_INVERSER_BIT, so only one thing is probed at a time. */
            static int im = -2; static long imfn = -2;
            if (im == -2) { const char *e = getenv("REJEU_INVERSER_MIDAMBULE");
                            im = e ? atoi(e) : -1; }
            if (imfn == -2) { const char *e = getenv("REJEU_INVERSER_FN");
                              imfn = e ? atol(e) : -1; }
            if (im >= 0 && im < 64 && (imfn < 0 || (long)fn == imfn)) bits[42 + im] ^= 1;
        }
        type = 'S';
    } else {
        memcpy(bits, factice, 148);
        type = '.';
    }
    if (type == 'S' && marge > 0) {
        int mf = (cellule_marge_fin >= 0) ? cellule_marge_fin : marge;
        memset(iq, 0, (size_t)marge * 2 * sizeof(int16_t));
        gmsk_moduler(bits, 148, amp, 0.0, decalage, iq + 2 * marge);
        memset(iq + 2 * (marge + 148), 0, (size_t)mf * 2 * sizeof(int16_t));
        *n_iq = 2 * (148 + marge + mf);
    } else {
        gmsk_moduler(bits, 148, amp, 0.0, decalage, iq);
        *n_iq = 2 * 148;
    }
    return type;
}

/* Expected 78 coded bits for a frame, to compare sign by sign with the soft bits
 * the DSP produces. */
void cellule_code_attendu(uint32_t fn, uint8_t bsic, unsigned char *code78)
{
    uint8_t sb_info[4];
    sb_info_de_fn(fn, bsic, sb_info);
    ubit_t code[78];
    gsm0503_sch_encode(code, sb_info);
    for (int i = 0; i < 78; i++) code78[i] = (unsigned char)code[i];
}

/* Reference demodulator, in C, to prove the samples handed to the DSP do carry the
 * message. GMSK at 1 sample/symbol: the phase advances by (pi/2)*alpha_n per symbol,
 * so alpha_n = sign(arg(x[n] * conj(x[n-1]))). Recover the alpha sequence, correlate
 * it with the midamble to find the offset, then read the data bits from there. */
#include <math.h>
int cellule_demod_reference(const int16_t *x, int n_ech, unsigned char *bits148, int *offset)
{
    static double alpha[512];
    if (n_ech > 512) n_ech = 512;
    for (int n = 1; n < n_ech; n++) {
        double i0 = x[2*(n-1)], q0 = x[2*(n-1)+1];
        double i1 = x[2*n],     q1 = x[2*n+1];
        /* x[n] * conj(x[n-1]) */
        double re = i1*i0 + q1*q0, im = q1*i0 - i1*q0;
        alpha[n] = atan2(im, re);
    }
    alpha[0] = 0;
    /* reference alpha of the midamble: d_j = t_j XOR t_{j-1}, burst positions 43..105 */
    double ref[63];
    for (int j = 1; j < 64; j++) ref[j-1] = (train_sb[j] ^ train_sb[j-1]) ? -1.0 : 1.0;
    int best = -1; double bestv = -1e30;
    for (int k = 0; k + 63 < n_ech; k++) {
        double acc = 0;
        for (int j = 0; j < 63; j++) acc += ref[j] * alpha[k + j];
        if (acc > bestv) { bestv = acc; best = k; }
    }
    if (best < 0) return -1;
    /* the midamble starts 43 symbols into the burst */
    int b0 = best - 43;
    *offset = b0;
    if (b0 < 0 || b0 + 148 > n_ech) return -1;
    int prev = 1;
    for (int i = 0; i < 148; i++) {
        int d = (alpha[b0 + i] < 0) ? 1 : 0;   /* alpha = 1-2d */
        bits148[i] = (unsigned char)(d ^ prev);
        prev = bits148[i];
    }
    return 0;
}

/* Raw DIFFERENTIAL bits d_i, with no differential decoding: an isolated error does
 * not propagate here, unlike in b_i = d_i XOR b_{i-1}. This is the honest measure of
 * demodulator quality. */
int cellule_demod_d(const int16_t *x, int n_ech, int b0, unsigned char *d148)
{
    if (b0 < 1 || b0 + 148 > n_ech) return -1;
    for (int i = 0; i < 148; i++) {
        int n = b0 + i;
        double i0 = x[2*(n-1)], q0 = x[2*(n-1)+1];
        double i1 = x[2*n],     q1 = x[2*n+1];
        double im = q1*i0 - i1*q0, re = i1*i0 + q1*q0;
        d148[i] = (unsigned char)(atan2(im, re) < 0 ? 1 : 0);
    }
    return 0;
}

int cellule_train_sb(int i) { return (i >= 0 && i < 64) ? train_sb[i] : -1; }

/* The dummy burst (45.002 5.2.6) as 148 GMSK samples: what the BCCH carrier
 * transmits on every timeslot that carries nothing else. The FB search of the
 * ROM receives the whole frame, so the seven other timeslots must look like a
 * real C0 carrier, not like silence. */
void cellule_factice(int amp, double decalage, int16_t *iq)
{
    gmsk_moduler(factice, 148, amp, 0.0, decalage, iq);
}
