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
#include <math.h>
#include "hw/arm/calypso/calypso_debug.h"
#include <stdlib.h>
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>
#include "calypso_gmsk.h"
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

/* ---- BCCH / CCCH : normal bursts of the synthetic cell [2026-09-20] ----
 *
 * Downlink TN0 of a combined CCCH+SDCCH/4 cell (44.018 6.3.1.3, 45.002 clause
 * 7 table 3): BCCH norm on p51 2..5, CCCH on 6..9, 12..15, 16..19, SDCCH/4
 * elsewhere (left as dummy bursts). The BCCH block of 51-multiframe TC =
 * (fn / 51) % 8 carries SI1 (TC 0, 4), SI2 (1, 5), SI3 (2, 6), SI4 (3, 7);
 * the CCCH blocks carry an empty PAGING REQUEST TYPE 1. Coding is the one of
 * osmo-bts sched_lchan_xcch.c: gsm0503_xcch_encode() (Fire CRC, r=1/2
 * convolutional code, 4-burst diagonal interleaving) then 3 tail, 57 data,
 * hl, 26-bit training sequence BCC of the BSIC (45.002 5.2.3 set 1), hu, 57
 * data, 3 tail. The ROM demodulates with the TSC the ARM hands it in
 * dsp_load_rx_task(ALLC_DSP_TASK, burst_id, tsc), decodes the four bursts and
 * leaves 23 octets in a_cd[3..] with the Fire result in a_cd[0]; prim_rx_nb.c
 * copies them into an L1CTL_DATA_IND and mobile's rr reads the SI.
 *
 * Identity: MCC 001 MNC 01 LAC 1 CI 6001, ARFCN 514, the bench network of
 * /etc/osmocom (run_si.sh checks the decoded LAI against it). Override with
 * CELLULE_MCC / CELLULE_MNC / CELLULE_LAC / CELLULE_CI / CELLULE_ARFCN. */
#include <arpa/inet.h>
#include <osmocom/gsm/gsm48.h>
#include <osmocom/gsm/gsm48_ie.h>
#include <osmocom/gsm/gsm23003.h>
#include <osmocom/gsm/protocol/gsm_04_08.h>
#include <osmocom/gsm/sysinfo.h>

static const uint8_t train_nb[8][26] = {   /* 45.002 table 5.2.3a */
    { 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1 },
    { 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1 },
    { 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0 },
    { 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0 },
    { 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1 },
    { 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0 },
    { 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1 },
    { 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0 },
};

int cellule_marge_nb = -1;    /* head margin of a normal burst; < 0: bare 148 samples */
int cellule_tsc_force = -1;   /* training sequence of the normal bursts; < 0: BCC of the BSIC */
double cellule_dec_nb = -1;
double cellule_phase_nb = 0;  /* carrier phase (deg) of the last normal burst (probe) */  /* sampling instant used for the last normal burst (probe) */
int cellule_fenetre_nb = 0;   /* one-shot NB window length in samples (0: 148 + 2 x margin) */
int cellule_sans_bcch;        /* 1: dummy bursts on BCCH/CCCH, the old cell */

static int env_int(const char *nom, int defaut)
{
    const char *e = calypso_getenv(nom);
    return (e && *e) ? (int)strtol(e, NULL, 0) : defaut;
}

/* The 23 octets of the L2 frame carried by BCCH block TC (0..7), or by a CCCH
 * block (tc < 0). Buffers are static, the caller copies. */
static const uint8_t *cellule_l2(int tc)
{
    static uint8_t si1[23], si2[23], si3[23], si4[23], pag[23];
    static int pret;
    if (!pret) {
        pret = 1;
        struct osmo_location_area_id lai = {
            .plmn = { .mcc = (uint16_t)env_int("CELLULE_MCC", 1),
                      .mnc = (uint16_t)env_int("CELLULE_MNC", 1), .mnc_3_digits = false },
            .lac = (uint16_t)env_int("CELLULE_LAC", 1) };
        uint16_t ci = (uint16_t)env_int("CELLULE_CI", 6001);
        int arfcn = env_int("CELLULE_ARFCN", 514) & 0x3ff;
        struct gsm48_rach_control rach = { .re = 1, .cell_bar = 0, .tx_integer = 9, .max_trans = 3,
                                           .t2 = 0x00, .t3 = 0x00 };
        struct gsm48_cell_sel_par csp = { .ms_txpwr_max_ccch = 0, .cell_resel_hyst = 2,
                                          .rxlev_acc_min = 0, .neci = 1, .acs = 0 };
        memset(si1, GSM_MACBLOCK_PADDING, 23); memset(si2, GSM_MACBLOCK_PADDING, 23);
        memset(si3, GSM_MACBLOCK_PADDING, 23); memset(si4, GSM_MACBLOCK_PADDING, 23);
        memset(pag, GSM_MACBLOCK_PADDING, 23);

        /* SI1: cell channel description in variable bitmap format (44.018
         * 10.5.2.13.7, the layout gsm48_decode_freq_list() reads): ORIG-ARFCN
         * = our carrier, no further bit set. Rest octet 0x2b: L, no NCH; L,
         * band indicator 1800. */
        struct gsm48_system_information_type_1 *s1 = (void *)si1;
        s1->header.l2_plen = (uint8_t)((21 << 2) | 1);
        s1->header.rr_protocol_discriminator = GSM48_PDISC_RR;
        s1->header.skip_indicator = 0;
        s1->header.system_information = GSM48_MT_RR_SYSINFO_1;
        memset(s1->cell_channel_description, 0, 16);
        s1->cell_channel_description[0] = (uint8_t)(0x8e | ((arfcn >> 9) & 1));
        s1->cell_channel_description[1] = (uint8_t)((arfcn >> 1) & 0xff);
        s1->cell_channel_description[2] = (uint8_t)((arfcn & 1) << 7);
        s1->rach_control = rach;
        {   /* self-check: decode what we encoded */
            static struct gsm_sysinfo_freq f[1024];
            memset(f, 0, sizeof f);
            gsm48_decode_freq_list(f, s1->cell_channel_description, 16, 0xce, 1);
            int n = 0, ok = 0;
            for (int i = 0; i < 1024; i++) if (f[i].mask) { n++; if (i == arfcn) ok = 1; }
            if (!ok || n != 1)
                printf("cellule : SI1 cell channel description FAUSSE (%d ARFCN decodes, %d attendu %s)\n",
                       n, arfcn, ok ? "present" : "ABSENT");
        }

        /* SI2: no neighbour (bitmap 0 all clear), all NCC permitted */
        struct gsm48_system_information_type_2 *s2 = (void *)si2;
        s2->header.l2_plen = (uint8_t)((22 << 2) | 1);
        s2->header.rr_protocol_discriminator = GSM48_PDISC_RR;
        s2->header.skip_indicator = 0;
        s2->header.system_information = GSM48_MT_RR_SYSINFO_2;
        memset(s2->bcch_frequency_list, 0, 16);
        s2->ncc_permitted = 0xff;
        s2->rach_control = rach;

        /* SI3: identity, combined CCCH, IMSI attach, no periodic LU. Rest
         * octets 0x2b: every optional element absent (all L). */
        struct gsm48_system_information_type_3 *s3 = (void *)si3;
        s3->header.l2_plen = (uint8_t)((18 << 2) | 1);
        s3->header.rr_protocol_discriminator = GSM48_PDISC_RR;
        s3->header.skip_indicator = 0;
        s3->header.system_information = GSM48_MT_RR_SYSINFO_3;
        s3->cell_identity = htons(ci);
        gsm48_generate_lai2(&s3->lai, &lai);
        s3->control_channel_desc.ccch_conf = 1;        /* 1 CCCH combined with SDCCH/4 */
        s3->control_channel_desc.bs_ag_blks_res = 1;
        s3->control_channel_desc.att = 1;
        s3->control_channel_desc.bs_pa_mfrms = 0;      /* 2 multiframes */
        s3->control_channel_desc.t3212 = 0;
        s3->cell_options.radio_link_timeout = 7;       /* 32 */
        s3->cell_options.dtx = 2;
        s3->cell_options.pwrc = 0;
        s3->cell_sel_par = csp;
        s3->rach_control = rach;

        /* SI4: identity again, no CBCH; rest octets all L */
        struct gsm48_system_information_type_4 *s4 = (void *)si4;
        s4->header.l2_plen = (uint8_t)((12 << 2) | 1);
        s4->header.rr_protocol_discriminator = GSM48_PDISC_RR;
        s4->header.skip_indicator = 0;
        s4->header.system_information = GSM48_MT_RR_SYSINFO_4;
        gsm48_generate_lai2(&s4->lai, &lai);
        s4->cell_sel_par = csp;
        s4->rach_control = rach;

        /* CCCH: PAGING REQUEST TYPE 1, page mode normal, no identity (the
         * fill osmo-bts sends on an idle paging block) */
        static const uint8_t vide[] = { 0x15, 0x06, 0x21, 0x00, 0x01, 0xf0 };
        memcpy(pag, vide, sizeof vide);

        printf("cellule : BCCH SI1-4 MCC=%03u MNC=%02u LAC=%u CI=%u ARFCN=%d, CCCH = paging vide\n",
               lai.plmn.mcc, lai.plmn.mnc, lai.lac, ci, arfcn);
    }
    if (tc < 0) return pag;
    switch (tc & 3) { case 0: return si1; case 1: return si2; case 2: return si3; default: return si4; }
}

/* Normal burst bid (0..3) of the block starting at fn0, or -1 if no block
 * starts there. The four bursts of one block are cached. */
static int cellule_nb(uint32_t fn0, int bid, uint8_t bsic, uint8_t bits[148])
{
    static uint32_t fn_cache = 0xffffffffu;
    static ubit_t bursts[4 * 116];
    if (fn0 != fn_cache) {
        uint32_t p51 = fn0 % 51;
        const uint8_t *l2;
        if (p51 == 2)                                   l2 = cellule_l2((int)((fn0 / 51) % 8));
        else if (p51 == 6 || p51 == 12 || p51 == 16)    l2 = cellule_l2(-1);
        else return -1;
        gsm0503_xcch_encode(bursts, l2);
        fn_cache = fn0;
    }
    /* CELLULE_NB_REPEAT=<k>: burst k of the block in all four positions, to
     * hand the ROM's decoder four bursts it is known to demodulate. */
    { static int rep = -2; if (rep == -2) rep = env_int("CELLULE_NB_REPEAT", -1); if (rep >= 0 && rep < 4) bid = rep; }
    const ubit_t *b = bursts + bid * 116;
    memset(bits, 0, 3);
    memcpy(bits + 3, b, 58);                    /* 57 data + hl */
    memcpy(bits + 61, train_nb[cellule_tsc_force >= 0 ? cellule_tsc_force & 7 : bsic & 7], 26);
    memcpy(bits + 87, b + 58, 58);              /* hu + 57 data */
    memset(bits + 145, 0, 3);
    return 0;
}

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
            if (inv == -2) { const char *e = calypso_getenv("REJEU_INVERSER_BIT"); inv = e ? atoi(e) : -1; }
            if (invfn == -2) { const char *e = calypso_getenv("REJEU_INVERSER_FN"); invfn = e ? atol(e) : -1; }
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
            if (im == -2) { const char *e = calypso_getenv("REJEU_INVERSER_MIDAMBULE");
                            im = e ? atoi(e) : -1; }
            if (imfn == -2) { const char *e = calypso_getenv("REJEU_INVERSER_FN");
                              imfn = e ? atol(e) : -1; }
            if (im >= 0 && im < 64 && (imfn < 0 || (long)fn == imfn)) bits[42 + im] ^= 1;
        }
        type = 'S';
    } else if (!cellule_sans_bcch && p51 >= 2 && p51 <= 49 && p51 % 10 >= 2 &&
               cellule_nb(fn - ((p51 % 10 - 2) & 3), (int)((p51 % 10 - 2) & 3), bsic, bits) == 0) {
        type = (p51 <= 5) ? 'B' : 'C';
    } else {
        memcpy(bits, factice, 148);
        type = '.';
    }
    /* CELLULE_NB_FINE=1 : burst position swept from 2.3 to 4.7 samples in 0.1
     * steps by multiframe (head margin + sampling instant together) */
    if (type == 'B' || type == 'C') {
        static int fine = -2; if (fine == -2) fine = env_int("CELLULE_NB_FINE", 0);
        if (fine && cellule_marge_nb >= 0) {
            double total = 2.3 + 0.1 * (double)((fn / 51u) % 25u);
            cellule_marge_nb = (int)total; decalage = total - (double)cellule_marge_nb;
            cellule_dec_nb = decalage;
        }
    }
    int m_tete = -1, m_fin = 0;
    if (type == 'S' && marge > 0) {
        m_tete = marge;
        m_fin = (cellule_marge_fin >= 0) ? cellule_marge_fin : marge;
    } else if ((type == 'B' || type == 'C') && cellule_marge_nb >= 0) {
        /* the ROM takes 151 samples for an NB window (ALGTH 604): 3 + 148 */
        /* exactly the window: a frame longer than the DMA window leaves
         * samples in the RIF and the next burst starts on them */
        m_tete = cellule_marge_nb;
        m_fin = cellule_fenetre_nb > m_tete + 148 ? cellule_fenetre_nb - m_tete - 148 : 0;
    }
    /* CELLULE_NB_AMP=<n>: amplitude of the normal bursts alone (FCCH/SCH keep
     * amp), to probe the ROM's fixed-point headroom on the NB path. */
    if (type == 'B' || type == 'C') { static int nb_amp = -2; if (nb_amp == -2) nb_amp = env_int("CELLULE_NB_AMP", -1); if (nb_amp > 0) amp = nb_amp; }
    /* CELLULE_NB_DEC=<x>|auto : sampling instant of the normal bursts alone
     * (FCCH/SCH keep decalage); auto sweeps 0, 0.25, 0.5, 0.75 by multiframe */
    if (type == 'B' || type == 'C') {
        static int mode = -2; static double d = 0;
        if (mode == -2) { const char *e = calypso_getenv("CELLULE_NB_DEC"); mode = 0;
                          if (e && !strcmp(e, "auto")) mode = 2; else if (e && *e) { mode = 1; d = atof(e); } }
        if (mode == 1) decalage = d; else if (mode == 2) decalage = 0.25 * (double)((fn / 51u) % 4u);
        cellule_dec_nb = decalage;
    }
    /* CELLULE_NB_PHASE=<deg>|auto : carrier phase of the normal bursts (auto:
     * 0, 22.5, 45, 67.5 degrees by multiframe). Samples exactly on the I/Q
     * axes (decalage 0, phase 0) or exactly on the diagonals (decalage 0.5)
     * are what a synthetic GMSK gives and what no radio ever gives. */
    double phase0 = 0.0;
    if (type == 'B' || type == 'C') {
        static int pm = -2; static double pd = 0;
        if (pm == -2) { const char *e = calypso_getenv("CELLULE_NB_PHASE"); pm = 0;
                        if (e && !strcmp(e, "auto")) pm = 2; else if (e && *e) { pm = 1; pd = atof(e); } }
        if (pm == 1) phase0 = pd * M_PI / 180.0; else if (pm == 2) phase0 = 22.5 * (double)((fn / 51u) % 4u) * M_PI / 180.0;
        /* CELLULE_NB_PHASE=quad : 0, 90, 180, 270 degrees by multiframe */
        { static int q = -1; if (q < 0) { const char *e = calypso_getenv("CELLULE_NB_PHASE"); q = (e && !strcmp(e, "quad")) ? 1 : 0; }
          if (q) phase0 = 90.0 * (double)((fn / 51u) % 4u) * M_PI / 180.0; }
        cellule_phase_nb = phase0 * 180.0 / M_PI;
    }
    /* CELLULE_NB_MSK=1 (experiment): pure MSK for the normal bursts, no
     * Gaussian filter: the phase advances linearly by +-90 degrees per bit,
     * so a sample taken at decalage 0.5 sits EXACTLY on a diagonal, with
     * |I| = |Q|. Tests whether the ROM's NB demodulator hard-decides on the
     * signs of I and Q (measured: only decalage 0.5 ever works, 0.4 and 0.6
     * fail on every burst, an equaliser would degrade smoothly). */
    static int nb_msk = -2; if (nb_msk == -2) nb_msk = env_int("CELLULE_NB_MSK", 0);
    if (nb_msk && (type == 'B' || type == 'C')) {
        int16_t *o = iq + 2 * (m_tete >= 0 ? m_tete : 0);
        double ph = phase0; int prev = 1;
        for (int k = 0; k < 148; k++) {
            int d = (bits[k] & 1) ^ prev; prev = bits[k] & 1;
            double al = 1.0 - 2.0 * d;
            double pk = ph + al * (M_PI / 2.0) * decalage;     /* phase at the sampling instant */
            o[2*k] = (int16_t)lrint(amp * cos(pk)); o[2*k+1] = (int16_t)lrint(amp * sin(pk));
            ph += al * (M_PI / 2.0);
        }
        if (m_tete >= 0) {
            memset(iq, 0, (size_t)m_tete * 2 * sizeof(int16_t));
            memset(iq + 2 * (m_tete + 148), 0, (size_t)m_fin * 2 * sizeof(int16_t));
            *n_iq = 2 * (148 + m_tete + m_fin);
        } else *n_iq = 2 * 148;
        return type;
    }
    /* CELLULE_SB_PHASE=<deg> : carrier phase of the SCH burst */
    if (type == 'S') { static int sp = -2; if (sp == -2) sp = env_int("CELLULE_SB_PHASE", 0); phase0 = sp * M_PI / 180.0; }
    int16_t *burst_iq = iq;
    if (m_tete >= 0) {
        memset(iq, 0, (size_t)m_tete * 2 * sizeof(int16_t));
        gmsk_moduler(bits, 148, amp, phase0, decalage, iq + 2 * m_tete);
        memset(iq + 2 * (m_tete + 148), 0, (size_t)m_fin * 2 * sizeof(int16_t));
        *n_iq = 2 * (148 + m_tete + m_fin);
        burst_iq = iq + 2 * m_tete;
    } else {
        gmsk_moduler(bits, 148, amp, phase0, decalage, iq);
        *n_iq = 2 * 148;
    }
    /* [2026-09-21] CELLULE_NB_ISI=<h1>[,<h2>[,<h3>]] : causal tail on the normal
     * bursts, y[n] = x[n] + h1 x[n-1] + h2 x[n-2] + h3 x[n-3]. Why: the ROM
     * correlates the 16 central TSC bits over 10 lags, then picks the 7-lag
     * window of maximum energy (0x8551, sliding sum) and cuts its 5-tap
     * channel estimate from it. Our GMSK at 1 sample/symbol has energy on 3
     * lags only (main + the +-1 ISI), so three of the four windows tie to 0.1 %
     * and the choice is decided by e[lag 8] against e[lag 1] = the GMSK +-2
     * tap (912) against the data leakage: measured over 9 bursts, the window
     * was right (s=2) exactly when e[8] > e[1]. A receiver's analogue filter
     * spreads energy over the following lags and settles it; this tail does
     * the same for the synthetic cell. */
    if (type == 'B' || type == 'C') {
        static int isi_n = -2; static double h[4];
        if (isi_n == -2) { isi_n = 0; const char *e = calypso_getenv("CELLULE_NB_ISI");
            if (e && *e) { char tmp[64]; strncpy(tmp, e, sizeof tmp - 1); tmp[sizeof tmp - 1] = 0;
                for (char *t = strtok(tmp, ","); t && isi_n < 3; t = strtok(NULL, ",")) h[++isi_n] = atof(t); } }
        if (isi_n > 0) {
            double xi[148], xq[148];
            for (int k = 0; k < 148; k++) { xi[k] = burst_iq[2*k]; xq[k] = burst_iq[2*k+1]; }
            for (int k = 0; k < 148; k++) {
                double yi = xi[k], yq = xq[k];
                for (int d = 1; d <= isi_n; d++) if (k - d >= 0) { yi += h[d] * xi[k-d]; yq += h[d] * xq[k-d]; }
                if (yi > 32767) yi = 32767; if (yi < -32768) yi = -32768; if (yq > 32767) yq = 32767; if (yq < -32768) yq = -32768;
                burst_iq[2*k] = (int16_t)lrint(yi); burst_iq[2*k+1] = (int16_t)lrint(yq);
            }
        }
    }
    /* [2026-09-21] CELLULE_NB_SYM=<a> : symmetric spread y[n] = x[n] + a (x[n-1]
     * + x[n+1]) on the normal bursts, timing unchanged. Why: the ROM zeroes
     * every channel tap whose energy is below 1/16 of the window energy
     * (0x7f0c-0x7f1c, threshold = total >> 4, i.e. 25 % in amplitude). The
     * +-1 taps of our GMSK at 1 sample/symbol are 25 % (6467..7189 against a
     * main tap of 27000): measured, the pre-cursor tap was kept (e = 4.70e7 >
     * 4.50e7) on the bursts that decoded and zeroed (4.18e7 < 4.25e7) on the
     * ones that did not, and a 5-tap model without its 25 % pre-cursor gives
     * 45 % errors. A receiver's channel filter widens the pulse; a = 0.3 puts
     * the +-1 taps near 55 % and the +-2 taps near 10 %, both far from the
     * threshold, whatever the data. */
    if (type == 'B' || type == 'C') {
        static double a = -2; if (a == -2) { const char *e = calypso_getenv("CELLULE_NB_SYM"); a = (e && *e) ? atof(e) : 0.0; }
        if (a != 0.0) {
            double xi[148], xq[148];
            for (int k = 0; k < 148; k++) { xi[k] = burst_iq[2*k]; xq[k] = burst_iq[2*k+1]; }
            for (int k = 0; k < 148; k++) {
                double yi = xi[k], yq = xq[k];
                if (k > 0)   { yi += a * xi[k-1]; yq += a * xq[k-1]; }
                if (k < 147) { yi += a * xi[k+1]; yq += a * xq[k+1]; }
                yi /= (1 + 2 * a); yq /= (1 + 2 * a);   /* keep the peak amplitude */
                burst_iq[2*k] = (int16_t)lrint(yi); burst_iq[2*k+1] = (int16_t)lrint(yq);
            }
        }
    }
    /* [2026-09-21] CELLULE_NB_NOISE=<sigma> : Gaussian noise on the normal
     * bursts (deterministic seed per frame). Why: the ROM scales its soft bits
     * by a noise estimate before the 4-bit quantiser (0x8168 -> 0x82d0, a
     * 129-entry table indexed by soft >> 8); with a noiseless burst the scaled
     * values are 2..5 instead of thousands, every index is 0 and all 116
     * quantised soft bits come out +1 (measured in the storage at 0x4200 +
     * 29 x burst): the sign is lost before the deinterleaver. A radio always
     * carries noise; the cell now does too. */
    if (type == 'B' || type == 'C') {
        static double sigma = -2; if (sigma == -2) { const char *e = calypso_getenv("CELLULE_NB_NOISE"); sigma = (e && *e) ? atof(e) : 0.0; }
        if (sigma > 0) {
            uint32_t seed = fn * 2654435761u + 12345u;
            for (int k = 0; k < 296; k++) {
                /* Box-Muller on a small LCG */
                seed = seed * 1103515245u + 12345u; double u1 = ((seed >> 8) & 0xffff) / 65536.0 + 1e-6;
                seed = seed * 1103515245u + 12345u; double u2 = ((seed >> 8) & 0xffff) / 65536.0;
                double g = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
                double v = burst_iq[k] + sigma * g;
                if (v > 32767) v = 32767; if (v < -32768) v = -32768;
                burst_iq[k] = (int16_t)lrint(v);
            }
        }
    }
    /* [2026-09-21] CELLULE_NB_ZERO_DC=1 (experiment): remove the mean of the
     * 148 samples of a normal burst. Measured: the ROM demodulates a normal
     * burst perfectly when the data-induced mean of its samples is below ~7 %
     * of the amplitude and loses it entirely above ~10 % (16 bursts of SI1-4,
     * no exception), the SCH being immune. */
    if (type == 'B' || type == 'C') {
        static int zdc = -2; if (zdc == -2) zdc = env_int("CELLULE_NB_ZERO_DC", 0);
        if (zdc) {
            long si = 0, sq = 0;
            for (int k = 0; k < 148; k++) { si += burst_iq[2*k]; sq += burst_iq[2*k+1]; }
            int mi = (int)(si / 148), mq = (int)(sq / 148);
            for (int k = 0; k < 148; k++) { burst_iq[2*k] = (int16_t)(burst_iq[2*k] - mi); burst_iq[2*k+1] = (int16_t)(burst_iq[2*k+1] - mq); }
        }
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

/* The 148 bits of the TN0 burst of frame fn as cellule_burst() sends them, for
 * probes that look for them inside the DSP memory. 0 if it is a normal burst. */
int cellule_bits_attendus(uint32_t fn, uint8_t bsic, uint8_t bits[148])
{
    uint32_t p51 = fn % 51;
    if (cellule_sans_bcch || p51 < 2 || p51 > 49 || p51 % 10 < 2) return -1;
    return cellule_nb(fn - ((p51 % 10 - 2) & 3), (int)((p51 % 10 - 2) & 3), bsic, bits);
}

/* Expected decoder-side vectors for the block that ends at frame fn (burst 3):
 * the 456 convolutionally coded bits in deinterleaved order (45.003 4.1.4:
 * bit k of the coded block sits in burst k mod 4 at position 2*((49k) mod 57)
 * + ((k mod 8) div 4) of the 114 data bits) and the 184 information bits +
 * 40 parity + 4 tail (the 23 octets as sent, MSB first, then the Fire parity
 * as gsm0503 computes it: we only need the 184 here). */
int cellule_bloc_attendu(uint32_t fn, uint8_t bsic, uint8_t code456[456], uint8_t info184[184])
{
    uint32_t p51 = fn % 51;
    if (cellule_sans_bcch || p51 < 2 || p51 > 49 || p51 % 10 < 2) return -1;
    uint32_t fn0 = fn - ((p51 % 10 - 2) & 3);
    uint8_t bits[148]; uint8_t data[4][114];
    for (int b = 0; b < 4; b++) {
        if (cellule_nb(fn0, b, bsic, bits) < 0) return -1;
        memcpy(data[b], bits + 3, 57); memcpy(data[b] + 57, bits + 88, 57);
    }
    for (int k = 0; k < 456; k++) {
        int b = k & 3, j = 2 * ((49 * k) % 57) + ((k % 8) / 4);
        code456[k] = data[b][j];
    }
    uint32_t tc = (fn0 / 51) % 8;
    const uint8_t *l2 = (p51 <= 5) ? cellule_l2((int)tc) : cellule_l2(-1);
    for (int i = 0; i < 184; i++) info184[i] = (l2[i / 8] >> (7 - (i % 8))) & 1;
    return 0;
}

/* The 228 bits the Viterbi decoder must output: 184 information bits, 40 Fire
 * parity bits, 4 tail bits (45.003 4.1.1-4.1.2). */
#include <osmocom/core/crc64gen.h>
#include <osmocom/coding/gsm0503_parity.h>
int cellule_u228_attendu(uint32_t fn, uint8_t bsic, uint8_t u228[228])
{
    uint8_t code[456], info[184];
    if (cellule_bloc_attendu(fn, bsic, code, info) < 0) return -1;
    memcpy(u228, info, 184);
    osmo_crc64gen_set_bits(&gsm0503_fire_crc40, info, 184, u228 + 184);
    memset(u228 + 224, 0, 4);
    return 0;
}
