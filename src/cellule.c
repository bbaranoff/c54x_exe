/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * cellule.c - les bursts de synchronisation d'une cellule, pour le DSP.
 *
 * [2026-09-17] FCCH = 148 zeros (45.002 §5.2.4). SCH = 3 tail + 39 + 64 (ETS,
 * 45.002 §5.2.5) + 39 + 3 tail, les 78 bits codes par gsm0503_sch_encode()
 * de libosmocoding a partir de sb_info (BSIC + T1'/T2/T3', 44.018 §9.1.30),
 * assemblage repris de osmo-bts sched_lchan_fcch_sch.c. Burst factice : le
 * motif fixe de 45.002 §5.2.6, repris de osmo-bts scheduler.c.
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

int cellule_sch_partout;
/* [2026-09-18] Marge de tete reglable, marge de queue ajustee pour garder 190
 * complexes au total. Sert a la contre-epreuve : si la fenetre d'influence est un
 * INDEX ABSOLU dans le tampon, decaler le burst fait perdre de l'influence aux bits
 * tardifs et en fait gagner aux precoces ; si c'est une fenetre RELATIVE au debut du
 * burst, le profil se deplace avec lui sans changer de forme. */
int cellule_marge_fin = -1;   /* diagnostic : SCH sur toutes les trames non-FCCH */

char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq)
{
    uint8_t bits[148];
    uint32_t p51 = fn % 51;
    char type;
    if (p51 % 10 == 0 && p51 <= 40) {
        memset(bits, 0, sizeof(bits));
        type = 'F';
    } else if ((p51 % 10 == 1 && p51 <= 41) || cellule_sch_partout) {
        uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = p51, t3p = t3 / 10;
        uint8_t sb_info[4] = {
            (uint8_t)(((bsic & 0x3f) << 2) | ((t1 & 0x600) >> 9)),
            (uint8_t)((t1 & 0x1fe) >> 1),
            (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1)),
            (uint8_t)(t3p & 0x1),
        };
        ubit_t code[78];
        gsm0503_sch_encode(code, sb_info);
        /* [2026-09-18] REPONSE IMPULSIONNELLE DU DEMODULATEUR. On inverse UN seul des
         * 78 bits codes et on regarde quelles positions de 0x2c72 bougent :
         *   1 position           -> permutation, 78 runs donnent la table complete
         *   3 ou 4 autour d'une  -> ISI normale, la chaine est saine a cet endroit
         *   tout                 -> traceback globale et fausse (sur C54x : CMPS/TRN) */
        {
            /* [2026-09-18] L'inversion doit etre confinee a UNE trame, sinon elle
             * modifie tout l'historique du run (ordonnancement, AFC) et la mesure
             * differentielle compare deux histoires, pas deux bursts.
             * REJEU_INVERSER_FN=<f> restreint l'inversion a la trame f. */
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
        /* [2026-09-18] Perturber le MIDAMBULE. Les 78 bits codes ne permettent pas
         * de sonder la sequence d'apprentissage, or c'est elle qui doit servir a
         * l'estimation de canal. Si inverser un bit du midambule ne change RIEN aux
         * bits souples, le demodulateur n'utilise pas la sequence connue, et son
         * egaliseur travaille sur une estimation batie ailleurs : cela expliquerait
         * une sortie non correlee a l'entree malgre des echantillons parfaits. */
        {
            static int im = -2;
            if (im == -2) { const char *e = getenv("REJEU_INVERSER_MIDAMBULE");
                            im = e ? atoi(e) : -1; }
            if (im >= 0 && im < 64) bits[42 + im] ^= 1;
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

/* [2026-09-17] Diagnostic defaut B : les 78 bits codes attendus pour une trame,
 * afin de comparer signe a signe avec les bits souples produits par le DSP. */
void cellule_code_attendu(uint32_t fn, uint8_t bsic, unsigned char *code78)
{
    uint32_t p51 = fn % 51;
    uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = p51, t3p = t3 / 10;
    uint8_t sb_info[4] = {
        (uint8_t)(((bsic & 0x3f) << 2) | ((t1 & 0x600) >> 9)),
        (uint8_t)((t1 & 0x1fe) >> 1),
        (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1)),
        (uint8_t)(t3p & 0x1),
    };
    ubit_t code[78];
    gsm0503_sch_encode(code, sb_info);
    for (int i = 0; i < 78; i++) code78[i] = (unsigned char)code[i];
}

/* [2026-09-17] Demodulateur de reference, en C, pour prouver que les echantillons
 * livres au DSP portent bien le message. GMSK a 1 echantillon/symbole : la phase
 * avance de (pi/2)*alpha_n par symbole, donc alpha_n = signe(angle(x[n] x[n-1]*)).
 * On recupere la sequence alpha, on la correle au midambule pour trouver l'offset,
 * puis on relit les 78 bits de donnees a cet offset. */
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
    /* alpha de reference du midambule : d_j = t_j XOR t_{j-1}, positions 43..105 */
    double ref[63];
    for (int j = 1; j < 64; j++) ref[j-1] = (train_sb[j] ^ train_sb[j-1]) ? -1.0 : 1.0;
    int best = -1; double bestv = -1e30;
    for (int k = 0; k + 63 < n_ech; k++) {
        double acc = 0;
        for (int j = 0; j < 63; j++) acc += ref[j] * alpha[k + j];
        if (acc > bestv) { bestv = acc; best = k; }
    }
    if (best < 0) return -1;
    /* le midambule commence au symbole best-43 dans le burst */
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

/* Les bits DIFFERENTIELS bruts d_i, sans decodage differentiel : une erreur isolee
 * n'y propage pas, contrairement a b_i = d_i XOR b_{i-1}. C'est la vraie mesure de
 * la qualite du demodulateur. */
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
