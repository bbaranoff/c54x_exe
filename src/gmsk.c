/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gmsk.c - modulateur GMSK GSM (45.004) pour l'injection de bursts dans le DSP.
 *
 * [2026-09-17] Codage differentiel d_i = b_i XOR b_{i-1} (b_{-1} = 1), alpha_i
 * = 1 - 2 d_i, phase(t) = (pi/2) * somme_i alpha_i * q(t - iT) avec q la
 * primitive de l'impulsion gaussienne BT = 0,3 tronquee a 3T. Sur-echantillonne
 * a 16 par symbole en interne, puis rend un echantillon par symbole a l'instant
 * iT + decalage*T. Une suite de 148 zeros donne exactement +pi/2 par symbole :
 * c'est la FCCH, et c'est ce que le detecteur FB de la ROM a reconnu le 17/09.
 */
#include <math.h>
#include <string.h>
#include "gmsk.h"

#define OS 16          /* sur-echantillonnage interne */
#define L  3           /* longueur de l'impulsion, en symboles */
#define BT 0.3

static double q_tab[L * OS + 1];   /* q(t) pour t dans [-1.5T, 1.5T], pas T/OS */
static int q_pret;

static double Qf(double x) { return 0.5 * erfc(x / sqrt(2.0)); }

static void q_init(void)
{
    /* g(t) = (1/T) [ Q(2 pi BT (t - T/2) / (T sqrt(ln 2))) - Q(2 pi BT (t + T/2) / (T sqrt(ln 2))) ]
     * q(t) = integrale de g de -inf a t, avec q(-1.5T) = 0 et q(+1.5T) = 1 (T = 1). */
    double k = 2.0 * M_PI * BT / sqrt(log(2.0));
    double acc = 0.0, dt = 1.0 / OS;
    for (int i = 0; i <= L * OS; i++) {
        double t = -1.5 + i * dt;
        double g = Qf(k * (t - 0.5)) - Qf(k * (t + 0.5));
        q_tab[i] = acc;
        acc += g * dt;
    }
    /* normalisation exacte a 1 */
    for (int i = 0; i <= L * OS; i++) q_tab[i] /= acc;
    q_pret = 1;
}

static double q_de(double t)   /* t en symboles, relatif au centre de l'impulsion */
{
    if (t <= -1.5) return 0.0;
    if (t >= 1.5) return 1.0;
    double x = (t + 1.5) * OS;
    int i = (int)x; double f = x - i;
    if (i >= L * OS) return 1.0;
    return q_tab[i] * (1 - f) + q_tab[i + 1] * f;
}

void gmsk_moduler(const uint8_t *bits, int n, int amp, double phase0, double decalage, int16_t *iq)
{
    if (!q_pret) q_init();
    int prev = 1;                       /* b_{-1} = 1 (45.004 §2.2) */
    double alpha[200];
    for (int i = 0; i < n && i < 200; i++) {
        int d = (bits[i] & 1) ^ prev;
        prev = bits[i] & 1;
        alpha[i] = 1.0 - 2.0 * d;
    }
    for (int k = 0; k < n; k++) {
        double t = k + decalage;        /* instant d'echantillonnage, en symboles */
        double ph = phase0;
        for (int i = 0; i < n; i++) {
            double dt = t - i;
            if (dt <= -1.5) break;      /* les symboles suivants n'ont pas encore commence */
            ph += (M_PI / 2.0) * alpha[i] * q_de(dt);
        }
        iq[2 * k]     = (int16_t)lrint(amp * cos(ph));
        iq[2 * k + 1] = (int16_t)lrint(amp * sin(ph));
    }
}
