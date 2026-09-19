/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * gmsk.c - GSM GMSK modulator (45.004) used to inject bursts into the DSP.
 *
 * Differential coding d_i = b_i XOR b_{i-1} (b_{-1} = 1), alpha_i = 1 - 2 d_i,
 * phase(t) = (pi/2) * sum_i alpha_i * q(t - iT), with q the integral of the
 * gaussian pulse BT = 0.3 truncated to 3T. Internally oversampled 16x, then one
 * sample per symbol is returned at iT + decalage*T.
 *
 * [2026-09-17] 148 zero bits give exactly +pi/2 per symbol, i.e. the FCCH: the
 * ROM FB detector locked on that waveform.
 */
#include <math.h>
#include <string.h>
#include "gmsk.h"

#define OS 16          /* internal oversampling factor */
#define L  3           /* pulse length, in symbols */
#define BT 0.3

static double q_tab[L * OS + 1];   /* q(t) for t in [-1.5T, 1.5T], step T/OS */
static int q_pret;

static double Qf(double x) { return 0.5 * erfc(x / sqrt(2.0)); }

static void q_init(void)
{
    /* g(t) = (1/T) [ Q(2 pi BT (t - T/2) / (T sqrt(ln 2))) - Q(2 pi BT (t + T/2) / (T sqrt(ln 2))) ]
     * q(t) = integral of g from -inf to t, with q(-1.5T) = 0 and q(+1.5T) = 1 (T = 1). */
    double k = 2.0 * M_PI * BT / sqrt(log(2.0));
    double acc = 0.0, dt = 1.0 / OS;
    for (int i = 0; i <= L * OS; i++) {
        double t = -1.5 + i * dt;
        double g = Qf(k * (t - 0.5)) - Qf(k * (t + 0.5));
        q_tab[i] = acc;
        acc += g * dt;
    }
    /* normalise exactly to 1 */
    for (int i = 0; i <= L * OS; i++) q_tab[i] /= acc;
    q_pret = 1;
}

static double q_de(double t)   /* t in symbols, relative to the pulse centre */
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
    int prev = 1;                       /* b_{-1} = 1 (45.004 2.2) */
    double alpha[200];
    for (int i = 0; i < n && i < 200; i++) {
        int d = (bits[i] & 1) ^ prev;
        prev = bits[i] & 1;
        alpha[i] = 1.0 - 2.0 * d;
    }
    for (int k = 0; k < n; k++) {
        double t = k + decalage;        /* sampling instant, in symbols */
        double ph = phase0;
        for (int i = 0; i < n; i++) {
            double dt = t - i;
            if (dt <= -1.5) break;      /* later symbols have not started contributing yet */
            ph += (M_PI / 2.0) * alpha[i] * q_de(dt);
        }
        iq[2 * k]     = (int16_t)lrint(amp * cos(ph));
        iq[2 * k + 1] = (int16_t)lrint(amp * sin(ph));
    }
}
