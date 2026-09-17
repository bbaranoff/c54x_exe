/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef GMSK_H
#define GMSK_H
#include <stdint.h>
/* Module 148 bits en GMSK BT=0,3 (3GPP 45.004), 1 echantillon complexe par
 * symbole, int16 I,Q entrelaces. phase0 = phase initiale (rad) ; decalage =
 * instant d'echantillonnage dans le symbole, en fraction [0,1[ (0,5 = centre). */
void gmsk_moduler(const uint8_t *bits, int n, int amp, double phase0, double decalage, int16_t *iq);
#endif
