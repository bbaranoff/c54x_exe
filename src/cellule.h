/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CELLULE_H
#define CELLULE_H
#include <stdint.h>
/* Downlink TN0 burst of a GSM cell for frame fn (51-multiframe: FCCH on
 * 0/10/20/30/40, SCH on 1/11/21/31/41, dummy burst elsewhere), GMSK modulated,
 * 148 int16 I/Q samples. Returns the burst type: 'F', 'S' or '.'.
 * marge = silence samples prepended and appended to an SCH burst: the ROM SB
 * decoder reads a 190-sample window, 148 + 2 x 21 (BSP_IQ_MAX_I16 in
 * calypso_bsp.c, "SB en demande 190"). *n_iq gets the number of int16 written. */
extern int cellule_sch_partout;
extern int cellule_marge_fin;
int cellule_train_sb(int i);
int cellule_demod_d(const int16_t *x, int n_ech, int b0, unsigned char *d148);
int cellule_demod_reference(const int16_t *x, int n_ech, unsigned char *bits148, int *offset);
void cellule_code_attendu(uint32_t fn, uint8_t bsic, unsigned char *code78);
char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq);
void cellule_factice(int amp, double decalage, int16_t *iq);   /* 148 GMSK samples of the dummy burst */
#endif
