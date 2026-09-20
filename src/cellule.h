/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CELLULE_H
#define CELLULE_H
#include <stdint.h>
/* Downlink TN0 burst of a GSM cell for frame fn (51-multiframe: FCCH on
 * 0/10/20/30/40, SCH on 1/11/21/31/41, dummy burst elsewhere), GMSK modulated,
 * 148 int16 I/Q samples. Returns the burst type: 'F', 'S', 'B' (BCCH, SI1-4),
 * 'C' (CCCH, empty paging) or '.' (dummy).
 * marge = silence samples prepended and appended to an SCH burst: the ROM SB
 * decoder reads a 190-sample window, 148 + 2 x 21 (BSP_IQ_MAX_I16 in
 * calypso_bsp.c, "SB en demande 190"). *n_iq gets the number of int16 written. */
extern int cellule_sch_partout;
extern int cellule_marge_fin;
extern int cellule_marge_nb;     /* head/tail margin of BCCH/CCCH normal bursts, < 0 = none (pont.c sets it per frame) */
extern int cellule_sans_bcch;
extern double cellule_dec_nb;
extern double cellule_phase_nb;
extern int cellule_fenetre_nb;   /* NB window length in samples, the frame is padded to it exactly */
extern int cellule_tsc_force;    /* TSC of the normal bursts, < 0 = BCC (CELLULE_TSC, pont.c) */    /* 1: the old cell, dummy bursts on BCCH/CCCH */
int cellule_train_sb(int i);
int cellule_demod_d(const int16_t *x, int n_ech, int b0, unsigned char *d148);
int cellule_demod_reference(const int16_t *x, int n_ech, unsigned char *bits148, int *offset);
void cellule_code_attendu(uint32_t fn, uint8_t bsic, unsigned char *code78);
char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq);
int cellule_bits_attendus(uint32_t fn, uint8_t bsic, uint8_t bits[148]);
void cellule_factice(int amp, double decalage, int16_t *iq);   /* 148 GMSK samples of the dummy burst */
#endif
