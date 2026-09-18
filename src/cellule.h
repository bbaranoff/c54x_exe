/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef CELLULE_H
#define CELLULE_H
#include <stdint.h>
/* Le burst descendant du TN0 d'une cellule GSM pour la trame fn (multitrame 51 :
 * FCCH en 0/10/20/30/40, SCH en 1/11/21/31/41, burst factice ailleurs), module
 * en GMSK, 148 echantillons I/Q int16. Rend le type : 'F', 'S' ou '.'. */
/* marge : echantillons de silence ajoutes AVANT et APRES le burst SCH (le
 * decodeur SB de la ROM lit une fenetre de 190 echantillons, 148 + 2 x 21 :
 * BSP_IQ_MAX_I16 « SB en demande 190 »). *n_iq recoit le nombre d'int16 ecrits. */
extern int cellule_sch_partout;
extern int cellule_marge_fin;
int cellule_train_sb(int i);
int cellule_demod_d(const int16_t *x, int n_ech, int b0, unsigned char *d148);
int cellule_demod_reference(const int16_t *x, int n_ech, unsigned char *bits148, int *offset);
void cellule_code_attendu(uint32_t fn, uint8_t bsic, unsigned char *code78);
char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq);
#endif
