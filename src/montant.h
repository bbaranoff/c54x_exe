/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * montant.h - le lien montant du montage DSP (RACH, SDCCH, SACCH, FACCH, parole).
 */
#ifndef C54X_EXE_MONTANT_H
#define C54X_EXE_MONTANT_H

#include <stdint.h>
#include <stdbool.h>

/* A appeler une fois par trame, apres que l'ARM a rendu la main (fin de
 * scenario : d_dsp_page ecrit, taches posees dans la page W). `page` est le
 * bit 0 de d_dsp_page, celui que QEMU transporte deja dans PONT_TICK.m.b. */
void montant_scruter(uint16_t *api_ram, uint32_t fn, unsigned page);

/* Le canal dedie est libere : oublier le dernier bloc SDCCH publie. */
void montant_canal_libere(void);

/* Une ligne de bilan en fin de session. */
void montant_bilan(void);

/* Vrai tant que le BSP joue l'intervalle du TCH (bascule suivie par le firmware). */
bool montant_sur_tch(void);

#endif
