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

/* [2026-10-22] FACCH montante lue au GO, avant que la ROM n'efface B_BLUD de a_fu. */
void montant_capter_fu(uint16_t *api_ram, uint32_t fn);

/* Le canal dedie est libere : oublier le dernier bloc SDCCH publie. */
void montant_canal_libere(void);

/* Une ligne de bilan en fin de session. */
void montant_bilan(void);

/* Vrai tant que le BSP joue l'intervalle du TCH (bascule suivie par le firmware). */
bool montant_sur_tch(void);
int  montant_tn_dedie(void);   /* [2026-10-20] TS du TCH (side-band), 0 sinon : etiquette des bursts publies */

#endif
