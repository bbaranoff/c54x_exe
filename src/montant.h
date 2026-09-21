/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * montant.h - le lien montant du montage DSP (RACH, SDCCH, SACCH, FACCH, parole).
 */
#ifndef C54X_EXE_MONTANT_H
#define C54X_EXE_MONTANT_H

#include <stdint.h>

/* A appeler une fois par trame, apres que l'ARM a rendu la main (fin de
 * scenario : d_dsp_page ecrit, taches posees dans la page W). `page` est le
 * bit 0 de d_dsp_page, celui que QEMU transporte deja dans PONT_TICK.m.b. */
void montant_scruter(uint16_t *api_ram, uint32_t fn, unsigned page);

/* Une ligne de bilan en fin de session. */
void montant_bilan(void);

#endif
