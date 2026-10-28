/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * montant.h - the uplink side of the DSP setup (RACH, SDCCH, SACCH, FACCH, speech).
 */
#ifndef C54X_EXE_MONTANT_H
#define C54X_EXE_MONTANT_H

#include <stdint.h>
#include <stdbool.h>

/* To be called once per frame, after the ARM has handed over (end of
 * scenario: d_dsp_page written, tasks posted in the W page). `page` is bit 0
 * of d_dsp_page, the one QEMU already carries in PONT_TICK.m.b. */
void montant_scruter(uint16_t *api_ram, uint32_t fn, unsigned page);

/* Uplink FACCH read at the GO, before the ROM clears B_BLUD from a_fu. */
void montant_capter_fu(uint16_t *api_ram, uint32_t fn);

/* The dedicated channel is released: forget the last published SDCCH block. */
void montant_canal_libere(void);

/* One summary line at the end of the session. */
void montant_bilan(void);

/* True while the BSP plays the TCH timeslot (switch followed from the firmware). */
bool montant_sur_tch(void);
int  montant_tn_dedie(void);   /* TS of the TCH (side-band), 0 otherwise: label of the published bursts */

#endif
