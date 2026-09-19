/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef PONT_H
#define PONT_H
#include <stdint.h>
#include <stdbool.h>
#include "calypso_c54x.h"

/* Allocates the C54xState with data[] page-aligned, then maps the shared segment
 * OVER data[0x0800..0x27FF] (protocol in calypso_dsp_pont.h). Replaces
 * c54x_init() in --arm mode; api_ram must then be &dsp->data[C54X_API_BASE]. */
C54xState *pont_allouer_dsp(void);

/* Serves QEMU's ARM: one connection after another, runs one frame per TICK,
 * answers DONE. Returns only on SIGINT/SIGTERM.
 * iq_mode: NULL/"none" = nothing, "fcch" = synthetic FCCH burst (+pi/2 rotation
 * per sample, 1 sample/symbol), "noise" = noise, "tone:<dphi>" = dphi radians
 * per sample, "cell[:bsic[:decalage[:marge]]]" = full 51-multiframe cell.
 * amp = int16 amplitude (real FCCH bursts run at ~32500 rms). Injected on every
 * frame through calypso_bsp_rx_burst(). */
int pont_serveur(C54xState *dsp, uint16_t *api_ram, const char *socket_path,
                 long insns, bool verbeux, const char *iq_mode, int amp);

#endif
