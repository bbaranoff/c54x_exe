/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef REJOUER_H
#define REJOUER_H
#include <stdint.h>
#include "calypso_c54x.h"
extern uint32_t g_c54x_exe_fn;
int rejouer(C54xState *dsp, uint16_t *api_ram, long trames, long insns,
            const char *iq_mode, int amp, int bsic, int verbeux);
#endif
