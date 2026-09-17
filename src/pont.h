/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef PONT_H
#define PONT_H
#include <stdint.h>
#include <stdbool.h>
#include "calypso_c54x.h"

/* Alloue le C54xState avec data[] alignee sur une page, et pose le segment
 * partage PAR-DESSUS data[0x0800..0x27FF] (cf. calypso_dsp_pont.h). Remplace
 * c54x_init() en mode --arm. api_ram doit ensuite etre &dsp->data[C54X_API_BASE]. */
C54xState *pont_allouer_dsp(void);

/* Sert l'ARM de QEMU : accepte une connexion apres l'autre, joue une trame par
 * TICK, repond DONE. Ne revient que sur SIGINT/SIGTERM. */
/* iq_mode : NULL/"none" = rien, "fcch" = burst FCCH synthetique (rotation
 * +pi/2 par echantillon, 1 ech/symbole), "noise" = bruit, "tone:<dphi>" = rotation
 * de dphi radians par echantillon. amp = amplitude int16 (les FCCH reels sont
 * ~32500 rms). Injecte a chaque trame via calypso_bsp_rx_burst(). */
int pont_serveur(C54xState *dsp, uint16_t *api_ram, const char *socket_path,
                 long insns, bool verbeux, const char *iq_mode, int amp);

#endif
