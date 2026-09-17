/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * cellule.c - les bursts de synchronisation d'une cellule, pour le DSP.
 *
 * [2026-09-17] FCCH = 148 zeros (45.002 §5.2.4). SCH = 3 tail + 39 + 64 (ETS,
 * 45.002 §5.2.5) + 39 + 3 tail, les 78 bits codes par gsm0503_sch_encode()
 * de libosmocoding a partir de sb_info (BSIC + T1'/T2/T3', 44.018 §9.1.30),
 * assemblage repris de osmo-bts sched_lchan_fcch_sch.c. Burst factice : le
 * motif fixe de 45.002 §5.2.6, repris de osmo-bts scheduler.c.
 */
#include <string.h>
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>
#include "gmsk.h"
#include "cellule.h"

static const uint8_t train_sb[64] = {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};
static const uint8_t factice[148] = {
    0,0,0,
    1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
    0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
    0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
    0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
    0,0,1,0,1,1,1,1,1,0,1,0,1,0,
    0,0,0,
};

int cellule_sch_partout;   /* diagnostic : SCH sur toutes les trames non-FCCH */

char cellule_burst(uint32_t fn, uint8_t bsic, int amp, double decalage, int marge, int16_t *iq, int *n_iq)
{
    uint8_t bits[148];
    uint32_t p51 = fn % 51;
    char type;
    static int sch_only = -1;
    if (sch_only < 0) { const char *e = getenv("CELLULE_SCH_ONLY"); sch_only = (e && *e=='1') ? 1 : 0; }
    if (sch_only) p51 = 1;   /* diag : force la position SCH sur toutes les trames */
    if (p51 % 10 == 0 && p51 <= 40) {
        memset(bits, 0, sizeof(bits));
        type = 'F';
    } else if ((p51 % 10 == 1 && p51 <= 41) || cellule_sch_partout) {
        uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = p51, t3p = t3 / 10;
        uint8_t sb_info[4] = {
            (uint8_t)(((bsic & 0x3f) << 2) | ((t1 & 0x600) >> 9)),
            (uint8_t)((t1 & 0x1fe) >> 1),
            (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1)),
            (uint8_t)(t3p & 0x1),
        };
        ubit_t code[78];
        gsm0503_sch_encode(code, sb_info);
        memset(bits, 0, 3);
        memcpy(bits + 3, code, 39);
        memcpy(bits + 42, train_sb, 64);
        memcpy(bits + 106, code + 39, 39);
        memset(bits + 145, 0, 3);
        type = 'S';
    } else {
        memcpy(bits, factice, 148);
        type = '.';
    }
    if (type == 'S' && marge > 0) {
        memset(iq, 0, (size_t)marge * 2 * sizeof(int16_t));
        gmsk_moduler(bits, 148, amp, 0.0, decalage, iq + 2 * marge);
        memset(iq + 2 * (marge + 148), 0, (size_t)marge * 2 * sizeof(int16_t));
        *n_iq = 2 * (148 + 2 * marge);
    } else {
        gmsk_moduler(bits, 148, amp, 0.0, decalage, iq);
        *n_iq = 2 * 148;
    }
    return type;
}
