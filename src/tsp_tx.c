/* SPDX-License-Identifier: GPL-2.0-or-later */
/* tsp_tx.c - voir tsp_tx.h. */
#include <string.h>
#include "tsp_tx.h"

/* DROM 0xa0c9..0xa0d8 : les 8 sequences d'apprentissage des bursts normaux (26 bits, 2 mots chacune),
 * telles que la ROM les copie dans son emetteur (0x85d6 : 0xa0c9 + 2 * tsc). */
static const char *const TSC_ROM[8] = {
    "00100101110000100010010111", "00101101110111100010110111", "01000011101110100100001110",
    "01000111101101000100011110", "00011010111001000001101011", "01001110101100000100111010",
    "10100111110110001010011111", "11101111000100101110111100",
};
/* DROM 0xa0d9..0xa0db : la sequence de synchronisation de l'access-burst (41 bits, 05.02 5.2.7). */
static const char SYNC_RACH[] = "01001011011111111001100110101010001111000";

static bool motif_a(const uint8_t *flux, int pos, const char *m, int n)
{
    if (pos < 0 || pos + n > 160) return false;
    for (int i = 0; i < n; i++) if (flux[pos + i] != (uint8_t)(m[i] - '0')) return false;
    return true;
}

bool tsp_tx_lire(const uint16_t *data, struct tsp_tx_burst *b, bool *nouveau)
{
    static uint16_t prec[16];
    for (unsigned a = 0x3cbb; a + 18 <= 0x3d00; a++) {
        if (data[a] != 0x1c0a) continue;
        const uint16_t *w = &data[a + 2];
        int ok = 1;
        for (int i = 0; i < 16; i++) if ((w[i] & 0x3e) != 0x06 && (w[i] & 0x3e) != 0x04) { ok = 0; break; }
        if (!ok) continue;
        memset(b, 0, sizeof *b);
        b->adresse = (uint16_t)a;
        memcpy(b->mots, w, sizeof b->mots);
        for (int i = 0; i < 16; i++) for (int k = 0; k < 10; k++) b->flux[10 * i + k] = (w[i] >> (15 - k)) & 1;
        *nouveau = memcmp(w, prec, sizeof prec) != 0;
        memcpy(prec, w, sizeof prec);
        b->tsc = -1; b->type = TSP_TX_INCONNU; b->offset = -1;
        /* access-burst : [8 de garde][8 queue etendue][41 sync]... -> sync a 16, burst a 8 */
        if (motif_a(b->flux, 16, SYNC_RACH, 41)) { b->type = TSP_TX_RACH; b->offset = 8; }
        else {
            /* burst normal : 3 queue, 57, hl, 26 TSC, hu, 57, 3 -> TSC a offset + 61 */
            for (int t = 0; t < 8 && b->type == TSP_TX_INCONNU; t++)
                for (int p = 61; p + 26 <= 160 && p - 61 + 148 <= 160; p++)
                    if (motif_a(b->flux, p, TSC_ROM[t], 26)) { b->type = TSP_TX_NB; b->tsc = t; b->offset = p - 61; break; }
        }
        if (b->offset >= 0) memcpy(b->bits, b->flux + b->offset, 148);
        else memcpy(b->bits, b->flux, 148);
        return true;
    }
    return false;
}
