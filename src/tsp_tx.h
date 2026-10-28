/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * tsp_tx.h - the FINAL uplink burst of the ROM, as it hands it to the ABB.
 *
 * Every frame, the ROM builds in data[0x3cbb..] the TSP script for the TWL3025 (PROM0
 * 0xb5b8 `stm #15547,ar2`). When a burst goes out, the script holds the word TOGBR2 = 0x1c0a, a
 * power word, then the 16 BULDATA words written by the transmitter 0x8605: (10 data bits) << 6 |
 * (register BULDATA1 = 3) << 1, i.e. `xxxx xxxx xx00 0110`. The 160 bits serialised this way are
 * the ROM output after training sequence (DROM 0xa0c9), RACH sync (DROM 0xa0d9), tail bits
 * (DROM 0xa0dc), A5 ciphering (XOR at 0x85d1) and I/Q inversion (XOR 0xaa80 at 0xb773): what the
 * silicon sends to the antenna, up to the format. Measured (dsp_tester rach, tx-sdcch,
 * tx-sdcch-a5): bit-identical to libosmocoding.
 */
#ifndef TSP_TX_H
#define TSP_TX_H
#include <stdint.h>
#include <stdbool.h>

/* SB, DUMMY, FB: the bursts of a BTS (trx firmware, start-native.sh --osmo-trx), sent by the
 * RACH task that the extcode hijacks; published in calypso_tx_rom for the air. */
enum tsp_tx_type { TSP_TX_INCONNU = 0, TSP_TX_RACH = 1, TSP_TX_NB = 2, TSP_TX_SB = 3, TSP_TX_DUMMY = 4, TSP_TX_FB = 5 };

struct tsp_tx_burst {
    uint16_t mots[16];      /* the 16 BULDATA words as they are */
    uint8_t  flux[160];     /* the 160 serialised bits (10 per word, MSB first) */
    uint8_t  bits[148];     /* the 148-bit burst (0/1): access burst or normal burst */
    int      type;          /* enum tsp_tx_type */
    int      tsc;           /* NB: recognised training sequence (0..7), -1 otherwise */
    int      offset;        /* position of the burst in flux[] (8 for the RACH, else TSC - 61) */
    uint16_t adresse;       /* address of the 0x1c0a marker in data memory */
};

/* Looks for the TSP script in data[0x3cbb..0x3d00[ and reads the burst from it. Returns true if a
 * complete burst (16 BULDATA words) is present; `nouveau` tells whether it differs from the 16 words
 * of the previous call (same process): this is the "a burst has just been sent" signal when no ring
 * pointer has moved (RACH). */
bool tsp_tx_lire(const uint16_t *data, struct tsp_tx_burst *b, bool *nouveau);

#endif
