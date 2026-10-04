/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * tsp_tx.h - le burst montant FINAL de la ROM, tel qu'elle le remet a l'ABB.
 *
 * [2026-10-04] Chaque trame, la ROM construit en data[0x3cbb..] le script TSP pour le TWL3025 (PROM0
 * 0xb5b8 `stm #15547,ar2`). Quand un burst part, le script contient le mot TOGBR2 = 0x1c0a, un mot de
 * puissance, puis les 16 mots BULDATA ecrits par l'emetteur 0x8605 : (10 bits de donnees) << 6 |
 * (registre BULDATA1 = 3) << 1, soit `xxxx xxxx xx00 0110`. Les 160 bits ainsi serialises sont la
 * sortie de la ROM apres sequence d'apprentissage (DROM 0xa0c9), synchro RACH (DROM 0xa0d9), queues
 * (DROM 0xa0dc), chiffrement A5 (XOR en 0x85d1) et inversion I/Q (XOR 0xaa80 en 0xb773) : ce que le
 * silicium envoie a l'antenne, au format pres. Mesure (dsp_tester rach, tx-sdcch, tx-sdcch-a5) :
 * identique a libosmocoding au bit pres.
 *
 * Deux bugs du coeur (CMPR absent, LD src,ASM sans decalage) empechaient cet emetteur de tourner en
 * emulation jusqu'au 2026-10-04 ; d'ou l'ancienne capture en 0x3f8a (bits avant modulateur).
 */
#ifndef TSP_TX_H
#define TSP_TX_H
#include <stdint.h>
#include <stdbool.h>

enum tsp_tx_type { TSP_TX_INCONNU = 0, TSP_TX_RACH = 1, TSP_TX_NB = 2 };

struct tsp_tx_burst {
    uint16_t mots[16];      /* les 16 mots BULDATA tels quels */
    uint8_t  flux[160];     /* les 160 bits serialises (10 par mot, MSB d'abord) */
    uint8_t  bits[148];     /* le burst de 148 bits (0/1) : access-burst ou burst normal */
    int      type;          /* enum tsp_tx_type */
    int      tsc;           /* NB : sequence d'apprentissage reconnue (0..7), -1 sinon */
    int      offset;        /* position du burst dans flux[] (8 pour le RACH, sinon TSC - 61) */
    uint16_t adresse;       /* adresse du marqueur 0x1c0a dans la memoire de donnees */
};

/* Cherche le script TSP dans data[0x3cbb..0x3d00[ et y lit le burst. Rend true si un burst complet
 * (16 mots BULDATA) est present ; `nouveau` dit s'il differe des 16 mots de l'appel precedent (meme
 * processus) : c'est le signal « un burst vient d'etre emis » quand aucun pointeur d'anneau n'a bouge
 * (RACH). */
bool tsp_tx_lire(const uint16_t *data, struct tsp_tx_burst *b, bool *nouveau);

#endif
