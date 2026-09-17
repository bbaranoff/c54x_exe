/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef VERBOSITE_H
#define VERBOSITE_H
#include <stdio.h>

/* Niveaux : 0 = erreurs seulement (defaut)   -v = + avertissements
 *           -vv = + cycle de vie (boot, reset, banniere des gates)
 *           -vvv = + taches, API RAM, interruptions
 *           -vvvv = + sondes memoire (WATCH, DUMP, SCAN, MAP, SP-*)
 *           -vvvvv = + traces instruction par instruction (BRANCH, LOOP, TERM, CYCLE)
 *           -vvvvvv = tout, stderr brut, sans filtre */
#define VERBOSITE_MAX 6

void verbosite_installer(int niveau);   /* a appeler avant tout appel au DSP */
void verbosite_retirer(void);           /* vide le tube, rend stderr          */
void verbosite_bilan(FILE *out);        /* combien de lignes masquees, par niveau */

#endif
