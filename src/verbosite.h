/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef VERBOSITE_H
#define VERBOSITE_H
#include <stdio.h>

/* Levels: 0 = errors only (default)    -v = + warnings
 *         -vv    = + lifecycle (boot, reset, gate banner)
 *         -vvv   = + tasks, API RAM, interrupts
 *         -vvvv  = + memory probes (WATCH, DUMP, SCAN, MAP, SP-*)
 *         -vvvvv = + per-instruction traces (BRANCH, LOOP, TERM, CYCLE)
 *         -vvvvvv = everything, raw stderr, unfiltered */
#define VERBOSITE_MAX 6

void verbosite_installer(int niveau);   /* call before any DSP call           */
void verbosite_retirer(void);           /* drain the pipe, restore stderr     */
void verbosite_bilan(FILE *out);        /* count of suppressed lines, per level */

#endif
