/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * verbosite.c - the -v levels of c54x_exe.
 *
 * The C54x core (qosmo/hw/arm/calypso/l1-dsp/calypso_c54x.c) carries about a
 * hundred "[c54x] NAME ..." probes on stderr, some of them unconditional
 * (BRANCH-TRACE alone emits 700 lines at boot) [2026-09-16]. They serve QEMU
 * as much as this binary, so gating them one by one through environment
 * variables would mean touching the shared sources for every new probe.
 *
 * Instead the core is left alone: stderr is redirected into a pipe and a
 * thread re-reads the lines, passing through only those whose level is <= the
 * requested one. Classification is by keyword on the probe NAME rather than an
 * exhaustive table, so a new probe lands at a sensible level without being
 * declared. The summary reports how many lines each level hid, so the caller
 * knows what to ask for.
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "verbosite.h"

static int       g_niveau = 0;
static int       g_stderr_reel = -1;   /* dup(2) of the original */
static int       g_tube_lecture = -1;
static pthread_t g_fil;
static bool      g_actif = false;
static unsigned long g_masquees[VERBOSITE_MAX + 1];
static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;

static bool contient(const char *l, const char *mot) { return strstr(l, mot) != NULL; }

/* Lowest -v level at which a line is shown. */
static int niveau_ligne(const char *l)
{
    /* memory probes whose NAME contains an error keyword: classify first */
    if (contient(l, "ERRWATCH"))
        return 4;
    /* 0: what breaks */
    if (contient(l, "FATAL") || contient(l, "ERR") || contient(l, "abort") ||
        contient(l, "CORRUPT") || contient(l, "TRAP") || contient(l, "manquante") ||
        contient(l, "cannot") || contient(l, "impossible") || contient(l, "a echoue"))
        return 0;
    /* 1: what is worrying */
    if (contient(l, "WARN") || contient(l, "echou") || contient(l, "failed") ||
        contient(l, "pas parque") || contient(l, "perdue"))
        return 1;
    /* 5: single-stepping */
    if (contient(l, "TRACE") || contient(l, "LOOP") || contient(l, "CYCLE") ||
        contient(l, "HIST") || contient(l, "RING") || contient(l, "-STACK"))
        return 5;
    /* 4: memory probes */
    if (contient(l, "WATCH") || contient(l, "DUMP") || contient(l, "SCAN") ||
        contient(l, "PROBE") || contient(l, "MAP") || contient(l, "SP-") ||
        contient(l, "FLOW") || contient(l, "HOT-OPS") || contient(l, "FIRST"))
        return 4;
    /* 3: tasks, API RAM, interrupts, FB/SB paths */
    if (contient(l, "TASK") || contient(l, "DISPATCH") || contient(l, "] FB") ||
        contient(l, "-FB") || contient(l, "FBDET") || contient(l, "FBWATCH") ||
        contient(l, "FBSB") || contient(l, "FBCALL") || contient(l, "FBROUTE") ||
        contient(l, "FBENTRY") || contient(l, "FBGATE") || contient(l, "FBMODE") ||
        contient(l, "SYNC") || contient(l, "FEED") || contient(l, "VEC") ||
        contient(l, "INTM") || contient(l, "IFR") || contient(l, "IMR") ||
        contient(l, "PMST") || contient(l, "MMR") || contient(l, "AFC") ||
        contient(l, "IRQ") || contient(l, "ACK") || contient(l, "IDLE") ||
        contient(l, "DMA") || contient(l, "RIF") || contient(l, "bsp") ||
        contient(l, "BSP") || contient(l, "TPU") || contient(l, "API"))
        return 3;
    /* 2: the rest - gate banners (ACTIF/INACTIVE), CALYPSO_* reads, boot,
     *    reset, [calypso-debug], and anything not coming from the core */
    return 2;
}

static void *fil_filtre(void *arg)
{
    (void)arg;
    FILE *in = fdopen(g_tube_lecture, "r");
    FILE *out = fdopen(g_stderr_reel, "w");
    if (!in || !out) {
        return NULL;
    }
    setvbuf(out, NULL, _IOLBF, 0);
    char ligne[4096];
    while (fgets(ligne, sizeof(ligne), in)) {
        int n = niveau_ligne(ligne);
        if (n <= g_niveau) {
            fputs(ligne, out);
        } else {
            pthread_mutex_lock(&g_mu);
            g_masquees[n]++;
            pthread_mutex_unlock(&g_mu);
        }
    }
    fflush(out);
    return NULL;
}

void verbosite_installer(int niveau)
{
    if (niveau < 0) niveau = 0;
    if (niveau > VERBOSITE_MAX) niveau = VERBOSITE_MAX;
    g_niveau = niveau;
    if (niveau >= VERBOSITE_MAX) {
        return;                         /* raw: nothing to install */
    }
    int tube[2];
    if (pipe(tube) < 0) {
        return;
    }
    fflush(stderr);
    g_stderr_reel = dup(STDERR_FILENO);
    g_tube_lecture = tube[0];
    dup2(tube[1], STDERR_FILENO);
    close(tube[1]);
    if (pthread_create(&g_fil, NULL, fil_filtre, NULL) != 0) {
        dup2(g_stderr_reel, STDERR_FILENO);
        return;
    }
    g_actif = true;
}

void verbosite_retirer(void)
{
    if (!g_actif) {
        return;
    }
    fflush(stderr);
    /* restore fd 2: the pipe write end closes, the thread sees EOF */
    dup2(g_stderr_reel, STDERR_FILENO);
    pthread_join(g_fil, NULL);
    g_actif = false;
}

void verbosite_bilan(FILE *out)
{
    unsigned long total = 0;
    for (int i = 0; i <= VERBOSITE_MAX; i++) total += g_masquees[i];
    if (!total) {
        return;
    }
    fprintf(out, "  traces DSP masquees : %lu  (", total);
    const char *nom[] = { "erreurs", "-v", "-vv", "-vvv", "-vvvv", "-vvvvv", "brut" };
    bool premier = true;
    for (int i = 1; i <= 5; i++) {
        if (!g_masquees[i]) continue;
        fprintf(out, "%s%lu avec %s", premier ? "" : ", ", g_masquees[i], nom[i]);
        premier = false;
    }
    fprintf(out, ")\n");
}
