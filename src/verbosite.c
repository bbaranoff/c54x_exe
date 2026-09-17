/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * verbosite.c - les niveaux -v de c54x_exe.
 *
 * [2026-09-16] Le coeur C54x (qosmo/hw/arm/calypso/l1-dsp/calypso_c54x.c)
 * porte une centaine de sondes « [c54x] NOM ... » sur stderr, dont certaines
 * inconditionnelles (BRANCH-TRACE : 700 lignes au boot). Elles servent QEMU
 * autant que ce binaire, et les garder une par une par variable
 * d'environnement demanderait de toucher les sources partagees pour chaque
 * nouvelle sonde.
 *
 * Ici on ne touche pas au coeur : stderr est detourne dans un tube, un fil
 * relit les lignes et ne laisse passer que celles dont le niveau est <= au
 * niveau demande. Le classement est par mots-clefs sur le NOM de la sonde,
 * pas par table exhaustive : une sonde nouvelle tombe dans un niveau
 * raisonnable sans qu'on ait a la declarer. Le bilan dit combien de lignes
 * ont ete masquees a chaque niveau, pour qu'on sache quoi demander.
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "verbosite.h"

static int       g_niveau = 0;
static int       g_stderr_reel = -1;   /* dup(2) d'origine */
static int       g_tube_lecture = -1;
static pthread_t g_fil;
static bool      g_actif = false;
static unsigned long g_masquees[VERBOSITE_MAX + 1];
static pthread_mutex_t g_mu = PTHREAD_MUTEX_INITIALIZER;

static bool contient(const char *l, const char *mot) { return strstr(l, mot) != NULL; }

/* Le niveau minimal auquel une ligne est montree. */
static int niveau_ligne(const char *l)
{
    /* 0 : ce qui casse */
    if (contient(l, "FATAL") || contient(l, "ERR") || contient(l, "abort") ||
        contient(l, "CORRUPT") || contient(l, "TRAP") || contient(l, "manquante") ||
        contient(l, "cannot") || contient(l, "impossible") || contient(l, "a echoue"))
        return 0;
    /* 1 : ce qui inquiete */
    if (contient(l, "WARN") || contient(l, "echou") || contient(l, "failed") ||
        contient(l, "pas parque") || contient(l, "perdue"))
        return 1;
    /* 5 : le pas a pas */
    if (contient(l, "TRACE") || contient(l, "LOOP") || contient(l, "CYCLE") ||
        contient(l, "HIST") || contient(l, "RING") || contient(l, "-STACK"))
        return 5;
    /* 4 : les sondes memoire */
    if (contient(l, "WATCH") || contient(l, "DUMP") || contient(l, "SCAN") ||
        contient(l, "PROBE") || contient(l, "MAP") || contient(l, "SP-") ||
        contient(l, "FLOW") || contient(l, "HOT-OPS") || contient(l, "FIRST"))
        return 4;
    /* 3 : taches, API RAM, interruptions, chemins FB/SB */
    if (contient(l, "TASK") || contient(l, "DISPATCH") || contient(l, "FB") ||
        contient(l, "SYNC") || contient(l, "FEED") || contient(l, "VEC") ||
        contient(l, "INTM") || contient(l, "IFR") || contient(l, "IMR") ||
        contient(l, "PMST") || contient(l, "MMR") || contient(l, "AFC") ||
        contient(l, "IRQ") || contient(l, "ACK") || contient(l, "IDLE") ||
        contient(l, "DMA") || contient(l, "RIF") || contient(l, "bsp") ||
        contient(l, "BSP") || contient(l, "TPU") || contient(l, "API"))
        return 3;
    /* 2 : le reste - banniere des gates (ACTIF/INACTIVE), CALYPSO_* lus,
     *     boot, reset, [calypso-debug], et ce qui ne vient pas du coeur */
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
        return;                         /* brut : rien a installer */
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
    /* rendre fd 2 : le bout d'ecriture du tube se ferme, le fil voit EOF */
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
