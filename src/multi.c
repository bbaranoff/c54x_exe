/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * multi.c - c54x_exe on several cores: N independent DSPs at once.
 *
 * Why processes and not threads. The C54x core and the Calypso peripherals
 * compiled from qosmo keep their state in globals (c54x_probes.c, the BSP,
 * the DMA, the GMSK tables) and cache their environment flags in function
 * statics read once (calypso_getenv in rejouer.c, calypso_bsp.c). Two cores in
 * one address space would share all of it. One process per job gives each DSP
 * its own copy, and exec()ing a fresh image means a job's flags never leak
 * into the next one: each job is exactly the single-core c54x_exe, byte for
 * byte the same code path, so a result found here reproduces alone.
 *
 * Plan: one job per line, '#' starts a comment, blank lines skipped. A line is
 * environment assignments followed by c54x_exe arguments:
 *
 *     REJEU_MARGE=21 REJEU_DECALAGE_SYMB=0.5 --rejouer --verbeux --trames 600
 *
 * Words are separated by blanks; no quoting (none of the options need it).
 * Job k (from 1, in plan order) writes stdout to SORTIE/NNNNN.log, stderr to
 * SORTIE/NNNNN.err, and runs pinned to one CPU of the affinity mask. delai > 0
 * kills a job after that many seconds (SIGALRM, armed before exec: status
 * 142). When everything has finished, SORTIE/bilan.tsv holds: job, exit
 * status, seconds, CPU, line.
 *
 * Not for --arm: it serves ONE QEMU over a fixed socket and shared memory. Jobs
 * that open dedicated-channel recordings (/dev/shm/calypso_tch_dl.bin ...)
 * still share those fixed paths; the FB/SB replay does not write them.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include "multi.h"

typedef struct {
    char *ligne;          /* the plan line, as written (for the report) */
    char **env;           /* NAME=value, NULL-terminated */
    char **args;          /* argv[1..], NULL-terminated */
    pid_t pid;
    int cpu;
    int statut;           /* exit code, 128+signal, -1 = not run */
    double debut, duree;
} Tache;

static double maintenant(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static bool est_affectation(const char *m)
{
    if (!(m[0] == '_' || (m[0] >= 'A' && m[0] <= 'Z') || (m[0] >= 'a' && m[0] <= 'z')))
        return false;
    for (const char *p = m; *p; p++) {
        if (*p == '=') return true;
        if (!(*p == '_' || (*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
              (*p >= '0' && *p <= '9')))
            return false;
    }
    return false;
}

/* 0 = job parsed, 1 = blank/comment line, -1 = error (message printed). */
static int analyser(char *texte, int no, Tache *t)
{
    char *diese = strchr(texte, '#');
    if (diese) *diese = '\0';
    texte[strcspn(texte, "\r\n")] = '\0';
    t->ligne = strdup(texte);

    size_t cap = 8, ne = 0, na = 0;
    t->env = calloc(cap, sizeof(char *));
    t->args = calloc(cap, sizeof(char *));
    size_t cap_e = cap, cap_a = cap;
    bool args_vus = false;
    for (char *m = strtok(texte, " \t"); m; m = strtok(NULL, " \t")) {
        if (!args_vus && est_affectation(m)) {
            if (ne + 2 > cap_e) { cap_e *= 2; t->env = realloc(t->env, cap_e * sizeof(char *)); }
            t->env[ne++] = strdup(m);
            t->env[ne] = NULL;
            continue;
        }
        args_vus = true;
        if (!strcmp(m, "--arm") || !strcmp(m, "--plan")) {
            fprintf(stderr, "plan ligne %d : %s interdit dans une tache\n", no, m);
            return -1;
        }
        if (na + 2 > cap_a) { cap_a *= 2; t->args = realloc(t->args, cap_a * sizeof(char *)); }
        t->args[na++] = strdup(m);
        t->args[na] = NULL;
    }
    if (ne == 0 && na == 0) return 1;
    return 0;
}

static Tache *lire_plan(const char *plan, int *n_out)
{
    FILE *f = strcmp(plan, "-") ? fopen(plan, "r") : stdin;
    if (!f) { fprintf(stderr, "plan %s : %s\n", plan, strerror(errno)); return NULL; }
    size_t cap = 64, n = 0;
    Tache *t = calloc(cap, sizeof(Tache));
    char *buf = NULL; size_t len = 0; int no = 0;
    while (getline(&buf, &len, f) >= 0) {
        no++;
        if (n == cap) { cap *= 2; t = realloc(t, cap * sizeof(Tache)); }
        memset(&t[n], 0, sizeof(Tache));
        int r = analyser(buf, no, &t[n]);
        if (r < 0) { free(buf); if (f != stdin) fclose(f); return NULL; }
        if (r == 0) { t[n].statut = -1; t[n].cpu = -1; n++; }
    }
    free(buf);
    if (f != stdin) fclose(f);
    *n_out = (int)n;
    return t;
}

static void lancer(Tache *t, int k, int cpu, const char *sortie, const char *exe, int delai)
{
    char journal[4096], erreurs[4096];
    snprintf(journal, sizeof(journal), "%s/%05d.log", sortie, k + 1);
    snprintf(erreurs, sizeof(erreurs), "%s/%05d.err", sortie, k + 1);
    fflush(stdout);
    fflush(stderr);
    pid_t pid = fork();
    if (pid < 0) {
        fprintf(stderr, "fork : %s\n", strerror(errno));
        t->statut = 127;
        return;
    }
    if (pid == 0) {
        int fd = open(journal, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        int fe = open(erreurs, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0 || fe < 0) _exit(126);
        dup2(fd, 1);
        dup2(fe, 2);
        close(fd);
        close(fe);
        int nul = open("/dev/null", O_RDONLY);
        if (nul >= 0) { dup2(nul, 0); close(nul); }
        if (cpu >= 0) {
            cpu_set_t un;
            CPU_ZERO(&un);
            CPU_SET(cpu, &un);
            sched_setaffinity(0, sizeof(un), &un);
        }
        for (char **e = t->env; *e; e++) putenv(*e);
        char num[16];
        snprintf(num, sizeof(num), "%d", k + 1);
        setenv("C54X_TACHE", num, 1);
        size_t na = 0;
        while (t->args[na]) na++;
        char **av = calloc(na + 2, sizeof(char *));
        av[0] = (char *)exe;
        memcpy(av + 1, t->args, na * sizeof(char *));
        if (delai > 0) alarm((unsigned)delai);
        execv(exe, av);
        fprintf(stderr, "execv %s : %s\n", exe, strerror(errno));
        _exit(127);
    }
    t->pid = pid;
    t->cpu = cpu;
    t->debut = maintenant();
}

int multi_executer(const char *plan, int coeurs, const char *sortie, int delai)
{
    int n = 0;
    Tache *t = lire_plan(plan, &n);
    if (!t) return 2;
    if (n == 0) { fprintf(stderr, "plan %s : aucune tache\n", plan); return 2; }

    /* The CPUs we may use: the affinity mask (taskset, cgroups), not nproc. */
    cpu_set_t masque;
    int cpus[CPU_SETSIZE], n_cpus = 0;
    if (sched_getaffinity(0, sizeof(masque), &masque) == 0) {
        for (int c = 0; c < CPU_SETSIZE; c++)
            if (CPU_ISSET(c, &masque)) cpus[n_cpus++] = c;
    }
    if (coeurs <= 0) coeurs = n_cpus > 0 ? n_cpus : 1;
    if (coeurs > n) coeurs = n;
    /* Pinning only makes sense with no more jobs than CPUs at a time. */
    bool epingler = n_cpus > 0 && coeurs <= n_cpus;

    if (mkdir(sortie, 0755) < 0 && errno != EEXIST) {
        fprintf(stderr, "sortie %s : %s\n", sortie, strerror(errno));
        return 2;
    }
    char exe[4096];
    ssize_t l = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (l <= 0) { fprintf(stderr, "readlink /proc/self/exe : %s\n", strerror(errno)); return 2; }
    exe[l] = '\0';

    printf("[multi] %d tache(s) sur %d coeur(s)%s -> %s/\n", n, coeurs,
           epingler ? ", un CPU par tache" : "", sortie);
    fflush(stdout);

    /* slot s runs on cpus[s]; occupe[s] = index of its job, -1 if free */
    int *occupe = malloc(coeurs * sizeof(int));
    for (int s = 0; s < coeurs; s++) occupe[s] = -1;
    int prochaine = 0, en_cours = 0, finies = 0, echecs = 0;
    double t0 = maintenant();

    while (finies < n) {
        while (en_cours < coeurs && prochaine < n) {
            int s = 0;
            while (occupe[s] >= 0) s++;
            lancer(&t[prochaine], prochaine, epingler ? cpus[s] : -1, sortie, exe, delai);
            if (t[prochaine].pid > 0) { occupe[s] = prochaine; en_cours++; }
            else { finies++; echecs++; }
            prochaine++;
        }
        if (en_cours == 0) continue;
        int st;
        pid_t p = wait(&st);
        if (p < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "wait : %s\n", strerror(errno));
            break;
        }
        int s = 0;
        while (s < coeurs && (occupe[s] < 0 || t[occupe[s]].pid != p)) s++;
        if (s == coeurs) continue;            /* not one of ours */
        Tache *x = &t[occupe[s]];
        x->duree = maintenant() - x->debut;
        x->statut = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + WTERMSIG(st);
        if (x->statut) echecs++;
        occupe[s] = -1;
        en_cours--;
        finies++;
        if (x->statut || finies % 25 == 0 || finies == n) {
            printf("[multi] %d/%d  %.1fs%s", finies, n, maintenant() - t0,
                   x->statut ? "" : "\n");
            if (x->statut)
                printf("  tache %d : statut %d (%s/%05d.log)\n", (int)(x - t) + 1,
                       x->statut, sortie, (int)(x - t) + 1);
            fflush(stdout);
        }
    }

    char bilan[4096];
    snprintf(bilan, sizeof(bilan), "%s/bilan.tsv", sortie);
    FILE *b = fopen(bilan, "w");
    if (b) {
        fprintf(b, "tache\tstatut\tsecondes\tcpu\tligne\n");
        for (int k = 0; k < n; k++)
            fprintf(b, "%d\t%d\t%.3f\t%d\t%s\n", k + 1, t[k].statut, t[k].duree,
                    t[k].cpu, t[k].ligne);
        fclose(b);
    }
    double total = maintenant() - t0, cumul = 0;
    for (int k = 0; k < n; k++) cumul += t[k].duree;
    printf("[multi] termine : %d tache(s), %d echec(s), %.1fs ecoulees pour %.1fs de calcul"
           " (x%.1f) ; bilan %s\n", n, echecs, total, cumul, total > 0 ? cumul / total : 0, bilan);
    return echecs ? 1 : 0;
}
