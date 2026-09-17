/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_exe - le DSP TMS320C54x du Calypso, sans QEMU et sans ARM.
 *
 * [2026-09-16] Pourquoi ce binaire existe. Le README de qemu-calypso documente
 * un mur precis en mode natif : « a_sch[0] = 0x8100 (B_BLUD | B_SCH_CRC) »,
 * et surtout « a_sch[3] sort 0xf8d8, CONSTANT sur 21/21 ecritures, alors que
 * 10 contenus de burst distincts lui ont ete presentes. Un decodeur dont la
 * sortie ne depend pas de l'entree ne decode pas. »
 *
 * Pour observer ca, il fallait jusqu'ici booter QEMU, l'ARM, le firmware
 * osmocom-bb et toute la pile - des secondes par essai, et une trentaine de
 * variables entre la question et la reponse. Alors que la mesure d'accroche
 * dit que le DSP n'a presque pas besoin de tout ca : sur 26 631 lignes de L1
 * C54x, QUATORZE symboles viennent de QEMU, dont deux seulement dans le coeur
 * (des mutex), et calypso_{arm2dsp,dma,fbsb,mailbox}.c n'en demandent aucun.
 *
 * Ici : la mask-ROM TI tourne seule, en millisecondes, et on regarde ce que le
 * DSP ecrit dans l'API RAM. Ce qui rend la question « la sortie depend-elle de
 * l'entree ? » repondable en boucle, donc en CI.
 *
 * Les sources ne sont PAS recopiees : ce binaire compile celles de
 * /opt/GSM/qosmo. Recopier, c'etait refaire la divergence qu'on vient de
 * supprimer.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "qemu/thread.h"
#include "calypso_c54x.h"
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_dsp_pont.h"
#include "calypso_dma.h"
#include "calypso_bsp.h"
#include "verbosite.h"
#include "pont.h"

/* ── ce que la plateforme fournirait ───────────────────────────────────── */
uint32_t g_c54x_exe_fn;      /* non static : pont.c le met a jour a chaque TICK */
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }

/* Sans controleur d'interruptions, l'acquittement emis par le DSP n'a nulle
 * part ou aller. Inerte, et c'est exact : le C54x acquitte dans le vide ici.
 * Si un jour le comportement du DSP depend de cet acquittement, ce binaire
 * divergera de QEMU a cet endroit precis - et c'est ici qu'il faudra regarder. */
void calypso_inth_arm_ack(void) { }

/* Le verrou DARAM vit normalement dans calypso_full_pcb.c, exclu de ce binaire
 * parce qu'il inclut hw/core/cpu.h - tout QEMU derriere. Ce n'est pas un
 * leurre : c'est le meme pthread_mutex, le DSP se verrouille pour de vrai. */
QemuMutex calypso_pcb_daram_lock;

/* Memoire « invitee » : sans ARM, personne n'ecrit dedans, mais des sources
 * partagees la referencent. */
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{
    (void)addr;
    if (!wr) {
        memset(buf, 0, len);
    }
}

/* ── les ROM, a leurs adresses silicium (cf. calypso_trx.c de qosmo-dsp) ── */
static const struct { const char *suffixe; uint32_t adresse; bool programme; }
ROMS[] = {
    { "PROM0",  0x07000, true  },
    { "PROM1",  0x18000, true  },   /* page 1, atteignable via XPC=1 */
    { "PROM2",  0x28000, true  },
    { "PROM3",  0x38000, true  },
    { "DROM",   0x09000, false },
    { "PDROM",  0x0E000, false },   /* visible cote DATA ... */
    { "PDROM",  0x0E000, true  },   /* ... ET cote PROGRAMME (vecteurs IT) */
};

static void usage(const char *prog)
{
    fprintf(stderr,
        "usage: %s [options]\n"
        "  --rom-dir DIR     ou sont calypso_dsp.*.bin   (defaut /opt/GSM)\n"
        "  --trames N        nombre de trames TDMA       (defaut 100)\n"
        "  --insns N         instructions par trame      (defaut 2300, 32000 avec --arm)\n"
        "  --verbeux         une ligne par trame\n"
        "  --iq MODE         avec --arm : injecter un burst I/Q a chaque trame :\n"
        "                    fcch (rotation +pi/2/ech.), noise, tone:<dphi>, none,\n"
        "                    cell[:bsic[:decalage]] = FCCH+SCH+factice GMSK (multitrame 51)\n"
        "  --amp N           amplitude int16 des echantillons injectes (defaut 30000)\n"
        "  --arm [SOCKET]    servir l'ARM de QEMU (qosmo, CALYPSO_DSP_EXTERN=1) :\n"
        "                    API RAM partagee dans /dev/shm%s, trame verrouillee\n"
        "                    sur %s\n"
        "  -v .. -vvvvvv     traces du coeur C54x (stderr), par niveau :\n"
        "                    (rien)  erreurs seulement\n"
        "                    -v      + avertissements\n"
        "                    -vv     + cycle de vie : boot, reset, gates ACTIF/INACTIVE\n"
        "                    -vvv    + taches, API RAM, interruptions, chemins FB/SB\n"
        "                    -vvvv   + sondes memoire : WATCH, DUMP, SCAN, MAP, SP-*\n"
        "                    -vvvvv  + pas a pas : BRANCH-TRACE, LOOPTRACE, TERM, CYCLE\n"
        "                    -vvvvvv tout, stderr brut\n",
        prog, CALYPSO_PONT_SHM, CALYPSO_PONT_SOCK);
}

int main(int argc, char **argv)
{
    const char *rom_dir = "/opt/GSM";
    const char *arm_sock = NULL;
    const char *iq_mode = "none";
    int amp = 30000;
    long trames = 100, insns = -1;
    bool verbeux = false;
    int niveau = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--rom-dir") && i + 1 < argc)      rom_dir = argv[++i];
        else if (!strcmp(a, "--trames") && i + 1 < argc)  trames = atol(argv[++i]);
        else if (!strcmp(a, "--insns") && i + 1 < argc)   insns = atol(argv[++i]);
        else if (!strcmp(a, "--verbeux"))                 verbeux = true;
        else if (!strcmp(a, "--iq") && i + 1 < argc)      iq_mode = argv[++i];
        else if (!strcmp(a, "--amp") && i + 1 < argc)     amp = atoi(argv[++i]);
        else if (!strcmp(a, "--arm")) {
            arm_sock = (i + 1 < argc && argv[i + 1][0] != '-') ? argv[++i] : CALYPSO_PONT_SOCK;
        }
        else if (a[0] == '-' && a[1] == 'v' && strspn(a + 1, "v") == strlen(a + 1)) {
            niveau = (int)strlen(a + 1);
        }
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); return 0; }
        else { usage(argv[0]); return 2; }
    }
    if (insns < 0) {
        insns = arm_sock ? 32000 : 2300;   /* 32000 = CALYPSO_DSP_BUDGET de bsp.env */
    }
    verbosite_installer(niveau);

    qemu_mutex_init(&calypso_pcb_daram_lock);

    /* L'API RAM : privee, ou partagee avec l'ARM de QEMU. Posee AVANT les ROM,
     * le chargeur y recopie ce qui tombe dans la fenetre 0x0800. En mode --arm
     * le segment partage EST data[0x0800..] et api_ram en est l'alias : toute
     * ecriture du coeur, par l'un ou l'autre chemin, est vue de l'ARM. */
    static uint16_t api_ram_privee[CALYPSO_API_WORDS];
    uint16_t *api_ram = api_ram_privee;
    C54xState *dsp;
    if (arm_sock) {
        /* [2026-09-17] Une IT en attente (IFR&IMR) est prise des que INTM
         * retombe : c'est le C54x (SPRU131 §6), pas une bequille. Le coeur le
         * gate derriere CALYPSO_C54X_IRQ_LEVEL ; on l'allume ici sauf avis
         * contraire explicite (CALYPSO_C54X_IRQ_LEVEL= vide pour l'eteindre). */
        { const char *e = getenv("CALYPSO_C54X_IRQ_LEVEL");
          if (!e) setenv("CALYPSO_C54X_IRQ_LEVEL", "1", 1);
          else if (!*e) unsetenv("CALYPSO_C54X_IRQ_LEVEL"); }
        dsp = pont_allouer_dsp();
        if (!dsp) { return 1; }
        api_ram = &dsp->data[C54X_API_BASE];
    } else {
        dsp = c54x_init();
        if (!dsp) { fprintf(stderr, "c54x_init a echoue\n"); return 1; }
    }
    c54x_set_api_ram(dsp, api_ram);

    int charges = 0;
    for (unsigned i = 0; i < sizeof(ROMS) / sizeof(ROMS[0]); i++) {
        char chemin[512];
        snprintf(chemin, sizeof(chemin), "%s/calypso_dsp.%s.bin", rom_dir, ROMS[i].suffixe);
        int n = c54x_load_section(dsp, chemin, ROMS[i].adresse, ROMS[i].programme);
        if (n < 0) {
            fprintf(stderr, "ROM manquante : %s\n", chemin);
            return 1;
        }
        printf("  %-6s %-5s 0x%05x  %6d mots\n", ROMS[i].suffixe,
               ROMS[i].programme ? "prog" : "data", ROMS[i].adresse, n);
        charges++;
    }
    { char chemin[512];
      snprintf(chemin, sizeof(chemin), "%s/calypso_dsp.Registers.bin", rom_dir);
      int n = c54x_load_registers(dsp, chemin);
      printf("  %-6s %-5s %-7s  %6d mots\n", "Regs", "mmr", "", n); }

    printf("%d sections chargees, reset...\n", charges);
    c54x_reset(dsp);

    if (arm_sock) {
        /* Comme calypso_trx_init() de qosmo-dsp apres le reset. */
        calypso_dma_init();
        calypso_bsp_init(dsp);
        int rc = pont_serveur(dsp, api_ram, arm_sock, insns, verbeux, iq_mode, amp);
        verbosite_retirer();
        verbosite_bilan(stdout);
        return rc;
    }

    /* Les observables du README, echantillonnes a chaque trame. */
    uint16_t *d_fb_det = &api_ram[(API_NDB + NDB_D_FB_DET) / 2];
    uint16_t *a_sch0   = &api_ram[(API_R_PAGE(0) + RP_A_SCH) / 2];

    unsigned fb_vus = 0, sch_ecrits = 0;
    uint16_t fb_precedent = 0, sch_sig_prec = 0xFFFF;
    uint16_t sch3_premier = 0; bool sch3_varie = false, sch3_vu = false;

    if (verbeux) {
        printf("\n  trame    d_fb_det   a_sch[0]  a_sch[1]  a_sch[2]  a_sch[3]\n");
    }
    for (long t = 0; t < trames; t++) {
        g_c54x_exe_fn = (uint32_t)t;
        c54x_run(dsp, (int)insns);

        /* [2026-09-16] On compte les TRANSITIONS, pas les trames ou la valeur
         * est non nulle : un bit reste leve, et le compter a chaque trame
         * donnait « 50 sur 50 » qui ne mesurait que la duree du test. */
        if (*d_fb_det && !fb_precedent) fb_vus++;
        fb_precedent = *d_fb_det;

        uint16_t sch_sig = (uint16_t)(a_sch0[0] ^ a_sch0[3]);
        if ((a_sch0[0] || a_sch0[3]) && sch_sig != sch_sig_prec) {
            sch_ecrits++;
            if (!sch3_vu) { sch3_premier = a_sch0[3]; sch3_vu = true; }
            else if (a_sch0[3] != sch3_premier) sch3_varie = true;
            sch_sig_prec = sch_sig;
        }
        if (verbeux) {
            printf("  %5ld    %6u     0x%04x    0x%04x    0x%04x    0x%04x\n",
                   t, *d_fb_det, a_sch0[0], a_sch0[1], a_sch0[2], a_sch0[3]);
        }
    }

    verbosite_retirer();
    printf("\n─── bilan sur %ld trames ───\n", trames);
    printf("  AUCUN burst injecte : le DSP tourne sur une API RAM vierge, sans\n"
           "  ARM ni TPU. Les valeurs ci-dessous disent que la mask-ROM EXECUTE,\n"
           "  pas encore qu'elle decode. Injecter des bursts est l'etape suivante.\n\n");
    printf("  d_fb_det leve      : %u transition(s) 0 -> 1\n", fb_vus);
    printf("  a_sch change       : %u fois\n", sch_ecrits);
    if (sch3_vu) {
        printf("  a_sch[3]           : 0x%04x%s\n", sch3_premier,
               sch3_varie ? " puis VARIE" : " CONSTANT sur toutes les ecritures");
        if (!sch3_varie) {
            printf("\n  ^ c'est le symptome du README : une sortie qui ne depend\n"
                   "    pas de l'entree. Ici l'entree est vide, donc attendu.\n");
        }
    } else {
        printf("  a_sch              : jamais ecrit (la tache SB n'a pas tourne)\n");
    }
    verbosite_bilan(stdout);
    return 0;
}
