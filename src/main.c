/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * c54x_exe - the Calypso TMS320C54x DSP, run without QEMU and without the ARM.
 *
 * Bench for the symptom recorded in the qemu-calypso README [2026-09-16]:
 * a_sch[0] = 0x8100 (B_BLUD | B_SCH_CRC), and a_sch[3] = 0xf8d8 CONSTANT over
 * 21/21 writes while 10 distinct burst contents were presented. A decoder
 * whose output does not depend on its input is not decoding.
 *
 * Reaching that point used to mean booting QEMU, the ARM and the osmocom-bb
 * firmware: seconds per run. The DSP needs almost none of it - out of 26631
 * lines of C54x L1, 14 symbols come from QEMU (two of them mutexes in the
 * core), and calypso_{arm2dsp,dma,fbsb,mailbox}.c need none. Here the TI mask
 * ROM runs alone in milliseconds and we watch what the DSP writes into API
 * RAM, which makes the question "does the output depend on the input?"
 * answerable in a loop, hence in CI.
 *
 * The sources are NOT copied here: this binary compiles those of
 * /opt/GSM/qosmo. Copying them would recreate the divergence this bench exists
 * to remove.
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
#include "rejouer.h"
#include "pont.h"

/* ── what the platform would otherwise provide ─────────────────────────── */
extern int c54x_rapide;      /* calypso_c54x.h : fast path of the core */
extern int c54x_sondes;      /* c54x_internal.h : sondes pures, coupees par defaut */
uint32_t g_c54x_exe_fn;      /* non-static: pont.c updates it on every TICK */
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }

/* No interrupt controller here, so the ack the DSP raises has nowhere to go.
 * If DSP behaviour ever turns out to depend on that ack, this binary diverges
 * from QEMU at exactly this point. */
void calypso_inth_arm_ack(void) { }

/* The DARAM lock normally lives in l1-dsp/calypso_full_pcb.c, excluded from
 * this binary because it includes hw/core/cpu.h - all of QEMU behind it. Same
 * pthread_mutex: the DSP really locks. */
QemuMutex calypso_pcb_daram_lock;

/* Guest memory: with no ARM nobody writes into it, but shared sources
 * reference it. */
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{
    (void)addr;
    if (!wr) {
        memset(buf, 0, len);
    }
}

/* ── ROMs at their silicon addresses (cf. calypso_trx.c of qosmo-dsp) ───── */
static const struct { const char *suffixe; uint32_t adresse; bool programme; }
ROMS[] = {
    { "PROM0",  0x07000, true  },
    { "PROM1",  0x18000, true  },   /* page 1, reached with XPC=1 */
    { "PROM2",  0x28000, true  },
    { "PROM3",  0x38000, true  },
    { "DROM",   0x09000, false },
    { "PDROM",  0x0E000, false },   /* mapped on the DATA side ... */
    { "PDROM",  0x0E000, true  },   /* ... AND on the PROGRAM side (IT vectors) */
};

static void usage(const char *prog)
{
    fprintf(stderr,
        "usage: %s [options]\n"
        "  --rom-dir DIR     ou sont calypso_dsp.*.bin   (defaut /opt/GSM)\n"
        "  --trames N        nombre de trames TDMA       (defaut 100, 4000 avec --rejouer)\n"
        "  --insns N         instructions par trame      (defaut 2300, 200000 avec --arm)\n"
        "  --verbeux         une ligne par trame\n"
        "  --iq MODE         avec --arm : injecter un burst I/Q a chaque trame :\n"
        "                    fcch (rotation +pi/2/ech.), noise, tone:<dphi>, none,\n"
        "                    cell[:bsic[:decalage]] = FCCH+SCH+factice GMSK (multitrame 51)\n"
        "  --amp N           amplitude int16 des echantillons injectes (defaut 30000)\n"
        "  --rejouer         rejeu DETERMINISTE de l'acquisition FB/SB (sans QEMU)\n"
        "  --bsic N          BSIC de la cellule injectee en rejeu (defaut 7)\n"
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
    int rejeu = 0, bsic_rej = 7;
    const char *iq_mode = "none";
    int amp = 30000;
    long trames = -1, insns = -1;
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
        else if (!strcmp(a, "--rejouer")) { rejeu = 1; }
        else if (!strcmp(a, "--bsic") && i + 1 < argc) bsic_rej = atoi(argv[++i]);
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
        /* [2026-09-20] 200000 under --arm: the FB search over whole 1250-symbol
         * frames costs the ROM 24-36k instructions per frame, and a frame cut
         * short by the budget leaves the ARM reading half-written results (a
         * zeroed a_sch read as a CRC-OK SB). Silicon has ~360k cycles per frame
         * at 78 MHz; 32000 was a bench constant, not a hardware one. */
        insns = arm_sock ? 200000 : 2300;
    }
    if (trames < 0) {
        trames = rejeu ? 4000 : 100;       /* an explicit --trames is honoured as is */
    }
    verbosite_installer(niveau);
    /* Fast path of the core (calypso_c54x.h): on unless a probe is armed. */
    { const char *d = getenv("CALYPSO_DEBUG"), *r = getenv("CALYPSO_C54X_RAPIDE");
      c54x_rapide = (r && *r) ? (*r != '0') : !(d && *d); }
    /* [2026-09-23] Sondes pures du coeur (c54x_internal.h) : coupees par defaut ;
     * -vvvv et plus les allument, puisque c'est a ce niveau qu'on les lit.
     * Sinon le coeur resout CALYPSO_SONDES / CALYPSO_DEBUG au premier c54x_init. */
    if (niveau >= 4) c54x_sondes = 1;

    qemu_mutex_init(&calypso_pcb_daram_lock);

    /* API RAM: private, or shared with the QEMU ARM. It must be installed
     * BEFORE the ROMs, since the loader copies into it whatever falls inside
     * the 0x0800 window. Under --arm the shared segment IS data[0x0800..] and
     * api_ram aliases it, so every core write reaches the ARM by either
     * path. */
    static uint16_t api_ram_privee[CALYPSO_API_WORDS];
    uint16_t *api_ram = api_ram_privee;
    C54xState *dsp;
    if (arm_sock || rejeu) {
        /* A pending interrupt (IFR&IMR) is taken as soon as INTM drops: real
         * C54x behaviour (SPRU131 ch.6), not a workaround. The core gates it
         * behind CALYPSO_C54X_IRQ_LEVEL; turn it on here unless explicitly
         * overridden (empty CALYPSO_C54X_IRQ_LEVEL turns it off). Applied to
         * BOTH benches: the replay used to run the core without it and could
         * not reproduce the bridge. */
        const char *e = getenv("CALYPSO_C54X_IRQ_LEVEL");
        if (!e) setenv("CALYPSO_C54X_IRQ_LEVEL", "1", 1);
        else if (!*e) unsetenv("CALYPSO_C54X_IRQ_LEVEL");
    }
    if (arm_sock) {
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

    if (rejeu) {
        calypso_dma_init();
        calypso_bsp_init(dsp);
        if (insns < 32000) insns = 32000;
        if (!iq_mode || !*iq_mode || !strcmp(iq_mode, "none")) iq_mode = "cell";
        int rc = rejouer(dsp, api_ram, trames, insns, iq_mode, amp, bsic_rej, verbeux);
        verbosite_retirer();
        verbosite_bilan(stdout);
        return rc;
    }
    if (arm_sock) {
        /* Same sequence as calypso_trx_init() of qosmo-dsp after reset. */
        calypso_dma_init();
        calypso_bsp_init(dsp);
        int rc = pont_serveur(dsp, api_ram, arm_sock, insns, verbeux, iq_mode, amp);
        verbosite_retirer();
        verbosite_bilan(stdout);
        return rc;
    }

    /* The README observables, sampled once per frame. */
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

        /* Count TRANSITIONS, not frames where the value is non-zero: the bit
         * stays raised, so counting per frame reported "50 out of 50", which
         * only measured how long the run lasted. */
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
