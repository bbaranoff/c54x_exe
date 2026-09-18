/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * rejouer.c — REJEU DETERMINISTE de l'acquisition FB/SB sur le vrai DSP.
 *
 * Pourquoi. Deux runs identiques du banc complet (QEMU + osmocon + mobile)
 * divergeaient : l'ARM (TCG) avance a la vitesse de l'hote entre deux IT trame,
 * donc la trame ou il poste la tache FB change d'un run a l'autre. Un echec
 * fragile ne se debugue pas. Ici l'ARM n'est PAS emule : sa couche 1 est
 * REJOUEE en C (l1_sync + fb_sched_set/sb_sched_set de prim_fbsb.c/sync.c),
 * l'injection est verrouillee sur le compteur de trames, et rien ne lit
 * l'horloge de l'hote. Meme entree => meme sortie, toujours.
 *
 * Ce que ca N'EST PAS : un mode de fonctionnement. C'est un banc de mesure du
 * DSP (FB puis SB) avec la meme sequence de commandes que le vrai firmware.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "calypso_c54x.h"
#include "calypso_bsp.h"
#include "hw/arm/calypso/calypso_api.h"
#include "cellule.h"
#include "calypso_twl3025.h"
#include "rejouer.h"

/* ---- offsets API en MOTS depuis la base API (= DSP 0x800) ---------------- */
#define W_PAGE(p)     ((p) ? 0x14u : 0x00u)   /* T_DB_MCU_TO_DSP, 17 mots */
#define R_PAGE(p)     ((p) ? 0x3Cu : 0x28u)   /* T_DB_DSP_TO_MCU, 20 mots */
#define W_SIZE        17u
#define R_SIZE        20u
#define W_TASK_D      0u
#define W_TASK_MD     4u
#define W_CTRL_ABB    11u
#define W_AFC         15u
#define W_CTRL_SYS    16u
#define NDB           0xD4u
#define NDB_PAGE      (NDB + 0u)
#define NDB_ERRSTAT   (NDB + 1u)
#define NDB_FB_DET    (NDB + 36u)
#define NDB_FB_MODE   (NDB + 37u)
#define NDB_SYNC      (NDB + 38u)   /* a_sync_demod[TOA,PM,ANGLE,SNR] */
#define R_SERV        8u            /* a_serv_demod[4] */
/* [2026-09-18] DECALAGE D'ECHANTILLONNAGE. Le rejeu echantillonnait le GMSK a
 * 0,0, c'est-a-dire A LA FRONTIERE de symbole, alors que le pont utilise 0,5, le
 * CENTRE. Avec la primitive gaussienne de gmsk.c (BT=0,3), le poids du symbole
 * visee dans la difference de phase entre deux echantillons consecutifs vaut :
 *
 *   decalage   symbole visee   voisin precedent   voisin suivant
 *     0,0          0,454            0,023              0,492
 *     0,25         0,590            0,069              0,333
 *     0,5          0,653            0,158              0,189
 *
 * A 0,0 le voisin SUIVANT pese plus lourd que le symbole qu'on veut lire :
 * l'offset n'est pas bruite, il est INDETERMINE, et le demodulateur peut
 * s'accrocher indifferemment a l'un ou a l'autre. C'est ce qui explique d'un seul
 * coup offset=22 pour marge=21, le midambule a 56/63, les donnees a 71-73/78 avec
 * des echantillons pourtant parfaits, et le pic de correlation qui saute entre
 * rangs adjacents. Le pont, lui, etait juste depuis le debut. */
#define DECALAGE_SYMB 0.5
/* marge de tete reglable (REJEU_MARGE, defaut 21) ; la queue complete a 190 complexes */
static int marge_tete(void)
{
    static int m = -1;
    if (m < 0) { const char *e = getenv("REJEU_MARGE"); m = e ? atoi(e) : 21;
                 if (m < 0 || m > 41) m = 21;
                 cellule_marge_fin = 190 - 148 - m; }
    return m;
}

#define R_SCH         15u           /* a_sch[5] */
#define B_SCH_CRC     8
#define B_GSM_TASK    2
#define B_AFC         4
#define FB_DSP_TASK   5
#define SB_DSP_TASK   6
#define BITS_PER_TDMA 156

/* seuils du firmware (prim_fbsb.c : SNR non gatant, #else FB*_SNR_THRESH=0) */
#define THRESH1       (11000 - 1000)
#define THRESH2       (1000 - 200)
#define AFC_RETRY_MAX 30
#define FB0_RETRY_MAX 3
/* sync.h : ANGLE_TO_FREQ(a) = a * BITFREQ_DIV_PI / ANG2FREQ_SCALING */
#ifndef BITFREQ_DIV_PI
#define BITFREQ_DIV_PI   86208   /* sync.h:203 : 270kHz/pi */
#endif
#ifndef ANG2FREQ_SCALING
#define ANG2FREQ_SCALING (2<<15) /* sync.h:204 : fx1.15 */
#endif
#define ANGLE2FREQ(a) ((int)(int16_t)(a) * BITFREQ_DIV_PI / ANG2FREQ_SCALING)

/* ---- etat ARM rejoue ---------------------------------------------------- */
typedef void (*cb_t)(int attempt);
struct item { int frame; cb_t cb; int attempt; };

static uint16_t *api;
static C54xState *dsp;
static unsigned w_page, r_page, r_page_used;
static uint32_t fn_cur;              /* l1s.current_time.fn */
static unsigned long histo_pc[64];
static unsigned long insn_total;
static unsigned char *vu_sb, *vu_hors;
static int16_t g_livre_iq[2*256]; static int g_livre_niq;
static char g_livre_type; static uint32_t g_livre_fn; static int g_livre_n;
static int      afc_dac = -700;      /* afc_initial_dac_value (compal) */
static int      fb_mode, afc_retries, fb0_retries;
static struct   item sched[32]; static int n_sched;
static struct   { int toa, pm, angle, snr, freq_diff, attempt; uint32_t fnr; } fb;
static int      verdict;             /* 0 en cours, 1 SB OK, -1 abandon */
static uint32_t sb_word; static int sb_bsic; static uint32_t sb_fn;
static int      n_fb_ok, n_sb_try, n_sb_crcfail, n_crc_ok, n_sb_vraies;
static int      n_err_dsp, n_err8;
static long     n_cmps_sb, n_cmps_hors, n_e0_sb, n_sttrn_sb;
static long     n_e0x_sb[4];
static int      n_firs_vus;
static long     n_cmps_reg[64];
static long     n_xc_reg[64];
static int      n_xc_vus;
static int      n_bal;
static int      n_sat_vus;
static int      n_ecr;
static int      n_2a0;   /* E0=FIRS E1=LMS E2=SQDST E3=ABDST */
static int      g_bsic_injecte;   /* BSIC reellement emis par la cellule */
static int      trace;
static unsigned long hit_7c31, hit_9841, hit_84a1, hit_770a, hit_b219, hit_7a16;

static void plan(int delay, cb_t cb, int attempt)
{
    if (n_sched >= 32) return;
    sched[n_sched].frame = (int)fn_cur + delay;
    sched[n_sched].cb = cb; sched[n_sched].attempt = attempt; n_sched++;
}
static void sched_reset(void) { n_sched = 0; }

static uint16_t *dbw(void) { return &api[W_PAGE(w_page)]; }
static uint16_t *dbr(void) { return &api[R_PAGE(r_page)]; }

/* ---- callbacks : copie fidele de prim_fbsb.c ---------------------------- */
static void fbdet_cmd(int unused);
static void fbdet_resp(int attempt);
static void sbdet_cmd(int attempt);

/* [2026-09-18] PIEGE CORRIGE : on testait `getenv("X")`, VRAI meme pour X=0. Une
 * ligne ecrite `REJEU_SCH_PARTOUT=0 REJEU_SB_FORCE=0` croyait couper les hacks
 * alors qu'elle les ACTIVAIT tous. Desormais "0", "" et "off" valent faux. */
static int drapeau_env(const char *nom)
{
    const char *e = getenv(nom);
    if (!e || !*e) return 0;
    if (e[0] == '0' && e[1] == 0) return 0;
    if (!strcmp(e, "non") || !strcmp(e, "no") || !strcmp(e, "off")) return 0;
    return 1;
}

static void plan_sb_annule(void) { }

static void sbdet_resp(int attempt);

static void plan_fb_set(int delay, int mode)
{
    fb_mode = mode;
    plan(delay + 0, fbdet_cmd, 0);
    for (int a = 1; a <= 12; a++) plan(delay + 1 + a, fbdet_resp, a);
}
static void plan_sb_set(int delay)
{
    plan(delay + 0, sbdet_cmd, 1);
    plan(delay + 1, sbdet_cmd, 2);
    plan(delay + 3, sbdet_resp, 1);
    plan(delay + 4, sbdet_resp, 2);
}

static uint32_t fb_cmd_fn;      /* trame ou l'ARM a poste la tache FB */
static void fbdet_cmd(int unused)
{
    (void)unused;
    fb_cmd_fn = fn_cur;
    dbw()[W_TASK_MD]  = FB_DSP_TASK;
    api[NDB_FB_MODE]  = (uint16_t)fb_mode;
}

static void fbdet_resp(int attempt)
{
    if (!api[NDB_FB_DET]) {
        if (attempt < 12) return;
        sched_reset();
        if (fb0_retries < FB0_RETRY_MAX) { fb0_retries++; plan_fb_set(1, 0); }
        else verdict = -1;
        return;
    }
    /* read_fb_result */
    fb.toa   = (int16_t)api[NDB_SYNC + 0];
    fb.pm    = (int16_t)api[NDB_SYNC + 1] >> 3;
    fb.angle = (int16_t)api[NDB_SYNC + 2];
    fb.snr   = (int16_t)api[NDB_SYNC + 3];
    fb.freq_diff = ANGLE2FREQ(fb.angle);
    fb.fnr = fn_cur; fb.attempt = attempt;
    api[NDB_FB_DET] = 0; api[NDB_SYNC + 0] = 0;
    n_fb_ok++;
    if (trace) {
        /* FCCH reelle = premiere trame >= cmd avec p51 in {0,10,20,30,40} */
        uint32_t f = fb_cmd_fn; while ((f % 51) % 10 != 0 || (f % 51) > 40) f++;
        int dframe = (int)(f - fb_cmd_fn);                 /* trame FCCH dans la fenetre */
        int toa_attendu = dframe * BITS_PER_TDMA + 23;      /* ce que le firmware sait lire */
        int ntdma_lu = (fb.toa - 23) / BITS_PER_TDMA;
        printf("  FB%d att=%d fn=%u TOA=%d (df=%d) | cmd=%u FCCH reelle=%u (+%d trames) "
               "=> TOA attendu ~%d, ntdma lu=%d au lieu de %d\n",
               fb_mode, attempt, fn_cur, fb.toa, fb.freq_diff,
               fb_cmd_fn, f, dframe, toa_attendu, ntdma_lu, dframe);
    }
    /* afc_correct : delta = (norm * err)/slope ; slope compal_e88 = 287 */
    afc_dac += (int)(((32768 / 947) * (long)fb.freq_diff) / 287);
    if (afc_dac > 4095) afc_dac = 4095; if (afc_dac < -4096) afc_dac = -4096;
    sched_reset();
    if (fb_mode == 0) {
        if (abs(fb.freq_diff) < THRESH1) plan_fb_set(1, 1);
        else if (afc_retries < AFC_RETRY_MAX) { afc_retries++; plan_fb_set(1, 0); }
        else verdict = -1;
    } else {
        int toa = fb.toa - 23, ntdma, qbits;
        if (toa < 0) { qbits = (toa + BITS_PER_TDMA) * 4; ntdma = -1; }
        else { ntdma = toa / BITS_PER_TDMA; qbits = (toa - ntdma * BITS_PER_TDMA) * 4; }
        int fn_offset = (int)fn_cur - attempt + ntdma;
        int delay = fn_offset + 11 - (int)fn_cur - 1;
        if (trace) printf("    -> toa-23=%d ntdma=%d qbits=%d delay=%d\n", toa, ntdma, qbits, delay);
        if (abs(fb.freq_diff) < THRESH2) {
            if (delay < 0) delay = 0; if (delay > 20) delay = 20;
            /* [diag] REJEU_SB_FORCE=1 : viser la PROCHAINE trame SCH (p51 in
             * {1,11,21,31,41}) au lieu du delay calcule. Isole le DEMODULATEUR du
             * CADENCAGE : si le CRC passe ici, seul le cadencage (ntdma) est en
             * cause ; s'il echoue, c'est bien la demodulation SB. */
            static int force = -1;
            if (force < 0) force = drapeau_env("REJEU_SB_FORCE") ? 1 : 0;
            if (force) {
                uint32_t f = fn_cur + 1;
                /* [2026-09-18] REJEU_SB_FN=<n> vise une trame ABSOLUE, la meme dans
                 * tous les runs. Sans cela la cible est « la prochaine trame SCH »,
                 * qui depend de l'instant ou la FB converge : deux runs n'atterrissent
                 * pas sur la meme trame, donc ne comparent pas le meme mot de code, et
                 * toute mesure differentielle devient ininterpretable. */
                /* Le forcage sur une trame ABSOLUE ne marche pas : le champ delay du
                 * firmware est borne a 20 trames, donc viser une trame lointaine ne
                 * planifie rien du tout. La confusion d'appariement est supprimee
                 * autrement, a la source : on ne perturbe QUE la trame observee
                 * (REJEU_PERTURBER_FN), si bien que tout l'ordonnancement en amont est
                 * rigoureusement identique d'un run a l'autre. */
                while (!((f % 51) % 10 == 1 && (f % 51) <= 41)) f++;
                delay = (int)(f - fn_cur);
                if (trace) printf("    [force] SB vise fn=%u (p51=%u), delay=%d\n", f, f % 51, delay);
            }
            plan_sb_set(delay);
        } else plan_fb_set(1, 1);
    }
}

static void sbdet_cmd(int attempt)
{
    if (trace) { unsigned p = fn_cur % 51;
        printf("  SBcmd att=%d fn=%u p51=%u %s\n", attempt, fn_cur, p,
               (p % 10 == 1 && p <= 41) ? "<- trame SCH (bon)" : "<- PAS une trame SCH"); }
    dbw()[W_TASK_MD] = SB_DSP_TASK;
    api[NDB_FB_MODE] = 0;
}

static void sbdet_resp(int attempt)
{
    n_sb_try++;
    r_page_used = 1;
    if (trace) {
        /* internes du demodulateur SB (cf. RE §9.4 et §9.19) :
         * 0x2f06 = index du pic de correlation
         * 0x2bf8 = drapeau CRC interne
         * 0x2c72 = LES 78 BITS SOUPLES. [2026-09-18] Cette sonde lisait 0x2a00,
         * qui est le tampon 296 mots du correlateur FB : d'ou les « souples »
         * qui ne prenaient que 0x0000 et 0xffff et la fausse piste d'une
         * magnitude annihilee par un decalage. Les vrais bits souples, en
         * 0x2c72, ont des magnitudes variees. On imprime aussi leur etendue. */
        {
            int16_t mn = 32767, mx = -32768, nnul = 0;
            for (int k = 0; k < 78; k++) {
                int16_t v = (int16_t)dsp->data[0x2c72 + k];
                if (v) nnul++;
                if (v < mn) mn = v;
                if (v > mx) mx = v;
            }
            printf("    [sb-int] pic(2f06)=%u  crc(2bf8)=%u  souples(2c72)[0..5]=%04x %04x %04x %04x %04x %04x"
                   "  non nuls=%d/78  etendue=[%d..%d]\n",
                   dsp->data[0x2f06], dsp->data[0x2bf8],
                   dsp->data[0x2c72], dsp->data[0x2c73], dsp->data[0x2c74],
                   dsp->data[0x2c75], dsp->data[0x2c76], dsp->data[0x2c77], nnul, mn, mx);
        }
    }
    if (trace) { unsigned p = fn_cur % 51;
        printf("  SBresp att=%d fn=%u p51=%u crc=%s\n", attempt, fn_cur, p,
               (dbr()[R_SCH + 0] & (1 << B_SCH_CRC)) ? "FAUX" : "OK"); }
    if (dbr()[R_SCH + 0] & (1 << B_SCH_CRC)) {
        n_sb_crcfail++;
        if (attempt == 2) { sched_reset(); plan_fb_set(1, 0); }
        return;
    }
    sb_word = dbr()[R_SCH + 3] | ((uint32_t)dbr()[R_SCH + 4] << 16);
    sb_bsic = (sb_word >> 2) & 0x3f;
    unsigned t1 = ((sb_word >> 23) & 1) | ((sb_word >> 7) & 0x1fe) | ((sb_word << 9) & 0x600);
    unsigned t2 = (sb_word >> 18) & 0x1f;
    unsigned t3p = ((sb_word >> 24) & 1) | ((sb_word >> 15) & 6);
    unsigned t3 = t3p * 10 + 1;
    sb_fn = 51u * ((t3 - t2 + 26u) % 26u) + t3 + 26u * 51u * t1;
    /* Un CRC OK n'est un decodage que s'il rend le BSIC INJECTE et un T3 valide
     * (T3 <= 50 par construction). On qualifie chaque passage et on continue au
     * lieu de s'arreter au premier. REJEU_ARRET_1ER=1 retablit l'arret. */
    /* [2026-09-18] TROISIEME CONDITION : la FN reconstruite doit valoir la trame
     * courante. Sans elle on ne qualifie jamais le seul chiffre qui compte.
     *
     * ATTENTION, REJEU_SCH_PARTOUT=1 CORROMPT LA VERITE DE REFERENCE : cellule.c
     * calcule t3p = p51 / 10 pour n'importe quel p51, ce qui n'est juste que sur
     * 1, 11, 21, 31 et 41. Sur p51=45 la cellule encode T3'=4, que le decodeur
     * reconstruit en T3=41, donc sb_fn != fn_cur meme pour un decodage PARFAIT.
     * La FN ne peut donc etre validee que SANS ce drapeau. */
    int t3_ok = (t3 <= 50), bsic_ok = (sb_bsic == (unsigned)g_bsic_injecte);
    int fn_ok = (sb_fn == fn_cur);
    n_crc_ok++;
    if (bsic_ok && t3_ok && fn_ok) n_sb_vraies++;
    printf("  SB%d fn=%u : sb=0x%08x BSIC=%d (injecte %d) T1=%u T2=%u T3=%u -> FN=%u  %s%s\n",
           attempt, fn_cur, sb_word, sb_bsic, g_bsic_injecte, t1, t2, t3, sb_fn,
           (bsic_ok && t3_ok && fn_ok) ? "** VRAIE **"
           : !bsic_ok ? "FAUX POSITIF (BSIC ne colle pas)"
           : !t3_ok   ? "FAUX POSITIF (T3 > 50, impossible)"
           :            "FAUX POSITIF (FN ne colle pas)",
           cellule_sch_partout ? "  [FN non qualifiable sous SCH_PARTOUT]" : "");
    if (drapeau_env("REJEU_ARRET_1ER") || (bsic_ok && t3_ok && fn_ok)) { verdict = 1; return; }
    /* [2026-09-18] SANS CECI LE BANC SE FIGE, ET LE FIGEAGE SE LIT COMME UN
     * RESULTAT. Un CRC OK non qualifie (faux positif) ne replanifiait rien : la
     * file d'items se vidait, plus aucune commande n'etait postee, et le rejeu
     * traversait les milliers de trames restantes sans rien demander au DSP. Le
     * bilan etait alors IDENTIQUE a 1200 et a 12000 trames -- non parce que le
     * DSP stagnait, mais parce que l'ARM rejoue s'etait tu. Le branchement
     * d'echec CRC, lui, relancait bien (plan_fb_set sur attempt 2) : seule la
     * sortie « CRC OK mais faux positif » etait un cul-de-sac. Le vrai firmware
     * relance une acquisition tant qu'il n'a pas de SB valide ; on fait pareil,
     * ce qui rend enfin le nombre de trames significatif. */
    sched_reset(); plan_fb_set(1, 0);
}


/* ---- init DSP cote ARM : ce que fait dsp_power_on() du firmware ---------
 * Sans cela le DSP tourne sur une API RAM vierge : les seuils de detection FB
 * (d_fb_thr_det_iacq/track) et les marges (d_fb_margin_beg/end, qui alimentent
 * la formule de TOA en 0x794b/0x7956) valent 0, et la detection FB part en
 * vrille des la premiere trame. Valeurs = dsp_params.c / dsp_ndb_init(). */
#define PARAM   0x431u          /* BASE_API_PARAM (API 0x862) en mots */
#define A_SCH26 (NDB + 42u)
/* bootloader DSP : offsets API en MOTS (dsp.c : BASE_API_RAM + 0x0ff8..0x0ffe) */
#define BL_ADDR_HI_W  0x7FCu
#define BL_SIZE_W     0x7FDu
#define BL_ADDR_LO_W  0x7FEu
#define BL_STATUS_W   0x7FFu
#define DSP_START     0x7000u

static long pump(long max, int stop_on_idle)
{
    long b = 0;
    while (b < max && dsp->running) {
        int ex = c54x_run(dsp, 256);
        if (ex <= 0) break;
        b += ex;
        if (stop_on_idle && dsp->idle) break;
    }
    return b;
}

static void arm_dsp_init(void)
{
    memset(api, 0, 0x2000u * sizeof(uint16_t));          /* dsp_api_memset(API) */

    /* dsp_pre_boot() : le DSP vient d'etre reset ; attendre BL_STATUS_IDLE */
    long b = 0;
    while (api[BL_STATUS_W] != 1 && b < 8000000) {
        int ex = c54x_run(dsp, 256); if (ex <= 0) break; b += ex;
    }
    printf("  bootloader DSP : BL_STATUS=%u apres %ld insn\n", api[BL_STATUS_W], b);

    /* dsp_set_params() : NDB d'abord */
    api[NDB + 8]  = 0x0074;  api[NDB + 9]  = 0x0001;
    api[NDB + 10] = 0x0154;  api[NDB + 11] = 0x17ff;
    api[NDB + 12] = 7;       api[NDB + 13] = 0;
    api[NDB + 14] = 3;                                  /* d_dsp_state = IDLE3 */
    /* puis la table de parametres (dsp_params.c). Les 4 premiers champs FB
     * alimentent la formule de TOA (0x794b/0x7956) et les seuils de detection. */
    static const int16_t P[] = {
        0x6666, 15, 12, 5, 4, 0x7002, 1, 0xE, 0, 0, 0, 0,
        24, 22, 296, 30,                                 /* margin_beg/end, nsubb_idle/dedic */
        0x3333, (int16_t)0x28f6,                         /* fb_thr_det_iacq / _track */
        0x7fff, 17408, 26624, 20152,
        7872, -4, 7872, 5772, 7872, 53, -892, 208,
    };
    for (unsigned i = 0; i < sizeof P / sizeof P[0]; i++) api[PARAM + i] = (uint16_t)P[i];

    /* dsp_bl_start_at(DSP_START) */
    api[BL_ADDR_HI_W] = 0; api[BL_ADDR_LO_W] = DSP_START; api[BL_SIZE_W] = 0;
    api[BL_STATUS_W]  = 2;                               /* BL_CMD_COPY_BLOCK */
    b = pump(4000000, 1);
    printf("  DSP demarre : %ld insn, idle=%d, version=0x%04x%04x\n",
           b, dsp->idle, api[NDB + 6], api[NDB + 7]);

    /* dsp_ndb_init() : ce qui compte pour FB/SB */
    api[NDB + 2]  = 0x0179;                              /* d_spcx_rif */
    api[NDB + 3]  = 0x0800 | ((8 - 4) << 7);             /* d_tch_mode */
    api[NDB_FB_MODE] = 1;
    api[NDB_FB_DET]  = 0;
    api[A_SCH26]     = (1u << B_SCH_CRC);
    /* dsp_db_init() */
    memset(&api[W_PAGE(0)], 0, W_SIZE * sizeof(uint16_t));
    memset(&api[W_PAGE(1)], 0, W_SIZE * sizeof(uint16_t));
    memset(&api[R_PAGE(0)], 0, R_SIZE * sizeof(uint16_t));
    memset(&api[R_PAGE(1)], 0, R_SIZE * sizeof(uint16_t));
}

/* ---- une trame : l1_sync() puis le DSP --------------------------------- */
static void l1_sync(void)
{
    r_page_used = 0;
    memset(dbw(), 0, W_SIZE * sizeof(uint16_t));          /* memset db_w */
    dbw()[W_AFC] = (uint16_t)(int16_t)afc_dac;            /* afc_load_dsp */
    dbw()[W_CTRL_ABB] |= (1u << B_AFC);
    /* [2026-09-17] BOUCLE AFC FERMEE. qemu-src relaie l'ecriture d_afc vers le
     * modele TWL3025 (calypso_trx.c, offsets 0x001E/0x0046) ; qosmo ne le fait
     * NULLE PART (set_afc_dac n'y est jamais appele) : la boucle etait ouverte,
     * la rotation des echantillons ne bougeait pas, l'erreur de frequence ne
     * convergeait donc jamais sous le seuil SB (800 Hz). */
    calypso_twl3025_set_afc_dac((int16_t)afc_dac);
    if (api[NDB_ERRSTAT]) {                                /* comme sync.c:249 */
        static int n; if (trace && n < 6) { printf("  DSP Error Status: %u\n", api[NDB_ERRSTAT]); n++; }
        api[NDB_ERRSTAT] = 0;
    }
    /* Executer les items de CETTE trame. Un callback peut en planifier un
     * nouveau pour la trame COURANTE (delay=0 : c'est le cas quand le firmware
     * calcule delay=0 apres FB1) ; il faut donc re-balayer jusqu'a epuisement,
     * sinon cet item-la est silencieusement perdu -- et c'est precisement la
     * commande SB visant la bonne trame SCH. */
    for (int tour = 0; tour < 8; tour++) {
        int fait = 0;
        for (int i = 0; i < n_sched; i++) {
            if (sched[i].frame == (int)fn_cur && sched[i].cb) {
                cb_t cb = sched[i].cb; int at = sched[i].attempt;
                sched[i].cb = NULL;
                cb(at); fait = 1;
                if (verdict) return;
            }
        }
        if (!fait) break;
    }
    if (r_page_used) {
        memset(dbr(), 0, R_SIZE * sizeof(uint16_t));
        dbr()[R_SCH + 0] = (1u << B_SCH_CRC);
        r_page ^= 1;
    }
    api[NDB_PAGE] = (uint16_t)((1u << (B_GSM_TASK - 1)) | w_page);  /* B_GSM_TASK=bit1 */
    w_page ^= 1;
}

int rejouer(C54xState *d, uint16_t *api_ram, long trames, long insns,
            const char *iq_mode, int amp, int bsic, int verbeux)
{
    dsp = d; api = api_ram; trace = verbeux;
    w_page = r_page = r_page_used = 0; fn_cur = 0; afc_dac = -700;
    fb_mode = 0; afc_retries = fb0_retries = 0; n_sched = 0; verdict = 0;
    n_fb_ok = n_sb_try = n_sb_crcfail = n_crc_ok = n_sb_vraies = 0;
    n_err_dsp = n_err8 = 0;
    g_bsic_injecte = bsic;
    memset(&fb, 0, sizeof fb);

    if (drapeau_env("REJEU_SCH_PARTOUT")) { cellule_sch_partout = 1; printf("  [stimulus] SCH sur toutes les trames non-FCCH (masque le cadencage)\n"
               "  [stimulus] ATTENTION : t3p = p51/10 n'est juste que sur p51 in {1,11,21,31,41},\n"
               "             donc la FN reconstruite ne peut PAS etre validee sous ce drapeau.\n"); }
    printf("rejeu deterministe : %ld trames max, %ld insn/trame, cellule BSIC=%d, iq=%s\n",
           trames, insns, bsic, iq_mode ? iq_mode : "cell");
    /* dsp_power_on() du firmware : boot bootloader + parametres + NDB */
    arm_dsp_init();
    plan_fb_set(1, 0);                       /* premier FBSB_REQ */

    int16_t iq[2 * 256]; int n_iq;
    for (long t = 0; t < trames && !verdict; t++) {
        fn_cur = (uint32_t)t;
        g_c54x_exe_fn = fn_cur;
        l1_sync();
        /* [2026-09-18] Le firmware (sync.c) lit d_error_status a chaque trame,
         * l'imprime et le remet a zero. Le rejeu ne le regardait pas du tout, d'ou
         * son silence sur les « DSP Error Status: 8 » visibles cote firmware.
         * 8 = DSP_ERR_DMA_PROG : debordement de l'anneau de jobs DMA 0x4330
         * (orm *(0x3f92),#8 en 0xaa83), point deja ouvert au rapport. */
        if (api[NDB_ERRSTAT]) {
            unsigned e = api[NDB_ERRSTAT];
            n_err_dsp++;
            if (e & 8) n_err8++;
            if (n_err_dsp <= 8)
                printf("  [erreur DSP] status=%u%s a fn=%u\n", e,
                       (e & 8) ? " (bit 3 = DSP_ERR_DMA_PROG, anneau DMA 0x4330 sature)" : "",
                       fn_cur);
            api[NDB_ERRSTAT] = 0;
        }
        if (verdict) break;
        /* [2026-09-17] ORDRE MATERIEL. Sur silicium : l'ARM poste la tache ->
         * le DSP la lit sur l'IT trame et ARME sa fenetre RX (DMA) -> les
         * echantillons arrivent -> le DSP les traite. On injectait AVANT que le
         * DSP n'ait arme, si bien que le transfert utilisait la programmation DMA
         * de la tache PRECEDENTE (celle de la FB) et la SB recevait le mauvais
         * burst. REJEU_RX_AVANT=1 restaure l'ancien ordre pour comparer. */
        static int rx_avant = -1;
        if (rx_avant < 0) rx_avant = drapeau_env("REJEU_RX_AVANT") ? 1 : 0;
        n_iq = 2 * 148;
        int injecter = (!iq_mode || strcmp(iq_mode, "none") != 0);
        if (injecter && rx_avant) {
            g_livre_type = cellule_burst(fn_cur, (uint8_t)bsic, amp, DECALAGE_SYMB, marge_tete(), iq, &n_iq);
            g_livre_fn = fn_cur; g_livre_n = n_iq;
            calypso_bsp_rx_burst(0, fn_cur, iq, n_iq);
        }
        /* IT trame : le DSP lit la tache et arme sa fenetre RX */
        if (dsp->imr & (1u << 12)) c54x_interrupt_ex(dsp, 28, 12);
        if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
        if (injecter && !rx_avant) {
            /* laisser le DSP armer (budget court), PUIS livrer les echantillons */
            long arm = 0;
            while (arm < insns / 4 && dsp->running && !dsp->idle) {
                int ex = c54x_run(dsp, 64); if (ex <= 0) break; arm += ex;
            }
            g_livre_type = cellule_burst(fn_cur, (uint8_t)bsic, amp, DECALAGE_SYMB, marge_tete(), iq, &n_iq);
            g_livre_fn = fn_cur; g_livre_n = n_iq;
            /* [2026-09-18] Perturber UN echantillon brut, sans passer par les bits :
             * cela cartographie l'influence index par index, sans confusion due au
             * packing du burst ni au changement d'ordonnancement qu'entraine une
             * inversion de bit code. REJEU_PERTURBER_ECH=<n> ajoute un delta a
             * l'echantillon complexe n du tampon livre. */
            { static int pe = -2; static long pf = -2;
              if (pe == -2) { const char *e = getenv("REJEU_PERTURBER_ECH"); pe = e ? atoi(e) : -1; }
              if (pf == -2) { const char *e = getenv("REJEU_PERTURBER_FN");  pf = e ? atol(e) : -1; }
              if (pe >= 0 && 2 * pe + 1 < n_iq && (pf < 0 || (long)fn_cur == pf)) {
                  iq[2 * pe]     = (int16_t)(iq[2 * pe]     + 3000);
                  iq[2 * pe + 1] = (int16_t)(iq[2 * pe + 1] - 3000);
              } }
            memcpy(g_livre_iq, iq, (size_t)n_iq * sizeof(int16_t)); g_livre_niq = n_iq;
            calypso_bsp_rx_burst(0, fn_cur, iq, n_iq);
            if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
        }
        long done = 0;
        static int probe = -1;
        if (probe < 0) probe = drapeau_env("REJEU_PROBE_TOA") ? 1 : 0;
        if (!probe) {
            /* [2026-09-18] CE CHEMIN MENTAIT. Il executait 256 instructions d'un coup
             * sans jamais regarder un seul PC, si bien que hit_b219/7c31/84a1/9841/770a
             * restaient a zero — et le bilan les imprimait quand meme. La ligne
             * "job b219=0 ... corr FB 770a=0" d'un run sans REJEU_PROBE_TOA ne disait
             * donc PAS que la chaine SB n'avait pas tourne : elle disait que personne
             * ne comptait. On compte desormais TOUJOURS, au pas a pas, et on tient un
             * histogramme des PC pour savoir ce que le DSP execute reellement. */
            while (done < insns && dsp->running && !dsp->idle) {
                uint16_t pc = dsp->pc & 0xffff;
                histo_pc[pc >> 10]++;
                switch (pc) {
                case 0x7c31: hit_7c31++; break;
                case 0x9841: hit_9841++; break;
                case 0x84a1: hit_84a1++; break;
                case 0x770a: hit_770a++; break;
                case 0xb219: hit_b219++; break;
                case 0x7a16: hit_7a16++; break;
                default: break; }
                /* [2026-09-18] Ces compteurs vivaient dans le chemin de sondage
                 * seulement : meme defaut que les compteurs de la chaine SB. Ils
                 * sont desormais renseignes dans les DEUX chemins. */
                { uint16_t o = dsp->prog[pc]; uint8_t h = o >> 8;
                  static int dans_sb2;
                  if (pc == 0x7c31) dans_sb2 = 1;
                  if (pc == 0x9841) dans_sb2 = 0;
                  if (dans_sb2) {
                      if (h == 0x8E || h == 0x8F) n_cmps_sb++;
                      if (h >= 0xE0 && h <= 0xE3) { n_e0_sb++; n_e0x_sb[h - 0xE0]++; }
                      /* [2026-09-18] FIRS lit ses coefficients en Pmem[pmad], avec
                       * pmad ~ 0x0061. Or 0x0060-0x007F est le SCRATCH-PAD DARAM du
                       * C54x, et la fenetre OVLY du coeur ne debute qu'a 0x0080 :
                       * prog_read(0x61) retombe donc sur prog[] ou rien n'est charge
                       * sous 0x7000. Si les coefficients ne vivent qu'en data[], FIRS
                       * multiplie par du vide et sa sortie ne peut pas dependre de
                       * l'entree. On lit les DEUX espaces au moment du FIRS. */
                      if (h == 0xE0 && n_firs_vus < 6) {
                          uint16_t pmad = dsp->prog[(pc + 1) & 0xffff];
                          int nzp = 0, nzd = 0;
                          for (int k = 0; k < 6; k++) {
                              if (dsp->prog[(pmad + k) & 0xffff]) nzp++;
                              if (dsp->data[(pmad + k) & 0x3fff]) nzd++;
                          }
                          /* [2026-09-18] Lire prog[] BRUT est trompeur : avec
                           * PMST_OVLY arme et le plancher d'alias a 0x0060 (gate
                           * CALYPSO_OVLY_SCRATCH, defaut 1), une lecture PROGRAMME
                           * dans 0x0060-0x27FF est redirigee vers data[]. On reproduit
                           * donc la traduction du coeur pour savoir ce que FIRS LIT
                           * REELLEMENT, au lieu de ce que contient le tableau prog[]. */
                          int ovly = (dsp->pmst & 0x0020) != 0;
                          int alias = ovly && pmad >= 0x0060 && pmad < 0x2800;
                          printf("    [firs] pc=%04x pmad=%04x PMST=%04x OVLY=%d alias=%s"
                                 " -> FIRS lit", pc, pmad, dsp->pmst, ovly,
                                 alias ? "data[] (scratch-pad visible)" : "prog[] (PAS d'alias)");
                          for (int k = 0; k < 6; k++)
                              printf(" %04x", alias ? dsp->data[(pmad + k) & 0x3fff]
                                                    : dsp->prog[(pmad + k) & 0xffff]);
                          printf("\n");
                          printf("    [firs] pc=%04x pmad=%04x | prog[pmad..+5]=", pc, pmad);
                          for (int k = 0; k < 6; k++) printf(" %04x", dsp->prog[(pmad + k) & 0xffff]);
                          printf(" (%d non nuls)\n                      | data[pmad..+5]=", nzp);
                          for (int k = 0; k < 6; k++) printf(" %04x", dsp->data[(pmad + k) & 0x3fff]);
                          printf(" (%d non nuls)\n", nzd);
                          n_firs_vus++;
                      }
                  } else if (h == 0x8E || h == 0x8F) n_cmps_hors++;
                  /* [2026-09-18] Ma fenetre `dans_sb` se fermait a 0x9841, donc elle
                   * EXCLUAIT le decodeur SCH et son Viterbi (0x9a78). Le « 0 CMPS dans
                   * le demod SB » ne disait donc rien sur le Viterbi. On compte CMPS
                   * par region de PC, sans fenetre, ce qui est sans ambiguite. */
                  if (h == 0x8E || h == 0x8F) n_cmps_reg[pc >> 10]++;
                  /* [2026-09-18] QUI ECRIT 0x2a00 ? Le decodeur lit 78 mots a 0x2a00 et les
                 * trouve TOUS NULS a son entree. Soit l'etage d'egalisation n'y ecrit
                 * jamais, soit quelque chose les efface avant. On surveille donc la zone
                 * 0x2a00..0x2a8d pendant TOUT le job SB (pas seulement le demod), en notant
                 * le PC de chaque modification et le sens (vers une valeur, ou vers zero). */
                /* [2026-09-18] 0x8389 = 0x4594 = `ADD *AR4+,16,A,B` (0x4400/0xFC00, bit9=src,
                 * bit8=dst). Le coeur n'a AUCUN gestionnaire pour 0x4400-0x47FF : le seul de
                 * la zone est (op & 0xFC00) == 0x4000, qui ne couvre que SUB. On verifie donc
                 * si B change bien a travers cette instruction. */
                { static int nav; static int64_t bavant;
                  if (pc == 0x8389) { bavant = dsp->b; nav = 1; }
                  else if (nav == 1 && pc == 0x838a) {
                      static int n=0;
                      if (n < 4) {
                          printf("    [add-4594] B avant=%010llx  B apres=%010llx  A=%010llx  %s\n",
                                 (unsigned long long)(bavant & 0xffffffffffULL),
                                 (unsigned long long)(dsp->b & 0xffffffffffULL),
                                 (unsigned long long)(dsp->a & 0xffffffffffULL),
                                 (dsp->b == bavant) ? "B INCHANGE => instruction NON EXECUTEE" : "B modifie");
                          n++;
                      }
                      nav = 0;
                  } }
                { static int nb838d;
                  if (pc == 0x838e && nb838d < 6) {
                      printf("    [B@838d] B=%010llx  A=%010llx  AR6=%04x  BRC=%u  ST1=%04x\n",
                             (unsigned long long)(dsp->b & 0xffffffffffULL),
                             (unsigned long long)(dsp->a & 0xffffffffffULL),
                             dsp->ar[6], dsp->brc, dsp->st1);
                      nb838d++;
                  } }
                { static uint16_t omb2[142]; static int arme6; static long ecr[64], vers0[64];
                  static uint16_t pmin[64], pmax[64];
                  static struct { uint16_t pc; long v, z; } parpc2[12];
                  if (pc == 0xb219) {
                      for (int k = 0; k < 142; k++) omb2[k] = dsp->data[0x2a00 + k];
                      arme6 = 1;
                      for (int b = 0; b < 64; b++) { ecr[b] = 0; vers0[b] = 0; }
                      for (int t = 0; t < 12; t++) { parpc2[t].pc = 0; parpc2[t].v = 0; parpc2[t].z = 0; }
                  }
                  if (arme6) {
                      for (int k = 0; k < 142; k++) {
                          if (dsp->data[0x2a00 + k] != omb2[k]) {
                              /* [2026-09-18] Bucketiser par PC EXACT, et separer les
                               * ecritures de valeur des mises a zero : 284 modifications
                               * dont 142 vers zero veut dire que quelque chose ecrit les
                               * 142 bits souples puis les EFFACE tous. Il faut nommer les
                               * deux instructions. Rappel : le PC imprime est celui de
                               * l'instruction SUIVANTE. */
                              { int z = (dsp->data[0x2a00 + k] == 0);
                                for (int t = 0; t < 12; t++) {
                                    if (parpc2[t].pc == 0 || parpc2[t].pc == pc) {
                                        parpc2[t].pc = pc;
                                        if (z) parpc2[t].z++; else parpc2[t].v++;
                                        break;
                                    }
                                } }
                              int b = pc >> 10;
                              if (!ecr[b]) { pmin[b] = pc; pmax[b] = pc; }
                              if (pc < pmin[b]) pmin[b] = pc;
                              if (pc > pmax[b]) pmax[b] = pc;
                              ecr[b]++;
                              if (dsp->data[0x2a00 + k] == 0) vers0[b]++;
                              omb2[k] = dsp->data[0x2a00 + k];
                          }
                      }
                  }
                  if (pc == 0x9841 && arme6 && n_2a0 < 3) {
                      printf("    [qui-2a00] modifications de 0x2a00..0x2a8d pendant le job SB :\n");
                      long tot = 0;
                      for (int b = 0; b < 64; b++)
                          if (ecr[b]) {
                              printf("        region 0x%04x : %ld modifs (dont %ld vers zero), PC 0x%04x a 0x%04x\n",
                                     b << 10, ecr[b], vers0[b], pmin[b], pmax[b]);
                              tot += ecr[b];
                          }
                      if (!tot) printf("        AUCUNE : rien n'ecrit jamais dans 0x2a00\n");
                      printf("        par PC exact (PC imprime = instruction SUIVANTE) :\n");
                      for (int t = 0; t < 12; t++)
                          if (parpc2[t].pc)
                              printf("            pc=%04x (donc ecrivain %04x) : %ld valeurs, %ld mises a ZERO\n",
                                     parpc2[t].pc, parpc2[t].pc - 1, parpc2[t].v, parpc2[t].z);
                      n_2a0++;
                  } }
                /* [2026-09-18] QUI ECRIT LES 78 BITS SOUPLES, ET A QUELLE ADRESSE ?
                 * Le balayage des 78 bits code montre que la premiere moitie atterrit en
                 * position b+3 et la seconde a DEUX positions distantes de 51 (b-38 et
                 * b+13), donc que les deux blocs de donnees se RECOUVRENT au lieu de se
                 * concatener. On cherche les sites d'ecriture : ombre de la zone, et on
                 * note le PC des que le contenu change. */
                { static uint16_t ombre[78]; static int arme5; static long parpc[64];
                  static uint16_t pcmin[64], pcmax[64]; static long seq;
                  if (dans_sb2) {
                      if (!arme5) { for (int k = 0; k < 78; k++) ombre[k] = dsp->data[0x2c72 + k]; arme5 = 1; seq = 0; }
                      for (int k = 0; k < 78; k++) {
                          if (dsp->data[0x2c72 + k] != ombre[k]) {
                              int b = pc >> 10;
                              if (!parpc[b]) { pcmin[b] = pc; pcmax[b] = pc; }
                              if (pc < pcmin[b]) pcmin[b] = pc;
                              if (pc > pcmax[b]) pcmax[b] = pc;
                              parpc[b]++;
                              /* [2026-09-18] CHRONOLOGIE EXACTE : numero d'ordre, PC, indice
                               * touche, ancienne et nouvelle valeur. Permet de savoir (a) dans
                               * quel ORDRE les deux passes ecrivent, donc qui detruit qui, et
                               * (b) si deux ecritures au meme indice portent la MEME valeur
                               * (copie mal adressee) ou des valeurs DIFFERENTES (deux moities
                               * d'une somme qui devaient atterrir ensemble). */
                              if (n_ecr == 0 && seq < 100)
                                  printf("    [w] %3ld pc=%04x k=%2d  %04x -> %04x  "
                                         "AR0=%04x AR1=%04x AR2=%04x AR3=%04x AR4=%04x AR5=%04x AR6=%04x AR7=%04x BRC=%u\n",
                                         seq, pc, k, ombre[k], dsp->data[0x2c72 + k],
                                         dsp->ar[0], dsp->ar[1], dsp->ar[2], dsp->ar[3],
                                         dsp->ar[4], dsp->ar[5], dsp->ar[6], dsp->ar[7], dsp->brc);
                              seq++;
                              ombre[k] = dsp->data[0x2c72 + k];
                          }
                      }
                  } else if (arme5 && n_ecr < 2) {
                      printf("    [ecrit-2c72] sites qui modifient les 78 bits souples :\n");
                      for (int b = 0; b < 64; b++)
                          if (parpc[b])
                              printf("        region 0x%04x : %ld ecritures, PC de 0x%04x a 0x%04x\n",
                                     b << 10, parpc[b], pcmin[b], pcmax[b]);
                      n_ecr++; arme5 = 0;
                      for (int b = 0; b < 64; b++) parpc[b] = 0;
                  } }
                /* [2026-09-18] POURQUOI LES BORDS SONT PERDUS. Le profil en max|delta|
                 * montre trois ordres de grandeur entre le milieu du burst (13452,
                 * 19534, valeurs extremes REPETEES) et ses bords (2 a 30). Des valeurs
                 * extremes identiques qui reviennent, c'est une SATURATION. Or dans un
                 * MLSE en virgule fixe les metriques de chemin DOIVENT etre
                 * renormalisees a chaque pas ; sans cela elles croissent, saturent, et
                 * la contribution relative des symboles de bord s'effondre. On compte
                 * donc les saturations d'accumulateur et l'etat des drapeaux pendant le
                 * demod SB. */
                { static long nsat, nova, novb; static int ovm_vu;
                  if (dans_sb2) {
                      int64_t a = dsp->a, b = dsp->b;
                      if (a > 0x7FFFFFFFLL || a < -0x80000000LL) nsat++;
                      if (b > 0x7FFFFFFFLL || b < -0x80000000LL) nsat++;
                      if (dsp->st0 & 0x0400) nova++;      /* OVA */
                      if (dsp->st0 & 0x0200) novb++;      /* OVB */
                      if (dsp->st1 & 0x0200) ovm_vu = 1;  /* OVM : mode saturation */
                  } else if ((nsat || nova || novb) && n_sat_vus < 3) {
                      printf("    [saturation] demod SB : %ld depassements 32 bits,"
                             " OVA pose %ld fois, OVB %ld fois, mode OVM %s\n",
                             nsat, nova, novb, ovm_vu ? "ACTIF" : "inactif");
                      n_sat_vus++; nsat = nova = novb = 0;
                  } }
                /* [2026-09-18] OU EST L'ADRESSE DE LA FENETRE FIXE ? On surveille les
                 * registres d'adresse qui PARCOURENT le tampon d'entree 0x0cce pendant
                 * le demod SB, et on imprime, par registre, la plage d'index
                 * d'echantillon parcourue et le PC qui l'a posee. Si cette plage ne se
                 * deplace PAS de 10 quand on passe de la marge 21 a 31, l'adresse est
                 * calculee une fois et reutilisee telle quelle. On imprime aussi BK :
                 * une taille de bloc circulaire mal emulee ferait boucler l'acces sur
                 * une fenetre fixe de la bonne longueur avec une base correcte. */
                { static uint16_t amin[8], amax[8], apc[8]; static long an[8];
                  static int arme4;
                  if (dans_sb2) {
                      if (!arme4) { for (int k = 0; k < 8; k++) { amin[k] = 0xffff; amax[k] = 0; an[k] = 0; } arme4 = 1; }
                      for (int k = 0; k < 8; k++) {
                          uint16_t v = dsp->ar[k];
                          if (v >= 0x0cce && v < 0x0cce + 380) {
                              uint16_t idx = (uint16_t)((v - 0x0cce) / 2);
                              if (idx < amin[k]) { amin[k] = idx; apc[k] = pc; }
                              if (idx > amax[k]) amax[k] = idx;
                              an[k]++;
                          }
                      }
                  } else if (arme4 && n_bal < 3) {
                      printf("    [tampon-AR] parcours de 0x0cce pendant le demod SB (BK=%u) :\n", dsp->bk);
                      for (int k = 0; k < 8; k++)
                          if (an[k])
                              printf("        AR%d : echantillons %u..%u (%u larges), %ld acces, 1er a pc=%04x\n",
                                     k, amin[k], amax[k], amax[k] - amin[k] + 1, an[k], apc[k]);
                      n_bal++; arme4 = 0;
                  } }
                /* [2026-09-18] HASARD DE PIPELINE DU XC. Sur C54x la condition du
                   * XC est echantillonnee deux cycles avant son execution : une
                   * instruction qui pose le drapeau juste avant le XC n'est pas encore
                   * visible. Le coeur, lui, evalue au moment de l'execution. Le hasard
                   * ne mord donc QUE si la ROM place le poseur de drapeau a moins de
                   * deux rangs du XC (du code ecrit pour du vrai silicium ne le fait
                   * normalement pas). On mesure la DISTANCE reelle, au lieu de la
                   * supposer : on garde les 4 derniers PC et le ST0 avant chacun. */
                  { static uint16_t ring_pc[4], ring_op[4], ring_st0[4]; static int ri;
                    if ((h == 0xFD || h == 0xFF) && pc >= 0x8400 && pc < 0x8800 && n_xc_vus < 10) {
                        printf("    [xc] pc=%04x op=%04x cc=%02x | 3 precedentes :", pc, o, o & 0xff);
                        for (int k = 3; k >= 1; k--) {
                            int q = (ri - k) & 3;
                            printf("  %04x:%04x(ST0=%04x)", ring_pc[q], ring_op[q], ring_st0[q]);
                        }
                        printf(" | ST0 au XC=%04x\n", dsp->st0);
                        n_xc_vus++;
                    }
                    ring_pc[ri] = pc; ring_op[ri] = o; ring_st0[ri] = dsp->st0;
                    ri = (ri + 1) & 3;
                    if (h == 0xFD || h == 0xFF) n_xc_reg[pc >> 10]++; } }
                int ex = c54x_run(dsp, 1);
                if (ex <= 0) break;
                done += ex;
                insn_total++;
            }
        } else {
            /* pas-a-pas autour du calcul de TOA (0x7940..0x795c) : A, B, T, 0x3fb4 */
            static int nlog;
            while (done < insns && dsp->running && !dsp->idle) {
                uint16_t pc = dsp->pc & 0xffff;
                /* Le demod SB ecrit 78 bits souples en 0x2a00 (repack 0x7e94) ;
                 * le decodeur 0x9841 les relit. Le correlateur FB utilise LE MEME
                 * tampon. On compare les deux instants pour prouver l'ecrasement. */
                { static uint16_t apres_demod[8]; static int arme, np;
                  if (pc == 0x7e94 && !arme) {
                      for (int k = 0; k < 8; k++) apres_demod[k] = dsp->data[0x2a00 + k];
                      arme = 1;
                  } else if (pc == 0x9841 && arme && np < 6) {
                      int diff = 0;
                      for (int k = 0; k < 8; k++) if (dsp->data[0x2a00 + k] != apres_demod[k]) diff = 1;
                      printf("    [2a00] apres demod: %04x %04x %04x %04x | a l'entree du decodeur: %04x %04x %04x %04x  => %s\n",
                             apres_demod[0], apres_demod[1], apres_demod[2], apres_demod[3],
                             dsp->data[0x2a00], dsp->data[0x2a01], dsp->data[0x2a02], dsp->data[0x2a03],
                             diff ? "ECRASE" : "intact");
                      np++; arme = 0;
                  } }
                /* [2026-09-17] PREUVE que les echantillons portent le message : on
                 * demodule le MEME tampon 0x0cce que lit le DSP, avec un demodulateur
                 * de reference ecrit en C. S'il retrouve le mot de code et pas le DSP,
                 * le signal est bon et le defaut est entierement dans le DSP emule. */
                /* Le tampon contient-il EXACTEMENT les echantillons livres pour cette
                 * trame, ou un melange de plusieurs bursts ? Le midambule est identique
                 * dans toutes les SCH : un melange le renforcerait tout en noyant les
                 * donnees, ce qui est precisement le symptome observe. */
                { static int nx;
                  if (pc == 0x9841 && nx < 4 && g_livre_niq) {
                      int ident = 0, n = g_livre_niq < 380 ? g_livre_niq : 380;
                      int prem_diff = -1;
                      for (int k = 0; k < n; k++) {
                          if ((int16_t)dsp->data[0x0cce + k] == g_livre_iq[k]) ident++;
                          else if (prem_diff < 0) prem_diff = k;
                      }
                      printf("    [tampon] identique aux echantillons livres : %d/%d mots"
                             " (1re difference au mot %d)\n", ident, n, prem_diff);
                      nx++;
                  } }
                /* [2026-09-18] BILAN-2A00 : a l entree du decodeur, mesurer ce qu il
                 * LIT reellement, job par job. Les signes des 78 mots compactes en
                 * 0x2a00 contre le mot de code attendu, meilleure trame cherchee comme
                 * dans [ref] (sous REJEU_SCH_PARTOUT la FN livree n est pas fiable).
                 * Les deux polarites sont imprimees : sur du GMSK differentiel la
                 * convention de signe n est pas connue a priori. Sert a relier
                 * « l etage qui reecrit 0x2a00 sort des valeurs » et « le decodeur a
                 * de quoi travailler ». */
                { static int nb2;
                  if (pc == 0x9841 && nb2 < 20) {
                      int16_t sb[78]; int nz = 0, mn = 32767, mx = -32768;
                      for (int k = 0; k < 78; k++) {
                          sb[k] = (int16_t)dsp->data[0x2a00 + k];
                          if (!sb[k]) nz++;
                          if (sb[k] < mn) mn = sb[k];
                          if (sb[k] > mx) mx = sb[k];
                      }
                      int meil_f = -1, meil_s = -1, s0 = -1;
                      for (int df = -12; df <= 3; df++) {
                          long f = (long)g_livre_fn + df; if (f < 0) continue;
                          unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)bsic, a2);
                          int ok = 0;
                          for (int k = 0; k < 78; k++) { int bit = sb[k] < 0 ? 1 : 0; if (bit == a2[k]) ok++; }
                          int meilleur = ok > 78 - ok ? ok : 78 - ok;
                          if (meilleur > meil_s) { meil_s = meilleur; meil_f = (int)f; }
                          if (df == 0) s0 = ok;
                      }
                      printf("    [2a00-lu] job %d : zeros=%d/78 etendue=[%d..%d] |"
                             " signes trame livree %d/78 | meilleure trame %d avec %d/78\n",
                             nb2 + 1, nz, mn, mx, s0, meil_f, meil_s);
                      nb2++;
                  } }
                { static int nr;
                  if (pc == 0x9841 && nr < 4) {
                      int16_t ech[2 * 190];
                      for (int k = 0; k < 380; k++) ech[k] = (int16_t)dsp->data[0x0cce + k];
                      unsigned char b148[148]; int off = 0;
                      unsigned char att[78];
                      cellule_code_attendu(g_livre_fn, (uint8_t)bsic, att);
                      if (cellule_demod_reference(ech, 190, b148, &off) == 0) {
                          /* sanite du demod de reference : le midambule doit ressortir */
                          int okm = 0;
                          for (int i = 0; i < 64; i++) if (b148[42 + i] == cellule_train_sb(i)) okm++;
                          /* bits differentiels bruts : pas de propagation d'erreur */
                          unsigned char d148[148]; int okd = 0, okdd = 0;
                          if (cellule_demod_d(ech, 190, off, d148) == 0) {
                              for (int i = 1; i < 64; i++) {
                                  int att_d = cellule_train_sb(i) ^ cellule_train_sb(i - 1);
                                  if (d148[42 + i] == att_d) okd++;
                              }
                              /* memes bits, polarite inverse */
                              for (int i = 1; i < 64; i++) {
                                  int att_d = cellule_train_sb(i) ^ cellule_train_sb(i - 1);
                                  if (d148[42 + i] != att_d) okdd++;
                              }
                          }
                          /* Les DONNEES aussi, en bits differentiels bruts, contre le
                           * burst complet attendu (3 tail + 39 + midambule + 39 + 3 tail).
                           * Comparer sur b148 n'a aucun sens : le decodage differentiel
                           * propage la moindre erreur sur tout le reste du burst. */
                          int okdat = 0, ndat = 0, meil_f = -1, meil_s = -1;
                          for (int df = -12; df <= 3; df++) {
                              long f = (long)g_livre_fn + df; if (f < 0) continue;
                              unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)bsic, a2);
                              unsigned char burst[148];
                              memset(burst, 0, 3);
                              memcpy(burst + 3, a2, 39);
                              for (int i = 0; i < 64; i++) burst[42 + i] = (unsigned char)cellule_train_sb(i);
                              memcpy(burst + 106, a2 + 39, 39);
                              memset(burst + 145, 0, 3);
                              int ok2 = 0, n2 = 0, prev2 = 1;
                              for (int i = 0; i < 148; i++) {
                                  int att_d = burst[i] ^ prev2; prev2 = burst[i];
                                  int est_donnee = (i >= 3 && i < 42) || (i >= 106 && i < 145);
                                  if (!est_donnee) continue;
                                  n2++; if (d148[i] == att_d) ok2++;
                              }
                              if (ok2 > meil_s) { meil_s = ok2; meil_f = (int)f; }
                              if (df == 0) { okdat = ok2; ndat = n2; }
                          }
                          printf("    [d-brut] midambule %d/63 (inverse %d) | DONNEES trame livree %d/%d"
                                 " | meilleure trame %d avec %d/%d\n",
                                 okd, okdd, okdat, ndat, meil_f, meil_s, ndat);
                          /* le midambule est IDENTIQUE pour toutes les trames SCH : s'il
                           * accroche mais pas les donnees, le tampon contient le burst
                           * d'une AUTRE trame. On cherche donc laquelle. */
                          int meilleure = -1, meilleur_score = -1;
                          for (int df = -12; df <= 3; df++) {
                              long f = (long)g_livre_fn + df; if (f < 0) continue;
                              unsigned char a2[78]; cellule_code_attendu((uint32_t)f, (uint8_t)bsic, a2);
                              int ok2 = 0;
                              for (int i = 0; i < 39; i++) if (b148[3 + i] == a2[i]) ok2++;
                              for (int i = 0; i < 39; i++) if (b148[106 + i] == a2[39 + i]) ok2++;
                              if (ok2 > meilleur_score) { meilleur_score = ok2; meilleure = (int)f; }
                          }
                          int ok = 0;
                          for (int i = 0; i < 39; i++) if (b148[3 + i] == att[i]) ok++;
                          for (int i = 0; i < 39; i++) if (b148[106 + i] == att[39 + i]) ok++;
                          printf("    [ref] fn livree=%u offset=%d | midambule %d/64 |"
                                 " mot de code de la trame livree %d/78 |"
                                 " MEILLEURE trame = %d avec %d/78\n",
                                 g_livre_fn, off, okm, ok, meilleure, meilleur_score);
                      } else {
                          printf("    [ref] fn=%u  demod de reference : pas de pic exploitable\n", g_livre_fn);
                      }
                      nr++;
                  } }
                /* [2026-09-17] Chasse au bug d'ISA suivant. Le chemin FB fonctionne :
                 * tout opcode qu'il execute est donc valide de fait. On liste les
                 * opcodes que SEUL le demodulateur SB execute : c'est la seule
                 * famille qui n'a jamais ete validee, donc le vivier des suspects. */
                { static int dans_sb;
                  if (!vu_sb) { vu_sb = calloc(65536, 1); vu_hors = calloc(65536, 1); }
                  if (pc == 0x7c31) dans_sb = 1;
                  if (pc == 0x9841) dans_sb = 0;
                  /* [2026-09-18] Inverser UN bit code fait bouger 78 positions sur 78
                   * de 0x2c72 : la traceback est globale. Sur C54x cela pointe vers
                   * CMPS (0x8E/0x8F) et le registre TRN. On compte donc qui s'execute
                   * reellement dans le chemin SB : le vrai CMPS, ou la famille 0xE0
                   * (FIRS/LMS/SQDST/ABDST) dont une variante porte encore une
                   * semantique pseudo-CMPS dans le coeur. */
                  { uint16_t o = dsp->prog[pc]; uint8_t h = o >> 8;
                    if (dans_sb) {
                        if (h == 0x8E || h == 0x8F) n_cmps_sb++;
                        if (h >= 0xE0 && h <= 0xE3) { n_e0_sb++; n_e0x_sb[h - 0xE0]++; }
                        if (o == 0x8D00 || (o & 0xFF00) == 0x8D00) n_sttrn_sb++;
                    } else {
                        if (h == 0x8E || h == 0x8F) n_cmps_hors++;
                    } }
                  { uint16_t o = dsp->prog[pc];
                    if (dans_sb) vu_sb[o] = 1; else vu_hors[o] = 1; }
                }
                /* [2026-09-17] Les 78 bits souples sont en 0x2c72 (et NON 0x2a00 :
                 * 0x2a00 est le tampon 296 mots du correlateur FB). On compare le
                 * SIGNE de chacun au bit reellement emis : si les signes concordent,
                 * le demodulateur est sain et le defaut est dans le decodeur ; s'ils
                 * concordent avec un decalage, c'est l'offset du correlateur. */
                { static int nb;
                  if (pc == 0x9841 && nb < 400) {
                      unsigned char att[78];
                      cellule_code_attendu(g_livre_fn, (uint8_t)bsic, att);
                      /* On essaie les conventions plausibles : polarite, moities
                       * echangees (le DSP peut rendre 39+39 dans l'autre ordre), et
                       * un decalage. Si une combinaison monte nettement au-dessus du
                       * hasard, le demodulateur est sain et c'est une convention. */
                          /* [2026-09-18] La famille testee ne contenait PAS le
                           * desentrelacement pair/impair. Le code convolutif de la SCH sort
                           * ses 78 bits dans l'ordre C(2k), C(2k+1) par pas de treillis, et
                           * le burst les range en deux moities de 39. Si le DSP stocke ses
                           * bits souples dans l'ordre INTERNE (tous les pairs puis tous les
                           * impairs), aucun decalage cyclique, aucune polarite et aucun
                           * echange de moities ne le rattrape : chaque combinaison testee
                           * tombe exactement au hasard, et c'est precisement le plateau
                           * observe. On ajoute la permutation et son inverse.
                           *   0 identite   1 moities echangees
                           *   2 pairs d'abord   3 impairs d'abord */
                          int best = -99, bestd = 0, bestv = 0;
                          for (int v = 0; v < 8; v++) {
                              int inv = v & 1, perm = v >> 1;
                              for (int d = -8; d <= 8; d++) {
                                  int ok = 0, tot = 0;
                                  for (int i = 0; i < 78; i++) {
                                      int src;
                                      switch (perm) {
                                      case 1:  src = (i + 39) % 78; break;
                                      case 2:  src = (i % 2 == 0) ? (i / 2) : (39 + (i - 1) / 2); break;
                                      case 3:  src = (i % 2 == 1) ? ((i - 1) / 2) : (39 + i / 2); break;
                                      default: src = i; break;
                                      }
                                  int j = src + d; if (j < 0 || j >= 78) continue;
                                  int16_t sv = (int16_t)dsp->data[0x2c72 + j];
                                  if (!sv) continue;
                                  tot++;
                                  int bit = att[i] != 0; if (inv) bit = !bit;
                                  if ((sv < 0) == bit) ok++;
                              }
                              if (tot >= 40) { int pc2 = ok * 100 / tot;
                                  if (pc2 > best) { best = pc2; bestd = d; bestv = v; } }
                          }
                      }
                      int nznb = 0; for (int i = 0; i < 78; i++) if (dsp->data[0x2c72 + i]) nznb++;
                          /* La sortie du DSP est-elle une TRANCHE du burst a un
                           * offset quelconque ? On correle les 78 signes contre la
                           * sequence differentielle des 148 bits du burst, a tous les
                           * decalages, dans les deux polarites. */
                          { unsigned char burst[148]; int prev3 = 1; unsigned char dref[148];
                            memset(burst, 0, 3); memcpy(burst + 3, att, 39);
                            for (int i = 0; i < 64; i++) burst[42 + i] = (unsigned char)cellule_train_sb(i);
                            memcpy(burst + 106, att + 39, 39); memset(burst + 145, 0, 3);
                            for (int i = 0; i < 148; i++) { dref[i] = burst[i] ^ prev3; prev3 = burst[i]; }
                            int bs = -1, bo = 0, bp = 0;
                            for (int o = -78; o <= 148; o++) {
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok = 0, tot = 0;
                                    for (int i = 0; i < 78; i++) {
                                        int j = o + i; if (j < 0 || j >= 148) continue;
                                        int16_t sv = (int16_t)dsp->data[0x2c72 + i];
                                        if (!sv) continue;
                                        tot++; int b = dref[j]; if (pol) b = !b;
                                        if ((sv < 0) == b) ok++;
                                    }
                                    if (tot >= 50 && ok * 100 / tot > bs) { bs = ok * 100 / tot; bo = o; bp = pol; }
                                }
                            }
                            printf("    [tranche] sortie DSP vs burst complet : max %d%% a l'offset %d (polarite %d)\n",
                                   bs, bo, bp); }
                      /* [2026-09-18] pic et concordance sur la MEME ligne, pour
                       * pouvoir les correler trame par trame : si les trames dont le
                       * pic vaut 21 (debut reel du burst) concordent mieux, il reste un
                       * probleme de PRECISION du pic et l'estimation de canal est posee
                       * a cote, ce qui suffit a ruiner un egaliseur MLSE sur du GMSK
                       * dont l'ISI porte sur trois symboles. Si la concordance reste
                       * plate quel que soit le pic, le correlateur est hors de cause. */
                      /* empreinte des 78 bits souples, pour diff entre deux runs */
                      { static int nemq; static long cible = -2;
                        if (cible == -2) { const char *e = getenv("REJEU_EMPREINTE_FN");
                                           cible = e ? atol(e) : -1; }
                        /* [2026-09-18] LA TRAME DOIT ETRE VERROUILLEE. Prendre la
                         * premiere occurrence venue ne compare pas la meme chose d'un
                         * run a l'autre : la trame sur laquelle la tentative SB atterrit
                         * depend de l'ordonnancement FB, et sous SCH_PARTOUT une trame
                         * differente signifie un T1/T2/T3 different, donc un mot de code
                         * entierement different — d'ou 190/380 echantillons differents et
                         * 38 signes sur 78 qui basculent, c'est-a-dire le hasard.
                         * REJEU_EMPREINTE_FN=<n> n'imprime que la trame n. */
                        if (nemq < 1 && (cible < 0 || (long)g_livre_fn == cible)) {
                            /* [2026-09-18] Verifier le MODULATEUR avant d'accuser le
                             * demodulateur. Inverser le bit code 0 touche le bit 3 du
                             * burst, donc alpha_3 et alpha_4 changent tous deux de signe
                             * et leur SOMME est conservee : la phase se realigne au bout
                             * de deux symboles, et seuls ~6 echantillons complexes autour
                             * de l'indice 24 doivent differer sur 380. Si les 380
                             * different, le fautif est le modulateur. */
                            printf("    [empreinte-iq] fn=%u :", g_livre_fn);
                            for (int k = 0; k < 380; k++) printf(" %04x", dsp->data[0x0cce + k]);
                            printf("\n"); nemq++;
                        } }
                      { static int nemp; static long cible2 = -2;
                        if (cible2 == -2) { const char *e = getenv("REJEU_EMPREINTE_FN");
                                            cible2 = e ? atol(e) : -1; }
                        if (nemp < 3 && (cible2 < 0 || (long)g_livre_fn == cible2)) {
                            printf("    [empreinte] fn=%u (trame courante %u%s) pic=%u :", g_livre_fn, fn_cur,
                                   g_livre_fn == fn_cur ? "" : " DESALIGNE", dsp->data[0x2f06]);
                            for (int k = 0; k < 78; k++) printf(" %04x", dsp->data[0x2c72 + k]);
                            printf("\n"); nemp++;
                        } }
                      /* [2026-09-18] LE VRAI TAMPON. Le desassemblage du redacteur montre
                       * deux pointeurs, AR1 des 0x2c56 et AR5 des 0x2c88, 50 iterations
                       * chacun avec un pas de +1 : le tampon fait donc 100 mots a 0x2c56,
                       * et la sonde lisait les 78 derniers a 0x2c72. On cherche, pour
                       * CHAQUE MOITIE separement, l'emplacement et le sens qui concordent
                       * avec le mot de code emis. Chaque moitie n'a qu'une centaine de
                       * candidats, donc la recherche est concluante, contrairement a la
                       * famille jointe de 136 combinaisons. */
                      /* [2026-09-18] Quatre decodages independants du bloc 0x84a0-0x84d8
                       * concluent que ce n'est PAS le redacteur des bits souples mais le
                       * CORRELATEUR DE MIDAMBULE : 50 retards, 64 taps d'une reference fixe
                       * en 0x2cea (64 = longueur du midambule), sorties Re en 0x2c56 et Im en
                       * 0x2c88, puis |corr|^2 en 0x2be4 pour l'argmax. L'index y est le
                       * RETARD, pas un rang de bit code : il n'y a donc rien a y chercher.
                       * Ils designent 0x2a00 comme le vrai tampon (boucle 0x848d-0x849f,
                       * BRC=141 donc 142 tours, `sth B,*AR6+` en 0x8497). 142 est proche des
                       * 148 bits du burst : on teste donc l'hypothese « 0x2a00 indexe par
                       * POSITION DANS LE BURST », ou le bit code b est en 3+b pour b<39 et en
                       * 106+(b-39) au-dela. */
                      /* [2026-09-18] ARRETER DE DEVINER : CHERCHER. A l'entree du decodeur,
                       * on balaie TOUTE la memoire donnee a la recherche d'une zone dont les
                       * SIGNES concordent avec le mot de code emis, sous deux dispositions :
                       * 78 mots contigus indexes par rang de bit code, et indexation par
                       * position dans le burst (3+b puis 106+b-39). On imprime les meilleures
                       * bases. Si aucune ne depasse nettement le hasard, les bits souples ne
                       * sont pas en memoire donnee sous une de ces deux formes. */
                      /* [2026-09-18] DISCRIMINANT ENTREE / SORTIE pour le candidat 0x2ad5.
                       * La concordance ne separe pas les deux : l'entree derotatee et la
                       * sortie du demodulateur sont toutes deux indexees par position de
                       * burst et repondraient pareil a une inversion de bit. Ce qui les
                       * separe est la dependance au CANAL : une sortie de demodulateur
                       * depend de l'estimation de canal, donc du pic ; une representation
                       * de l'entree n'en depend pas. On perturbe donc le MIDAMBULE seul
                       * (ce qui deplace le pic sans toucher aux donnees) et on regarde si
                       * 0x2ad5 bouge AUX POSITIONS DE DONNEES. Invariant => entree.
                       * Ca bouge => sortie, et le tampon est trouve. */
                      /* [2026-09-18] LE VRAI TAMPON, trouve en ROM : 0x2a00.
                       * Deux etages successifs, tous deux a 0x2a00 :
                       *   - sortie de l'egaliseur, 142 mots, index = position de burst - 3
                       *     (`sth B,*AR6+`, AR6 init 0x2a00, BRC=0x8d donc 142 tours) ;
                       *   - codeword COMPACTE, 78 mots contigus, index = rang de bit code,
                       *     produit par 0x7e65-0x7e7a : 39 copies, puis `mar *+AR2(0x0040)`
                       *     en 0x7e76 qui SAUTE LES 64 BITS DE MIDAMBULE, puis 39 copies.
                       *     Le `+64` existe donc bien, il est litteral en ROM.
                       * Le decodeur le confirme : 0x984a `stm #0x2a00,AR1`, BRC=0x26, et le
                       * corps de 0x9a78 avance AR1 de 2 par pas de treillis, 39 pas = 78 mots.
                       *
                       * ET SURTOUT : on ne jette PLUS les zeros. Mon balayage precedent
                       * faisait `if (!v) continue;` puis exigeait n >= 60 ; or ces bits
                       * souples valent 0x0000 et 0xffff, donc la moitie des echantillons
                       * etait jetee, n tombait sous 60, et 0x2a00 etait ELIMINE avant
                       * d'etre note. La recherche exhaustive etait aveugle au bon tampon. */
                      { static int n2a0;
                        if (n2a0 < 4) {
                            int meil_c = -1, pol_c = 0, meil_b = -1, pol_b = 0;
                            for (int pol = 0; pol < 2; pol++) {
                                int ok = 0;
                                for (int b = 0; b < 78; b++) {
                                    int16_t v = (int16_t)dsp->data[0x2a00 + b];
                                    int bit = att[b] != 0; if (pol) bit = !bit;
                                    if ((v < 0) == bit) ok++;
                                }
                                if (ok * 100 / 78 > meil_c) { meil_c = ok * 100 / 78; pol_c = pol; }
                                ok = 0;
                                for (int b = 0; b < 78; b++) {
                                    int j = (b < 39) ? b : (64 + b);
                                    int16_t v = (int16_t)dsp->data[0x2a00 + j];
                                    int bit = att[b] != 0; if (pol) bit = !bit;
                                    if ((v < 0) == bit) ok++;
                                }
                                if (ok * 100 / 78 > meil_b) { meil_b = ok * 100 / 78; pol_b = pol; }
                            }
                            int nz = 0, nff = 0, naut = 0;
                            for (int k = 0; k < 142; k++) {
                                uint16_t v = dsp->data[0x2a00 + k];
                                if (v == 0) nz++; else if (v == 0xffff) nff++; else naut++;
                            }
                            printf("    [2a00] COMPACTE (0x2a00+b) : %d%% (polarite %d) | "
                                   "BRUT (saut de 64) : %d%% (polarite %d)\n",
                                   meil_c, pol_c, meil_b, pol_b);
                            printf("    [2a00] contenu sur 142 mots : %d nuls, %d a 0xffff, %d autres |",
                                   nz, nff, naut);
                            for (int k = 0; k < 10; k++) printf(" %04x", dsp->data[0x2a00 + k]);
                            printf("\n");
                            n2a0++;
                        } }
                      { static int n2d;
                        if (n2d < 1) {
                            printf("    [2ad5] pic=%u valeurs aux positions de burst 0..147 :\n      ",
                                   dsp->data[0x2f06]);
                            for (int k = 0; k < 148; k++) {
                                printf(" %04x", dsp->data[0x2ad5 + k]);
                                if (k % 12 == 11) printf("\n      ");
                            }
                            printf("\n");
                            n2d++;
                        } }
                      { static int nch;
                        if (nch < 1) {
                            struct { int sc, base, disp, pol; } top[6];
                            for (int t = 0; t < 6; t++) { top[t].sc = -1; top[t].base = 0; }
                            for (int base = 0; base < 0x3f00; base++)
                              for (int disp = 0; disp < 2; disp++)
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok = 0, n = 0;
                                    for (int b = 0; b < 78; b++) {
                                        int j = disp ? ((b < 39) ? (3 + b) : (106 + b - 39)) : b;
                                        int16_t v = (int16_t)dsp->data[(base + j) & 0x3fff];
                                        if (!v) continue;
                                        n++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok++;
                                    }
                                    if (n < 60) continue;
                                    int sc = ok * 100 / n;
                                    for (int t = 0; t < 6; t++)
                                        if (sc > top[t].sc) {
                                            for (int u = 5; u > t; u--) top[u] = top[u-1];
                                            top[t].sc = sc; top[t].base = base; top[t].disp = disp; top[t].pol = pol;
                                            break;
                                        }
                                }
                            printf("    [chasse] meilleure zone : base 0x%04x %s pol %d -> %d%%"
                                   "   (2e: 0x%04x %d%%, 3e: 0x%04x %d%%)\n",
                                   top[0].base, top[0].disp ? "burst" : "contigu", top[0].pol, top[0].sc,
                                   top[1].base, top[1].sc, top[2].base, top[2].sc);
                            nch++;
                        } }
                      { static int n2a;
                        if (n2a < 3) {
                            int meil = -1, mo = 0, mp = 0;
                            for (int o = -8; o <= 8; o++)
                              for (int pol = 0; pol < 2; pol++) {
                                  int ok = 0, n = 0;
                                  for (int b = 0; b < 78; b++) {
                                      int bp = (b < 39) ? (3 + b) : (106 + b - 39);
                                      int j = bp + o; if (j < 0 || j >= 148) continue;
                                      int16_t v = (int16_t)dsp->data[0x2a00 + j]; if (!v) continue;
                                      n++; int bit = att[b] != 0; if (pol) bit = !bit;
                                      if ((v < 0) == bit) ok++;
                                  }
                                  if (n >= 50) { int pc2 = ok * 100 / n;
                                      if (pc2 > meil) { meil = pc2; mo = o; mp = pol; } }
                              }
                            int nz = 0; for (int k = 0; k < 148; k++) if (dsp->data[0x2a00 + k]) nz++;
                            printf("    [2a00] indexe par position de burst : concordance max %d%%"
                                   " (decalage %d, polarite %d) | %d/148 mots non nuls\n",
                                   meil, mo, mp, nz);
                            printf("    [2a00] valeurs :");
                            for (int k = 0; k < 14; k++) printf(" %04x", dsp->data[0x2a00 + k]);
                            printf("\n");
                            n2a++;
                        } }
                      { static int nv;
                        if (nv < 3) {
                            int m1b = -1, m1o = 0, m1s = 0, m2b = -1, m2o = 0, m2s = 0;
                            for (int o = 0; o < 100; o++)
                              for (int sens = -1; sens <= 1; sens += 2)
                                for (int pol = 0; pol < 2; pol++) {
                                    int ok1 = 0, n1 = 0, ok2 = 0, n2 = 0;
                                    for (int b = 0; b < 39; b++) {
                                        int j = o + sens * b; if (j < 0 || j >= 100) continue;
                                        int16_t v = (int16_t)dsp->data[0x2c56 + j]; if (!v) continue;
                                        n1++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok1++;
                                    }
                                    for (int b = 39; b < 78; b++) {
                                        int j = o + sens * (b - 39); if (j < 0 || j >= 100) continue;
                                        int16_t v = (int16_t)dsp->data[0x2c56 + j]; if (!v) continue;
                                        n2++; int bit = att[b] != 0; if (pol) bit = !bit;
                                        if ((v < 0) == bit) ok2++;
                                    }
                                    if (n1 >= 30) { int pc1 = ok1 * 100 / n1;
                                        if (pc1 > m1b) { m1b = pc1; m1o = o; m1s = sens * (pol ? -2 : 1); } }
                                    if (n2 >= 30) { int pc2 = ok2 * 100 / n2;
                                        if (pc2 > m2b) { m2b = pc2; m2o = o; m2s = sens * (pol ? -2 : 1); } }
                                }
                            printf("    [2c56] moitie 1 : meilleur %d%% a l'offset %d (code %d) | "
                                   "moitie 2 : meilleur %d%% a l'offset %d (code %d)\n",
                                   m1b, m1o, m1s, m2b, m2o, m2s);
                            int nz = 0; for (int k = 0; k < 100; k++) if (dsp->data[0x2c56 + k]) nz++;
                            printf("    [2c56] %d/100 mots non nuls, carte des nuls (. = nul, X = non nul) :\n      ", nz);
                            for (int k = 0; k < 100; k++) {
                                printf("%c", dsp->data[0x2c56 + k] ? 'X' : '.');
                                if (k % 50 == 49) printf("\n      ");
                            }
                            printf("\n");
                            for (int lig = 0; lig < 10; lig++) {
                                printf("      +%2d :", lig * 10);
                                for (int k = lig * 10; k < lig * 10 + 10; k++) printf(" %04x", dsp->data[0x2c56 + k]);
                                printf("\n");
                            }
                            nv++;
                        } }
                      printf("    [pic-conc] pic=%u concordance=%d%%\n", dsp->data[0x2f06], best);
                      printf("    [bits] fn=%u  0x2c72 non-nuls=%d/78 : %04x %04x %04x %04x %04x %04x\n"
                             "           concordance max = %d%% (decalage %d, polarite %d, ordre %s)\n",
                             g_livre_fn, nznb, dsp->data[0x2c72], dsp->data[0x2c73], dsp->data[0x2c74],
                             dsp->data[0x2c75], dsp->data[0x2c76], dsp->data[0x2c77],
                             best, bestd, bestv & 1,
                             (bestv >> 1) == 1 ? "moities echangees"
                             : (bestv >> 1) == 2 ? "pairs d'abord"
                             : (bestv >> 1) == 3 ? "impairs d'abord" : "identite");
                      nb++;
                  } }
                /* [2026-09-18] Piste : si les magnitudes au-dela d'un certain rang
                 * restent nulles, l'argmax ne peut tomber que dans la partie remplie,
                 * et tous les pics observes sont sous 16. On compte donc combien de
                 * magnitudes sont REELLEMENT ecrites dans la zone de correlation,
                 * et on compare a la longueur de fenetre attendue. */
                /* [2026-09-18] Le mot `sb` rendu est IDENTIQUE pour BSIC 7, 14 et 49,
                 * alors que les bits souples en 0x2c72, eux, changent bien avec la
                 * charge utile. Donc le mot que l'ARM relit ne vient PAS du decodeur.
                 * Question : le DSP ecrit-il JAMAIS les mots a_sch de la page R ?
                 * API en 0x0800, R_PAGE0=0x28 et R_PAGE1=0x3C, a_sch au mot +15. */
                { static uint16_t vu0[10], vu1[10]; static int arme3, nw;
                  const uint16_t A0 = 0x800 + 0x28 + 15, A1 = 0x800 + 0x3C + 15;
                  if (!arme3) {
                      for (int k = 0; k < 10; k++) { vu0[k] = dsp->data[A0 + k]; vu1[k] = dsp->data[A1 + k]; }
                      arme3 = 1;
                  } else if (nw < 12) {
                      for (int k = 0; k < 5; k++) {
                          if (dsp->data[A0 + k] != vu0[k]) {
                              printf("    [a_sch] page0 mot %d : %04x -> %04x  ecrit par pc=%04x\n",
                                     k, vu0[k], dsp->data[A0 + k], pc);
                              vu0[k] = dsp->data[A0 + k]; nw++;
                          }
                          if (dsp->data[A1 + k] != vu1[k]) {
                              printf("    [a_sch] page1 mot %d : %04x -> %04x  ecrit par pc=%04x\n",
                                     k, vu1[k], dsp->data[A1 + k], pc);
                              vu1[k] = dsp->data[A1 + k]; nw++;
                          }
                      }
                  } }
                /* [2026-09-18] Les magnitudes sont presentes et l'argmax se trompe
                 * quand meme. Question suivante : QUELLE zone balaie-t-il ? On suit
                 * l'etendue des registres d'adresse pendant le choix de pic
                 * (0x84c8..0x84ef) pour la comparer aux zones de magnitudes. */
                { static uint16_t armin[8], armax[8]; static int arme2;
                  if (pc >= 0x84c8 && pc <= 0x84ef) {
                      if (!arme2) { for (int k = 0; k < 8; k++) { armin[k] = 0xffff; armax[k] = 0; } arme2 = 1; }
                      for (int k = 0; k < 8; k++) {
                          uint16_t v = dsp->ar[k];
                          if (v < armin[k]) armin[k] = v;
                          if (v > armax[k]) armax[k] = v;
                      }
                  }
                { static int nq;
                  if (pc == 0x84ef && nq < 5) {
                      printf("    [balayage] registres d'adresse pendant 0x84c8-0x84ef :");
                      for (int k = 0; k < 8; k++)
                          if (armax[k] >= armin[k] && armax[k] > armin[k])
                              printf(" AR%d=[0x%04x..0x%04x]", k, armin[k], armax[k]);
                      printf("\n");
                      arme2 = 0;
                      printf("    [magn@84ef] plages non nulles entre 0x2b00 et 0x2d00 :\n");
                      int deb = -1, fin = -1;
                      for (int a = 0x2b00; a <= 0x2d00; a++) {
                          int nz = (a <= 0x2cff) && dsp->data[a] != 0;
                          if (nz && deb < 0) deb = a;
                          if (nz) fin = a;
                          if (!nz && deb >= 0) {
                              /* [2026-09-18] Cette sonde comparait en uint16_t : les
                               * « max » annonces (65502, 65104, 65318) etaient en fait
                               * -34, -432, -218 en int16, donc elle rapportait la valeur
                               * la plus proche de -1 et le rang associe etait du bruit.
                               * On compare desormais en int16_t, et on donne AUSSI le
                               * maximum en valeur absolue : si 0x2be4 porte une
                               * correlation SIGNEE, c'est |v| (ou v*v) qui doit decider,
                               * pas v. Les deux rangs cote a cote le disent. */
                              int n = fin - deb + 1;
                              int16_t vmax = -32768; int imax = -1;
                              int amax = -1, iamax = -1;
                              for (int k = deb; k <= fin; k++) {
                                  int16_t v = (int16_t)dsp->data[k];
                                  int a = v < 0 ? -(int)v : (int)v;
                                  if (v > vmax) { vmax = v; imax = k - deb; }
                                  if (a > amax) { amax = a; iamax = k - deb; }
                              }
                              printf("        0x%04x-0x%04x : %3d mots, max signe=%d au rang %d,"
                                     " max |v|=%d au rang %d\n",
                                     deb, fin, n, vmax, imax, amax, iamax);
                              deb = -1;
                          }
                      }
                      printf("        argmax DSP 0x2f06=%u\n", dsp->data[0x2f06]);
                      nq++;
                  } } }
                /* [2026-09-17] Ou le correlateur ecrit-il VRAIMENT ? On photographie
                 * la RAM donnee a l'entree de 0x84a1 et on liste ce qui a change a la
                 * sortie, plutot que de deviner l'adresse du tampon d'energies. */
                { static uint16_t *snap; static int armec, ns;
                  if (pc == 0x84a1 && !armec && ns < 3) {
                      if (!snap) snap = malloc(0x4000 * sizeof(uint16_t));
                      memcpy(snap, dsp->data, 0x4000 * sizeof(uint16_t));
                      armec = 1;
                  } else if (pc == 0x7e94 && armec) {
                      int deb = -1, fin = -1, nch = 0;
                      printf("    [ecrit] plages modifiees par le correlateur SB :\n");
                      for (int k = 0; k < 0x4000; k++) {
                          int d = (dsp->data[k] != snap[k]);
                          if (d && deb < 0) deb = k;
                          if (d) { fin = k; nch++; }
                          if (!d && deb >= 0 && k > fin + 8) {
                              printf("        0x%04x-0x%04x (%d mots)  ex: %04x->%04x\n",
                                     deb, fin, fin - deb + 1, snap[deb], dsp->data[deb]);
                              deb = -1;
                          }
                      }
                      if (deb >= 0) printf("        0x%04x-0x%04x\n", deb, fin);
                      printf("        total %d mots changes\n", nch);
                      armec = 0; ns++;
                  } }
                /* [2026-09-17] DEFAUT B : avec un VRAI burst SCH en entree, les bits
                 * souples en 0x2a00 sortent nuls. On regarde le maillon du milieu :
                 * l'energie de correlation du midambule (0x2be4) et l'argmax (0x2f06). */
                { static int nc;
                  if (pc == 0x84ef && nc < 8) {
                      /* meme correctif de type que plus haut : int16_t, plus |v| */
                      int nzc = 0, emax = -1, iemax = -1, samax = -32768, isamax = -1;
                      for (int k = 0; k < 64; k++) {
                          int16_t v = (int16_t)dsp->data[0x2be4 + k];
                          int a = v < 0 ? -(int)v : (int)v;
                          if (v) nzc++;
                          if (a > emax) { emax = a; iemax = k; }
                          if (v > samax) { samax = v; isamax = k; }
                      }
                      int nzin = 0; for (int k = 0; k < 380; k++) if (dsp->data[0x0cce + k]) nzin++;
                      /* [2026-09-18] Cette ligne avait 15 conversions pour 13 arguments :
                       * le fragment « max=%u a k=%d » etait un reste de l'ancienne
                       * version, laisse en place lors du passage a int16_t. Tout ce qui
                       * suivait glissait d'un cran, si bien que le champ etiquete
                       * « max signe » affichait en realite 0x2f06, plus un mot de pile
                       * en fin de ligne. Fragment redondant supprime. */
                      printf("    [corr] entree 0x0cce non-nuls=%d/380 | 0x2be4 non-nuls=%d/64"
                             " | max |v|=%d au rang %d, max signe=%d au rang %d"
                             " | argmax 0x2f06=%u | 0x2be4: %04x %04x %04x %04x %04x %04x\n",
                             nzin, nzc, emax, iemax, samax, isamax, dsp->data[0x2f06],
                             dsp->data[0x2be4], dsp->data[0x2be5], dsp->data[0x2be6],
                             dsp->data[0x2be7], dsp->data[0x2be8], dsp->data[0x2be9]);
                      nc++;
                  } }
                /* entree du demod SB : le DMA doit avoir depose 190 complexes en 0x0cce */
                { static int ne;
                  if (pc == 0x7c31 && ne < 6) {
                      int nz = 0; for (int k = 0; k < 380; k++) if (dsp->data[0x0cce + k]) nz++;
                      int z0 = 0; while (z0 < 380 && !dsp->data[0x0cce + z0]) z0++;
                      printf("    [in-sb] fn=%u p51=%u  DERNIER BURST LIVRE: type=%c fn=%u n=%d\n"
                             "            0x0cce non-nuls=%d/380 zeros_tete=%d : %04x %04x %04x %04x %04x %04x\n",
                             fn_cur, fn_cur % 51, g_livre_type ? g_livre_type : '?', g_livre_fn, g_livre_n,
                             nz, z0, dsp->data[0x0cce], dsp->data[0x0ccf], dsp->data[0x0cd0],
                             dsp->data[0x0cd1], dsp->data[0x0cd2], dsp->data[0x0cd3]);
                      ne++;
                  } }
                switch (pc) {                       /* chaine SB (RE §9.4) */
                case 0x7c31: hit_7c31++; break;     /* demodulateur SB */
                case 0x9841: hit_9841++; break;     /* decodeur SCH */
                case 0x84a1: hit_84a1++; break;     /* correlateur midambule */
                case 0x770a: hit_770a++; break;     /* correlateur FB */
                case 0xb219: hit_b219++; break;     /* job SB (arme le DMA) */
                case 0x7a16: hit_7a16++; break;     /* fcall vers le demod SB */
                default: break; }
                histo_pc[pc >> 10]++; insn_total++;
                { static uint16_t tprev, tpc, ppc; static int tn;
                  if (dsp->t != tprev) { tpc = ppc; tprev = dsp->t; }
                  ppc = pc;
                  if (pc == 0x7947 && tn < 10 && drapeau_env("REJEU_PROBE_T")) {
                      printf("    [T] au mpya (0x7947) : T=%04x (%u), dernier ecrit par pc=%04x\n",
                             dsp->t, dsp->t, tpc); tn++; } }
                if (!drapeau_env("REJEU_PROBE_T") && pc >= 0x7940 && pc <= 0x795c && nlog < 60) {
                    printf("    [toa] pc=%04x A=%010llx B=%010llx T=%04x 3fb4=%04x AR4=%04x\n",
                           pc, (unsigned long long)(dsp->a & 0xffffffffffULL),
                           (unsigned long long)(dsp->b & 0xffffffffffULL),
                           dsp->t, dsp->data[0x3fb4], dsp->ar[4]);
                    nlog++;
                }
                int ex = c54x_run(dsp, 1);
                if (ex <= 0) break;
                done += ex;
            }
        }
    }
    {   /* [2026-09-18] Desassemblage brut de la zone du redacteur des bits souples.
         * On imprime les mots programme tels que le DSP les lit (alias OVLY compris),
         * pour pouvoir lire le CALCUL D'ADRESSE des deux passes. */
        printf("  mots programme 0x8378-0x8396 (l'ecrivain 0x838d qui met 142 zeros) :\n");
        for (uint16_t a = 0x8378; a <= 0x8396; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x8378) % 8 == 7) ? "\n" : "");
        printf("\n  mots programme 0x81c8-0x81e0 (les ecrivains des 142 valeurs) :\n");
        for (uint16_t a = 0x81c8; a <= 0x81e0; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x81c8) % 8 == 7) ? "\n" : "");
        printf("\n");
        printf("  mots programme 0x84a0-0x84d8 (correlateur de midambule) :\n");
        for (uint16_t a = 0x84a0; a <= 0x84d8; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x84a0) % 8 == 7) ? "\n" : "");
        printf("\n  mots programme 0x7cb0-0x7cc0 (site qui remet a zero) :\n");
        for (uint16_t a = 0x7cb0; a <= 0x7cc0; a++)
            printf("    %04x: %04x%s", a, dsp->prog[a], ((a - 0x7cb0) % 8 == 7) ? "\n" : "");
        printf("\n");
    }
    printf("\n─── bilan rejeu (deterministe) ───\n");
    {
        printf("  instructions DSP executees : %lu\n", insn_total);
        printf("  histogramme des PC (plages de 1024 mots, non vides) :\n");
        for (int b = 0; b < 64; b++)
            if (histo_pc[b])
                printf("      0x%04x-0x%04x : %10lu  (%4.1f%%)\n",
                       b << 10, ((b + 1) << 10) - 1, histo_pc[b],
                       insn_total ? 100.0 * histo_pc[b] / insn_total : 0.0);
    }
    if (vu_sb && vu_hors) {
        int n = 0;
        printf("  opcodes executes UNIQUEMENT par le demodulateur SB (suspects d'ISA,\n"
               "  jamais valides par le chemin FB qui, lui, fonctionne) :\n    ");
        int fam[256] = {0}, famt[256] = {0};
        for (int o = 0; o < 65536; o++) {
            if (vu_sb[o]) famt[o >> 8]++;
            if (vu_sb[o] && !vu_hors[o]) { fam[o >> 8]++; n++; }
        }
        printf("\n");
        for (int f = 0; f < 256; f++)
            if (fam[f]) printf("    famille %02x.. : %3d opcodes exclusifs SB / %3d executes en SB\n",
                               f, fam[f], famt[f]);
        printf("    (%d opcodes distincts exclusifs au demodulateur SB)\n", n);
    }
    printf("  trames jouees      : %u\n", fn_cur + 1);
    printf("  chaine SB (pas-a-pas) : job b219=%lu  fcall 7a16=%lu  demod 7c31=%lu  corr 84a1=%lu  decodeur 9841=%lu  | corr FB 770a=%lu\n",
           hit_b219, hit_7a16, hit_7c31, hit_84a1, hit_9841, hit_770a);
    printf("  FB acceptes (ARM)  : %d\n", n_fb_ok);
    printf("  SB tentees / CRC KO: %d / %d\n", n_sb_try, n_sb_crcfail);
    printf("  CMPS (0x8E/0x8F) : %ld fois dans le demod SB, %ld hors | famille 0xE0-0xE3 en SB : %ld | ST TRN : %ld\n",
           n_cmps_sb, n_cmps_hors, n_e0_sb, n_sttrn_sb);
    {
        printf("  XC (0xFD/0xFF) par region de PC :");
        for (int b = 0; b < 64; b++) if (n_xc_reg[b]) printf(" 0x%04x=%ld", b << 10, n_xc_reg[b]);
        printf("\n");
        printf("  CMPS (0x8E/0x8F) par region de PC :");
        for (int b = 0; b < 64; b++)
            if (n_cmps_reg[b]) printf(" 0x%04x=%ld", b << 10, n_cmps_reg[b]);
        printf("\n");
    }
    printf("      detail : FIRS(E0)=%ld  LMS(E1)=%ld  SQDST(E2)=%ld  ABDST(E3)=%ld"
           "   <- inertes si CALYPSO_ISA_E0_FAM=1\n",
           n_e0x_sb[0], n_e0x_sb[1], n_e0x_sb[2], n_e0x_sb[3]);
    printf("  erreurs DSP signalees: %d, dont erreur 8 (anneau DMA): %d\n", n_err_dsp, n_err8);
    printf("  CRC OK: %d, dont VRAIES (BSIC injecte + T3 valide + FN = trame): %d%s\n",
           n_crc_ok, n_sb_vraies,
           cellule_sch_partout ? "   [FN non qualifiable : SCH_PARTOUT actif]" : "");
    if (verdict == 1) printf("  VERDICT : SB DECODEE  BSIC=%d FN=%u (sb=0x%08x)\n", sb_bsic, sb_fn, sb_word);
    else if (verdict == -1) printf("  VERDICT : ABANDON (le firmware a renonce)\n");
    else printf("  VERDICT : AUCUNE SB (ni decodee ni abandon) en %ld trames\n", trames);
    return verdict == 1 ? 0 : 1;
}
