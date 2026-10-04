/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * dsp_tester.c - testeur de fonctions du DSP Calypso (mask-ROM TI 3606).
 *
 * Sans QEMU ni ARM : le testeur JOUE l'ARM osmocom-bb (tools/dsp_banc_commun.c :
 * dsp_power_on, l1_sync, ordonnanceur TDMA, items de prim_*.c recopies) et la
 * BTS (bursts synthetiques codes par libosmocoding, modules en GMSK comme le
 * BSP du banc). Pour chaque fonction du DSP il pose les codes de tache dans
 * l'API RAM exactement comme le firmware, fait tourner les trames, lit les
 * sorties (pages R, NDB, bursts montants emis) et les compare a la reference
 * libosmocoding. Une ligne PASS / FAIL / NON-IMPL par fonction.
 *
 * Un test qui revele un defaut du DSP (ou du coeur emule) reste FAIL, avec le
 * diagnostic ; aucun contournement n'est applique pour le faire passer.
 *
 *   ./dsp_tester --list            les tests
 *   ./dsp_tester --all             tout
 *   ./dsp_tester fb sb nb ...      les tests nommes
 *   options : -v (detail par trame), --rom-dir DIR, --insns N (budget/trame),
 *             --bsic N
 * Code de retour : nombre de FAIL.
 *
 * Chaque test tourne dans un processus fils, a partir du DSP deja demarre
 * (fork apres dsp_power_on) : etat de depart identique, et un plantage du
 * coeur n'emporte que son test.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <math.h>
#include "dsp_banc_commun.h"
#include "calypso_rhea_dma.h"
#include "verbosite.h"
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>
#include <osmocom/gsm/a5.h>
#include <osmocom/codec/codec.h>

/* ---- verdicts ---------------------------------------------------------------- */
enum { V_PASS = 0, V_FAIL = 1, V_NONIMPL = 2 };
static const char *NOMV[] = { "PASS", "FAIL", "NON-IMPL" };
static char g_detail[8192];
static size_t g_dl;
static int g_v;                      /* -v */
static int g_trace;                  /* --trace N : niveau des traces du coeur (src/verbosite.c) */
static void det(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void det(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(g_detail + g_dl, sizeof g_detail - g_dl, fmt, ap);
    va_end(ap);
    if (n > 0) { g_dl += (size_t)n; if (g_dl >= sizeof g_detail) g_dl = sizeof g_detail - 1; }
}
static void vlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void vlog(const char *fmt, ...)
{
    if (!g_v) return;
    va_list ap; va_start(ap, fmt);
    printf("      ");
    vprintf(fmt, ap);
    va_end(ap);
}

static struct banc_boot g_boot;
static int g_boot_rc;

/* ---- outils --------------------------------------------------------------- */
static uint32_t g_alea = 0x1234567u;
static uint8_t alea8(void) { g_alea = g_alea * 1103515245u + 12345u; return (uint8_t)(g_alea >> 16); }
/* Bloc L2 de 23 octets determine par une graine : contenu unique par bloc,
 * de sorte qu'un a_cd perime ne puisse pas passer pour un bon decodage. */
static void l2_bloc(uint32_t graine, uint8_t l2[23])
{
    uint32_t s = graine * 2654435761u + 0x9e3779b9u;
    for (int i = 0; i < 23; i++) { s = s * 1103515245u + 12345u; l2[i] = (uint8_t)(s >> 16); }
    l2[0] = (uint8_t)(graine >> 8); l2[1] = (uint8_t)graine;     /* lisible dans les traces */
}
static void hex23(char *o, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) sprintf(o + 3 * i, "%02x ", b[i]);
    if (n) o[3 * n - 1] = 0; else o[0] = 0;
}
static uint32_t gsmtime2fn(unsigned t1, unsigned t2, unsigned t3)
{
    return 51u * 26u * t1 + 51u * (((int)t3 - (int)t2 + 26 * 3) % 26) + t3;
}
#define BITFREQ_DIV_PI    86208
#define ANG2FREQ_SCALING  (2 << 15)
#define ANGLE_TO_FREQ(a)  ((int)(int16_t)(a) * BITFREQ_DIV_PI / ANG2FREQ_SCALING)
#define BITS_PER_TDMA     1250
static unsigned popdiff(const uint8_t *a, const uint8_t *b, int n)
{
    unsigned d = 0; for (int i = 0; i < n; i++) d += (a[i] & 1) != (b[i] & 1); return d;
}

/* source radio courante + contenu des blocs de la cellule */
static uint8_t g_l2_cell[64][23];
static const uint8_t *cellule_nb_alea(uint32_t fn0, int p51)
{
    (void)p51;
    uint8_t *l2 = g_l2_cell[(fn0 / 51 * 10 + (unsigned)p51) % 64];
    l2_bloc(fn0, l2);
    return l2;
}

/* ===================================================================== */
/*  boot / idle / checksum                                                */
/* ===================================================================== */
static int t_boot(void)
{
    det("chargeur : BL_STATUS=%u apres %ld insn ; demarrage 0x7000 -> IDLE en %ld insn ; version API %04x %04x",
        g_boot.bl_status, g_boot.insn_chargeur, g_boot.insn_demarrage, g_boot.version1, g_boot.version2);
    if (g_boot_rc || g_boot.bl_status != 1 || !g_boot.idle) { det("\nle DSP n'a pas atteint l'IDLE"); return V_FAIL; }
    if (g_boot.version1 != 0x3606) { det("\nversion attendue 3606 (calypso_api.h API_VERSION_VALUE)"); return V_FAIL; }
    return V_PASS;
}

static int scenario_vide(uint32_t fn) { (void)fn; return TDMA_IFLG_DSP; }
static int t_idle(void)
{
    /* L'ARM clot un scenario vide a chaque trame (page W remise a zero par
     * l1_sync, aucune tache) : la ROM recoit l'IT trame, lit une page vide et
     * doit revenir a l'IDLE, sans erreur. */
    banc_source = banc_cellule;
    banc_crochet_l1s = scenario_vide;
    uint32_t i0 = banc_dsp->insn_count;
    banc_courir(100);
    long moy = (long)(banc_dsp->insn_count - i0) / 100;
    det("100 trames, %lu IT trame ; %ld insn/trame en moyenne, max %ld ; trames finies hors IDLE : %lu ; d_error_status : %lu",
        banc_stats.irq_trame, moy, banc_stats.insn_max, banc_stats.pas_idle_fin, banc_stats.erreurs_dsp);
    if (banc_stats.irq_trame < 99) { det("\nIT trame non prise (IMR=%04x)", banc_dsp->imr); return V_FAIL; }
    if (banc_stats.pas_idle_fin || banc_stats.erreurs_dsp) return V_FAIL;
    if (moy == 0) { det("\nle DSP n'execute rien sur l'IT trame"); return V_FAIL; }
    return V_PASS;
}

static int g_chk_vu;
static int chk_cmd(uint8_t p1, uint8_t p2, uint16_t p3)
{
    (void)p1; (void)p2; (void)p3;
    dsp_api.db_w->d_task_md = CHECKSUM_DSP_TASK;
    dsp_api.ndb->d_fb_mode = 1;
    return 0;
}
static const struct tdma_sched_item chk_set[] = { SCHED_ITEM_DT(chk_cmd, 0, 0, 0), SCHED_END_FRAME(), SCHED_END_SET() };
static int t_checksum(void)
{
    /* calypso/dsp.c dsp_checksum_task() : d_task_md = CHECKSUM_DSP_TASK (33),
     * puis a_pm[0] = version du code, a_pm[1] = somme, API 0x08DB = patch. */
    T_DB_DSP_TO_MCU *r0 = (T_DB_DSP_TO_MCU *)&banc_api[0x28], *r1 = (T_DB_DSP_TO_MCU *)&banc_api[0x3C];
    banc_source = banc_cellule;
    tdma_schedule_set(1, chk_set, 0);
    uint16_t av[2][3]; memcpy(av[0], r0->a_pm, 6); memcpy(av[1], r1->a_pm, 6);
    banc_courir(6);
    (void)g_chk_vu;
    det("R0 a_pm=%04x %04x %04x d_task_md=%u | R1 a_pm=%04x %04x %04x d_task_md=%u | patch (API 0x08DB)=%04x",
        r0->a_pm[0], r0->a_pm[1], r0->a_pm[2], r0->d_task_md, r1->a_pm[0], r1->a_pm[1], r1->a_pm[2], r1->d_task_md,
        banc_api[0xDB]);
    int ok = 0;
    for (int p = 0; p < 2; p++) {
        T_DB_DSP_TO_MCU *r = p ? r1 : r0;
        if (r->a_pm[0] == 0x3606 || (r->a_pm[0] && memcmp(av[p], r->a_pm, 6))) ok = 1;
    }
    if (!ok) { det("\naucune page R ne porte de version/somme : tache non executee"); return V_FAIL; }
    return V_PASS;
}

/* ===================================================================== */
/*  FB / SB (layer1/prim_fbsb.c)                                          */
/* ===================================================================== */
static struct { int det, attempt, mode; uint32_t fn_resp; int16_t toa, pm, angle, snr; int freq; } g_fb;
static uint32_t g_fb_cmd_fn;
static int fbdet_cmd(uint8_t p1, uint8_t p2, uint16_t fb_mode)
{
    (void)p1; (void)p2;
    g_fb_cmd_fn = banc_courant.fn;
    dsp_api.db_w->d_task_md = FB_DSP_TASK;
    dsp_api.ndb->d_fb_mode = fb_mode;
    return 0;
}
static int fbdet_resp(uint8_t p1, uint8_t attempt, uint16_t fb_mode)
{
    (void)p1;
    if (g_fb.det) return 0;
    if (!dsp_api.ndb->d_fb_det) {
        if (attempt == 12) g_fb.det = -1;
        return 0;
    }
    l1s_reset_hw();
    /* read_fb_result() */
    g_fb.det = 1; g_fb.attempt = attempt; g_fb.mode = fb_mode; g_fb.fn_resp = banc_courant.fn;
    g_fb.toa = (int16_t)dsp_api.ndb->a_sync_demod[D_TOA];
    g_fb.pm = (int16_t)(dsp_api.ndb->a_sync_demod[D_PM] >> 3);
    g_fb.angle = (int16_t)dsp_api.ndb->a_sync_demod[D_ANGLE];
    g_fb.snr = (int16_t)dsp_api.ndb->a_sync_demod[D_SNR];
    g_fb.freq = ANGLE_TO_FREQ(g_fb.angle);
    dsp_api.ndb->d_fb_det = 0;
    dsp_api.ndb->a_sync_demod[D_TOA] = 0;
    tdma_sched_reset();
    return 0;
}
static const struct tdma_sched_item fb_sched_set[] = {
    SCHED_ITEM_DT(fbdet_cmd, 0, 0, 0), SCHED_END_FRAME(),
    SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 1), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 2), SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 3), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 4), SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 5), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 6), SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 7), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 8), SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 9), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 10), SCHED_END_FRAME(),
    SCHED_ITEM(fbdet_resp, -4, 0, 11), SCHED_END_FRAME(), SCHED_ITEM(fbdet_resp, -4, 0, 12), SCHED_END_FRAME(),
    SCHED_END_SET()
};

/* Une recherche FB en mode `mode`, commande posee a la prochaine l1_sync.
 * Rend la trame FCCH que le firmware en deduit (fn_offset de prim_fbsb.c, avec
 * l'arrondi que synchronize_tdma() fait par les qbits), -1 si rien ;
 * *residu = ecart du TOA a la position nominale (23 + n x 1250), en symboles. */
static long une_fb(int mode, uint32_t *debut, int *residu)
{
    memset(&g_fb, 0, sizeof g_fb);
    tdma_schedule_set(0, fb_sched_set, (uint16_t)mode);   /* hors l1_sync : 0 = la prochaine */
    banc_trame();                         /* la l1_sync de cette trame pose la commande */
    *debut = g_fb_cmd_fn + 1;             /* next_time de la commande : 1re trame de recherche */
    for (int k = 0; k < 14 && !g_fb.det; k++) banc_trame();
    if (g_fb.det <= 0) return -1;
    int toa = g_fb.toa - 23;
    int n = (int)lrint((double)toa / BITS_PER_TDMA);
    *residu = toa - n * BITS_PER_TDMA;
    /* prim_fbsb.c : fn_offset = current - attempt + ntdma, et current - attempt = debut */
    return (long)(g_fb.fn_resp - (uint32_t)g_fb.attempt) + n;
}
static int est_fcch(uint32_t f) { return (f % 51) % 10 == 0 && (f % 51) <= 40; }

static int t_fb_mode(int mode, int16_t dac, int hz_attendu, const char *quoi)
{
    banc_source = banc_cellule;
    banc_cfg.afc_dac = dac;
    int ok = 0, n = 0, sautees = 0;
    for (int essai = 0; essai < 6; essai++) {
        banc_courir(3 + 7 * (unsigned)essai);           /* positions differentes vis-a-vis de la FCCH */
        uint32_t debut; int residu = 0;
        long fcch = une_fb(mode, &debut, &residu);
        n++;
        uint32_t premiere = debut; while (!est_fcch(premiere)) premiere++;
        if (fcch < 0) { det("essai %d : recherche des fn=%u, pas de FB en 12 trames (FCCH en %u)\n", essai, debut, premiere); continue; }
        int fe = g_fb.freq - hz_attendu;
        int juste = fcch >= (long)debut && est_fcch((uint32_t)fcch) && abs(residu) <= 64 &&
                    abs(fe) <= (hz_attendu ? abs(hz_attendu) / 4 : 100);
        if (juste && fcch != (long)premiere) sautees++;
        det("essai %d : recherche des fn=%u -> FB%d att=%d TOA=%d (residu %+d symb.) PM=%d angle=%d (%+d Hz) SNR=%d ; FCCH deduite %ld%s%s\n",
            essai, debut, mode, g_fb.attempt, g_fb.toa, residu, g_fb.pm, g_fb.angle, g_fb.freq, g_fb.snr, fcch,
            fcch == (long)premiere ? "" : est_fcch((uint32_t)fcch) ? " (une FCCH sautee)" : " (PAS une trame FCCH)",
            juste ? "" : "  <- FAUX");
        ok += juste;
    }
    det("%s : %d/%d detections justes (FCCH reelle, TOA a +-64 symboles, frequence %+d Hz +-%d) ; %d fois la 1re FCCH sautee",
        quoi, ok, n, hz_attendu, hz_attendu ? abs(hz_attendu) / 4 : 100, sautees);
    return ok == n ? V_PASS : V_FAIL;
}
static int t_fb0(void) { return t_fb_mode(0, -700, 0, "FB mode 0 (acquisition)"); }
static int t_fb1(void) { return t_fb_mode(1, -700, 0, "FB mode 1 (poursuite)"); }
/* calypso_twl3025.c : (dac + 700) x 287 / (2^15/947) Hz, ~8.29 Hz/LSB. Le VCXO
 * tire de +60 LSB : la porteuse recue parait a -497 Hz, signe que la boucle du
 * firmware (afc_correct : dac += norm x err / pente) ramene vers -700. */
static int t_fb_afc(void)
{
    int hz = (int)lrint(-60.0 * 287.0 / (32768.0 / 947.0));
    return t_fb_mode(0, -700 + 60, hz, "FB mode 0, VCXO +60 LSB (d_afc -640)");
}

/* SB */
static struct { int fait, crc_ok; uint32_t sb; int16_t toa, pm, angle, snr; uint32_t fn_resp; } g_sb[3];
static int sbdet_cmd(uint8_t p1, uint8_t p2, uint16_t p3)
{
    (void)p1; (void)p2; (void)p3;
    dsp_api.db_w->d_task_md = SB_DSP_TASK;
    dsp_api.ndb->d_fb_mode = 0;
    return 0;
}
static int sbdet_resp(uint8_t p1, uint8_t attempt, uint16_t p3)
{
    (void)p1; (void)p3;
    if (attempt > 2) return 0;
    g_sb[attempt].fait = 1;
    g_sb[attempt].fn_resp = banc_courant.fn;
    dsp_api.r_page_used = 1;
    if (dsp_api.db_r->a_sch[0] & (1 << B_SCH_CRC)) return 0;
    g_sb[attempt].crc_ok = 1;
    g_sb[attempt].toa = (int16_t)dsp_api.db_r->a_serv_demod[D_TOA];
    g_sb[attempt].pm = (int16_t)(dsp_api.db_r->a_serv_demod[D_PM] >> 3);
    g_sb[attempt].angle = (int16_t)dsp_api.db_r->a_serv_demod[D_ANGLE];
    g_sb[attempt].snr = (int16_t)dsp_api.db_r->a_serv_demod[D_SNR];
    g_sb[attempt].sb = dsp_api.db_r->a_sch[3] | (uint32_t)dsp_api.db_r->a_sch[4] << 16;
    return 0;
}
static const struct tdma_sched_item sb_sched_set[] = {
    SCHED_ITEM_DT(sbdet_cmd, 0, 0, 1), SCHED_END_FRAME(),
    SCHED_ITEM_DT(sbdet_cmd, 0, 0, 2), SCHED_END_FRAME(),
    SCHED_END_FRAME(),
    SCHED_ITEM(sbdet_resp, -4, 0, 1), SCHED_END_FRAME(),
    SCHED_ITEM(sbdet_resp, -4, 0, 2), SCHED_END_FRAME(),
    SCHED_END_SET()
};
/* l1s_decode_sb() */
static uint8_t decode_sb(uint32_t sb, uint32_t *fn)
{
    unsigned t1 = ((sb >> 23) & 1) | ((sb >> 7) & 0x1fe) | ((sb << 9) & 0x600);
    unsigned t2 = (sb >> 18) & 0x1f;
    unsigned t3p = ((sb >> 24) & 1) | ((sb >> 15) & 6);
    *fn = gsmtime2fn(t1, t2, t3p * 10 + 1);
    return (sb >> 2) & 0x3f;
}

static int t_sb(void)
{
    /* La commande SB vise directement chaque trame SCH (p51 = 1, 11, .., 41) :
     * la 1re commande du sb_sched_set a next_time = trame SCH. */
    banc_source = banc_cellule;
    int n = 0, ok = 0, crc_faux = 0, faux = 0, rate = 0, fen = 0;
    char ligne[400];
    for (int k = 0; k < 20; k++) {
        uint32_t c = banc_fn();                 /* current_time de la prochaine l1_sync */
        uint32_t s = c + 2; while ((s % 51) % 10 != 1 || (s % 51) > 41) s++;
        unsigned long fen0 = banc_stats.depots_fenetre_sb;
        memset(g_sb, 0, sizeof g_sb);
        while (banc_fn() < s - 2) banc_trame();
        /* l1_sync(s-2) programme le set a +1 : cmd1 a l1_sync(s-1), next_time s */
        tdma_schedule_set(1, sb_sched_set, 0);
        for (int i = 0; i < 6; i++) banc_trame();
        n++;
        bool dans_fenetre = banc_stats.depots_fenetre_sb > fen0;
        fen += dans_fenetre;
        uint32_t fn_dec = 0; uint8_t bsic = 0;
        if (g_sb[1].crc_ok) {
            bsic = decode_sb(g_sb[1].sb, &fn_dec);
            bool juste = bsic == banc_bsic && fn_dec == s;
            if (juste) ok++; else { faux++; crc_faux++; }
            snprintf(ligne, sizeof ligne, "SCH fn=%u : CRC ok, BSIC=%u fn=%u TOA=%d SNR=%d%s\n", s, bsic, fn_dec,
                     g_sb[1].toa, g_sb[1].snr, juste ? "" : "  <- CONTENU FAUX");
        } else {
            rate++;
            snprintf(ligne, sizeof ligne, "SCH fn=%u : CRC KO%s (2e essai sur trame factice : %s)\n", s,
                     dans_fenetre ? "" : ", jamais livre dans une fenetre SB", g_sb[2].crc_ok ? "CRC ok ?!" : "CRC KO");
            if (g_sb[2].crc_ok) faux++;
        }
        if (g_v || !g_sb[1].crc_ok || faux) det("%s", ligne);
        vlog("%s", ligne);
    }
    det("%d SCH vises : %d decodes justes, %d CRC KO, %d CRC ok au contenu faux ; %d livres dans une fenetre SB",
        n, ok, rate, faux, fen);
    (void)crc_faux;
    if (faux) return V_FAIL;
    /* un recepteur sain decode un SCH propre a coup sur ; on tolere 1 sur 20 */
    return (ok >= n - 1) ? V_PASS : V_FAIL;
}

/* FB0 -> FB1 -> SB, comme l1s_fbsb_req() + les reponses de prim_fbsb.c */
static int t_fbsb(void)
{
    banc_source = banc_cellule;
    banc_cfg.afc_dac = -700;
    banc_courir(7);
    uint32_t debut; int residu;
    long f0 = une_fb(0, &debut, &residu);
    if (f0 < 0) { det("FB0 introuvable"); return V_FAIL; }
    det("FB0 : TOA=%d %+d Hz -> FCCH %ld\n", g_fb.toa, g_fb.freq, f0);
    long f1 = une_fb(1, &debut, &residu);
    if (f1 < 0) { det("FB1 introuvable"); return V_FAIL; }
    det("FB1 : TOA=%d %+d Hz -> FCCH %ld\n", g_fb.toa, g_fb.freq, f1);
    /* delay = fn_offset + 11 - fn - 1, programme dans la l1_sync de la reponse */
    int toa = g_fb.toa - 23;
    int ntdma = toa < 0 ? -1 : toa / BITS_PER_TDMA;
    int fn_offset = (int)g_fb.fn_resp - g_fb.attempt + ntdma;
    /* nous sommes apres la trame de reponse : le set part de la prochaine l1_sync */
    int delay = fn_offset + 11 - (int)banc_fn() - 1;
    if (delay < 0) delay = 0;
    uint32_t s_vise = (uint32_t)(fn_offset + 11);
    memset(g_sb, 0, sizeof g_sb);
    tdma_schedule_set((uint8_t)(delay + 1), sb_sched_set, 0);
    for (int i = 0; i < delay + 8; i++) banc_trame();
    for (int a = 1; a <= 2; a++) {
        if (!g_sb[a].crc_ok) { det("SB essai %d : CRC KO\n", a); continue; }
        uint32_t fn_dec; uint8_t bsic = decode_sb(g_sb[a].sb, &fn_dec);
        det("SB essai %d : CRC ok BSIC=%u fn=%u (vise %u) TOA=%d\n", a, bsic, fn_dec, s_vise, g_sb[a].toa);
        if (bsic == banc_bsic && fn_dec == s_vise) { det("acquisition complete FB0/FB1/SB : BSIC %u, horloge %u", bsic, fn_dec); return V_PASS; }
    }
    det("SB non decode apres FB1 (SCH vise %u)", s_vise);
    return V_FAIL;
}

/* ===================================================================== */
/*  NB descendant (layer1/prim_rx_nb.c)                                    */
/* ===================================================================== */
static struct { int n_blocs, ok, fire, faux, vide, mauvais_id, err_tot; uint16_t tache; uint8_t chan_type;
                uint32_t blocs_fn[64]; int toa[4], snr[4]; char premier_ko[400]; } g_nb;
static uint8_t g_nb_l2_attendu[23];
static const uint8_t *(*g_nb_l2_de)(uint32_t fn0);
static int nb_resp(uint8_t p1, uint8_t burst_id, uint16_t p3)
{
    (void)p1; (void)p3;
    if (dsp_api.db_r->d_task_d == 0) { g_nb.vide++; return 0; }          /* "EMPTY" */
    if (dsp_api.db_r->d_burst_d != burst_id) { g_nb.mauvais_id++; return 0; }
    g_nb.toa[burst_id] = (int16_t)dsp_api.db_r->a_serv_demod[D_TOA];
    g_nb.snr[burst_id] = dsp_api.db_r->a_serv_demod[D_SNR];
    if (burst_id == 3) {
        uint32_t fn0 = banc_courant.fn - 5;       /* rx du burst 0 : current - 4 - 1 */
        uint8_t l2[23];
        uint16_t fire = (dsp_api.ndb->a_cd[0] & ((1 << B_FIRE1) | (1 << B_FIRE0))) >> B_FIRE0;
        uint16_t err = dsp_api.ndb->a_cd[2];
        dsp_memcpy_from_api(l2, &dsp_api.ndb->a_cd[3], 23, 0);
        const uint8_t *att = g_nb_l2_de(fn0);
        g_nb.n_blocs++;
        g_nb.err_tot += err;
        bool egal = att && !memcmp(l2, att, 23);
        if (!fire && egal) g_nb.ok++;
        else {
            if (fire) g_nb.fire++; else g_nb.faux++;
            if (!g_nb.premier_ko[0]) {
                char h1[80], h2[80]; hex23(h1, l2, 8); hex23(h2, att ? att : l2, 8);
                snprintf(g_nb.premier_ko, sizeof g_nb.premier_ko, "bloc fn0=%u : a_cd0=%04x FIRE=%u err=%u L2 %s.. attendu %s..",
                         fn0, dsp_api.ndb->a_cd[0], fire, err, h1, h2);
            }
        }
        vlog("bloc fn0=%u a_cd0=%04x FIRE=%u err=%u %s TOA=%d/%d/%d/%d\n", fn0, dsp_api.ndb->a_cd[0], fire, err,
             egal ? "L2 egal" : "L2 DIFFERENT", g_nb.toa[0], g_nb.toa[1], g_nb.toa[2], g_nb.toa[3]);
        dsp_api.db_w->d_task_d = 0;
    }
    dsp_api.r_page_used = 1;
    return 0;
}
static int nb_cmd(uint8_t p1, uint8_t burst_id, uint16_t p3)
{
    (void)p1; (void)p3;
    dsp_load_tch_param(&banc_suivant, SIG_ONLY_MODE, g_nb.chan_type, 0, 0, 0, 0);
    dsp_load_rx_task(g_nb.tache, burst_id, banc_bsic & 7);
    return 0;
}
static const struct tdma_sched_item nb_sched_set[] = {
    SCHED_ITEM_DT(nb_cmd, 0, 0, 0), SCHED_END_FRAME(),
    SCHED_ITEM_DT(nb_cmd, 0, 0, 1), SCHED_END_FRAME(),
    SCHED_ITEM(nb_resp, -4, 0, 0), SCHED_ITEM_DT(nb_cmd, 0, 0, 2), SCHED_END_FRAME(),
    SCHED_ITEM(nb_resp, -4, 0, 1), SCHED_ITEM_DT(nb_cmd, 0, 0, 3), SCHED_END_FRAME(),
    SCHED_ITEM(nb_resp, -4, 0, 2), SCHED_END_FRAME(),
    SCHED_ITEM(nb_resp, -4, 0, 3), SCHED_END_FRAME(),
    SCHED_END_SET()
};
static const uint8_t *nb_l2_cellule(uint32_t fn0) { return cellule_nb_alea(fn0, (int)(fn0 % 51)); }

/* Lecture de n blocs xCCH de la cellule a la position p51 (2 = BCCH,
 * 6 = CCCH, 22 = SDCCH/4 n0, 42 = SACCH/4 ...) avec la tache `tache`. */
static int nb_lecture(uint16_t tache, uint8_t chan_type, int p51, int modulo, int n_blocs, const char *quoi)
{
    memset(&g_nb, 0, sizeof g_nb);
    g_nb.tache = tache; g_nb.chan_type = chan_type;
    g_nb_l2_de = nb_l2_cellule;
    banc_source = banc_cellule;
    banc_cellule_nb = cellule_nb_alea;
    banc_mframe_ajouter(nb_sched_set, (uint16_t)modulo, (uint16_t)p51, 0);
    uint32_t lim = banc_fn() + (uint32_t)(modulo * (n_blocs + 1) + 10);
    while (g_nb.n_blocs < n_blocs && banc_fn() < lim) banc_trame();
    banc_mframe_vider();
    det("%s (tache %u) : %d/%d blocs decodes justes, %d FIRE KO, %d contenu faux, %d erreurs canal au total ; "
        "R vide %d, d_burst_d faux %d ; TOA dernier bloc %d/%d/%d/%d",
        quoi, tache, g_nb.ok, g_nb.n_blocs, g_nb.fire, g_nb.faux, g_nb.err_tot, g_nb.vide, g_nb.mauvais_id,
        g_nb.toa[0], g_nb.toa[1], g_nb.toa[2], g_nb.toa[3]);
    if (g_nb.premier_ko[0]) det("\n1er echec : %s", g_nb.premier_ko);
    if (g_nb.n_blocs == 0) { det("\naucun bloc rendu"); return V_FAIL; }
    return (g_nb.ok == g_nb.n_blocs && g_nb.n_blocs >= n_blocs) ? V_PASS : V_FAIL;
}
static int t_nb_bcch(void) { return nb_lecture(ALLC_DSP_TASK, SDCCH_4, 2, 51, 8, "BCCH norm. (ALLC, comme osmocom-bb)"); }

/* ===================================================================== */
/*  NB montant (layer1/prim_tx_nb.c)                                       */
/* ===================================================================== */
static struct { uint16_t tache; uint8_t l2[16][23]; uint32_t fn_cmd[16][4]; int n_blocs; int sacch; } g_tx;
static int tx_resp(uint8_t p1, uint8_t burst_id, uint16_t p3)
{
    (void)p1; (void)burst_id; (void)p3;
    dsp_api.r_page_used = 1;
    return 0;
}
static int tx_cmd(uint8_t p1, uint8_t burst_id, uint16_t p3)
{
    (void)p3;
    if (burst_id == 0) {
        uint16_t *info = dsp_api.ndb->a_cu;
        int b = g_tx.n_blocs < 16 ? g_tx.n_blocs : 15;
        l2_bloc(0x7000u + (uint32_t)banc_suivant.fn, g_tx.l2[b]);
        info[0] = (1 << B_BLUD);
        info[1] = 0;
        info[2] = 0;
        dsp_memcpy_to_api(&info[3], g_tx.l2[b], 23, 0);
        g_tx.n_blocs++;
    }
    if (g_tx.n_blocs >= 1 && g_tx.n_blocs <= 16) g_tx.fn_cmd[g_tx.n_blocs - 1][burst_id & 3] = banc_suivant.fn;
    if (p1 == 2) dsp_load_tch_param(&banc_suivant, SIG_ONLY_MODE, SDCCH_4, 0, 0, 0, 0);
    else         dsp_load_tch_param(&banc_suivant, SIG_ONLY_MODE, INVALID_CHANNEL, 0, 0, 0, 0);
    dsp_load_tx_task(g_tx.tache, burst_id, banc_bsic & 7);
    return 0;
}
static const struct tdma_sched_item nb_sched_set_ul[] = {
    SCHED_ITEM_DT(tx_cmd, 3, 2, 0), SCHED_END_FRAME(),
    SCHED_ITEM_DT(tx_cmd, 3, 2, 1), SCHED_END_FRAME(),
    SCHED_ITEM(tx_resp, -4, 2, 0), SCHED_ITEM_DT(tx_cmd, 3, 2, 2), SCHED_END_FRAME(),
    SCHED_ITEM(tx_resp, -4, 2, 1), SCHED_ITEM_DT(tx_cmd, 3, 2, 3), SCHED_END_FRAME(),
    SCHED_ITEM(tx_resp, -4, 2, 2), SCHED_END_FRAME(),
    SCHED_ITEM(tx_resp, -4, 2, 3), SCHED_END_FRAME(),
    SCHED_END_SET()
};

/* Comparer les bursts emis (banc_ul) apres chaque commande de bloc avec
 * gsm0503_xcch_encode ; a5 : flux attendu XORe (algo, Kc) au fn du burst. */
static int ul_comparer_xcch(int a5_algo, const uint8_t *kc, const char *quoi)
{
    int blocs_ok = 0, bursts_ok = 0, bursts_n = 0, manquants = 0;
    char premier[600] = "";
    for (int b = 0; b < g_tx.n_blocs && b < 16; b++) {
        ubit_t e[4 * 116];
        gsm0503_xcch_encode(e, g_tx.l2[b]);
        int bon = 0;
        for (int k = 0; k < 4; k++) {
            uint32_t f = g_tx.fn_cmd[b][k];
            /* le burst emis pour la trame f : capture a la fin de la trame f */
            const struct banc_ul_burst *u = NULL;
            for (int i = 0; i < banc_n_ul && i < BANC_UL_MAX; i++)
                if (banc_ul[i].fn == f) u = &banc_ul[i];
            bursts_n++;
            if (!u) { manquants++; if (!premier[0]) snprintf(premier, sizeof premier, "bloc %d burst %d (fn %u) : aucun burst emis (0x3d9b immobile)", b, k, f); continue; }
            uint8_t att[116];
            memcpy(att, e + 116 * k, 116);
            if (a5_algo) {
                ubit_t dl[114], ul[114];
                osmo_a5(a5_algo, kc, f, dl, ul);
                for (int j = 0; j < 57; j++) { att[j] ^= ul[j]; att[59 + j] ^= ul[57 + j]; }
            }
            unsigned d = popdiff(u->bits, att, 116);
            if (!d && u->rang == k) { bon++; bursts_ok++; continue; }
            if (!premier[0]) {
                int dec = 99;
                if (a5_algo) {            /* chiffre a une autre trame ? */
                    for (int df = -3; df <= 3; df++) {
                        uint8_t a2[116]; memcpy(a2, e + 116 * k, 116);
                        ubit_t dl[114], ul[114]; osmo_a5(a5_algo, kc, f + df, dl, ul);
                        for (int j = 0; j < 57; j++) { a2[j] ^= ul[j]; a2[59 + j] ^= ul[57 + j]; }
                        if (!popdiff(u->bits, a2, 116)) { dec = df; break; }
                    }
                }
                unsigned dclair = popdiff(u->bits, e + 116 * k, 116);
                snprintf(premier, sizeof premier, "bloc %d burst %d (fn %u, rang ROM %d) : %u bits faux sur 116 (%u contre le burst en clair)%s",
                         b, k, f, u->rang, d, dclair,
                         dec != 99 ? (dec ? " ; chiffre avec le flux d'une autre trame" : "") : "");
                if (dec != 99 && dec) { size_t l = strlen(premier); snprintf(premier + l, sizeof premier - l, " (fn%+d)", dec); }
            }
        }
        if (bon == 4) blocs_ok++;
    }
    det("%s : %d/%d blocs identiques a gsm0503_xcch_encode%s (%d/%d bursts, %d non emis)",
        quoi, blocs_ok, g_tx.n_blocs, a5_algo ? " + A5" : "", bursts_ok, bursts_n, manquants);
    if (premier[0]) det("\n1er ecart : %s", premier);
    return (g_tx.n_blocs > 0 && blocs_ok == g_tx.n_blocs) ? V_PASS : V_FAIL;
}

static int ul_bloc(uint16_t tache, int p51, int modulo, int n_blocs, int a5, const uint8_t *kc, const char *quoi)
{
    memset(&g_tx, 0, sizeof g_tx);
    g_tx.tache = tache;
    banc_source = banc_cellule;
    if (a5) dsp_load_ciph_param(a5, kc);
    banc_mframe_ajouter(nb_sched_set_ul, (uint16_t)modulo, (uint16_t)p51, 0);
    uint32_t lim = banc_fn() + (uint32_t)(modulo * (n_blocs + 1) + 10);
    while (banc_fn() < lim && g_tx.n_blocs < n_blocs) banc_trame();
    banc_mframe_vider();
    banc_courir(8);
    return ul_comparer_xcch(a5, kc, quoi);
}
static int t_tx_sdcch(void) { return ul_bloc(DUL_DSP_TASK, 22 + 15, 51, 6, 0, NULL, "SDCCH/4 n0 montant (DUL, comme osmocom-bb)"); }
/* Meme bloc, chiffre A5/1 par la ROM : ul_comparer_xcch compare au flux osmo_a5(kc, fn) de la trame
 * de la commande (next_time) et dit si la ROM a chiffre avec le flux d'une autre trame (fn+-d). C'est
 * la question ouverte « a quelle heure la ROM chiffre le montant » (99-couverture.sh:137). */
static int t_tx_sdcch_a5(void)
{
    static const uint8_t kc[8] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 };
    return ul_bloc(DUL_DSP_TASK, 22 + 15, 51, 6, 1, kc, "SDCCH/4 n0 montant chiffre A5/1 (DUL)");
}

/* ===================================================================== */
/*  RACH montant (layer1/prim_rach.c)                                      */
/* ===================================================================== */
/* Le firmware ecrit d_rach = (ra << 8) | (bsic << 2) dans le NDB et d_task_ra = RACH_DSP_TASK (10) dans
 * la page W (prim_rach.c l1s_tx_rach_cmd). La ROM code l'access-burst (36 bits codes) dans son anneau
 * TX de mode 2 (pointeur 0x3d97), le serialiseur 0x8900 le copie dans data[0x3f8a] a la trame d'emission,
 * et l'emetteur 0x85a2 y ajoute la sequence de synchro (DROM 0xa0d9, 41 bits) et les queues. Ici on
 * compare les 36 bits codes trouves dans 0x3f8a a gsm0503_rach_ext_encode(ra, bsic) (bits 49..84 du
 * burst de 148). */
static struct { uint8_t ra, bsic; uint32_t fn_cmd; } g_rach;
static int rach_cmd(uint8_t p1, uint8_t p2, uint16_t p3)
{
    (void)p1; (void)p2; (void)p3;
    dsp_api.ndb->d_rach = (uint16_t)((g_rach.ra << 8) | (g_rach.bsic << 2));
    dsp_api.db_w->d_task_ra = RACH_DSP_TASK;
    g_rach.fn_cmd = banc_suivant.fn;
    return 0;
}
static int rach_resp(uint8_t p1, uint8_t p2, uint16_t p3) { (void)p1; (void)p2; (void)p3; dsp_api.r_page_used = 1; return 0; }
static const struct tdma_sched_item rach_sched_set[] = {
    SCHED_ITEM_DT(rach_cmd, 3, 1, 0), SCHED_END_FRAME(),
    SCHED_END_FRAME(),
    SCHED_ITEM(rach_resp, -4, 1, 0), SCHED_END_FRAME(),
    SCHED_END_SET()
};
static int t_rach(void)
{
    banc_source = banc_cellule;
    static const char *sync = "01001011011111111001100110101010001111000";   /* 05.02 5.2.7, 41 bits */
    int n = 0, ok36 = 0, ok148 = 0;
    for (int essai = 0; essai < 5; essai++) {
        banc_courir(4);
        g_rach.ra = alea8(); g_rach.bsic = banc_bsic;
        int n0 = banc_n_tsp;
        tdma_schedule_set(0, rach_sched_set, 0);
        banc_courir(6);
        n++;
        ubit_t cod[36];
        gsm0503_rach_ext_encode(cod, g_rach.ra, g_rach.bsic, false);
        /* 1. les 36 bits codes dans data[0x3f8a..0x3f8c] (le codeur de la ROM) */
        const uint16_t *m = banc_dsp->data;
        int d36 = 0;
        for (int k = 0; k < 36; k++) d36 += ((m[0x3f8a + k / 16] >> (15 - k % 16)) & 1) != cod[k];
        /* 2. le burst final (script TSP, emetteur 0x8605) : [8 garde][8 queue etendue 00111010][41 sync][36][3 queue][garde] */
        char att[8 + 41 + 36 + 3 + 1]; int p = 0;
        memcpy(att, "00111010", 8); p = 8;
        memcpy(att + p, sync, 41); p += 41;
        for (int k = 0; k < 36; k++) att[p++] = '0' + cod[k];
        memcpy(att + p, "000", 3); p += 3; att[p] = 0;
        int trouve = -1; char flux[161];
        for (int i = n0; i < banc_n_tsp; i++) {
            const struct banc_tsp_burst *t = &banc_tsp[i % BANC_UL_MAX];
            for (int w = 0; w < 16; w++) for (int b = 0; b < 10; b++) flux[10 * w + b] = '0' + ((t->mots[w] >> (15 - b)) & 1);
            flux[160] = 0;
            const char *q = strstr(flux, att);
            if (q) { trouve = (int)(q - flux); break; }
        }
        det("essai %d : ra=%02x bsic=%u : 36 bits codes dans 0x3f8a : %d faux ; burst final (TSP, %d capture%s) : [queue 00111010][sync][36][000] %s\n",
            essai, g_rach.ra, g_rach.bsic, d36, banc_n_tsp - n0, banc_n_tsp - n0 > 1 ? "s" : "",
            trouve >= 0 ? "trouve" : "ABSENT");
        if (trouve >= 0) det("          flux de 160 bits : %s (burst a l'offset %d)\n", flux, trouve);
        ok36 += d36 == 0; ok148 += trouve >= 0;
    }
    det("RACH : %d/%d codages (36 bits) = gsm0503_rach_ext_encode ; %d/%d access-bursts complets dans le script TSP de la ROM", ok36, n, ok148, n);
    return (ok36 == n && ok148 == n) ? V_PASS : V_FAIL;
}

/* ===================================================================== */
/*  table des tests                                                        */
/* ===================================================================== */
/* TOA de la FB quand la FCCH est dans la TOUTE PREMIERE trame de recherche
 * (commande a p51 = 9, recherche des p51 = 10 : FCCH). FBPOS_MODE=0|1 (defaut 1).
 * Avec BANC_FB_DECAL=k (decalage du burst depuis l'ouverture de la fenetre), donne
 * le TOA de la FB en fonction de k, sans les termes de pages de 48 d'une FCCH lointaine. */
static int t_fb_pos(void)
{
    banc_source = banc_cellule;
    banc_cfg.afc_dac = -700;
    const char *em = getenv("FBPOS_MODE");
    int mode = em ? atoi(em) : 1;
    int vus = 0;
    for (int essai = 0; essai < 3; essai++) {
        uint32_t cible = (banc_fn() / 51 + 2) * 51 + 9;
        while (banc_fn() < cible) banc_trame();
        uint32_t debut; int residu = 0;
        long fcch = une_fb(mode, &debut, &residu);
        if (fcch < 0 || g_fb.det <= 0) { det("essai %d : debut=%u (p51=%u) : pas de FB\n", essai, debut, debut % 51); continue; }
        vus++;
        det("essai %d : debut=%u (p51=%u) FB%d att=%d TOA=%d PM=%d angle=%d\n",
            essai, debut, debut % 51, mode, g_fb.attempt, g_fb.toa, g_fb.pm, g_fb.angle);
    }
    return vus ? V_PASS : V_FAIL;
}

typedef struct { const char *nom, *tache, *desc; int (*f)(void); } Test;
static const Test TESTS[] = {
    { "boot",      "-",            "chargeur + dsp_power_on() du firmware, IDLE, version API", t_boot },
    { "idle",      "0 (aucune)",   "scenario vide chaque trame : IT trame, retour IDLE, pas d'erreur", t_idle },
    { "checksum",  "33 CHECKSUM",  "dsp_checksum_task() : version/somme du code dans a_pm", t_checksum },
    { "fb0",       "5 FB, mode 0", "detection FCCH (d_fb_det, a_sync_demod : TOA -> trame, angle -> Hz)", t_fb0 },
    { "fb1",       "5 FB, mode 1", "detection FCCH en poursuite", t_fb1 },
    { "fbpos",     "5 FB",         "TOA FB, FCCH dans la 1re trame de recherche (FBPOS_MODE, BANC_FB_DECAL)", t_fb_pos },
    { "fb-afc",    "5 FB, mode 0", "erreur de frequence mesuree avec un VCXO decale (+60 LSB)", t_fb_afc },
    { "sb",        "6 SB",         "SCH : CRC (a_sch[0]), BSIC et T1/T2/T3 (a_sch[3..4]) sur 20 SCH", t_sb },
    { "fbsb",      "5,5,6",        "acquisition complete FB0 -> FB1 -> SB (prim_fbsb.c)", t_fbsb },
    { "nb",        "24 ALLC",      "BCCH : 8 blocs, a_cd FIRE + 23 octets == gsm0503_xcch_encode", t_nb_bcch },
    { "rach",      "10 RACH",      "access-burst montant : codage ROM (0x3f8a) et burst final (script TSP) == 05.03/05.02", t_rach },
    { "tx-sdcch",  "12 DUL",       "SDCCH montant : a_cu -> bursts 0x3f8a == gsm0503_xcch_encode", t_tx_sdcch },
    { "tx-sdcch-a5","12 DUL + A5", "SDCCH montant chiffre : bursts 0x3f8a == xcch_encode XOR osmo_a5(kc, fn de la commande)", t_tx_sdcch_a5 },
};
#define N_TESTS (int)(sizeof TESTS / sizeof TESTS[0])

/* Un test dans un fils, depuis le DSP demarre. */
static int lancer(const Test *t)
{
    int tube[2];
    /* DSP_TESTER_NOFORK=1 : le test tourne dans ce processus (gdb, gprof) ; un plantage emporte tout. */
    static int nofork = -1;
    if (nofork < 0) { const char *e = getenv("DSP_TESTER_NOFORK"); nofork = (e && *e == '1') ? 1 : 0; }
    if (nofork) {
        g_dl = 0; g_detail[0] = 0;
        int v = t->f();
        printf("[%-8s] %-10s %-14s %s\n", NOMV[v], t->nom, t->tache, g_detail);
        return v;
    }
    if (pipe(tube) < 0) { perror("pipe"); return V_FAIL; }
    fflush(stdout);
    pid_t p = fork();
    if (p == 0) {
        close(tube[0]);
        alarm(900);
        g_dl = 0; g_detail[0] = 0;
        int v = t->f();
        fflush(stdout);
        char buf[sizeof g_detail + 4];
        buf[0] = (char)v;
        memcpy(buf + 1, g_detail, g_dl + 1);
        if (write(tube[1], buf, g_dl + 2) < 0) _exit(3);
        close(tube[1]);
        _exit(0);
    }
    close(tube[1]);
    static char buf[sizeof g_detail + 4];
    ssize_t n = 0, r;
    while ((r = read(tube[0], buf + n, sizeof buf - 1 - (size_t)n)) > 0) n += r;
    close(tube[0]);
    int st = 0;
    waitpid(p, &st, 0);
    int v;
    const char *d;
    char crash[200];
    if (n >= 2) { v = buf[0]; buf[n] = 0; d = buf + 1; }
    else {
        v = V_FAIL;
        snprintf(crash, sizeof crash, "le test s'est arrete sans verdict (%s %d) : plantage du coeur ou delai depasse",
                 WIFSIGNALED(st) ? "signal" : "code", WIFSIGNALED(st) ? WTERMSIG(st) : WEXITSTATUS(st));
        d = crash;
    }
    printf("[%-8s] %-10s %-14s ", NOMV[v], t->nom, t->tache);
    /* premiere ligne du detail sur la ligne du verdict, le reste indente */
    const char *q = d;
    while (*q) {
        const char *e = strchr(q, '\n');
        size_t l = e ? (size_t)(e - q) : strlen(q);
        if (q != d) printf("%38s", "");
        printf("%.*s\n", (int)l, q);
        q += l + (e ? 1 : 0);
    }
    if (!*d) printf("\n");
    return v;
}

static void usage(const char *p)
{
    printf("usage : %s [-v] [--trace 0..6] [--rom-dir DIR] [--insns N] [--bsic N] (--all | --list | TEST...)\n", p);
}

int main(int argc, char **argv)
{
    int tous = 0, liste = 0, n_choix = 0;
    const char *choix[64];
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "-v")) g_v = 1;
        else if (!strcmp(a, "--all")) tous = 1;
        else if (!strcmp(a, "--list")) liste = 1;
        else if (!strcmp(a, "--rom-dir") && i + 1 < argc) banc_cfg.rom_dir = argv[++i];
        else if (!strcmp(a, "--insns") && i + 1 < argc) banc_cfg.budget = atol(argv[++i]);
        else if (!strcmp(a, "--bsic") && i + 1 < argc) banc_bsic = (uint8_t)(atoi(argv[++i]) & 0x3f);
        else if (!strcmp(a, "--trace") && i + 1 < argc) g_trace = atoi(argv[++i]);
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); return 0; }
        else if (a[0] == '-') { usage(argv[0]); return 255; }
        else if (n_choix < 64) choix[n_choix++] = a;
    }
    if (liste) {
        for (int i = 0; i < N_TESTS; i++) printf("  %-10s %-14s %s\n", TESTS[i].nom, TESTS[i].tache, TESTS[i].desc);
        return 0;
    }
    if (!tous && !n_choix) { usage(argv[0]); return 255; }
    for (int k = 0; k < n_choix; k++) {
        int vu = 0;
        for (int i = 0; i < N_TESTS; i++) if (!strcmp(choix[k], TESTS[i].nom)) vu = 1;
        if (!vu) { fprintf(stderr, "test inconnu : %s (--list)\n", choix[k]); return 255; }
    }
    banc_cfg.verbeux = g_v;
    /* Les sondes du coeur ecrivent sur stderr ; filtrees comme dans c54x_exe
     * (src/verbosite.c). Les fils ecrivent dans le meme tube, le fil de
     * filtrage reste dans le pere. */
    verbosite_installer(g_trace);
    if (banc_init() < 0) return 255;
    g_boot_rc = banc_dsp_power_on(&g_boot);
    printf("dsp_tester : ROM %s (API %04x), %ld insn/trame, BSIC %u ; l'ARM et la BTS sont simules, rien n'est envoye au dehors\n",
           banc_cfg.rom_dir, g_boot.version1, banc_cfg.budget, banc_bsic);
    int n[3] = { 0, 0, 0 };
    for (int i = 0; i < N_TESTS; i++) {
        int pris = tous;
        for (int k = 0; k < n_choix; k++) if (!strcmp(choix[k], TESTS[i].nom)) pris = 1;
        if (!pris) continue;
        n[lancer(&TESTS[i])]++;
    }
    printf("bilan : %d PASS, %d FAIL, %d NON-IMPL\n", n[0], n[1], n[2]);
    verbosite_retirer();
    if (g_trace) verbosite_bilan(stdout);
    return n[1] > 254 ? 254 : n[1];
}
