/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * dsp_banc_commun.h - faire tourner le DSP Calypso (mask-ROM TI) comme le fait
 * l'ARM osmocom-bb, sans QEMU ni ARM. Module commun, reutilisable par les bancs
 * hors ligne (tools/dsp_tester.c en est le premier client).
 *
 * Ce que le module fournit :
 *   - le coeur C54x + la ROM + le DMA + le BSP, compiles depuis qosmo ;
 *   - dsp_power_on() du firmware (calypso/dsp.c) : chargeur, parametres
 *     (dsp_params.c du firmware, inclus tel quel), NDB, pages ;
 *   - les primitives du firmware : dsp_end_scenario, dsp_load_rx_task,
 *     dsp_load_tx_task, dsp_load_tch_param, dsp_load_ciph_param,
 *     dsp_memcpy_to/from_api, l1s_reset_hw ;
 *   - l'ordonnanceur TDMA du firmware (layer1/tdma_sched.c, recopie) et un
 *     ordonnanceur de multitrame minimal (layer1/mframe_sched.c : SCHEDULE_AHEAD
 *     2, SCHEDULE_LATENCY 1) ;
 *   - la trame du banc, dans l'ordre de src/pont.c (jouer_trame + servir) :
 *     IT trame si l'ARM a clos un scenario, phase A jusqu'a l'armement de la
 *     fenetre RX (jusqu'a l'IDLE sur TCH), l1_sync() de l'ARM, depot du burst
 *     de la trame (bits -> GMSK, cadre comme bsp_ts0_livrer), reste du budget,
 *     pompe DMA ;
 *   - la capture des bursts montants que la ROM emet (data[0x3f8a], pointeur
 *     data[0x3d9b], voir README « ROM and uplink probes »).
 *
 * Les structures de l'API RAM sont celles du firmware (dsp_api.h d'osmocom-bb,
 * DSP == 36), posees sur data[0x0800..] : le code des bancs s'ecrit donc comme
 * celui du firmware (dsp_api.ndb->a_cd[0], dsp_api.db_w->d_task_d, ...).
 */
#ifndef DSP_BANC_COMMUN_H
#define DSP_BANC_COMMUN_H

#include <stdint.h>
#include <stdbool.h>
#include "calypso_c54x.h"
#include "dsp_api.h"            /* osmocom-bb : include/calypso/dsp_api.h */

/* ---- etat cote ARM, comme la struct dsp_api du firmware ------------------ */
struct banc_dsp_api {
    T_NDB_MCU_DSP     *ndb;
    T_DB_DSP_TO_MCU   *db_r;
    T_DB_MCU_TO_DSP   *db_w;
    T_PARAM_MCU_DSP   *param;
    int r_page, w_page, r_page_used;
    uint32_t frame_ctr;
};
extern struct banc_dsp_api dsp_api;
extern C54xState *banc_dsp;
extern uint16_t  *banc_api;            /* = &banc_dsp->data[0x0800] */

/* GSM time (l1s.current_time / l1s.next_time) */
struct banc_temps { uint32_t fn; uint16_t t1; uint8_t t2, t3; };
void banc_fn2temps(struct banc_temps *t, uint32_t fn);
extern struct banc_temps banc_courant, banc_suivant;

/* ---- configuration -------------------------------------------------------- */
struct banc_config {
    const char *rom_dir;    /* calypso_dsp.*.bin (defaut /opt/GSM) */
    long budget;            /* instructions par trame (defaut 300000, ~65 MHz) */
    int verbeux;
    int16_t afc_dac;        /* d_afc ecrit par l'ARM (-700 = pas d'erreur de frequence) */
};
extern struct banc_config banc_cfg;

/* Boot: what dsp_power_on() observed. */
struct banc_boot {
    long insn_chargeur;     /* instructions jusqu'a BL_STATUS_IDLE */
    long insn_demarrage;    /* instructions de 0x7000 jusqu'a l'IDLE */
    uint16_t bl_status;
    uint16_t version1, version2;
    bool idle;
};

int  banc_init(void);                          /* coeur, ROM, reset, DMA, BSP */
int  banc_dsp_power_on(struct banc_boot *b);   /* calypso/dsp.c dsp_power_on() */

/* ---- primitives du firmware (calypso/dsp.c, layer1/sync.c) ---------------- */
void dsp_end_scenario(void);
void dsp_load_rx_task(uint16_t task, uint8_t burst_id, uint8_t tsc);
void dsp_load_tx_task(uint16_t task, uint8_t burst_id, uint8_t tsc);
void dsp_load_tch_param(const struct banc_temps *next_time, uint8_t chan_mode,
                        uint8_t chan_type, uint8_t chan_sub, uint8_t tch_loop,
                        uint8_t sync_tch, uint8_t tn);
void dsp_load_ciph_param(int mode, const uint8_t *key);
void dsp_memcpy_to_api(volatile uint16_t *dsp_buf, const uint8_t *mcu_buf, int n, int be);
void dsp_memcpy_from_api(uint8_t *mcu_buf, const volatile uint16_t *dsp_buf, int n, int be);
void l1s_reset_hw(void);

/* ---- ordonnanceur TDMA (layer1/tdma_sched.c) ------------------------------ */
#define TDMA_IFLG_TPU   (1 << 0)
#define TDMA_IFLG_DSP   (1 << 1)
typedef int tdma_sched_cb(uint8_t p1, uint8_t p2, uint16_t p3);
struct tdma_sched_item {
    tdma_sched_cb *cb;
    uint8_t p1, p2;
    uint16_t p3;
    int16_t prio;
    uint16_t flags;
};
int tdma_end_set(uint8_t p1, uint8_t p2, uint16_t p3);
#define SCHED_ITEM(x, p, y, z)      { .cb = x, .p1 = y, .p2 = z, .prio = p, .flags = 0 }
#define SCHED_ITEM_DT(x, p, y, z)   { .cb = x, .p1 = y, .p2 = z, .prio = p, \
                                      .flags = TDMA_IFLG_TPU | TDMA_IFLG_DSP }
#define SCHED_END_FRAME()           { .cb = NULL, .p1 = 0, .p2 = 0 }
#define SCHED_END_SET()             { .cb = &tdma_end_set, .p1 = 0, .p2 = 0 }
int  tdma_schedule(uint8_t frame_offset, tdma_sched_cb *cb, uint8_t p1, uint8_t p2,
                   uint16_t p3, int16_t prio);
int  tdma_schedule_set(uint8_t frame_offset, const struct tdma_sched_item *set, uint16_t p3);
void tdma_sched_reset(void);

/* ---- multitrame (layer1/mframe_sched.c, reduit) --------------------------- */
#define MF_F_SACCH  (1 << 0)
struct banc_mframe_item { const struct tdma_sched_item *set; uint16_t modulo, frame_nr; uint16_t p3; };
void banc_mframe_ajouter(const struct tdma_sched_item *set, uint16_t modulo,
                         uint16_t frame_nr, uint16_t p3);
void banc_mframe_vider(void);

/* ---- le canal radio descendant -------------------------------------------- */
/* Le burst que la radio recoit a la trame fn (l'intervalle suivi : TS0 hors
 * dedie, l'intervalle du canal dedie sur ses trames, comme bsp_ts0_livrer).
 * Rend le type : 'F' FCCH, 'S' SCH, 'N' burst normal, 'D' factice, 0 = rien
 * (silence). amp : amplitude (defaut 30000 si 0). */
typedef char (*banc_source_fn)(uint32_t fn, uint8_t bits[148], int *amp);
extern banc_source_fn banc_source;
/* Bits de l'autre intervalle tn (1..7) en DMA continue (recherche FB) ; NULL =
 * burst factice (porteuse C0). */
extern const uint8_t *(*banc_source_autre)(uint32_t fn, unsigned tn);

/* Cellule synthetique TS0 combinee CCCH + SDCCH/4 (45.002 table 3) :
 * FCCH 0/10/20/30/40, SCH 1/11/21/31/41 (BSIC banc_bsic), le reste : le
 * contenu rendu par banc_cellule_nb (bloc xCCH de 23 octets, NULL = factice). */
extern uint8_t banc_bsic;
typedef const uint8_t *(*banc_cellule_nb_fn)(uint32_t fn0, int bloc_p51);
extern banc_cellule_nb_fn banc_cellule_nb;
char banc_cellule(uint32_t fn, uint8_t bits[148], int *amp);

/* Formes de burst (45.002 5.2) */
extern const uint8_t banc_tsc[8][26];
extern const uint8_t banc_train_sb[64];
extern const uint8_t banc_factice[148];
void banc_burst_nb(const uint8_t *e116, uint8_t tsc, uint8_t bits[148]);   /* 57+hl, TSC, hu+57 */
void banc_burst_sch(uint32_t fn, uint8_t bsic, uint8_t bits[148]);
void banc_sb_info(uint32_t fn, uint8_t bsic, uint8_t sb_info[4]);

/* ---- une trame TDMA ------------------------------------------------------ */
/* Crochets : banc_crochet_l1s est appele par l1_sync() de chaque trame, apres
 * l'execution des items de l'ordonnanceur et avant la bascule des pages (logique
 * de test cote ARM) ; s'il rend TDMA_IFLG_DSP, le scenario DSP est clos comme
 * pour un item SCHED_ITEM_DT. banc_crochet_fin : en fin de trame (sorties). */
extern int (*banc_crochet_l1s)(uint32_t fn);
extern void (*banc_crochet_fin)(uint32_t fn);
/* sur_tch : phase A jusqu'a l'IDLE avant le depot (pont.c, PONT_TCH_DEPOT_IDLE) */
extern bool banc_sur_tch;
void banc_trame(void);                 /* une trame : phase A, l1_sync, depot, phase B, pompe */
void banc_courir(uint32_t n_trames);
uint32_t banc_fn(void);                /* trame du prochain banc_trame() */
void banc_set_fn(uint32_t fn);         /* recaler l'horloge (avant la premiere trame) */

/* Statistiques de la trame */
struct banc_stats {
    unsigned long trames, irq_trame, depots, depots_fenetre_sb, pas_idle_fin;
    unsigned long erreurs_dsp;          /* d_error_status non nul */
    uint16_t derniere_erreur;
    long insn_max;
};
extern struct banc_stats banc_stats;

/* ---- bursts montants emis par la ROM ------------------------------------- */
/* La ROM copie a chaque trame d'emission le burst courant (116 bits : 57,
 * hl, hu, 57 -> ordre libosmocoding) dans data[0x3f8a..0x3f91] et avance
 * data[0x3d9b] (pont.c tx_rom_publier). flux = flux A5 montant data[0x3f9b..]. */
/* Un burst serialise par la ROM dans data[0x3f8a..0x3f91] (serialiseur PROM0 0x8900) : anneau xCCH
 * (pointeur 0x3d9b, mode 0x3fac = 3), TCH (0x3d91, mode 0), mode 1 (0x3d93/0x3d95), RACH (0x3d97/0x3d99,
 * mode 2). rang : rang du burst dans le bloc xCCH (0..3), -1 sinon ; raw : les 8 mots tels quels. */
struct banc_ul_burst { uint32_t fn; int rang; int mode; uint16_t ptr; uint16_t raw[8]; uint8_t bits[116]; uint8_t flux[116]; };
#define BANC_UL_MAX 512
extern struct banc_ul_burst banc_ul[BANC_UL_MAX];
/* Le burst montant FINAL tel que la ROM le remet au BSP de l'ABB : 16 mots BULDATA (10 bits de donnees
 * en [15:6], registre BULDATA1 = 3 en [5:1]) ecrits par l'emetteur PROM0 0x8605 dans le script TSP
 * (data[0x3cbb..], juste apres le mot TOGBR2 = 0x1c0a et un mot de puissance). 160 bits = garde/queue
 * etendue + burst de 148 + garde. Capture a chaque trame ou ces 16 mots changent. */
struct banc_tsp_burst { uint32_t fn; uint16_t mots[16]; };
extern struct banc_tsp_burst banc_tsp[BANC_UL_MAX];
extern int banc_n_tsp;
extern int banc_n_ul;
void banc_ul_capturer(uint32_t fn);    /* appele en fin de trame par banc_trame() */

/* Helpers */
long banc_executer(long n);            /* c54x_run en boucle (rend la main tous les 32768) */
void banc_trame_vide(void);            /* une trame sans burst ni IT (DSP au repos) */

#endif
