/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * dsp_banc_commun.h - run the Calypso DSP (TI mask-ROM) the way the osmocom-bb
 * ARM does, without QEMU and without the ARM. Common module, reusable by the
 * offline benches (tools/dsp_tester.c is its first client).
 *
 * What the module provides:
 *   - the C54x core + the ROM + the DMA + the BSP, compiled from qosmo;
 *   - dsp_power_on() of the firmware (calypso/dsp.c): loader, parameters
 *     (dsp_params.c of the firmware, included as is), NDB, pages;
 *   - the firmware primitives: dsp_end_scenario, dsp_load_rx_task,
 *     dsp_load_tx_task, dsp_load_tch_param, dsp_load_ciph_param,
 *     dsp_memcpy_to/from_api, l1s_reset_hw;
 *   - the TDMA scheduler of the firmware (layer1/tdma_sched.c, copied) and a
 *     minimal multiframe scheduler (layer1/mframe_sched.c: SCHEDULE_AHEAD 2,
 *     SCHEDULE_LATENCY 1);
 *   - the bench frame, in the order of src/pont.c (jouer_trame + servir):
 *     frame IT if the ARM closed a scenario, phase A until the RX window is
 *     armed (until the IDLE on TCH), l1_sync() of the ARM, deposit of the
 *     frame's burst (bits -> GMSK, framed like bsp_ts0_livrer), rest of the
 *     budget, DMA pump;
 *   - the capture of the uplink bursts the ROM sends (data[0x3f8a], pointer
 *     data[0x3d9b], see README "ROM and uplink probes").
 *
 * The API RAM structures are those of the firmware (dsp_api.h of osmocom-bb,
 * DSP == 36), laid over data[0x0800..]: bench code is therefore written like
 * firmware code (dsp_api.ndb->a_cd[0], dsp_api.db_w->d_task_d, ...).
 */
#ifndef DSP_BANC_COMMUN_H
#define DSP_BANC_COMMUN_H

#include <stdint.h>
#include <stdbool.h>
#include "calypso_c54x.h"
#include "dsp_api.h"            /* osmocom-bb: include/calypso/dsp_api.h */

/* ---- ARM-side state, like the firmware's struct dsp_api ------------------ */
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
    const char *rom_dir;    /* calypso_dsp.*.bin (default /opt/GSM) */
    long budget;            /* instructions per frame (default 300000, ~65 MHz) */
    int verbeux;
    int16_t afc_dac;        /* d_afc written by the ARM (-700 = no frequency error) */
};
extern struct banc_config banc_cfg;

/* Boot: what dsp_power_on() observed. */
struct banc_boot {
    long insn_chargeur;     /* instructions up to BL_STATUS_IDLE */
    long insn_demarrage;    /* instructions from 0x7000 up to the IDLE */
    uint16_t bl_status;
    uint16_t version1, version2;
    bool idle;
};

int  banc_init(void);                          /* core, ROM, reset, DMA, BSP */
int  banc_dsp_power_on(struct banc_boot *b);   /* calypso/dsp.c dsp_power_on() */

/* ---- firmware primitives (calypso/dsp.c, layer1/sync.c) ------------------- */
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

/* ---- TDMA scheduler (layer1/tdma_sched.c) --------------------------------- */
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

/* ---- multiframe (layer1/mframe_sched.c, reduced) -------------------------- */
#define MF_F_SACCH  (1 << 0)
struct banc_mframe_item { const struct tdma_sched_item *set; uint16_t modulo, frame_nr; uint16_t p3; };
void banc_mframe_ajouter(const struct tdma_sched_item *set, uint16_t modulo,
                         uint16_t frame_nr, uint16_t p3);
void banc_mframe_vider(void);

/* ---- the downlink radio channel ------------------------------------------- */
/* The burst the radio receives at frame fn (the followed timeslot: TS0 outside
 * dedicated mode, the dedicated channel's timeslot on its frames, like
 * bsp_ts0_livrer). Returns the type: 'F' FCCH, 'S' SCH, 'N' normal burst,
 * 'D' dummy, 0 = nothing (silence). amp: amplitude (default 30000 if 0). */
typedef char (*banc_source_fn)(uint32_t fn, uint8_t bits[148], int *amp);
extern banc_source_fn banc_source;
/* Bits of the other timeslot tn (1..7) in continuous DMA (FB search); NULL =
 * dummy burst (C0 carrier). */
extern const uint8_t *(*banc_source_autre)(uint32_t fn, unsigned tn);

/* Synthetic TS0 cell, combined CCCH + SDCCH/4 (45.002 table 3):
 * FCCH 0/10/20/30/40, SCH 1/11/21/31/41 (BSIC banc_bsic), the rest: the
 * content returned by banc_cellule_nb (23-byte xCCH block, NULL = dummy). */
extern uint8_t banc_bsic;
typedef const uint8_t *(*banc_cellule_nb_fn)(uint32_t fn0, int bloc_p51);
extern banc_cellule_nb_fn banc_cellule_nb;
char banc_cellule(uint32_t fn, uint8_t bits[148], int *amp);

/* Burst formats (45.002 5.2) */
extern const uint8_t banc_tsc[8][26];
extern const uint8_t banc_train_sb[64];
extern const uint8_t banc_factice[148];
void banc_burst_nb(const uint8_t *e116, uint8_t tsc, uint8_t bits[148]);   /* 57+hl, TSC, hu+57 */
void banc_burst_sch(uint32_t fn, uint8_t bsic, uint8_t bits[148]);
void banc_sb_info(uint32_t fn, uint8_t bsic, uint8_t sb_info[4]);

/* ---- one TDMA frame ------------------------------------------------------ */
/* Hooks: banc_crochet_l1s is called by l1_sync() of every frame, after the
 * scheduler items have run and before the page switch (ARM-side test logic);
 * if it returns TDMA_IFLG_DSP, the DSP scenario is closed as for a
 * SCHED_ITEM_DT item. banc_crochet_fin: at the end of the frame (outputs). */
extern int (*banc_crochet_l1s)(uint32_t fn);
extern void (*banc_crochet_fin)(uint32_t fn);
/* sur_tch: phase A until the IDLE before the deposit (pont.c, PONT_TCH_DEPOT_IDLE) */
extern bool banc_sur_tch;
void banc_trame(void);                 /* one frame: phase A, l1_sync, deposit, phase B, pump */
void banc_courir(uint32_t n_trames);
uint32_t banc_fn(void);                /* frame of the next banc_trame() */
void banc_set_fn(uint32_t fn);         /* reset the clock (before the first frame) */

/* Frame statistics */
struct banc_stats {
    unsigned long trames, irq_trame, depots, depots_fenetre_sb, pas_idle_fin;
    unsigned long erreurs_dsp;          /* non-zero d_error_status */
    uint16_t derniere_erreur;
    long insn_max;
};
extern struct banc_stats banc_stats;

/* ---- uplink bursts sent by the ROM --------------------------------------- */
/* On every transmit frame the ROM copies the current burst (116 bits: 57,
 * hl, hu, 57 -> libosmocoding order) into data[0x3f8a..0x3f91] and advances
 * data[0x3d9b] (pont.c tx_rom_publier). flux = uplink A5 stream data[0x3f9b..]. */
/* One burst serialised by the ROM in data[0x3f8a..0x3f91] (serialiser PROM0 0x8900): xCCH ring
 * (pointer 0x3d9b, mode 0x3fac = 3), TCH (0x3d91, mode 0), mode 1 (0x3d93/0x3d95), RACH
 * (0x3d97/0x3d99, mode 2). rang: rank of the burst in the xCCH block (0..3), -1 otherwise;
 * raw: the 8 words as they are. */
struct banc_ul_burst { uint32_t fn; int rang; int mode; uint16_t ptr; uint16_t raw[8]; uint8_t bits[116]; uint8_t flux[116]; };
#define BANC_UL_MAX 512
extern struct banc_ul_burst banc_ul[BANC_UL_MAX];
/* The FINAL uplink burst as the ROM hands it to the ABB's BSP: 16 BULDATA words (10 data bits
 * in [15:6], register BULDATA1 = 3 in [5:1]) written by the transmitter PROM0 0x8605 into the TSP
 * script (data[0x3cbb..], right after the word TOGBR2 = 0x1c0a and a power word). 160 bits =
 * guard/extended tail + 148-bit burst + guard. Captured on every frame where these 16 words
 * change. */
struct banc_tsp_burst { uint32_t fn; uint16_t mots[16]; };
extern struct banc_tsp_burst banc_tsp[BANC_UL_MAX];
extern int banc_n_tsp;
extern int banc_n_ul;
void banc_ul_capturer(uint32_t fn);    /* called at the end of the frame by banc_trame() */

/* Helpers */
long banc_executer(long n);            /* c54x_run in a loop (yields every 32768) */
void banc_trame_vide(void);            /* one frame without burst or IT (DSP at rest) */

#endif
