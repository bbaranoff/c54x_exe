/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * dsp_banc_commun.c - the Calypso DSP driven as by the osmocom-bb ARM.
 * See dsp_banc_commun.h. The sequences are copied from the firmware
 * (osmocom-bb src/target/firmware: calypso/dsp.c, layer1/sync.c,
 * layer1/tdma_sched.c, layer1/mframe_sched.c) and from the bench (src/pont.c:
 * jouer_trame, servir; qosmo calypso_bsp.c: bsp_ts0_livrer, sb_moduler).
 * Every deviation is stated where it is made.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "qemu/thread.h"
#include "dsp_banc_commun.h"
#include "calypso_dma.h"
#include "calypso_bsp.h"
#include "calypso_rhea_dma.h"
#include "calypso_rif.h"
#include "calypso_twl3025.h"
#include "calypso_gmsk.h"
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>

/* ---- what main.c of c54x_exe normally provides ---------------------------- */
uint32_t g_c54x_exe_fn;
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }
void calypso_inth_arm_ack(void) { }
QemuMutex calypso_pcb_daram_lock;
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{ (void)addr; if (!wr) memset(buf, 0, len); }
extern int c54x_rapide;

/* ---- state ---------------------------------------------------------------- */
struct banc_dsp_api dsp_api;
C54xState *banc_dsp;
uint16_t *banc_api;
struct banc_temps banc_courant, banc_suivant;
struct banc_config banc_cfg = { .rom_dir = "/opt/GSM", .budget = 300000, .verbeux = 0, .afc_dac = -700 };
struct banc_stats banc_stats;
banc_source_fn banc_source = banc_cellule;
const uint8_t *(*banc_source_autre)(uint32_t fn, unsigned tn);
uint8_t banc_bsic = 7;
banc_cellule_nb_fn banc_cellule_nb;
int (*banc_crochet_l1s)(uint32_t fn);
void (*banc_crochet_fin)(uint32_t fn);
bool banc_sur_tch;
struct banc_ul_burst banc_ul[BANC_UL_MAX];
struct banc_tsp_burst banc_tsp[BANC_UL_MAX];
int banc_n_tsp;
static uint16_t g_tsp_prec[16];
int banc_n_ul;

static int g_irq_armee;            /* dsp_end_scenario() at the last l1_sync */
static uint16_t g_ul_prec;
static uint16_t g_ul_prec_ptr[6];   /* last pointers of the TX rings: 0x3d91, 0x3d93, 0x3d95, 0x3d97, 0x3d99, 0x3d9b */

/* API RAM, in words from data[0x0800] (dsp_api.h: BASE_API_*) */
#define W_PAGE_MOT(p)   ((p) ? 0x14u : 0x00u)
#define R_PAGE_MOT(p)   ((p) ? 0x3Cu : 0x28u)
#define NDB_MOT         0xD4u
#define PARAM_MOT       0x431u
#define BL_ADDR_HI_W    0x7FCu     /* BL_ADDR_HI = BASE_API_RAM + 0x0ff8 */
#define BL_SIZE_W       0x7FDu
#define BL_ADDR_LO_W    0x7FEu
#define BL_STATUS_W     0x7FFu
#define BL_STATUS_IDLE  1
#define BL_CMD_COPY_BLOCK 2
#define DSP_START       0x7000

/* TWL3025 registers (osmocom-bb include/abb/twl3025.h; ABB_VAL only keeps
 * the 5 low bits, the page is dropped). */
#define PAGE(n) ((n) << 7)
enum { R_AFCCTLADD = PAGE(1) | 21, R_APCDEL1 = PAGE(0) | 2, R_APCDEL2 = PAGE(1) | 26,
       R_APCRAM = PAGE(0) | 10, R_APCOFF = PAGE(0) | 11, R_AUXDAC = PAGE(0) | 12,
       R_BULIOFF = PAGE(1) | 2, R_BULQOFF = PAGE(1) | 3, R_BULGCAL = PAGE(1) | 14,
       R_BBCTRL = PAGE(1) | 6, R_VBCTRL1 = PAGE(1) | 8, R_VBCTRL2 = PAGE(1) | 11,
       R_VBUCTRL = PAGE(1) | 7, R_VBDCTRL = PAGE(0) | 6 };
#define ABB_RAMP_VAL(up, down)  ((((down) & 0x1F) << 5) | ((up) & 0x1F))

/* calypso/dsp_params.c of the firmware, included as is: the parameter table
 * of a real phone (ROM 3306/3606) that dsp_set_params() copies into the API. */
#include "dsp_params.c"

void banc_fn2temps(struct banc_temps *t, uint32_t fn)
{
    fn %= 2715648u;
    t->fn = fn;
    t->t1 = (uint16_t)(fn / (26 * 51));
    t->t2 = (uint8_t)(fn % 26);
    t->t3 = (uint8_t)(fn % 51);
}

long banc_executer(long n)
{
    long k = 0;
    while (k < n && banc_dsp->running && !banc_dsp->idle) {
        long m = n - k;
        int r = c54x_run(banc_dsp, (int)(m > 0x40000000L ? 0x40000000L : m));
        if (r <= 0) break;
        k += r;
    }
    return k;
}

/* A DSP at IDLE is woken up by a pending interrupt (SPRU131); INT10n (end of
 * RHEA DMA, IMR bit 14) is a level line. pont.c. */
static void reveil(void)
{
    C54xState *d = banc_dsp;
    if (d->idle && calypso_rhea_dma_irq_level() && (d->imr & (1u << 14)) && !(d->ifr & (1u << 14)))
        c54x_interrupt_ex(d, 30, 14);
    if (d->idle && (d->ifr & d->imr) && !(d->st1 & 0x800))
        d->idle = false;
}

static void pages(void)
{
    dsp_api.db_w = (T_DB_MCU_TO_DSP *)&banc_api[W_PAGE_MOT(dsp_api.w_page)];
    dsp_api.db_r = (T_DB_DSP_TO_MCU *)&banc_api[R_PAGE_MOT(dsp_api.r_page)];
}

int banc_init(void)
{
    /* Nothing towards the outside: no TRXD socket (an empty replay file
     * bypasses the UDP listener of calypso_bsp_init), no published clock,
     * no recorder, no mailbox monitor. */
    setenv("CALYPSO_BSP_REPLAY_FILE", "/dev/null", 1);
    setenv("CALYPSO_BSP_HORLOGE", "0", 1);
    setenv("CALYPSO_REJEU_ENREG", "0", 1);
    setenv("CALYPSO_MAILBOX", "0", 1);
    /* Pending interrupt taken as soon as INTM drops: C54x behaviour
     * (SPRU131 ch. 6), enabled on both benches by main.c. */
    setenv("CALYPSO_C54X_IRQ_LEVEL", "1", 0);
    /* The A5 coprocessor model realigns the COUNT of SACCH/TF frames when it
     * targets the frame already deposited (calypso_a5.c, CALYPSO_A5_RECALE): a
     * workaround for a bench timing defect. The tester turns it off to measure
     * the ROM as it is (unless the caller has set it). */
    setenv("CALYPSO_A5_RECALE", "0", 0);
    qemu_mutex_init(&calypso_pcb_daram_lock);
    { const char *d = getenv("CALYPSO_DEBUG"), *r = getenv("CALYPSO_C54X_RAPIDE");
      c54x_rapide = (r && *r) ? (*r != '0') : !(d && *d); }
    banc_dsp = c54x_init();
    if (!banc_dsp) { fprintf(stderr, "c54x_init a echoue\n"); return -1; }
    banc_api = &banc_dsp->data[C54X_API_BASE];
    c54x_set_api_ram(banc_dsp, banc_api);       /* BEFORE the ROMs (main.c) */
    static const struct { const char *s; uint32_t a; bool p; } R[] = {
        { "PROM0", 0x07000, true }, { "PROM1", 0x18000, true }, { "PROM2", 0x28000, true },
        { "PROM3", 0x38000, true }, { "DROM", 0x09000, false }, { "PDROM", 0x0E000, false },
        { "PDROM", 0x0E000, true } };
    for (unsigned i = 0; i < sizeof R / sizeof R[0]; i++) {
        char c[512];
        snprintf(c, sizeof c, "%s/calypso_dsp.%s.bin", banc_cfg.rom_dir, R[i].s);
        if (c54x_load_section(banc_dsp, c, R[i].a, R[i].p) < 0) {
            fprintf(stderr, "ROM manquante : %s\n", c);
            return -1;
        }
    }
    { char c[512]; snprintf(c, sizeof c, "%s/calypso_dsp.Registers.bin", banc_cfg.rom_dir);
      c54x_load_registers(banc_dsp, c); }
    c54x_reset(banc_dsp);
    calypso_dma_init();
    calypso_bsp_init(banc_dsp);
    calypso_twl3025_set_afc_dac(banc_cfg.afc_dac);
    dsp_api.ndb = (T_NDB_MCU_DSP *)&banc_api[NDB_MOT];
    dsp_api.param = (T_PARAM_MCU_DSP *)&banc_api[PARAM_MOT];
    dsp_api.r_page = dsp_api.w_page = dsp_api.r_page_used = 0;
    pages();
    banc_fn2temps(&banc_courant, 0);
    banc_fn2temps(&banc_suivant, 0);
    return 0;
}

/* ---- calypso/dsp.c ----------------------------------------------------------- */
static void dsp_audio_init(void)
{
    T_NDB_MCU_DSP *ndb = dsp_api.ndb;
    ndb->d_vbctrl1 = ABB_VAL_T(R_VBCTRL1, 0x00B);
    ndb->d_vbctrl2 = ABB_VAL_T(R_VBCTRL2, 0x000);
    ndb->d_vbuctrl = ABB_VAL_T(R_VBUCTRL, 0x009);
    ndb->d_vbdctrl = ABB_VAL_T(R_VBDCTRL, 0x066);
    ndb->d_toneskb_init = 0;
    ndb->d_toneskb_status = 0;
    ndb->d_shiftul = 0x100;
    ndb->d_shiftdl = 0x100;
    ndb->d_melo_osc_used = 0;
    ndb->d_melo_osc_active = 0;
    ndb->a_melo_note0[0] = ndb->a_melo_note1[0] = ndb->a_melo_note2[0] = ndb->a_melo_note3[0] = 0xfffe;
    ndb->a_melo_note4[0] = ndb->a_melo_note5[0] = ndb->a_melo_note6[0] = ndb->a_melo_note7[0] = 0xfffe;
    dsp_api.param->a_fir31_downlink[0] = 0x4000;
    dsp_api.param->a_fir31_uplink[0] = 0x4000;
    for (int i = 1; i < 31; i++) {
        dsp_api.param->a_fir31_downlink[i] = 0;
        dsp_api.param->a_fir31_uplink[i] = 0;
    }
#define B_GSM_ONLY      ((1L << 13) | (1L << 11))
#define B_BT_CORDLESS   (1L << 12)
#define B_BT_HEADSET    (1L << 14)
#define B_FIR_LOOP      (1L << 1)
    ndb->d_audio_init &= ~(B_FIR_LOOP | B_GSM_ONLY | B_BT_HEADSET | B_BT_CORDLESS);
    ndb->d_audio_init |= B_GSM_ONLY;
    ndb->d_aec_ctrl = 0;
    dsp_api.param->d_gsm_bgd_mgt = 0;
    ndb->d_audio_compressor_ctrl = 0x0401;
    ndb->d_melody_selection = 0;
}

static void dsp_ndb_init(void)
{
    T_NDB_MCU_DSP *ndb = dsp_api.ndb;
#define APCDEL_DOWN (2 + 0)
#define APCDEL_UP   (6 + 3 + 1)
    for (int i = 0; i < 16; i++)
        ndb->a_ramp[i] = ABB_VAL(R_APCRAM, ABB_RAMP_VAL(0, 0));
    ndb->d_debug1 = ABB_VAL_T(0, 0x000);
    ndb->d_afcctladd = ABB_VAL_T(R_AFCCTLADD, 0x000);
    ndb->d_vbuctrl = ABB_VAL_T(R_VBUCTRL, 0x0C9);
    ndb->d_vbdctrl = ABB_VAL_T(R_VBDCTRL, 0x006);
    ndb->d_bbctrl = ABB_VAL_T(R_BBCTRL, 0x2C1);
    ndb->d_bulgcal = ABB_VAL_T(R_BULGCAL, 0x000);
    ndb->d_apcoff = ABB_VAL_T(R_APCOFF, 0x040);
    ndb->d_bulioff = ABB_VAL_T(R_BULIOFF, 0x0FF);
    ndb->d_bulqoff = ABB_VAL_T(R_BULQOFF, 0x0FF);
    ndb->d_dai_onoff = ABB_VAL_T(R_APCOFF, 0x000);
    ndb->d_auxdac = ABB_VAL_T(R_AUXDAC, 0x000);
    ndb->d_vbctrl1 = ABB_VAL_T(R_VBCTRL1, 0x00B);
    ndb->d_vbctrl2 = ABB_VAL_T(R_VBCTRL2, 0x000);
    ndb->d_apcdel1 = ABB_VAL_T(R_APCDEL1, ((APCDEL_DOWN - 2) << 5) | (APCDEL_UP - 6));
    ndb->d_apcdel2 = ABB_VAL_T(R_APCDEL2, 0x000);
    ndb->d_fb_mode = 1;
    ndb->d_fb_det = 0;
    ndb->a_cd[0] = (1 << B_FIRE1);
    ndb->a_dd_0[0] = 0;
    ndb->a_dd_0[2] = 0xffff;
    ndb->a_dd_1[0] = 0;
    ndb->a_dd_1[2] = 0xffff;
    ndb->a_du_0[0] = 0;
    ndb->a_du_0[2] = 0xffff;
    ndb->a_du_1[0] = 0;
    ndb->a_du_1[2] = 0xffff;
    ndb->a_fd[0] = (1 << B_FIRE1);
    ndb->a_fd[2] = 0xffff;
    ndb->d_a5mode = 0;
    ndb->d_tch_mode = 0x0800;
#define GUARD_BITS 8
    ndb->d_tch_mode |= (((GUARD_BITS - 4) & 0x000F) << 7);
    ndb->a_sch26[0] = (1 << B_SCH_CRC);
    ndb->d_spcx_rif = 0x179;
    dsp_audio_init();
}

int banc_dsp_power_on(struct banc_boot *b)
{
    struct banc_boot vide;
    if (!b) b = &vide;
    memset(b, 0, sizeof *b);
    memset(banc_api, 0, 0x2000u * sizeof(uint16_t));        /* dsp_api_memset(API) */

    /* dsp_pre_boot(): the DSP leaves reset and waits in its loader
     * (BL_STATUS_IDLE). Deviation: the firmware first uploads dsp_bootcode
     * there through the 0xe000 mirror; here the ROM's loader runs. */
    long n = 0;
    while (banc_api[BL_STATUS_W] != BL_STATUS_IDLE && n < 8000000) {
        int r = c54x_run(banc_dsp, 256); if (r <= 0) break; n += r;
    }
    b->insn_chargeur = n;
    b->bl_status = banc_api[BL_STATUS_W];

    /* dsp_set_params() */
    T_NDB_MCU_DSP *ndb = dsp_api.ndb;
    ndb->d_background_enable = 0;
    ndb->d_background_abort = 0;
    ndb->d_background_state = 0;
    ndb->d_debug_ptr = 0x0074;
    ndb->d_debug_bk = 0x0001;
    ndb->d_pll_config = 0x154;
    ndb->p_debug_buffer = 0x17ff;
    ndb->d_debug_buffer_size = 7;
    ndb->d_debug_trace_type = 0;
    ndb->d_dsp_state = 3;
    ndb->d_audio_gain_ul = 0;
    ndb->d_audio_gain_dl = 0;
    ndb->d_es_level_api = 0x5213;
    ndb->d_mu_api = 0x5000;
    memcpy(dsp_api.param, &dsp_params, sizeof dsp_params);
    /* dsp_bl_start_at(DSP_START) */
    banc_api[BL_ADDR_HI_W] = 0;
    banc_api[BL_ADDR_LO_W] = DSP_START;
    banc_api[BL_SIZE_W] = 0;
    banc_api[BL_STATUS_W] = BL_CMD_COPY_BLOCK;
    n = 0;
    while (n < 4000000 && banc_dsp->running) {
        int r = c54x_run(banc_dsp, 256); if (r <= 0) break; n += r;
        if (banc_dsp->idle) break;
    }
    b->insn_demarrage = n;
    b->idle = banc_dsp->idle;
    b->version1 = ndb->d_version_number1;
    b->version2 = ndb->d_version_number2;

    dsp_ndb_init();
    /* dsp_db_init() */
    memset(&banc_api[W_PAGE_MOT(0)], 0, sizeof(T_DB_MCU_TO_DSP));
    memset(&banc_api[W_PAGE_MOT(1)], 0, sizeof(T_DB_MCU_TO_DSP));
    memset(&banc_api[R_PAGE_MOT(0)], 0, sizeof(T_DB_DSP_TO_MCU));
    memset(&banc_api[R_PAGE_MOT(1)], 0, sizeof(T_DB_DSP_TO_MCU));
    dsp_api.frame_ctr = 0;
    dsp_api.r_page = dsp_api.w_page = dsp_api.r_page_used = 0;
    /* layer1 init: l1s_reset_hw() */
    l1s_reset_hw();
    g_ul_prec = banc_dsp->data[0x3d9b];
    { static const uint16_t ptrs[6] = { 0x3d91, 0x3d93, 0x3d95, 0x3d97, 0x3d99, 0x3d9b };
      for (int i = 0; i < 6; i++) g_ul_prec_ptr[i] = banc_dsp->data[ptrs[i]]; }
    return (b->bl_status == BL_STATUS_IDLE && b->idle) ? 0 : -1;
}

void l1s_reset_hw(void)
{
    dsp_api.w_page = 0;
    dsp_api.r_page = 0;
    dsp_api.r_page_used = 0;
    pages();
    dsp_api.ndb->d_dsp_page = 0;
}

void dsp_end_scenario(void)
{
    dsp_api.ndb->d_dsp_page = B_GSM_TASK | dsp_api.w_page;
    dsp_api.w_page ^= 1;
    g_irq_armee = 1;            /* tpu_dsp_frameirq_enable(): frame IT at the next frame */
}

void dsp_load_rx_task(uint16_t task, uint8_t burst_id, uint8_t tsc)
{
    dsp_api.db_w->d_task_d = task;
    dsp_api.db_w->d_burst_d = burst_id;
    dsp_api.db_w->d_ctrl_system |= tsc & 0x7;
}

void dsp_load_tx_task(uint16_t task, uint8_t burst_id, uint8_t tsc)
{
    dsp_api.db_w->d_task_u = task;
    dsp_api.db_w->d_burst_u = burst_id;
    dsp_api.db_w->d_ctrl_system |= tsc & 0x7;
}

void dsp_load_tch_param(const struct banc_temps *next_time, uint8_t chan_mode, uint8_t chan_type,
                        uint8_t chan_sub, uint8_t tch_loop, uint8_t sync_tch, uint8_t tn)
{
    uint16_t d_ctrl_tch, fn;
    d_ctrl_tch = (chan_mode << B_CHAN_MODE) | (chan_type << B_CHAN_TYPE) | (chan_sub << B_SUBCHANNEL) |
                 (sync_tch << B_SYNC_TCH_UL) | (sync_tch << B_SYNC_TCH_DL) | (tch_loop << B_TCH_LOOP);
    if (chan_type == TCH_F)
        fn = (uint16_t)(((next_time->fn - (tn * 13) + 104) % 104) | ((next_time->fn % 104) << 8));
    else if (chan_type == TCH_H) {
        uint8_t tn_report = (tn & ~1) | chan_sub;
        fn = (uint16_t)(((next_time->fn - (tn_report * 13) + 104) % 104) | ((next_time->fn % 104) << 8));
    } else
        fn = 0;
    dsp_api.db_w->d_fn = fn;
    dsp_api.db_w->a_a5fn[0] = (uint16_t)(((uint16_t)next_time->t3 << 5) | next_time->t2);
    dsp_api.db_w->a_a5fn[1] = next_time->t1;
    dsp_api.db_w->d_ctrl_tch = d_ctrl_tch;
}

void dsp_load_ciph_param(int mode, const uint8_t *key)
{
    dsp_api.ndb->d_a5mode = mode;
    if (!mode || !key)
        return;
    dsp_api.ndb->a_kc[0] = (uint16_t)key[7] | ((uint16_t)key[6] << 8);
    dsp_api.ndb->a_kc[1] = (uint16_t)key[5] | ((uint16_t)key[4] << 8);
    dsp_api.ndb->a_kc[2] = (uint16_t)key[3] | ((uint16_t)key[2] << 8);
    dsp_api.ndb->a_kc[3] = (uint16_t)key[1] | ((uint16_t)key[0] << 8);
}

void dsp_memcpy_to_api(volatile uint16_t *dsp_buf, const uint8_t *mcu_buf, int n, int be)
{
    int odd = n & 1;
    n >>= 1;
    for (int i = 0; i < n; i++) {
        uint16_t w;
        if (be) { w = (uint16_t)(*(mcu_buf++) << 8); w |= *(mcu_buf++); }
        else    { w = *(mcu_buf++); w |= (uint16_t)(*(mcu_buf++) << 8); }
        *(dsp_buf++) = w;
    }
    if (odd)
        *dsp_buf = be ? (uint16_t)(*mcu_buf << 8) : *mcu_buf;
}

void dsp_memcpy_from_api(uint8_t *mcu_buf, const volatile uint16_t *dsp_buf, int n, int be)
{
    int odd = n & 1;
    n >>= 1;
    for (int i = 0; i < n; i++) {
        uint16_t w = *(dsp_buf++);
        if (be) { *(mcu_buf++) = w >> 8; *(mcu_buf++) = (uint8_t)w; }
        else    { *(mcu_buf++) = (uint8_t)w; *(mcu_buf++) = w >> 8; }
    }
    if (odd)
        *mcu_buf = be ? (uint8_t)(*dsp_buf >> 8) : (uint8_t)*dsp_buf;
}

/* ---- layer1/tdma_sched.c ------------------------------------------------------ */
#define TDMASCHED_NUM_FRAMES 25
#define TDMASCHED_NUM_CB     8
static struct { struct tdma_sched_item item[TDMASCHED_NUM_CB]; uint8_t num_items; } g_bucket[TDMASCHED_NUM_FRAMES];
static uint8_t g_cur_bucket;

int tdma_end_set(uint8_t p1, uint8_t p2, uint16_t p3) { (void)p1; (void)p2; (void)p3; return 0; }
static uint8_t wrap_bucket(uint8_t off) { return (uint8_t)((g_cur_bucket + off) % TDMASCHED_NUM_FRAMES); }

int tdma_schedule(uint8_t frame_offset, tdma_sched_cb *cb, uint8_t p1, uint8_t p2, uint16_t p3, int16_t prio)
{
    uint8_t nr = wrap_bucket(frame_offset);
    if (g_bucket[nr].num_items >= TDMASCHED_NUM_CB) { printf("tdma_schedule bucket overflow\n"); return -1; }
    struct tdma_sched_item *it = &g_bucket[nr].item[g_bucket[nr].num_items++];
    it->cb = cb; it->p1 = p1; it->p2 = p2; it->p3 = p3; it->prio = prio; it->flags = 0;
    return 0;
}

int tdma_schedule_set(uint8_t frame_offset, const struct tdma_sched_item *set, uint16_t p3)
{
    uint8_t nr = wrap_bucket(frame_offset);
    int j = 0;
    for (int i = 0; ; i++) {
        const struct tdma_sched_item *si = &set[i];
        if (si->cb == &tdma_end_set) break;
        if (si->cb == NULL) { nr = wrap_bucket(++frame_offset); j++; continue; }
        if (g_bucket[nr].num_items >= TDMASCHED_NUM_CB) { printf("tdma_schedule bucket overflow\n"); return -1; }
        g_bucket[nr].item[g_bucket[nr].num_items] = *si;
        g_bucket[nr].item[g_bucket[nr].num_items].p3 = p3;
        g_bucket[nr].num_items++;
    }
    return j;
}

static uint16_t tdma_sched_flag_scan(void)
{
    uint16_t f = 0;
    for (int i = 0; i < g_bucket[g_cur_bucket].num_items; i++) f |= g_bucket[g_cur_bucket].item[i].flags;
    return f;
}

static void tdma_sched_execute(void)
{
    int seq[TDMASCHED_NUM_CB];
    __typeof__(g_bucket[0]) *b = &g_bucket[g_cur_bucket];
    for (int i = 0; i < TDMASCHED_NUM_CB; i++) seq[i] = i;
    for (int i = 0; i < b->num_items; i++) {          /* _tdma_sched_bucket_sort, identical */
        struct tdma_sched_item *ii = &b->item[seq[i]];
        for (int j = i + 1; j < b->num_items; j++) {
            struct tdma_sched_item *jj = &b->item[seq[j]];
            if (ii->prio > jj->prio) { ii = jj; int k = seq[i]; seq[i] = seq[j]; seq[j] = k; }
        }
    }
    for (int i = 0; i < b->num_items; i++) {
        struct tdma_sched_item *it = &b->item[seq[i]];
        if (it->cb(it->p1, it->p2, it->p3) < 0) break;
    }
    b->num_items = 0;
}

void tdma_sched_reset(void)
{
    for (int i = 0; i < TDMASCHED_NUM_FRAMES; i++)
        if (i != g_cur_bucket) g_bucket[i].num_items = 0;
}

/* ---- layer1/mframe_sched.c (SCHEDULE_AHEAD 2, SCHEDULE_LATENCY 1) -------------- */
static struct banc_mframe_item g_mf[32];
static int g_n_mf;
void banc_mframe_ajouter(const struct tdma_sched_item *set, uint16_t modulo, uint16_t frame_nr, uint16_t p3)
{
    if (g_n_mf < 32) g_mf[g_n_mf++] = (struct banc_mframe_item){ set, modulo, frame_nr, p3 };
}
void banc_mframe_vider(void) { g_n_mf = 0; }
static void mframe_schedule(void)
{
    for (int i = 0; i < g_n_mf; i++) {
        unsigned trigger = g_mf[i].frame_nr % g_mf[i].modulo;
        unsigned current = (banc_courant.fn + 2) % g_mf[i].modulo;
        if (current == trigger) tdma_schedule_set(1, g_mf[i].set, g_mf[i].p3);
    }
}

/* ---- layer1/sync.c l1_sync() ------------------------------------------------- */
static void l1_sync(void)
{
    banc_courant = banc_suivant;
    banc_fn2temps(&banc_suivant, banc_courant.fn + 1);
    dsp_api.frame_ctr++;
    dsp_api.r_page_used = 0;
    pages();
    memset((void *)dsp_api.db_w, 0, sizeof(*dsp_api.db_w));
    /* afc_load_dsp() */
    dsp_api.db_w->d_afc = (uint16_t)banc_cfg.afc_dac;
    dsp_api.db_w->d_ctrl_abb |= (1 << B_AFC);
    if (dsp_api.ndb->d_error_status) {
        banc_stats.erreurs_dsp++;
        banc_stats.derniere_erreur = dsp_api.ndb->d_error_status;
        if (banc_cfg.verbeux) printf("    [l1s] fn=%u DSP Error Status: %u\n", banc_courant.fn, dsp_api.ndb->d_error_status);
        dsp_api.ndb->d_error_status = 0;
    }
    uint16_t flags = tdma_sched_flag_scan();
    tdma_sched_execute();
    if (banc_crochet_l1s) flags |= (uint16_t)banc_crochet_l1s(banc_courant.fn);
    if (dsp_api.r_page_used) {
        memset((void *)dsp_api.db_r, 0, sizeof(*dsp_api.db_r));
        dsp_api.db_r->a_sch[0] = (1 << B_SCH_CRC);
        dsp_api.r_page ^= 1;
    }
    if (flags & TDMA_IFLG_DSP)
        dsp_end_scenario();
    mframe_schedule();
    g_cur_bucket = wrap_bucket(1);       /* tdma_sched_advance() */
}

/* ---- the radio channel --------------------------------------------------------- */
const uint8_t banc_tsc[8][26] = {      /* 45.002 table 5.2.3a */
    { 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1 },
    { 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1 },
    { 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0 },
    { 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0 },
    { 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1 },
    { 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0 },
    { 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1 },
    { 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0 },
};
const uint8_t banc_train_sb[64] = {
    1,0,1,1,1,0,0,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,1,1,1,
    0,0,1,0,1,1,0,1,0,1,0,0,0,1,0,1,0,1,1,1,0,1,1,0,0,0,0,1,1,0,1,1,
};
const uint8_t banc_factice[148] = {    /* 45.002 5.2.6 */
    0,0,0,
    1,1,1,1,1,0,1,1,0,1,1,1,0,1,1,0,0,0,0,0,1,0,1,0,0,1,0,0,1,1,1,0,
    0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,1,1,1,0,0,
    0,1,0,1,1,1,0,0,0,1,0,1,1,1,0,0,0,1,0,1,0,1,1,1,0,1,0,0,1,0,1,0,
    0,0,1,1,0,0,1,1,0,0,1,1,1,0,0,1,1,1,1,0,1,0,0,1,1,1,1,1,0,0,0,1,
    0,0,1,0,1,1,1,1,1,0,1,0,1,0,
    0,0,0,
};

void banc_burst_nb(const uint8_t *e116, uint8_t tsc, uint8_t bits[148])
{
    memset(bits, 0, 3);
    memcpy(bits + 3, e116, 58);
    memcpy(bits + 61, banc_tsc[tsc & 7], 26);
    memcpy(bits + 87, e116 + 58, 58);
    memset(bits + 145, 0, 3);
}

void banc_sb_info(uint32_t fn, uint8_t bsic, uint8_t sb_info[4])
{
    uint32_t t1 = fn / 1326, t2 = fn % 26, t3 = fn % 51;
    uint32_t t3p = t3 ? (t3 - 1) / 10 : 0;
    sb_info[0] = (uint8_t)(((bsic & 0x3f) << 2) | ((t1 & 0x600) >> 9));
    sb_info[1] = (uint8_t)((t1 & 0x1fe) >> 1);
    sb_info[2] = (uint8_t)(((t1 & 0x001) << 7) | ((t2 & 0x1f) << 2) | ((t3p & 0x6) >> 1));
    sb_info[3] = (uint8_t)(t3p & 0x1);
}

void banc_burst_sch(uint32_t fn, uint8_t bsic, uint8_t bits[148])
{
    uint8_t sb_info[4];
    ubit_t code[78];
    banc_sb_info(fn, bsic, sb_info);
    gsm0503_sch_encode(code, sb_info);
    memset(bits, 0, 3);
    memcpy(bits + 3, code, 39);
    memcpy(bits + 42, banc_train_sb, 64);
    memcpy(bits + 106, code + 39, 39);
    memset(bits + 145, 0, 3);
}

char banc_cellule(uint32_t fn, uint8_t bits[148], int *amp)
{
    (void)amp;
    uint32_t p51 = fn % 51;
    if (p51 % 10 == 0 && p51 <= 40) { memset(bits, 0, 148); return 'F'; }
    if (p51 % 10 == 1 && p51 <= 41) { banc_burst_sch(fn, banc_bsic, bits); return 'S'; }
    static const int debuts[] = { 2, 6, 12, 16, 22, 26, 32, 36, 42, 46 };
    for (unsigned i = 0; i < sizeof debuts / sizeof debuts[0]; i++) {
        int d = debuts[i];
        if ((int)p51 < d || (int)p51 >= d + 4) continue;
        uint32_t fn0 = fn - (p51 - (uint32_t)d);
        const uint8_t *l2 = banc_cellule_nb ? banc_cellule_nb(fn0, d) : NULL;
        if (!l2) break;
        static ubit_t e[4 * 116];
        gsm0503_xcch_encode(e, l2);
        banc_burst_nb(e + 116 * (p51 - (uint32_t)d), banc_bsic & 7, bits);
        return 'N';
    }
    memcpy(bits, banc_factice, 148);
    return 'D';
}

/* SCH: the BSP waveform (calypso_bsp.c sb_moduler, default settings:
 * spread 1.0, instant 0.35, Gaussian noise sigma 3000 seeded by the fn,
 * shape A). Copied because it is static in calypso_bsp.c. */
static int g_sb_propre = -1;
static void sb_moduler(const uint8_t *bits, int16_t *dst, uint32_t fn, int amp)
{
    if (g_sb_propre < 0) { const char *e = getenv("BANC_SB_PROPRE"); g_sb_propre = (e && *e == '1'); }
    if (g_sb_propre) {                                    /* bare GMSK, like a normal burst */
        gmsk_moduler(bits, 148, amp, 0.0, 0.5, dst);
        gmsk_elargir(dst, 148, 0.3);
        return;
    }
    { static double dsb = -9; if (dsb < -1) { const char *e = getenv("BANC_SB_DEC"); dsb = e ? atof(e) : 0.35; }
      gmsk_moduler(bits, 148, amp, 0.0, dsb, dst); }
    gmsk_elargir(dst, 148, 1.0);
    uint32_t seed = fn * 2654435761u + 777u;
    for (int k = 0; k < 296; k++) {
        seed = seed * 1103515245u + 12345u; double u1 = ((seed >> 8) & 0xffff) / 65536.0 + 1e-6;
        seed = seed * 1103515245u + 12345u; double u2 = ((seed >> 8) & 0xffff) / 65536.0;
        double g = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
        double v = dst[k] + 3000.0 * g;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        dst[k] = (int16_t)lrint(v);
    }
}

static struct { uint32_t fn, tick; uint8_t bits[148]; int amp; bool valide, livre_sb; } g_sch;

/* bsp_ts0_livrer(): framing of the burst according to the window the ROM armed. */
static void deposer(uint32_t fn)
{
    static int16_t iq[2 * 512], iq2[2 * 148];
    uint8_t bits[148];
    int amp = 0;
    char type = banc_source ? banc_source(fn, bits, &amp) : 0;
    if (amp <= 0) amp = 30000;
    const bool one_shot = calypso_rhea_dma_one_shot();
    const int nwin = one_shot ? calypso_rhea_dma_get_len_words() / 2 : 0;
    int marge = nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0;
    /* BANC_FB_DECAL=k: in FB search (continuous DMA), the FCCH is deposited k
     * samples after the window opens (TOA measurement = f(k)). */
    static int fb_decal = -1;
    if (fb_decal < 0) { const char *e = getenv("BANC_FB_DECAL"); fb_decal = e ? atoi(e) : 0; }
    if (!one_shot && type == 'F') marge = fb_decal;
    /* BANC_SB_DECAL=k: in the SB window (one-shot), the SCH is deposited k samples
     * after the window opens (default 21). */
    static int sb_decal = -2;
    if (sb_decal == -2) { const char *e = getenv("BANC_SB_DECAL"); sb_decal = e ? atoi(e) : -1; }
    if (type == 'S' && one_shot && nwin >= 190 && sb_decal >= 0) marge = sb_decal;
    static int sb_neg = 0; { const char *e = getenv("BANC_SB_NEG"); sb_neg = e ? atoi(e) : 0; }
    if (type == 'S' && one_shot && nwin >= 190 && sb_neg > 0) marge = 0;
    /* BANC_NB_DECAL=k: normal burst (one-shot window of 150..189 samples), default 3. */
    static int nb_decal = -2;
    if (nb_decal == -2) { const char *e = getenv("BANC_NB_DECAL"); nb_decal = e ? atoi(e) : -1; }
    if (type != 'S' && type != 'F' && type && one_shot && nwin >= 150 && nwin < 190 && nb_decal >= 0) marge = nb_decal;
    memset(iq, 0, sizeof iq);
    if (type == 'S') {
        g_sch.fn = fn; g_sch.tick = fn; g_sch.amp = amp;
        memcpy(g_sch.bits, bits, 148);
        g_sch.livre_sb = one_shot && nwin >= 190;
        g_sch.valide = true;
        if (g_sch.livre_sb) banc_stats.depots_fenetre_sb++;
        { static int cel = -1; if (cel < 0) cel = getenv("BANC_SB_CELLULE") ? 1 : 0;
          extern char cellule_burst(uint32_t, uint8_t, int, double, int, int16_t *, int *) __attribute__((weak));
          if (cel && cellule_burst) {   /* same waveform as the bridge: src/cellule.c cellule_burst() */
              static int16_t tmp[2 * 512]; int n_iq = 0;
              const char *ed = getenv("BANC_SB_DEC");
              cellule_burst(fn, banc_bsic, amp, ed ? atof(ed) : 0.5, marge, tmp, &n_iq);
              memcpy(iq, tmp, (size_t)(n_iq < 2 * 256 ? n_iq : 2 * 256) * sizeof(int16_t));
          } else {
              sb_moduler(bits, iq + 2 * marge, fn, amp);
              if (sb_neg > 0) { memmove(iq, iq + 2 * sb_neg, (size_t)(2 * (512 - sb_neg)) * sizeof(int16_t)); marge = 1; }
          } }
    } else if (type) {
        /* BANC_NB_NEG=n: normal burst n samples early (start cut off) */
        static int nb_neg = -1; if (nb_neg < 0) { const char *e = getenv("BANC_NB_NEG"); nb_neg = e ? atoi(e) : 0; }
        const bool nb_win = type != 'F' && one_shot && nwin >= 150 && nwin < 190;
        if (nb_neg > 0 && nb_win) marge = 0;
        gmsk_moduler(bits, 148, amp, 0.0, 0.5, iq + 2 * marge);
        if (type != 'F') gmsk_elargir(iq + 2 * marge, 148, 0.3);
        if (nb_neg > 0 && nb_win) { memmove(iq, iq + 2 * nb_neg, (size_t)(2 * (512 - nb_neg)) * sizeof(int16_t)); marge = 1; }
    }
    if (type == 'S' && getenv("BANC_NWIN_DBG")) { static int n; if (n++ < 3) { FILE *f = fopen(getenv("BANC_NWIN_DBG"), "a"); if (f) { fprintf(f, "[nwin] SB one_shot=%d nwin=%d marge=%d len_words=%d\n", one_shot, nwin, marge, calypso_rhea_dma_get_len_words()); fclose(f); } } }
    int total = marge > 0 ? (nwin > marge + 148 ? nwin : marge + 148) : 148;
    if (total > 256) total = 256;
    calypso_bsp_rx_burst(0, fn, iq, 2 * total);
    banc_stats.depots++;
    if (!one_shot) {             /* continuous DMA (FB search): the whole frame */
        for (unsigned tn = 1; tn < 8; tn++) {
            const uint8_t *b = banc_source_autre ? banc_source_autre(fn, tn) : NULL;
            if (!b) b = banc_factice;
            memset(iq2, 0, sizeof iq2);
            gmsk_moduler(b, 148, 30000, 0.0, 0.5, iq2);
            gmsk_elargir(iq2, 148, 0.3);
            calypso_bsp_rx_burst((uint8_t)tn, fn, iq2, 2 * 148);
        }
    }
}

/* calypso_bsp_sb_retenter(): an SCH deposited before the SB window is armed
 * is delivered again in the following one-shot window, at most one tick later. */
static void sb_retenter(uint32_t tick)
{
    static int16_t iq[2 * 512];
    if (!g_sch.valide || g_sch.livre_sb) return;
    if ((int32_t)(tick - g_sch.tick) > 1) { g_sch.valide = false; return; }
    if (!calypso_rhea_dma_rx_armed() || !calypso_rhea_dma_one_shot()) return;
    int total = calypso_rhea_dma_get_len_words() / 2;
    if (total < 190) return;
    if (total > 512) total = 512;
    int marge = 21;
    { const char *e = getenv("BANC_SB_DECAL"); if (e && atoi(e) >= 0) marge = atoi(e); }
    if (marge > total - 148) marge = total - 148;
    memset(iq, 0, sizeof iq);
    sb_moduler(g_sch.bits, iq + 2 * marge, g_sch.fn, g_sch.amp);
    calypso_rif_flush();
    calypso_bsp_rx_burst(0, g_sch.fn, iq, 2 * total);
    g_sch.livre_sb = true;
    banc_stats.depots_fenetre_sb++;
}

/* ---- one frame (src/pont.c: jouer_trame in two phases + servir) ---------------- */
uint32_t banc_fn(void) { return banc_suivant.fn; }
void banc_set_fn(uint32_t fn) { banc_fn2temps(&banc_suivant, fn); banc_fn2temps(&banc_courant, fn ? fn - 1 : 0); }

void banc_trame(void)
{
    C54xState *d = banc_dsp;
    const uint32_t fn = banc_suivant.fn;    /* this frame's l1_sync will have current_time = fn */
    const long budget = banc_cfg.budget;
    uint32_t i0 = d->insn_count;
    long fait = 0;
    g_c54x_exe_fn = fn;
    banc_stats.trames++;

    /* phase A: frame IT (if the ARM closed a scenario), the ROM reads the W
     * page and arms its RX window */
    calypso_dma_tick(d);
    reveil();
    if (g_irq_armee && (d->imr & (1u << C54X_IT_TPU_FRAME_BIT))) {
        c54x_interrupt_ex(d, C54X_IT_TPU_FRAME_VEC, C54X_IT_TPU_FRAME_BIT);
        banc_stats.irq_trame++;
    }
    g_irq_armee = 0;
    if (d->idle && (d->ifr & d->imr) && !(d->st1 & 0x800)) d->idle = false;
    if (!d->idle) fait = banc_executer(budget / 8);
    while (!d->idle && d->running && (banc_sur_tch || !calypso_rhea_dma_rx_armed()) && fait < budget / 2)
        fait += banc_executer(256);

    /* the ARM: l1_sync() of frame fn; the DSP will read its page at the next frame */
    l1_sync();
    calypso_twl3025_set_afc_dac(banc_cfg.afc_dac);   /* relay of d_afc (pont.c, TICK) */

    /* phase B: the frame's burst, then the rest of the budget */
    deposer(fn);
    reveil();
    if (!d->idle && budget - fait > 0) banc_executer(budget - fait);
    for (int k = 0; k < 40 && d->running; k++) {
        sb_retenter(fn);
        if (!calypso_rhea_dma_pump(d)) break;
        if (d->idle && (d->ifr & d->imr) && !(d->st1 & 0x800)) d->idle = false;
        if (!d->idle) banc_executer(budget / 4);
    }
    if (!d->idle) banc_stats.pas_idle_fin++;
    long n = (long)(d->insn_count - i0);
    if (n > banc_stats.insn_max) banc_stats.insn_max = n;
    banc_ul_capturer(fn);
    if (banc_crochet_fin) banc_crochet_fin(fn);
}

void banc_courir(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) banc_trame();
}

/* ---- uplink bursts of the ROM (pont.c tx_rom_publier) ---------------------------- */
static void banc_tsp_capturer(uint32_t fn)
{
    const uint16_t *m = banc_dsp->data;
    for (unsigned a = 0x3cbb; a < 0x3d00 - 18; a++) {
        if (m[a] != 0x1c0a) continue;
        const uint16_t *w = &m[a + 2];
        int ok = 1;
        for (int i = 0; i < 16; i++) if ((w[i] & 0x3e) != 0x06 && (w[i] & 0x3e) != 0x04) { ok = 0; break; }
        if (!ok) continue;
        if (!memcmp(w, g_tsp_prec, sizeof g_tsp_prec)) return;
        memcpy(g_tsp_prec, w, sizeof g_tsp_prec);
        struct banc_tsp_burst *t = &banc_tsp[banc_n_tsp % BANC_UL_MAX];
        t->fn = fn; memcpy(t->mots, w, sizeof t->mots);
        banc_n_tsp++;
        return;
    }
}

void banc_ul_capturer(uint32_t fn)
{
    banc_tsp_capturer(fn);
    const uint16_t *m = banc_dsp->data;
    static const uint16_t ptrs[6] = { 0x3d91, 0x3d93, 0x3d95, 0x3d97, 0x3d99, 0x3d9b };
    int bouge = 0;
    for (int i = 0; i < 6; i++) if (m[ptrs[i]] != g_ul_prec_ptr[i]) { g_ul_prec_ptr[i] = m[ptrs[i]]; bouge = 1; }
    if (!bouge) return;
    uint16_t p = m[0x3d9b];
    g_ul_prec = p;
    struct banc_ul_burst *u = &banc_ul[banc_n_ul % BANC_UL_MAX];
    u->fn = fn;
    u->ptr = p;
    u->mode = m[0x3fac];
    for (int i = 0; i < 8; i++) u->raw[i] = m[0x3f8a + i];
    u->rang = (u->mode == 3 && p >= 0x4280 && p < 0x42a0 && (p - 0x4280) % 8 == 0) ? (int)(((p - 0x4280) / 8 + 3) & 3) : -1;
    for (int i = 0; i < 116; i++) {
        /* 57 data, hl, hu, 57 data; hl/hu = the 2 LOW bits of word 7 (k = 126, 127), like
         * pont.c tx_rom_publier. */
        int k = i < 57 ? i : (i < 59 ? 126 + (i - 57) : i - 2);
        u->bits[i] = (m[0x3f8a + k / 16] >> (15 - k % 16)) & 1;
        u->flux[i] = (m[0x3f9b + k / 16] >> (15 - k % 16)) & 1;
    }
    banc_n_ul++;
}

void banc_trame_vide(void)
{
    banc_source_fn s = banc_source;
    banc_source = NULL;
    banc_trame();
    banc_source = s;
}
