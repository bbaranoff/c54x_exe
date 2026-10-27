/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * montant.c - uplink side of the DSP setup: RACH, SDCCH, SACCH, FACCH, speech.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * Under CALYPSO_DSP_EXTERN=1, calypso_l1_do_init() calls calypso_l1_disable():
 * no layer 1 is registered in QEMU, so calypso_l1_do_rach_written() and
 * calypso_l1_do_page_written() (calypso_l1_dispatch.c) are no-ops. In the
 * grgsm setup those were the functions publishing the uplink into the
 * /dev/shm side-bands that pont.py consumes (pont/uplink.py):
 *
 *   /dev/shm/calypso_rach           RACH        (ra, bsic)
 *   /dev/shm/calypso_sdcch_ul       SDCCH UL    (23-byte L2 block)
 *   /dev/shm/calypso_tch_facch_ul   FACCH UL
 *   /dev/shm/calypso_tch_sacch_ul   SACCH UL
 *   /dev/shm/calypso_tch_ul         speech (ring of FR frames)
 *
 * Without the RACH there is no IMM ASS, hence no SDCCH and never a LOCATION
 * UPDATING ACCEPT.
 *
 * HOW
 * ---
 * This process holds the shared API RAM, so the captures of
 * qosmo-grgsm/hw/arm/calypso/calypso_l1_grgsm.c are reproduced as is, but
 * triggered by POLLING once per frame instead of QEMU's write callbacks. The
 * call site (pont.c, end of PONT_TICK) matches the end of the ARM scenario:
 * dsp_end_scenario() has just written d_dsp_page = B_GSM_TASK | page, and the
 * task words of that W page are current.
 *
 * The RACH is the only case where polling is not equivalent to a callback:
 * the firmware writes d_rach (prim_rach.c:72) then d_task_ra, and nobody
 * clears them afterwards (sync.c:307 only zeroes them on ABORT). Triggering
 * is therefore EDGE based: first non-zero value, or a value different from
 * the previous one. Accepted blind spot: two consecutive attempts with the
 * same (RA, BSIC), about 1/256 since gsm48_rr draws the RA at random; the
 * next attempt goes through. MONTANT_CONSOMME_RACH=1 zeroes d_rach after
 * publication, which makes the trigger exact at the cost of a write into the
 * API window the ROM might read.
 *
 * This module does not speak TRXD: calypso_bsp_send_rach_ra() exists (dead
 * code) but would send the access burst to 127.0.0.1:5702, the DOWNLINK
 * socket of pont.py, whose run_data() does "self.bts_data = addr" on every
 * packet received: the burst would be read back as downlink and the BTS
 * address overwritten. The side-band path goes through the uplink machinery
 * of pont.py, the one already proven in the grgsm setup.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "calypso_bsp.h"
#include "montant.h"
#include <osmocom/core/bits.h>
#include <osmocom/coding/gsm0503_coding.h>   /* downlink FACCH prosthesis */

#define SHM_RACH        "/dev/shm/calypso_rach"
#define SHM_SDCCH_UL    "/dev/shm/calypso_sdcch_ul"
#define SHM_FACCH_UL    "/dev/shm/calypso_tch_facch_ul"
#define SHM_SACCH_UL    "/dev/shm/calypso_tch_sacch_ul"
#define SHM_TCH_UL      "/dev/shm/calypso_tch_ul"
#define SHM_KC          "/dev/shm/calypso_kc_l1"

/* Sizes and layouts: pont/uplink.py. */
#define REC_RACH        16      /* 12 read: seq(4) ra(1) bsic(1) ..(2) fn(4)   */
#define REC_L2          48      /* 39 read: seq(4) l1s(4) fn(4) task(2) . p51(1) . l2(23) */
#define TCH_UL_SLOTS    16
#define TCH_UL_SLOT_SZ  64      /* TCH_UL_SLOT       */
#define TCH_UL_FR_OFS   16      /* TCH_UL_FR_OFS     */
#define FR_BYTES        33

/* Search window for the L2 header in a_cu (SDCCH_UL_WINDOW_OFS of
 * calypso_l1_grgsm.c). */
#define SDCCH_UL_WINDOW_OFS  6

/* Uplink SDCCH duplicate filter (heuristic path).
 *
 * The firmware leaves its block in a_cu; republishing it would make the bridge
 * resend it and the BSC would see a second SABM on an already established
 * link ("SABM frame with information not allowed in this state"), dropping
 * the channel in the middle of the procedure.
 *
 * So a block is published ONLY when its content changes. MONTANT_SDCCH_REPETE
 * = N re-enables republication of the same block after N frames (0 = never,
 * the default). A LAPDm retransmission by the mobile carries the same bytes
 * and is therefore swallowed; this is the accepted trade-off. */
#define SDCCH_UL_REPETE_DEFAUT 0

/* Minimum number of frames between two published RACHs: one attempt by the
 * mobile spans several frames and the same d_rach stays readable meanwhile. */
#define RACH_GARDE_TRAMES 4

static struct {
    unsigned long rach, sdcch, facch, sacch, parole;
    uint16_t prev_rach;
    uint32_t fn_rach;
    bool     rach_vu;      /* at least one RACH published          */
    bool     base_rach;    /* the reference value has been taken   */
    /* Last uplink SDCCH block published, for the duplicate filter. */
    uint8_t  sdcch_dernier[23];
    uint32_t sdcch_trame;
    bool     sdcch_a_dernier;
    bool     blud_vu;          /* a_cu has already announced a block via B_BLUD */
    unsigned sans_blud;        /* uplink tasks seen without B_BLUD         */
    bool     dedie_arme;
    /* SDCCH <-> TCH switch followed from the firmware tasks (see
     * suivre_tache_tch). The last SDCCH read from calypso_dcch_cfg is kept to
     * return to it; the TCH announcement comes from the bridge
     * (calypso_tch_cfg). */
    int      sd_tn, sd_genre, sd_ss;
    bool     sd_valide;        /* an SDCCH is known (dcch_cfg armed)       */
    int      tch_tn, tch_tsc;  /* bridge announcement: 0 = no TCH announced */
    bool     sur_tch;          /* the BSP plays the TCH timeslot           */
} g;
static int g_tn_tch_sb;   /* TCH TS seen on the side-band (0: TS0 / no TCH) */

/* Read tasks the ARM posts in d_task_d of the W page (osmocom-bb firmware,
 * include/calypso/l1_environment.h:45-52). Reading an SDCCH/SACCH block goes
 * through ALLC (prim_rx_nb.c:200), like the BCCH and the CCCH; DDL/ADL are
 * kept as a precaution. */
#ifndef DDL_DSP_TASK
#define DDL_DSP_TASK 26
#endif
#ifndef ADL_DSP_TASK
#define ADL_DSP_TASK 27
#endif
/* d_tch_mode: fourth word of the NDB (dsp_api.h:121, rejouer.c:623). */
#define NDB_D_TCH_MODE 0x006u

static int journal(void)
{
    static int n = -1;
    if (n < 0) {
        const char *e = calypso_getenv("MONTANT_DEBUG");
        n = (e && *e) ? atoi(e) : 20;   /* the first 20 events, by default */
    }
    return n;
}

static int sb_ouvrir(const char *chemin, off_t taille)
{
    int fd = open(chemin, O_CREAT | O_RDWR, 0644);
    if (fd >= 0 && ftruncate(fd, taille) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void sb_ecrire(int fd, const void *buf, size_t n, off_t off)
{
    if (fd >= 0 && pwrite(fd, buf, n, off) < 0) {
        return;
    }
}

static void publier_l2(int *fdp, const char *chemin, uint32_t *seq,
                       const uint8_t *l2, uint16_t task_u, uint32_t fn)
{
    if (*fdp == -2) {
        *fdp = sb_ouvrir(chemin, REC_L2);
    }
    uint8_t buf[REC_L2];
    memset(buf, 0, sizeof(buf));
    (*seq)++;
    memcpy(buf + 0, seq, 4);
    memcpy(buf + 4, &fn, 4);
    memcpy(buf + 8, &fn, 4);
    memcpy(buf + 12, &task_u, 2);
    buf[14] = (uint8_t)(fn % 51u);
    memcpy(buf + 16, l2, 23);
    sb_ecrire(*fdp, buf, sizeof(buf), 0);
}

/* UPLINK SPEECH RECORDING. Every uplink frame is logged here, raw TI (type 1)
 * and converted to FR (type 0), in the same 48-byte record format as noter_dl;
 * tools/decoder_add.py decodes them with libgsm and looks for the test tone.
 * The file is reset at each call (more than 500 frames without a record),
 * 16000 records at most. */
static void noter_ul(uint32_t fn, uint8_t type, uint16_t etat, const uint8_t *data, int n)
{
    static FILE *f;
    static unsigned nrec;
    static uint32_t fn_prec;
    if (!f || (uint32_t)(fn - fn_prec) > 500u) {
        if (f) fclose(f);
        f = fopen("/dev/shm/calypso_add_ul.bin", "wb");
        nrec = 0;
    }
    fn_prec = fn;
    if (!f || nrec >= 16000) return;
    uint8_t r[48];
    memset(r, 0, sizeof r);
    memcpy(r, &fn, 4);
    r[4] = type; r[5] = (uint8_t)n;
    memcpy(r + 6, &etat, 2);
    memcpy(r + 12, data, n > 36 ? 36 : n);
    fwrite(r, 1, sizeof r, f);
    if ((++nrec % 32) == 0) fflush(f);
}
static void publier_parole(const uint8_t *fr, uint32_t fn)
{
    static int fd = -2;
    static uint32_t seq;
    if (fd == -2) {
        fd = sb_ouvrir(SHM_TCH_UL, 8 + TCH_UL_SLOTS * TCH_UL_SLOT_SZ);
        uint32_t hdr[2] = { 0, TCH_UL_SLOTS };
        sb_ecrire(fd, hdr, sizeof(hdr), 0);
    }
    uint8_t buf[TCH_UL_SLOT_SZ];
    memset(buf, 0, sizeof(buf));
    seq++;
    memcpy(buf + 0, &seq, 4);
    memcpy(buf + 4, &fn, 4);
    memcpy(buf + 8, &fn, 4);
    memcpy(buf + TCH_UL_FR_OFS, fr, FR_BYTES);
    /* Header (the write counter) last: pont.py reads the header first, then
     * the slot; the other order would hand it a half-written slot. */
    sb_ecrire(fd, buf, sizeof(buf), 8 + (off_t)((seq - 1) % TCH_UL_SLOTS) * TCH_UL_SLOT_SZ);
    sb_ecrire(fd, &seq, 4, 0);
}

/* Uplink block deposited by the firmware in the NDB: word 0 = header (B_BLUD
 * flags "block ready"), data from word 3 on. The 33 speech bytes are stored
 * high byte first, the 23 L2 bytes low byte first.
 *
 * B_BLUD BELONGS TO THE ROM, NOT TO THE HOST. For a_cu (SDCCH, SACCH) the ROM
 * encodes the block as soon as the DUL/AUL task is posted and the flag drops
 * at the next tick (posted at T, cleared by the ROM at T+1, bursts from T+1 to
 * T+4). For a_fu (FACCH) and a_du (speech) the TCHT task uses the flag to
 * choose between FACCH and speech, and the ROM only reads it at the next
 * tick: clearing it here at the end of tick T would prevent the ROM from ever
 * sending a FACCH or speech burst. The flag is therefore left to the ROM; a
 * block is published once per posting (flag seen dropping in between, or
 * different content). MONTANT_CONSOMME_BLUD=1: the host clears the flag after
 * reading. */
static bool prendre_ul(uint16_t *api_ram, unsigned off, uint8_t *out, int n)
{
    uint16_t *w = &api_ram[(API_NDB + off) / 2];
    static int consomme = -1;
    static struct { unsigned off; bool publie; uint8_t contenu[FR_BYTES]; } vu[4];
    static int nvu;
    if (consomme < 0) {
        const char *e = calypso_getenv("MONTANT_CONSOMME_BLUD");
        consomme = (e && *e == '1') ? 1 : 0;
        printf("  [montant] B_BLUD des tampons montants (a_cu, a_fu, a_du) %s\n",
               consomme ? "efface par l'hote apres lecture (MONTANT_CONSOMME_BLUD=1) : la ROM n'emet ni FACCH ni parole"
                        : "laisse a la ROM, qui le consomme (MONTANT_CONSOMME_BLUD=1 pour l'ancien comportement)");
    }
    int k;
    for (k = 0; k < nvu && vu[k].off != off; k++) { }
    if (k == nvu && nvu < 4) { vu[nvu].off = off; vu[nvu].publie = false; nvu++; }
    if (!(w[0] & B_BLUD)) {
        if (k < nvu) vu[k].publie = false;     /* the ROM consumed it: the next posting is published */
        return false;
    }
    for (int i = 0; i < n; i += 2) {
        uint16_t v = w[3 + i / 2];
        uint8_t premier = (n == FR_BYTES) ? (uint8_t)(v >> 8) : (uint8_t)(v & 0xff);
        uint8_t second  = (n == FR_BYTES) ? (uint8_t)(v & 0xff) : (uint8_t)(v >> 8);
        out[i] = premier;
        if (i + 1 < n) {
            out[i + 1] = second;
        }
    }
    if (consomme) {
        w[0] &= (uint16_t)~B_BLUD;
        return true;
    }
    if (k < nvu) {
        if (vu[k].publie && memcmp(vu[k].contenu, out, (size_t)n) == 0) return false;   /* already published, not yet consumed */
        vu[k].publie = true;
        memcpy(vu[k].contenu, out, (size_t)n);
    }
    return true;
}

/* UPLINK SPEECH IS IN TI FORMAT, THE BRIDGE EXPECTS STANDARD FR.
 * mobile_pont.cfg: io-tch-format ti. gapk converts the microphone voice to
 * the TI DSP format (osmo-gapk fmt_ti.c, ti_fr_from_canon), the firmware
 * copies it as is into a_du, and the bridge passes it to
 * gsm0503_tch_fr_encode(..., net_order=1), which expects TS 101 318 FR (the
 * RTP "gsm" format, leading nibble 0xd). The 260-bit order differs, so the
 * block is converted here as in fmt_ti.c (ti_fr_to_canon) then fmt_gsm.c
 * (gsm_from_canon). The downlink stays TI end to end. MONTANT_PAROLE_TI=0:
 * raw pass-through. */
#include <osmocom/codec/codec.h>
static int bit_msb(const uint8_t *b, int i) { return (b[i >> 3] >> (7 - (i & 7))) & 1; }
static void mettre_bit_msb(uint8_t *b, int i, int v)
{
    if (v) b[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
    else   b[i >> 3] &= (uint8_t)~(0x80 >> (i & 7));
}
static void parole_ti_vers_fr(uint8_t fr[FR_BYTES])
{
    static int conv = -1;
    if (conv < 0) { const char *e = calypso_getenv("MONTANT_PAROLE_TI"); conv = !(e && *e == '0'); }
    if (!conv) return;
    uint8_t canon[FR_BYTES];
    memset(canon, 0, sizeof canon);
    for (int i = 0; i < 260; i++) {                 /* ti_fr_to_canon */
        int si = i >= 182 ? i + 4 : i;
        mettre_bit_msb(canon, gsm610_bitorder[i], bit_msb(fr, si));
    }
    fr[0] = (uint8_t)(0xd0 | (canon[0] >> 4));      /* gsm_from_canon: 0xd + 260 bits */
    for (int i = 1; i < FR_BYTES; i++)
        fr[i] = (uint8_t)((canon[i - 1] << 4) | (canon[i] >> 4));
}

static int fd_facch = -2;
static uint32_t seq_facch;

/* UPLINK FACCH AT THE GO. The ARM posts a_fu (B_BLUD) between phase A and the GO;
 * the ROM reads it and clears B_BLUD during phase B of the SAME frame, so
 * montant_scruter (end of frame) mostly finds the flag already dropped. Called
 * by pont.c at the GO, before the ROM runs; the duplicate filter of
 * prendre_ul() keeps montant_scruter from republishing the same block.
 * MONTANT_FU_AU_GO=0 disables it. */
void montant_capter_fu(uint16_t *api_ram, uint32_t fn)
{
    static int on = -1;
    if (on < 0) { const char *e = calypso_getenv("MONTANT_FU_AU_GO"); on = !(e && *e == '0'); }
    if (!on || !g.sur_tch) return;
    uint8_t l2[23];
    if (prendre_ul(api_ram, NDB_A_FU, l2, 23)) {
        publier_l2(&fd_facch, SHM_FACCH_UL, &seq_facch, l2, TCHT_DSP_TASK, fn);
        if (g.facch++ < (unsigned long)journal())
            printf("  [montant] FACCH UL fn=%u (au GO)\n", fn);
    }
    /* UPLINK SPEECH AT THE GO. a_du (NDB_A_DU_1) behaves like a_fu: B_BLUD is
     * cleared by the ROM during phase B of the frame, so capture_tch_ul
     * (montant_scruter, end of frame) comes too late. It is captured here at
     * the GO, symmetric to the FACCH above. The duplicate filter of
     * prendre_ul() (content dedup, vu[]) prevents a double send when
     * capture_tch_ul retries at the end of the frame. MONTANT_DU_AU_GO=0
     * disables it. */
    static int du = -1;
    if (du < 0) { const char *e = calypso_getenv("MONTANT_DU_AU_GO"); du = !(e && *e == '0'); }
    if (du) {
        uint8_t fr[FR_BYTES];
        if (prendre_ul(api_ram, NDB_A_DU_1, fr, FR_BYTES)) {
            noter_ul(fn, 1, TCHT_DSP_TASK, fr, FR_BYTES);   /* raw, TI format (a_du) */
            parole_ti_vers_fr(fr);
            noter_ul(fn, 0, TCHT_DSP_TASK, fr, FR_BYTES);   /* converted to FR, what the bridge gets */
            publier_parole(fr, fn);
            if (g.parole++ < (unsigned long)journal())
                printf("  [montant] parole UL fn=%u (au GO)\n", fn);
        }
    }
}

static bool capture_tch_ul(uint16_t *api_ram, uint16_t task_u, uint32_t fn)
{
    static int fd_sacch = -2;
    static uint32_t seq_sacch;
    uint8_t l2[23], fr[FR_BYTES];

    switch (task_u & 0x7FFF) {
    case TCHT_DSP_TASK:
        if (prendre_ul(api_ram, NDB_A_FU, l2, 23)) {
            publier_l2(&fd_facch, SHM_FACCH_UL, &seq_facch, l2, task_u, fn);
            if (g.facch++ < (unsigned long)journal())
                printf("  [montant] FACCH UL fn=%u task=0x%04x\n", fn, task_u);
        }
        if (prendre_ul(api_ram, NDB_A_DU_1, fr, FR_BYTES)) {
            noter_ul(fn, 1, task_u, fr, FR_BYTES);      /* raw, TI format (a_du) */
            parole_ti_vers_fr(fr);
            noter_ul(fn, 0, task_u, fr, FR_BYTES);      /* converted to FR, what the bridge gets */
            publier_parole(fr, fn);
            if (g.parole++ < (unsigned long)journal())
                printf("  [montant] parole UL fn=%u\n", fn);
        }
        return true;
    case TCHA_DSP_TASK:
        if (prendre_ul(api_ram, NDB_A_CU, l2, 23)) {
            publier_l2(&fd_sacch, SHM_SACCH_UL, &seq_sacch, l2, task_u, fn);
            if (g.sacch++ < (unsigned long)journal())
                printf("  [montant] SACCH UL fn=%u task=0x%04x\n", fn, task_u);
        }
        return true;
    case TCHD_DSP_TASK:
        return true;
    default:
        return false;
    }
}

/* Uplink SDCCH / SACCH.
 *
 * The firmware ANNOUNCES its block, nothing has to be guessed:
 * prim_tx_nb.c:80-101 writes into a_cu the header `(1 << B_BLUD)`, two zero
 * words, then the 23 L2 bytes from word 3 on -- exactly the layout that
 * prendre_ul() reads. The flag is single-use: one block = one publication.
 *
 * MONTANT_SDCCH_FENETRE=1 forces the heuristic path (LAPDm header scan in
 * a_cu+6 with a content-based duplicate filter, as in the gr-gsm layer 1), and
 * that path takes over by itself if B_BLUD never rises while the firmware
 * posts uplink tasks (the case where the ROM would consume the flag first).
 */
/* DUPLICATE FILTER EXPIRY (heuristic path only).
 *
 * The gr-gsm layer 1 republishes an identical block after 60 ticks
 * (calypso_l1_grgsm.c:673). This is NOT a retransmission engine: in that
 * setup QEMU clears d_task_u at every tick (calypso_trx.c, non-bridge branch),
 * so layer 1 only sees an uplink task on the frames where the firmware has
 * just posted it. The 60 ticks only prevent publishing the same block four
 * times (once per burst) while letting a REAL retransmission by the mobile
 * through, when its T200 makes it re-post.
 *
 * On the B_BLUD path this problem does not exist: the flag is single-use, one
 * posting = one publication, and a retransmission by the mobile raises the
 * flag again. Nothing to delay. */
#define SDCCH_TTL_DEFAUT      60   /* frames, like SDCCH_UL_DEDUP_TICKS */

static int sabm_ttl(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = calypso_getenv("MONTANT_SDCCH_TTL");
        if (!e || !*e) {
            e = calypso_getenv("MONTANT_SDCCH_REPETE");   /* alias */
        }
        v = (e && *e) ? atoi(e) : SDCCH_TTL_DEFAUT;
    }
    return v;
}

static void publier_sdcch(uint16_t task_u, uint32_t fn, const uint8_t *l2)
{
    static int fd = -2;
    static uint32_t seq;
    if (l2[1] == 0x03) {      /* empty frame (UI without data) */
        return;
    }
    publier_l2(&fd, SHM_SDCCH_UL, &seq, l2, task_u, fn);
    if (g.sdcch++ < (unsigned long)journal()) {
        printf("  [montant] SDCCH UL fn=%u task=0x%04x L2=", fn, task_u);
        for (int k = 0; k < 23; k++) printf("%02x%s", l2[k], k == 22 ? "" : " ");
        printf("\n");
    }
}

/* Heuristic path: window scan, duplicate filter on the useful content,
 * republication after MONTANT_SDCCH_REPETE frames (0 = never). */
static void capture_sdcch_fenetre(uint16_t *api_ram, uint16_t task_u, uint32_t fn)
{
    uint8_t fen[30];
    const uint8_t *src = (const uint8_t *)api_ram + API_NDB + NDB_A_CU + SDCCH_UL_WINDOW_OFS;
    memcpy(fen, src, sizeof(fen));

    int kk = 0;
    for (int j = 0; j <= 6; j++) {
        uint8_t a = fen[j], c = fen[j + 1], l = fen[j + 2];
        int sapi = (a >> 2) & 7;
        bool addr_ok = (a & 0x01) && ((a & 0x60) == 0) && (sapi == 0 || sapi == 3);
        bool ctrl_ok = (c != 0x2b) && (c != 0xff);
        bool len_ok = (l & 0x01) && ((l >> 2) <= 20);
        if (addr_ok && ctrl_ok && len_ok) {
            kk = j;
            break;
        }
    }
    const uint8_t *l2 = fen + kk;
    int repete = sabm_ttl();
    unsigned utile = 3u + (unsigned)(l2[2] >> 2);
    if (utile > 23u) {
        utile = 23u;
    }
    if (g.sdcch_a_dernier && !memcmp(g.sdcch_dernier, l2, utile) &&
        (repete <= 0 || (uint32_t)(fn - g.sdcch_trame) < (uint32_t)repete)) {
        return;
    }
    memcpy(g.sdcch_dernier, l2, 23);
    g.sdcch_trame = fn;
    g.sdcch_a_dernier = true;
    publier_sdcch(task_u, fn, l2);
}

static void capture_sdcch_ul(uint16_t *api_ram, uint16_t task_u, uint32_t fn)
{
    static int fenetre = -1;
    if (fenetre < 0) {
        const char *e = calypso_getenv("MONTANT_SDCCH_FENETRE");
        fenetre = (e && *e == '1') ? 1 : 0;
    }
    if (!fenetre) {
        uint8_t l2[23];
        if (prendre_ul(api_ram, NDB_A_CU, l2, 23)) {
            g.blud_vu = true;
            publier_sdcch(task_u, fn, l2);
            return;
        }
        if (g.blud_vu) {
            return;               /* the flag works: nothing to publish */
        }
        /* B_BLUD never seen while the firmware posts uplink tasks: the ROM
         * may consume it first. Switch to the window scan. */
        if (++g.sans_blud == 400) {
            printf("  [montant] a_cu : B_BLUD jamais vu en %u taches montantes, "
                   "bascule sur la fenetre heuristique\n", g.sans_blud);
        }
        if (g.sans_blud < 400) {
            return;
        }
    }
    capture_sdcch_fenetre(api_ram, task_u, fn);
}

/* The dedicated channel, read directly from the side-band written by the
 * QEMU L1CTL tap (calypso_dcch_tap.c) and already read by pont.py.
 *
 * A PONT_DCCH message from the bridge would be lost in the two-phase lock
 * step (the PONT_GO wait loop discards everything that is not a GO). The
 * file depends on no protocol: one 8-byte read per frame, and the same
 * sequence number pont.py uses to know whether it changed. */
static void scruter_dcch(uint32_t fn)
{
    static int fd = -2;
    static uint32_t seq;
    static uint32_t dernier_essai = UINT32_MAX;

    if (fd < 0) {
        /* At most one open attempt per frame, compared by equality so that
         * the hyperframe wrap-around (~3 h 29) never blocks a reopen. */
        if (fn == dernier_essai) {
            return;
        }
        /* Retried EVERY frame: run.sh clears the side-bands at startup and
         * the file only appears at the first write (QEMU tap here, bridge for
         * calypso_tch_cfg); any slower polling would see the announcement
         * frames too late. A failing open() costs a microsecond. */
        dernier_essai = fn;
        fd = open("/dev/shm/calypso_dcch_cfg", O_RDONLY);
        if (fd < 0) {
            return;
        }
    }
    uint8_t b[16];
    if (pread(fd, b, sizeof(b), 0) != (ssize_t)sizeof(b)) {
        return;
    }
    uint32_t s2;
    memcpy(&s2, b, 4);
    if (!s2 || s2 == seq) {
        return;
    }
    seq = s2;
    int genre = b[4], ss = b[5], tn = b[6];
    if (genre == 4) {
        /* PDCH (GPRS, firmware l1gprs_fw.c, tap calypso_dcch_tap.c): the BSP
         * plays the whole TN, like a TCH; no SDCCH memorised, no TCH switch. */
        printf("  [montant] canal dedie (side-band seq=%u) : PDCH TS%d (GPRS), toutes les trames au BSP\n", seq, tn);
        fflush(stdout);
        calypso_bsp_set_dedie(tn, 4, 0);
        g.dedie_arme = true;
        return;
    }
    /* Kind 2/3: the QEMU tap (calypso_dcch_tap.c) announces the TCH (TCH/F,
     * TCH/H). Only noted here: the BSP switch follows the firmware task
     * (suivre_tache_tch), the timeslot and TSC come from the bridge
     * (scruter_tch), and the memorised SDCCH must NOT be overwritten -- it is
     * the one an ASSIGNMENT FAILURE returns to (the tap republishes it when
     * the firmware goes back). */
    if (genre == 2 || genre == 3) {
        g_tn_tch_sb = tn;   /* TCH TS, to label the published uplink bursts (air) */
        /* With an A5/1-ciphered SDCCH neither the bridge nor the air reads the
         * ASSIGNMENT COMMAND, so calypso_tch_cfg never comes. The QEMU tap sees
         * the TS the firmware programs: it is taken as the TCH timeslot when
         * nobody announced one; suivre_tache_tch switches the BSP at the next
         * task. */
        if (!g.tch_tn && tn >= 1 && tn <= 7) {
            g.tch_tn = tn;
            printf("  [montant] TCH : intervalle TS%d pris sur le side-band (pas d'annonce calypso_tch_cfg)\n", tn);
        }
        printf("  [montant] canal dedie (side-band seq=%u) : TCH/%c TS%d SS=%d, firmware sur le TCH "
               "(%s%s)\n", seq, genre == 2 ? 'F' : 'H', tn, ss,
               g.sur_tch ? "BSP deja sur le TCH" : "BSP pas encore bascule",
               g.sd_valide ? ", SDCCH memorise" : "");
        fflush(stdout);
        return;
    }
    printf("  [montant] canal dedie (side-band seq=%u) : %s TS%d SDCCH/%d SS=%d%s\n",
           seq, genre == 0xFF ? "libere" : "arme", tn, genre == 1 ? 8 : 4, ss,
           (genre != 0xFF && g.sur_tch) ? " (memorise : le firmware est sur le TCH)" : "");
    g_tn_tch_sb = 0;   /* SDCCH (TS0) armed or channel released: the mobile no longer sends on the TCH */
    if (genre == 0xFF) {
        calypso_bsp_set_dedie(0, 0xFF, 0);
        g.dedie_arme = false;
        g.sd_valide = false;
        g.sur_tch = false;
        montant_canal_libere();
        return;
    }
    /* The SDCCH is MEMORISED: the BSP returns to it when the firmware leaves
     * the TCH (ASSIGNMENT FAILURE). While the firmware is on the TCH the BSP
     * is left alone. */
    g.sd_tn = tn;
    g.sd_genre = genre;
    g.sd_ss = ss;
    g.sd_valide = true;
    if (g.sur_tch) {
        return;
    }
    calypso_bsp_set_dedie(tn, genre, ss);
    g.dedie_arme = true;
}

/* Hand the memorised SDCCH back to the BSP (or release it if there is none). */
static void revenir_sdcch(const char *raison, uint32_t fn)
{
    g.sur_tch = false;
    if (g.sd_valide) {
        printf("  [montant] %s a fn=%u : BSP rendu au SDCCH TS%d SS=%d\n",
               raison, fn, g.sd_tn, g.sd_ss);
        calypso_bsp_set_dedie(g.sd_tn, g.sd_genre, g.sd_ss);
        g.dedie_arme = true;
    } else {
        printf("  [montant] %s a fn=%u : aucun SDCCH connu, canal dedie libere\n",
               raison, fn);
        calypso_bsp_set_dedie(0, 0xFF, 0);
        g.dedie_arme = false;
    }
    fflush(stdout);
}

/* MONTANT_TCH_TACHE=0: arm the BSP as soon as the bridge announces the TCH.
 * See scruter_tch. */
static bool tch_sur_tache(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = calypso_getenv("MONTANT_TCH_TACHE");
        v = (e && *e == '0') ? 0 : 1;
    }
    return v != 0;
}

/* THE TCH TIMESLOT.
 *
 * pont.py publishes the traffic channel timeslot in /dev/shm/calypso_tch_cfg
 * (pont/state.py, Tch._write_cfg: seq, tn, tsc, arfcn) as soon as it decodes
 * the downlink ASSIGNMENT COMMAND. MONTANT_TCH=0 disables reading it.
 *
 * THE ANNOUNCEMENT IS NOT THE SWITCH.
 * The bridge writes this file when it DECODES the ASSIGNMENT COMMAND, i.e. at
 * BTS time, while the BSP plays frames at DSP time -- 12 to ~200 frames
 * behind. Arming the BSP here would replace with TS2 bursts ALL the frames
 * not yet played (bsp_dedie_trame is true everywhere for a TCH), the
 * ASSIGNMENT COMMAND included, and TS0 with them.
 *
 * So this file is only an ANNOUNCEMENT (timeslot, TSC). The switch comes from
 * the firmware itself: suivre_tache_tch() arms the TCH at the first
 * TCHT/TCHA/TCHD task it posts, and hands the SDCCH back as soon as it posts
 * a block read task (ALLC) again. A non-zero seq with tn=0 is the bridge
 * giving up (pont/dsp/tch.py, TchDsp.abandon): back to the SDCCH without
 * dropping the Kc. seq=0 remains the full release.
 * MONTANT_TCH_TACHE=0 restores arming at the announcement. */
static void scruter_tch(uint32_t fn)
{
    static int fd = -1, coupe = -1;
    static uint32_t seq, dernier_essai = UINT32_MAX;

    if (coupe < 0) {
        const char *e = calypso_getenv("MONTANT_TCH");
        coupe = (e && *e == '0') ? 1 : 0;
    }
    if (coupe) {
        return;
    }
    if (fd < 0) {
        if (fn == dernier_essai) {          /* see scruter_dcch */
            return;
        }
        dernier_essai = fn;
        fd = open("/dev/shm/calypso_tch_cfg", O_RDONLY);
        if (fd < 0) {
            return;
        }
    }
    uint8_t b[16];
    if (pread(fd, b, sizeof(b), 0) != (ssize_t)sizeof(b)) {
        return;
    }
    uint32_t s2;
    memcpy(&s2, b, 4);
    if (s2 == seq) {
        return;
    }
    seq = s2;
    int tn = b[4], tsc = b[5];
    if (!s2) {
        /* Full release (Tch.close). */
        int etait = g.tch_tn;
        g.tch_tn = 0;
        if (g.sur_tch) {
            printf("  [montant] TCH (seq=0) : libere TS%d\n", etait);
            calypso_bsp_set_dedie(0, 0xFF, 0);
            g.sur_tch = false;
            g.dedie_arme = false;
            montant_canal_libere();
        } else if (etait) {
            printf("  [montant] TCH (seq=0) : annonce TS%d retiree (le firmware n'y etait pas)\n", etait);
        }
        return;
    }
    if (tn <= 0 || tn > 7) {
        /* Given up by the bridge (ASSIGNMENT FAILURE, mobile back on the
         * SDCCH): the SDCCH is still alive, and so is the Kc. */
        int etait = g.tch_tn;
        g.tch_tn = 0;
        printf("  [montant] TCH (seq=%u) : annonce TS%d abandonnee par le pont\n", seq, etait);
        if (g.sur_tch) {
            revenir_sdcch("abandon du TCH", fn);
        }
        return;
    }
    g.tch_tn = tn;
    g.tch_tsc = tsc;
    if (!tch_sur_tache()) {
        printf("  [montant] TCH (seq=%u) : arme TS%d TSC=%d - toutes ses trames (MONTANT_TCH_TACHE=0)\n",
               seq, tn, tsc);
        calypso_bsp_set_dedie(tn, 2 /* BSP_DEDIE_TCH */, 0);
        g.sur_tch = true;
        g.dedie_arme = true;
        return;
    }
    printf("  [montant] TCH (seq=%u) : TS%d TSC=%d annonce par le pont, "
           "bascule du BSP a la premiere tache TCH du firmware\n", seq, tn, tsc);
    fflush(stdout);
}

/* THE SDCCH <-> TCH SWITCH FOLLOWS THE FIRMWARE.
 *
 * d_task_d of the W page the ARM has just written says what it listens to on
 * the next frame: TCHT (13, traffic and FACCH), TCHA (14, SACCH/T) or TCHD
 * (28) on a TCH (prim_tch.c:520-585, 791-802), ALLC (24) for an SDCCH/SACCH
 * block (prim_rx_nb.c:200). This task is posted at DSP pace, so the BSP
 * changes timeslot on the frame the ARM really reads, whatever the DSP lag
 * behind the BTS. The other frames (neighbour PM, idle frame) change
 * nothing. */
/* d_fn PROBE: the position the firmware gives the DSP in the 104-multiframe
 * (dsp.c:537, d_fn = fn_report | (fn%104) << 8), on TCH tasks. To compare with
 * the BTS fn the BSP plays at the same tick ([sacch_tf]): an offset that is a
 * multiple of 26 lets speech and FACCH through but de-interleaves the SACCH
 * out of order (a_cd FIRE KO on every block, LOS). 24 lines. */
static void sonde_dfn(uint16_t *api_ram, unsigned pg, bool taches, uint32_t fn)
{
    static unsigned n;
    if (!taches || !g.sur_tch || n >= 24) {
        return;
    }
    uint16_t t = api_ram[(API_W_PAGE(pg) + WP_D_TASK_D) / 2] & 0x7fffu;
    uint16_t dfn = api_ram[(API_W_PAGE(pg) + WP_D_FN) / 2];
    if (t != TCHA_DSP_TASK) {
        return;
    }
    n++;
    printf("  [d_fn] tick=%u tache=%u fn_report=%u fn%%104=%u\n",
           fn, t, dfn & 0xffu, (dfn >> 8) & 0xffu);
}

static void suivre_tache_tch(uint16_t *api_ram, unsigned pg, bool taches, uint32_t fn)
{
    static bool sans_annonce_dit;
    if (!taches || !tch_sur_tache()) {
        return;
    }
    uint16_t t = api_ram[(API_W_PAGE(pg) + WP_D_TASK_D) / 2] & 0x7fffu;
    bool tache_tch = (t == TCHT_DSP_TASK || t == TCHA_DSP_TASK || t == TCHD_DSP_TASK);
    bool tache_bloc = (t == ALLC_DSP_TASK || t == DDL_DSP_TASK || t == ADL_DSP_TASK);

    if (tache_tch && !g.sur_tch) {
        if (!g.tch_tn) {
            /* No announcement (A5/1-ciphered SDCCH: the air cannot read the
             * ASSIGNMENT COMMAND): the timeslot is the one of the RX window the
             * firmware programmed in the TPU. */
            int tn_tpu = calypso_bsp_fenetre_tn();
            if (tn_tpu >= 1 && tn_tpu <= 7) {
                g.tch_tn = tn_tpu;
                printf("  [montant] TCH : tache %u postee a fn=%u sans annonce (calypso_tch_cfg) : "
                       "intervalle TS%d lu sur la fenetre TPU\n", t, fn, tn_tpu);
            } else {
                if (!sans_annonce_dit) {
                    sans_annonce_dit = true;
                    printf("  [montant] TCH : tache %u postee a fn=%u sans annonce du pont "
                           "(calypso_tch_cfg) : intervalle inconnu (TPU %d), BSP inchange\n", t, fn, tn_tpu);
                }
                return;
            }
        }
        sans_annonce_dit = false;
        printf("  [montant] TCH : le firmware poste la tache %u a fn=%u, BSP bascule sur TS%d (TSC=%d)\n",
               t, fn, g.tch_tn, g.tch_tsc);
        fflush(stdout);
        calypso_bsp_set_dedie(g.tch_tn, 2 /* BSP_DEDIE_TCH */, 0);
        g.sur_tch = true;
        g.dedie_arme = true;
        return;
    }
    /* TI FIRMWARE: ALLC AFTER THE TCH IS NOT NECESSARILY THE SDCCH. The TI L1
     * also reads the BCCH and the paging in ALLC: after the CHANNEL RELEASE of
     * a call, handing the BSP back to SDCCH/8 TS1 (which replaces TS0 on every
     * frame while the TI dedicated mode lasts, CALYPSO_BSP_SDCCH8_TOUT) would
     * lose the cell. Here the BSP first returns to TS0; the SDCCH is only
     * restored if an UPLINK task follows within 102 frames (assignment
     * failure: the L1 goes back to its SDCCH); in idle there is none.
     * MONTANT_TI_ALLC_DIRECT=1: return to the SDCCH directly. */
    static int ti = -1, direct = -1;
    static bool attente_ul; static uint32_t attente_fn;
    if (ti < 0) { ti = calypso_getenv("MS_MAGNETITE") != NULL;
                  const char *e = calypso_getenv("MONTANT_TI_ALLC_DIRECT"); direct = (e && *e == '1'); }
    if (tache_bloc && g.sur_tch) {
        if (!ti || direct) {
            revenir_sdcch("le firmware est revenu sur le SDCCH (tache ALLC)", fn);
        } else {
            g.sur_tch = false;
            g.dedie_arme = false;
            calypso_bsp_set_dedie(0, 0xFF, 0);
            attente_ul = g.sd_valide;
            attente_fn = fn;
            printf("  [montant] firmware TI : tache ALLC apres le TCH a fn=%u : BSP rendu a TS0, retour au SDCCH "
                   "seulement sur une tache montante%s\n", fn, g.sd_valide ? "" : " (aucun SDCCH connu)");
            fflush(stdout);
        }
        return;
    }
    if (attente_ul) {
        uint16_t u = api_ram[(API_W_PAGE(pg) + WP_D_TASK_U) / 2] & 0x7fffu;
        if (u) {
            attente_ul = false;
            revenir_sdcch("firmware TI : tache montante apres le TCH (echec d'assignation)", fn);
        } else if ((uint32_t)(fn - attente_fn) > 102u) {
            attente_ul = false;
            printf("  [montant] firmware TI : aucune tache montante 102 trames apres le TCH (fn=%u) : en veille\n", fn);
        }
    }
}

/* a_fd PROBE: DOWNLINK FACCH ON THE TCH.
 * The firmware only passes a FACCH up if ((fn%13)%4)==3 AND a_fd[0] carries
 * B_BLUD (prim_tch.c:235-246), then resets a_fd[0] = B_FIRE1: only the DSP
 * sets B_BLUD. This probe tells a ROM problem (B_BLUD never set) from a
 * plumbing one (set but FIRE). MONTANT_AFD=0 disables it. */
/* DOWNLINK FRAMES DELIVERED BY THE ROM, FOR COMPARISON.
 * Every speech frame (a_dd_0, converted from TI format to standard FR like
 * the uplink) and every FACCH (a_fd) seen with B_BLUD, with the tick, the
 * status word and the reported error count (word 2). 48-byte records: fn
 * LE32, type (0 speech, 1 FACCH), n, status LE16, err LE16, 2 padding bytes,
 * 36 data bytes. File reset at each call (more than 500 frames without a
 * record), 8000 records at most. tools/comparer_parole.py compares it with
 * /dev/shm/calypso_tch_dl.bin (BSP bursts) and /dev/shm/pont_tch_dl.bin
 * (bridge decoding). */
static void noter_dl(uint32_t fn, uint8_t type, uint16_t etat, uint16_t err,
                     const uint8_t *data, int n)
{
    static FILE *f;
    static unsigned nrec;
    static uint32_t fn_prec;
    if (!f || (uint32_t)(fn - fn_prec) > 500u) {
        if (f) fclose(f);
        f = fopen("/dev/shm/calypso_add_dl.bin", "wb");
        nrec = 0;
    }
    fn_prec = fn;
    if (!f || nrec >= 8000) return;
    uint8_t r[48];
    memset(r, 0, sizeof r);
    memcpy(r, &fn, 4);
    r[4] = type; r[5] = (uint8_t)n;
    memcpy(r + 6, &etat, 2);
    memcpy(r + 8, &err, 2);
    memcpy(r + 12, data, n > 36 ? 36 : n);
    fwrite(r, 1, sizeof r, f);
    if ((++nrec % 32) == 0) fflush(f);
}

static void sonde_afd(uint16_t *api_ram, uint32_t fn)
{
    static int sonde = -1;
    static uint16_t prec;
    static unsigned long ok, ko;
    if (sonde < 0) {
        const char *e = calypso_getenv("MONTANT_AFD");
        sonde = (e && *e == '0') ? 0 : 1;
    }
    if (!sonde || !g.sur_tch) {
        return;
    }
    uint16_t etat = api_ram[(API_NDB + NDB_A_FD) / 2];
    if ((etat & B_BLUD) && etat != prec) {
        const uint8_t *d = (const uint8_t *)api_ram + API_NDB + NDB_A_FD + 6;
        uint16_t mode = api_ram[(API_NDB + NDB_D_TCH_MODE) / 2];
        bool fire = (etat & 0x0040) != 0;          /* B_FIRE1 */
        if (fire) ko++; else ok++;
        noter_dl(fn, 1, etat, api_ram[(API_NDB + NDB_A_FD) / 2 + 2], d, 23);
        if ((ok + ko) <= 60 || ((ok + ko) % 50) == 0) {
            printf("  [a_fd] fn=%u fn%%13=%u etat=%04x BLUD=1 FIRE=%d d_tch_mode=%04x "
                   "L2=%02x %02x %02x %02x | ok=%lu ko=%lu\n",
                   fn, fn % 13u, etat, fire ? 1 : 0, mode, d[0], d[1], d[2], d[3], ok, ko);
        }
    }
    prec = etat;
}

/* a_dd PROBE: DOWNLINK SPEECH ON THE TCH.
 * The firmware only passes a speech frame up to the mobile
 * (L1CTL_TRAFFIC_IND) if a_dd_0[0] carries B_BLUD (prim_tch.c:322-327); GAPK
 * only encodes the uplink in response. This probe tells whether the DSP still
 * delivers downlink speech. One line for the first 20, then every 100.
 * MONTANT_ADD=0 disables it.
 *
 * 0x0040 is B_FIRE1 and the Fire code only protects control channels, so
 * speech is judged by bit 2 = B_BFI (l1_environment.h:272). The firmware only
 * tests B_BLUD (prim_tch.c:327) and does not pass the BFI up: GAPK decodes
 * the 33 bytes as they are. The probe therefore counts the BFI too and shows
 * a_dd_0[2], the error count the ROM reports (num_biterr on the firmware
 * side). */
static void sonde_add(uint16_t *api_ram, uint32_t fn)
{
    static int sonde = -1;
    static uint16_t prec;
    static unsigned long vues, ko, bfi;
    static uint32_t fn_dernier;
    if (sonde < 0) {
        const char *e = calypso_getenv("MONTANT_ADD");
        sonde = (e && *e == '0') ? 0 : 1;
    }
    if (!sonde || !g.sur_tch) {
        return;
    }
    uint16_t etat = api_ram[(API_NDB + NDB_A_DD_0) / 2];
    if ((etat & B_BLUD) && etat != prec) {
        bool fire = (etat & 0x0040) != 0;
        bool mauvaise = (etat & 0x0004) != 0;      /* B_BFI */
        uint16_t erreurs = api_ram[(API_NDB + NDB_A_DD_0) / 2 + 2];
        {
            const uint16_t *w = &api_ram[(API_NDB + NDB_A_DD_0) / 2];
            uint8_t fr[FR_BYTES];
            for (int k = 0; k < FR_BYTES; k += 2) {       /* high byte first, like prendre_ul */
                fr[k] = (uint8_t)(w[3 + k / 2] >> 8);
                if (k + 1 < FR_BYTES) fr[k + 1] = (uint8_t)(w[3 + k / 2] & 0xff);
            }
            parole_ti_vers_fr(fr);
            noter_dl(fn, 0, etat, erreurs, fr, FR_BYTES);
        }
        vues++;
        if (fire) ko++;
        if (mauvaise) bfi++;
        if (vues <= 20 || vues % 100 == 0) {
            printf("  [a_dd] fn=%u etat=%04x FIRE=%d BFI=%d err=%u (ecart %u trames) | vues=%lu ko=%lu bfi=%lu\n",
                   fn, etat, fire ? 1 : 0, mauvaise ? 1 : 0, erreurs, fn - fn_dernier, vues, ko, bfi);
        }
        fn_dernier = fn;
    }
    prec = etat;
}

bool montant_sur_tch(void)
{
    return g.sur_tch;
}

/* Timeslot the firmware transmits on: the TCH TS announced by the side-band
 * (kind 2/3) until it is released, 0 otherwise (SDCCH/4 on TS0, RACH). Used to
 * label the bursts published in calypso_tx_rom (bits 4-6 of the src byte) so
 * that the air carries them on the right TRX timeslot. */
int montant_tn_dedie(void)
{
    return g_tn_tch_sb;
}

void tx_rach_vers_air(uint8_t ra, uint8_t bsic, uint32_t tick);   /* pont.c */

static void publier_rach(uint8_t ra, uint8_t bsic, uint32_t fn)
{
    tx_rach_vers_air(ra, bsic, fn);   /* Magnetite: to the air (CALYPSO_RACH_VERS_AIR) */
    static int fd = -2;
    static uint32_t seq;
    if (fd == -2) {
        fd = sb_ouvrir(SHM_RACH, REC_RACH);
    }
    uint8_t buf[REC_RACH];
    memset(buf, 0, sizeof(buf));
    seq++;
    memcpy(buf + 0, &seq, 4);
    buf[4] = ra;
    buf[5] = bsic;
    memcpy(buf + 8, &fn, 4);
    sb_ecrire(fd, buf, sizeof(buf), 0);
    /* An access attempt is a new connection: the duplicate filter memory of
     * the heuristic path restarts (no effect on the B_BLUD path, which does
     * not need it). */
    g.sdcch_a_dernier = false;
    if (g.rach++ < (unsigned long)journal())
        printf("  [montant] RACH ra=0x%02x bsic=%u fn=%u -> %s\n", ra, bsic, fn, SHM_RACH);
}

/* ── THE Kc: THE BRIDGE CAN NEITHER CIPHER NOR DECIPHER WITHOUT IT ─────────
 *
 * Under CALYPSO_DSP_EXTERN=1 the gr-gsm layer 1 is disabled, and it was the
 * one publishing /dev/shm/calypso_kc_l1 (calypso_l1_grgsm.c, publish_kc).
 * Without the file, `Cipher.current()` of pont.py returns None and
 * `cipher.apply()` leaves the burst UNCHANGED in both directions. There is no
 * A5 in the Calypso model either (`d_a5mode` only exists in l1-grgsm/, not in
 * l1-dsp/): the bridge does it, as it already does the channel coding.
 *
 * Layout taken as is from publish_kc() so that pont/cipher.py (KC_RECLEN=32)
 * reads it unchanged: seq(4) algo(1) length(1) Kc[8] 0xFF. The four words of
 * a_kc come out big-endian AND reversed, as in the original -- a layout the
 * reader expects is not "corrected".
 *
 * MONTANT_KC=0 disables the publication. */
#define KC_RECLEN        32
#define KC_PUBLIER_TOUTES 22   /* frames between two polls, like grgsm */
#define KC_GRACE_CLAIR    5    /* cf. publish_kc: the firmware clears d_a5mode
                                * at every DM_REL_REQ, including when the Kc
                                * comes right back (Assignment Command),
                                * while the BTS keeps ciphering. */

/* Raised by montant_canal_libere(): the next poll must publish the return to
 * clear WITHOUT waiting for the grace period. See publier_kc(). */
static bool g_kc_liberer;

static void publier_kc(uint16_t *api_ram)
{
    static int actif = -1, fd = -1, tick, clair_en_attente;
    static uint32_t seq;
    static uint8_t dernier[KC_RECLEN];
    static bool a_dernier;

    if (actif < 0) {
        const char *e = calypso_getenv("MONTANT_KC");
        actif = (e && *e == '0') ? 0 : 1;
    }
    if (!actif) {
        return;
    }
    /* A MODE CHANGE CANNOT WAIT FOR THE PERIODIC POLL. The mobile switches as
     * soon as it processes the CIPHERING MODE COMMAND and sends its CIPHERING
     * MODE COMPLETE right away; an uplink SDCCH block falls every 51 frames,
     * and that block -- the FIRST ciphered uplink message -- must not leave
     * in clear while the BTS already deciphers it. d_a5mode is therefore read
     * EVERY frame (two memory accesses) and the full poll is only deferred
     * while the mode does not change. */
    uint16_t mode = api_ram[(API_NDB + NDB_D_A5MODE) / 2];
    uint8_t mode_algo = (mode >= 1 && mode <= 3) ? (uint8_t)mode : 0;
    bool bascule = (a_dernier && mode_algo != dernier[4]) || g_kc_liberer;
    if (!bascule && ++tick < KC_PUBLIER_TOUTES) {
        return;
    }
    tick = 0;
    const uint16_t *kw = &api_ram[(API_NDB + NDB_A_KC) / 2];
    uint8_t rec[KC_RECLEN] = {0};
    bool nul = true;
    for (int i = 0; i < 4; i++) {
        rec[6 + 6 - 2 * i] = (uint8_t)(kw[i] >> 8);
        rec[6 + 7 - 2 * i] = (uint8_t)(kw[i] & 0xFF);
    }
    for (int i = 6; i < 14; i++) {
        if (rec[i]) {
            nul = false;
        }
    }
    uint8_t algo = (mode >= 1 && mode <= 3 && !nul) ? (uint8_t)mode : 0;
    if (!algo) {
        memset(rec + 6, 0, 8);
    }
    rec[4] = algo;
    rec[5] = algo ? 8 : 0;
    rec[14] = 0xFF;

    if (a_dernier && !memcmp(dernier + 4, rec + 4, KC_RECLEN - 4)) {
        clair_en_attente = 0;
        /* This early return must CONSUME g_kc_liberer, otherwise the release
         * stays armed indefinitely: every following poll would compute
         * bascule=true, republish the same record and bump `seq`, which
         * pont.py reads as "new key". */
        g_kc_liberer = false;
        return;
    }
    /* THE GRACE PERIOD MUST NOT SURVIVE THE CHANNEL RELEASE. It comes from
     * publish_kc() and serves the INTRA-connection case: the firmware clears
     * d_a5mode at every DM_REL_REQ, including during an Assignment Command
     * where the Kc comes right back, while the BTS keeps ciphering. Between
     * TWO connections it is harmful: the algo=1 record would stay readable
     * five polls longer, and pont.py -- which drops its key at every
     * IMMEDIATE ASSIGNMENT (downlink.py) -- would read it back from the file
     * and cipher the uplink of the NEXT connection from its first block while
     * the mobile still sends in clear. The channel release is the right
     * signal: montant_canal_libere(), called on PONT_DCCH kind 0xFF. */
    if (!algo && a_dernier && dernier[4] && !g_kc_liberer &&
        ++clair_en_attente < KC_GRACE_CLAIR) {
        return;   /* ciphered -> clear: wait, the Kc may come back */
    }
    clair_en_attente = 0;
    g_kc_liberer = false;

    if (fd < 0 && (fd = open(SHM_KC, O_WRONLY | O_CREAT, 0644)) < 0) {
        actif = 0;
        return;
    }
    seq++;
    memcpy(rec, &seq, 4);
    if (pwrite(fd, rec, sizeof rec, 0) != (ssize_t)sizeof rec) {
        close(fd); fd = -1; seq--;
        return;
    }
    memcpy(dernier, rec, sizeof rec);
    a_dernier = true;
    if (algo) {
        printf("  [montant] chiffrement A5/%u : Kc publie vers %s (seq=%u)\n",
               algo, SHM_KC, seq);
    } else {
        printf("  [montant] retour en clair (seq=%u)\n", seq);
    }
    fflush(stdout);
}

void calypso_bsp_set_dedie_ti(int actif);   /* calypso_bsp.c */
static bool g_ti_dedie;                      /* TI firmware in dedicated mode (ALLC tasks), see below */

/* DOWNLINK FACCH PROSTHESIS (TI firmware). Under the Magnetite TI L1 the ROM
 * decodes the FACCH/F with the last two bursts of the block missing (Fire KO,
 * RR Assignment Failure), while the TS2 bursts the BSP delivers carry the
 * block intact. They are decoded here at every end of a FACCH/F block
 * (fn % 13 % 4 == 3) and, if it is a FACCH with a good CRC, posted into
 * a_fd: the TI L1 reads it on the next frame (l1_cmplx.c, fn_mod13 % 4 == 0),
 * after the end of this tick. The rest (speech) stays with the ROM.
 * CALYPSO_PROTHESE_FACCH_DL=0 disables it (live setting). */
/* DOWNLINK SPEECH PROSTHESIS (TI firmware), same mechanism as the FACCH: the
 * FR block decoded here from the BSP bursts (good CRC) is posted into a_dd in
 * TI format (inverse of parole_ti_vers_fr: gsm_to_canon then
 * ti_fr_from_canon), BFI at 0, so that the ROM speech decoder takes it
 * instead of its own. Only replaces the frames the ROM judged bad.
 * CALYPSO_PROTHESE_PAROLE_DL=0 disables it (live setting), =2 also replaces
 * the good ones. */
static void prothese_parole_dl(uint16_t *api_ram, const uint8_t fr[FR_BYTES], int ne, uint32_t e)
{
    static unsigned n_ok;
    int mode = calypso_reglage("CALYPSO_PROTHESE_PAROLE_DL", 1);
    if (!mode) return;
    uint16_t *dd = &api_ram[(API_NDB + NDB_A_DD_0) / 2];
    if (mode == 1 && (dd[0] & B_BLUD) && !(dd[0] & 0x0004)) return;   /* the ROM already has it good */
    uint8_t canon[FR_BYTES], ti[FR_BYTES + 1];
    memset(canon, 0, sizeof canon);
    for (int i = 0; i < 260; i++)                   /* gsm_to_canon: the leading 0xd nibble is skipped */
        mettre_bit_msb(canon, i, bit_msb(fr, i + 4));
    memset(ti, 0, sizeof ti);
    for (int i = 0; i < 260; i++) {                 /* ti_fr_from_canon */
        int si = i >= 182 ? i + 4 : i;
        mettre_bit_msb(ti, si, bit_msb(canon, gsm610_bitorder[i]));
    }
    dd[0] = (uint16_t)((dd[0] & ~0x0044u) | B_BLUD);   /* block present, neither FIRE nor BFI */
    dd[2] = (uint16_t)ne;
    for (int k = 0; k < FR_BYTES; k += 2)              /* high byte first, like sonde_add */
        dd[3 + k / 2] = (uint16_t)((ti[k] << 8) | (k + 1 < FR_BYTES ? ti[k + 1] : 0));
    if (n_ok++ < 40 || n_ok % 500 == 0)
        printf("  [parole-dl] fn=%u : trame FR de la BTS posee dans a_dd (%d err) [%u]\n", e, ne, n_ok);
}

static void prothese_facch_dl(uint16_t *api_ram)
{
    static uint32_t deja = 0xffffffffu;
    static unsigned n_ok;
    uint32_t e;
    if (!calypso_reglage("CALYPSO_PROTHESE_FACCH_DL", 1) || !g_ti_dedie) return;
    if (!calypso_bsp_tch_dernier_fn(&e) || e == deja) return;
    deja = e;
    if (e % 26u == 12u || e % 26u == 25u || (e % 13u) % 4u != 3u) return;
    uint32_t seq[8]; int n = 0;
    for (uint32_t f = e; n < 8; f--) {
        if (f % 26u == 12u || f % 26u == 25u) continue;
        seq[7 - n++] = f;
    }
    sbit_t soft[8 * 116];
    for (int k = 0; k < 8; k++) {
        uint8_t b[148];
        if (!calypso_bsp_tch_burst(seq[k], b)) return;
        for (int j = 0; j < 58; j++) {
            soft[k * 116 + j]      = b[3 + j]  ? -127 : 127;
            soft[k * 116 + 58 + j] = b[87 + j] ? -127 : 127;
        }
    }
    uint8_t l2[33]; int ne = 0, nb = 0;
    int rc = gsm0503_tch_fr_decode(l2, soft, 1, 0, &ne, &nb);
    if (rc == FR_BYTES) { prothese_parole_dl(api_ram, l2, ne, e); return; }
    if (rc != 23) return;
    uint16_t *fd = &api_ram[(API_NDB + NDB_A_FD) / 2];
    fd[0] = B_BLUD;                 /* block present, B_FIRE1/B_FIRE0 at 0: CRC good */
    fd[2] = (uint16_t)ne;
    for (int k = 0; k < 12; k++)
        fd[3 + k] = (uint16_t)(l2[2 * k] | (2 * k + 1 < 23 ? l2[2 * k + 1] << 8 : 0));
    if (n_ok++ < 40 || n_ok % 100 == 0)
        printf("  [facch-dl] fn=%u : FACCH de la BTS posee dans a_fd (%02x %02x %02x %02x, %d err) [%u]\n",
               e, l2[0], l2[1], l2[2], l2[3], ne, n_ok);
    fflush(stdout);
}

void montant_scruter(uint16_t *api_ram, uint32_t fn, unsigned page)
{
    static int coupe = -1;
    if (coupe < 0) {
        const char *e = calypso_getenv("MONTANT");
        coupe = (e && *e == '0') ? 1 : 0;
    }
    if (coupe || !api_ram) {
        return;
    }

    /* Which W page carries the tasks? dsp_end_scenario() (firmware,
     * calypso/dsp.c:471) writes d_dsp_page = B_GSM_TASK | w_page BEFORE
     * flipping w_page: the NDB word is the fresh source, including when
     * l1_sync() ran between the TICK and the GO. The `page` argument (the
     * d_dsp_page QEMU sampled at the TICK) is only a fallback. */
    uint16_t v_page = api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2];
    bool taches = (v_page & B_GSM_TASK) != 0;
    unsigned pg = taches ? ((v_page & B_GSM_PAGE) ? 1u : 0u) : (page & 1u);

    {   /* CIPHERING PROBE: every change of d_a5mode or a_kc as the ARM leaves
         * them (NDB 0x1ce / 0x2ce), with the tick. The DSP ciphers and
         * deciphers by itself (calypso_a5.c) from these two fields; a reset
         * on the TCH stops deciphering while the BTS keeps ciphering. */
        static uint16_t a5_prec = 0xffff, kc_prec[4];
        uint16_t a5 = api_ram[(API_NDB + 0x1ce) / 2];
        const uint16_t *kc = &api_ram[(API_NDB + 0x2ce) / 2];
        if (a5 != a5_prec || memcmp(kc, kc_prec, sizeof kc_prec)) {
            printf("  [a5-arm] tick=%u d_a5mode=%u a_kc=%04x %04x %04x %04x%s\n", fn, a5, kc[0], kc[1], kc[2], kc[3],
                   g.sur_tch ? "  (BSP sur le TCH)" : "");
            fflush(stdout);
            a5_prec = a5; memcpy(kc_prec, kc, sizeof kc_prec);
        }
    }
    scruter_dcch(fn);
    scruter_tch(fn);      /* the TCH announcement by the bridge            */
    suivre_tache_tch(api_ram, pg, taches, fn);   /* the switch, by the firmware */
    sonde_dfn(api_ram, pg, taches, fn);
    if (taches) calypso_bsp_dfn_observe(fn, api_ram[(API_W_PAGE(pg) + WP_D_FN) / 2], api_ram[(API_W_PAGE(pg) + WP_D_TASK_D) / 2]);

    /* B_SYNC_TCH_DL FOR THE TI L1. osmocom (calypso/dsp.c dsp_load_tch_param)
     * sets at the first TCH both B_SYNC_TCH_UL (bit 8 of d_ctrl_tch) AND
     * B_SYNC_TCH_DL (bit 9); the Magnetite TI L1 sets bit 8 alone, its bit 9
     * is named B_SYNC_AMR and stays 0 (d_ctrl_tch 0x01d1 then 0x0091). Without
     * downlink sync the ROM keeps an arbitrary TCH block phase and decodes
     * every FACCH with its last two bursts missing (Fire KO, RR Assignment
     * Failure). Set here, at the end of the tick, on the W page the ROM reads
     * at the next tick. CALYPSO_TI_SYNC_DL=0 disables it. */
    {
        /* n = 1: on the page carrying bit 8; n > 1: on the (n-1)-th following
         * task page (shifts the phase). */
        int sync_dl = calypso_reglage("CALYPSO_TI_SYNC_DL", 1);   /* live, /dev/shm/calypso_reglages */
        static int attente = -1;
        static bool fait_tch;   /* once per TCH */
        uint16_t *ctrl = &api_ram[(API_W_PAGE(pg) + 0x12u) / 2];   /* d_ctrl_tch, word 9 of the W page */
        uint16_t td = api_ram[(API_W_PAGE(pg) + WP_D_TASK_D) / 2] & 0x7fffu;
        if (!g.sur_tch) fait_tch = false;
        if (sync_dl > 0 && taches) {
            /* ALSO ON THE FIRST TCH TASK: the TI L1 may set d_ctrl_tch=0x0091
             * (without bit 8) from the first TCH page on, so bit 8 alone is
             * not a reliable trigger. */
            if ((*ctrl & 0x0100) && !(*ctrl & 0x0200)) attente = sync_dl - 1;
            else if (td == TCHT_DSP_TASK && !fait_tch && !(*ctrl & 0x0200)) { attente = sync_dl - 1; fait_tch = true; }
            else if (attente > 0) attente--;
            if (attente == 0) {
                *ctrl |= 0x0200;
                printf("  [sync-dl] tick=%u page W%u : d_ctrl_tch |= B_SYNC_TCH_DL -> %04x (reglage %d)\n", fn, pg, *ctrl, sync_dl);
                fflush(stdout);
                attente = -1;
            }
        }
    }
    /* VOICE BY THE HOST: NO ROM CODEC. With PONT_VOIX_HOTE=1 (pont/voix_hote.py,
     * libgsm) speech no longer goes through the ROM, but the TI L1 sets
     * B_VOCODER_ON (bit 7 of d_ctrl_tch, 0x0091) and the ROM would still
     * encode and decode every frame (8.4 ms per frame for 4.62 available,
     * with the bridge clock slaved to the DSP: the whole network slows down).
     * The bit is cleared on the task W page. CALYPSO_TI_VOCODEUR_OFF (live
     * setting): default 1 if PONT_VOIX_HOTE=1, else 0. */
    {
        static int defaut = -1;
        if (defaut < 0) { const char *e = calypso_getenv("PONT_VOIX_HOTE"); defaut = (e && *e == '1'); }
        if (taches && calypso_reglage("CALYPSO_TI_VOCODEUR_OFF", defaut)) {
            uint16_t *ctrl = &api_ram[(API_W_PAGE(pg) + 0x12u) / 2];   /* d_ctrl_tch, word 9 of the W page */
            if (*ctrl & 0x0080) {
                static unsigned n_voc;
                *ctrl &= (uint16_t)~0x0080u;
                if (n_voc++ == 0) {
                    printf("  [vocodeur-off] tick=%u : B_VOCODER_ON efface de d_ctrl_tch (voix par l'hote) -> %04x\n", fn, *ctrl);
                    fflush(stdout);
                }
            }
        }
    }
    prothese_facch_dl(api_ram);   /* see above */

    /* THE DEDICATED CHANNEL IS ALSO RELEASED WHEN THE ARM SEARCHES FOR SYNC.
     * The QEMU L1CTL tap does not always announce the release; the BSP would
     * then keep replacing TS0 on the channel frames -- for SS=0 those are
     * fn%%51 = 0..3, and fn%%51=0 carries the FCCH, fn%%51=1 the SCH -- and
     * the mobile would lose its sync for good.
     *
     * An FB (5) or SB (6) task posted by the ARM means it searches for sync on
     * TS0: it is no longer in dedicated mode, whatever the tap says. That
     * signal comes from the firmware itself. */
    {
        uint16_t md0 = api_ram[(API_W_PAGE(0) + WP_D_TASK_MD) / 2] & 0xff;
        uint16_t md1 = api_ram[(API_W_PAGE(1) + WP_D_TASK_MD) / 2] & 0xff;
        bool cherche_synchro = (md0 == FB_DSP_TASK || md0 == SB_DSP_TASK ||
                                md1 == FB_DSP_TASK || md1 == SB_DSP_TASK);
        if (g.dedie_arme && cherche_synchro) {
            printf("  [montant] tache %s postee : le mobile cherche la synchro, "
                   "canal dedie libere (TS0 rendu au FCCH/SCH)\n",
                   (md0 == FB_DSP_TASK || md1 == FB_DSP_TASK) ? "FB" : "SB");
            g.dedie_arme = false;
            g.sur_tch = false;
            g.sd_valide = false;
            calypso_bsp_set_dedie(0, 0xFF, 0);
            montant_canal_libere();
        }
    }
    sonde_afd(api_ram, fn);
    sonde_add(api_ram, fn);

    /* a_cd PROBE: block by block verdict of the dedicated downlink, the a_cd
     * status word (bit 15 = block present, bit 6 = Fire error) with the frame
     * and its position in the multiframe, to see whether failures follow the
     * SDCCH, the SACCH, or a particular position. MONTANT_ACD=0 disables
     * it. */
    {
        static int sonde = -1;
        if (sonde < 0) { const char *e = calypso_getenv("MONTANT_ACD"); sonde = (e && *e == '0') ? 0 : 1; }
        if (sonde && (g.dedie_arme || g_ti_dedie)) {
            static uint16_t prec;
            static unsigned long ok, ko;
            uint16_t etat = api_ram[(API_NDB + NDB_A_CD) / 2];
            /* TI dedicated mode: every block (etat != prec alone would hide consecutive FIRE KO) */
            static uint8_t d_prec[4];
            const uint8_t *d = (const uint8_t *)api_ram + API_NDB + NDB_A_CD + 6;
            bool nouveau = (etat != prec) || (g_ti_dedie && memcmp(d, d_prec, 4) != 0);
            if ((etat & B_BLUD) && nouveau) {
                memcpy(d_prec, d, 4);
                bool fire = (etat & 0x0040) != 0;
                if (fire) ko++; else ok++;
                if ((ok + ko) <= 60 || ((ok + ko) % 50) == 0) {
                    printf("  [a_cd] fn=%u p51=%u p102=%u etat=%04x %s "
                           "L2=%02x %02x %02x %02x | ok=%lu ko=%lu\n",
                           fn, fn % 51u, fn % 102u, etat, fire ? "FIRE KO" : "ok",
                           d[0], d[1], d[2], d[3], ok, ko);
                }
            }
            prec = etat;
        }
    }

    uint16_t *wp = &api_ram[API_W_PAGE(pg) / 2];
    uint16_t task_u  = taches ? wp[WP_D_TASK_U / 2] : 0;
    uint16_t task_ra = wp[WP_D_TASK_RA / 2];
    uint16_t d_rach  = api_ram[(API_NDB + NDB_D_RACH) / 2];

    /* DEDICATED UPLINK PROBE (Magnetite, classic BTS). At the first ALLC task
     * seen in a W page, 300 lines: tasks of both pages, d_dsp_page, chosen
     * page, a_cu header. And if a_cu carries B_BLUD without an uplink task
     * seen, it is taken anyway (the TI L1 posts it with the block to send,
     * l1ddsp_load_info). */
    {
        static unsigned reste;
        static bool arme;
        uint16_t td0 = api_ram[(API_W_PAGE(0) + WP_D_TASK_D) / 2] & 0x7fffu;
        uint16_t td1 = api_ram[(API_W_PAGE(1) + WP_D_TASK_D) / 2] & 0x7fffu;
        uint16_t cu0 = api_ram[(API_NDB + NDB_A_CU) / 2];
        if (!arme && (td0 == ALLC_DSP_TASK || td1 == ALLC_DSP_TASK)) {
            arme = true;
            reste = 300;
            printf("  [ul-dedie] premiere tache ALLC (tick %u) : sonde du montant sur 300 trames\n", fn);
        }
        if (reste) {
            reste--;
            printf("  [ul-dedie] tick=%u dsp_page=%04x pg=%u W0 d=%u u=%u W1 d=%u u=%u a_cu[0]=%04x\n", fn, v_page, pg,
                   td0, api_ram[(API_W_PAGE(0) + WP_D_TASK_U) / 2] & 0x7fffu,
                   td1, api_ram[(API_W_PAGE(1) + WP_D_TASK_U) / 2] & 0x7fffu, cu0);
        }
        if (arme && task_u == 0 && (cu0 & B_BLUD))
            capture_sdcch_ul(api_ram, 0, fn);
        /* Dedicated state of the TI firmware: active while an uplink task has
         * been seen in the last 102 frames. Arms the a_cd probe and the BSP
         * offset in dedicated mode (CALYPSO_BSP_DEDIE_DECAL). */
        /* Based on the UPLINK TASKS (d_task_u, 11/12): ALLC (24) also serves
         * the BCCH and the CCCH in idle, so it would trigger the offset in
         * idle too. */
        static uint32_t derniere_allc;
        static bool vu_allc;
        uint16_t tu0 = api_ram[(API_W_PAGE(0) + WP_D_TASK_U) / 2] & 0x7fffu;
        uint16_t tu1 = api_ram[(API_W_PAGE(1) + WP_D_TASK_U) / 2] & 0x7fffu;
        if (tu0 || tu1) { derniere_allc = fn; vu_allc = true; }
        bool dedie = vu_allc && (uint32_t)(fn - derniere_allc) < 102u;
        if (dedie != g_ti_dedie) {
            g_ti_dedie = dedie;
            calypso_bsp_set_dedie_ti(dedie);
        }
    }

    /* RACH: the task, not the value.
     *
     * The NDB word d_rach (byte 0x474 on the ARM side) is ALSO C54x memory
     * (data[0x0A3A]): the ROM leaves residue there (typically 0xfe00), so
     * triggering on the d_rach value would publish a bogus access burst
     * (ra=0xfe, bsic=0) at every start. The right signal is d_task_ra:
     * prim_rach.c:77 writes dsp_task_iq_swap(RACH_DSP_TASK=10, arfcn, 1)
     * there when it posts the RA, and nothing else is worth 10. Optionally
     * the word is cleared after publication (like
     * qosmo-dsp/calypso_trx.c:1955): the firmware never reads it back (only
     * prim_rach.c:77 and sync.c:307 write it), which gives a clean edge for
     * the next attempt.
     *
     * MONTANT_RACH_SUR_DRACH=1 selects the value trigger (d_rach transition,
     * first value taken as reference) for the case where something would
     * consume d_task_ra before this poll. */
    static int sur_drach = -1;
    if (sur_drach < 0) {
        const char *e = calypso_getenv("MONTANT_RACH_SUR_DRACH");
        sur_drach = (e && *e == '1') ? 1 : 0;
    }
    bool tache_rach = ((task_ra & 0x7fffu) == RACH_DSP_TASK);
    bool front_valeur = false;
    if (!g.base_rach) {
        g.base_rach = true;
        g.prev_rach = d_rach;
    } else if (d_rach != g.prev_rach) {
        front_valeur = (d_rach != 0);
        g.prev_rach = d_rach;
    }
    /* TASK EDGE when the ROM keeps the task (MONTANT_EFFACE_TASK_RA=0): nobody
     * clears d_task_ra then, it stays at RACH_DSP_TASK in the W page, and the
     * RACH_GARDE_TRAMES guard alone would republish the same RACH every 4
     * frames. Publication only happens on the edge (task absent -> present)
     * or on a new d_rach. MONTANT_RACH_DEDUP=0: publish on the guard alone. */
    static int front_seul = -1;
    static bool tache_avant;
    if (front_seul < 0) {
        const char *ea = calypso_getenv("MONTANT_EFFACE_TASK_RA");
        const char *ed = calypso_getenv("MONTANT_RACH_DEDUP");
        front_seul = (ea && *ea == '0') && !(ed && *ed == '0');
        if (front_seul)
            printf("  [montant] RACH publie sur le front de d_task_ra (MONTANT_RACH_DEDUP=0 : toutes les %d trames)\n",
                   RACH_GARDE_TRAMES);
    }
    bool nouveau_rach = !front_seul || !tache_avant || front_valeur;
    tache_avant = tache_rach;
    /* DEFERRED RACH (MONTANT_RACH_DIFFERE=N frames, Magnetite: 1 via run.sh).
     * The TI L1 posts d_task_ra BEFORE writing the d_rach of the attempt: read
     * at the edge, d_rach would be the one of the PREVIOUS attempt. The tick of
     * the edge is kept and d_rach is read N frames later. */
    static int differe = -1;
    static bool rach_attente;
    static uint32_t fn_attente;
    if (differe < 0) {
        const char *e = calypso_getenv("MONTANT_RACH_DIFFERE");
        differe = (e && *e) ? atoi(e) : 0;
        if (differe > 0) printf("  [montant] RACH differe de %d trame(s) : d_rach lu apres le front de d_task_ra\n", differe);
    }
    if (rach_attente && (uint32_t)(fn - fn_attente) >= (uint32_t)differe) {
        rach_attente = false;
        if (d_rach != 0)
            publier_rach((uint8_t)(d_rach >> 8), (uint8_t)((d_rach & 0xff) >> 2), fn_attente);
        g.prev_rach = d_rach;
    }
    if (d_rach != 0 && (tache_rach || (sur_drach && front_valeur)) && nouveau_rach &&
        (!g.rach_vu || (uint32_t)(fn - g.fn_rach) >= RACH_GARDE_TRAMES)) {
        if (differe > 0) { rach_attente = true; fn_attente = fn; }
        else publier_rach((uint8_t)(d_rach >> 8), (uint8_t)((d_rach & 0xff) >> 2), fn);
        g.prev_rach = d_rach;
        g.fn_rach = fn;
        g.rach_vu = true;
        if (g.rach <= (unsigned long)journal())
            printf("  [montant]   declencheur : %s (task_ra=0x%04x d_rach=0x%04x)\n",
                   tache_rach ? "d_task_ra=RACH_DSP_TASK" : "transition de d_rach",
                   task_ra, d_rach);
        static int consomme = -1;
        if (consomme < 0) {
            const char *e = calypso_getenv("MONTANT_CONSOMME_RACH");
            consomme = (e && *e == '1') ? 1 : 0;
        }
        if (consomme) {
            api_ram[(API_NDB + NDB_D_RACH) / 2] = 0;
            g.prev_rach = 0;
        }
        /* MONTANT_EFFACE_TASK_RA=1 clears d_task_ra in the W page: the ROM
         * never sees the RACH task (it would read it on the next frame) and
         * the bridge sends the access burst (side-band calypso_rach). */
        /* By default the ROM KEEPS the task: it encodes the access burst
         * (0x3f8a, 36 bits = gsm0503_rach_ext_encode) and its transmitter
         * serialises it in the TSP script, published by pont.c
         * (calypso_tx_rom). */
        static int efface_ra = -1;
        if (efface_ra < 0) {
            const char *e = calypso_getenv("MONTANT_EFFACE_TASK_RA");
            efface_ra = (e && *e == '1') ? 1 : 0;
            printf("  [montant] d_task_ra %s\n", efface_ra ? "efface avant la ROM (MONTANT_EFFACE_TASK_RA=1) : RACH code par le pont"
                                                           : "laisse a la ROM : elle code l'access-burst (MONTANT_EFFACE_TASK_RA=1 pour l'ancien comportement)");
        }
        if (task_ra && efface_ra) {
            wp[WP_D_TASK_RA / 2] = 0;   /* like qosmo-dsp/calypso_trx.c:1955 */
        }
    }

    /* SDCCH / SACCH / FACCH / speech: same dispatch as the gr-gsm layer 1. */
    if (task_u != 0 && !capture_tch_ul(api_ram, task_u, fn)) {
        capture_sdcch_ul(api_ram, task_u, fn);
    }

    publier_kc(api_ram);
}

/* The dedicated channel has just been released (PONT_DCCH, kind 0xFF): the
 * duplicate filter memory must restart, otherwise the SABM of the NEXT
 * connection, byte for byte identical to the previous one, would be taken
 * for a duplicate and never leave. */
void montant_canal_libere(void)
{
    g.sdcch_a_dernier = false;
    g_kc_liberer = true;   /* the Kc of THIS connection is worthless now */
}

void montant_bilan(void)
{
    if (g.rach || g.sdcch || g.facch || g.sacch || g.parole) {
        printf("  montant publie : RACH %lu, SDCCH %lu, FACCH %lu, SACCH %lu, parole %lu\n",
               g.rach, g.sdcch, g.facch, g.sacch, g.parole);
    } else {
        printf("  montant : rien publie (aucun d_rach ni d_task_u vu dans l'API RAM)\n");
    }
}
