/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * montant.c - le lien montant du montage DSP : RACH, SDCCH, SACCH, FACCH, parole.
 *
 * POURQUOI CE FICHIER EXISTE
 * --------------------------
 * Sous CALYPSO_DSP_EXTERN=1, calypso_l1_do_init() appelle calypso_l1_disable()
 * (« couche 1 « grgsm » desactivee (DSP externe) ») : plus aucune couche 1
 * n'est enregistree dans QEMU, donc calypso_l1_do_rach_written() et
 * calypso_l1_do_page_written() (calypso_l1_dispatch.c) sont des no-op. Or
 * c'etaient eux qui, en montage grgsm, publiaient le montant dans les
 * side-bands /dev/shm que pont.py consomme (pont/uplink.py) :
 *
 *   /dev/shm/calypso_rach           RACH        (ra, bsic)
 *   /dev/shm/calypso_sdcch_ul       SDCCH UL    (bloc L2 de 23 octets)
 *   /dev/shm/calypso_tch_facch_ul   FACCH UL
 *   /dev/shm/calypso_tch_sacch_ul   SACCH UL
 *   /dev/shm/calypso_tch_ul         parole (anneau de trames FR)
 *
 * Resultat mesure le 2026-09-21 : le firmware emettait bien ses
 * L1CTL_RACH_REQ, le mobile comptait ses « RANDOM ACCESS (requests left 8..4) »,
 * mais pont.py affichait « UL bursts=0 rach=0 » et /dev/shm/calypso_rach
 * n'existait meme pas. Sans RACH il n'y a pas d'IMM ASS, donc pas de SDCCH,
 * donc jamais de LOCATION UPDATING ACCEPT.
 *
 * COMMENT
 * -------
 * Ce processus tient l'API RAM partagee : on reprend donc, a l'identique, les
 * captures de qosmo-grgsm/hw/arm/calypso/calypso_l1_grgsm.c, mais declenchees
 * par SCRUTATION une fois par trame au lieu des callbacks d'ecriture de QEMU.
 * Le point d'appel (pont.c, fin du PONT_TICK) correspond a la fin du scenario
 * de l'ARM : dsp_end_scenario() vient d'ecrire d_dsp_page = B_GSM_TASK | page,
 * et les mots de tache de cette page W sont a jour.
 *
 * Le RACH est le seul cas ou la scrutation n'est pas equivalente a un
 * callback : le firmware ecrit d_rach (prim_rach.c:72) puis d_task_ra, et
 * personne ne les efface ensuite (sync.c:307 ne les remet a zero que sur
 * ABORT). On declenche donc sur FRONT : premiere valeur non nulle, ou valeur
 * differente de la precedente. Angle mort assume : deux tentatives de suite
 * avec le meme (RA, BSIC), soit ~1/256 puisque la RA est tiree au hasard par
 * gsm48_rr ; la tentative suivante passe. MONTANT_CONSOMME_RACH=1 remet d_rach
 * a zero apres publication, ce qui rend le declenchement exact — au prix d'une
 * ecriture dans la fenetre API que la ROM pourrait lire.
 *
 * Ce module ne parle pas TRXD : calypso_bsp_send_rach_ra() existe (code mort
 * depuis le refactor « couche 1 enregistree ») mais enverrait l'access-burst
 * sur 127.0.0.1:5702, c'est-a-dire la socket DESCENDANTE de pont.py, dont
 * run_data() fait « self.bts_data = addr » sur tout paquet recu : le burst
 * serait relu comme une descente et l'adresse de la BTS ecrasee. La voie des
 * side-bands passe par la machinerie montante de pont.py, celle qui est deja
 * eprouvee en montage grgsm.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include "hw/arm/calypso/calypso_api.h"
#include "hw/arm/calypso/calypso_debug.h"
#include "montant.h"

#define SHM_RACH        "/dev/shm/calypso_rach"
#define SHM_SDCCH_UL    "/dev/shm/calypso_sdcch_ul"
#define SHM_FACCH_UL    "/dev/shm/calypso_tch_facch_ul"
#define SHM_SACCH_UL    "/dev/shm/calypso_tch_sacch_ul"
#define SHM_TCH_UL      "/dev/shm/calypso_tch_ul"

/* Tailles et dispositions : pont/uplink.py. */
#define REC_RACH        16      /* lu 12 : seq(4) ra(1) bsic(1) ..(2) fn(4)   */
#define REC_L2          48      /* lu 39 : seq(4) l1s(4) fn(4) task(2) . p51(1) . l2(23) */
#define TCH_UL_SLOTS    16
#define TCH_UL_SLOT_SZ  64      /* TCH_UL_SLOT       */
#define TCH_UL_FR_OFS   16      /* TCH_UL_FR_OFS     */
#define FR_BYTES        33

/* Fenetre de recherche de l'en-tete L2 dans a_cu, et anti-doublon SDCCH :
 * memes valeurs que calypso_l1_grgsm.c (SDCCH_UL_WINDOW_OFS, SDCCH_UL_DEDUP_TICKS). */
#define SDCCH_UL_WINDOW_OFS  6
#define SDCCH_UL_DEDUP_TRAMES 60

/* Nombre de trames minimum entre deux RACH publies : une tentative du mobile
 * dure plusieurs trames et le meme d_rach reste lisible entre-temps. */
#define RACH_GARDE_TRAMES 4

static struct {
    unsigned long rach, sdcch, facch, sacch, parole;
    uint16_t prev_rach;
    uint32_t fn_rach;
    bool     rach_vu;      /* au moins un RACH publie              */
    bool     base_rach;    /* la valeur de reference a ete prise   */
} g;

static int journal(void)
{
    static int n = -1;
    if (n < 0) {
        const char *e = calypso_getenv("MONTANT_DEBUG");
        n = (e && *e) ? atoi(e) : 20;   /* les 20 premiers evenements, par defaut */
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
    /* L'entete (le compteur d'ecriture) en dernier : pont.py lit d'abord
     * l'entete, puis la case ; l'inverse lui livrerait une case a moitie ecrite. */
    sb_ecrire(fd, buf, sizeof(buf), 8 + (off_t)((seq - 1) % TCH_UL_SLOTS) * TCH_UL_SLOT_SZ);
    sb_ecrire(fd, &seq, 4, 0);
}

/* Bloc montant depose par le firmware dans le NDB : mot 0 = en-tete (B_BLUD
 * signale « bloc pret »), donnees a partir du mot 3. Les 33 octets de parole
 * sont ranges octet fort d'abord, les 23 octets L2 octet faible d'abord. */
static bool prendre_ul(uint16_t *api_ram, unsigned off, uint8_t *out, int n)
{
    uint16_t *w = &api_ram[(API_NDB + off) / 2];
    if (!(w[0] & B_BLUD)) {
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
    w[0] &= (uint16_t)~B_BLUD;   /* consomme, comme le ferait le DSP */
    return true;
}

static bool capture_tch_ul(uint16_t *api_ram, uint16_t task_u, uint32_t fn)
{
    static int fd_facch = -2, fd_sacch = -2;
    static uint32_t seq_facch, seq_sacch;
    uint8_t l2[23], fr[FR_BYTES];

    switch (task_u & 0x7FFF) {
    case TCHT_DSP_TASK:
        if (prendre_ul(api_ram, NDB_A_FU, l2, 23)) {
            publier_l2(&fd_facch, SHM_FACCH_UL, &seq_facch, l2, task_u, fn);
            if (g.facch++ < (unsigned long)journal())
                printf("  [montant] FACCH UL fn=%u task=0x%04x\n", fn, task_u);
        }
        if (prendre_ul(api_ram, NDB_A_DU_1, fr, FR_BYTES)) {
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

/* SDCCH montant : le firmware laisse le bloc L2 dans a_cu, a un decalage qui
 * depend de la version de l'API. On cherche l'en-tete LAPDm plausible dans une
 * fenetre de 7 octets, comme la couche 1 gr-gsm, puis on deduplique : la meme
 * tache d_task_u reste lisible tant que le firmware ne la reecrit pas. */
static void capture_sdcch_ul(uint16_t *api_ram, uint16_t task_u, uint32_t fn)
{
    static int fd = -2;
    static uint32_t seq;
    static uint8_t dernier[23];
    static uint32_t derniere_trame;
    static bool a_dernier;

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
    if (a_dernier && !memcmp(dernier, l2, 23) &&
        (uint32_t)(fn - derniere_trame) < SDCCH_UL_DEDUP_TRAMES) {
        return;
    }
    memcpy(dernier, l2, 23);
    derniere_trame = fn;
    a_dernier = true;
    if (l2[1] == 0x03) {      /* trame vide (UI sans donnee) */
        return;
    }
    publier_l2(&fd, SHM_SDCCH_UL, &seq, l2, task_u, fn);
    if (g.sdcch++ < (unsigned long)journal())
        printf("  [montant] SDCCH UL fn=%u task=0x%04x L2=%02x %02x %02x %02x %02x %02x\n",
               fn, task_u, l2[0], l2[1], l2[2], l2[3], l2[4], l2[5]);
}

static void publier_rach(uint8_t ra, uint8_t bsic, uint32_t fn)
{
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
    if (g.rach++ < (unsigned long)journal())
        printf("  [montant] RACH ra=0x%02x bsic=%u fn=%u -> %s\n", ra, bsic, fn, SHM_RACH);
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

    /* Quelle page W porte les taches ? dsp_end_scenario() (firmware,
     * calypso/dsp.c:471) ecrit d_dsp_page = B_GSM_TASK | w_page AVANT de
     * basculer w_page : le mot du NDB est donc la source fraiche, y compris
     * quand l1_sync() a tourne entre le TICK et le GO. L'argument `page` (le
     * d_dsp_page que QEMU avait echantillonne au TICK) ne sert que de repli. */
    uint16_t v_page = api_ram[(API_NDB + NDB_D_DSP_PAGE) / 2];
    bool taches = (v_page & B_GSM_TASK) != 0;
    unsigned pg = taches ? ((v_page & B_GSM_PAGE) ? 1u : 0u) : (page & 1u);

    uint16_t *wp = &api_ram[API_W_PAGE(pg) / 2];
    uint16_t task_u  = taches ? wp[WP_D_TASK_U / 2] : 0;
    uint16_t task_ra = wp[WP_D_TASK_RA / 2];
    uint16_t d_rach  = api_ram[(API_NDB + NDB_D_RACH) / 2];

    /* RACH : la tache, pas la valeur.
     *
     * [2026-09-21, mesure sur le banc reel] Le mot NDB d_rach (octet 0x474
     * cote ARM) est AUSSI de la memoire du C54x (data[0x0A3A]) : la ROM y
     * laisse du residu. Echantillonnage a 4 ms pendant 90 s, une ligne par
     * changement :
     *
     *     d_rach   W0.task_ra W1.task_ra   occurrences
     *     0xfe00   0x0000     0x0000       3219   <- residu de la ROM
     *     0x0d54   0x000a     0x0000          1   <- vraie tentative
     *     0x0954   0x0000     0x000a          1   <- vraie tentative
     *     0x0c54   0x0000     0x000a          1   <- vraie tentative
     *
     * Declencher sur la valeur de d_rach publiait donc un access-burst bidon
     * (ra=0xfe, bsic=0) a chaque demarrage. Le signal juste est d_task_ra :
     * prim_rach.c:77 y ecrit dsp_task_iq_swap(RACH_DSP_TASK=10, arfcn, 1) au
     * moment ou il pose la RA, et rien d'autre ne vaut 10. On efface le mot
     * apres publication (comme qosmo-dsp/calypso_trx.c:1955) : le firmware ne
     * le relit jamais (seuls prim_rach.c:77 et sync.c:307 l'ecrivent) et c'est
     * ce qui donne un front propre a la tentative suivante.
     *
     * MONTANT_RACH_SUR_DRACH=1 retablit l'ancien declencheur (transition de
     * d_rach, premiere valeur prise comme reference) pour le cas ou quelque
     * chose consommerait d_task_ra avant cette scrutation. */
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
    if (d_rach != 0 && (tache_rach || (sur_drach && front_valeur)) &&
        (!g.rach_vu || (uint32_t)(fn - g.fn_rach) >= RACH_GARDE_TRAMES)) {
        publier_rach((uint8_t)(d_rach >> 8), (uint8_t)((d_rach & 0xff) >> 2), fn);
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
        if (task_ra) {
            wp[WP_D_TASK_RA / 2] = 0;   /* comme qosmo-dsp/calypso_trx.c:1955 */
        }
    }

    /* SDCCH / SACCH / FACCH / parole : meme aiguillage que la couche 1 gr-gsm. */
    if (task_u != 0 && !capture_tch_ul(api_ram, task_u, fn)) {
        capture_sdcch_ul(api_ram, task_u, fn);
    }
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
