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
#include "calypso_bsp.h"
#include "montant.h"

#define SHM_RACH        "/dev/shm/calypso_rach"
#define SHM_SDCCH_UL    "/dev/shm/calypso_sdcch_ul"
#define SHM_FACCH_UL    "/dev/shm/calypso_tch_facch_ul"
#define SHM_SACCH_UL    "/dev/shm/calypso_tch_sacch_ul"
#define SHM_TCH_UL      "/dev/shm/calypso_tch_ul"
#define SHM_KC          "/dev/shm/calypso_kc_l1"

/* Tailles et dispositions : pont/uplink.py. */
#define REC_RACH        16      /* lu 12 : seq(4) ra(1) bsic(1) ..(2) fn(4)   */
#define REC_L2          48      /* lu 39 : seq(4) l1s(4) fn(4) task(2) . p51(1) . l2(23) */
#define TCH_UL_SLOTS    16
#define TCH_UL_SLOT_SZ  64      /* TCH_UL_SLOT       */
#define TCH_UL_FR_OFS   16      /* TCH_UL_FR_OFS     */
#define FR_BYTES        33

/* Fenetre de recherche de l'en-tete L2 dans a_cu (SDCCH_UL_WINDOW_OFS de
 * calypso_l1_grgsm.c). */
#define SDCCH_UL_WINDOW_OFS  6

/* Anti-doublon SDCCH montant.
 *
 * [2026-09-21, mesure] La couche 1 gr-gsm republiait un bloc identique passe
 * 60 trames (SDCCH_UL_DEDUP_TICKS). Repris tel quel ici, ca tuait la
 * connexion : le firmware laisse son bloc dans a_cu, on le republiait, le
 * pont le reemettait, et le BSC repondait
 *
 *   lchan(0-0-1-SDCCH8-0){ESTABLISHED}: ERROR INDICATION
 *     cause=SABM frame with information not allowed in this state
 *
 * -- un deuxieme SABM sur un lien deja etabli. Le canal tombait, le MSC
 * passait en MSC_A_ST_RELEASING et repondait LOCATION UPDATING REJECT au
 * milieu de la procedure, apres avoir pourtant mene l'IDENTITY REQUEST et
 * l'AUTHENTICATION REQUEST a bien.
 *
 * Donc : un bloc n'est publie QUE si son contenu change. MONTANT_SDCCH_REPETE
 * = N retablit une republication du meme bloc au bout de N trames (0 = jamais,
 * le defaut). Une retransmission LAPDm du mobile porte les memes octets et
 * serait donc avalee ; c'est le compromis assume, l'inverse casse le lien a
 * coup sur. */
#define SDCCH_UL_REPETE_DEFAUT 0

/* Nombre de trames minimum entre deux RACH publies : une tentative du mobile
 * dure plusieurs trames et le meme d_rach reste lisible entre-temps. */
#define RACH_GARDE_TRAMES 4

static struct {
    unsigned long rach, sdcch, facch, sacch, parole;
    uint16_t prev_rach;
    uint32_t fn_rach;
    bool     rach_vu;      /* au moins un RACH publie              */
    bool     base_rach;    /* la valeur de reference a ete prise   */
    /* Dernier bloc SDCCH montant publie, pour l'anti-doublon. */
    uint8_t  sdcch_dernier[23];
    uint32_t sdcch_trame;
    bool     sdcch_a_dernier;
    bool     blud_vu;          /* a_cu a deja annonce un bloc par B_BLUD   */
    unsigned sans_blud;        /* taches montantes vues sans B_BLUD        */
    bool     dedie_arme;
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

/* SDCCH / SACCH montant.
 *
 * [2026-09-21] Le firmware ANNONCE son bloc, il n'y a rien a deviner :
 * prim_tx_nb.c:80-101 ecrit dans a_cu l'en-tete `(1 << B_BLUD)`, deux mots a
 * zero, puis les 23 octets L2 a partir du mot 3 -- la disposition exacte que
 * prendre_ul() sait lire. Le drapeau est a usage unique : on le consomme, et
 * un bloc = une publication.
 *
 * Avant d'avoir lu ce code, cette fonction reprenait la fenetre heuristique de
 * la couche 1 gr-gsm (balayage d'en-tete LAPDm dans a_cu+6) avec un anti-
 * doublon sur le contenu. Trois echecs de suite en sont sortis : republication
 * du meme SABM toutes les 60 trames -> « SABM frame with information not
 * allowed in this state » et canal casse en pleine procedure ; puis
 * comparaison sur 23 octets dont la friture de fin bouge -> 32 blocs
 * « neufs » ; puis verrou « un seul SABM par connexion » -> plus aucun SABM
 * des que le verrou restait arme. Le drapeau du firmware rend tout ca inutile.
 *
 * MONTANT_SDCCH_FENETRE=1 force l'ancienne voie heuristique, et elle prend le
 * relais toute seule si B_BLUD ne se leve jamais alors que le firmware pose
 * des taches montantes (le cas ou la ROM consommerait le drapeau avant nous).
 */
/* PEREMPTION DE L'ANTI-DOUBLON (voie heuristique seulement).
 *
 * [2026-09-21] La couche 1 gr-gsm republie un bloc identique passe 60 ticks
 * (calypso_l1_grgsm.c:673). Ce n'est PAS un moteur de renvoi : dans ce
 * montage-la, QEMU efface d_task_u a chaque tick (calypso_trx.c, branche non
 * pont), donc la couche 1 ne voit une tache montante que sur les trames ou le
 * firmware vient de la poser. Les 60 ticks ne font qu'empecher de publier
 * quatre fois le meme bloc (un par burst) tout en laissant passer une VRAIE
 * retransmission du mobile, quand son T200 le fait re-poster.
 *
 * Sur la voie B_BLUD ce probleme n'existe pas : le drapeau est a usage unique,
 * une pose = une publication, et une retransmission du mobile repose le
 * drapeau. Rien a temporiser.
 *
 * Avoir lu ces 60 ticks comme un renvoi a coute un banc : le SABM repartait
 * apres l'etablissement du lien, et le BTS repondait « SABM frame with
 * information not allowed in this state » -- 4 ERROR INDICATION pour 4
 * ESTABLISHED. */
#define SDCCH_TTL_DEFAUT      60   /* trames, comme SDCCH_UL_DEDUP_TICKS */

static int sabm_ttl(void)
{
    static int v = -1;
    if (v < 0) {
        const char *e = calypso_getenv("MONTANT_SDCCH_TTL");
        if (!e || !*e) {
            e = calypso_getenv("MONTANT_SDCCH_REPETE");   /* ancien nom */
        }
        v = (e && *e) ? atoi(e) : SDCCH_TTL_DEFAUT;
    }
    return v;
}

static void publier_sdcch(uint16_t task_u, uint32_t fn, const uint8_t *l2)
{
    static int fd = -2;
    static uint32_t seq;
    if (l2[1] == 0x03) {      /* trame vide (UI sans donnee) */
        return;
    }
    publier_l2(&fd, SHM_SDCCH_UL, &seq, l2, task_u, fn);
    if (g.sdcch++ < (unsigned long)journal()) {
        printf("  [montant] SDCCH UL fn=%u task=0x%04x L2=", fn, task_u);
        for (int k = 0; k < 23; k++) printf("%02x%s", l2[k], k == 22 ? "" : " ");
        printf("\n");
    }
}

/* L'ancienne voie : balayage de la fenetre, anti-doublon sur le contenu
 * utile, republication apres MONTANT_SDCCH_REPETE trames (0 = jamais). */
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
            return;               /* le drapeau fonctionne : rien a publier */
        }
        /* Jamais vu B_BLUD alors que le firmware pose des taches montantes :
         * la ROM le consomme peut-etre avant nous. On bascule sur la fenetre. */
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

/* Le canal dedie, lu directement dans le side-band que le tap L1CTL de QEMU
 * ecrit (calypso_dcch_tap.c) et que pont.py lit deja.
 *
 * [2026-09-21, mesure] Le canal etait annonce au DSP par un message du pont,
 * PONT_DCCH. Trace du banc : QEMU imprime bien « [dcch] canal dedie arme :
 * chan_nr=0x51 SDCCH/8 SS=2 TN=1 », et cote DSP, RIEN -- ni « pont : canal
 * dedie », ni « [BSP] canal dedie arme », ni message inconnu. Le message se
 * perd dans le pas-a-pas en deux phases (la boucle d'attente du PONT_GO jette
 * tout ce qui n'est pas un GO). Resultat : le BSP continuait de livrer TS0
 * pendant que l'ARM ecoutait TS1, et TOUS les blocs de la descente dediee
 * echouaient au code de Fire.
 *
 * Le fichier, lui, ne depend d'aucun protocole : une lecture de 8 octets par
 * trame, et le meme numero de sequence que pont.py utilise pour savoir s'il a
 * change. */
static void scruter_dcch(uint32_t fn)
{
    static int fd = -2;
    static uint32_t seq;
    static uint32_t prochain_essai;

    if (fd < 0) {
        if (fn < prochain_essai) {
            return;
        }
        prochain_essai = fn + 200;          /* ~1 s entre deux tentatives */
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
    printf("  [montant] canal dedie (side-band seq=%u) : %s TS%d SDCCH/%d SS=%d\n",
           seq, genre == 0xFF ? "libere" : "arme", tn, genre == 1 ? 8 : 4, ss);
    calypso_bsp_set_dedie(genre == 0xFF ? 0 : tn, genre, ss);
    g.dedie_arme = (genre != 0xFF);
    if (genre == 0xFF) {
        montant_canal_libere();
    }
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
    /* Une tentative d'acces, c'est une nouvelle connexion : la memoire de
     * l'anti-doublon de la voie heuristique repart (sans effet sur la voie
     * B_BLUD, qui n'en a pas besoin). */
    g.sdcch_a_dernier = false;
    if (g.rach++ < (unsigned long)journal())
        printf("  [montant] RACH ra=0x%02x bsic=%u fn=%u -> %s\n", ra, bsic, fn, SHM_RACH);
}

/* ── LE Kc : LE PONT NE PEUT NI CHIFFRER NI DECHIFFRER SANS LUI ────────────
 *
 * [2026-09-22] Meme cause que tout ce fichier : sous CALYPSO_DSP_EXTERN=1 la
 * couche 1 gr-gsm est desactivee, et c'etait ELLE qui publiait
 * /dev/shm/calypso_kc_l1 (calypso_l1_grgsm.c, publish_kc). En montage DSP le
 * fichier n'existait donc pas, `Cipher.current()` de pont.py rendait None, et
 * `cipher.apply()` rendait le burst INCHANGE dans les deux sens -- releve sur
 * le banc : « A5 dl=0 ul=0 » a chaque STATS.
 *
 * Ce que ca coutait, mesure du 2026-09-22 avec ENCRYPTION="a5 1" : la
 * transaction allait jusqu'au bout de l'authentification en clair, puis
 *
 *     11:26:51  CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)
 *     11:26:51  CIPHERING MODE COMPLETE (cr 1)
 *     11:26:53  Dropping frame with 96 bit errors   (et sans fin ensuite)
 *
 * -- la descente chiffree par la BTS que personne ne dechiffre, et la montee
 * que personne ne chiffre. En « a5 0 » la meme transaction va au bout. Il n'y
 * a pas non plus d'A5 dans le modele Calypso (`d_a5mode` n'existe que dans
 * l1-grgsm/, rien dans l1-dsp/) : c'est bien au pont de le faire, comme il
 * fait deja le codage de canal.
 *
 * Disposition reprise telle quelle de publish_kc() pour que pont/cipher.py
 * (KC_RECLEN=32) la lise sans changement : seq(4) algo(1) longueur(1)
 * Kc[8] 0xFF. Les quatre mots de a_kc sortent en gros-boutiste ET a l'envers,
 * comme dans l'original -- on ne "corrige" pas une disposition que le lecteur
 * attend.
 *
 * MONTANT_KC=0 coupe la publication. */
#define KC_RECLEN        32
#define KC_PUBLIER_TOUTES 22   /* trames entre deux scrutations, comme grgsm */
#define KC_GRACE_CLAIR    5    /* cf. publish_kc : le firmware efface d_a5mode
                                * a chaque DM_REL_REQ, y compris quand le Kc
                                * revient juste apres (Assignment Command),
                                * alors que la BTS, elle, chiffre toujours. */

/* Leve par montant_canal_libere() : la prochaine scrutation doit publier le
 * retour en clair SANS attendre la grace. Voir publier_kc(). */
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
    /* [2026-09-22] LE CHANGEMENT DE MODE NE PEUT PAS ATTENDRE LA SCRUTATION.
     * Version precedente : on ne lisait d_a5mode qu'une trame sur 22 (~128 ms).
     * Or le mobile bascule des qu'il traite le CIPHERING MODE COMMAND et emet
     * son CIPHERING MODE COMPLETE dans la foulee ; un bloc SDCCH montant tombe
     * toutes les 51 trames. Ce bloc-la -- le PREMIER message chiffre du montant
     * -- pouvait donc partir en clair alors que la BTS le dechiffrait deja.
     * Releve du 2026-09-22, run de 11:49 :
     *     fn=2438  01 64 35  06 32 17 ...   CIPHERING MODE COMPLETE
     *     fn=2591  01 74 35  06 32 17 ...   LE MEME, retransmis (bit P)
     * la BTS ne l'acquittait pas, LAPDm (fenetre de 1) restait bloque dessus,
     * le TMSI REALLOCATION COMPLETE n'etait jamais emis et le MSC repondait
     * LOCATION UPDATING REJECT alors que le mobile se croyait a jour.
     * On lit donc d_a5mode a CHAQUE trame -- deux acces memoire -- et la
     * scrutation complete n'est differee que tant que le mode ne change pas. */
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
        /* [2026-09-22] Ce retour anticipe doit CONSOMMER g_kc_liberer, sinon la
         * liberation reste armee indefiniment : chaque scrutation suivante
         * calcule bascule=vrai, republie le meme enregistrement et incremente
         * `seq`. Cote pont.py, un `seq` qui bouge veut dire « nouvelle cle » :
         * il rechargeait sans fin une cle inchangee. */
        g_kc_liberer = false;
        return;
    }
    /* [2026-09-22] LA GRACE NE DOIT PAS SURVIVRE A LA LIBERATION DU CANAL.
     * Elle vient de publish_kc() et sert au cas INTRA-connexion : le firmware
     * efface d_a5mode a chaque DM_REL_REQ, y compris pendant un Assignment
     * Command ou le Kc revient juste apres, alors que la BTS chiffre toujours.
     * Mais entre DEUX connexions elle est nuisible : l'enregistrement algo=1
     * restait lisible cinq scrutations de plus, et pont.py -- qui relache
     * pourtant sa cle a chaque IMMEDIATE ASSIGNMENT (downlink.py) -- la
     * relisait aussitot dans le fichier et la restaurait. Il chiffrait alors
     * le montant de la connexion SUIVANTE des son premier bloc, pendant que le
     * mobile emettait encore en clair.
     * Mesure du 2026-09-22, run de 11:53 : « A5 dl=0 ul=200 » -- tout le
     * montant chiffre, rien de descendant dechiffre -- et l'AUTHENTICATION
     * RESPONSE (fn=2525, `05 14`) retransmise a fn=2729 faute d'acquittement.
     * La liberation du canal est le bon signal, et il existe deja :
     * montant_canal_libere(), appele sur PONT_DCCH genre 0xFF. */
    if (!algo && a_dernier && dernier[4] && !g_kc_liberer &&
        ++clair_en_attente < KC_GRACE_CLAIR) {
        return;   /* chiffre -> clair : on attend, le Kc revient peut-etre */
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

    scruter_dcch(fn);

    /* [2026-09-21] LE CANAL DEDIE SE LIBERE AUSSI QUAND L'ARM RECHERCHE LA
     * SYNCHRO. Le tap L1CTL de QEMU n'annonce pas toujours la liberation ;
     * mesure : /dev/shm/calypso_bsp_dedie a garde « tn=1 ss=0 » bien apres la
     * fin de la communication. Le BSP continuait donc de remplacer TS0 sur les
     * trames du canal -- et pour SS=0 ce sont fn%%51 = 0..3, or fn%%51=0 porte
     * la FCCH et fn%%51=1 la SCH. Le mobile perdait sa synchro pour de bon :
     * « FBSB RESP: result=255 » en boucle, d_fb_det=0, DSP parque.
     *
     * Une tache FB (5) ou SB (6) postee par l'ARM veut dire qu'il cherche la
     * synchro sur TS0 : il n'est plus en mode dedie, quoi qu'en dise le tap.
     * Ce signal-la vient du firmware lui-meme. */
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
            calypso_bsp_set_dedie(0, 0xFF, 0);
            montant_canal_libere();
        }
    }

    /* [2026-09-21] QUI RATE, ET QUAND. La descente dediee perd environ un bloc
     * sur deux (« Dropping frame with 110 bit errors », fire_crc >= 2 cote
     * layer23) alors que pont decode les MEMES blocs sans une seule erreur
     * (TS1/0:38/0 TS1/32:14/0) et que le BSP livre tous les bursts
     * (manques=0). C'est donc la demodulation dans la ROM qui flanche, pas la
     * plomberie. Cette sonde donne le verdict bloc par bloc : le mot d'etat de
     * a_cd (bit 15 = bloc present, bit 6 = erreur de Fire) avec la trame et sa
     * position dans la multitrame, de quoi voir si l'echec suit le SDCCH, la
     * SACCH, ou une position particuliere. MONTANT_ACD=0 la coupe. */
    {
        static int sonde = -1;
        if (sonde < 0) { const char *e = calypso_getenv("MONTANT_ACD"); sonde = (e && *e == '0') ? 0 : 1; }
        if (sonde && g.dedie_arme) {
            static uint16_t prec;
            static unsigned long ok, ko;
            uint16_t etat = api_ram[(API_NDB + NDB_A_CD) / 2];
            if ((etat & B_BLUD) && etat != prec) {
                const uint8_t *d = (const uint8_t *)api_ram + API_NDB + NDB_A_CD + 6;
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

    publier_kc(api_ram);
}

/* Le canal dedie vient d'etre libere (PONT_DCCH, genre 0xFF) : la memoire de
 * l'anti-doublon doit repartir a zero, sinon le SABM de la connexion SUIVANTE,
 * octet pour octet identique au precedent, serait pris pour un doublon et ne
 * partirait jamais. */
void montant_canal_libere(void)
{
    g.sdcch_a_dernier = false;
    g_kc_liberer = true;   /* le Kc de CETTE connexion ne vaut plus rien */
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
