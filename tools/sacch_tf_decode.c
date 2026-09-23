/* sacch_tf_decode.c - decode hors DSP la SACCH/TF jouee par le BSP.
 *
 * Lit /dev/shm/calypso_sacch_tf.bin (calypso_bsp.c, sonde [sacch_tf] :
 * enregistrements de 156 octets = tick BE32, fn BE32, 148 bits 0/1), regroupe
 * les bursts en blocs selon 45.002 (TCH/F, bloc du TN a fn%104 = 12 + 26*TN/2
 * modulo 104, puis +26, +52, +78) et les passe a gsm0503_xcch_decode, en clair
 * et dechiffres avec le Kc de /dev/shm/calypso_kc_l1.
 *
 * Si la SACCH decode ici et pas dans la ROM, le defaut est dans le chemin
 * BSP -> ROM ; si elle ne decode pas ici non plus, dans ce que le BSP recoit.
 *
 * ./sacch_tf_decode [fichier] [TN=2] [Kc en hexa, A5/1 ; defaut : calypso_kc_l1]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <osmocom/core/bits.h>
#include <osmocom/gsm/a5.h>
#include <osmocom/coding/gsm0503_coding.h>

#define MAXB 1024
static struct { uint32_t tick, fn; uint8_t b[148]; } r[MAXB];

static int kc_lire(uint8_t *algo, uint8_t kc[8])
{
    FILE *f = fopen("/dev/shm/calypso_kc_l1", "rb");
    uint8_t b[32];
    if (!f) return -1;
    size_t n = fread(b, 1, sizeof b, f);
    fclose(f);
    if (n < 14) return -1;
    *algo = b[4];
    memcpy(kc, b + 6, 8);
    return (*algo >= 1 && *algo <= 3) ? 0 : -1;
}

static int trouver(int n, uint32_t fn)
{
    for (int i = 0; i < n; i++) if (r[i].fn == fn) return i;
    return -1;
}

int main(int argc, char **argv)
{
    const char *nom = argc > 1 ? argv[1] : "/dev/shm/calypso_sacch_tf.bin";
    int tn = argc > 2 ? atoi(argv[2]) : 2;
    FILE *f = fopen(nom, "rb");
    if (!f) { perror(nom); return 1; }
    int n = 0;
    uint8_t rec[156];
    while (n < MAXB && fread(rec, 1, 156, f) == 156) {
        r[n].tick = (uint32_t)rec[0] << 24 | rec[1] << 16 | rec[2] << 8 | rec[3];
        r[n].fn = (uint32_t)rec[4] << 24 | rec[5] << 16 | rec[6] << 8 | rec[7];
        memcpy(r[n].b, rec + 8, 148);
        n++;
    }
    fclose(f);
    uint8_t algo = 0, kc[8] = { 0 };
    int a_kc = kc_lire(&algo, kc) == 0;
    if (argc > 3 && strlen(argv[3]) == 16) {
        for (int i = 0; i < 8; i++) sscanf(argv[3] + 2 * i, "%2hhx", &kc[i]);
        algo = 1; a_kc = 1;
    }
    printf("%d bursts, Kc %s", n, a_kc ? "" : "absent");
    if (a_kc) printf("A5/%u %02x%02x%02x%02x%02x%02x%02x%02x", algo, kc[0], kc[1], kc[2], kc[3], kc[4], kc[5], kc[6], kc[7]);
    printf("\n");

    unsigned debut = (12 + 26 * (tn / 2)) % 104;   /* TN2 : 38 */
    int ok[2] = { 0, 0 }, vus = 0;
    for (int i = 0; i < n; i++) {
        if (r[i].fn % 104 != debut) continue;
        int idx[4];
        int complet = 1;
        for (int k = 0; k < 4; k++) {
            idx[k] = trouver(n, r[i].fn + 26 * k);
            if (idx[k] < 0) complet = 0;
        }
        if (!complet) continue;
        vus++;
        printf("bloc fn=%u tick=%u :", r[i].fn, r[i].tick);
        for (int chiffre = 0; chiffre < 2; chiffre++) {
            if (chiffre && !a_kc) break;
            sbit_t sb[464];
            for (int k = 0; k < 4; k++) {
                uint8_t b[148];
                memcpy(b, r[idx[k]].b, 148);
                if (chiffre) {
                    ubit_t dl[114], ul[114];
                    osmo_a5(algo, kc, r[idx[k]].fn, dl, ul);
                    for (int j = 0; j < 57; j++) { b[3 + j] ^= dl[j]; b[88 + j] ^= dl[57 + j]; }
                }
                for (int j = 0; j < 58; j++) {
                    sb[k * 116 + j] = b[3 + j] ? -127 : 127;
                    sb[k * 116 + 58 + j] = b[87 + j] ? -127 : 127;
                }
            }
            uint8_t l2[23];
            int nerr = 0, nbits = 0;
            int rc = gsm0503_xcch_decode(l2, sb, &nerr, &nbits);
            if (rc == 0) ok[chiffre]++;
            printf("  %s rc=%d err=%d/%d L2=%02x %02x %02x %02x %02x %02x", chiffre ? "dechiffre" : "tel-quel",
                   rc, nerr, nbits, l2[0], l2[1], l2[2], l2[3], l2[4], l2[5]);
        }
        printf("\n");
    }
    printf("blocs complets=%d  bons tel-quel=%d  bons dechiffres=%d\n", vus, ok[0], ok[1]);
    return 0;
}
