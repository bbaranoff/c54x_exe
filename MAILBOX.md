# Boîte aux lettres ARM <-> DSP (Calypso API RAM)

Deux vues de la MÊME mémoire : l'ARM l'adresse en octets depuis 0xFFD00000,
le DSP en mots depuis 0x0800. Conversion : `mot_dsp = 0x0800 + octet_arm/2`.

Sources : `qosmo/include/hw/arm/calypso/calypso_api.h` et
`osmocom-bb/src/target/firmware/include/calypso/dsp_api.h` — vérifiés
concordants (`#define DSP 36`, branche a_serv_demod/a_pm/a_sch).

## Plan général

    zone          ARM (octets)   DSP (mots)   taille
    W page 0      0xFFD00000     0x0800       20 mots   ARM -> DSP
    W page 1      0xFFD00028     0x0814       20 mots
    R page 0      0xFFD00050     0x0828       20 mots   DSP -> ARM
    R page 1      0xFFD00078     0x083C       20 mots
    NDB           0xFFD001A8     0x08D4      268 mots   partagé, non paginé

## Page W (ARM -> DSP), offsets en mots

    +0  d_task_d      0x0800 / 0x0814
    +1  d_burst_d
    +2  d_task_u
    +3  d_burst_u
    +4  d_task_md     0x0804 / 0x0818   <- la mission : 5=FB 6=SB 8/9=TCH
    +8  d_fn          0x0808 / 0x081C
    +15 d_afc         0x080F / 0x0823   <- DAC AFC relayé au TWL3025
    +16 d_ctrl_system 0x0810 / 0x0824

## Page R (DSP -> ARM), offsets en mots

    +0   d_task_d
    +8   a_serv_demod[4]  0x0830 / 0x0844   TOA, PM, ANGLE, SNR
    +12  a_pm[3]          0x0834 / 0x0848
    +15  a_sch[5]         0x0837 / 0x084B   <- en-tête + charge utile SB
         a_sch[0] statut : bit15 B_BLUD (bloc présent), bit8 B_SCH_CRC (1=ERREUR)
         a_sch[3..4] : le mot SB, sb = a_sch[3] | a_sch[4]<<16
                       BSIC = (sb>>2) & 0x3f

## NDB (partagé)

    +0    d_dsp_page      0x08D4   0=reset, 2=armé page0, 3=armé page1
    +14   d_dsp_state     0x08E2
    +36   d_fb_det        0x08F8   non nul = FCCH trouvée
    +37   d_fb_mode       0x08F9   0=recherche large, 1=étroite
    +38   a_sync_demod[4] 0x08FA   TOA / PM / ANGLE / SNR
                          0x08FA TOA, 0x08FB PM, 0x08FC ANGLE, 0x08FD SNR

## Qui écrit quoi — côté ARM (osmocom-bb, vérifié en source)

    dsp_end_scenario()   calypso/dsp.c:471
        d_dsp_page = B_GSM_TASK | w_page ; puis w_page ^= 1
    l1s_reset_hw()       layer1/sync.c:158
        d_dsp_page = 0 ; r_page = 0 ; db_r -> R page 0
        appelé par prim_fbsb.c:254 et :429, donc à CHAQUE cycle FBSB raté
    l1s_dsp_post()       layer1/sync.c:263
        memset de la page R, puis a_sch[0] = (1<<B_SCH_CRC) = 0x0100,
        puis r_page ^= 1
    -> l'ARM n'écrit a_sch[0] qu'avec 0x0100. Rien d'autre.

## Qui écrit quoi — côté DSP (ROM masque, PC relevés à l'adresse exacte)

    0xb446   a_sch[0..4] <- 0 sur LES DEUX pages        initialisation
    0xaba2   a_sch[0] <- 0x1111                         marqueur
             précédé de 0xaba0 : LD #0x1111, A
             encodage 80e2 000f = store *(AR2 + 0x0f), AR2 = base page R
    0xb214   a_sch[0..4] <- statut + charge utile       RÉSULTAT
             statut observé : 0x8100 (BLUD + CRC faux) uniquement
    0xb2cc   d_fb_det <- 0        entrée de tâche FB : remise à zéro
    0xb2cf   a_sync[TOA] <- 0
    0xb2d2   a_sync[PM] <- 0
    0xb2d5   a_sync[ANGLE] <- 0
    0xb2d8   a_sync[SNR] <- 0
    0xb2c4   *(0x3fb5) <- 0x0cce   pointeur du tampon d'entrée du corrélateur FB
    0xb2c9   *(0x3fb5) <- 0x0d2e   idem, second tampon
    0x795a   a_sync[TOA] <- valeur
    0x798b   a_sync[ANGLE] <- valeur
    0x798d   a_sync[SNR] <- 0x4000
    0x79d4   a_sync[SNR] <- valeur
    0x79de   a_sync[PM] <- valeur

## Séquence d'une tâche SB

    ARM : d_task_md = 6 sur la page W courante
    ARM : d_dsp_page = 2|w_page       (dsp_end_scenario)
    DSP : 0xb446 met a_sch à zéro
    DSP : 0xaba2 pose le marqueur 0x1111
    DSP : 0xb214 écrit le résultat, a_sch[0] = 0x8100 (CRC faux)
    ARM : lit db_r->a_sch, voit B_SCH_CRC=1, conclut « SB not found »
    ARM : memset page R, a_sch[0] = 0x0100, r_page ^= 1
    ARM : après 2 tentatives, L1CTL_RESET_REQ -> l1s_reset_hw -> d_dsp_page = 0

## Pièges rencontrés

- Deux chemins d'écriture côté émulateur : `data_write()` et
  `data_write_locked()` (c54x_mem.c). Une sonde posée sur un seul en rate la
  moitié — a_sch passe par le second, a_sync_demod par le premier.
- L'ARM et le DSP sont dans DEUX PROCESSUS qui partagent ce mapping. Une sonde
  qui compare avant/après une instruction DSP peut imputer au DSP une écriture
  de l'ARM. Toujours relever l'adresse passée en argument, jamais un delta.
- d_dsp_page annonce la page d'ÉCRITURE de la tâche. La page de LECTURE est
  `dsp_api.r_page`, un compteur distinct côté ARM. Les deux ne sont pas liés.

## Pourquoi le TOA ne vaut jamais 23 [2026-09-19]

Le firmware vise un ToA de 23 (`prim_fbsb.c:207 last_fb->toa -= 23`) et ne le
voit jamais. Mesure sur 49 stores de `a_sync[TOA]` par la ROM en 0x795a, le mot
stocké valant exactement `B` et `T` valant 48 sur tous :

    AR2 = 0x0cce  (tampon d'entree du correlateur FB)   26 stores
       valeurs : 48 x10, 96 x8, 144 x4, 192 x2, 240, 384
       multiples exacts de 48 : 26/26 = 100 %

    AR2 = autre pointeur                                 23 stores
       valeurs : 27, 31, 43, 47, 51, 55, 71, 79, 87, 91
       multiples de 48 : 0/23 = 0 %

Separation parfaite, aucun recouvrement. Et 48 echantillons complexes = 96 mots
= UNE PAGE DMA (ALGTH=192 octets, mesure "en 4 page(s)" de 96 mots pour un
burst de 296).

Quand le correlateur lit le tampon de burst, son pic tombe donc sur une
frontiere de page DMA, a tous les coups. 23 est a l'interieur d'une page : il
n'appartient pas a l'ensemble des valeurs atteignables. Ce n'est pas un
probleme de seuil ni de rapport signal/bruit.

    0x7953 :  770e 0030    store de la constante 48 -> T
    0x795a :  81f8 08fa    STH A, *(0x08fa)  -> a_sync[TOA], vaut B

A rapprocher de calypso_rhea_dma.c:490, qui decrit le mode defaillant du
chainage de pages comme "corrupted FCCH, correlator peaking at the edge
(TOA=39)". Le correctif "pages contigues" a fait tomber la proportion de
multiples de 48 de 86,6 % (CALYPSO_RHEA_DMA_PINGPONG=1) a 46,4 %, sans
l'eliminer : il reste une couture a la granularite de la page.

## Chaine complete : l'angle mort commande le doublement d'horloge [2026-09-19]

    a_sync[ANGLE] rend ~0 quel que soit l'offset injecte (pente nulle sur +-2000 Hz)
      -> prim_fbsb.c:312  freq_diff = ANGLE_TO_FREQ(angle) ~ 0
      -> prim_fbsb.c:488  if (abs(freq_diff) < freq_err_thresh2 && snr > FB1_SNR_THRESH)
                          mesure a chaud : thresh2 = 800 Hz (0x008320b2),
                          FB1_SNR_THRESH = 0 (les seuils 2000/3000 sont sous #if 0),
                          SNR = 0x4000 ecrit par la ROM en 0x798d
                          -> 175/180 detections passent le garde, soit 97 %
      -> prim_fbsb.c:492  synchronize_tdma() sur le chemin FB, rien ne repose
                          l'horloge derriere
      -> prim_fbsb.c:345  fnr_delta = fnr_report - attempt, fnr_report etant une
                          COPIE de current_time.fn et attempt valant 1 ou 2
      -> prim_fbsb.c:348  cinfo->fn_offset = fnr_delta   (un ABSOLU)
      -> sync.c:149       l1s_time_inc(current_time, fn_offset)  -> horloge doublee
      -> bursts normaux EMPTY -> L1CTL_RESET_REQ -> le cycle FBSB repart

L'autre appel, prim_fbsb.c:231 (chemin SB), est inoffensif : gsm_fn2gsmtime()
repose l'horloge en absolu trois lignes plus loin. D'ou l'echec du SB force : il
repare une fois par cycle ce que le chemin FB casse a chaque detection.

Mesure a chaud confirmant le doublement, par le moniteur QEMU :

    current_time.fn   fn_offset    rapport
       64797           129070       1.992     fn_offset gele entre deux FB
       65499           129070       1.971
       65565           131012       1.998     rafraichi a ~2x l'horloge

Adresses de surveillance (xp/1wx sur /tmp/qemu-monitor-pont.sock) :

    l1s.current_time.fn         0x0083762c
    l1s.serving_cell.arfcn      0x00837644   = 514
    l1s.serving_cell.bsic       0x00837646   = 42 avec PONT_CAN_SB
    l1s.serving_cell.fn_offset  0x00837648   doit rester petit ; vaut ~2x l'horloge
    fbs.req.band_arfcn          0x008320ac   = 514
    fbs.req.freq_err_thresh2    0x008320b2   = 800

## Le chemin SB lit d'AUTRES cellules que le chemin FB [2026-09-19]

    read_fb_result()  prim_fbsb.c:306   dsp_api.ndb->a_sync_demod[]   NDB    0x08FA..0x08FD
    read_sb_result()  prim_fbsb.c:148   dsp_api.db_r->a_serv_demod[]  page R 0x0830 / 0x0844

Deux groupes distincts. Toute mesure faite sur l'un ne dit rien de l'autre.

Les quatre resultats du SB sont publies par la ROM en 0xb1e7..0xb1f5 depuis
quatre cellules de travail, avec ANGLE et SNR CROISES par rapport a l'ordre
des adresses :

    0x3fa4 -> a_serv_demod[TOA]     store 0xb1e9
    0x3fa5 -> a_serv_demod[PM]      store 0xb1ed
    0x3fa7 -> a_serv_demod[ANGLE]   store 0xb1f1
    0x3fa6 -> a_serv_demod[SNR]     store 0xb1f5
    la cellule 0x3fa6 est elle-meme ecrite en PC=0x7e88

## L'AFC est sous le seuil, pas morte

    AFC_SNR_THRESHOLD = 2560   (afc.h:4)
    AFC_PERIOD        = 40     AFC_MIN_MUN_VALID = 8   (afc.c)

    mesure sur 111 publications de a_serv_demod[SNR] :
       SNR > 2560 : 17 = 15,3 %   -> 6,1 mesures valides par fenetre de 40
       il en faut 8. Distribution BINAIRE : 16384 (0x4000) ou un ou deux chiffres.

    runavg_check_output (avg.c:34) ne remet PAS les compteurs a zero quand le
    minimum n'est pas atteint, donc l'AFC finit par emettre -- vers 52 trames.
    Or un cycle FBSB dure 53,6 trames en moyenne et afc_reset() remet le DAC a
    -700 a chaque l1s_reset_hw(). Les deux echeances sont quasi identiques :
    la correction arrive au moment ou elle est effacee.

## Ce qui est DISCULPE par la mesure

- Le tampon du correlateur 0x0cce : quand une vraie FCCH y est, le DSP la voit
  parfaitement -- coherence 0.999, dphi +1.565 pour +1.571 theorique (mesure
  fn=2060, p51=20). Le signal arrive et l'estimateur le reconnait.
- Le chemin de resultat SB de bout en bout : avec PONT_CAN_SB, le firmware
  extrait BSIC=42 sur 14 acceptations sur 14, et le stocke en
  l1s.serving_cell.bsic (0x00837646). Ecriture des deux pages, handshake
  r_page, read_sb_result, desassemblage T1/T2/T3' : tout fonctionne.
- L'AFC comme cause de l'echec SB : CALYPSO_TWL3025_AFC=0 ne change ni le
  verdict CRC ni la quantification du TOA.

## Retractations de la session

- "le DSP n'ecrit jamais a_sch" : faux, sonde posee sur un seul des deux
  chemins d'ecriture (data_write vs data_write_locked).
- "le maximum du SNR est 864, 100 % des mesures AFC invalides" : faux,
  echantillon tronque. Le maximum est 16384 et 15,3 % passent le seuil.
- "l'estimateur d'angle est mort" : vrai pour le NDB (chemin FB), faux pour
  a_serv_demod (chemin SB) ou il prend des valeurs variees.
- "le doublement d'horloge est benin car ecrase par la pose absolue" : faux,
  l'ordre reel est inverse sur le chemin FB (prim_fbsb.c:492).

## Pourquoi le TOA vaut 1251 en vivant et pas en rejeu [2026-09-20]

Rejeu (`--rejouer`) : le TOA du FB1 croit avec la distance de la FCCH dans la
fenetre (6288, 7584, 8832, 9991, 11243 = 5 a 9 trames), ntdma est juste, la SB
est decodee : 56 CRC OK sur 3000 trames, 56/56 avec le BSIC injecte, T3 valide
et FN = trame du burst. Balayage BSIC 0/7/13/21/42/63 : `BSIC=(sb>>2)&0x3f`
colle a chaque fois. La sortie depend de l'entree.

Vivant (`run.sh`) : TOA = 1251 (ou 1247, 1296) a CHAQUE detection, quelle que
soit la distance de la FCCH. Cause, dans qemu.log :

    [trx] pont DSP : DSP en retard, tick saute (fn=5548, 4340 sauts, 1202 trames jouees)

Le C54x emule coute 6,7 ms par trame (mesure : 500 trames de rejeu en 3,35 s)
contre 4,615 ms de temps reel GSM. QEMU cadence le TDMA a l'horloge murale et
saute la trame quand le DONE du DSP n'est pas arrive : 3 trames sur 4 perdues,
la ROM ne recoit qu'une trame sur 4 a 6 (transferts DMA en paquets de 13 paires
aux fn 252, 258, 264, 270, 276...). Son compteur de blocs FB n'avance donc que
d'une trame environ entre la commande et la FCCH : TOA ~ 1250 + quelques
echantillons, ntdma = 0, fn_offset faux, la SB visee tombe 1 a 2 trames apres
la trame SCH, CRC KO a tous les coups.

Correctif : `CALYPSO_PONT_LOCKSTEP=1` (calypso_trx.c, existait deja) - QEMU
n'avance la trame que quand le DSP a fini la precedente. `run.sh` l'exporte
par defaut en montage dsp (`LOCKSTEP=0` pour revenir a l'horloge murale).
Mesure en pas-a-pas : plus aucun saut (trames=1953 a fn=1951), TOA = 11239 a
11243 (ntdma=8, delay=10, comme en rejeu), et dans osmocon.log :

    SB1 (1089343:1): TOA=   24, Power= -52dBm, Angle= -152Hz
    => SB 0x001001a8: BSIC=42 fn=3826(2/ 4/ 1) qbits=4

BSIC 42 = celui de la cellule synthetique, TOA 24 pour un attendu de 23 : la
ROM a decode une vraie SB sur le chemin vivant, ARM reel + DSP reel.

## Les SB « delirantes » : une page R lue a zero [2026-09-20]

Symptome (osmocon.log) :

    SB1 (1649571:1): TOA=    0, Power=-138dBm, Angle=    0Hz
    => SB 0x00000000: BSIC=0 fn=52(0/ 0/ 1) qbits=4908

Le mot SB vaut 0, TOA 0, puissance -138 dBm (a_serv_demod a zero aussi) : l'ARM
a lu une page R que la ROM venait de remettre a zero en 0xb446 (init de tache,
LES DEUX pages) et sur laquelle aucun resultat n'etait encore ecrit.
`l1s_sbdet_resp` ne teste que B_SCH_CRC (bit 8) : 0x0000 passe pour un CRC OK
et BSIC=0 / FN=52 sont pris pour argent comptant. Le firmware se cale alors sur
un FN faux, lit les bursts normaux n'importe ou et le mobile jette tout
(`Dropping frame with 210 bit errors` : ~46 % d'erreurs sur 456 bits, du hasard).
Vu dans qemu.log comme `page0 0100->0000` suivi d'une lecture ARM de la page 0.

Deux populations de CRC OK, a distinguer par le mot lui-meme :

    sb = 0, TOA = 0, PM = -138 dBm    -> page vide, faux positif
    sb != 0, TOA ~ 23, PM ~ -52 dBm   -> vraie SB (BSIC 42 ici)

En rejeu le faux positif n'existe pas : l'ARM rejoue lit a la fin de la trame,
apres l'ecriture du DSP. Il n'apparait qu'avec l'ARM QEMU, dont la lecture peut
tomber entre la remise a zero et le resultat, ou sur l'autre page R quand les
bascules r_page (ARM) et page du DSP se sont desynchronisees par des trames
sautees. Le pas-a-pas reduit le second cas ; le premier reste a mesurer.

Le mobile apres une SB acceptee : avec `PONT=0` la cellule synthetique n'a que
FCCH, SCH et bursts factices ; les trames BCCH decodees sont donc du bruit
(`Dropping frame with N bit errors`, N ~ 190-224) meme quand la SB est vraie.
Pour aller au-dela il faut le BTS via pont.py (`PONT=1`) ou des bursts BCCH
(SI1-4) dans cellule.c.

## Etat du banc ISA [2026-09-20]

`make isa_test && ./isa_test tools/isa_tests.txt` : 208 exemples SPRU172C,
139 ok, 69 FAIL, 22 non assembles. Une partie des FAIL vient d'attendus mal
extraits du PDF (ex. `LD *AR4+, A` attend un AR5 qui n'intervient pas), le
reste sont de vrais ecarts (RETF, RPTB, SUBC, MVDP/MVPD, NEG/RND/SFTA sur les
drapeaux). Aucun n'empeche le decodage SB observe ci-dessus.
