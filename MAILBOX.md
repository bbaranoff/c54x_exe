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

## Romload fige a 38-55 % en pas-a-pas [2026-09-20]

Symptome : osmocon reste sur `Progress: 38%` (ou 55 %), QEMU dit pourtant que
le firmware charge par `-kernel` a deja lance son TDMA. Cause : c'est
`tdma_tick` (calypso_trx.c) qui pompe le pty serie vers l'UART emulee
(`calypso_uart_poll_backend`). Sous `CALYPSO_PONT_LOCKSTEP=1`, quand le DSP
n'a pas fini la trame precedente, le tick sortait AVANT ce pompage et se
rearmait : la serie n'avancait qu'au rythme du DSP (~7 ms par trame) et le
romload, qui a un delai par bloc, decrochait. Les runs precedents passaient
de justesse. Correctif (qosmo 77f61dd) : l'UART est pompee aussi sur le
chemin d'attente du DSP. Mesure : 71 blocs, « your code is running now »,
puis FBSB_REQ dans la foulee.

## L'interruption trame du DSP est un bit a usage unique [2026-09-20]

Etat de depart, en pas-a-pas : timing FB juste (TOA 11239/11243, delay=10),
tache SB postee sur la trame SCH avec le burst S livre, et pourtant 1 SB en
130 cycles. Sondes :

    D_TASK_MD-RD : la ROM lit d_task_md a CHAQUE trame (0xb011 -> 0xb554 ->
                   0xb0b4 -> 0xab7a pour FB, -> 0xaba4 pour SB) et relit la
                   MEME page avec la meme valeur 3 a 4 trames de suite.
    PONT_PC_COUNT : aba4 (dispatch SB) 27 passages / 217 trames, b219 : 0.
    DMA2 (PC ajoute aux journaux) : armements par 0xa5ef/0xa5f6, desarmement
                   0xa646 ; en vivant, flux continu FB rearme a CHAQUE FCCH
                   (trames 204, 214, 234, 244...), jamais de fenetre 764.
    Rejeu : fenetre one-shot de 764 octets (382 mots) armee sur la trame de
                   la commande SB, et la FB1 (mode etroit) en une fenetre 764
                   a la trame predite, pas en flux continu.

Pourquoi la ROM relit la page : sync.c l1_sync() efface la page W COURANTE a
chaque trame (ligne 244) mais ne bascule w_page que si la trame porte un item
DSP (dsp_end_scenario, ligne 276). La page remise au DSP garde donc sa tache
tant qu'aucun nouveau scenario ne rebascule ; la ROM n'ecrit jamais d_task_md
ni d_dsp_page (WATCH-WR : 0 ecriture). Le rejeu, lui, bascule w_page a chaque
trame (rejouer.c l1_sync) : les pages y sont propres, d'ou le decodage.

Ce qui l'empeche sur silicium : dsp_end_scenario() appelle
tpu_dsp_frameirq_enable() (TPU_CTRL_DSP_EN) a CHAQUE scenario et personne ne
l'eteint jamais (tpu_frame_irq_en(1,1) seulement). Un bit qu'on rearme a
chaque fois est un bit a usage unique : le TPU ne donne l'interruption trame
au DSP que sur les trames ou l'ARM lui a remis une page. Le pont la levait a
chaque tick. qosmo ne modelisait pas ce bit (le commentaire de calypso_c54x.c
qui le pretend est perime : grep TPU_CTRL_DSP_EN ne donne que le #define).

Correctif : qosmo (calypso_trx.c) met dans TICK.b bit 16 « l'ARM a arme
DSP_EN depuis le tick precedent » et consomme le bit ; pont.c ne leve vec 28
que sur ce bit (PONT_IRQ_TRAME=1 pour l'ancien comportement). Verifie d'abord
en rejeu avec REJEU_IRQ_SCENARIO=1 : memes TOA, meme taux SB (27/76).

Mesure en vivant, 10 cycles FB1 :

    SB acceptees            : 5, toutes BSIC=42, TOA=24, -51 dBm   (avant : 1/130)
    fenetres one-shot 764   : 7 (380 mots), burst S (n_iq=380) sur p51 = 41/11/31
    flux FB continu         : plus rearme a chaque FCCH

Restes :
- FB1 lue vide : `FB1 (5294:8): TOA=0 Power=-138dBm` deux fois de suite, meme
  mecanisme que la SB delirante (a_sync pas encore ecrit), l'ARM retente.
- FB0 a l'essai 1 avec TOA=1296 : la premiere paire de pages porte encore une
  FCCH du flux precedent (src=GRILLE), detection immediate et fausse distance.
- Apres une SB vraie le mobile lit le BCCH et jette tout (`Dropping frame with
  208 bit errors`, `MM_EVENT_NO_CELL_FOUND`) : la cellule synthetique n'a ni
  SI1-4 ni bursts normaux. Etape suivante : bursts BCCH dans cellule.c, ou
  `PONT=1` avec le BTS.

## Banc BTS reel (pont.py --dsp-port 6702) : ce qui manquait [2026-09-20]

1. pont.py sans `--dsp-port 6702` : rien ne part vers le DSP (defaut 0 = coupe).
2. Le BSP n'appariait les bursts que par FN (fenetre +-64) : le FN du firmware
   est arbitraire avant la SB et saute de centaines de milliers a chaque
   detection FB (prim_fbsb.c, l1s_time_inc absolu). Rien n'etait livre.
   `CALYPSO_BSP_STREAM=1` livre dans l'ordre d'arrivee, c'est le chemin a
   utiliser des qu'une source temps reel alimente le DSP.
3. Le BSP ajoutait ses 7 timeslots de remplissage derriere chaque TS0 alors
   que pont.py envoie les 8 TS : 15 TS par trame (qosmo 6ad7003).
4. Le DSP consomme ~100 trames/s contre 217 emises : la file de 128 bursts
   par TS debordait chaque seconde et jetait les plus anciens ; espacements
   FCCH mesures 4, 26, 18 trames au lieu de 10, 88 SB tentees / 0 decodee.
   File portee a 8192 (qosmo 54d8320) : flux coherent, seulement en retard.
5. La livraison STREAM chargeait les bursts bruts (148 symboles) et rien pour
   les TS idle que le BTS n'envoie pas : trames de 1184 symboles ou moins, le
   compteur de la ROM derivait (offsets intra-trame du TOA FB : 264, 434,
   632, 399). Assembleur de trame (qosmo 28e9989) : 8 TS a 156/157 symboles,
   burst factice pour les absents, fenetre SB = TS0 + 21 de marge.

Vitesse du coeur (qosmo d68baaf, 54d8320) : getenv memoise, chemin rapide
sans sondes ni mutex par acces memoire. Rejeu 6,7 -> 2,5-3,0 ms/trame. Le
vivant reste vers 10 ms (24-38 k instructions de ROM par trame, ~200 ns
chacune, le corps de c54x_run porte encore des dizaines de comparaisons de
sonde par instruction) : lockstep toujours necessaire, et la file profonde
compense le retard sur le BTS.

## Bursts BCCH dans la cellule, ordre ARM/DSP dans la trame [2026-09-21]

cellule.c porte maintenant les bursts normaux du TN0 : SI1-4 sur le bloc BCCH
(p51 2..5, TC = (fn/51)%8 : SI1 0/4, SI2 1/5, SI3 2/6, SI4 3/7), paging vide
sur les CCCH (6..9, 12..15, 16..19), factice ailleurs. Codage libosmocoding
(gsm0503_xcch_encode) + assemblage sched_lchan_xcch.c d'osmo-bts, TSC = BCC,
identite MCC 001 MNC 01 LAC 1 CI 6001 ARFCN 514 (CELLULE_MCC/MNC/LAC/CI/ARFCN).
Verifie hors DSP : les 4 bursts reassembles redonnent les 23 octets exacts
(xcch_decode). Fenetre NB de la ROM : 151 echantillons (ALGTH 604), burst a
3 de marge (tpu_window.c L1_NB_MARGIN_Q), trame remplie a la longueur exacte.

Premier symptome cote firmware : `EMPTY`, `BURST ID 2!=1`, `3!=2`, `2!=0`
avec un decalage qui derive. Cause : dans le tick QEMU, le TICK partait au DSP
et l'IRQ trame a l'ARM en meme temps ; le DSP (un autre processus) ecrivait
la page R du burst N pendant que l1_sync(N) lisait encore le burst N-2, avec
un retard variable selon la charge de l'ARM. Ordre silicium mesure par sonde
(PONT_NB_DEBUG, [pgA]/[pgG]/[pgB]) : la ROM copie d_task_d/d_burst_d dans la
page R et arme la fenetre dans son ISR de trame, AVANT que l'ARM ne change
d_dsp_page ; le resultat est ecrit apres le burst, dans la meme trame.
Correctif (qosmo calypso_trx.c + calypso_inth.c, c54x_exe pont.c) : TICK en
deux phases. QEMU envoie TICK(N) (bit 17 CALYPSO_PONT_TICK_DEUX_PHASES), le
DSP joue l'ISR jusqu'a l'armement de la fenetre et repond DONE|PHASE_A ; QEMU
leve alors l'IRQ trame, attend la fin de l1_sync(N) (ecriture IRQ_CTRL bit 0
par irq() apres le handler, comptee par l'INTH quand IRQ_NUM valait 4), puis
envoie PONT_GO ; le DSP livre le burst et finit la trame. Cible EOI
cumulative, resynchronisee sur timeout (256 x 290 us). Mesure : 0 EMPTY,
0 BURST ID sur des dizaines de blocs, aucun timeout. CALYPSO_PONT_ARM_FIRST=0
revient a l'ancien ordre.

Ce qui reste, et ce qui a ete mesure sur la demodulation NB de la ROM :

- Les bits demodules sont dans data[0x2be2..+148] (sonde [scan] : signe
  positif = 1, 146-147/148 sur un bon burst, le dernier bit toujours faux :
  la ROM lit 150 echantillons). Par bloc, 1 a 3 bursts sortent parfaits, les
  autres a ~45 % d'erreurs avec un motif CONSTANT sur la sequence
  d'apprentissage ; TOA=3 SNR>0 quand c'est bon, TOA=5 SNR=0 quand c'est
  mauvais. Deterministe : meme burst, meme resultat d'un run a l'autre.
- Le meme burst repete aux 4 positions donne 4 resultats differents : ce
  n'est pas le contenu seul, l'etat interne de la ROM entre en jeu.
- Sans effet sur le partage bon/mauvais : amplitude (30000 -> 6000), moyenne
  nulle forcee, phase porteuse (0/22.5/45), TSC 0..7 dans le burst (2 = BCC
  attendu), marge 0..7 (seule 3 donne quelque chose), fenetre exacte.
- Instant d'echantillonnage : SEUL 0.5 symbole marche (balayage 2.3 a 4.7
  par pas de 0.1 : 3.4 et 3.6 echouent sur tous les bursts) ; MSK pur
  (diagonales exactes) echoue partout. Un egaliseur se degraderait en pente
  douce ; ici c'est un fil du rasoir.
- SNR rapporte par la ROM sur un burst PARFAIT : 91 a 600 (la SB rend
  16384). L'estimateur voit un gros residu meme quand les decisions sont
  justes.
- Le tampon DARAM (AAD 0x0cce, 302 mots) est identique aux echantillons
  livres juste apres le DMA ; la ROM y recrit ensuite les mots 1..29.
- La capture reelle (IQ=reelle, BSIC 48) : TOA stable 1-2, SNR 500-660,
  ~200 erreurs sur 456 a chaque bloc, le motif 0x9999 dans a_cd aussi.
- Coeur C54x : histogramme d'opcodes de la phase B (PONT_NB_HIST) : 12-14 k
  instructions par burst, 39 k sur le 4e (decodage). La ROM bascule OVM 20
  fois par burst, SAT 214 fois, NEG 720, SFTA 193, NORM 217. Le coeur
  n'implementait ni OVA/OVB ni la saturation OVM (le mot n'apparaissait
  qu'en commentaire) : ajoute en post-instruction (calypso_c54x.c,
  CALYPSO_C54X_OVM=0 pour revenir), plus RND src/dst, MIN/MAX (C=1 si egaux),
  SFTA (C = bit 32-SHIFT), SUBB (retenue inversee), retenue de ADD/SUB
  src,SHIFT,dst, OV efface une fois teste. Banc ISA : 139 -> 147 ok. Aucun
  effet sur le partage bon/mauvais des bursts.

Le decodage SB en rejeu (56 CRC OK / ~300 SCH) a le meme profil : une
demodulation qui reussit sur une fraction des bursts selon le contenu. Cause
commune probable, dans le coeur emule ou dans la forme du signal 1 ech/symbole
que la ROM attend de la chaine analogique ; a chercher sur le chemin SB, mieux
instrumente (rejouer.c), plutot que sur le NB.

Rejouer : `IQ=cell PONT_NB_DEBUG=1 ./run.sh` puis `grep -a '\[scan\]\|\[nb\]\|\[pg'
/tmp/c54x-pont/dsp.log` ; balayages CELLULE_NB_DEC=auto|x, CELLULE_NB_PHASE,
CELLULE_TSC=auto, PONT_NB_MARGE=auto, CELLULE_NB_FINE=1, CELLULE_NB_REPEAT=k,
CELLULE_NB_AMP, CELLULE_NB_ZERO_DC, CELLULE_NB_MSK ; PONT_NB_HIST=<dir>.

## Phase porteuse = temps, et le TOA 24 de la SB [2026-09-21, suite]

- Kill-switch OVM (CALYPSO_C54X_OVM=0) : partage bon/mauvais identique, drapeaux
  OVA/OVB/C/TC tous a zero a l'entree de chaque burst (sonde [zones]). L'ajout
  OVM n'est ni la cause ni un remede. Pas de fuite de drapeaux entre bursts.
- Pas d'inversion I/Q demandee par le firmware en reception (trf6151_iq_swapped
  rend 0 hors TX 850) : d_task_d = 24, sans le bit 0x8000.
- Balayage de la phase porteuse par quadrants (CELLULE_NB_PHASE=quad) : 0 deg
  = le partage habituel ; 90 et 270 = tout mauvais ; 180 = le burst bon ressort
  INVERSE (143/148). Marge 2 + 270 deg reproduit exactement marge 3 + 0 deg, et
  marge 2 + 90 deg rend le bloc SI2 parfait mais inverse. Pour la ROM un
  echantillon de decalage vaut 90 deg (rotation j^n du MSK) : sa demodulation
  est COHERENTE sur une reference de phase fixe, l'estimation de canal sur le
  TSC ne resout ni le quadrant ni le signe.
- Pourquoi la SB converge a 24 et non 23 : notre burst S est place a 21
  echantillons dans la fenetre, un de trop pour la geometrie que la ROM attend.
  IQ=cell:42:0.5:20 avec CELLULE_SB_PHASE=270 : TOA=23, qbits=0, 8/8 SB. Le
  NB n'en profite pas (marge 2 + 270 = marge 3 + 0).
- Traces d'execution (PONT_NB_HIST : trace_<fn>.txt, watch_<fn>.txt) : deux
  bursts 0 (bon 359, mauvais 410) suivent le MEME chemin jusqu'au pas 1698,
  une boucle argmax en 0x8551-0x8557 (MAX B, XC 1,NC, valeurs A ~0x3dc685d
  contre 0x3a0e498 : un profil PLAT a 6 % pres, pas un pic de correlation) ;
  c'est la que la position est choisie et qu'elle part a 5 au lieu de 3. Les
  « taps » ecrits en 0x2cd1.. (7 groupes) sont ensuite decales d'un mot dans
  le groupe pour les mauvais bursts. La reference 0x2b28+48 est lue depuis le
  tampon de burst a AR2 = 0x0d9f.. (mot 209, echantillon 104, un echantillon
  sur deux) — a comprendre en desassemblant 0x7ef4-0x8557 (PROM0).

Prochaine etape : desassembler la routine NB de la ROM autour de 0x8551
(argmax) et 0x81cd (lecture du tampon a un echantillon sur deux) pour savoir
quel profil elle attend a cet endroit ; les traces de 8 bursts sont dans le
repertoire donne a PONT_NB_HIST.

## Le BCCH ne decode pas : le quantifieur des soft bits sort +1 partout [2026-09-21, soir]

Etat : les 4 bursts d'un bloc BCCH sont demodules sans erreur (147/148 en
0x2be2, le dernier bit hors fenetre) une fois le burst elargi
(CELLULE_NB_SYM=0.3, cf. supra). Le bloc reste faux (Fire KO, a_cd = bruit).
La chaine apres l'egaliseur, lue dans les traces (PONT_NB_HIST) :

    0x75c9-0x75e2  division SUBC : ratio = (count << 15) / somme, count et
                   somme = statistiques du residu sur le TSC (0x7638-0x7697),
                   ex. 6 / 1523 -> 0x81 (129)
    0x8154-0x815e  echelle T = ((ratio << 10) >> 16) * 0x4eb8 << 2 >> 16 = 2
                   (ou 0 quand count = 0)
    0x8166-0x8177  soft_scaled = (rnd(T * soft) << 4 >> 1) >> 16, soft = +-3000..6800
                   -> 0..3, signe perdu
    0x82bd-0x82cd  table de 129 mots lue en PROM (reada 0x7b9e..) en 0x2a8e
    0x82d0-0x82dd  index = soft_scaled >> 8, dest = table[0x2ace + index]
                   -> index 0 partout -> +1 partout
    0x9a07/0x9a0a  stockage 4 bits par soft bit, 29 mots par burst en
                   0x4200 + 29 x burst : mesure 0x1111 sur tous les mots
    0x9a6a-0x9a76  desentrelacement par table + LUT (0x2c08) vers 0x2a00
    0x9a78-0x9aad  treillis 16 etats (dadst/dsadt/cmps), st TRN
    0x9ab8-0x9ad1  traceback (bitt/roltc), puis compaction 0x9ac5.., a_cd

Le treillis recoit donc des metriques (c0+c1, c0-c1) sans information et
sort un chemin d'egalites, juste sur les suites de zeros et faux ailleurs
(36 a 78 des 228 bits) : ce n'est PAS un defaut du desentrelaceur (test A
du dossier tools/cch_ref impossible tant que le bloc ne porte pas de signe :
aucun des trois candidats ne matche, 65-70 %).

Corriges au passage (isa_test 148 ok) : MPYR / MACR / MASR effacent les 16
bits bas apres l'arrondi (isa_test 132). Sans effet sur le bloc.

Bequille de diagnostic CALYPSO_HACK_SOFT_SCALE=k (c54x_exec.c, MPYR aux
trois sites de l'echelle) : x256 fait apparaitre quelques -4 dans le bloc et
change a_cd, sans decoder. Le facteur manquant est de l'ordre de 2^12 a
2^20 d'apres l'arithmetique ci-dessus, ce qui n'est pas plausible pour une
seule instruction : une des semantiques de la chaine (sfta/sth ASM/norm/exp
sur ces valeurs, ou le residu 0x7638 qui fixe count et somme) est lue de
travers par le coeur ou par moi. Prochaine etape : rejouer cette chaine
(entree 0x2be2 -> sortie 0x2a00) sur les valeurs des traces avec les
semantiques du manuel, instruction par instruction, jusqu'au premier ecart.

## Le BCCH decode : SI1-4, lai=001-01-1, le mobile campe [2026-09-21, apres-midi]

Le quantifieur n'etait pas en cause : quatre semantiques du coeur, toutes en
aval de l'egaliseur, lues de travers. Trouvees en rejouant chaque etage en
Python sur les traces (PONT_NB_HIST) jusqu'au premier ecart :

1. `add Xmem,Ymem,B` (0xA1xx) etait execute comme SQDST (qui est 0xE2xx) :
   A <- Ymem<<16, B += (AH-Xmem)^2. Site 0x8251 (fin du filtre 3 coefficients
   de l'egaliseur, 206 fois par burst) : la somme construite dans A etait
   remplacee par l'echantillon Q derotate. D'ou les "soft bits" propres mais
   inverses et decales d'un symbole (out[n] = -bit[n+2] au lieu de
   +bit[n+3]), la correlation TSC du residu nulle (count 6), l'echelle T=2 et
   les +1 partout. Aussi 0x8565, 0x81fa, 0x7f03, 0x7fab-0x80ec.
   (c54x_exec.c, bloc `hi8 == 0xA1` desactive.)
2. `*+ARx(lk)%` (mode 14) utilisait encore la grille "base = AR - AR % BK" :
   au 4e passage du desentrelaceur (0x9a15/0x9a34/0x9a59, BK=456,
   `mar *+AR4(57)%`), AR4 = 0x2aab+57 donnait 0x291c au lieu de 0x2ae4 et la
   moitie impaire du bloc partait sous le tampon. (c54x_decode.c ->
   c54x_circ_ref, comme les modes 8-11.)
3. `sfta A,1` posait C = bit 31 (regle SFTL) ; la LFSR du code de Fire
   (0xa168-0xa175, registre 40 bits dans A, generateur 0x04820009 via
   `xc C -> xor #0x0482,16,A ; xor @60(=9),A`) veut le bit 39 : syndrome nul
   sur un bloc juste seulement avec C = src(40-SHIFT). Le manuel dit
   src(39-SHIFT) et son exemple (80AA001234<<5 -> C=1) ne colle a aucune des
   deux ; isa_test 188 echoue desormais, la ROM a raison. Avant : FIRE1
   (a_cd[0]=0x8040) sur chaque bloc, "Dropping frame with N bit errors".
4. `and/or/xor src,SHIFT,dst` avec src=B (0xF2xx/0xF3xx) : operandes
   inverses (dst = src OP (dst<<SHIFT)) ; corrige en dst OP (src<<SHIFT)
   (audit qosmo-dsp vs l1-dsp). Sites 0x8ec7-0x8eca (repliement du CRC).

Verifications intermediaires (scratchpad repro.py / vit.py) : egaliseur
Python avec les coefficients de la ROM = +bit[n+3] 142/142 et = la sortie
ROM apres (1) ; nibbles 0x4200 = 456/456 via tools/cch_ref ; bloc 0x2a00 =
456/456 apres (2) ; mots TRN du treillis identiques au modele (228/228),
traceback -> 0 erreur sur u224 ; mots 0x2c3c.. exacts.

Attention, la reference u228 de cellule_u228_attendu() etait fausse : les
octets L2 se deplient LSB en premier (osmo_pbit2ubit_ext lsb_mode=1) avant
le Fire et le convolutif ; en MSB d'abord la sonde [u228] annoncait "78
faux" sur un bloc juste. Corrige ; la sonde [u228]/[bloc] reste a revoir
(elle lit 0x2d66, qui n'est pas la sortie).

Resultat (IQ=cell CELLULE_NB_SYM=0.3) : a_cd = 8000 06f9 0026 : 0631 001c
10f1 0100 4040 00e5 2b00 ... = SI4 LAI 001-01-1 ; mobile.log : New SYSTEM
INFORMATION 1/2/3/4, lai=001-01-1, 0 "Dropping frame", "We are camping
normally". Reste : a_cd[2] (0x26/0x35 "erreurs de bits") non nul sur un bloc
parfait, sans effet (fire_crc=0) ; isa_test 147 ok / 61 FAIL.

## Pas de LU ACCEPT : en montage DSP, le lien montant n'existait pas [2026-09-21, soir]

Symptome : le mobile campe (SI 1-4, `lai=001-01-1`, `CGI=001-01-1-6001`, cote
pont `DL bursts=2268 blocs=567 crc=0`), demande sa mise a jour de position,
puis tourne en rond :

    mobile.log  : CHANNEL REQUEST: 00 (Location Update with NECI)
                  RANDOM ACCESS (requests left 8..4), T3211 qui refire
    osmocon.log : L1CTL_RACH_REQ (ra=0x01, offset=9, combined=1, uic=0xff) x5
    pont.log    : UL bursts=0 tard=0 rach=0            <- rien ne remonte
    dsp.log     : aucune occurrence de RACH ni de UL
    /dev/shm/   : calypso_api_ram seul, pas de calypso_rach

Sans RACH, pas d'IMM ASS, donc pas de SDCCH, donc jamais de LU ACCEPT. La
descente n'etait pas en cause.

Cause, en trois ruptures sur le meme chemin :

1. `calypso_trx.c:325` voit bien l'ecriture de `d_rach` et appelle
   `calypso_l1_do_rach_written()`, qui relaie a `l1->rach_written`
   (`calypso_l1_dispatch.c:102`). Mais sous `CALYPSO_DSP_EXTERN=1`,
   `calypso_l1_do_init()` appelle `calypso_l1_disable()` (qemu.log :
   « couche 1 « grgsm » desactivee (DSP externe) ») : `l1 == NULL`, le hook
   est un no-op. Idem pour `_page_written` (d_task_u).
2. Cote `c54x_exe`, le BSP contient tout le necessaire —
   `calypso_bsp_tx_rach_burst()` (calypso_bsp.c:2487), `send_rach_ra()`
   (:2541), `send_ul()` (:2344), `tx_burst()` (:2397) — mais **personne ne
   les appelle** : code mort. Dans les arbres precedents les appels etaient
   cote QEMU (`qosmo-dsp/hw/arm/calypso/calypso_trx.c:1200` sur ecriture de
   d_rach, `:1945-1959` poll de d_task_ra/d_task_u par trame) ; le refactor
   « couche 1 enregistree » les a perdus et ils n'ont pas ete reportes dans le
   processus DSP.
3. `pont.py` n'a qu'une entree montante : les side-bands `/dev/shm`
   (`pont/uplink.py:13,134`), ecrits uniquement par la couche 1 gr-gsm
   (`qosmo-grgsm/.../calypso_l1_grgsm.c:738`) — justement celle qui est
   desactivee. `--dsp-port 6702` est unidirectionnel (`pont/trx.py:83-89`).

Correctif : `src/montant.c`, appele une fois par trame depuis le PONT_TICK de
`src/pont.c`. Il scrute l'API RAM partagee et alimente les memes side-bands
qu'en montage grgsm (RACH, SDCCH UL, FACCH, SACCH, parole), avec les captures
reprises telles quelles de la couche 1 gr-gsm (fenetre a_cu + 6 et son
heuristique d'en-tete LAPDm, `take_ul` sur B_BLUD, anneau TCH). La page W est
choisie sur le `d_dsp_page` frais du NDB (dsp_end_scenario ecrit
`B_GSM_TASK | w_page` avant de basculer), pas sur celui echantillonne au TICK :
`l1_sync()` tourne entre le TICK et le GO en mode deux phases.

Seul point ou la scrutation n'est pas equivalente au callback : le RACH. Le
firmware ecrit `d_rach` (prim_rach.c:72) puis `d_task_ra`, et rien ne les
efface (sync.c:307 ne le fait que sur ABORT). On declenche donc sur front de
`d_rach`, avec une garde de 4 trames. Angle mort : deux tentatives de suite
avec la meme RA (tiree au hasard par gsm48_rr, ~1/256) ;
`MONTANT_CONSOMME_RACH=1` remet `d_rach` a zero apres publication et rend le
declenchement exact. `MONTANT=0` coupe tout, `MONTANT_DEBUG=N` regle le nombre
d'evenements imprimes (20 par defaut).

Pourquoi les side-bands et pas TRXD : `calypso_bsp_send_ul()` emet vers
127.0.0.1:5702, c'est-a-dire la socket **descendante** de pont.py, dont
`run_data()` fait `self.bts_data = addr` sur tout paquet recu — le burst
montant serait relu comme une descente et l'adresse de la BTS ecrasee. Une
voie TRXD native demanderait d'abord un port montant dedie cote pont.

A cote, meme session : une SB annoncant `BSIC=7` alors que le BSC est a 21
signifie que le QEMU lance n'a **pas** `CALYPSO_DSP_EXTERN=1` (il ecoute alors
sur udp/4730-4731). La SB est alors fabriquee par le shunt gr-gsm a partir du
paquet `SCH2` de `pont/downlink.py:30`, avec `cfg.bsic` = 7 par defaut
(`pont/config.py:45`) : `PONT_BSIC=21`, ou le montage DSP.

## Le RACH passe, l'IMM ASS revient, le mobile la jette : la reference de requete [2026-09-21, nuit]

Avec `src/montant.c` en place, la boucle complete se mesure sur le banc reel
(sonde a 3 ms sur `/dev/shm/calypso_api_ram` et `/dev/shm/calypso_rach`) :

    22:07:26 RACH publie seq=2 ra=0x0d bsic=21
    22:07:29 >>> IMM ASS ra=0x0d  ref T1'=3 T2=9 T3=35
             brut = 2d 06 3f 03 41 a2 02 0d 1c 69 00 00 2b...
    22:07:30 >>> IMM ASS ra=0x04  ref T1'=3 T2=17 T3=23
    22:07:32 >>> IMM ASS ra=0x06  ref T1'=3 T2=2 T3=14

Donc : le RACH part, la BTS l'entend, le BSC ouvre un SDCCH, l'IMMEDIATE
ASSIGNMENT redescend sur l'AGCH, le DSP la decode et le bloc arrive dans
`a_cd` avec la BONNE RA. Et pourtant le mobile reste en `connection pending`,
`/dev/shm/calypso_sdcch_ul` n'est jamais cree (aucune tache `d_task_u`), et le
BSC compte douze `lchan allocation failed ... WAIT_RLL_RTP_ESTABLISH Timeout`
en quatre minutes.

Cause : `gsm48_match_ra()` (osmocom-bb `gsm48_rr.c:3359`) n'accepte une
assignation que si la RA **et** T1'/T2/T3 correspondent a ce que sa propre
couche 1 lui a confirme, et journalise sinon « request %02x matches but not
frame number ». Or le banc n'emet pas l'access-burst a la trame ou le firmware
a cru l'emettre : `pont.py` le programme sur SON horloge
(`pont/uplink.py:_poll_rach` -> `_next_fn(4, ...)`), plusieurs trames plus
tard, et la BTS horodate la reference avec cette trame-la.

Ce n'est pas une decouverte : la couche 1 gr-gsm contient deja le contournement
(`qosmo-grgsm/.../calypso_l1_grgsm.c:836-845`, `feed_agch()` reecrit les octets
8-9 de tout IMM ASS avec le `last_rach` du firmware, lu par symbole ELF). Sous
`CALYPSO_DSP_EXTERN=1` cette couche 1 est desactivee, donc plus personne ne le
faisait.

Correctif, cote QEMU cette fois (`hw/arm/calypso/calypso_trx.c`) :

- `calypso_l1_dispatch.c` retient le chemin de l'ELF passe a
  `calypso_l1_do_init()` et expose `calypso_firmware_symbol()` (table des
  symboles ELF32, reprise de la couche 1 gr-gsm). `last_rach` est GLOBAL a
  0x00837624 dans `layer1.highram.elf`.
- `api_write` retient la RA a chaque ecriture de `d_rach`, et
  `pont_rach_suivi()` releve `last_rach.fn` une fois par trame (sur l'ecriture
  de `d_dsp_page`, que `dsp_end_scenario()` fait exactement une fois par
  trame) pour tenir un historique **par RA**. C'est necessaire : les
  assignations reviennent dans le desordre (mesure ci-dessus : 0x0d, puis
  0x04, puis 0x06 alors que la derniere RA ecrite etait 0x0e).
- `api_read` intercepte la lecture ARM du seul mot concerne,
  `API_NDB + NDB_A_CD + 14` (octets 8-9 du bloc L2, la reference de requete),
  quand le bloc est bien `06 3f`, et rend la reference recalculee depuis la
  trame memorisee pour CETTE RA. Si aucune trame n'est connue pour elle, on ne
  corrige pas : fabriquer une correspondance serait pire que l'echec.
  `MONTANT_REQREF=0` coupe la correction.

Au passage, deux corrections sur le montant lui-meme :

- Le declencheur du RACH etait la valeur de `d_rach`. Faux : ce mot du NDB est
  aussi de la memoire du C54x (`data[0x0A3A]`) et la ROM y laisse du residu.
  Echantillonnage a 4 ms pendant 90 s : `d_rach = 0xfe00` avec `d_task_ra = 0`
  sur les deux pages W dans 3219 relevés, contre trois vraies tentatives
  (0x0d54, 0x0954, 0x0c54) portant toutes `d_task_ra = 0x000a`. Or
  `RACH_DSP_TASK = 10` (firmware `include/calypso/l1_environment.h:49`). Le
  declencheur est donc `d_task_ra`, et un access-burst bidon (ra=0xfe, bsic=0)
  partait vers la BTS a chaque demarrage. `MONTANT_RACH_SUR_DRACH=1` retablit
  l'ancien comportement.
- `calypso_bsp.c` imprimait une ligne `[ts0] tick=... NB fenetre=151` par
  trame livree : la condition `nwin > 0` est vraie pour tout burst normal
  depuis la DMA one-shot. Repliee derriere `CALYPSO_BSP_TS0_DEBUG=1`.
