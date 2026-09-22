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

## Le mobile passe en mode dedie, sur un intervalle que le DSP ne recoit pas [2026-09-21, nuit, suite]

Avec la reference de requete corrigee, la suite se deroule : le mobile accepte
l'assignation, passe en mode dedie et emet sa demande. Le bloc montant capture
dans `/dev/shm/calypso_sdcch_ul` est exactement celui qu'on cherchait :

    01 3f 49 | 05 08 70 00 f1 10 ff fe 30 08 09 10 10 00 10 00 00 10
    │  │  └── L = 18
    │  └───── SABM, P=1
    └──────── SAPI 0
              05 08 = MM / LOCATION UPDATING REQUEST, LAI 001-01
              LAC=0xfffe (efface), classmark 30, IMSI 001010001000001

Deux choses l'empechaient d'arriver a la BTS, toutes deux du meme genre que le
reste : un point de branchement que `calypso_l1_disable()` avait neutralise.

**1. Le canal dedie n'etait annonce a personne.** `pont.py` ne lit pas les
IMM ASS : sa classe `Dedicated` (`pont/state.py`) attend le canal dans
`/dev/shm/calypso_dcch_cfg`, et tant qu'il manque, `uplink.py:_poll_sdcch()`
jette tout le montant. Ce fichier etait ecrit par le tap L1CTL de la couche 1
gr-gsm (`l1-grgsm/calypso_l1ctl_tap.c`), branche par la vtable. Nouveau
`hw/arm/calypso/calypso_dcch_tap.c`, qui ne depend d'aucune couche 1 : il
renifle le flux sercomm du firmware, retient le `chan_nr` des
`L1CTL_DATA_CONF`/`DATA_IND` et publie. `calypso_l1_do_uart_tx_byte()`
l'appelle quand aucune L1 n'est enregistree ; `d_dsp_page = 0` libere.
Verifie sur le banc : `[dcch] canal dedie arme : chan_nr=0x51 SDCCH/8 SS=2 TN=1`.

**2. Le DSP ne recevait que TS0.** Le BSC alloue le SDCCH/8 sur **TS1** ; sous
`CALYPSO_BSP_STREAM=1`, `calypso_bsp.c` arretait TS1..TS7 a la reception et
`bsp_ts0_livrer()` completait la trame avec du bourrage a zero. Le mobile
n'entendait donc ni le UA ni le LU ACCEPT, d'ou le
`[dcch] canal dedie libere` immediat et le retour en `C1 normal cell
selection`. `g_bsp_tpu_offset`, la position de la fenetre RX relayee par QEMU,
etait stockee et jamais lue - et de toute facon TPU_OFFSET est le decalage de
synchro global, pas l'intervalle.

Correctif : QEMU sait desormais quel canal le mobile utilise (point 1), donc il
l'annonce au DSP par un nouveau message du pont, `PONT_DCCH` (a = TN,
b = genre, c = sous-voie), emis juste avant un TICK - jamais pendant l'attente
d'un `PONT_GO`, que le DSP ignorerait. Le BSP garde alors les bursts de CET
intervalle par numero de trame BTS (`bsp_dedie_stocker`, anneau de 2^16 trames
parce que le DSP en pas-a-pas derive de plusieurs secondes) et
`bsp_ts0_livrer()` les joue **a la place** de TS0. Un seul burst par tick,
donc le cadencement en 1250 symboles par trame n'est pas touche.

**Au passage, l'appariement RA -> trame.** Premiere version : relever
`last_rach.fn` une fois par trame et l'attribuer a la derniere RA ecrite.
Mesure : trois `IMM ASS ra=0x0c : aucune trame memorisee` de suite. En rafale,
le firmware ecrit le `d_rach` suivant avant que `last_rach` n'ait bouge pour le
precedent. Deuxieme version, exacte : les RA sont mises en file a l'ecriture de
`d_rach` et chaque `L1CTL_RACH_CONF` (lu par le meme tap sercomm) en depile
une - c'est l'appariement que fait le mobile lui-meme dans `cr_hist`.

**Et une mesure qui commande le reste.** L'ecart entre la reference de la BTS
et celle du mobile n'est pas constant, il grandit :

    ra=0x08  BTS 7/14/45 = fn 9582   mobile fn 8074   1508 trames  ~7,0 s
    ra=0x0e  BTS  8/5/33 = fn 10743  mobile fn 9464   1279 trames  ~5,9 s
    ra=0x0e  BTS  8/0/14 = fn 11336  mobile fn 9464   1872 trames  ~8,6 s

C'est la derive du pas-a-pas : le C54x emule coute ~6,7 ms par trame contre
4,615 ms de temps reel, donc l'horloge du mobile prend du retard sur celle du
reseau en continu. Toute comparaison de numero de trame entre les deux cotes
doit donc passer par la valeur du mobile, jamais par celle du reseau.

## Le LU va jusqu'a l'authentification, et meurt d'un SABM de trop [2026-09-21, nuit, fin]

Avec l'intervalle dedie livre au DSP, la procedure se deroule enfin :

    IMMEDIATE ASSIGNMENT: (ta 0/0m ra 0x0b chan_nr 0x41 ARFCN 514 TS 1 SS 0 TSC 5)
    request 0b matches (fn=4,6,23)            <- la reference de requete colle
    new state connection pending -> dedicated
    New SYSTEM INFORMATION 6 / 5 (SACCH descendante decodee)
    RR_EST_CNF -> location updating initiated
    MT_MM_ID_REQ  -> IDENTITY RESPONSE
    MT_MM_AUTH_REQ -> AUTHENTICATION RESPONSE
    MT_MM_LOC_UPD_REJECT                      <- et la, non

Le rejet n'est pas une affaire d'authentification (la Ki du `test-sim` et celle
de `auc_2g` sont la meme, comp128v1) : c'est une consequence. Le BSC dit
pourquoi, a la seconde pres :

    22:30:54 lchan(0-0-1-SDCCH8-0){ESTABLISHED}: ERROR INDICATION
             cause=SABM frame with information not allowed in this state

Un deuxieme SABM sur un lien deja etabli. La LAPDm du BTS casse le canal, le
MSC se retrouve en `MSC_A_ST_RELEASING` et repond LOCATION UPDATING REJECT au
milieu de la procedure (`/var/log/osmocom/osmo-msc.log`, gsm_04_08.c:112).

Coupable : l'anti-doublon repris de la couche 1 gr-gsm, qui republie un bloc
identique passe 60 trames (`SDCCH_UL_DEDUP_TICKS`). Le firmware laisse son
bloc dans `a_cu` ; nous le republiions, le pont le reemettait. Desormais un
bloc n'est publie que si son CONTENU change, et la memoire de l'anti-doublon
repart a zero a la liberation du canal (`montant_canal_libere()`, appele sur
PONT_DCCH genre 0xFF) -- sinon le SABM de la connexion suivante, octet pour
octet identique, serait pris pour un doublon et ne partirait jamais.
`MONTANT_SDCCH_REPETE=N` retablit une republication au bout de N trames.
Compromis assume : une retransmission LAPDm du mobile porte les memes octets
et sera avalee ; l'inverse casse le lien a coup sur.

A surveiller au prochain run : les « Dropping frame with 110 bit errors » de la
descente dediee. 110 erreurs sur 456 bits, c'est la signature d'UN burst sur
quatre manquant ou faux dans le bloc. Elles sont nombreuses au moment de
l'etablissement (le canal n'est arme qu'au premier DATA_CONF/IND, donc les
premiers blocs partent sans intervalle dedie), puis rares pendant la
transaction (SI5, SI6, ID REQUEST, AUTH REQUEST passent tous), puis
permanentes apres le rejet (le BTS n'emet plus rien sur TS1).

## Le SABM ne partait plus du tout : la liberation lue au mauvais endroit [2026-09-21, nuit, suite]

Apres avoir mis « un seul SABM par connexion », le BSC ne dit plus
`ERROR INDICATION` mais `WAIT_RLL_RTP_ESTABLISH: Timeout` : plus de doublon, et
plus de SABM du tout. La trace QEMU montre le defaut en deux lignes :

    [dcch] canal dedie arme   : chan_nr=0x41 SDCCH/8 SS=0 TN=1
    [dcch] canal dedie libere : chan_nr=0x00 SDCCH/4 SS=0 TN=0

Armé puis libéré dans la seconde. Le tap publiait « libere » sur
`d_dsp_page == 0`, condition reprise du `l1_reset()` de la couche 1 gr-gsm.
Faux ici : le firmware fait justement un `l1s_dsp_abort()` (sync.c:308) au
moment ou il bascule VERS le canal dedie -- c'est le « resetting scheduler »
du journal du mobile, juste apres l'IMMEDIATE ASSIGNMENT. `pont.py` voyait
donc un canal libere et jetait le SABM.

Tant que le SABM etait republie toutes les secondes, le defaut etait masque :
une republication finissait toujours par tomber dans une fenetre ou le canal
etait arme. C'est ce qui explique que la session de 22:30 ait pu aller jusqu'a
l'AUTHENTICATION RESPONSE malgre les doublons.

Deux corrections :

- **La liberation se lit sur le retour aux voies communes**, pas sur
  `d_dsp_page`. Un `L1CTL_DATA_IND`/`DATA_CONF` portant un `chan_nr` non dedie
  (BCCH, CCCH) veut dire que le mobile est revenu sur les voies communes ; en
  mode dedie il n'en lit aucun, donc ca ne peut pas arriver au milieu d'une
  connexion. `calypso_l1_do_page_written()` ne libere plus rien.
- **Le bloc montant attend son canal** (`pont/uplink.py:_poll_sdcch`). La
  couche 1 publie le SABM a l'instant ou elle l'emet, et `Dedicated.read()` ne
  relit `/dev/shm/calypso_dcch_cfg` qu'une fois par `DCCH_TTL` (100 ms) : le
  premier bloc d'une connexion tombait regulierement dans cette fenetre et
  disparaissait. On force desormais une relecture des qu'un bloc arrive, et on
  garde le bloc jusqu'a une seconde si le canal n'est pas encore connu, au
  lieu de le jeter (`SDCCH_ATTENTE`).

## Le canal dedie battait : arme/libere des dizaines de fois par seconde [2026-09-21, nuit, suite]

Deuxieme version de la liberation (« un bloc sur une voie commune veut dire que
le mobile est revenu ») : pire que la premiere. Trace QEMU pendant une
connexion :

    [dcch] canal dedie arme   : chan_nr=0x41 SDCCH/8 SS=0 TN=1
    [dcch] canal dedie libere : chan_nr=0x00 SDCCH/4 SS=0 TN=0
    [dcch] canal dedie arme   : chan_nr=0x41 SDCCH/8 SS=0 TN=1
    [dcch] canal dedie libere : chan_nr=0x00 ...        (x N, en boucle)

Le BSP basculait donc entre TS0 et TS1 a chaque bloc. Cote mobile : tous les
blocs descendants a 110/98/87 erreurs, aucun UA, `MDL-ERROR-IND cause 1`
(T200/N200) et liberation -- alors que l'IMMEDIATE ASSIGNMENT avait ete
acceptee (`request 07 matches (fn=2,13,26)`).

Deux criteres ajoutes, tous deux necessaires pour liberer :

- **Un vrai canal commun.** 44.004 8.3 : 0x80 BCCH, 0x88 RACH, 0x90 PCH/AGCH,
  donc `(chan_nr & 0xE0) == 0x80`. Ce qui declenchait la liberation portait
  `chan_nr = 0x00`, qui n'est pas un canal.
- **Un ecart de trames.** Le bloc commun doit etre au moins
  DCCH_LIBERE_APRES_TRAMES (100) trames apres le dernier bloc du canal dedie,
  d'apres le numero de trame que porte l'en-tete L1CTL. Un bloc CCCH en retard,
  delivre juste apres la bascule, porte un numero proche et ne libere donc
  rien.

Rappel de l'historique de ce seul point, parce qu'il resume la difficulte : la
liberation a d'abord ete lue sur `d_dsp_page == 0` (tire a l'entree en mode
dedie, pas a la sortie), puis sur le premier bloc commun venu (tire en boucle
pendant la connexion). Le bon signal est le retour durable sur les voies
communes.

## a_cu porte un drapeau : il n'y avait rien a deviner [2026-09-21, nuit, fin de l'histoire]

Trois versions successives de l'anti-doublon SDCCH montant, trois echecs :

1. Fenetre heuristique + republication apres 60 trames (repris de la couche 1
   gr-gsm) : le meme SABM repartait sur un lien etabli, le BTS repondait
   « SABM frame with information not allowed in this state » et cassait le
   canal en pleine procedure.
2. Comparaison du contenu sur 23 octets : la friture apres la charge utile
   bouge d'une lecture a l'autre, 32 blocs « neufs » publies pour un seul
   SABM.
3. Verrou « un seul SABM par connexion » : plus aucun SABM des que le verrou
   restait arme, et il ne retombait que sur un RACH publie ou une liberation
   annoncee -- deux evenements qui peuvent ne jamais venir. Un verrou qui
   coince le banc definitivement.

Le firmware annonce pourtant chaque bloc, explicitement
(`layer1/prim_tx_nb.c:80-101`) :

    uint16_t *info_ptr = dsp_api.ndb->a_cu;
    info_ptr[0] = (1 << B_BLUD);                   /* bloc present */
    info_ptr[1] = 0; info_ptr[2] = 0;
    dsp_memcpy_to_api(&info_ptr[3], data, 23, 0);  /* les 23 octets L2 */

C'est exactement la disposition que `prendre_ul()` lit deja pour le TCH, et le
drapeau est a usage unique. Donc : on le teste, on prend les 23 octets du mot
3, on l'efface. Un bloc pose = une publication, sans fenetre, sans
comparaison, sans temporisation, sans verrou.

`MONTANT_SDCCH_FENETRE=1` force l'ancienne voie, et elle prend le relais toute
seule si B_BLUD ne se leve jamais alors que le firmware pose des taches
montantes (cas ou la ROM consommerait le drapeau avant la scrutation) : 400
taches sans drapeau, un message, et bascule.

Lecon : chercher le signal que le firmware pose deja, avant d'inventer une
heuristique pour le reconstituer.

## L'horloge du banc etait celle du mur, pas celle du DSP [2026-09-22]

Le run de 10:01 campait, lisait SI1-4 et faisait sa mise a jour de
localisation... pendant dix secondes. Apres, plus rien : a 10:04 l'appel
partait en `T3126`, a 10:06 le SMS n'obtenait meme plus d'IMMEDIATE
ASSIGNMENT, et le canal dedie sortait des « Dropping frame with 110 bit
errors » en continu.

La mesure qui tranche, deux compteurs lus au meme instant a 10:09 :

    pont.py   STATS fn=99674
    [ts0]     tick=81089 fn=80410        (offset ARM-tick fige a -679)

19 000 trames, **87 secondes** d'ecart, et l'ecart grandissait. Le C54x emule
coute ~5,8 ms par trame contre 4,615 ms de temps reel ; `bsp_ts0_service()`
joue la trame BTS `tick + g_ts0_offset`, l'offset est fixe une fois pour toutes
sur la premiere SB, et rien ne rattrapait le reste. La BTS remplissait l'anneau
1,25 fois plus vite que le DSP ne le vidait : le mobile vivait une minute et
demie dans le passe. Tout le reste en decoule --

- `T3126`, `T3101`, `T200` sont des temporisations en secondes de MUR : une
  reponse qui met 87 s a revenir les a toutes epuisees ;
- le canal dedie encore plus vite : `g_dedie` ne commence a se remplir qu'a
  l'armement du canal, or le DSP lisait des trames d'AVANT cet instant. Releve
  dans `/dev/shm/calypso_bsp_dedie` : `stockes=3202 joues=244 manques=150`.
  38 % des trames du canal partaient sans burst -- un burst sur quatre absent
  d'un bloc, c'est exactement 110 erreurs sur 456.

Le DSP ne peut pas rattraper, il tourne deja a fond. C'est donc la BTS qui
ralentit : `calypso_bsp.c` publie la trame que le BSP reclame
(`/dev/shm/calypso_horloge`, 16 octets : seq, cale, fn_bts, tick) et
`pont/trx.py` y asservit l'horloge qu'il envoie a osmo-bts-trx en IND CLOCK.

**Asservir en FREQUENCE, pas par recalage.** Premiere version : repousser `t0`
des qu'on devance le DSP de plus de 12 trames. Mesure immediate,
`UL bursts=11 tard=232` : `Transmitter.run()` jette tout burst dont la trame
s'ecarte de plus de `window_tol` (1 trame) de l'horloge au moment de l'envoi,
et une horloge qui avance par a-coups en sort a chaque fois. Plus un SABM
n'arrivait a la BTS -> pas de UA -> `MDL-ERROR-IND cause 1`, plus aucune mise a
jour. La version qui tient ne change que la VITESSE (`self.dur`, la duree
effective d'une trame) : cadence du DSP mesuree toutes les 250 ms, correction
proportionnelle de la phase sur ~400 trames, et `t0` rebase a chaque
changement pour que `fn()` reste continue.

Apres (run de 10:20, MSC) :

    10:20:04  VLR: update ... TMSInew-0x46FB2C10
    10:20:06  VLR: update ... TMSI-0x46FB2C10

`TMSInew-` devenu `TMSI-`, donc le TMSI REALLOCATION COMPLETE est revenu et le
VLR l'a confirme : le LU va au bout, sans `LOCATION UPDATING REJECT`, sans
`ERROR INDICATION cause=SABM frame with information not allowed in this state`
(le double SABM de 10:01 etait une retransmission T200 du mobile, pas un
doublon du pont : le UA mettait plus de 700 ms a revenir). Cote pont,
`fn=26609` contre `fn_bts=27045` : verrouille.

`PONT_HORLOGE=0` revient a l'horloge murale, `PONT_HORLOGE_AVANCE` regle
l'avance visee (12 trames), `CALYPSO_BSP_HORLOGE=0` coupe la publication.

Reste ouvert : l'etablissement DESCENDANT. Le MSC tente un MT SMS, reste
10 s en `MM_CONN_PENDING` puis `MMSMS-REL-IND` -- le paging ou la reponse du
mobile ne passe pas. A regarder avec le filtre des pagings vides de
`pont/downlink.py` (`is_empty_paging`, qui ne sert qu'au montage gr-gsm) et le
groupe de paging que le firmware ecoute.

## Le magasin du canal dedie commencait trop tard [2026-09-22, suite]

Le paging n'est PAS en cause : le mobile recoit bien le sien et repond
(`CHANNEL REQUEST: 80 (PAGING Any channel)`, 10:22:33). Ce qui le tue est la
ligne d'apres, `LOS during RACH request` -- la couche 3 avait deja declare la
perte de couverture. Meme cause que tout le reste : le canal dedie.

Les compteurs de `/dev/shm/calypso_bsp_dedie` a la liberation :

    tn=-1 ss=0 stockes=4311 joues=404 manques=138 libere

138 trames du canal sur 542 jouees **sans burst**, une sur quatre. C'est
exactement la signature observee cote mobile :

- `Dropping frame with 110 bit errors` (110 sur 456 = un burst sur quatre) ;
- `MON: f=514 lev=<=-110 snr=0 ... TS=1/0` -- rien du tout sur l'intervalle
  dedie pendant trois secondes ;
- `Received frame for unsupported SAPI 2!` et `MDL-ERROR-IND cause 3`, ce que
  LAPDm sort d'un bloc reconstitue a partir de trois bursts sur quatre.

Au meme instant, cote TS0 : 11 manques en tout. Ce n'est donc pas la BTS qui
est en retard sur le DSP (l'horloge asservie tient), c'est ce magasin-ci qui ne
couvre pas assez loin. `g_dedie` n'est alloue et rempli qu'a l'ARMEMENT du
canal, or le BSP joue la trame BTS `tick + g_ts0_offset`, en retard sur celle
qui arrive : toutes les trames du canal anterieures a l'armement sortaient
vides.

Il n'y avait rien a stocker de plus. `g_autres` garde DEJA les sept intervalles
de chaque trame, sans condition et des le premier burst (il sert a completer la
trame continue) : `g_dedie` en est un doublon partiel. `bsp_dedie_bits()`
retombe donc dessus quand son propre anneau n'a pas la trame, et compte ses
replis. `MONTANT_DEDIE_STRICT=1` retablit l'ancien comportement pour remesurer
l'ecart.

A lire au prochain run, dans `/dev/shm/calypso_bsp_dedie` : `manques` doit
tomber pres de zero et `replis` dire combien de trames le repli a rattrapees.
S'il reste des manques, c'est que la BTS n'a vraiment rien emis sur ces
trames-la, et il faudra le chercher cote osmo-bts-trx.

Lecon, la meme que pour a_cu : la donnee etait deja la, dans un autre magasin
du meme fichier. Avant d'en remplir un nouveau, regarder qui garde deja ce
qu'on cherche.

### Correction : le compteur `manques` ne voulait pas dire ca [2026-09-22]

Le « une trame du canal sur quatre sans burst » ci-dessus s'appuyait sur
`manques=138` de `/dev/shm/calypso_bsp_dedie`. Ce compteur etait faux, dans les
deux sens :

- `a_nous` etait calcule par `bsp_dedie_trame(fn)` SANS verifier qu'un canal
  soit arme. Hors connexion, `g_dedie_ss` vaut 0, donc une trame sur huit etait
  declaree « du canal », `bsp_dedie_bits()` rendait NULL sur sa garde
  `g_dedie_tn <= 0`, et `manques` montait -- pendant tout le campement, ou
  jouer TS0 est justement la bonne chose ;
- `manques` (et le nouveau `replis`) n'etaient pas remis a zero a l'armement,
  contrairement a `stockes` et `joues` : le fichier melangeait toutes les
  sessions depuis le demarrage.

Ce qui donnait des lectures impossibles, `joues=72 manques=162` -- plus de
trames manquees que jouees sur un canal ouvert quelques secondes. C'est aussi
pourquoi le repli sur `g_autres` affichait `replis=0` : il est place APRES la
garde `g_dedie_tn <= 0`, donc jamais atteint dans le cas qui gonflait le
compteur.

Corrige : `a_nous` exige `g_dedie_tn > 0`, et les quatre compteurs repartent de
zero a chaque armement. Le repli sur `g_autres` reste : il couvre le vrai trou,
les trames du canal anterieures a l'armement. La mesure est donc A REFAIRE
avant de conclure quoi que ce soit sur le canal dedie.

Lecon : un compteur qu'on n'a pas verifie n'est pas une mesure. Celui-ci a
servi de preuve a un diagnostic chiffre, et le chiffre etait du bruit.

### start-direct --dsp : appeler l'amorce qui marche, pas la reecrire

Rapport du banc : « ca marche avec run_real.sh et pas avec start-direct ».
Les environnements des trois processus, releves dans `/proc/<pid>/environ`,
sont pourtant identiques a deux variables pres, toutes deux sans effet
(`CALYPSO_BSP_DIRECT_FEED` n'est plus dans le binaire -- `strings` ne donne que
`CALYPSO_BSP_DIRECT_BRINT0` -- et `PONT_NB_DEBUG` n'est qu'une trace). La
difference etait donc dans l'AMORCE -- et plus precisement dans ce qui n'etait
PAS monte : le coeur reseau, la BTS, le side-car et tmux sont des run_modules
du fork, et `--dsp` avait remplace le run.sh du fork par celui de c54x_exe, qui
ne les connait pas. Deuxieme essai, delegation a `run_real.sh --secondes 0` :
meme trou, plus un « ECHEC : aucun SI lu » imprime apres zero seconde
d'observation.

Version qui tient : `--dsp` ne change ni de fork ni de profil (l'hybride reste
le defaut). Le plan du fork se joue en entier, avec `--skip qemu,pty,osmocon,l2`
-- les quatre modules de la chaine Calypso -- et `--no-attach` ; le banc DSP
prend le relais juste apres avec ses cinq etapes. `MOD_REQUIRED[bts]=0` et sa
barriere porte sur la VTY, pas sur le transceiver : osmo-bts-trx peut donc
demarrer avant pont.py, comme dans run_real.sh.

Deux details d'usage corriges au passage : `--stop` arrete le banc DSP meme
sans `--dsp` sur la ligne de commande (sinon cinq processus survivaient en
tenant 5700-5702, et la pile paraissait arretee), et les aides phonesim/RIL
sont `disown`ees -- `setsid` detache la session mais pas le job, et bash
annoncait leur mort par un « line NNNN: <pid> Killed setsid ... » par-dessus le
prompt.

### Deux scripts, deux sens pour `MODE` [2026-09-22]

    [ .. ] Arret de la pile via run.sh[run] ECHEC : MODE=faketrx-qemu inconnu (dsp|grgsm)

`start-direct.sh` exporte son propre `MODE` (le profil : `faketrx-qemu`,
ligne `export CALYPSO_PROFILE MODE`), et `c54x_exe/run.sh` lit la MEME variable
pour choisir `dsp|grgsm`. Le banc heritait donc du profil et sortait avant
d'avoir rien arrete -- silencieusement, l'appelant ignorant son code de retour.
Verifie des deux cotes :

    $ MODE=faketrx-qemu bash run.sh --status
    [run] ECHEC : MODE=faketrx-qemu inconnu (dsp|grgsm)
    $ MODE=faketrx-qemu bash -c 'MODE=dsp bash run.sh --status'
      dsp      pid 147597   /tmp/c54x-pont/dsp.log
      ...

Ce que ca laissait derriere, releve juste apres un `--stop` : `dsp` encore
vivant, `qemu/osmocon/mobile/pont` arretes -- le teardown du fork connait ces
quatre-la (ils sont dans ses patterns) mais pas `c54x_exe`, qui restait seul a
tenir `/tmp/calypso_dsp.sock` et `/dev/shm/calypso_api_ram`.

On ne renomme pas le `MODE` du banc (run_real.sh et les habitudes s'en
servent) : `start-direct.sh` passe desormais par un `banc_dsp()` qui pose
`MODE=dsp` a chaque appel.

### Ou en est le canal dedie [2026-09-22, 11:08]

Etabli, compteurs corriges a l'appui :

- **les bursts sont tous la.** `/dev/shm/calypso_bsp_dedie` :
  `stockes=4288 joues=416 manques=0 replis=5`. Le repli sur `g_autres` ne
  rattrape que 5 trames (le retard d'armement) : la theorie du « un burst sur
  quatre manquant » est morte, c'etait le compteur qui mentait ;
- **TS0 n'est pas affame** : 11 a 12 `[ts0] pas de burst BTS` par run, tous au
  demarrage. L'horloge asservie tient ;
- **la signature a change** : plus de `unsupported SAPI 2`, plus de
  `MDL-ERROR cause 3`. Reste 89 blocs a **exactement 96 erreurs sur 456**, la
  meme valeur a chaque fois. Constant = corruption systematique, pas un trou.
  Pour comparaison, un bloc BCCH du meme run est a ber 46-50 et passe.

Cote reseau, la consequence se lit maintenant en une ligne : le VLR alloue le
TMSI (`TMSInew-0x1BCA53DE`), donc le LOCATION UPDATING ACCEPT est parti, et
cinq secondes plus tard `MSC_A_ST_RELEASING: LOCATION UPDATING REJECT` -- le
TMSI REALLOCATION COMPLETE n'est jamais revenu. Le side-car MS#2, lui, boucle
son LU (`TMSInew-` puis `TMSI-`) : le coeur reseau est hors de cause.

Question ouverte : pourquoi 96, toujours 96 ? La geometrie de fenetre est la
seule difference plausible entre un bloc SDCCH et un bloc BCCH dans ce chemin
de livraison, et elle n'etait pas mesurable -- la trace `[ts0]` est plafonnee a
20 lignes puis une sur 5000, donc on n'avait que le demarrage. Elle trace
desormais TOUTE trame du canal dedie (300 au plus, sans
`CALYPSO_BSP_TS0_DEBUG`) et dit si son burst vient bien de cet intervalle :

    [ts0] tick=... fn=... p51=0 NB fenetre=151 marge=3 rif_avant=0  <- canal dedie

A comparer avec un bloc BCCH (p51 = 2-5) du meme run. Avec `PONT_NB_DEBUG=1`
en plus, `[nb]` donne le TOA et l'en-tete `a_cd` (mot de Fire, erreurs) par
burst : si le TOA des bursts dedies differe de celui des bursts BCCH, c'est le
calage de la fenetre, et les 96 erreurs s'expliquent.

### « Trop tot » etait compte comme « trop tard » [2026-09-22, 11:12]

    UL bursts=116 tard=18

13 % des bursts montants JETES. Un bloc en demande quatre : ~40 % des blocs
montants n'arrivaient pas entiers a la BTS. C'est exactement ce qu'on lisait en
bout de chaine -- TMSI REALLOCATION COMPLETE absent (LU REJECT a 11:06), CP-ACK
absent (MT SMS a 11:10, `WAIT_CP_ACK` puis abandon), SABM retransmis par T200.

`Transmitter.schedule()` calcule l'instant d'emission `post` a la MISE EN FILE,
d'apres la cadence de l'horloge a ce moment-la. Depuis qu'elle suit le DSP,
cette horloge n'avance plus au rythme du mur : quand le DSP marque le pas,
elle aussi, et le reveil tombe AVANT que la trame visee ne soit arrivee.
`run()` comparait alors `abs(off) > window_tol` et jetait -- sans distinguer le
burst en avance (qu'il suffit d'attendre) du burst en retard (perdu).

Corrige : en avance, on remet en file avec un `post` recalcule sur l'horloge
courante, jusqu'a `PONT_WINDOW_ESSAIS` fois (12) ; seul `off < -window_tol`
compte comme un retard. C'est une consequence directe de l'asservissement
([[horloge-banc-suit-dsp]]) : une file d'emission datee en temps mur ne peut pas
servir une horloge qui ne l'est plus.

### Deux pistes ecartees, une mesuree

- **geometrie de fenetre** : ecartee. La trace dit la meme chose des deux cotes,
  `fenetre=151 marge=3` pour une trame du canal dedie comme pour un bloc BCCH.
- **bursts manquants** : ecartee, `manques=0`.
- **chiffrement** : `ENCRYPTION="a5 0"` arrive enfin jusqu'a
  `/etc/osmocom/osmo-bsc.cfg` (`encryption a5 0`) depuis que l'environnement
  gagne sur globals.conf, et c'est a partir de la que le LU passe en entier
  (`TMSInew-0xA7CFD9FF` puis `TMSI-0xA7CFD9FF`, 11:09:45).

Reste, par ordre :

1. `Received frame for unsupported SAPI 5!` + `MDL-ERROR-IND cause 3` en rafale
   pendant la connexion dediee, avec `MON: lev=<=-110 snr=0 ... TS=1/0`. Des
   blocs qui passent le code de Fire mais dont l'adresse LAPDm est fausse : ce
   ne sont pas des blocs abimes, ce sont d'AUTRES blocs. Piste : un bloc SACCH
   rendu sur la liaison SDCCH. L'octet 0 d'un en-tete L1 SACCH est le niveau de
   puissance ordonne, et `(0x08 >> 2) & 7 = 2` comme `(0x15 >> 2) & 7 = 5` :
   les deux SAPI vus, 2 et 5, sont exactement ce que donne un en-tete L1 lu
   comme une adresse. A verifier sur le tap GSMTAP (udp/4729) en comparant ce
   que la BTS emet en TS1/p51=0-3 et en TS1/p51=32-35.
2. `FBSB RESP: result=255` en rafale sur la cellule SERVANTE (281 demandes sur
   l'ARFCN 514 contre 5 sur 614, 228 echecs) -> `MM_EVENT_LOST_COVERAGE` ->
   « no cell available ». Toute transaction lancee dans cette fenetre meurt
   sur-le-champ : c'est ce qui tue l'appel de 11:12:17 (`MMCC_EST_REQ` recu en
   « no cell available », `MMCC_REL_IND` dans la seconde) et ce qui produisait
   les `LOS during RACH request`.

### Le correctif montant, mesure sur le banc [2026-09-22, 11:16]

Avant / apres, meme banc, meme configuration :

    UL bursts=116 tard=18    (13,4 % jetes)
    UL bursts=185 tard=4     ( 2,1 % jetes)

Six fois moins, et on retrouve le niveau d'avant l'asservissement de l'horloge
(310/4, soit 1,3 %). Le reste tient a la gigue residuelle du DSP : un burst
vraiment en retard reste perdu, c'est le comportement voulu.

Ce n'etait pas toute l'histoire pour autant : le LU de 11:15:46 echoue encore
(`TMSInew-0x2BC8C863` puis REJECT), et le canal dedie sort toujours ses blocs a
96 erreurs (87 dans ce run). Le montant n'etait qu'un des deux chemins.

### FBSB mid-campement : le DSP ne detecte rien [2026-09-22, 11:16]

Sonde en lecture seule sur `/dev/shm/calypso_api_ram` (cf. [[sonde-api-ram-vivante]]),
3850 echantillons a 3 ms pendant que le mobile campait et que la couche 3
enchainait ses `FBSB RESP: result=255` :

    fb_det   fb_mode    TOA     PM       angle    SNR     vu
    0        0          0       5424     -1       2539    3850

`d_fb_det` reste a 0 sur TOUTE la fenetre : le DSP ne pose aucun resultat de
detection FB. Ce n'est donc pas la porte FB0->FB1 de prim_fbsb.c qui rejette,
c'est la tache FB qui ne rend rien du tout -- alors que le meme DSP demodule
sans peine les blocs BCCH de la meme cellule (SI1-4 en continu, ber 46-50).
Sur ce run : 19 `L1CTL_FBSB_REQ` sur l'ARFCN 514, 16 `result=255`.

La consequence se lit trois lignes plus loin dans le journal du mobile :
`MM_EVENT_LOST_COVERAGE` -> « no cell available », et toute transaction lancee
dans cette fenetre meurt sur-le-champ (`MMCC_EST_REQ` recu en « no cell
available », `MMCC_REL_IND` dans la seconde, appel de 11:12:17).

Piste a suivre : au demarrage la meme tache FB reussit. La difference est que
`g_ts0_offset` vaut alors INT64_MIN et que `bsp_ts0_service()` joue les bursts
DANS L'ORDRE D'ARRIVEE, un par tick -- un flux continu. Une fois cale, il joue
`tick + offset` et saute les ticks dont la trame manque. A verifier : ce que
voit la tache FB quand elle est relancee en cours de campement, et si un
retour au mode « ordre d'arrivee » pendant une recherche FB la debloque.

### Ce n'est pas un bloc abime, c'est le meme bloc [2026-09-22, 11:18]

Le LU passe desormais en entier -- `LOCATION UPDATING ACCEPT (lai=001-01-1)`,
`got TMSI 0x0e67ce01`, `TMSI REALLOCATION COMPLETE` emis. C'est APRES, en
attente de la liberation, que le canal part en vrille, et la signature est
enfin lisible :

    N(S) sequence error: N(S)=1, V(R)=2     (x25, TOUJOURS la meme paire)
    Received frame for unsupported SAPI 6!  (2 le 11:11, 5 le 11:15, 6 ici)

Deux faits qui tranchent :

- `N(S)=1` repete alors que le mobile attend `N(S)=2` : LAPDm recoit encore et
  encore LE MEME I-frame. Un bloc abime ne repasse pas le code de Fire vingt
  fois de suite avec le meme N(S) ;
- le SAPI illegal CHANGE d'un run a l'autre (2, 5, 6) mais reste CONSTANT dans
  un run. C'est le contenu fige d'un tampon, pas du bruit.

Ce qui disqualifie l'hypothese « bloc SACCH rendu sur la liaison SDCCH »
avancee plus haut, et renvoie a deux choses deja ecrites ici :
[[tpu-dsp-frame-irq-oneshot]] (la ROM re-dispatche une page perimee quand elle
recoit une IRQ trame qu'elle n'attendait pas) et [[sb-delirant-page-zero]]
(l'ARM relit une page R qui ne porte pas de nouveau resultat). Les deux ont ete
diagnostiquees en mode FB/SB ; ici c'est le mode DEDIE.

Cote BSP le magasin est hors de cause : `g_ts0`, `g_dedie` et `g_autres`
n'apparient que sur `fn` EXACT, aucun ne peut rejouer une trame. Et
`manques=0`.

Prochaine mesure : `PONT_NB_DEBUG=1` imprime l'en-tete `a_cd` (mot de Fire,
erreurs) et les premiers octets decodes a chaque burst 3. Si le meme `a_cd`
ressort trame apres trame pendant la connexion dediee, la boucle est cote
ROM/page et non cote radio -- et les 96 erreurs ne sont qu'un effet de bord du
bloc fige.

### Le chiffrement : personne ne dechiffrait le descendant [2026-09-22, 11:21]

Run avec `ENCRYPTION="a5 1"`. La transaction va plus loin que jamais -- SABM/UA,
IDENTITY REQUEST/RESPONSE, puis AUTHENTICATION REQUEST/RESPONSE, tout en clair
-- et la tempete commence a la ligne EXACTE ou le chiffrement s'arme :

    11:21:02  CIPHERING MODE COMMAND (sc=1, algo=A5/1 cr=1)
    11:21:02  CIPHERING MODE COMPLETE (cr 1)
    11:21:02  Dropping frame with 96 bit errors     <- et sans interruption ensuite

En « a5 0 » la meme transaction va au bout (LU complet du 11:18). La
correlation est nette dans les deux sens.

Deux faits qui l'expliquent :

- **il n'y a aucun A5 dans le modele Calypso.** `d_a5mode` n'apparait que dans
  `hw/arm/calypso/l1-grgsm/calypso_l1_grgsm.c` ; rien dans `l1-dsp/`. En
  montage DSP, le mobile n'a donc rien pour dechiffrer ;
- **le pont est asymetrique.** `Trx.send_ul()` CHIFFRE le montant
  (`self.cipher.apply(burst, fn, True)`) : il tient la place de la voie
  d'emission du DSP. Mais `run_data()` relayait le descendant BRUT vers le DSP.
  Le `cipher.apply(..., False)` de `Downlink._decode()` ne sert qu'au decodage
  L2 du pont lui-meme et ne touche pas ce que recoit le DSP.

Corrige : `run_data()` dechiffre le burst avant de l'envoyer au DSP, A5 etant
symetrique. **Seulement sur l'intervalle dedie** (`_tn_dedie()`, lu via
`Dedicated` deja mis en cache) : la BCCH et la CCCH ne sont jamais chiffrees,
les toucher detruirait le campement. La condition est `cipher.dl_active`, que
`Downlink._decode()` leve deja quand un bloc ne decode qu'une fois dechiffre --
le meme signal que la BTS (osmo-bts l1sap.c check_for_first_ciphrd).

Effet de bord utile : `stats.a5_dl` cesse d'etre a zero, ce qui rend le
dechiffrement visible dans la ligne STATS.

A noter pour la suite : dans ce meme run le MSC a lache a 11:21:00, soit DEUX
SECONDES AVANT que le mobile ne recoive la commande de chiffrement. La fenetre
morte de 11:20:58-11:21:01 (`MON lev=<=-110 snr=0`, `MDL-ERROR cause 12`) reste
donc un defaut a part entiere, independant du chiffrement.

### `./start-direct.sh` sans --dsp : QEMU ne demarrait pas [2026-09-22, 11:23]

    [FAIL] Calypso emulator (QEMU) (started but never ready:
           socket du moniteur QEMU : toujours pas pret apres 30s)

Le message designe le moniteur, mais rien n'avait ete lance. Les deux
enveloppes `/usr/local/bin/qosmo-dsp` et `/usr/local/bin/qosmo-grgsm` (hors
depot) trient ainsi :

    case "${1:-}" in -k|-kernel|-r|--rundir|-M|-s|-p|--bin|--gdb|--bind|-o)
        exec /usr/local/bin/qosmo-grgsm-launch "$@";; esac
    exec env MODE=grgsm PONT=1 /opt/GSM/c54x_exe/run.sh "$@"

`--qemu`, `--cpu` et `--monitor` manquent a la liste, et le test ne porte que
sur le PREMIER argument. Or `run_modules/40-qemu.sh` appelle

    qosmo-grgsm --qemu <bin> -k <elf> --bin <bin> --cpu arm946 \
                --gdb N --rundir <dir> --monitor <dir>/qemu-monitor.sock

soit `--qemu` en tete. Le motif ne matche pas, l'appel part sur
`c54x_exe/run.sh`, qui sort immediatement :

    $ qosmo-grgsm --qemu ... -k ... --monitor ...
    [run] ECHEC : option inconnue : --qemu (voir --help)

Le vrai lanceur C, lui, comprend les six (`strings` : `--bin --cpu --gdb
--monitor --qemu --rundir`). Corrige dans les deux enveloppes : balayage de
TOUS les arguments, et les trois options ajoutees. Sauvegardes dans
/tmp/qosmo-{grgsm,dsp}.bak. Elles ne sont dans aucun depot -- une
reinstallation les ecrasera ; c'est note dans LAUNCH.md.

Defaut PRE-EXISTANT, sans rapport avec le montage DSP : `--dsp` ne passe pas
par ce module (il est dans `--skip qemu,pty,osmocon,l2`), ce qui explique qu'on
ne l'ait vu qu'en lancant start-direct SANS `--dsp`.

### Le Kc n'etait publie par personne [2026-09-22, 11:29]

Le dechiffrement ajoute dans `pont/trx.py` n'a rien change au run de 11:26, et
la raison est en amont :

    $ ls /dev/shm/calypso_kc_l1
    ls: cannot access '/dev/shm/calypso_kc_l1': No such file or directory
    $ grep -rn calypso_kc_l1 qosmo/hw/arm/calypso/
    l1-grgsm/calypso_l1_grgsm.c:36:#define SHM_KC "/dev/shm/calypso_kc_l1"

Un seul ecrivain, et c'est la couche 1 gr-gsm -- celle que
`calypso_l1_disable("DSP externe")` desactive justement en montage DSP. Sans ce
fichier, `Cipher.current()` rend None et `cipher.apply()` rend le burst
INCHANGE dans les deux sens : ni dechiffrement de la descente, ni chiffrement
de la montee. Le compteur le disait depuis le debut, on ne l'avait pas lu :
`A5 dl=0 ul=0` a chaque ligne STATS.

C'est exactement la cause qui a fait naitre ce fichier (montant.c) pour le
RACH et le SDCCH : sous DSP_EXTERN, tout ce que publiait la couche 1 gr-gsm
disparait, et il faut le republier par scrutation. Le Kc avait ete oublie.

`montant.c` publie donc maintenant `/dev/shm/calypso_kc_l1` depuis
`d_a5mode` et `a_kc[4]` du NDB, disposition reprise TELLE QUELLE de
`publish_kc()` (seq(4) algo(1) longueur(1) Kc[8] 0xFF, mots de a_kc en
gros-boutiste et a l'envers) pour que `pont/cipher.py` la lise sans changement.
Meme grace de 5 scrutations avant d'annoncer un retour en clair, pour la meme
raison : le firmware efface `d_a5mode` a chaque DM_REL_REQ, y compris quand le
Kc revient juste apres, alors que la BTS ne cesse jamais de chiffrer.
`MONTANT_KC=0` coupe la publication.

A verifier au prochain run en A5/1 : `[montant] chiffrement A5/1 : Kc publie`
dans dsp.log, puis `A5 dl=... ul=...` non nuls dans la ligne STATS du pont, et
la transaction qui passe le CIPHERING MODE COMPLETE sans tomber a 96 erreurs.

### CORRECTION : le DSP detecte bien la FB [2026-09-22, 11:30]

L'entree « FBSB mid-campement : le DSP ne detecte rien » ci-dessus est FAUSSE.
Elle s'appuyait sur un echantillonnage de `/dev/shm/calypso_api_ram` a 3 ms
pendant 12 s qui ne voyait jamais `d_fb_det=1`. Les jalons du meme run disent
le contraire :

    total jalons d_fb_det=1 : 196
    [jalon] fn=17980 SB PLAUSIBLE page=1 BSIC=7 a_sch=8000 0707 061c 0191
    SB decodees (osmocon) : 25   pour   147 L1CTL_FBSB_REQ

`d_fb_det` est TRANSITOIRE -- leve puis efface dans la trame. L'echantillonner
a 3 ms sur des trames de 5,8 ms le rate la plupart du temps, et ne pas le voir
ne prouve rien. La sonde de [[sonde-api-ram-vivante]] vaut pour les valeurs qui
DURENT (TOA, PM, SNR, d_fb_mode) ; pour un drapeau fugace, il faut les jalons
de dsp.log, qui sont poses par le code au moment ou il le lit.

Ce que disent les vrais chiffres : 25 succes pour 147 demandes, soit 17 % --
le meme taux que les 58/281 releves a 10:09. Le DSP detecte la FB et decode la
SB ; c'est la PROCEDURE FBSB qui expire avant que la SB n'arrive. Le banc en
pas-a-pas est plus lent que le budget de tentatives du firmware. C'est la qu'il
faut chercher, pas dans la detection.

### Le Kc est publie ; le blocage est maintenant AVANT le chiffrement [2026-09-22, 11:33]

Le side-band manquant est comble : `/dev/shm/calypso_kc_l1` existe (32 octets,
ecrit une seconde apres le demarrage de c54x_exe) et `[montant] retour en clair
(seq=1)` est trace. Mais `d_a5mode` n'est jamais devenu non nul dans ce run :
le CIPHERING MODE COMMAND n'est jamais arrive. **La voie A5 n'est donc toujours
pas exercee** -- ni le publieur de Kc de montant.c, ni le dechiffrement de
pont/trx.py. Ne pas les compter comme valides.

Ce qui bloque avant, et c'est net dans le journal du mobile :

    11:32:50  AUTHENTICATION RESPONSE            <- la transaction va jusque-la
    11:32:50  Unnumbered frame not allowed       <- puis 4 s de
    11:32:50  MDL-ERROR-IND cause 12                « fenetre morte »
    11:32:52  MON: lev=<=-110 snr=0 ... TS=1/0
    11:32:54  Dropping frame with 96 bit errors  <- et la tempete s'installe
    11:33:09  T3210 expire

Le MSC, lui, avait lache a 11:32:53.

Et les compteurs du meme run disent que ce n'est PAS un probleme de livraison :

    tn=-1 ss=0 stockes=4392 joues=428 manques=0 replis=6
    trames du canal tracees : 300     dont « BURST MANQUANT » : 0
    UL bursts=218 tard=3              (1,4 %)

Chaque trame du canal a recu son burst, aucune n'a ete jouee vide, et le
montant ne perd plus rien. Pendant que le mobile mesure `lev=<=-110 snr=0` sur
son intervalle dedie, le BSP lui a bel et bien remis un burst pour chacune de
ses trames.

**La perte est donc APRES la remise, dans la demodulation du burst dedie par le
DSP** -- ni dans le pont, ni dans le magasin, ni dans l'horloge. Les trois
premiers sont maintenant mesures et hors de cause. C'est la que doit porter la
suite : `PONT_NB_DEBUG=1` donne le TOA, le PM, le SNR et l'en-tete `a_cd` par
burst ; comparer ceux d'un burst du canal dedie a ceux d'un burst BCCH de la
meme seconde dira si le probleme est un calage (TOA) ou une amplitude (PM).

Et l'`Unnumbered frame not allowed` arrive sur un AUTRE datalink que les
I-frames (`dl=0x...b9e8` contre `0x...bda8` ailleurs) : c'est la liaison
SAPI 3, restee IDLE. Une trame U qui atterrit sur SAPI 3 pendant une
transaction SAPI 0, c'est encore une adresse LAPDm lue de travers -- meme
famille que les SAPI 2/5/6 deja vus.

### A5/1 : la chaine fonctionne [2026-09-22, 11:49]

Premier run qui franchit le chiffrement. Les deux correctifs de 11:22 et 11:29
sont valides par la mesure :

    [montant] chiffrement A5/1 : Kc publie vers /dev/shm/calypso_kc_l1 (seq=2)
    [pont]    chiffrement descendant confirme par la BTS (fn=1836)
    [pont]    A5 dl=904 ul=68

    11:49:02  LOCATION UPDATING ACCEPT (lai=001-01-1)
    11:49:02  got TMSI 0x9e61e668
    11:49:02  TMSI REALLOCATION COMPLETE

Le mobile lit un LOCATION UPDATING ACCEPT **chiffre** : impossible une heure
plus tot, ou la tempete de 96 erreurs commencait a la ligne du CIPHERING MODE
COMPLETE. 904 bursts descendants dechiffres par le pont, et le drapeau
`dl_active` leve par la BTS comme prevu.

Le blocage a donc encore avance d'un cran -- il est maintenant sur le DERNIER
bloc MONTANT. Cote MSC :

    11:49:01  TMSInew-0x9E61E668
    11:49:03  LOCATION UPDATING REJECT     (reste « TMSInew », jamais « TMSI- »)

Le mobile a emis son TMSI REALLOCATION COMPLETE, le MSC ne l'a pas recu.

Hypothese a verifier, encore une asymetrie de chiffrement : le pont chiffre le
montant des que `cipher.current()` rend une cle, donc des que `montant.c` a
publie le Kc. Or `publier_kc()` scrute toutes les 22 trames (~100 ms) et peut
publier AVANT que le mobile n'ait bascule lui-meme -- il ne chiffre qu'apres
avoir emis son CIPHERING MODE COMPLETE. Dans cette fenetre le pont chiffrerait
un bloc que le mobile a emis en clair, et la BTS ne le lirait pas. Ca
corromprait exactement les blocs autour de la bascule.

Mesure a faire : comparer l'horodatage du `seq=2` (publication du Kc) a celui
du CIPHERING MODE COMPLETE. Si le premier precede le second, la fenetre existe.
Le remede serait le symetrique de ce que fait deja la descente : ne chiffrer le
montant qu'apres une preuve, pas des que la cle est connue.

### Le Kc perime chiffrait la connexion suivante [2026-09-22, 11:57]

Bug introduit par le publieur de Kc, trouve et corrige dans la foulee. La grace
`KC_GRACE_CLAIR` (5 scrutations avant d'annoncer un retour en clair), recopiee
de publish_kc(), sert au cas INTRA-connexion : le firmware efface d_a5mode a
chaque DM_REL_REQ, y compris pendant un Assignment Command ou le Kc revient
juste apres. Mais entre DEUX connexions elle est nuisible : l'enregistrement
algo=1 restait lisible cinq scrutations de plus, et pont.py -- qui relache
pourtant sa cle a chaque IMMEDIATE ASSIGNMENT (downlink.py, `cipher.release`)
-- la relisait aussitot dans le fichier et la restaurait.

Il chiffrait alors le montant de la connexion SUIVANTE des son premier bloc,
pendant que le mobile emettait encore en clair. Mesure du run de 11:53 :

    A5 dl=0 ul=200          <- tout le montant chiffre, rien de dechiffre
    fn=2525  01 52 19 05 14 ...   AUTHENTICATION RESPONSE
    fn=2729  01 52 19 05 14 ...   LE MEME, faute d'acquittement

Corrige : `montant_canal_libere()` (deja appele sur PONT_DCCH genre 0xFF) leve
un drapeau qui fait publier le retour en clair IMMEDIATEMENT, sans la grace.
Au passage, `d_a5mode` est desormais lu a CHAQUE trame (deux acces memoire) et
non plus une sur 22 : un changement de mode ne peut plus attendre 128 ms, ce
qui laissait partir en clair le premier bloc chiffre du montant.

Effet mesure au run de 11:58, retour au bon profil :

    A5 dl=905 ul=68     chiffrement descendant confirme par la BTS (fn=1836)
    11:58:13  LOCATION UPDATING ACCEPT + got TMSI 0x9846D415

### Ce qui reste : l'acquittement LAPDm descendant

    fn=2430  01 64 35 06 32 ...   CIPHERING MODE COMPLETE
    fn=2583  01 74 35 06 32 ...   LE MEME, retransmis (bit P)
    (aucun TMSI REALLOCATION COMPLETE n'est jamais publie)

Le reseau a pourtant RECU le CIPHERING MODE COMPLETE -- sans lui le MSC
n'aurait pas envoye le LOCATION UPDATING ACCEPT. Ce qui manque est donc
l'acquittement LAPDm DESCENDANT : le mobile ne le voit pas, reste en
retransmission, et sa fenetre de 1 l'empeche d'emettre le bloc suivant.

Le montant est hors de cause (`tard=0`), le chiffrement aussi (correct dans les
deux sens, mesure). Il reste la perte residuelle de blocs DESCENDANTS sur le
canal dedie -- la famille des « 96 erreurs », deja isolee comme etant apres la
remise des bursts par le pont (`manques=0`, `BURST MANQUANT=0`) et donc dans la
demodulation du burst dedie par le DSP.

CORRECTION : l'hypothese « le pont chiffre le montant trop TOT parce que le Kc
est publie en avance » ecrite plus haut est fausse dans ce sens-la. `publier_kc`
scrutait en RETARD (22 trames), pas en avance. Le vrai defaut etait le Kc
PERIME d'une connexion precedente, ci-dessus.

### Le burst dedie arrive au DSP a moitie puissance [2026-09-22, 12:12]

Mesure par la sonde `[nb]` (plafond porte de 400 a 20000 : les 400 etaient
consommees par le campement avant toute connexion), bornee a la fenetre ou le
canal etait REELLEMENT arme (ticks 2047..3529 du run de 12:04) :

    bursts DEDIES        n=176    TOA moy 1,9   PM 2310   SNR 232
    bursts BCCH / CCCH   n=2800   TOA moy 4,2   PM ~4500  SNR ~500

Le burst du canal dedie arrive **deux qbits trop tot** dans la fenetre et a
**la moitie de la puissance**, avec un SNR deux fois moindre. Ce n'est pas du
bruit : 176 bursts, ecart systematique.

⚠️ Piege de lecture : le `p51` imprime par `[nb]` est celui du TICK de l'ARM,
pas celui de la trame BTS. Les deux different de `g_ts0_offset` (-670 sur ce
run). Un premier regroupement fait sans cette conversion donnait des moyennes
mixtes et ne montrait rien.

Trois causes possibles, toutes ECARTEES par la mesure :

- **la geometrie de fenetre** : identique, `fenetre=151 marge=3` sur les 75
  trames dediees jouees, comme pour un bloc BCCH ;
- **le dechiffrement A5 que j'ai ajoute** : `gsm.a5_xor()` n'XORe que les bits
  3-59 et 88-144, les deux moities de donnees. La sequence d'apprentissage
  reste intacte, le correlateur du DSP n'est pas touche ;
- **la qualite des bursts eux-memes** : le pont decode les MEMES bursts avec
  zero echec -- `TS1/0:28/0 TS1/32:10/0`, `crc=0`. Les bits sont bons.

Donc : le pont livre des bursts CORRECTS, TOUS (`manques=0`, `BURST
MANQUANT=0`), dans la BONNE fenetre -- et le DSP les demodule a moitie
puissance et deux qbits trop tot. Le defaut est dans l'injection/demodulation
cote DSP de l'intervalle dedie substitue, pas dans le pont.

C'est la cause directe des « 96 bit errors », de la perte de ~3 blocs
descendants sur 4 (chaque aller-retour LAPDm coute 4 multitrames au lieu
d'une, cf. la cadence du montant plus haut) et donc du LOCATION UPDATING
REJECT : le MSC lache avant que le TMSI REALLOCATION COMPLETE ne puisse partir.

Ou chercher : `bsp_ts0_livrer()` module le burst dedie exactement comme un
burst TS0 (`gmsk_moduler(bits,148,30000,0,0.5,iq+2*marge)` puis
`gmsk_elargir(..., CALYPSO_BSP_NB_SYM=0.3)`), et pourtant le resultat differe.
La difference doit etre en aval : ce que le RIF/DMA fait de ces echantillons
quand la fenetre est armee sur TS1 et non sur TS0. `CALYPSO_BSP_TS0_DEBUG=1`
donne `rif_avant=` par trame ; le comparer entre trames dediees et trames TS0
est la prochaine mesure.

### Deux pistes de plus ecartees, et une a creuser [2026-09-22, 12:20]

- **`rif_avant`** : identique. 432 trames dediees a `f=151 m=3 rif=0`, et 1297
  trames NON dediees exactement pareil. Le niveau du RIF ne distingue pas les
  deux. Piste morte.

- **`CALYPSO_BSP_VERIF=1`** (compare la DARAM au burst remis au BSP, apres que
  le DSP a tourne) : 2849 « partiel » contre 8 « VALIDE ». Reparti par type de
  trame, avec la conversion tick -> trame BTS (offset -690) :

        TS0    n=3360  echantillons identiques : 14 % en moyenne, 2775 a 0 %
        DEDIE  n=640   echantillons identiques :  1 % en moyenne,  626 a 0 %

  L'ecart va dans le bon sens mais la sonde est dominee par un effet attendu :
  elle compare APRES `jouer_trame`, donc apres que le DSP a consomme et
  reecrit le tampon. Un « 0 % » ne prouve pas que la livraison a rate. A
  reprendre en comparant AVANT que le DSP ne tourne, ou en marquant le tampon.

Etat du diagnostic, tout ce qui est mesure :

    le pont livre        des bursts corrects   (TS1/0:28/0, crc=0)
                         tous                  (manques=0, BURST MANQUANT=0)
                         dans la bonne fenetre (f=151 m=3, identique a TS0)
                         au bon niveau RIF     (rif_avant=0, identique a TS0)
    le DSP en tire       TOA 1,9  PM 2310  SNR 232
    alors que sur TS0    TOA 4,2  PM ~4500 SNR ~500

Tout ce qui precede la demodulation est desormais mesure et identique entre
les deux cas. Le defaut est dans ce que le DSP fait de ces echantillons quand
la fenetre est armee sur l'intervalle dedie.

### CALYPSO_BSP_PAGE_FOLLOW=1 : essai negatif [2026-09-22, 12:23]

L'experience laissee ouverte le 2026-09-19 a maintenant une reponse. Sa
premisse est confirmee par une mesure independante d'aujourd'hui :

    w_page=0 -> 0x0cce 1811x, 0x0e4e 41x
    w_page=1 -> 0x0cce 2115x, 0x0e4e 33x

98 % des bursts atterrissent a la MEME adresse quelle que soit la page que la
tache DSP va lire, alors que d_dsp_page alterne bien (0x0002/0x0003). Le taux
de correspondance DARAM suit : 12 % a 0x0cce contre 39-43 % a 0x0e4e.

Mais deposer a `base + w_page*stride` (PAGE_FOLLOW=1, pas 0x180) n'ameliore
pas : la transaction meurt juste apres l'IDENTITY RESPONSE, deux LOCATION
UPDATING REJECT sans meme un `TMSInew` cote VLR -- alors que la reference
atteignait le CIPHERING MODE COMPLETE et obtenait un TMSI. Un seul run, et la
variance de ce banc est grande, mais le sens est clair : ce n'est pas la bonne
correction. Defaut remis a 0.

Ce que ca apprend quand meme : l'adresse n'est pas le probleme, ou pas seule.
La ROM programme son AAD, le BSP la suit (AAD_FOLLOW=1), et forcer une autre
adresse casse. Le ping-pong manquant se joue ailleurs -- peut-etre que la ROM
ne reprogramme l'AAD qu'une fois sur N parce que son ISR de fin de DMA ne
s'execute pas a chaque trame (cf. DSP Error Status 24 = DMA_PROG|DMA_TASK,
permanent depuis le boot, files de requetes qui debordent).

C'est le fil a tirer : pourquoi la ROM ne reprogramme-t-elle pas son AAD a
chaque trame, et pourquoi ses files DMA debordent-elles en permanence.

### La ROM n'acquitte pas ses fins de DMA [2026-09-22, 12:47]

Chaine complete, chaque maillon mesure aujourd'hui :

    la ROM ne lit jamais DMA2_CTRL avec IRQ_STATE
        (0 trace « effaces a la lecture » sur un run de 2334 erreurs 24 ;
         20 sur le run precedent, contre des milliers de transferts)
      -> IRQ_STATE reste pose, ses files circulaires de 14 entrees debordent
        (`DSP Error Status: 24` = DMA_PROG|DMA_TASK, permanent depuis le boot)
      -> l'AAD n'est pas reprogrammee d'une trame a l'autre
        (98 % des bursts a 0x0cce quelle que soit la page : w_page=0 -> 1811x,
         w_page=1 -> 2115x, alors que d_dsp_page alterne bien)
      -> le burst N ecrase le N-1 avant lecture
      -> demodulation degradee sur l'intervalle dedie
        (TOA 1,9 / PM 2310 / SNR 232 contre 4,2 / ~4500 / ~500 sur TS0)
      -> ~3 blocs descendants perdus sur 4, chaque aller-retour LAPDm coute
         4 multitrames au lieu d'une
      -> le MSC lache avant le TMSI REALLOCATION COMPLETE : LU REJECT.

C'est le premier enchainement qui relie TOUT ce qu'on observe depuis ce matin,
et chaque maillon est chiffre.

L'interruption de fin de DMA est pourtant censee partir : `calypso_rhea_dma.c`
la leve si `CTRL_IRQ_MODE` est pose, et la valeur observee (0x05ab) l'a bien
(bit 7). Mais la trace « end-DMA -> INT10n » n'apparait PAS non plus.

⚠️ RESERVE SUR L'INSTRUMENT. Sur le dernier run, AUCUNE trace `[rhea-dma]` ne
sort, y compris celles qui existaient avant mes modifications et qui sortaient
la veille. Et le compteur que j'ai ajoute (« bilan : N transferts finis… »)
n'imprime jamais alors que `strings` le trouve dans le binaire EN COURS et que
la trace situee deux lignes plus bas, dans le meme bloc `if`, imprime. Cette
contradiction n'est pas resolue : tant qu'elle ne l'est pas, « 0 acquittement »
peut vouloir dire « la ROM n'acquitte pas » OU « la sortie de ce module est
perdue ». A trancher AVANT d'en tirer un correctif -- par exemple en verifiant
que stderr de calypso_rhea_dma.c arrive bien dans dsp.log (un test avec un
fprintf inconditionnel au premier appel suffit).

## 2026-09-22 16:00 — Le workflow refute deux de mes conclusions

Vingt agents relus contradictoirement. Deux de mes affirmations, que j'avais
presentees comme etablies, ne tiennent pas.

**1. « La ROM n'acquitte pas ses fins de DMA » : FAUX.**
Le code prouve le contraire sans meme lancer le banc. La trace d'acquittement
n'avait pas disparu du run : elle avait disparu de la SORTIE. `verbosite.c`
classe les lignes de stderr par mots-cles, dans l'ordre. En reformulant le
message le 2026-09-22 j'en avais retire le mot qui le placait au niveau 0 ;
ne restait que « DMA », donc niveau 3, donc invisible au `-v` par defaut. J'ai
lu une absence d'instrument comme une absence de comportement.

**2. « Le pont livre tout (manques=0), le defaut est dans la demodulation du
DSP » : NON ETAYE, et la perte majoritaire est ailleurs.**
`bsp_ts0_service()` repart par un `return` des qu'il n'a pas de burst du BTS
pour la trame reclamee — AVANT d'appeler `bsp_ts0_livrer()`. Or `joues` et
`manques` ne sont touches que DANS `bsp_ts0_livrer()`. Une trame dediee sautee
en entier n'est donc vue par aucun des deux. Mesure du workflow : **79 trames
dediees sur 172 jamais livrees, les deux compteurs a 0.** Mon « manques=0 »
ne disait pas « rien ne se perd », il disait « je ne regarde pas la ou ca se
perd ».

**3. Et la cause amont, c'est ma propre horloge.**
La boucle d'asservissement posee ce matin (periode 0,25 s, rattrapage en 400
trames) tourne a ~0,07 Hz — SOUS la cadence a laquelle la vitesse du DSP varie.
Elle ne suit plus : la phase part en cycle limite de +/- 100 trames, et a
chaque demi-tour negatif le pont se retrouve derriere la trame reclamee.
**12000 trames entierement sautees sur 120201 ticks, 10 %.** J'avais releve
« 11 a 12 par run » a 11:08 — je comptais les trames tardives, pas les trames
sautees.

Enchainement reel : mon horloge saute des trames -> le burst dedie n'est pas
la -> `bsp_ts0_service()` repart sans rien livrer -> aucun compteur ne bouge ->
je conclus que le DSP demodule mal. Trois fausses pistes de la journee
partaient de ce zero.

### Correctifs poses (16:04, binaire reconstruit)

| # | Fichier | Correctif |
|---|---------|-----------|
| 1 | `pont/trx.py` 41-43 | `HORLOGE_PERIODE` 0,25 -> 0,05 ; `HORLOGE_PHASE_N` 400 -> 100. Boucle remontee a ~1,3 Hz, au-dessus de la perturbation. Ne PAS masquer ca avec `PONT_HORLOGE_AVANCE`. |
| 2 | `pont/trx.py` | `_tn_dedie()` -> `_burst_dedie(tn, fn)`. Le test portait sur l'INTERVALLE : en CCCH+SDCCH/4 le canal dedie vit sur TS0, celui qui porte aussi FCCH/SCH/BCCH/CCCH — jamais chiffrees. Des que A5 s'activait, tout TS0 partait XORe vers le DSP et le campement se defaisait. Meme tri que `Downlink._signalling`. TCH ouvert couvert aussi (necessaire pour un appel chiffre). |
| 3 | `calypso_bsp.c` | Nouveau compteur `g_dedie_perdues` : les trames dediees sautees en entier. Publie dans `/dev/shm/calypso_bsp_dedie`. **Ne plus jamais lire `manques=0` sans lire `perdues` en meme temps.** |
| 4 | `montant.c` 550 | Le retour anticipe consomme `g_kc_liberer`. Sans ca la liberation restait armee, `seq` montait a chaque scrutation, et le pont rechargeait sans fin une cle inchangee. |
| 5 | `calypso_rhea_dma.c` | Bilan finis/acquittes promu en WARN (niveau 1) **quand il est mauvais** seulement. Pas de « ERR » force sur une ligne saine. |
| 6 | `calypso_rhea_dma.c` 554 | `CTRL_IDLE` rendu sur la sortie anticipee. Toutes les autres sorties le reposaient ; celle-la laissait le canal annoncer un transfert qui n'aurait pas lieu. |
| 7 | `start-direct.sh` | `OSMO_MOB_VTY_PORT` passe a phonesim : il retombait sur 4247 alors qu'en `--dsp` le mobile lie 4347. Le modem oFono parlait a un port que personne n'ecoute — sans erreur. |

Onze constats forts du workflow n'ont pas ete verifies (plafond a 12) : un
second passage les vaut.

## 2026-09-22 16:15 — La mecanique reelle, mesuree cette fois

Le correctif d'horloge pose a 16:04 **n'a pas corrige la perte** : premiere
mesure apres coup, `manques=2000` sur 10490 ticks, 19 % — soit pire que les
10 % que le workflow reprochait a l'ancien reglage. J'ai donc arrete de
regler a l'aveugle : `PONT_HORLOGE_PERIODE`, `_KP` et `_PHASE_N` sont
desormais lisibles dans l'environnement (valeurs par defaut inchangees), et
j'ai instrumente le SIGNE de l'ecart au lieu de le supposer.

`bsp_ts0_stocker()` retient maintenant `g_ts0_fn_max`, la trame la plus
recente recue du BTS. Sur un manque, la trace dit de quel cote vient l'ecart.

**Resultat, sans ambiguite :**

```
[ts0] tick=19316 : pas de burst BTS pour fn=18650 (manques=800/19316) ;
      derniere trame recue fn=18635, soit +15 : le DSP COURT DEVANT le BTS
[ts0] tick=20323 : pas de burst BTS pour fn=19657 (manques=1200/20323) ;
      derniere trame recue fn=19641, soit +16 : le DSP COURT DEVANT le BTS
```

L'ecart est **systematiquement positif**, de +1 a +16, jamais negatif. Le DSP
reclame des trames que le BTS n'a pas encore produites. Ce n'est pas un cycle
limite symetrique autour de zero — c'est un BIAIS constant.

**Ce que ca change.** Le workflow concluait « cycle limite de +/- 100 trames »
et recommandait explicitement de NE PAS augmenter `PONT_HORLOGE_AVANCE`, au
motif que la marge masquerait l'oscillation sans la supprimer. Ce
raisonnement vaut contre une oscillation ; il ne vaut pas contre un biais. Un
biais systematique se corrige justement par un terme d'avance. La consigne du
workflow reposait sur une hypothese que la mesure ne soutient pas.

Le taux en regime etabli reste ~20-25 % avec l'avance a 12 : de quoi expliquer
que le canal dedie ne tienne pas quatre bursts de suite, donc pas de bloc
LAPDm complet, donc pas de UA, donc pas de LU.

**Balayage en cours** de `PONT_HORLOGE_AVANCE` sur 12 / 30 / 60 / 120, 90 s de
periode calibree par point, taux de manques + `tard/UL` + ecart median. Le
reglage sera choisi sur la courbe, pas sur une intuition — la quatrieme de la
journee aurait ete une de trop.

## 2026-09-22 16:20 — Asservir le pont au DSP est une erreur d'architecture

Trois mesures successives, chacune refutant la precedente, finissent par
donner la mecanique. Dans l'ordre :

**1. Le signe de l'ecart.** Systematiquement positif, +1 a +16 : le DSP
reclame des trames que le BTS n'a pas produites. Pas un cycle limite.

**2. La perte n'est pas un taux, elle est bimodale.** Entre checkpoints :
27 %, 25 %, puis **100 % sur 200 trames consecutives**, en alternance. Des
coupures totales d'environ une seconde, pas une gigue.

**3. Le DSP ne sprinte pas — il traine.** Sonde sur
`/dev/shm/calypso_horloge`, 4591 echantillons a 5 ms :

```
temps reel GSM     : 216.7 trames/s
cadence DSP mediane: 195.7 trames/s      deficit 9.7 %
9e decile          : 197.3
plus gros bond     : 2 trames
part a plus de 2x le temps reel : 0.0 %
```

Jamais d'emballement. Un deficit **constant de 9,7 %**. Et les « 12000 trames
sautees sur 120201 ticks » du workflow, c'est 10,0 % : le meme nombre.

**4. Le journal du BTS nomme le coupable.**

```
DL1C NOTICE FN timer expire_count=7: We missed 6 timers (scheduler_trx.c:427)
DL1C ERROR  No more clock from transceiver (scheduler_trx.c:435)
```

**5. Et les sources du BTS expliquent pourquoi c'est structurel.**
`osmo-bts/src/osmo-bts-trx/scheduler_trx.c` : les trames sont battues par un
**timerfd cale en dur sur `GSM_TDMA_FN_DURATION_uS`**, 4615 us, temps reel.
`IND CLOCK` ne sert qu'a CORRIGER ce timer :

* `elapsed_fn < 0` -> « We were N FN faster than TRX, compensating », il retarde ;
* `|elapsed_fn| > MAX_FN_SKEW` (50) -> resynchronisation brutale ;
* `fn_without_clock_ind == TRX_LOSS_FRAMES` (400) -> il s'arrete.

Et surtout, ligne 571, un `TODO` du projet :

> *put this computed error_us_since_clk into some filter function and use that
> to adjust our regular timer interval to compensate for clock drift*

**Le filtre de derive n'existe pas.** osmo-bts-trx ne SAIT PAS tourner a une
cadence autre que le temps reel. Lui donner une horloge 9,7 % lente le laisse
en permanence « plus rapide que le TRX » : il compense, il resynchronise, et
entre deux son propre timer continue a battre au temps reel — produisant des
trames que le DSP n'a pas encore atteintes, et en sautant d'autres.

### Ce que ca dit de mon correctif du matin

**Asservir l'horloge du pont au DSP prend le probleme a l'envers.** Le BTS est
le maitre temps reel par construction ; on ne peut pas le ralentir. C'est le
DSP qui doit tenir la cadence. L'asservissement que j'ai pose ce matin a
supprime la derive non bornee — ca, c'etait un vrai gain — mais il l'a
remplacee par un desaccord permanent de 9,7 % que le BTS ne sait pas absorber.

**Balayage en cours** de `INSNS` (60000 / 54000 / 46000 / 38000) : cadence
mediane obtenue, deficit, taux de manques, nombre de resynchronisations du
BTS. Si la cadence suit bien 1/INSNS, le point qui annule le deficit annule
la perte. Reserve a verifier au point retenu : `INSNS` est aussi un budget de
FIDELITE — trop bas, la ROM n'a plus assez d'instructions pour finir son
travail de trame, et c'est la detection FB/SB qui tombe.

## 2026-09-22 16:35 — La cause racine, trouvee et chiffree

Deux mesures decisives, chacune reproductible.

**1. `INSNS` ne cadence rien.** Balayage sur un facteur 2 :

```
INSNS=60000 -> 195.6 trames/s      INSNS=40000 -> 195.8
INSNS=54000 -> 196.0               INSNS=30000 -> 195.7
INSNS=48000 -> 195.8
```

Plat. Le budget d'instructions du DSP n'a aucun effet : le DSP n'est pas
limite par son propre calcul.

**2. Sans pas-a-pas, la machine vole.** `LOCKSTEP=0` : **588,9 trames/s**
median, jusqu'a 797 — 2,7 fois le temps reel. L'hote n'est pas sature (32
coeurs, charge 1,97).

**Donc c'est l'echange de pas-a-pas lui-meme qui coute.** Le TICK/GO en deux
phases fait deux allers-retours de socket par trame entre QEMU et `c54x_exe`.
Le gestionnaire de trame met ~5,11 ms la ou une trame GSM en fait 4,615 : il
depasse le budget d'environ 0,5 ms.

**Et `tdma_pacer()` transforme ce depassement en perte silencieuse.**
`calypso_trx.c:1069` :

```c
target += GSM_TDMA_NS;
while (target <= now) {
    target += GSM_TDMA_NS;     /* en retard : on saute une trame */
}
```

Le stimulateur est juste — cible absolue, pas de derive accumulee — mais quand
le gestionnaire depasse systematiquement le budget, `target` est toujours
derriere `now`, la boucle avance d'une trame a chaque fois, et la cadence
effective devient 1/(duree du gestionnaire) = 195,7 trames/s. `s->fn`, lui,
n'avance que de 1 par tick : **l'interface air emulee prend 9,7 % de retard
sur le temps reel, en permanence.**

### Enchainement complet, du symptome a la cause

1. Le pas-a-pas coute ~0,5 ms/trame -> le DSP tourne a 195,7 au lieu de 216,7.
2. Mon horloge asservie repercute fidelement cette cadence lente au BTS.
3. `osmo-bts-trx` ne sait pas suivre une horloge lente : son timerfd est cale
   en dur sur 4615 us et le filtre de derive est un `TODO` vide
   (`scheduler_trx.c:571`). Il se croit en permanence « plus rapide que le
   TRX », compense, puis resynchronise.
4. Pendant une resynchronisation il ne produit rien : **coupures de ~200
   trames consecutives**, soit une seconde de silence.
5. Le canal dedie n'obtient plus 4 bursts de suite -> pas de bloc LAPDm ->
   pas de UA -> pas de mise a jour de localisation.

Les 10 % de trames sautees que le workflow avait comptes, c'est exactement le
deficit de cadence. Ce n'etait ni un cycle limite, ni la demodulation du DSP,
ni l'acquittement DMA.

### Trois voies possibles, aucune choisie sans arbitrage

| Voie | Ou | Portee |
|---|---|---|
| A. Reduire le cout du pas-a-pas | `calypso_trx.c` / `pont.c` | 2 allers-retours de socket par trame pour ~0,5 ms : c'est beaucoup trop pour un socket UNIX. Probablement une attente a timeout quelque part. Si on descend sous 4,615 ms, tout le reste tombe. **La plus propre.** |
| B. Implementer le filtre de derive du BTS | `osmo-bts/src/osmo-bts-trx/scheduler_trx.c` | Le `TODO` du projet lui-meme. Rend le BTS capable de suivre n'importe quelle cadence. Mais touche un 3e depot et le binaire `/usr/local/bin/osmo-bts-trx` sert AUSSI au mode sans `--dsp`. |
| C. Rendre la perte uniforme au lieu de groupee | `calypso_bsp.c` + `pont/trx.py` | Horloge du pont au temps reel (BTS content), DSP qui lit la trame la plus recente. On perd toujours 10 %, mais 1 sur 10 eparpillee au lieu de 200 d'affilee — un bloc LAPDm a 4 bursts et FEC survit a la premiere, pas a la seconde. Casse la continuite de FN vue par le firmware.

## 2026-09-22 16:40 — Trois hypotheses de plus, trois refutations

J'ai teste, chacune par balayage sur le banc, les trois causes candidates du
deficit de 9,7 %. **Les trois sont fausses.** Je les consigne pour qu'on ne les
re-essaie pas.

| Hypothese | Balayage | Resultat |
|---|---|---|
| Le budget d'instructions du DSP (`INSNS`) | 60000 / 54000 / 48000 / 40000 / 30000 | 195,6 / 196,0 / 195,8 / 195,8 / 195,7 — **plat sur un facteur 2** |
| Le quantum de re-essai du pas-a-pas | `GSM_TDMA_NS`/16 (288 us) puis /64 (72 us) | 195,7 / 195,5 — **aucun effet** |
| La periode du coup de pouce au CPU (`CPU_KICK_NS`) | 5,00 / 1,15 / 0,58 / 0,29 ms | 195,8 / 195,9 / 195,8 / 195,9 — **plat sur un facteur 17** |

Sur la troisieme j'ai failli me faire avoir : dans ce banc les variables
d'environnement ne vont pas toutes au meme processus (cf. la note
`env-calypso-quel-processus`), donc un balayage sans effet peut simplement
vouloir dire que la variable n'arrivait pas. Verification faite dans
`/proc/<qemu>/environ` : `CALYPSO_CPU_KICK_NS=288461` y etait bien. La
refutation tient.

Les deux valeurs par defaut ont ete **remises a l'original** — je ne livre pas
un changement de comportement que la mesure ne justifie pas. Les molettes
(`CALYPSO_PONT_RETRY_DIV`, `CALYPSO_CPU_KICK_NS`) restent, elles servent a
mesurer.

### Ce qu'il reste, et pourquoi INSNS ne pouvait pas marcher

Les 5,11 ms par trame sont du **travail**, pas de l'attente :

* sans pas-a-pas, QEMU seul fait 588 trames/s, soit 1,70 ms par trame ;
* le pas-a-pas **serialise** l'ARM et le DSP au lieu de les laisser se
  recouvrir ; le reste, ~3,4 ms, c'est l'interpreteur C54x qui execute la
  trame.

Et `INSNS` ne pouvait rien y faire : c'est un **plafond**, pas une quantite de
travail. La ROM finit son travail de trame et passe en idle bien avant de
l'atteindre — baisser le plafond ne retire donc aucune instruction. C'est
pour ca que le balayage est plat, et j'aurais du le prevoir avant de le lancer.

**La piste suivante est la VITESSE de l'interpreteur**, pas son budget :
combien d'instructions la ROM execute reellement par trame, et a quel debit
l'interpreteur les rend (~7 MIPS d'apres le calcul inverse). Les sondes de
`c54x_probes.c` et les copies memoire par trame de `PONT_NB_DEBUG` sont les
premieres choses a chiffrer.

## 2026-09-22 18:05 — SB : le TOA n'est pas la cause. La fenetre, peut-etre.

**Refutation de ma propre these du jour.** J'ai cru, et ecrit, que la SB
echouait parce qu'elle atterrit a TOA 7 au lieu de 23. La mesure dit non :

```
[sb] fn=302  toa=11243  a_sch=0100  crc_ko
[sb] fn=333  toa=8743   a_sch=8000  CRC_OK     <- 8743 % 156 = 7
[sb] fn=371  toa=8743   a_sch=0100  crc_ko     <- meme 7
[sb] fn=402  toa=8743   a_sch=0100  crc_ko     <- meme 7
```

**Le meme TOA donne CRC bon et CRC faux.** Le TOA ne discrimine pas. Le
correctif pose sur `g_toa_bias` perd donc sa justification (il est de toute
facon inerte, voir plus bas).

**Au passage, deux autres choses que j'avais dites et qui sont fausses :**
* « TOA=5 est une anomalie, le nominal est 23 » -- non : `calypso_bsp.c:1505`
  dit que 5 est la valeur attendue d'un burst NORMAL sur ce banc (marge de 3
  echantillons). Les 23 ne concernent que la SB (marge de 21).
* « La DMA est en mode continu quand le burst arrive » -- non : `one_shot=1`
  apparait 760 fois sur 3389.

### Ce que la mesure etablit, en revanche

Longueurs de fenetre RIF reellement demandees sur un run de 2 min :

| fenetre  | occurrences | `nwin = len/2` |
|----------|-------------|----------------|
| 302 mots | 3316        | 151            |
| 128 mots | 73          | 64             |

**Aucune fenetre >= 190.** Or le cadrage de `calypso_bsp.c:1527` choisit sa
marge ainsi : `nwin >= 190 ? 21 : nwin >= 150 ? 3 : 0`. La fenetre SB attendue
-- ALGTH 764, donc `nwin` 382 -- **n'existe jamais**. La branche a 21
echantillons est inatteignable par construction, et la SB est demodulee dans
une fenetre de burst normal.

C'est aussi pourquoi la sonde `[cadre]` compte **0** alors que `one_shot=1`
arrive 760 fois : le bloc est bien garde par `one_shot`, mais aucune fenetre
n'atteint jamais le seuil SB.

### Etat des deux correctifs du jour sur ce chemin

| Correctif | Verdict |
|---|---|
| `calypso_bsp.c` : appliquer `g_toa_bias` a la marge SB | **INERTE** -- le bloc n'est jamais atteint (`[cadre]=0`), et sa justification est refutee (meme TOA, issues opposees). A retirer ou a garder en dormance documentee. |
| `gsm322.c` : `SYNC_RETRIES_CONN 8` | **tient jusqu'ici** -- 0 `LOS during RACH` sur 5 tentatives reparties sur deux runs, contre 4 sur 8 avant. Pas encore etabli, mais rien ne le contredit. |

### La boucle TOA, defaut reel mais secondaire

`calypso_bsp_toa_feedback()` est appelee (`pont.c:1357`, `en=1` verifie), elle
integre dans `g_toa_bias`... que **personne ne lisait** avant aujourd'hui,
malgre le commentaire « samples, applied to the DARAM placement ». Et son
entree est instable : `within = toa % 156` donne 60, puis 11, puis 48 d'un
appel a l'autre -- le signe de l'erreur alterne, l'integrateur fait +/-1 et
revient a zero. Meme branchee, elle ne pourrait pas rattraper 16 echantillons.
Deux defauts distincts, a traiter ensemble ou pas du tout.

### La question suivante, et elle est nette

Pourquoi la ROM n'arme-t-elle jamais une fenetre de 764 mots pour la SB ?
C'est elle qui programme ALGTH. Soit elle ne le fait pas, soit le modele RHEA
ne le lui rend pas. C'est mesurable : tracer les ecritures de ALGTH par la ROM
dans `calypso_rhea_dma.c`, et comparer a ce que `calypso_rhea_dma_get_len_words()`
rend au moment de la SB.

## 2026-09-22 18:35 — Le recalage TS0 : mesure A/B, et refutation de mon predicteur

Hypothese : le DSP court devant le BTS (ecart toujours positif, +1 a +40) parce
que l'offset tick->trame BTS est pose une seule fois et jamais revu ; quand le
BTS s'arrete pour resynchroniser, le DSP reclame des trames inexistantes et le
flux se troue. Or un bloc LAPDm, ce sont QUATRE bursts consecutifs. Correctif
pose : reculer l'offset pour repartir de la trame la plus recente, en
s'appuyant sur le contrat du mode STREAM (« only the ORDER matters »).

**A/B, 2 x 6 min, meme banc, meme protocole :**

|                      | temoin | recalage |
|----------------------|--------|----------|
| perdues / joues      | 20/132 |   0/161  |
| recalages (recul)    |   0    | 433 (433)|
| MDL-ERROR            |   18   |     4    |
| LU ACCEPT / REQUEST  |  1/3   |    1/2   |
| **trames jetees**    |  686   | **1409** |
| bits faux (mediane)  |   95   |    96    |

**Ca ne marche pas.** `perdues` tombe a zero et MDL-ERROR est divise par
quatre, mais les trames jetees DOUBLENT et la mediane de bits faux ne bouge
pas. Le LU reste a 1 dans les deux bras.

Explication qui colle : en remplacant une trame absente par la plus recente
disponible, on ne livre pas un trou mais **le mauvais burst a la bonne place**.
Pour le desentrelaceur, une donnee fausse mais plausible est pire qu'une
absence -- il ne peut plus la traiter comme un effacement. J'ai converti des
effacements en erreurs.

**Corollaire, et c'est le point important : « perdues » n'est PAS un
predicteur du succes.** La correlation que j'avais tiree de trois runs
(7 % -> LU accepte, 0 % -> accepte, 42 % -> rejete) ne survit pas au test
controle. Trois points suffisaient a la suggerer, pas a l'etablir.

**Defaut remis a OFF** (`CALYPSO_BSP_RECALE=1` pour le reessayer). Comme pour
le quantum de re-essai et `CPU_KICK_NS` : on ne livre pas un changement de
comportement que la mesure ne justifie pas.

Deux pistes si on y revient : ne recaler que HORS du canal dedie, ou marquer
le burst rejoue comme peu fiable pour que le desentrelaceur l'efface au lieu
de le croire.

**Defaut de mesure a signaler** : le chemin de recalage sort avant `manques++`,
donc il aveugle ce compteur. Les deux bras n'etaient pas comparables sur cette
metrique. C'est moi qui ai casse l'instrument en posant le correctif -- exactement
le genre de piege que `g_dedie_perdues` avait ete ajoute pour eviter ce matin.

### Ce que la journee laisse debout

* LU : aboutit, chiffre A5/1, TMSI committe cote VLR. Repete de nombreuses fois.
  Quand le canal est propre, la transaction complete prend 4 secondes
  (assignation -> RR_EST_CNF -> IDENTITY -> LOC_UPD_ACCEPT -> TMSI REALLOC).
* `SYNC_RETRIES_CONN 8` (gsm322.c) : 0 `LOS during RACH` sur toutes les
  tentatives depuis qu'il est pose, contre 4 sur 8 avant. Le seul correctif du
  jour qui tienne.
* Le SMS atteint desormais `MMSMS-EST-CNF` puis `WAIT_CP_ACK`, et `SAPI 3
  established` a ete vu. Avant il mourait en `MM_CONN_PENDING`.
* Une transaction Call Control a ete allouee pour la premiere fois
  (callref 0x138c), finie en `Timeout of T308`.
* Defaut restant : le descendant dedie se corrompt (69 a 99 bits faux sur 184).
  Observation non expliquee, relevee a 18:23 : le BER monte MONOTONEMENT de 43
  a 95 en 17 s a `lev >= -47` constant. Une rampe, pas des creneaux -- ce qui
  ne ressemble pas a une perte de bursts par paquets et suggere un
  desalignement cumulatif. A creuser.

## 2026-09-22 19:00 — Deux correctifs de plus, et le SMS montant passe

### 1. Effacement au lieu de la page perimee (calypso_bsp.c)

Trouve par l'arbitrage du workflow `wahnj19z5`, qui a au passage REFUTE 3/3 ses
propres trois voies (cout du pas-a-pas, filtre de derive du BTS, perte
uniforme). Mecanisme verifie a la main, quatre points :

* `calypso_bsp_rx_burst()` est le seul ecrivain de la DARAM des bursts, et
  n'est appelee que depuis `bsp_ts0_livrer()` -- que le chemin de manque saute ;
* `calypso_rif_drain()` rend 0 sur FIFO vide ;
* le transfert sort alors par `if (got <= 0) break` SANS rien ecrire : **la
  page API garde le burst du tick precedent** ;
* sur le chemin DRR, le source dit lui-meme : « On an empty FIFO, DRR keeps its
  last value [...] returning 0 would fabricate a sample ».

Donc une trame manquante n'est pas un trou : **c'est la trame precedente
rejouee**, et le decodeur tourne dessus. Signature mesuree : les blocs rejetes
ont un nombre d'erreurs IDENTIQUE, 17 rejets = 96 neuf fois, 105 cinq fois. Un
canal bruite ne rend pas neuf fois le meme compte.

C'est aussi pourquoi mon A/B du recalage etait aveugle : ses deux bras
substituaient un burst FAUX (le plus recent d'un cote, le precedent de
l'autre), **jamais un effacement**.

Resultat, 2 min : MDL-ERROR 18 -> **0**, LU accepte au PREMIER essai,
trames jetees 114/min -> 66/min, perdues/joues 13 % -> 0,6 %.
Reserve : l'histogramme des comptes d'erreurs reste concentre, donc la
signature n'a pas disparu. Non explique.

### 2. Ne plus jeter la premiere FACCH montante (pont/uplink.py)

`_poll_facch()` faisait, au changement d'epoque TCH :
`skip_pending()` + `return`. Or `tch.seq` est incremente par `tch.arm()`,
appele quand le pont decode l'ASSIGNMENT COMMAND descendante -- et
l'ASSIGNMENT COMPLETE est LA PREMIERE chose que le mobile emet sur le nouveau
TCH. Elle tombait dans cette fenetre et etait marquee « deja vue ».

Mesure, appel vers 600 a 18:55 : le mobile emet bien « ASSIGNMENT COMPLETE
(cause #0) » ; le pont journalise « FACCH montante » SANS le suffixe
« , ASSIGNMENT COMPLETE » ; le BSC conclut « Assignment failed in state
WAIT_RR_ASS_COMPLETE, cause EQUIPMENT FAILURE: Timeout ».

La SACCH garde le saut (un rapport de mesure perime ne sert a rien), la FACCH
est desormais traitee. `PONT_FACCH_SKIP=1` retablit l'ancien comportement.

### Ce que le banc fait maintenant

* LU accepte au premier essai, chiffre, TMSI committe cote VLR.
* **SMS MONTANT ARRIVE AU RESEAU** : `db.c:695 Stored SMS id=33 in DB`.
* **SETUP d'appel recu par le MSC** : `gsm_04_08_cc.c:704 SETUP to 600`.
* Restent : la livraison MT du SMS (pas de CP-ACK) et l'aboutissement de
  l'appel.

### Deux erreurs de lecture a noter, meme cause qu'au matin

J'ai affirme (a) qu'aucun `L1CTL_RACH_REQ` n'existait, (b) que le pont ne
suivait pas le mobile sur le TCH. **Les deux etaient faux**, demolis par le
journal complet deux commandes plus tard : dans les deux cas un `head -10` ou
un `tail -6` avait tronque la sortie. C'est exactement la faute du matin avec
`manques=0` : conclure d'une absence sans verifier que l'instrument regardait
au bon endroit. A surveiller.
