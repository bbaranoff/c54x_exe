# c54x_exe — le DSP Calypso sans QEMU

Fait tourner la **mask-ROM TI du TMS320C54x** seule : pas de QEMU, pas d'ARM,
pas de firmware osmocom-bb.

```bash
make
./c54x_exe --trames 200
./c54x_exe --trames 50 --verbeux          # une ligne par trame
./c54x_exe -vv                            # traces du coeur : -v .. -vvvvvv, voir --help
```

Par defaut seules les erreurs du coeur C54x passent sur stderr ; le bilan dit
combien de lignes ont ete masquees et avec quel `-v` les voir. `-vvvvvv` rend
stderr brut. Le classement est par mots-clefs sur le nom de la sonde
(`src/verbosite.c`), pas par liste : une sonde nouvelle tombe dans un niveau
raisonnable sans declaration.

## Lancer exe par exe : le cote mobile, sans le reseau

[2026-09-17] Quatre executables, dans cet ordre, chacun attendant le
precedent. `run.sh` fait exactement ca ; ce qui suit est la version a la main,
pour voir ce que chaque etape produit. Aucun element reseau (BTS, BSC, MSC,
pont gr-gsm) : le mobile cherche une cellule et n'en trouve pas, c'est attendu
tant qu'aucun burst n'est injecte dans le DSP. Avec la BTS et `PONT=1`, voir
l'etat du 2026-09-23 plus bas.

```bash
./run.sh              # tout, dans l'ordre        ./run.sh --status   qui tourne
./run.sh --logs       # suivre les 4 journaux      ./run.sh --stop     tout arreter, nettoyer
./run.sh --step 3     # une seule etape (les precedentes doivent tourner)
INSNS=8000 VERB=-vv ./run.sh            # budget DSP par trame (80000 par defaut), niveau de traces
MODE=grgsm ./run.sh                     # l'autre montage : couche 1 gr-gsm dans QEMU, sans c54x_exe
PONT=1 ./run.sh                         # avec l'etape 5 (le pont TRX) ; defaut PONT=0 : le mobile seul
IQ=cell ./run.sh                        # c54x_exe fabrique une cellule GMSK (FCCH/SCH) a chaque trame
LOCKSTEP=0 ./run.sh                     # horloge murale : QEMU n'attend pas le DSP et saute des trames (defaut 1)
```

`run.sh` connait deux montages (`MODE=dsp`, le defaut, et `MODE=grgsm`) et une
cinquieme etape, le pont TRX, qui relie le BTS osmo-bts-trx (TRXD 5700-5702) a
la couche 1 du mobile : GSMTAP 4730/4731 pour gr-gsm, `--dsp-port 6702` vers le
BSP de `c54x_exe` pour le DSP. [2026-09-23] Le defaut est `PONT=0` (depuis le
17/09) : `PONT=1 ./run.sh` ajoute l'etape 5. En `MODE=dsp` elle lance
`pont/pont_dsp.py` (sous-paquet `pont/dsp/`, bascule TCH suivie par le
firmware), en `MODE=grgsm` `pont/pont.py` ; `PONT_PY` force l'un ou l'autre. Le
pont n'est plus lance avec `--no-record` (`PONT_AIRREC=1` par defaut, pour la
FFT du panneau ; `PONT_AIRREC=0` le remet). Les memes
chaines se lancent par leur nom : `qosmo-dsp`, `qosmo-grgsm`, et exe par exe
`c54x_exe`, `osmocon`, `grgsm_exe` (`/usr/local/bin`). Le detail processus par
processus, avec les lignes de journal attendues, est dans `LAUNCH.md`.

Journaux et pid dans `/tmp/c54x-pont/`. [2026-09-23] Le mobile de ce montage
utilise maintenant les memes sockets que le run sans `--dsp` :
`/tmp/osmocom_l2` et `/tmp/osmocom_sap` (plus de variantes `_pont`) ; VTY
mobile 4347 et moniteur QEMU `/tmp/qemu-monitor-pont.sock` inchanges. Lancer un
banc grgsm et celui-ci en meme temps les fait se disputer : compromis assume,
cf. l'en-tete de `mobile_pont.cfg`. `--stop` efface aussi `/tmp/osmocom_sap`,
`/tmp/ms_data`, `/dev/shm/calypso_horloge` et, en `MODE=dsp`,
`/dev/shm/calypso_tch_cfg`.

Autres variables de `run.sh` (2026-09-23) :

- `INSNS=80000` : 60000 debordait en TCH (jusqu'a 87000 insn/trame).
- `GDB=1` : gdbstub QEMU en `tcp:127.0.0.1:1234` et console `telnet 0 44444`
  (`qosmo-dsp/tools/gdb-telnet.py`, journal `gdb.log`) ; `GDB=0` coupe les deux.
- `ASSEMBLY_LOGS=1` : trace asm de l'ARM dans `qemu-asm.log`
  (`ASSEMBLY_LOGS_FLAGS`, defaut `in_asm,exec,nochain` ; `ASSEMBLY_LOGS_FILTRE`
  pour `-dfilter`).
- poses d'office : `CALYPSO_PONT_RETRY_DIV=64` sur QEMU (relance vers le DSP
  toutes les trame/64 au lieu de trame/16), `CALYPSO_BSP_ATTENTE_MS=40` sur
  `c54x_exe` (attendre une trame livree en retard par la BTS plutot que la
  jouer en effacement), `L23_SYNC_RETRIES_SELECTION=8` sur le mobile en
  `MODE=dsp` (defaut du binaire, 1, ailleurs).
- `PANNEAU_LOGS` : `qemu.log`, `osmocon.log` et `mobile.log` s'ecrivent dans
  `/run/user/0/osmo-nitb/logs/`, `pont.log` dans `/dev/shm/pont.log`, ceux que
  suit le panneau ; `$RUNDIR` n'en a que des liens (`dsp.log` y reste un vrai
  fichier). `PANNEAU_LOGS=none` pour ne rien toucher.
- `JOURNAUX_GARDES=10` : au lancement et a `--stop`, la session precedente est
  rangee dans `$RUNDIR/archives/<date>/`, avec `bsp_dedie.txt`.

### Le lien montant (RACH, SDCCH, SACCH, parole)

En montage `dsp`, la couche 1 gr-gsm de QEMU est desactivee
(`CALYPSO_DSP_EXTERN=1`), donc les hooks qui publiaient le montant cote QEMU ne
tirent plus. C'est `src/montant.c` qui s'en charge : une scrutation de l'API RAM
partagee a chaque trame, qui alimente les memes side-bands que consomme
`pont.py` (`pont/uplink.py`) :

    /dev/shm/calypso_rach          RACH (ra, bsic) lu dans NDB d_rach
    /dev/shm/calypso_sdcch_ul      bloc L2 montant (a_cu)
    /dev/shm/calypso_tch_facch_ul  FACCH montante
    /dev/shm/calypso_tch_sacch_ul  SACCH montante
    /dev/shm/calypso_tch_ul        anneau de trames de parole

Sans lui : `pont.log` affiche `UL bursts=0 rach=0`, la BTS ne voit aucun
acces aleatoire, le mobile epuise ses huit tentatives et il n'y a jamais de
LOCATION UPDATING ACCEPT. Reglages : `MONTANT=0` coupe la publication,
`MONTANT_DEBUG=N` regle le nombre d'evenements imprimes (20 par defaut),
`MONTANT_RACH_SUR_DRACH=1` revient a l'ancien declencheur (transition de
`d_rach` au lieu de `d_task_ra`).

[2026-09-23] Aussi dans `montant.c` :
- le Kc, publie dans `/dev/shm/calypso_kc_l1` (`MONTANT_KC=0` coupe) ;
- la parole montante, au format TI (`io-tch-format ti`), est convertie en FR
  TS 101 318 avant publication (`MONTANT_PAROLE_TI=0` = passage brut ; c'etait
  l'ancien comportement, qui donnait un echo sature) ;
- la bascule SDCCH <-> TCH du BSP suit la tache posee par le firmware dans la
  page W (`d_task_d` = TCHT 13, TCHA 14 ou TCHD 28 ; retour sur ALLC 24) ;
  `calypso_tch_cfg` ecrit par le pont n'est plus qu'une annonce
  (`MONTANT_TCH_TACHE=0` = armement a l'annonce) ;
- sondes `[a_fd]`, `[a_dd]` (TCH seulement ; `MONTANT_AFD=0`, `MONTANT_ADD=0`),
  `[a5-arm]` et `[d_fn]`.

Cote QEMU, `MONTANT_REQREF=0` coupe la correction de la reference de requete
des IMMEDIATE ASSIGNMENT (`calypso_trx.c`) : sans elle le mobile jette
l'assignation, parce que pont.py emet l'access-burst sur sa propre horloge et
que `gsm48_match_ra()` exige une correspondance exacte du numero de trame.
Causes et mesures : `MAILBOX.md`, sections « Pas de LU ACCEPT » et « Le RACH
passe, l'IMM ASS revient ».

### 1. Le DSP : `c54x_exe --arm`

```bash
cd /opt/GSM/c54x_exe && ./c54x_exe --arm -v
```

Cree `/dev/shm/calypso_api_ram` (la fenetre API, 16 Ko, qui EST
`data[0x0800..]` du C54x) et `/tmp/calypso_dsp.sock`, charge les 7 sections de
ROM, `c54x_reset()`, puis attend l'ARM. Attendu :

```
pont : en attente de l'ARM sur /tmp/calypso_dsp.sock (API RAM : /dev/shm/calypso_api_ram)
```

`--insns` : budget d'instructions par trame TDMA. [2026-09-23] Defaut 200000
avec `--arm` (2300 sans), plancher 32000 en `--rejouer` ; `run.sh` passe 80000.
Depuis le 2026-09-23 les sondes pures du coeur sont coupees par defaut
(`CALYPSO_SONDES=1`, `CALYPSO_DEBUG` ou `-vvvv` pour les rallumer), pour
accelerer le coeur (gain a remesurer). En pas-a-pas (`LOCKSTEP=1`,
defaut de `run.sh`) QEMU attend le DSP au lieu de lui sauter des ticks ; le DSP
rend `PONT_DONE` des le burst depose et finit la trame en parallele de l'ARM
(`PONT_DONE_TOT=1` par defaut, `0` = ancien ordre). La ligne `[chrono]` de
`dsp.log`, toutes les 1000 trames, dit ou passe le temps
(`qemu | A | go | B | apres DONE`, en ms par trame).

### 2. L'ARM : QEMU `qosmo` avec `CALYPSO_DSP_EXTERN=1`

```bash
cd /opt/GSM/qosmo && CALYPSO_DSP_EXTERN=1 build/qemu-system-arm -M calypso -cpu arm946 \
  -display none -parallel none -serial pty -serial pty \
  -monitor unix:/tmp/qemu-monitor-pont.sock,server,nowait \
  -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf
```

Sans `-kernel`, le CPU part a 0 et plante a `0x840000` : l'ELF est
obligatoire, le romload d'osmocon ne charge rien (le stub UART ne fait
qu'acquitter). Attendu sur stderr :

```
char device redirected to /dev/pts/N (label serial0)        <- le pty modem, pour osmocon
[trx] pont DSP : API RAM partagee (/calypso_api_ram) + socket /tmp/calypso_dsp.sock - ...
calypso: couche 1 « grgsm » desactivee (DSP externe, CALYPSO_DSP_EXTERN)
[trx] pont DSP : timer de boot lance (echange DSP seul, sans IRQ TPU, ...)
[trx] pont DSP : RESET_DSP relache par le firmware -> PONT_RESET (fn=0)
[trx] pont DSP : le TDMA du firmware prend le relais du timer de boot (N trames de boot)
```

Et cote `c54x_exe` :

```
pont : ARM connecte, API RAM 32768 mots, 200000 insn/trame
pont : RESET #1 (DL_STATUS=0x0000) fn=0 pc=0xff80
pont : DSP boote (premier IDLE) fn=0 insn=5701
  fn=1251  page=0 insn=428  IDLE  d_fb_det=0  a_sch=0000 ...
```

Le firmware a asserte puis relache `RESET_DSP` (registre CNTL_RST), la ROM est
repartie de `0xff80`, a pose IDLE, s'est parquee ; l'ARM a envoye
`COPY_BLOCK` vers `0x7000` ; la ROM a saute dans sa L1 et ecrit sa version.
Verifier depuis le moniteur QEMU :

```bash
printf 'xp /96bx 0x008305f0\n' | nc -U /tmp/qemu-monitor-pont.sock    # printf_buffer du firmware :
                                                                      # "DSP API Version: 0x4e2a 0x491a"
od -An -tx2 -j 0x01B4 -N 2 /dev/shm/calypso_api_ram                   # 3606 = version ecrite par la L1 DSP
```

### 3. osmocon : le relais L1CTL

```bash
/opt/GSM/osmocom-bb/src/host/osmocon/osmocon -m romload -i 100 -p /dev/pts/N \
  -s /tmp/osmocom_l2 /opt/GSM/firmware/board/compal_e88/layer1.highram.bin
```

`/dev/pts/N` est le `serial0` de l'etape 2. Attendu : `Received ident ack`,
`Progress: 100%`, puis **`Received branch ack, your code is running now!`**.
A partir de la, osmocon relaie le L1CTL entre le firmware et
`/tmp/osmocom_l2`. Si le telechargement reste a `starting download` sans
progres : un osmocon precedent a ete tue a mi-bloc et le stub romload de
l'UART attend la fin de ce bloc - relancer QEMU (etape 2) puis osmocon.

### 4. Le mobile

```bash
mobile -c /opt/GSM/c54x_exe/mobile_pont.cfg        # copie de ~/.osmocom/bb/mobile.cfg :
                                                   # layer2-socket /tmp/osmocom_l2, vty 4347
```

Attendu dans les 5 s : `L1CTL_PM_REQ`, `L1CTL_RESET_REQ: FULL!`,
`L1CTL_FBSB_REQ (arfcn=514 ...)` cote osmocon ; cote `c54x_exe`, le passage de
428 a ~570 insn par trame et `a_sch=0100 ...` : la L1 ARM a programme
`d_task_md=5` (recherche FB) et le DSP l'execute. Cote mobile,
`FBSB RESP: result=255` en boucle **sans pont** : `calypso_bsp.c` attend les
bursts descendants en UDP sur le port 6702 et personne ne les envoie. Avec la
BTS et `PONT=1`, c'est `pont_dsp.py` qui les envoie.

```bash
telnet 127.0.0.1 4347            # VTY du mobile ; « show ms »
od -An -tx2 -N 16 /dev/shm/calypso_api_ram     # page d'ecriture : d_task_md au 5e mot
```

### Le pont, en une page

Protocole dans `qosmo/include/hw/arm/calypso/calypso_dsp_pont.h`, cote QEMU
dans `qosmo/hw/arm/calypso/calypso_trx.c` (`pont_*`), `calypso_soc.c`
(CNTL_RST -> RESET_DSP) et `calypso_l1_dispatch.c` (`calypso_l1_disable`) ;
cote DSP dans `src/pont.c`. Il recopie section par section le `tdma_tick` de
qosmo-dsp : DMA tick, boot jusqu'au premier IDLE, IRQ TPU-frame si l'IMR
l'arme, un budget de `c54x_run`, front occupe -> IDLE = IRQ API cote ARM.

Quatre pieges rencontres, tous dans le pont et non dans le DSP :
- **la boucle principale de QEMU ne doit pas attendre le DSP** : une trame de
  64000 insn dure ~26 ms, l'ARM (qui a besoin du verrou global a chaque acces
  MMIO) etait affame et restait dans `hwtimer_config`. Le pont est en
  pipeline : DONE(N) releve au tick N+1, tick saute si le DSP est en retard.
- **le DSP doit tourner avant que le firmware n'active le TPU** : un timer de
  boot cadence l'echange seul, sans IRQ TPU-frame (l'ARM n'a pas encore ses
  vecteurs), jusqu'a ce que le vrai tick prenne le relais.
- **le segment partage doit ETRE `data[0x0800..]` du C54x**, pas une copie :
  le coeur ecrit sa fenetre API dans `data[]` et ne recopie `api_ram` que sur
  certains chemins. `pont_allouer_dsp()` aligne `data[]` sur une page et y
  pose le segment en `MAP_FIXED` ; `api_ram` en est l'alias.
- [2026-09-23] **sur le TCH, jouer la phase A jusqu'a l'IDLE (budget/2 au
  plus) avant de deposer le burst** : pour une tache TCHA (SACCH/TF), la ROM
  demodule au debut de N+1 le burst SACCH de N qu'elle a laisse en `0x0cce`. Deposer des l'armement de la
  fenetre l'ecrasait (a_cd FIRE KO a chaque bloc, LOS).
  `PONT_TCH_DEPOT_IDLE=0` retablit l'ancien depot ; trace `[depot_tch]`.

Et une chose a savoir sur `qosmo` : `fw_console.c` lit `printf_buffer` a une
adresse codee en dur (`0x831018`) qui n'est pas celle de cet ELF (`0x8305f0`,
`nm layer1.highram.elf | grep printf_buffer`), donc pas de `[fw-console]`.

### Outils de rejeu hors banc

[2026-09-23] Sur tout canal dedie, `c54x_exe` enregistre les ecritures de l'ARM
dans l'API RAM, les TICK et les livraisons d'I/Q du BSP dans
`/dev/shm/calypso_rejeu_tch.bin` (`CALYPSO_REJEU_ENREG=0` coupe, 60000
livraisons au plus).

```bash
tools/rejeu_banc [fichier] [ticks]               # rejoue hors banc, imprime chaque a_cd / a_fd
REJEU_SANS_D=1 tools/rejeu_banc                  # garde l'etat du boot local au lieu de 'D'
tools/sacch_tf_decode [fichier] [TN=2] [Kc]      # decode hors DSP la SACCH/TF jouee par le BSP
make isa_test && ./isa_test tools/isa_tests.txt  # conformite ISA (exemples SPRU172C)
```

`sacch_tf_decode` lit `/dev/shm/calypso_sacch_tf.bin` (sonde `[sacch_tf]` du
BSP) avec le Kc de `calypso_kc_l1` : si la SACCH decode la et pas dans la ROM,
le defaut est apres le BSP ; sinon, dans ce que le BSP recoit. Aucune cible
Makefile pour ces deux outils : les binaires sont commites a cote des sources.

## Pourquoi c'est possible

Mesure faite sur les objets compilés **avant** d'écrire une ligne : sur les
26 631 lignes de couche 1 C54x, l'accroche à QEMU tient en **14 symboles**.

| fichier | symboles QEMU |
|---|---|
| `calypso_c54x.c` (21 296 l.) | **2** — des mutex |
| `calypso_{arm2dsp,dma,fbsb,mailbox,rhea_dma,rif,twl3025}.c` | **0** |
| `calypso_full_pcb.c`, `calypso_tint0.c` | 8 — le câblage, pas le DSP |

Les cales sont dans [`qosmo/contrib/hors-qemu/`](../qosmo/contrib/hors-qemu) :
~120 lignes de doublures d'en-têtes (`qemu/osdep.h`, `thread.h`, `timer.h`…)
et 73 lignes d'équivalents POSIX. Rien n'y modélise QEMU — le jour où une cale
doit devenir autre chose qu'un `pthread`, c'est que le DSP s'est mis à dépendre
de QEMU, et il faut le savoir.

**Les sources ne sont pas recopiées.** Ce binaire compile celles de
`/opt/GSM/qosmo` (`QOSMO=... make` pour pointer ailleurs). [2026-09-23] `make`
reconstruit `c54x_exe` a chaque appel (cible `.PHONY`, qui depend aussi des
en-tetes), en `-O3 -march=native` : plus besoin de `make clean`.
`calypso_a5.c` (le coprocesseur A5 sur les ports XIO `0x2800..0x2818`,
`CALYPSO_A5=0` le coupe) est compile dedans. Recopier, c'était
refaire la divergence que `qosmo` vient de supprimer. La seule copie est
`src/pcb-minimal.c`, quatre helpers DARAM repris mot pour mot, et elle est
signalée comme telle dans le fichier.

## Ce que ça mesure, et ce que ça ne mesure pas encore

**État au 2026-09-23 (runs du banc DSP de 20:22 et 20:32).** En montage dsp
avec la BTS (`PONT=1`), le DSP détecte FB et SB, décode les BCCH (SI1-4), le
mobile obtient le LU ACCEPT, le premier SMS MT est livré de bout en bout
(2026-09-23 11:06), l'appel passe l'ASSIGNMENT (2026-09-22 19:47) et la
bascule TCH suit la tâche du firmware ; l'A5 est modélisé dans le DSP
(`calypso_a5.c`), la parole montante est convertie TI -> FR. Run de 20:22,
constaté dans les journaux : LU, appel MO vers l'écho 600 (ACTIVE 20:22:55,
DISCONNECT 20:23:27), SMS MO et MT dans les deux sens, appel MT depuis
100102 (ACTIVE 20:24:28, release normal), A5/1 confirmé par la BTS sur les
cinq établissements, parole audible dans les deux sens (décodage canal
TCH/F descendant par la ROM TI, codec GAPK FR et codage montant sur l'hôte),
29 513 trames avec un seul tick sauté (au boot, fn=0). Restent ouverts, par
ordre d'importance :
- **B_BFI sur toute la parole** : la ROM marque chaque trame TCH/F comme
  mauvaise. Run de 20:32, sonde `[a_dd]` étendue (`src/montant.c`
  `sonde_add`, non commitée) : `vues=2200 bfi=2200` ; `err` (a_dd_0[2],
  erreurs rapportées par la ROM) vaut 0 sur 19 des 20 premières trames
  après la bascule (`c214` ; la 18e, fn=5912, est `8084` à 58), puis 15 à
  93. Les trames sont réellement dégradées ; le
  FR reste intelligible parce que le firmware ne remonte pas le BFI
  (`prim_tch.c:327` ne teste que B_BLUD). Signal (BSP, IQ, égalisation) ou
  cœur C54x (Viterbi, recomptage) : à trancher par comparaison bit à bit
  avec les trames de la BTS. Le `ko` de la sonde (B_FIRE1) ne dit rien sur
  la parole ;
- la SACCH en TCH : le correctif MVKD/MVDK (`qosmo` `c54x_exec.c`,
  `CALYPSO_MVKD_DMAD_AVANT=1` = ancien ordre ; garde `[garde-3d89]` dans
  `c54x_mem.c`) tient au run de 20:22 (deux appels complets, aucun bloc
  SACCH/TF jeté par le mobile hors bascule et libérations, aucune LOS,
  aucune ligne `[garde-3d89]`). Mais au run de 20:32
  le premier appel tombe en LOS (20:32:45) : SACCH/TF FIRE KO à chaque bloc
  dès fn=6095, `err` de la parole 63 à 93 de fn=6273 à 9306 contre 15 à 38
  sur le troisième appel, sain ; le deuxième reste bloqué en attente
  de la connexion MM (T3230). Garde muette : autre
  cause, non localisée ;
- le SDCCH/8 descendant : au run de 20:22 le mobile jette 27 trames sur
  SDCCH/8 (4 à 7 par session dédiée), dont 15 SACCH (ligne « LOSS counter
  for ACCH ») et 12 du canal principal ; suspect, le BSP sur le SDCCH/8
  (table 45.002) ;
- la fenêtre SB, rarement armée par la ROM (d'où
  `L23_SYNC_RETRIES_SELECTION=8`) : une synchro sur trois à cinq ;
- la marge temps réel : en TCH, `[chrono]` donne A 0.33 + go 0.40 + B 0.16
  + après DONE 3.37-3.68 ms, soit 4.3 à 4.6 ms de travail DSP pour 4.62 ms.

Ne sont pas des anomalies : les échecs CRC du moniteur TCH du pont tant que
le RTP ne coule pas (décodage du pont, indépendant du DSP), et l'UA / SABM
répétés, disparus au run de 20:22 avec `osmo-operator` `pont/dsp/clock.py`
(`PONT_AVANCE_MIN=10`) : aucune ligne SABM dans les journaux osmocom (ni
ERROR INDICATION au BSC), marge
DL réelle min +0 au premier relevé (20:22:47), +9 ensuite, +13 à +15 à
partir de 20:23:17.

Le détail, jour par jour : `MAILBOX.md`.

Le cas particulier du mode autonome, sans ARM (`./c54x_exe --trames N`) :
les 7 sections de ROM se chargent, `c54x_reset()` passe, et la
mask-ROM **exécute** — les `BRANCH-TRACE` à `PC=0xb41f` sont du vrai code TI.
**Aucun burst n'est injecté** : le DSP tourne sur une API RAM vierge, sans
ARM ni TPU. Le binaire le dit lui-même dans son bilan. `a_sch[3]` y sort
`0x771a` et non le `0xf8d8` du README de qemu-calypso — les deux valeurs sont
constantes, mais elles diffèrent parce que l'entrée diffère, ce qui est déjà
une information.

L'intérêt visé est la question du README : *« un décodeur dont la sortie ne
dépend pas de l'entrée ne décode pas »*. Y répondre demande d'injecter des
bursts et de faire varier l'entrée — le montage dsp avec la BTS le fait
depuis, et le rejeu hors banc (`tools/rejeu_banc`) le rend tenable, parce
qu'un essai coûte des millisecondes au lieu d'un boot complet.

## Dépendances

`libosmocoding` / `libosmocore` (pour `calypso_bsp.c`), `pthread`, `libm`.
Les ROM : `calypso_dsp.{PROM0..3,DROM,PDROM,Registers}.bin` dans `--rom-dir`
(défaut `/opt/GSM`).
