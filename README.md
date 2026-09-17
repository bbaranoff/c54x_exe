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
tant qu'aucun burst n'est injecte dans le DSP.

```bash
./run.sh              # tout, dans l'ordre        ./run.sh --status   qui tourne
./run.sh --logs       # suivre les 4 journaux      ./run.sh --stop     tout arreter, nettoyer
./run.sh --step 3     # une seule etape (les precedentes doivent tourner)
INSNS=8000 VERB=-vv ./run.sh            # budget DSP par trame, niveau de traces
MODE=grgsm ./run.sh                     # l'autre montage : couche 1 gr-gsm dans QEMU, sans c54x_exe
PONT=0 ./run.sh                         # sans l'etape 5 (pont.py) : le mobile seul, cellule synthetique
IQ=cell ./run.sh                        # c54x_exe fabrique une cellule GMSK (FCCH/SCH) a chaque trame
```

`run.sh` connait deux montages (`MODE=dsp`, le defaut, et `MODE=grgsm`) et une
cinquieme etape, le pont TRX (`pont.py`, `PONT=1` par defaut) qui relie le BTS
osmo-bts-trx (TRXD 5700-5702) a la couche 1 du mobile : GSMTAP 4730/4731 pour
gr-gsm, `--dsp-port 6702` vers le BSP de `c54x_exe` pour le DSP. Les memes
chaines se lancent par leur nom : `qosmo-dsp`, `qosmo-grgsm`, et exe par exe
`c54x_exe`, `osmocon`, `grgsm_exe` (`/usr/local/bin`). Le detail processus par
processus, avec les lignes de journal attendues, est dans `LAUNCH.md`.

Journaux et pid dans `/tmp/c54x-pont/`. Sockets et fichiers propres a ce
montage, pour ne pas croiser le banc : `/tmp/osmocom_l2_pont`, VTY mobile
4347, moniteur QEMU `/tmp/qemu-monitor-pont.sock`.

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

`--insns` : budget d'instructions par trame TDMA (32000 par defaut, la valeur
de `bsp.env` de qosmo-dsp). Une trame coute ~0,4 ms pour 1000 insn : au-dela
de ~10000, le DSP est plus lent que le temps reel et QEMU lui saute des ticks
(c'est prevu, cf. pieges ci-dessous).

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
pont : ARM connecte, API RAM 32768 mots, 32000 insn/trame
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
  -s /tmp/osmocom_l2_pont /opt/GSM/firmware/board/compal_e88/layer1.highram.bin
```

`/dev/pts/N` est le `serial0` de l'etape 2. Attendu : `Received ident ack`,
`Progress: 100%`, puis **`Received branch ack, your code is running now!`**.
A partir de la, osmocon relaie le L1CTL entre le firmware et
`/tmp/osmocom_l2_pont`. Si le telechargement reste a `starting download` sans
progres : un osmocon precedent a ete tue a mi-bloc et le stub romload de
l'UART attend la fin de ce bloc - relancer QEMU (etape 2) puis osmocon.

### 4. Le mobile

```bash
mobile -c /opt/GSM/c54x_exe/mobile_pont.cfg        # copie de ~/.osmocom/bb/mobile.cfg :
                                                   # layer2-socket /tmp/osmocom_l2_pont, vty 4347
```

Attendu dans les 5 s : `L1CTL_PM_REQ`, `L1CTL_RESET_REQ: FULL!`,
`L1CTL_FBSB_REQ (arfcn=514 ...)` cote osmocon ; cote `c54x_exe`, le passage de
428 a ~570 insn par trame et `a_sch=0100 ...` : la L1 ARM a programme
`d_task_md=5` (recherche FB) et le DSP l'execute. Cote mobile,
`FBSB RESP: result=255` en boucle : **aucun burst n'est injecte**,
`calypso_bsp.c` attend les bursts descendants en UDP sur le port 6702 et
personne ne les envoie. C'est l'etape suivante, et elle a maintenant une
entree ARM reelle.

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

Trois pieges rencontres, tous dans le pont et non dans le DSP :
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

Et une chose a savoir sur `qosmo` : `fw_console.c` lit `printf_buffer` a une
adresse codee en dur (`0x831018`) qui n'est pas celle de cet ELF (`0x8305f0`,
`nm layer1.highram.elf | grep printf_buffer`), donc pas de `[fw-console]`.

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
`/opt/GSM/qosmo` (`QOSMO=... make` pour pointer ailleurs). Recopier, c'était
refaire la divergence que `qosmo` vient de supprimer. La seule copie est
`src/pcb-minimal.c`, quatre helpers DARAM repris mot pour mot, et elle est
signalée comme telle dans le fichier.

## Ce que ça mesure, et ce que ça ne mesure pas encore

Aujourd'hui les 7 sections de ROM se chargent, `c54x_reset()` passe, et la
mask-ROM **exécute** — les `BRANCH-TRACE` à `PC=0xb41f` sont du vrai code TI.

Mais **aucun burst n'est injecté** : le DSP tourne sur une API RAM vierge, sans
ARM ni TPU. Le binaire le dit lui-même dans son bilan. `a_sch[3]` y sort
`0x771a` et non le `0xf8d8` du README de qemu-calypso — les deux valeurs sont
constantes, mais elles diffèrent parce que l'entrée diffère, ce qui est déjà
une information.

L'intérêt visé est la question du README : *« un décodeur dont la sortie ne
dépend pas de l'entrée ne décode pas »*. Y répondre demande d'injecter des
bursts et de faire varier l'entrée — c'est l'étape suivante, et elle devient
tenable parce qu'un essai coûte des millisecondes au lieu d'un boot complet.

## Dépendances

`libosmocoding` / `libosmocore` (pour `calypso_bsp.c`), `pthread`, `libm`.
Les ROM : `calypso_dsp.{PROM0..3,DROM,PDROM,Registers}.bin` dans `--rom-dir`
(défaut `/opt/GSM`).
