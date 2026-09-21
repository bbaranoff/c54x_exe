# Lancer le côté mobile, processus par processus

Deux montages, cinq processus au plus, tous côté **mobile**. Le réseau (osmo-bts-trx,
BSC, MSC, HLR…) n'est jamais lancé ici ; le pont l'attend s'il existe.

| montage | nom à taper | ce qui tourne |
|---|---|---|
| **dsp** | `qosmo-dsp` | `c54x_exe --arm` → `qosmo` (QEMU, `CALYPSO_DSP_EXTERN=1`) → `osmocon` → `mobile` → `pont.py --dsp-port 6702` |
| **grgsm** | `qosmo-grgsm` | `qosmo` (QEMU, couche 1 gr-gsm intégrée) → `osmocon` → `mobile` → `pont.py` |

`qosmo-dsp` et `qosmo-grgsm` sans argument lancent tout dans l'ordre, chaque étape
attendant la précédente sur un critère observable. `--status`, `--logs`, `--stop`,
`--step N`. Journaux et pid dans `/tmp/c54x-pont/`. Les anciens lanceurs C du même nom
sont conservés en `qosmo-dsp-launch` / `qosmo-grgsm-launch` (ils visaient un rootfs ISO
disparu) et reçoivent les appels avec options QEMU (`-k`, `-dsp`, …).

Chaque processus se lance aussi seul, par son nom, dans cet ordre.

---

## 1. `c54x_exe` — le DSP (montage dsp seulement)

```bash
c54x_exe                      # = /opt/GSM/c54x_exe/c54x_exe --arm -v
c54x_exe --arm --iq cell      # + une cellule GMSK synthétique (FCCH/SCH/factice)
c54x_exe --trames 50          # mode autonome, sans ARM : la mask-ROM seule
```

- **Rôle** : la mask-ROM TI du TMS320C54x, exécutée hors QEMU (`/opt/GSM/qosmo/hw/arm/calypso/l1-dsp/`).
- **Publie** : `/dev/shm/calypso_api_ram` (la fenêtre API, alias de `data[0x0800..]` du C54x)
  et `/tmp/calypso_dsp.sock` (verrou trame par trame, protocole `calypso_dsp_pont.h`).
- **Écoute** : UDP **6702**, les bursts descendants pour le BSP (`calypso_bsp.c`).
- **Attendu** : `pont : en attente de l'ARM sur /tmp/calypso_dsp.sock`.
- **Vérifier** : `ls -la /dev/shm/calypso_api_ram /tmp/calypso_dsp.sock`.
- **Options** : `--insns N` budget par trame (32000), `--iq fcch|cell|tone:x|noise`,
  `--amp N`, `-v` à `-vvvvvv` pour les traces du cœur.
- **Lien montant** : publie `/dev/shm/calypso_rach`, `calypso_sdcch_ul`,
  `calypso_tch_facch_ul`, `calypso_tch_sacch_ul`, `calypso_tch_ul` en scrutant
  l'API RAM (`src/montant.c`) — c'est par là que le RACH du mobile atteint
  pont.py puis la BTS. `MONTANT=0` coupe, `MONTANT_DEBUG=N` règle les traces,
  `MONTANT_CONSOMME_RACH=1` rend le déclenchement exact.
  **Vérifier** : `./c54x_exe` imprime `[montant] RACH ra=0x.. bsic=..` et
  `pont.log` passe de `rach=0` à `rach=N`.

## 2. `qosmo` — l'ARM et la layer1 osmocom-bb

```bash
# montage dsp
CALYPSO_DSP_EXTERN=1 /opt/GSM/qosmo/build/qemu-system-arm -M calypso -cpu arm946 \
  -display none -parallel none -serial pty -serial pty \
  -monitor unix:/tmp/qemu-monitor-pont.sock,server,nowait \
  -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf
# montage grgsm : la même ligne sans CALYPSO_DSP_EXTERN
```

- **Rôle** : le Calypso (ARM946) qui exécute `layer1.highram.elf`. `-kernel` est obligatoire :
  sans lui le CPU part à 0 et plante en `0x840000`, le romload d'osmocon ne charge rien.
- **Avec `CALYPSO_DSP_EXTERN=1`** : la fenêtre API `0xFFD00000` est le segment partagé,
  la couche 1 gr-gsm est désactivée avant d'ouvrir ses ports, le registre `CNTL_RST`
  relaie `RESET_DSP` au DSP, un timer de boot cadence le DSP avant l'activation du TPU.
- **Sans** : la couche 1 gr-gsm (le shunt) écoute UDP **4730** (GSMTAP) et **4731** (SCH),
  nourris par le pont.
- **Attendu** (stderr) :
  ```
  char device redirected to /dev/pts/N (label serial0)       <- le pty modem
  [trx] pont DSP : API RAM partagee ... + socket /tmp/calypso_dsp.sock      (dsp)
  [trx] pont DSP : RESET_DSP relache par le firmware -> PONT_RESET          (dsp)
  [trx] pont DSP : le TDMA du firmware prend le relais du timer de boot     (dsp)
  [l1] backend gr-gsm : GSMTAP udp/4730, SCH udp/4731                       (grgsm)
  ```
  et côté `c54x_exe` : `RESET #1 ... pc=0xff80` puis `DSP boote (premier IDLE)`.
- **Vérifier** : `printf 'xp /96bx 0x008305f0\n' | nc -U /tmp/qemu-monitor-pont.sock`
  montre le dernier printf du firmware (`DSP API Version: 0x4e2a 0x491a`).
  `run.sh` écrit le pty dans `/tmp/c54x-pont/modem.pty`.

## 3. `osmocon` — le chargeur et le relais L1CTL

```bash
osmocon                       # = osmocon -m romload -i 100 -p $(cat /tmp/c54x-pont/modem.pty) \
                              #      -s /tmp/osmocom_l2_pont layer1.highram.bin
```

- **Rôle** : joue le protocole romload avec le stub UART de QEMU, puis relaie le L1CTL
  entre le firmware (sercomm sur le pty) et la socket `/tmp/osmocom_l2_pont`. Affiche la
  console du firmware (`FB0 (fn:att): TOA=… Power=… Angle=…`).
- **Attendu** : `Received ident ack`, `Progress: 100%`, **`Received branch ack, your code
  is running now!`**.
- **Piège** : un osmocon tué en plein téléchargement laisse le stub romload de l'UART à
  mi-bloc ; relancer QEMU (étape 2) puis osmocon.

## 4. `mobile` — les couches 2/3

```bash
mobile -c /opt/GSM/c54x_exe/mobile_pont.cfg     # layer2-socket /tmp/osmocom_l2_pont, VTY 4347
```

- **Attendu** dans les 5 s : côté osmocon `L1CTL_PM_REQ`, `L1CTL_RESET_REQ: FULL!`,
  `L1CTL_FBSB_REQ (arfcn=514 …)` ; côté DSP le passage de 428 à ~570 insn/trame et
  `a_sch=0100 …` (la L1 ARM a posé `d_task_md=5`, recherche FB).
- **Sans burst** (pas de pont, ou pas de BTS) : `FBSB RESP: result=255` en boucle, attendu.
- **Vérifier** : `telnet 127.0.0.1 4347` puis `show ms`. Le port 4347 est hors de la plage
  42xx des composants réseau ; `run.sh` refuse de lancer si le port est pris.

## 5. `grgsm_exe` — le pont TRX (gr-gsm)

```bash
grgsm_exe                     # = python3 pont/pont.py --no-record --dsp-port 6702
PONT_DSP_PORT=0 grgsm_exe     # montage grgsm : vers la L1 de QEMU seulement
```

- **Rôle** : reçoit les bursts du BTS en TRXD (UDP **5700-5702** depuis osmo-bts-trx),
  les décode avec gr-gsm et alimente la couche 1 du mobile ; renvoie l'uplink au BTS.
- **Vers la L1 gr-gsm** (montage grgsm) : blocs L2 en GSMTAP sur 4730, FN de synchro sur 4731.
- **Vers le DSP** (montage dsp, `--dsp-port 6702`, ajouté le 17/09) : chaque burst DL est
  ré-emballé au format du BSP, 8 octets `[tn, fn BE32, att, 0, 0]` + 148 bits 0/1 ; le BSP
  les convertit en I/Q et les dépose en DARAM `0x2a00`, d'où le DSP les lit.
- **Attendu** : `pont TRX : ports 5700/5701/5702, ARFCN 514, BSIC 7` puis des lignes
  `STATS fn=… | DL bursts=N`. `DL bursts=0` tant qu'aucun BTS n'émet : normal sans réseau.

---

## Ordre d'arrêt et nettoyage

`qosmo-dsp --stop` (ou `run.sh --stop`) arrête dans l'ordre inverse (pont, mobile, osmocon,
QEMU, DSP) et efface `/dev/shm/calypso_api_ram`, `/tmp/calypso_dsp.sock`,
`/tmp/osmocom_l2_pont`, `/tmp/qemu-monitor-pont.sock`.

## Ce que ça donne aujourd'hui (17/09)

- Montage dsp, cellule synthétique (`IQ=cell qosmo-dsp`) : le DSP **détecte la FCCH**
  (`d_fb_det=1`, `FB0`/`FB1` avec TOA/puissance/angle réels), grâce à deux bugs du décodeur
  corrigés ce jour (`CALYPSO_FIX_NORM_SD`, `CALYPSO_FIX_F7_DELAYED`). Le décodage du SCH
  reste ouvert (`a_sch[3]=0xf8d8` constant, `DSP Error Status: 8`), cf.
  `qosmo-dsp/hw/arm/calypso/doc/RAPPORT_DFBDET.md` §8.
- Montage grgsm : la chaîne complète du banc, qui campe et fait des appels dès que le
  réseau et un BTS sont là.
