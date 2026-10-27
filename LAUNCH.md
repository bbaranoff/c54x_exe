# Launching the mobile side, process by process

Two set-ups, at most five processes, all on the **mobile** side. The network
(osmo-bts-trx, BSC, MSC, HLR...) is never started here; the bridge waits for it
if it exists.

| Set-up | Command | What runs |
|---|---|---|
| **dsp** | `qosmo-dsp` | `c54x_exe --arm` -> `qosmo` (QEMU, `CALYPSO_DSP_EXTERN=1`) -> `osmocon` -> `mobile` -> `pont_dsp.py --dsp-port 6702` |
| **grgsm** | `qosmo-grgsm` | `qosmo` (QEMU, built-in gr-gsm layer 1) -> `osmocon` -> `mobile` -> `pont.py` |

Without arguments, `qosmo-dsp` and `qosmo-grgsm` start everything in order, each
step waiting for an observable condition of the previous one. Other options:
`--status`, `--logs`, `--stop`, `--step N`. Logs and pids go to `/tmp/c54x-pont/`.
Calls carrying QEMU options (`--qemu`, `-k`, `--bin`, `--cpu`, `--gdb`,
`--rundir`, `--monitor`, `-dsp`) are forwarded to the C launchers
`qosmo-dsp-launch` / `qosmo-grgsm-launch`.

Known limitation: the `/usr/local/bin/qosmo-dsp` and `qosmo-grgsm` wrappers are
not in any repository; a reinstall overwrites them.

Each process can also be started alone, in this order.

---

## 1. `c54x_exe` - the DSP (dsp set-up only)

```bash
c54x_exe                      # = /opt/GSM/c54x_exe/c54x_exe --arm -v
c54x_exe --arm --iq cell      # + a synthetic GMSK cell (FCCH/SCH/dummy)
c54x_exe --trames 50          # standalone, no ARM: the mask ROM alone
```

- **Role**: the TI TMS320C54x mask ROM, run outside QEMU (`/opt/GSM/qosmo/hw/arm/calypso/l1-dsp/`).
- **Publishes**: `/dev/shm/calypso_api_ram` (the API window, alias of the C54x
  `data[0x0800..]`) and `/tmp/calypso_dsp.sock` (frame lockstep, protocol
  `calypso_dsp_pont.h`).
- **Listens**: UDP **6702**, downlink bursts for the BSP (`calypso_bsp.c`).
- **Expected**: `pont : en attente de l'ARM sur /tmp/calypso_dsp.sock`.
- **Check**: `ls -la /dev/shm/calypso_api_ram /tmp/calypso_dsp.sock`.
- **Options**: `--insns N` budget per frame (200000 by default with `--arm`,
  80000 through `run.sh`), `--iq fcch|cell|tone:x|noise`, `--amp N`, `-v` to
  `-vvvvvv` for core traces (pure probes are off by default; `CALYPSO_SONDES=1`
  or `-vvvv` turn them on).
- **Environment**: `CALYPSO_BSP_ATTENTE_MS=40` (set by `run.sh`),
  `PONT_TCH_DEPOT_IDLE=0` (deposit the TCH burst before IDLE),
  `PONT_DONE_TOT=0` (return DONE at the end of the frame). Every 1000 frames
  `dsp.log` prints `[chrono] ... qemu | A | go | B | apres DONE`. Probes:
  `/dev/shm/calypso_bsp_dedie` (BSP dedicated store: played/missed/lost); on a
  dedicated channel, `/dev/shm/calypso_rejeu_tch.bin` is recorded for
  `tools/rejeu_banc` (`CALYPSO_REJEU_ENREG=0` turns it off).
- **Uplink**: publishes `/dev/shm/calypso_rach`, `calypso_sdcch_ul`,
  `calypso_tch_facch_ul`, `calypso_tch_sacch_ul`, `calypso_tch_ul` by scanning
  the API RAM (`src/montant.c`); this is how the mobile's RACH reaches pont.py
  and the BTS. `MONTANT=0` disables it, `MONTANT_DEBUG=N` sets the trace level,
  `MONTANT_CONSOMME_RACH=1` makes triggering exact, `MONTANT_KC=0` (Kc in
  `calypso_kc_l1`), `MONTANT_PAROLE_TI=0` (raw uplink speech, no TI -> FR
  conversion), `MONTANT_TCH_TACHE=0` (switch to TCH on the bridge's
  announcement instead of the firmware task), `MONTANT_AFD=0` / `MONTANT_ADD=0`
  (`[a_fd]` / `[a_dd]` probes).
  **Check**: `./c54x_exe` prints `[montant] RACH ra=0x.. bsic=..` and
  `pont.log` goes from `rach=0` to `rach=N`. During a call:
  `[montant] TCH : le firmware poste la tache 13 a fn=..., BSP bascule sur TS2`,
  then `le firmware est revenu sur le SDCCH (tache ALLC)` on release.

## 2. `qosmo` - the ARM and the osmocom-bb layer 1

```bash
# dsp set-up
CALYPSO_DSP_EXTERN=1 /opt/GSM/qosmo/build/qemu-system-arm -M calypso -cpu arm946 \
  -display none -parallel none -serial pty -serial pty \
  -monitor unix:/tmp/qemu-monitor-pont.sock,server,nowait \
  -kernel /opt/GSM/firmware/board/compal_e88/layer1.highram.elf
# grgsm set-up: the same line without CALYPSO_DSP_EXTERN
```

- **Role**: the Calypso (ARM946) running `layer1.highram.elf`. `-kernel` is
  required: without it the CPU starts at 0 and crashes at `0x840000`, and
  osmocon's romload loads nothing.
- **With `CALYPSO_DSP_EXTERN=1`**: the `0xFFD00000` API window is the shared
  segment, the gr-gsm L1 is disabled before it opens its ports, the `CNTL_RST`
  register relays `RESET_DSP` to the DSP, and a boot timer clocks the DSP until
  the TPU starts.
- **Without**: the gr-gsm L1 listens on UDP **4730** (GSMTAP) and **4731**
  (SCH), fed by the bridge.
- **Expected** (stderr):
  ```
  char device redirected to /dev/pts/N (label serial0)       <- modem pty
  [trx] pont DSP : API RAM partagee ... + socket /tmp/calypso_dsp.sock      (dsp)
  [trx] pont DSP : RESET_DSP relache par le firmware -> PONT_RESET          (dsp)
  [trx] pont DSP : le TDMA du firmware prend le relais du timer de boot     (dsp)
  [l1] backend gr-gsm : GSMTAP udp/4730, SCH udp/4731                       (grgsm)
  ```
  and on the `c54x_exe` side: `RESET #1 ... pc=0xff80` then `DSP boote (premier IDLE)`.
- **Check**: `printf 'xp /96bx 0x008305f0\n' | nc -U /tmp/qemu-monitor-pont.sock`
  shows the firmware's last printf (`DSP API Version: 0x4e2a 0x491a`).
  `run.sh` writes the pty to `/tmp/c54x-pont/modem.pty`.
- **Added by `run.sh`**: `-gdb tcp:127.0.0.1:1234` and the `telnet 0 44444`
  console (`tools/gdb-telnet.py` of qosmo; Ctrl-C stops the ARM, `continue &`
  resumes it; `GDB=0` disables it); `CALYPSO_PONT_RETRY_DIV=64` and, with
  `LOCKSTEP=1` (default), `CALYPSO_PONT_LOCKSTEP=1`; `ASSEMBLY_LOGS=1` writes
  `qemu-asm.log`. `qemu.log` goes to `/run/user/0/osmo-nitb/logs/qemu.log`,
  linked from `/tmp/c54x-pont/`.

## 3. `osmocon` - loader and L1CTL relay

```bash
osmocon                       # = osmocon -m romload -i 100 -p $(cat /tmp/c54x-pont/modem.pty) \
                              #      -s /tmp/osmocom_l2 layer1.highram.bin
```

- **Role**: speaks the romload protocol with QEMU's UART stub, then relays
  L1CTL between the firmware (sercomm on the pty) and the `/tmp/osmocom_l2`
  socket. Shows the firmware console (`FB0 (fn:att): TOA=... Power=... Angle=...`).
- **Expected**: `Received ident ack`, `Progress: 100%`, **`Received branch ack,
  your code is running now!`**.
- **Pitfall**: killing osmocon during the download leaves the UART romload stub
  mid-block; restart QEMU (step 2), then osmocon.

## 4. `mobile` - layers 2/3

```bash
mobile -c /opt/GSM/c54x_exe/mobile_pont.cfg     # layer2-socket /tmp/osmocom_l2, VTY 127.0.0.1:4247
```

- **Config**: `layer2-socket /tmp/osmocom_l2`, `sap-socket /tmp/osmocom_sap`;
  `tch-voice` gapk `io-tch-format ti`, ALSA `gsm_out`/`gsm_in`; `tch-data`
  unix-sock `/tmp/ms_data` (CSD 9600). `run.sh` sets
  `L23_SYNC_RETRIES_SELECTION=8` with `MODE=dsp` (default 1 elsewhere).
- **Expected** within 5 s: on osmocon `L1CTL_PM_REQ`, `L1CTL_RESET_REQ: FULL!`,
  `L1CTL_FBSB_REQ (arfcn=514 ...)`; on the DSP the step from 428 to ~570
  insn/frame and `a_sch=0100 ...` (the ARM L1 set `d_task_md=5`, FB search).
- **Without bursts** (no bridge or no BTS): `FBSB RESP: result=255` in a loop, expected.
- **Check**: `telnet 127.0.0.1 4247`, then `show ms`.

## 5. `grgsm_exe` - the TRX bridge (gr-gsm)

```bash
grgsm_exe                     # = python3 pont/pont_dsp.py --no-record --dsp-port 6702
PONT_DSP_PORT=0 grgsm_exe     # grgsm set-up: pont/pont.py, towards QEMU's L1 only
```

- **Role**: receives the BTS bursts as TRXD (UDP **5700-5702** from
  osmo-bts-trx), decodes them with gr-gsm and feeds the mobile's layer 1;
  sends the uplink back to the BTS.
- **Towards the gr-gsm L1** (grgsm set-up): L2 blocks as GSMTAP on 4730, sync FN on 4731.
- **Towards the DSP** (dsp set-up, `--dsp-port 6702`): each DL burst is
  repacked in the BSP format, 8 bytes `[tn, fn BE32, att, 0, 0]` + 148 bits
  0/1; the BSP turns them into I/Q in DARAM `0x2a00`, where the DSP reads them.
- **Through `run.sh`** (`PONT=1`, default 0): the bridge runs **without**
  `--no-record` (`PONT_AIRREC=1`, for the panel FFT).
- **Expected**: `pont TRX : ports 5700/5701/5702, ARFCN 514, BSIC 7`, then
  `STATS fn=... | DL bursts=N` lines. `DL bursts=0` while no BTS transmits.

---

## Stop order and cleanup

`qosmo-dsp --stop` (or `run.sh --stop`) stops in reverse order (bridge, mobile,
osmocon, gdb, QEMU, DSP) and removes `/dev/shm/calypso_api_ram`,
`/tmp/calypso_dsp.sock`, `/tmp/osmocom_l2`, `/tmp/qemu-monitor-pont.sock`,
`/tmp/osmocom_sap`, `/tmp/ms_data`, `/dev/shm/calypso_horloge`, the uplink
side-bands (`calypso_rach`, `calypso_sdcch_ul`, `calypso_tch_*_ul`) and, in the
dsp set-up, `calypso_tch_cfg`. The session is archived in
`/tmp/c54x-pont/archives/<date>/` (`JOURNAUX_GARDES=10`).

## Known issues

- B_BFI is set on every downlink speech frame although speech is intelligible.
- Occasional LOS during a TCH call (SACCH/TF FIRE check fails on every block).
- SDCCH/8: part of the downlink frames (mostly SACCH) are dropped.
- SB synchronisation succeeds only once every three to five attempts.
- Real-time margin in TCH: 4.3 to 4.6 ms of DSP work per 4.62 ms frame (`[chrono]`).
