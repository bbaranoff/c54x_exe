# c54x_exe — the Calypso DSP, run as a native process

`c54x_exe` runs the **original TI mask ROM of the TMS320C54x DSP** found in
Calypso GSM basebands (the chips behind OsmocomBB's Motorola C1xx targets), on
a C54x core emulator, as a plain Linux process. It can run alone, replaying
recorded bursts in milliseconds, or serve as the DSP of a full phone: the
unmodified OsmocomBB layer 1 firmware runs under QEMU and talks to this process
through the real shared API RAM, frame by frame, exactly as the ARM talks to
the DSP on silicon. No radio, no SDR, no license anywhere in the chain.

State on 2026-10-03 (runs recorded in [`docs/README.fr.md`](docs/README.fr.md)
and [`MAILBOX.md`](MAILBOX.md)): cell selection, location update, SMS in both
directions, mobile-originated and mobile-terminated voice calls with A5/1
confirmed by the network, speech audible both ways. All downlink channel
decoding is done by the ROM. Since 2026-10-03 the **uplink SDCCH/SACCH bursts
sent to the BTS are the ROM's own coded bursts** (bit-exact with GSM 05.03 once
four C54x core bugs were fixed). Open: the ROM flags every downlink speech frame
as bad (BFI) although speech is intelligible, SACCH/8 downlink often fails its
FIRE check, and the SB window is armed rarely enough that sync takes a few tries.
The bench's coverage step prints, per signal-processing step, whether the run
did it in the ROM or on the host (`tests/modules/99-couverture.sh`).

## Architecture

```
 osmo-bts-trx ── TRXD (UDP 5700-5702) ──▶ pont_dsp.py ── UDP 6702 ──▶ c54x_exe
 + Osmocom core                            (osmo-operator)             │  BSP: bits → GMSK I/Q
                                                                       │  C54x core + TI mask ROM
                                                                       │  API RAM  (/dev/shm/calypso_api_ram)
                                                                       │  lockstep (/tmp/calypso_dsp.sock)
                                                                       ▼
                        osmocon ◀── serial ── qosmo (QEMU, Calypso machine, CALYPSO_DSP_EXTERN=1)
                           │                    └─ OsmocomBB layer1.highram, unmodified
                        mobile (OsmocomBB layer 2/3)
```

- **Downlink**: bursts leave the BTS as bits, the bridge forwards them, the BSP
  turns them into GMSK samples, and the ROM does what it does on a phone: FCCH/SCH
  detection, BCCH, TCH/F decoding, A5.
- **ARM ↔ DSP**: the ROM's API RAM window is memory-mapped and shared with QEMU.
  Both sides advance one TDMA frame at a time over a Unix socket; the per-frame
  sequence mirrors `calypso_tdma_tick()` of the in-QEMU DSP, so any divergence
  is localised (`src/pont.c`).
- **Uplink**: the ARM posts its blocks in the API RAM; `src/montant.c` scans it
  once per frame and publishes RACH, SDCCH, SACCH, FACCH and speech (TI format →
  FR) to `/dev/shm`. **SDCCH and SACCH-on-SDCCH are channel-coded by the ROM**:
  `src/pont.c` (`tx_rom_publier`) picks up each burst the ROM emits in
  `data[0x3f8a]` and publishes the 4 bursts of a block to
  `/dev/shm/calypso_xcch_ul_rom`; the bridge sends them (after checking they
  decode to the block, undoing the ROM's A5 XOR and re-ciphering at the air frame
  number). RACH, FACCH, SACCH on TCH, uplink speech and uplink A5 are still done
  by the bridge. `MONTANT_ROM_UL=0` (here) or `PONT_UL_ROM=0` (bridge) go back to
  host coding.
- Every deliberate departure from silicon behaviour is an environment variable,
  listed at startup (`hacks_actifs()` in `src/pont.c`).

## Building

The C54x core, the Calypso peripherals and the DSP glue live in
[qosmo](https://github.com/bbaranoff/qosmo) (`hw/arm/calypso/l1-dsp/`) and are
compiled from there — nothing is copied into this tree, on purpose.

```bash
# needs: qosmo checked out (default /opt/GSM/qosmo), libosmocore + libosmocoding, gcc
make                      # QOSMO=/path/to/qosmo make
./c54x_exe --trames 200   # the ROM alone, no ARM: does it boot, what does it write?
./c54x_exe --help
```

The ROM images (`rom/calypso_dsp.*.bin`) are dumps of the TI mask ROM at their
silicon addresses (`src/main.c`, `ROMS[]`).

## Running a phone

```bash
./run.sh              # DSP + QEMU + osmocon + mobile, in order; logs in /tmp/c54x-pont/
PONT=1 ./run.sh       # + the TRX bridge, against a running osmo-bts-trx / core network
./run.sh --status | --logs | --stop
```

Process-by-process launch, expected log lines and checks:
[`LAUNCH.md`](LAUNCH.md). The bridge and the network side are in
[osmo-operator](https://github.com/bbaranoff/osmo-operator).

## Benches and tools

| what | where |
|---|---|
| Replay the ROM alone on a recorded FB/SB acquisition, deterministic, no QEMU | `./c54x_exe --rejouer`, `src/rejouer.c` |
| Replay a recorded dedicated-channel session (TCH) through the ROM | `tools/rejeu_banc.c` |
| C54x instruction-level tests (the bugs they caught are listed in the source) | `tools/isa_test.c`, `tools/isa_tests.txt` |
| Bit-by-bit comparison: what the ROM decodes vs. libosmocoding on the same bursts | `tools/comparer_parole.py` |
| CCH interleaving reference vectors | `tools/cch_ref/` |
| Trace levels of the core, `-v` … `-vvvvvv` | `src/verbosite.c` |

## Repository layout

```
src/        main.c (ROM loading, CLI), pont.c (ARM/DSP bridge, per-frame sequence),
            montant.c (uplink from API RAM), cellule.c (synthetic FCCH/SCH/BCCH),
            rejouer.c (replay), verbosite.c (trace levels)
rom/        TI mask ROM dumps
tools/      benches, test vectors, analysis scripts
docs/       README.fr.md — the detailed, dated engineering notes (French)
LAUNCH.md   process-by-process launch guide
MAILBOX.md  dated log of hypotheses, refutations and measurements
```

## License

GPL-2.0-or-later, see `LICENSE`.

## ROM and uplink probes

The DSP ROM is **not** in this repository. `rom/fetch-rom.sh` downloads the FreeCalypso
dump (version 3606), converts it with `tools/dsp_txt2bin.py` into
`calypso_dsp.{DROM,PDROM,PROM0..3}.bin` and checks `rom/SHA256SUMS.3606`
(`--dest DIR` repeatable, `--version 3311` for the D-Sample dump). `calypso_dsp.Registers.bin`
is a register snapshot, not ROM: it stays in the repository.

Two inert-by-default probes in `src/pont.c` (environment variables, `--arm` mode):

- `PONT_TX_SONDE=N` — for N frames where the ARM has posted `d_task_u`/`d_task_ra`, log
  which DSP data words the ROM wrote and dump the changed ranges to `/tmp/tx-sonde/`.
- `PONT_TX_INJECT=FILE` — after `montant.c` has published a real uplink block (so the link
  is untouched), overwrite `a_cu` with the next 23-byte block of FILE (one hex line each);
  the ROM output for each block lands in `/tmp/tx-sonde/inj_<n>.txt`.
  `PONT_TX_TRACE_BLOC=N` single-steps the ROM on injected block N and logs every write to
  the TX buffers with its PC (`/tmp/c54x-pont/trace-tx.txt`).
- `PONT_TX_OMBRE=1` — compares each real SDCCH block coded by the ROM with
  `gsm0503_xcch_encode` (`/tmp/tx-sonde/ombre.txt`, `reel.txt`).

Uplink coding, as measured on 2026-10-03 (`tests/rom-tx/README.md`): the ROM writes the
interleaved block in `0x4280` (4 bursts × 8 words, circular, BK=32); FIRE parity is
computed at PROM0 `0x9013`; at each transmit frame `0x890e-0x891b` copies the current burst
to `data[0x3f8a..0x3f91]` (114 bits + hl/hu, MSB first), advances the pointer
`data[0x3d9b]` and clears word 7 of that burst in `0x4280`; in ciphered mode the A5
keystream (`0x3f9b..`, `calypso_a5.c`) is XORed onto `0x3f8a` itself. Four C54x core bugs
had to be fixed in qosmo `c54x_exec.c` for the output to match GSM 05.03 on all 456 bits:
logical (not arithmetic) 40-bit shift in `AND/OR/XOR src,SHIFT,dst`, SXM-respecting
`LD/ADD/SUB Smem,16` and `LDM`, and `F0Bx/F1Bx` decoded as `OR A,-n,dst` instead of
RSBX/SSBX (gates `CALYPSO_LOGIC40`, `CALYPSO_LD16_SEXT`, `CALYPSO_F0BX_SBIT` = old behaviour).

`C54X_BIN=/path/to/c54x_exe ./run.sh` runs another build of the executable.
