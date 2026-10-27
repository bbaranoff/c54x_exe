# ARM <-> DSP mailbox (Calypso API RAM)

Two views of the SAME memory: the ARM addresses it in bytes from 0xFFD00000,
the DSP in words from 0x0800. Conversion: `dsp_word = 0x0800 + arm_byte/2`.

Sources: `qosmo/include/hw/arm/calypso/calypso_api.h` and
`osmocom-bb/src/target/firmware/include/calypso/dsp_api.h` (they agree:
`#define DSP 36`, a_serv_demod/a_pm/a_sch branch).

## Overall map

    area          ARM (bytes)    DSP (words)  size
    W page 0      0xFFD00000     0x0800       20 words  ARM -> DSP
    W page 1      0xFFD00028     0x0814       20 words
    R page 0      0xFFD00050     0x0828       20 words  DSP -> ARM
    R page 1      0xFFD00078     0x083C       20 words
    NDB           0xFFD001A8     0x08D4      268 words  shared, not paged

## W page (ARM -> DSP), word offsets

    +0  d_task_d
    +1  d_burst_d
    +2  d_task_u
    +3  d_burst_u
    +4  d_task_md     0x0804 / 0x0818   task: 5=FB 6=SB 8/9=TCH
    +8  d_fn          0x0808 / 0x081C
    +15 d_afc         0x080F / 0x0823   AFC DAC, relayed to the TWL3025
    +16 d_ctrl_system 0x0810 / 0x0824

## R page (DSP -> ARM), word offsets

    +0   d_task_d
    +8   a_serv_demod[4]  0x0830 / 0x0844   TOA, PM, ANGLE, SNR
    +12  a_pm[3]          0x0834 / 0x0848
    +15  a_sch[5]         0x0837 / 0x084B   SB header + payload
         a_sch[0] status: bit15 B_BLUD (block present), bit8 B_SCH_CRC (1 = ERROR)
         a_sch[3..4]: SB word, sb = a_sch[3] | a_sch[4]<<16
                      BSIC = (sb>>2) & 0x3f

## NDB (shared)

    +0    d_dsp_page      0x08D4   0=reset, 2=armed page 0, 3=armed page 1
    +14   d_dsp_state     0x08E2
    +36   d_fb_det        0x08F8   non-zero = FCCH found
    +37   d_fb_mode       0x08F9   0=wide search, 1=narrow
    +38   a_sync_demod[4] 0x08FA   0x08FA TOA, 0x08FB PM, 0x08FC ANGLE, 0x08FD SNR

## Who writes what - ARM side (osmocom-bb source)

    dsp_end_scenario()   calypso/dsp.c
        d_dsp_page = B_GSM_TASK | w_page; then w_page ^= 1
    l1s_reset_hw()       layer1/sync.c
        d_dsp_page = 0; r_page = 0; db_r -> R page 0
        called from prim_fbsb.c on every failed FBSB cycle
    l1s_dsp_post()       layer1/sync.c
        memset of the R page, then a_sch[0] = (1<<B_SCH_CRC) = 0x0100,
        then r_page ^= 1
    -> the ARM only ever writes a_sch[0] with 0x0100.

## Who writes what - DSP side (mask ROM, exact PCs)

    0xb446   a_sch[0..4] <- 0 on BOTH pages             initialisation
    0xaba2   a_sch[0] <- 0x1111                         marker
             preceded by 0xaba0: LD #0x1111, A
             encoding 80e2 000f = store *(AR2 + 0x0f), AR2 = R page base
    0xb214   a_sch[0..4] <- status + payload            RESULT
    0xb2cc   d_fb_det <- 0        FB task entry: reset
    0xb2cf   a_sync[TOA] <- 0
    0xb2d2   a_sync[PM] <- 0
    0xb2d5   a_sync[ANGLE] <- 0
    0xb2d8   a_sync[SNR] <- 0
    0xb2c4   *(0x3fb5) <- 0x0cce   FB correlator input buffer pointer
    0xb2c9   *(0x3fb5) <- 0x0d2e   same, second buffer
    0x795a   a_sync[TOA] <- value
    0x798b   a_sync[ANGLE] <- value
    0x798d   a_sync[SNR] <- 0x4000
    0x79d4   a_sync[SNR] <- value
    0x79de   a_sync[PM] <- value

## SB task sequence

    ARM: d_task_md = 6 on the current W page
    ARM: d_dsp_page = 2|w_page       (dsp_end_scenario)
    DSP: 0xb446 clears a_sch
    DSP: 0xaba2 writes the 0x1111 marker
    DSP: 0xb214 writes the result into a_sch
    ARM: reads db_r->a_sch; B_SCH_CRC=1 means "SB not found"
    ARM: memset R page, a_sch[0] = 0x0100, r_page ^= 1
    ARM: after 2 attempts, L1CTL_RESET_REQ -> l1s_reset_hw -> d_dsp_page = 0

## Pitfalls when probing

- The emulator has two write paths, `data_write()` and `data_write_locked()`
  (`c54x_mem.c`); a probe on only one misses half the writes (a_sch goes
  through the second, a_sync_demod through the first).
- The ARM and the DSP are TWO PROCESSES sharing this mapping. A probe comparing
  memory before/after a DSP instruction can blame the DSP for an ARM write:
  record the address passed to the write function, never a delta.
- d_dsp_page announces the task's WRITE page. The READ page is
  `dsp_api.r_page`, a separate ARM-side counter; the two are not linked.
