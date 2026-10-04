# OsmocomBB's "magic 23", proven from the ROM, the layer1 code and hardware logs

The FIXME: `prim_fbsb.c:333` and `:469`, `/* FIXME: where did this magic 23 come from? */`,
followed by `last_fb->toa -= 23;`.

**Claim.** The DSP returns a position inside a 191-sample window hard-wired by TI; 23 is the only
margin that, in the `tpu_window.c` formula, produces that window; `toa − 23` turns the position into an
offset from an SB sitting at its nominal place, and the `+ 75` qbits of `synchronize_tdma` then move the
burst to the centre of the NB window, where the silicon does return the 16-qbit residual that `toa.c`
expects.

## Rules of evidence and sources

The evidence rests on three sources:

| source | what it is | where |
|---|---|---|
| **ROM** | TI Calypso DSP mask-ROM, version 3606, disassembled (binutils `tic54x`) | public dump: `ftp://ftp.freecalypso.org/pub/GSM/Calypso/dsp-rom-3606-dump.txt` (Pirelli DP-L10, Calypso PD751992AZHH, OsmocomBB dump tool). The PROM0 (0x7000-0xdfff, 28,672 words) of that dump is **identical word for word** to the PROM0 used here (0 differences out of 28,672). |
| **layer1** | OsmocomBB firmware sources, `src/target/firmware/layer1/{prim_fbsb.c,tpu_window.c,sync.c,toa.c}` and their git history | `osmocom-bb` repository |
| **hardware logs** | firmware output from real Calypso phones, posted to the baseband-devel list | [1] Aegean Chou, 2011-09-26, *Re: About sniff multi bursts in a frame, CCCH_CONF* (DSP API 0x3606, firmware without TX): https://lists.osmocom.org/pipermail/baseband-devel/2011-September/002526.html — [2] nish079858, 2024-10-02, *Osmocom bb mobile station cannot register to YateBTS basestation* (Motorola C115, `layer1.highram.bin` e88): https://www.mail-archive.com/baseband-devel@lists.osmocom.org/msg01046.html |

The emulation bench (`c54x_exe`, `dsp_tester`) is **not** a source of evidence: it served to find out where
to look (§7), and every time it is cited this is stated.

Levels: **PROVEN** (static reading of the ROM or of the code, or a hardware log landing on a numerical
prediction), **VERY LIKELY**, **HYPOTHESIS**.

### The result on one page

1. The DSP does not return a timing error. Its TOA is the **index of the lag it retained** in the
   correlation with the training sequence, the leading edge of its channel estimate (ROM, §2). A burst
   whose first sample is at position k of the window yields TOA = k + 3 (correlation starting at sample
   39, training sequence at bit 42), or less if an earlier lag of the 7-lag window reaches the
   threshold (§2.4).
2. The receive windows are **immediate constants in the ROM**: 191 samples for the SB, 151 for normal
   bursts (§1). `tpu_window.c` computes its own as `burst + 2·margin − 3` with `L1_SB_MARGIN_Q = 23×4`
   and `L1_NB_MARGIN_Q = 3×4`: **148 + 2·23 − 3 = 191** and **148 + 2·3 − 3 = 151**. Conversely
   (191 − 148 + 3)/2 = 23: 23 is the only margin compatible with TI's window (§4.1).
3. SB path: `TOA − 23`, then `+ 75` qbits, a net shift of `TOA_sb − 4.25` bits: the burst is placed at a
   TOA of **4.25**, the centre of the NB window (3 samples of play, centre 1.5, plus the ROM's offset of
   ≈ 3), which `toa.c` then holds at 16 qbits (§4.3). FB path: 23 subtracted **twice** since `cb71b972`
   (2010-05-20), net shift `TOA_fb − 27.25`, which makes the SB arrive at a TOA of 27.25 (§4.4).
4. The hardware logs land on these numbers with no free parameter (§5): printed qbits
   = ((TOA − 46) mod 1250)×4 (12 lines out of 12: 6 after an FB1, 6 after an SB); every first SB after
   an FB synchronisation, in both logs (n = 6): 29, 29, 27, 27, 24, 26 for a prediction of 27.25 (mean
   deviation −0.25, within the FB's granularity of 4); after SB synchronisation, every NB TOA deviation
   that `toa.c` prints is within 4 qbits of 16; FB TOA mode 1 ≡ 3 (mod 4) (10/10) and mode 0 ≡ 0
   (mod 48) (7/7), as the ROM's immediates say.
5. **23 is the margin of the SB window**, the firmware's and TI's. The FIXME persists because 23
   (`prim_fbsb.c`), 75 (`sync.c`) and 16 qbits (`toa.c`) only make sense together, and because the
   191-sample window is written nowhere in the firmware.

---

## 1. ROM: the receive windows are immediate constants — PROVEN

Common DMA programming routine (PROM0, binutils disassembly):

    a5cd  ld     #134,dp
    a5e3  ldm    ar2,a                ; A = API address of the page (0x0cce)
    a5e4  sub    #2048,a
    a5e6  stl    a,1,DP+0x56          ; (A − 0x800) << 1
    a5e8  portw  DP+0x56,pa64548      ; port 0xfc24: address (DMA2_AAD)
    a5ea  ldm    ar3,a                ; A = AR3 = length in samples
    a5ed  stl    a,2,DP+0x56          ; A << 2 = length in bytes (4 per I/Q sample)
    a5ef  portw  DP+0x56,pa64550      ; port 0xfc26: length (DMA2_ALGTH)
    a5f1  st     #1197,DP+0x56
    a5f3  xc     2,beq
    a5f4  st     #1193,DP+0x56
    a5f6  portw  DP+0x56,pa64552      ; port 0xfc28: control (DMA2_CTRL)

The three task routines load AR3 with an immediate and then call this code through the ROM's dispatcher
(`ld #41,a ; call 0xa9ea`: code 41 designates routine 0xa5cd; in emulation, AR3 does hold 191 on entry to
0xa5cd during an SB task):

| task | PC | instruction | window |
|---|---|---|---|
| SB | 0xb21d | `stm #191,ar3` | 191 samples (764 bytes) |
| NB | 0xb2b1 | `stm #151,ar3` | 151 samples (604 bytes) |
| FB | 0xb32d | `stm #48,ar3` | 48 samples per page (192 bytes), continuous stream |

No word written by the ARM is involved: the window lengths are fixed by TI in the ROM. (The identification
of ports 0xfc24/26/28 with the AAD/ALGTH/CTRL registers of the RHEA controller comes from the Calypso
register map; the next point depends only on the immediates 191 and 151.)

## 2. ROM: the SB TOA is the index of the retained lag (leading edge of the channel estimate, §2.4) — PROVEN

Complete chain, static reading of PROM0.

**2.1 The DMA page is copied without offset.** `0x7c3d-0x7c47`: `stm #3278,ar2` (API page 0x0cce),
`stm #10752,ar3` (0x2a00), `stm #10944,ar4` (0x2ac0), `stm #3660,ar5` (0x0e4c: I/Q DC component computed
at `0x81a5-0x81c7` over the 191 samples), `calld 0x81c8` with `stm #47,brc` in the delay slot
(48 iterations of 4 samples = 192 ≥ 191). The loop `0x81cc-0x81db` reads the page in order (`*ar2+`),
subtracts the DC component and stores sample n at **index n** of both buffers, applying the MSK derotation
(−j)^(n+1) through the signs and the two destinations: buffer 0x2a00 ← Q0, −I1, −Q2, I3, …; buffer 0x2ac0
← −I0, −Q1, I2, Q3, …

**2.2 The correlator looks for the extended training sequence.** `0x7c49-0x7c58`: 64 words read from
program memory starting at `0x7a1c` (`ld #31260,a ; rpt #25 ; reada *ar2+` ×2, `rpt #11 ; reada`) into
0x2cea. These 64 words are ±512 and their sign is, bit for bit, the **SB extended training sequence**
(GSM 05.02 §5.2.5, 64 bits), with 1 → −512 and 0 → +512:

    ETS            1011100101100010000001000000111100101101010001010111011000011011
    sign(coeff)    0100011010011101111110111111000011010010101110101000100111100100   (= bitwise complement)

**2.3 The correlation starts at sample 39.** `0x84a1-0x84c6`: `stm #10791,ar3` (0x2a27 = 0x2a00 +
**39**), `stm #10983,ar4` (0x2ae7 = 0x2ac0 + 39), `stm #49,brc` (50 lags), `stm #-63,ar0`; for each lag i,
64 MACs of coefficients × buffer[39 + i + j] on each component (`sth a,*ar1+` to 0x2c56 + i, `sth a,*ar5+`
to 0x2c88 + i), then `add *ar3+,*ar4+,a` which advances by one sample. `0x84ca-0x84d8`: energy I² + Q²
of lag i at 0x2be4 + 2i (`dst a,*ar4+`).

**2.4 Choice of lag: leading edge of the channel estimate.** Two blocks, static reading.

*7-lag window.* `0x84dd-0x84ef`: A = 0, B = 0, AR0 = AR1 = 0; for i = 0..42: `dadd *ar3+,a`
(A += E[i+7]), `dsub *ar2+,a` (A −= E[i]), `max b`, `mar *ar0+`, `xc 1,nc ; mvmm ar0,ar1`. After
iteration i, A = W[i+1] − W[0] where W[s] = E[s] + … + E[s+6]; AR1 receives i+1 whenever A exceeds the
current maximum. Hence **s = argmax W[s]** (s = 0 by default if W[0] dominates), stored at 0x2f06.

*Threshold.* `0x7c5f-0x7c80`: energy of the 64 samples starting at 0x2a2a + s (`squra` ×64, `sfta a,-6`:
average per sample), then × 1638/65536 (`stm #1638,t ; mpyu`) → DP+0x00 (0x2f00): **2.5 % of the mean
received energy per sample**.

*First significant lag.* `0x7c8f-0x7c95`: the 7 + 7 channel coefficients (0x2cce..) are zeroed. `0x7ca6`:
`rsbx tc`. `0x7ca7-0x7cae`: AR6 = AR7 = 0x2be4 + 2s. `0x7ca9-0x7cbb`, 7 iterations:

    7caf  dld    DP+0x00,a          ; A = threshold
    7cb0  dsub   *ar6,a             ; A = threshold − E[lag]
    7cb1  bc     0x7cb7,agt         ; E < threshold: lag ignored
    7cb3  ssbx   st0,tc             ; E ≥ threshold: TC = 1 (and what follows)
    7cb4  dadd   *ar6,b
    7cb5  mvdd   *ar2,*ar4          ; channel coefficient I ← correlation I of this lag
    7cb6  mvdd   *ar3,*ar5          ;                     Q
    7cb7  mvdd   *ar4+,*ar2+        ; copy of the coefficient (zero if ignored) into the correlation buffer
    7cb8  mvdd   *ar5+,*ar3+
    7cb9  dadd   *ar6+,a            ; A = threshold; next lag
    7cba  xc     1,ntc              ; as long as no lag has reached the threshold:
    7cbb  mvmm   ar6,ar7            ;     AR7 = pointer to the next lag

TC is never cleared inside the loop: AR7 stops on the **first lag of the window whose energy reaches the
threshold**, and the lags before it are erased from the channel estimate. The TOA is therefore the leading
edge of the estimated channel: equal to the peak k + 3 for a clean pulse, **k + 3 or less** otherwise
(k + 2 as soon as the energy at k + 2 exceeds 2.5 % of the mean energy, widened pulse; k + 1 with a
strong precursor, the rule taking the first lag ≥ threshold from s onwards). Then:

    7cbc  ldm    ar7,a
    7cbd  sub    #11236,a            ; 11236 = 0x2be4
    7cbf  stl    a,-1,DP+0x1f        ; TOA = (AR7 − 0x2be4) / 2  →  0x2f1f
    7e8a  mvdk   DP+0x1f,0x3fa4
    b1e7  ld     *(0x3fa4),a
    b1e9  stl    a,*ar0(0x8)         ; word 8 of the DB page = a_serv_demod[0] = D_TOA (dsp_api.h, DSP 36)

No constant is added: the TOA is the index of the retained lag. (Checked in emulation on three energy
profiles read from table 0x2be4: the rule yields 23, 24 and 23 where the ROM published 23, 24 and 23, §7.)

**2.5 Geometry.** In the SB, the extended training sequence occupies bits 42 to 105 (3 tail bits, 39 data,
64 training, 39, 3). If the first sample of the burst is at index k of the DMA window, hence of the buffer
(§2.1), the sequence starts at index k + 42, and the lag i that aligns it with the coefficients satisfies
39 + i = k + 42:

    TOA_SB = k + 3   (peak), or less if an earlier lag reaches the threshold (leading edge, §2.4)

**2.6 A false 23 not to be mistaken for the real one.** If the energy of the retained channel coefficients is
zero (`0x7ccd`, `bcd 0x7ced,aneq` not taken), the fallback `0x7ce5 mvmd ar0,0x2f1f` publishes the coarse
index AR0 instead. In that case there is no channel estimate and the SB does not decode (bad CRC). Every TOA
cited here comes with a correct CRC decode, hence from the main path (audit in §7).

## 3. ROM: the FB TOA — PROVEN for the form, partial for the detail

`0x7931-0x795a` (mode 1):

    7931  rptb   0x793d               ; scan of the 32-bit energy buffer (*ar2, dld), tracking the maximum
    793e  ld     *ar4+,b              ; B = c, index of the maximum
    793f  sfta   b,2                  ; B = 4·c
    7940  ld     *(0x3fb4),16,a       ; A = N (page counter, 0x3fb4)
    7942  sub    #3,16,a
    7944  add    *ar4,16,a            ; + p (word following the index)
    7945  sub    #2,16,a
    7947  mpya   a                    ; × T = 48 (one page)
    7948  add    a,b
    7949  sub    #5,b                 ; TOA = 48·(N + p − 5) + 4·c − 5
    794b  sub    *(0xc3d),b,a ; rc aleq        ; lower and upper bounds …
    795a  stl    b,*(0x8fa)           ; NDB a_sync_demod[D_TOA]

Immediates: 48 (DMA page), −5, step of 4 on c. In mode 1, **TOA ≡ 3 (mod 4)**, granularity 4 samples;
23 is reachable (48·m + 23: the logs give 10007 = 48·208 + 23). In mode 0 the logs give exact multiples of
48 (§5.2): the position is only known to the page, and 23 is not reachable there. The origin of N, p and
the −5 has not been worked out.

## 4. layer1: what the firmware does with this TOA — PROVEN (reading of the code and of git)

### 4.1 The firmware's margins are the ROM's

`tpu_window.c:38-44`:

    #define L1_NB_MARGIN_Q      (3 * 4)
    #define L1_SB_MARGIN_Q      (23 * 4)
    #define L1_TAIL_DURATION_Q  (3 * 4)
    #define L1_NB_DURATION_Q    (L1_BURST_LENGTH_Q + 2 * L1_NB_MARGIN_Q - L1_TAIL_DURATION_Q)
    #define L1_SB_DURATION_Q    (L1_BURST_LENGTH_Q + 2 * L1_SB_MARGIN_Q - L1_TAIL_DURATION_Q)

Same formula, with a 148-bit burst, as the immediates of §1:

    SB: 148 + 2·23 − 3 = 191        NB: 148 + 2·3 − 3 = 151        and  (191 − 148 + 3) / 2 = 23

Two windows, two margins, one formula: the 151 constrains the formula as much as the 191 does; this is not
a decomposition chosen after the fact. With `L1_BURST_LENGTH_Q = 625` qbits (one slot of 156.25 bits), the
firmware opens its TPU windows 8.25 bits longer than the DSP's DMA, on both sides: the guard period.

### 4.2 The windows open at the same instant

On the ARM side, `l1s_rx_win_ctrl` (`tpu_window.c:100-110`): `start = DSP_SETUP_TIME` for every window
type, only the duration changes. On the DSP side (§1), the SB and NB tasks arm the DMA through the same
routine 0xa5cd, with the same page address (`stm #3278,ar2` at 0xb21b and 0xb2af), the same control word
0x04ad (ONE_SHOT; the FB gets 0x04a9, without ONE_SHOT), only the length differs: the ROM programs no
task-specific offset. The start is therefore the same for both windows (the hardware trigger of the DMA
has not been traced; §5.4 confirms it on silicon), and a burst has the same position k in the SB window
(191) and in the NB window (151); only the room left around it changes.

### 4.3 SB path: TOA − 23, then + 75 qbits — target 4.25

`prim_fbsb.c:207-231` (`l1s_sbdet_resp`):

    last_fb->toa -= 23;
    qbits = last_fb->toa * 4;
    …
    cinfo->time_alignment = qbits;
    synchronize_tdma(&l1s.serving_cell);

`sync.c:113-121` (`synchronize_tdma`):

    uint32_t tpu_shift = cinfo->time_alignment;
    /* NB detection only works if the TOA of the SB
     * is within 0...8. We have to add 75 to get an SB TOA of 4. */
    tpu_shift += 75;

Time-base shift: (TOA_sb − 23)×4 + 75 qbits = **TOA_sb − 4.25 bits**. After this shift the burst sits at
a TOA of 4.25 in any window opened at the same instant (§4.2), hence in the NB window. And `toa.c` (2011,
`937023be`) then holds the average TOA of normal bursts at **16 qbits = 4 bits**, accepting only
0 ≤ TOA ≤ 31 qbits. In the 151-sample NB window a 148-sample burst is centred at k = 1.5, i.e. TOA = 4.5
(peak, §2.5) or 3.5 (leading edge): 4.25 is the centre of the NB window to within 0.25, which is also what
the comment says ("within 0...8", "an SB TOA of 4"). And 75 qbits = 23 − 4.25 bits: the +75 is the
difference between the firmware's nominal SB position and the centre of the NB window.

### 4.4 FB path: 23 is subtracted twice — net shift TOA_fb − 27.25

`prim_fbsb.c:465-493` (`l1s_fbdet_resp`, mode 1):

    last_fb->toa -= 23;                      /* 1st subtraction, for ntdma and delay */
    …
    fbinfo2cellinfo(&l1s.serving_cell, last_fb);   /* 2nd subtraction, prim_fbsb.c:333 */
    synchronize_tdma(&l1s.serving_cell);           /* + 75 */
    tdma_schedule_set(delay, sb_sched_set, 0);

`time_alignment` is ((TOA_fb − 46) mod 1250)×4, and the net shift is **TOA_fb − 46 + 18.75 = TOA_fb −
27.25 bits**. The SCH follows the FCCH by exactly one frame, at the same position within the frame: in the
SB window opened on the new time base it sits at 27.25 bits, plus any difference between the ROM's FB and
SB measurement conventions, written Δ:

    TOA_sb (first SB after FB synchronisation) = 27.25 + Δ

Git: the double subtraction dates from `cb71b972` (Welte, 2010-05-20, "Make new L1CTL_FBSB_REQ work
reliably"), which introduces `fbinfo2cellinfo()` without removing the `toa -= 23` of the mode-1 block. In
the initial import (`fbe7b94c`, 2010-02-18) the FB path subtracted 23 only once. `L1_SB_MARGIN_Q`,
`L1_NB_MARGIN_Q`, both `toa -= 23` and `tpu_shift += 75` with its comment are all in the initial import.

## 5. Hardware logs: the predictions land — PROVEN

Relevant lines, verbatim, from log [1] (2011, four complete synchronisations) and log [2] (2024, one
synchronisation):

```
[1] FB0 (11:6): TOA= 7200          FB1 (21:8): TOA= 9651         =>FB @ FNR 20 fn_offset=20 qbits=3420
    SB1 (46:1): TOA=   29          => SB 0x00c12184: BSIC=33 fn=89118(67/16/21) qbits=24
    =>FB @ FNR 45 fn_offset=89118 qbits=4932
    TOA AVG is not 16 qbits, correcting (got 20)
    FB0 (91861:3): TOA= 2544       FB1 (91871:8): TOA= 8751      =>FB @ FNR 91869 fn_offset=91869 qbits=4820
    SB1 (183745:1): TOA=   29      => SB 0x01e12284: BSIC=33 fn=91882(69/24/31) qbits=24
    =>FB @ FNR 183744 fn_offset=91882 qbits=4932
    FB0 (92514:8): TOA= 8784       FB1 (92524:8): TOA= 8755      =>FB @ FNR 92522 fn_offset=92522 qbits=4836
    SB1 (185051:1): TOA=   27      => SB 0x00852284: BSIC=33 fn=92535(69/ 1/21) qbits=16
    =>FB @ FNR 185050 fn_offset=92535 qbits=4924
    TOA AVG is not 16 qbits, correcting (got 19)
    FB0 (93574:9): TOA=10032       FB1 (93585:9): TOA=10003      =>FB @ FNR 93583 fn_offset=93583 qbits=4828
    SB1 (187172:1): TOA=   27      => SB 0x01582384: BSIC=33 fn=93596(70/22/11) qbits=16
    =>FB @ FNR 187171 fn_offset=93596 qbits=4924
    FB0 (94186:1): TOA=   48       FB1 (94197:9): TOA=10007
    FB1 (94217:11): TOA=12507      FB1 (94237:11): TOA=12507     FB1 (94248:2): TOA= 1259
[2] FB0 (192454:2): TOA= 1296      FB0 (192464:8): TOA= 8784     FB1 (192475:9): TOA=10007
    =>FB @ FNR 192473 fn_offset=192473 qbits=4844
    SB1 (384952:1): TOA=   24      => SB 0x019c4808: BSIC=2 fn=192485(145/ 7/11) qbits=4
    =>FB @ FNR 384951 fn_offset=192485 qbits=4912
    TOA AVG is not 16 qbits, correcting (got 17)
    FB0 (194158:8): TOA= 8784      FB0 (194168:8): TOA= 8784     FB1 (194178:8): TOA= 8755
    =>FB @ FNR 194176 fn_offset=194176 qbits=4836
    SB1 (388359:1): TOA=   26      => SB 0x01514908: BSIC=2 fn=194188(146/20/31) qbits=12
    =>FB @ FNR 388358 fn_offset=194188 qbits=4920
    (the "got 13" quoted earlier in this log precedes these lines)
```

**5.1 The double subtraction (§4.4) is in the binary that runs.** The qbits printed by
`fbinfo2cellinfo` after the FB1 are ((TOA − 46) mod 1250)×4, never ((TOA − 23) mod 1250)×4:

| FB1 TOA | printed qbits | ((TOA − 46) mod 1250)×4 | ((TOA − 23) mod 1250)×4 |
|---|---|---|---|
| 9651 | 3420 | **3420** | 3512 |
| 8751 | 4820 | **4820** | 4912 |
| 8755 | 4836 | **4836** | 4928 |
| 10003 | 4828 | **4828** | 4920 |
| 10007 [2] | 4844 | **4844** | 4936 |
| 8755 [2] | 4836 | **4836** | 4928 |

And the `=>FB @ …` line printed after each SB comes from the task-completion handler `l1a_fb_compl`
(`prim_fbsb.c:523-535`, registered at `:572`, triggered by `l1s_compl_sched(L1_COMPL_FB)` at `:266`), which
calls `fbinfo2cellinfo` again (`:532`) on `last_fb`, where the SB result has already been reduced by 23 by
`l1s_sbdet_resp` (`:207`): it reads ((TOA_sb − 46) mod 1250)×4, i.e. 29 → 4932 (×2), 27 → 4924 (×2),
24 → 4912, 26 → 4920. In total **12 lines out of 12** (6 after an FB1, 6 after an SB) follow the rule
"−23 on every pass through `fbinfo2cellinfo`".

**5.2 The ROM's immediates (§3).** FB mode 1: 9651, 8751, 8755, 10003, 10007, 12507, 12507, 1259, 10007,
8755 are **all ≡ 3 (mod 4)** (10/10; 10007 = 48·208 + 23). FB mode 0: 7200, 2544, 8784, 10032, 48, 1296,
8784 are **all multiples of 48** (7/7: pages 150, 53, 183, 209, 1, 27, 183).

**5.3 The hand-over from FB to SB (§4.4).** Criterion: *every* first SB following an FB synchronisation,
in both logs, with no exclusion (n = 6; synchronisations 2 and later start from an already aligned time
base, which does not change the prediction, which depends only on TOA_fb):

| log | FB1 TOA | shift applied (bits, mod 1250) | predicted SB position | measured SB TOA | Δ |
|---|---|---|---|---|---|
| [1] | 9651 | 873.75 | 27.25 | 29 | +1.75 |
| [1] | 8751 | −26.25 | 27.25 | 29 | +1.75 |
| [1] | 8755 | −22.25 | 27.25 | 27 | −0.25 |
| [1] | 10003 | −24.25 | 27.25 | 27 | −0.25 |
| [2] | 10007 | −20.25 | 27.25 | 24 | −3.25 |
| [2] | 8755 | −22.25 | 27.25 | 26 | −1.25 |

Mean Δ: **−0.25 bit**; spread ± 2.5, which is what the FB's granularity of 4 (§3) and the SB's integer
rounding impose. The ROM's FB and SB measurement conventions coincide on silicon (Δ = 0 ± 2), and the SB
arrives where the code puts it: 27 ± 2, **never 23**.

**5.4 The 4.25 target (§4.3).** After SB synchronisation, `toa.c` prints the average NB TOA over 250 frames
**only when it differs from 16**: the lines "got 20", "got 19" ([1]), "got 13", "got 17" ([2]) are
therefore the deviations by construction, and the periods at 16 are invisible. What can be said: every
printed deviation is within 4 qbits (1 bit) of 16, for a prediction of 17 qbits = 4.25 bits (+ Δ', the
NB/SB difference); none is at 23 or 27. And the FB1s of the subsequent synchronisations, taken from an
already aligned time base, have TOA mod 1250 = 1, 5, 3, 7, 7, 9 (mean 5.3): the FCCH shows up at ≈ 4 ± 4
where the NB loop holds the bursts at 4. The three DSP tasks measure the TOA in the same convention, up to
granularity.

## 6. What the 23 is — summary

| statement | level | support |
|---|---|---|
| The DSP's TOA is a position within the window (index of the retained lag, leading edge of the channel estimate, §2.4), not an offset from nominal time | PROVEN | §2 |
| The SB (191) and NB (151) windows are hard-wired in the ROM and equal 148 + 2·23 − 3 and 148 + 2·3 − 3 | PROVEN | §1, §4.1 |
| The 23 of `prim_fbsb.c` is `L1_SB_MARGIN_Q`: same originating commit, same value, and it is the only margin that reproduces TI's window | PROVEN for the equality and the common origin; the author's intent is not documented | §4.1, §4.4 |
| "TOA = 23" is, in the firmware's model, an SB at its nominal position, 23 bits after the window opens; in the ROM an SB centred in its 191 samples yields TOA ≈ 24.5 (21.5 + 3) | PROVEN (numbers); the 1-to-2-bit gap is absorbed by the +75 and by `toa.c` | §2.5, §4.3 |
| The +75 qbits (= 23 − 4.25 bits) move the burst from the nominal SB position to the centre of the NB window, 40 samples shorter; `toa.c` then holds 16 qbits | PROVEN (code) and verified (logs: 13 to 20 qbits) | §4.3, §5.4 |
| FB path: −23 twice, +75; the SB arrives at 27.25 | PROVEN (code, git) and verified (logs: 24 to 29, mean 27.2) | §4.4, §5.1, §5.3 |
| The second subtraction of the FB path was not removed when `fbinfo2cellinfo` was introduced (`cb71b972`); it places the SB near the centre of its window (27 instead of 4); intended or not: a question for the author | FACT (git); intent HYPOTHESIS | §4.4 |
| The number 23 was chosen from the DSP's 191-sample window (TI documentation or experiment) rather than the other way round | VERY LIKELY (the ROM is earlier and frozen; the NB confirms the formula); not documented | §1, §4.1 |

Phrased for the list: *the DSP returns the position of the burst within its window (the leading edge of its
channel estimate), not an offset; the DSP's SB window is 148 + 2·23 − 3 = 191 samples (a ROM immediate),
and 23 is the margin with which `tpu_window.c` opens its own; `toa − 23` turns the position into an offset
from an SB at its nominal place, and the `+ 75` of `synchronize_tdma` (= 23 − 4.25 bits) then shifts to
the centre of the NB window, which `toa.c` holds at 16 qbits; on silicon, the SB arrives at 27 ± 2 after FB
synchronisation (n = 6) and the NB TOA deviations that `toa.c` reports stay within 4 qbits of 16, as the
code predicts.*

## 7. What the bench showed (orientation, not evidence)

- SB TOA = k + 2 (widened waveform) or k + 3 (clean waveform, at sampling instant d = 0), slope 1 with no
  spread over k = 0 to 40; NB TOA = k + 2 over k = −2 to 6. The "36/40 at k + 3" of an earlier version was
  measured at d = 0.5, a rounding switch point (TOA = round(k + 3 − d)); at d = 0 it is 40/40. Energy
  profiles read from the ROM: peak at k + 3 in all three waveforms, with 4 sliding 7-lag sums within 1 % of
  one another. With the rule of §2.4 (first lag of the window above threshold): bench waveform,
  E[k+2]/E[k+3] = 41 % → 23; bridge without widening, 1.4 % → 24; widened bridge, 45 % → 23; the three
  values the ROM published. This is what led to the static readings of §2. Whether the spread of the
  silicon SBs (24 to 29) comes from the same mechanism (a precursor path, or the receive filter putting
  energy at lag k + 2) remains a hypothesis.
- SB window: SCH decoded for k from −3 to 46 (centre 21.4); NB window: valid TOAs [0; 8], centre 4: this
  is what led to §4.3.
- **Fallback audit (§2.6)**: on a complete SB test (20 SCHs targeted, 40 attempts), the `A == 0` test at
  0x7ccd was taken 20 times out of 40, always on the 2nd attempt, the one following the SCH frame (firmware
  and bench post two SB tasks on two consecutive frames, "as it is done by the TSM30"; the 2nd receives the
  next burst, which is not an SCH, and its channel coefficients are zero). All 20 retained TOAs (correct
  CRC) come from the main path. The 21/09 note in `MAILBOX.md` (TOA 23, 8/8 SBs decoded) meets the same
  criterion. The 23 published by the fallback is 21 + 2, the bench's default margin plus the ROM's offset:
  a coincidence of the bench, unrelated to the firmware's 23.
- **Withdrawn**: the "FCCH in the 1st search frame" test (TOA ≈ k − 7 to k − 11 in steps of 4,
  non-monotonic detection from k ≈ 34). Unexplained, hence outside the evidence; the logs (§5.2, §5.3) are
  sufficient for the FB.
- The `BANC_*` variables added to `tools/dsp_banc_commun.c` and the `fbpos` test remain in the repository
  for anyone who wants to redo these measurements.

## 8. What remains open, and whose it is

1. **The author's intent.** Can only be closed by Harald Welte or Dieter Spaar. Three questions for the
   list: was 23 chosen from the DSP's 191-sample window (TI documentation, experiment, TSM30); is the
   second subtraction of 23 in the FB path (`cb71b972`) intended; did the comment "an SB TOA of 4" refer
   to the residual after −23.
2. **Δ and Δ' to a tenth of a bit.** No bearing on the claim (Δ = 0 ± 2 is enough); out of scope without
   hardware. Verifiable by anyone with a C1xx: first SB after FB synchronisation at **27 ± 2** (firmware
   later than 2010-05-20), **4 ± 2** on earlier firmware. The list archives ("SB1 (", "TOA AVG is not")
   may still enlarge n.
3. **The ROM's choice between lags**: closed by static reading (§2.4, leading edge above 2.5 % of the mean
   energy). The behaviour on a real RF chain (how much energy at lag k + 2) is out of scope.
4. **N, p and the −5 of the FB formula** (§3): not worked out, no bearing on the conclusion.
5. **FreeCalypso / TI**: the TI sources (TCS211) may contain the SB window constant and its comment; not
   consulted here, status to be assessed.

## Reproducing

```
#!/bin/bash
# Reproduces the "magic 23" evidence and writes everything to toa.log.
# Idempotent: skips downloads and builds already present.
# Needs: curl, python3, gcc, make; osmocom-bb is cloned if absent.
set -u
LOG=toa.log
: > "$LOG"
say() { printf '%s\n' "$*" | tee -a "$LOG"; }
run() { say "\$ $*"; "$@" 2>&1 | tee -a "$LOG"; say ""; }

say "# magic 23 reproduction — $(date -u +%Y-%m-%dT%H:%MZ) — $(uname -srm)"
say ""

# ---------------------------------------------------------------- 1. ROM
say "## 1. ROM dump -> PROM0.bin"
[ -f dsp-rom-3606-dump.txt ] || run curl -sO ftp://ftp.freecalypso.org/pub/GSM/Calypso/dsp-rom-3606-dump.txt
run sha256sum dsp-rom-3606-dump.txt
python3 - <<'E' 2>&1 | tee -a "$LOG"
import re,struct
sec=None;w={}
for l in open("dsp-rom-3606-dump.txt",errors="replace"):
    m=re.match(r"DSP dump: (\w+) \[",l)
    if m: sec=m.group(1); continue
    m=re.match(r"([0-9a-f]{5}) : ((?:[0-9a-f]{4} ?)+)",l)
    if m and sec=="PROM0":
        a=int(m.group(1),16)
        for i,x in enumerate(m.group(2).split()): w[a+i]=int(x,16)
open("PROM0.bin","wb").write(b"".join(struct.pack("<H",w[a]) for a in range(0x7000,0xe000)))
print("PROM0.bin: %d words (0x7000-0xdfff)" % len(range(0x7000,0xe000)))
for a in (0x7cbd,0x84a4,0xb21d,0xb2b1,0xb32d):
    print("  %04x: %04x %04x" % (a,w[a],w[a+1]))
E
run sha256sum PROM0.bin
say ""

# ---------------------------------------------------------- 2. disassembler
say "## 2. objdump tic54x (binutils 2.21.1)"
OBJDUMP=build-tic54x/binutils/objdump
if [ ! -x "$OBJDUMP" ]; then
    [ -f binutils-2.21.1.tar.bz2 ] || run curl -sO https://ftp.gnu.org/gnu/binutils/binutils-2.21.1.tar.bz2
    [ -d binutils-2.21.1 ] || tar xjf binutils-2.21.1.tar.bz2
    mkdir -p build-tic54x && ( cd build-tic54x && \
      ../binutils-2.21.1/configure --target=tic54x-coff --disable-werror --disable-nls --disable-gdb \
        --disable-gas --disable-ld --disable-gold --disable-gprof --disable-sim MAKEINFO=true \
        CFLAGS='-O1 -w -std=gnu89' >/dev/null && make -j8 all-binutils MAKEINFO=true >/dev/null ) \
      && say "built $OBJDUMP" || say "BUILD FAILED"
fi
run "$OBJDUMP" --version
"$OBJDUMP" -D -b binary -m tms320c54x --adjust-vma=0x7000 PROM0.bin > PROM0.dis 2>&1
say "full disassembly: PROM0.dis ($(wc -l < PROM0.dis) lines)"
say ""
say "### expected: 7cbd sub #11236,a | 84a4 stm #10791,ar3 | b21d stm #191,ar3 | b2b1 stm #151,ar3 | b32d stm #48,ar3"
grep -E '^ *(7cbd|84a4|b21d|b2b1|b32d):' PROM0.dis | tee -a "$LOG"
say ""
say "### SB TOA chain"
for a in 7c3d 81c8 7c49 7a1c 84a1 84ca 84dd 7c5f 7ca6 7cbc 7e8a b1e7; do
    grep -E "^ *$a:" PROM0.dis | tee -a "$LOG"
done
say ""
say "### FB TOA: 0x793e-0x795a"
awk '/^ *793e:/,/^ *795a:/' PROM0.dis | tee -a "$LOG"
say ""
say "### DMA programming routine: 0xa5cd-0xa5f6"
awk '/^ *a5cd:/,/^ *a5f6:/' PROM0.dis | tee -a "$LOG"
say ""

# ---------------------------------------------------------------- 3. git
say "## 3. layer1 git history"
[ -d osmocom-bb ] || run git clone -q https://gitea.osmocom.org/phone-side/osmocom-bb
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -S"toa -= 23" -- src/target/firmware/layer1
run git -C osmocom-bb log --format='%h %ad %an %s' --date=short -S"magic 23" -- src/target/firmware/layer1
say "### cb71b972: toa -= 23 / fbinfo2cellinfo lines (expect added fbinfo2cellinfo, no removed toa -= 23)"
git -C osmocom-bb show cb71b972 -- src/target/firmware/layer1/prim_fbsb.c | grep -nE '^[-+].*(toa -= 23|fbinfo2cellinfo)|^@@' | tee -a "$LOG"
say ""
say "### current constants"
grep -nE 'L1_(SB|NB)_MARGIN_Q|L1_TAIL_DURATION_Q|L1_(SB|NB)_DURATION_Q' osmocom-bb/src/target/firmware/layer1/tpu_window.c | tee -a "$LOG"
grep -nE 'toa -= 23|magic 23' osmocom-bb/src/target/firmware/layer1/prim_fbsb.c | tee -a "$LOG"
grep -nE 'tpu_shift \+= 75|SB TOA of 4' osmocom-bb/src/target/firmware/layer1/sync.c | tee -a "$LOG"
say ""

# ---------------------------------------------------------------- 4. logs
say "## 4. hardware logs: qbits check ((TOA-46) mod 1250)*4"
say "sources: http://lists.osmocom.org/pipermail/baseband-devel/2011-September/002526.html"
say "         https://www.mail-archive.com/baseband-devel@lists.osmocom.org/msg01046.html"
python3 - <<'E' 2>&1 | tee -a "$LOG"
fb1 = [(9651,3420),(8751,4820),(8755,4836),(10003,4828),(10007,4844),(8755,4836)]
sb  = [(29,4932),(29,4932),(27,4924),(27,4924),(24,4912),(26,4920)]
ok = 0
for toa,q in fb1+sb:
    p46 = ((toa-46)%1250)*4; p23 = ((toa-23)%1250)*4
    ok += (p46==q)
    print("  TOA=%5d printed=%4d  -46:%4d %s  -23:%4d" % (toa,q,p46,"OK " if p46==q else "NO ",p23))
print("  %d/%d lines match the -46 rule" % (ok,len(fb1)+len(sb)))
fb0=[7200,2544,8784,10032,48,1296,8784]; m1=[9651,8751,8755,10003,10007,12507,12507,1259,10007,8755]
print("  FB mode 0 multiples of 48: %d/%d" % (sum(t%48==0 for t in fb0),len(fb0)))
print("  FB mode 1 == 3 mod 4:      %d/%d" % (sum(t%4==3 for t in m1),len(m1)))
d=[t-27.25 for t,_ in sb]
print("  first SB after FB sync: %s  mean delta %.2f" % ([t for t,_ in sb], sum(d)/len(d)))
print("  148 + 2*23 - 3 = %d ; 148 + 2*3 - 3 = %d ; (191-148+3)/2 = %g" % (148+2*23-3,148+2*3-3,(191-148+3)/2))
E
say ""
say "# done — $(wc -l < "$LOG") lines in $LOG"
```
