# "magic 23": references

Full argument: [`proof_toa_23.md`](../../proof_toa_23.md). Reproduce: `tools/toa23/reproduce.sh` (writes `toa.log`).

**OsmocomBB** at HEAD `168b8c5f` (2026-10-08), `src/target/firmware/layer1/`
| file:line | what |
|---|---|
| `prim_fbsb.c:333-334` | `/* FIXME: where did this magic 23 come from? */` `toa -= 23`, in `fbinfo2cellinfo` (blame `cb71b972`) |
| `prim_fbsb.c:469-470` | same FIXME, mode-1 block of `l1s_fbdet_resp` (blame `8978ec57`) |
| `prim_fbsb.c:206` | `toa -= 23` in `l1s_sbdet_resp` |
| `prim_fbsb.c:491`, `:532` | `fbinfo2cellinfo` called after FB1 and from `l1a_fb_compl` (registered at `:572`) |
| `tpu_window.c:38-44` | `L1_SB_MARGIN_Q (23 * 4)`, `L1_NB_MARGIN_Q (3 * 4)`, `burst + 2·margin − tail` |
| `sync.c:119-120` | "We have to add 75 to get an SB TOA of 4", `tpu_shift += 75` |
| `toa.c` | NB loop held at 16 qbits (`937023be`, 2011) |

**Commits**: `fbe7b94c` (2010-02-18, initial import: margins, both `toa -= 23`, `+= 75`) ·
`cb71b972` (2010-05-20, "Make new L1CTL_FBSB_REQ work reliably": `fbinfo2cellinfo`, double subtraction
on the FB path). Find them with `git log -G'toa -= 23'` (`-S` misses `cb71b972`).

**TI DSP ROM 3606**, PROM0
| address | instruction | meaning |
|---|---|---|
| `0xb21d` | `stm #191,ar3` | SB receive window, 191 samples = 148 + 2·23 − 3 |
| `0xb2b1` | `stm #151,ar3` | NB receive window, 151 samples = 148 + 2·3 − 3 |
| `0xb32d` | `stm #48,ar3` | FB, 48-sample DMA pages |
| `0xa5cd` | DMA programming routine | AR3 = length, ports 0xfc24/26/28 |
| `0x84a4` | `stm #10791,ar3` | correlation starts at sample 39 |
| `0x7cbd` | `sub #11236,a` | TOA = index of the retained lag |
| `0x7a1c` | 64 data words | SB extended training sequence, GSM 05.02 §5.2.5, 64/64 bits |

**Hardware logs** (baseband-devel)
- [1] 2011-09-26, DSP API 0x3606: https://lists.osmocom.org/pipermail/baseband-devel/2011-September/002526.html
- [2] 2024-10-02, Motorola C115: https://www.mail-archive.com/baseband-devel@lists.osmocom.org/msg01046.html

**SHA-256**
- `dsp-rom-3606-dump.txt` (ftp://ftp.freecalypso.org/pub/GSM/Calypso/): `e35798f50fd0a29aa61cd53e417c010d7f172582c7bc055211daab8af1e05470`
- `PROM0.bin` (0x7000-0xdfff, little-endian words): `75ce5375a07eac85ee42ba00af7ca82089d15b5df517710a756e6c26497021d7`
