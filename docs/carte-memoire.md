# Carte mémoire du DSP Calypso — API RAM, NDB, PARAM, mémoire interne, ports

> **État : PREMIÈRE VERSION, interrompue (limite d'usage, 2026-10-03 ~18:15).** Les tables de la
> section 2 sont générées (DWARF de l'ELF + mesure du rejeu) ; le reste est écrit à la main à
> partir de la mesure et des sources citées. Le générateur `tools/layer1_tester.py` est
> inachevé (voir « Reste à faire ») : cette page n'est pas encore régénérable d'une commande.

## Comment ce fichier a été produit

* Offsets des champs : `gdb-multiarch -batch -ex 'ptype /o T_NDB_MCU_DSP' /opt/GSM/firmware/board/compal_e88/layer1.highram.elf`
  (et `T_DB_MCU_TO_DSP`, `T_DB_DSP_TO_MCU`, `T_PARAM_MCU_DSP`) : c'est la disposition **compilée**
  (DSP=36, CHIPSET=12, ANLG_FAM=2, `include/calypso/l1_environment.h`). Bases :
  `print/x dsp_api` dans le même ELF (ndb=0xffd001a8, db_r=0xffd00050, db_w=0xffd00000, param=0xffd00862).
* Mesure : `make layer1_tester && tools/layer1_tester -o mesure.tsv -j journal.tsv -p pcs.tsv rejeu.bin`
  — rejeu de l'enregistrement de canal dédié (copie de `/dev/shm/calypso_rejeu_tch.bin`, ticks
  1712..35633, 18391 ticks, 393 M instructions, 46 s). Fidélité : SACCH 132 bonnes / 33 Fire KO,
  FACCH 35 / 0, parole 2654 trames toutes B_BFI = mêmes bilans que `tools/rejeu_banc`.
* Conversion : mot DSP = 0x0800 + (adresse ARM − 0xFFD00000)/2 ; « octet API » = adresse ARM − 0xFFD00000.
* « ROM lit / écrit » = accès faits par une instruction de la ROM (crochet `calypso_mbx_evt`) ;
  « ARM modifie » = mots changés côté QEMU entre deux trames (enregistrements `A`), donc des
  modifications, pas toutes les écritures ; valeurs = jusqu'à 4 plus fréquentes (valeur:compte).
  1er PC en hexadécimal (`x:pppp` si page XPC ≠ 0). Champs `*hole*` non touchés omis.

## 1. Plan de l'espace de données (vu du DSP)

| plage | contenu | source |
|---|---|---|
| 0x0000..0x001f | MMR du C54x (IMR, IFR, ST0/1, AR0..7, SP, BK, BRC, PMST, XPC…) | qosmo `l1-dsp/calypso_c54x.h:28-54` |
| 0x0020..0x005f | périphériques (timer 0x24..0x26, DMA 0x54..0x57) | `calypso_c54x.h`, `calypso_dma.h:45-48` |
| 0x0800..0x0813 | page W0 (`T_DB_MCU_TO_DSP`, ARM 0xFFD00000) | ELF |
| 0x0814..0x0827 | page W1 (ARM 0xFFD00028) | ELF |
| 0x0828..0x083b | page R0 (`T_DB_DSP_TO_MCU`, ARM 0xFFD00050) | ELF |
| 0x083c..0x084f | page R1 (ARM 0xFFD00078) | ELF |
| 0x08d4..0x153d | NDB (`T_NDB_MCU_DSP`, ARM 0xFFD001A8, 6356 octets) | ELF |
| 0x0c31..0x0cc1 | PARAM (`T_PARAM_MCU_DSP`, ARM 0xFFD00862) — **dans** NDB.a_sr_holes1 (290 octets = les 145 mots utiles de PARAM) | ELF |
| 0x0cce..0x0df5 | tampon DMA des I/Q reçus (AAD 0x99c) — dans NDB.a_cport_holes | `c54x_mem.c` (FEED-DST) ; mesure : 3219 écritures hors instruction (DMA/BSP) |
| 0x0ffc..0x0fff | mots du chargeur BL_ADDR_HI / BL_SIZE / BL_ADDR_LO / BL_CMD_STATUS (ARM 0xFFD00FF8..FFE) — dans NDB.a_cport_holes | `firmware/calypso/dsp.c:54-57` |
| 0x153e..0x27ff | API RAM hors structures layer1 | — |
| ≥ 0x2800 | mémoire interne du DSP (section 4) | — |

## 2. API RAM — tous les champs (générée)

| zone | champ | octet API | ARM | mot DSP | ROM lit (n, 1er PC) | ROM écrit (n, 1er PC) | ARM modifie (n, valeurs) |
|---|---|---|---|---|---|---|---|
| W0 | d_task_d | 0x000 | 0xFFD00000 | 0x0800 | 25920, b001 | 0 | 1296, 000d:445,000e:361,0000:288,0018:202 |
| W0 | d_burst_d | 0x002 | 0xFFD00002 | 0x0801 | 14950, b005 | 361, ba17 | 777, 0000:475,0002:102,0001:100,0003:100 |
| W0 | d_task_u | 0x004 | 0xFFD00004 | 0x0802 | 13229, b009 | 0 | 1298, 000d:445,000e:361,0000:288,000c:204 |
| W0 | d_burst_u | 0x006 | 0xFFD00006 | 0x0803 | 8345, b00d | 361, ba19 | 780, 0000:475,0002:103,0001:101,0003:101 |
| W0 | d_task_md | 0x008 | 0xFFD00008 | 0x0804 | 19843, b011 | 0 | 300, 0001:150,0000:150 |
| W0 | d_background | 0x00a | 0xFFD0000A | 0x0805 | 6614, b0db | 0 | 0 |
| W0 | d_debug | 0x00c | 0xFFD0000C | 0x0806 | 13229, b015 | 0 | 0 |
| W0 | d_task_ra | 0x00e | 0xFFD0000E | 0x0807 | 6614, b097 | 0 | 0 |
| W0 | d_fn | 0x010 | 0xFFD00010 | 0x0808 | 13613, b548 | 0 | 5760, ,+ |
| W0 | d_ctrl_tch | 0x012 | 0xFFD00012 | 0x0809 | 52823, b54b | 0 | 518, 0000:255,0030:169,0011:83,0311:1,+ |
| W0 | hole | 0x014 | 0xFFD00014 | 0x080a | 0 | 0 | 0 |
| W0 | d_ctrl_abb | 0x016 | 0xFFD00016 | 0x080b | 6615, b5a5 | 0 | 576, 0019:288,0010:288 |
| W0 | a_a5fn[2] | 0x018 | 0xFFD00018 | 0x080c..0x080d | 13228, b044 | 0 | 7247, ,+ |
| W0 | d_power_ctl | 0x01c | 0xFFD0001C | 0x080e | 6615, b5a9 | 0 | 587, 0000:276,2e92:111,3212:72,2bd2:68,+ |
| W0 | d_afc | 0x01e | 0xFFD0001E | 0x080f | 6615, b5ad | 0 | 1, fd44:1 |
| W0 | d_ctrl_system | 0x020 | 0xFFD00020 | 0x0810 | 19856, a53c | 13, a549 | 522, 0007:255,0000:254,8000:11,8007:2 |
| W1 | d_task_d | 0x028 | 0xFFD00028 | 0x0814 | 25607, b001 | 0 | 1293, 0000:567,000d:443,0018:202,000e:81 |
| W1 | d_burst_d | 0x02a | 0xFFD0002A | 0x0815 | 14297, b005 | 81, ba17 | 571, 0000:267,0001:102,0003:102,0002:100 |
| W1 | d_task_u | 0x02c | 0xFFD0002C | 0x0816 | 13206, b009 | 0 | 1294, 0000:566,000d:443,000c:204,000e:81 |
| W1 | d_burst_u | 0x02e | 0xFFD0002E | 0x0817 | 7703, b00d | 81, ba19 | 571, 0000:264,0003:103,0001:103,0002:101 |
| W1 | d_task_md | 0x030 | 0xFFD00030 | 0x0818 | 19845, b011 | 0 | 866, 0001:432,0000:427,0005:7 |
| W1 | d_background | 0x032 | 0xFFD00032 | 0x0819 | 6603, b0db | 0 | 0 |
| W1 | d_debug | 0x034 | 0xFFD00034 | 0x081a | 13206, b015 | 0 | 0 |
| W1 | d_task_ra | 0x036 | 0xFFD00036 | 0x081b | 6603, b097 | 0 | 0 |
| W1 | d_fn | 0x038 | 0xFFD00038 | 0x081c | 13325, b548 | 0 | 5756, 0000:1,+ |
| W1 | d_ctrl_tch | 0x03a | 0xFFD0003A | 0x081d | 51969, b54b | 0 | 1061, 0000:531,0011:363,0030:166,0311:1 |
| W1 | hole | 0x03c | 0xFFD0003C | 0x081e | 0 | 0 | 0 |
| W1 | d_ctrl_abb | 0x03e | 0xFFD0003E | 0x081f | 6603, b5a5 | 0 | 1132, 0019:566,0010:566 |
| W1 | a_a5fn[2] | 0x040 | 0xFFD00040 | 0x0820..0x0821 | 13206, b044 | 0 | 7791, 0095:1,0000:1,+ |
| W1 | d_power_ctl | 0x044 | 0xFFD00044 | 0x0822 | 6603, b5a9 | 0 | 1141, 0000:501,2392:255,2e92:42,3212:18,+ |
| W1 | d_afc | 0x046 | 0xFFD00046 | 0x0823 | 6603, b5ad | 0 | 1, fd44:1 |
| W1 | d_ctrl_system | 0x048 | 0xFFD00048 | 0x0824 | 19809, a53c | 0 | 1057, 0000:531,0007:526 |
| R0 | d_task_d | 0x050 | 0xFFD00050 | 0x0828 | 0 | 6615, b003 | 6076, 0000:6076 |
| R0 | d_burst_d | 0x052 | 0xFFD00052 | 0x0829 | 0 | 6615, b007 | 302, 0000:302 |
| R0 | d_task_u | 0x054 | 0xFFD00054 | 0x082a | 0 | 11838, b00b | 6080, 0000:6080 |
| R0 | d_burst_u | 0x056 | 0xFFD00056 | 0x082b | 0 | 6615, b00f | 305, 0000:305 |
| R0 | d_task_md | 0x058 | 0xFFD00058 | 0x082c | 0 | 6615, b013 | 150, 0000:150 |
| R0 | d_background | 0x05a | 0xFFD0005A | 0x082d | 0 | 0 | 0 |
| R0 | d_debug | 0x05c | 0xFFD0005C | 0x082e | 0 | 6615, b017 | 0 |
| R0 | d_task_ra | 0x05e | 0xFFD0005E | 0x082f | 0 | 0 | 0 |
| R0 | a_serv_demod[4] | 0x060 | 0xFFD00060 | 0x0830..0x0833 | 0 | 24288, b267 | 24225, 0000:6076 |
| R0 | a_pm[3] | 0x068 | 0xFFD00068 | 0x0834..0x0836 | 0 | 150, b34b | 150, 0000:150 |
| R0 | a_sch[5] | 0x06e | 0xFFD0006E | 0x0837..0x083b | 0 | 0 | 0 |
| R1 | d_task_d | 0x078 | 0xFFD00078 | 0x083c | 0 | 6603, b003 | 5796, 0000:5796 |
| R1 | d_burst_d | 0x07a | 0xFFD0007A | 0x083d | 0 | 6603, b007 | 304, 0000:304 |
| R1 | d_task_u | 0x07c | 0xFFD0007C | 0x083e | 0 | 11823, b00b | 5800, 0000:5800 |
| R1 | d_burst_u | 0x07e | 0xFFD0007E | 0x083f | 0 | 6603, b00f | 307, 0000:307 |
| R1 | d_task_md | 0x080 | 0xFFD00080 | 0x0840 | 0 | 6603, b013 | 427, 0000:427 |
| R1 | d_background | 0x082 | 0xFFD00082 | 0x0841 | 0 | 0 | 0 |
| R1 | d_debug | 0x084 | 0xFFD00084 | 0x0842 | 0 | 6603, b017 | 0 |
| R1 | d_task_ra | 0x086 | 0xFFD00086 | 0x0843 | 0 | 0 | 0 |
| R1 | a_serv_demod[4] | 0x088 | 0xFFD00088 | 0x0844..0x0847 | 0 | 23212, b267 | 23120, 0000:5796 |
| R1 | a_pm[3] | 0x090 | 0xFFD00090 | 0x0848..0x084a | 0 | 432, b34b | 427, 0000:427 |
| R1 | a_sch[5] | 0x096 | 0xFFD00096 | 0x084b..0x084f | 0 | 0 | 0 |
| NDB | d_dsp_page | 0x1a8 | 0xFFD001A8 | 0x08d4 | 13218, a51c | 0 | 13234, 0002:6614,0003:6610,0000:10 |
| NDB | d_error_status | 0x1aa | 0xFFD001AA | 0x08d5 | 0 | 13218, b10a | 0 |
| NDB | d_spcx_rif | 0x1ac | 0xFFD001AC | 0x08d6 | 13218, b56c | 0 | 0 |
| NDB | d_tch_mode | 0x1ae | 0xFFD001AE | 0x08d7 | 14711, 85e4 | 0 | 13, ca00:7,ca08:6 |
| NDB | d_debug1 | 0x1b0 | 0xFFD001B0 | 0x08d8 | 11838, b585 | 0 | 0 |
| NDB | d_dsp_test | 0x1b2 | 0xFFD001B2 | 0x08d9 | 10616, a90a | 0 | 0 |
| NDB | d_version_number1 | 0x1b4 | 0xFFD001B4 | 0x08da | 0 | 0 | 0 |
| NDB | d_version_number2 | 0x1b6 | 0xFFD001B6 | 0x08db | 0 | 0 | 0 |
| NDB | d_debug_ptr | 0x1b8 | 0xFFD001B8 | 0x08dc | 81581, b524 | 81581, b530 | 0 |
| NDB | d_debug_bk | 0x1ba | 0xFFD001BA | 0x08dd | 81581, b528 | 0 | 0 |
| NDB | d_pll_config | 0x1bc | 0xFFD001BC | 0x08de | 34471, a687 | 0 | 0 |
| NDB | p_debug_buffer | 0x1be | 0xFFD001BE | 0x08df | 0 | 0 | 0 |
| NDB | d_debug_buffer_size | 0x1c0 | 0xFFD001C0 | 0x08e0 | 0 | 0 | 0 |
| NDB | d_debug_trace_type | 0x1c2 | 0xFFD001C2 | 0x08e1 | 0 | 0 | 0 |
| NDB | d_dsp_state | 0x1c4 | 0xFFD001C4 | 0x08e2 | 0 | 0 | 0 |
| NDB | d_mcsi_select | 0x1ce | 0xFFD001CE | 0x08e7 | 0 | 0 | 0 |
| NDB | d_apcdel1_bis | 0x1d0 | 0xFFD001D0 | 0x08e8 | 0 | 0 | 0 |
| NDB | d_apcdel2_bis | 0x1d2 | 0xFFD001D2 | 0x08e9 | 0 | 0 | 0 |
| NDB | d_apcdel2 | 0x1d4 | 0xFFD001D4 | 0x08ea | 0 | 0 | 0 |
| NDB | d_vbctrl2 | 0x1d6 | 0xFFD001D6 | 0x08eb | 0 | 0 | 0 |
| NDB | d_bulgcal | 0x1d8 | 0xFFD001D8 | 0x08ec | 0 | 0 | 0 |
| NDB | d_afcctladd | 0x1da | 0xFFD001DA | 0x08ed | 13218, b5ba | 0 | 0 |
| NDB | d_vbuctrl | 0x1dc | 0xFFD001DC | 0x08ee | 13226, b5c4 | 0 | 0 |
| NDB | d_vbdctrl | 0x1de | 0xFFD001DE | 0x08ef | 13218, b5c9 | 0 | 0 |
| NDB | d_apcdel1 | 0x1e0 | 0xFFD001E0 | 0x08f0 | 11885, b660 | 0 | 0 |
| NDB | d_apcoff | 0x1e2 | 0xFFD001E2 | 0x08f1 | 36990, b5bf | 11886, b67e | 11886, 1817:11886 |
| NDB | d_bulioff | 0x1e4 | 0xFFD001E4 | 0x08f2 | 13218, b5d3 | 0 | 0 |
| NDB | d_bulqoff | 0x1e6 | 0xFFD001E6 | 0x08f3 | 13218, b5d8 | 0 | 0 |
| NDB | d_dai_onoff | 0x1e8 | 0xFFD001E8 | 0x08f4 | 13218, b5ed | 0 | 0 |
| NDB | d_auxdac | 0x1ea | 0xFFD001EA | 0x08f5 | 13218, b5dd | 0 | 0 |
| NDB | d_vbctrl1 | 0x1ec | 0xFFD001EC | 0x08f6 | 13230, b5e2 | 4, b6a8 | 0 |
| NDB | d_bbctrl | 0x1ee | 0xFFD001EE | 0x08f7 | 13218, b5ce | 0 | 0 |
| NDB | d_fb_det | 0x1f0 | 0xFFD001F0 | 0x08f8 | 6397, 778a | 6397, b2cc | 0 |
| NDB | d_fb_mode | 0x1f2 | 0xFFD001F2 | 0x08f9 | 18874, 781f | 0 | 0 |
| NDB | a_sync_demod[4] | 0x1f4 | 0xFFD001F4 | 0x08fa..0x08fd | 0 | 25468, b2cf | 0 |
| NDB | a_sch26[5] | 0x1fc | 0xFFD001FC | 0x08fe..0x0902 | 0 | 0 | 0 |
| NDB | d_audio_gain_ul | 0x206 | 0xFFD00206 | 0x0903 | 0 | 0 | 0 |
| NDB | d_audio_gain_dl | 0x208 | 0xFFD00208 | 0x0904 | 0 | 0 | 0 |
| NDB | d_audio_compressor_ctrl | 0x20a | 0xFFD0020A | 0x0905 | 0 | 0 | 0 |
| NDB | d_audio_init | 0x20c | 0xFFD0020C | 0x0906 | 13218, c1fe | 0 | 0 |
| NDB | d_audio_status | 0x20e | 0xFFD0020E | 0x0907 | 13218, c1ff | 0 | 0 |
| NDB | d_toneskb_init | 0x210 | 0xFFD00210 | 0x0908 | 5306, 7382 | 0 | 0 |
| NDB | d_toneskb_status | 0x212 | 0xFFD00212 | 0x0909 | 23882, 7365 | 5308, 73af | 0 |
| NDB | d_k_x1_t0 | 0x214 | 0xFFD00214 | 0x090a | 0 | 0 | 0 |
| NDB | d_k_x1_t1 | 0x216 | 0xFFD00216 | 0x090b | 0 | 0 | 0 |
| NDB | d_k_x1_t2 | 0x218 | 0xFFD00218 | 0x090c | 0 | 0 | 0 |
| NDB | d_pe_rep | 0x21a | 0xFFD0021A | 0x090d | 0 | 0 | 0 |
| NDB | d_pe_off | 0x21c | 0xFFD0021C | 0x090e | 0 | 0 | 0 |
| NDB | d_se_off | 0x21e | 0xFFD0021E | 0x090f | 0 | 0 | 0 |
| NDB | d_bu_off | 0x220 | 0xFFD00220 | 0x0910 | 0 | 0 | 0 |
| NDB | d_t0_on | 0x222 | 0xFFD00222 | 0x0911 | 0 | 0 | 0 |
| NDB | d_t0_off | 0x224 | 0xFFD00224 | 0x0912 | 0 | 0 | 0 |
| NDB | d_t1_on | 0x226 | 0xFFD00226 | 0x0913 | 0 | 0 | 0 |
| NDB | d_t1_off | 0x228 | 0xFFD00228 | 0x0914 | 0 | 0 | 0 |
| NDB | d_t2_on | 0x22a | 0xFFD0022A | 0x0915 | 0 | 0 | 0 |
| NDB | d_t2_off | 0x22c | 0xFFD0022C | 0x0916 | 0 | 0 | 0 |
| NDB | d_k_x1_kt0 | 0x22e | 0xFFD0022E | 0x0917 | 0 | 0 | 0 |
| NDB | d_k_x1_kt1 | 0x230 | 0xFFD00230 | 0x0918 | 0 | 0 | 0 |
| NDB | d_dur_kb | 0x232 | 0xFFD00232 | 0x0919 | 0 | 0 | 0 |
| NDB | d_shiftdl | 0x234 | 0xFFD00234 | 0x091a | 0 | 0 | 0 |
| NDB | d_shiftul | 0x236 | 0xFFD00236 | 0x091b | 0 | 0 | 0 |
| NDB | d_aec_ctrl | 0x238 | 0xFFD00238 | 0x091c | 10625, bf28 | 0 | 0 |
| NDB | d_es_level_api | 0x23a | 0xFFD0023A | 0x091d | 0 | 0 | 0 |
| NDB | d_mu_api | 0x23c | 0xFFD0023C | 0x091e | 0 | 0 | 0 |
| NDB | d_melo_osc_used | 0x23e | 0xFFD0023E | 0x091f | 0 | 0 | 0 |
| NDB | d_melo_osc_active | 0x240 | 0xFFD00240 | 0x0920 | 0 | 0 | 0 |
| NDB | a_melo_note0[4] | 0x242 | 0xFFD00242 | 0x0921..0x0924 | 0 | 0 | 0 |
| NDB | a_melo_note1[4] | 0x24a | 0xFFD0024A | 0x0925..0x0928 | 0 | 0 | 0 |
| NDB | a_melo_note2[4] | 0x252 | 0xFFD00252 | 0x0929..0x092c | 0 | 0 | 0 |
| NDB | a_melo_note3[4] | 0x25a | 0xFFD0025A | 0x092d..0x0930 | 0 | 0 | 0 |
| NDB | a_melo_note4[4] | 0x262 | 0xFFD00262 | 0x0931..0x0934 | 0 | 0 | 0 |
| NDB | a_melo_note5[4] | 0x26a | 0xFFD0026A | 0x0935..0x0938 | 0 | 0 | 0 |
| NDB | a_melo_note6[4] | 0x272 | 0xFFD00272 | 0x0939..0x093c | 0 | 0 | 0 |
| NDB | a_melo_note7[4] | 0x27a | 0xFFD0027A | 0x093d..0x0940 | 0 | 0 | 0 |
| NDB | d_melody_selection | 0x282 | 0xFFD00282 | 0x0941 | 0 | 0 | 0 |
| NDB | d_sr_status | 0x28a | 0xFFD0028A | 0x0945 | 0 | 0 | 0 |
| NDB | d_sr_param | 0x28c | 0xFFD0028C | 0x0946 | 0 | 0 | 0 |
| NDB | d_sr_bit_exact_test | 0x28e | 0xFFD0028E | 0x0947 | 0 | 0 | 0 |
| NDB | d_sr_nb_words | 0x290 | 0xFFD00290 | 0x0948 | 0 | 0 | 0 |
| NDB | d_sr_db_level | 0x292 | 0xFFD00292 | 0x0949 | 0 | 0 | 0 |
| NDB | d_sr_db_noise | 0x294 | 0xFFD00294 | 0x094a | 0 | 0 | 0 |
| NDB | d_sr_mod_size | 0x296 | 0xFFD00296 | 0x094b | 0 | 0 | 0 |
| NDB | a_n_best_words[4] | 0x298 | 0xFFD00298 | 0x094c..0x094f | 0 | 0 | 0 |
| NDB | a_n_best_score[8] | 0x2a0 | 0xFFD002A0 | 0x0950..0x0957 | 0 | 0 | 0 |
| NDB | a_dd_1[22] | 0x2b0 | 0xFFD002B0 | 0x0958..0x096d | 0 | 0 | 0 |
| NDB | a_du_1[22] | 0x2dc | 0xFFD002DC | 0x096e..0x0983 | 52416, a925 | 2656, a925 | 3815, 8000:2488 |
| NDB | d_v42b_nego0 | 0x308 | 0xFFD00308 | 0x0984 | 0 | 0 | 0 |
| NDB | d_v42b_nego1 | 0x30a | 0xFFD0030A | 0x0985 | 0 | 0 | 0 |
| NDB | d_v42b_control | 0x30c | 0xFFD0030C | 0x0986 | 0 | 0 | 0 |
| NDB | d_v42b_ratio_ind | 0x30e | 0xFFD0030E | 0x0987 | 0 | 0 | 0 |
| NDB | d_mcu_control | 0x310 | 0xFFD00310 | 0x0988 | 0 | 0 | 0 |
| NDB | d_mcu_control_sema | 0x312 | 0xFFD00312 | 0x0989 | 0 | 0 | 0 |
| NDB | d_background_enable | 0x314 | 0xFFD00314 | 0x098a | 94035, dde8 | 0 | 0 |
| NDB | d_background_abort | 0x316 | 0xFFD00316 | 0x098b | 0 | 0 | 0 |
| NDB | d_background_state | 0x318 | 0xFFD00318 | 0x098c | 0 | 59564, dde8 | 0 |
| NDB | d_max_background | 0x31a | 0xFFD0031A | 0x098d | 0 | 0 | 0 |
| NDB | a_background_tasks[16] | 0x31c | 0xFFD0031C | 0x098e..0x099d | 0 | 0 | 0 |
| NDB | a_back_task_io[16] | 0x33c | 0xFFD0033C | 0x099e..0x09ad | 0 | 0 | 0 |
| NDB | d_gea_mode_ovly | 0x35c | 0xFFD0035C | 0x09ae | 0 | 0 | 0 |
| NDB | a_gea_kc_ovly[4] | 0x35e | 0xFFD0035E | 0x09af..0x09b2 | 0 | 0 | 0 |
| NDB | d_thr_usf_detect | 0x374 | 0xFFD00374 | 0x09ba | 0 | 0 | 0 |
| NDB | d_a5mode | 0x376 | 0xFFD00376 | 0x09bb | 37225, b138 | 0 | 15, 0001:8,0000:7 |
| NDB | d_sched_mode_gprs_ovly | 0x378 | 0xFFD00378 | 0x09bc | 13, a541 | 0 | 0 |
| NDB | a_ramp[16] | 0x384 | 0xFFD00384 | 0x09c2..0x09d1 | 190160, b642 | 0 | 166, 4fd4:6,8fd4:1,+ |
| NDB | a_cd[15] | 0x3a4 | 0xFFD003A4 | 0x09d2..0x09e0 | 4056, 9718 | 8736, 96dd | 220, 0040:110 |
| NDB | a_fd[15] | 0x3c2 | 0xFFD003C2 | 0x09e1..0x09ef | 11180, b10d | 3634, a72c | 70, 0040:35 |
| NDB | a_dd_0[22] | 0x3e0 | 0xFFD003E0 | 0x09f0..0x0a05 | 97541, a38e | 65580, 928a | 5308, 0000:2654 |
| NDB | a_cu[15] | 0x40c | 0xFFD0040C | 0x0a06..0x0a14 | 4095, aa5a | 315, aa5a | 2239, 8000:315 |
| NDB | a_fu[15] | 0x42a | 0xFFD0042A | 0x0a15..0x0a23 | 8292, a8bd | 2656, a928 | 109, 8000:26 |
| NDB | a_du_0[22] | 0x448 | 0xFFD00448 | 0x0a24..0x0a39 | 65781, ad0d | 105565, ad90 | 0 |
| NDB | d_rach | 0x474 | 0xFFD00474 | 0x0a3a | 0 | 0 | 0 |
| NDB | a_kc[4] | 0x476 | 0xFFD00476 | 0x0a3b..0x0a3e | 147204, b145 | 0 | 8, 8c00:1,f000:1 |
| NDB | d_ra_conf | 0x47e | 0xFFD0047E | 0x0a3f | 2629, b124 | 0 | 0 |
| NDB | d_ra_act | 0x480 | 0xFFD00480 | 0x0a40 | 0 | 0 | 0 |
| NDB | d_ra_test | 0x482 | 0xFFD00482 | 0x0a41 | 0 | 0 | 0 |
| NDB | d_ra_statu | 0x484 | 0xFFD00484 | 0x0a42 | 0 | 0 | 0 |
| NDB | d_ra_statd | 0x486 | 0xFFD00486 | 0x0a43 | 0 | 0 | 0 |
| NDB | d_fax | 0x488 | 0xFFD00488 | 0x0a44 | 0 | 0 | 0 |
| NDB | a_data_buf_ul[21] | 0x48a | 0xFFD0048A | 0x0a45..0x0a59 | 0 | 0 | 0 |
| NDB | a_data_buf_dl[37] | 0x4b4 | 0xFFD004B4 | 0x0a5a..0x0a7e | 0 | 0 | 0 |
| NDB | a_sr_holes1[145] | 0x862 | 0xFFD00862 | 0x0c31..0x0cc1 | 133372, a5e0 | 0 | 0 |
| NDB | d_cport_init | 0x984 | 0xFFD00984 | 0x0cc2 | 0 | 0 | 0 |
| NDB | d_cport_ctrl | 0x986 | 0xFFD00986 | 0x0cc3 | 0 | 0 | 0 |
| NDB | a_cport_cfr[2] | 0x988 | 0xFFD00988 | 0x0cc4..0x0cc5 | 0 | 0 | 0 |
| NDB | d_cport_tcl_tadt | 0x98c | 0xFFD0098C | 0x0cc6 | 0 | 0 | 0 |
| NDB | d_cport_tdat | 0x98e | 0xFFD0098E | 0x0cc7 | 0 | 0 | 0 |
| NDB | d_cport_tvs | 0x990 | 0xFFD00990 | 0x0cc8 | 0 | 0 | 0 |
| NDB | d_cport_status | 0x992 | 0xFFD00992 | 0x0cc9 | 0 | 0 | 0 |
| NDB | d_cport_reg_value | 0x994 | 0xFFD00994 | 0x0cca | 0 | 0 | 0 |
| NDB | a_cport_holes[1011] | 0x996 | 0xFFD00996 | 0x0ccb..0x10bd | 14510066, 81ad | 1096940, 819b | 0 |
| NDB | a_model[1041] | 0x117c | 0xFFD0117C | 0x10be..0x14ce | 0 | 0 | 0 |
| NDB | a_amr_config[4] | 0x19ca | 0xFFD019CA | 0x14e5..0x14e8 | 0 | 0 | 0 |
| NDB | a_ratscch_ul[6] | 0x19d2 | 0xFFD019D2 | 0x14e9..0x14ee | 0 | 0 | 0 |
| NDB | a_ratscch_dl[6] | 0x19de | 0xFFD019DE | 0x14ef..0x14f4 | 0 | 0 | 0 |
| NDB | d_amr_snr_est | 0x19ea | 0xFFD019EA | 0x14f5 | 0 | 0 | 0 |
| NDB | d_thr_onset_afs | 0x19ee | 0xFFD019EE | 0x14f7 | 0 | 0 | 0 |
| NDB | d_thr_sid_first_afs | 0x19f0 | 0xFFD019F0 | 0x14f8 | 0 | 0 | 0 |
| NDB | d_thr_ratscch_afs | 0x19f2 | 0xFFD019F2 | 0x14f9 | 0 | 0 | 0 |
| NDB | d_thr_update_afs | 0x19f4 | 0xFFD019F4 | 0x14fa | 0 | 0 | 0 |
| NDB | d_thr_onset_ahs | 0x19f6 | 0xFFD019F6 | 0x14fb | 0 | 0 | 0 |
| NDB | d_thr_sid_ahs | 0x19f8 | 0xFFD019F8 | 0x14fc | 0 | 0 | 0 |
| NDB | d_thr_ratscch_marker | 0x19fa | 0xFFD019FA | 0x14fd | 0 | 0 | 0 |
| NDB | d_thr_sp_dgr | 0x19fc | 0xFFD019FC | 0x14fe | 0 | 0 | 0 |
| NDB | d_thr_soft_bits | 0x19fe | 0xFFD019FE | 0x14ff | 0 | 0 | 0 |
| PARAM | d_transfer_rate | 0x862 | 0xFFD00862 | 0x0c31 | 0 | 0 | 0 |
| PARAM | d_lat_mcu_bridge | 0x864 | 0xFFD00864 | 0x0c32 | 43808, a5e0 | 0 | 0 |
| PARAM | d_lat_mcu_hom2sam | 0x866 | 0xFFD00866 | 0x0c33 | 0 | 0 | 0 |
| PARAM | d_lat_mcu_bef_fast_access | 0x868 | 0xFFD00868 | 0x0c34 | 0 | 0 | 0 |
| PARAM | d_lat_dsp_after_sam | 0x86a | 0xFFD0086A | 0x0c35 | 0 | 0 | 0 |
| PARAM | d_gprs_install_address | 0x86c | 0xFFD0086C | 0x0c36 | 0 | 0 | 0 |
| PARAM | d_misc_config | 0x86e | 0xFFD0086E | 0x0c37 | 34471, a684 | 0 | 0 |
| PARAM | d_cn_sw_workaround | 0x870 | 0xFFD00870 | 0x0c38 | 0 | 0 | 0 |
| PARAM | d_hole2_param[4] | 0x872 | 0xFFD00872 | 0x0c39..0x0c3c | 0 | 0 | 0 |
| PARAM | d_fb_margin_beg | 0x87a | 0xFFD0087A | 0x0c3d | 29, 794b | 0 | 0 |
| PARAM | d_fb_margin_end | 0x87c | 0xFFD0087C | 0x0c3e | 29, 7956 | 0 | 0 |
| PARAM | d_nsubb_idle | 0x87e | 0xFFD0087E | 0x0c3f | 36, b316 | 0 | 0 |
| PARAM | d_nsubb_dedic | 0x880 | 0xFFD00880 | 0x0c40 | 0 | 0 | 0 |
| PARAM | d_fb_thr_det_iacq | 0x882 | 0xFFD00882 | 0x0c41 | 6224, 78ef | 0 | 0 |
| PARAM | d_fb_thr_det_track | 0x884 | 0xFFD00884 | 0x0c42 | 0 | 0 | 0 |
| PARAM | d_dc_off_thres | 0x886 | 0xFFD00886 | 0x0c43 | 11875, 7ea1 | 0 | 0 |
| PARAM | d_dummy_thres | 0x888 | 0xFFD00888 | 0x0c44 | 0 | 0 | 0 |
| PARAM | d_dem_pond_gewl | 0x88a | 0xFFD0088A | 0x0c45 | 11875, 8154 | 0 | 0 |
| PARAM | d_dem_pond_red | 0x88c | 0xFFD0088C | 0x0c46 | 11875, 815a | 0 | 0 |
| PARAM | d_maccthresh1 | 0x88e | 0xFFD0088E | 0x0c47 | 0 | 0 | 0 |
| PARAM | d_mldt | 0x890 | 0xFFD00890 | 0x0c48 | 4, e59f | 0 | 0 |
| PARAM | d_maccthresh | 0x892 | 0xFFD00892 | 0x0c49 | 4, e5ad | 0 | 0 |
| PARAM | d_gu | 0x894 | 0xFFD00894 | 0x0c4a | 4, e5b8 | 0 | 0 |
| PARAM | d_go | 0x896 | 0xFFD00896 | 0x0c4b | 4, e5bc | 0 | 0 |
| PARAM | d_attmax | 0x898 | 0xFFD00898 | 0x0c4c | 0 | 0 | 0 |
| PARAM | d_sm | 0x89a | 0xFFD0089A | 0x0c4d | 4, e5c4 | 0 | 0 |
| PARAM | d_b | 0x89c | 0xFFD0089C | 0x0c4e | 0 | 0 | 0 |
| PARAM | d_v42b_switch_hyst | 0x89e | 0xFFD0089E | 0x0c4f | 0 | 0 | 0 |
| PARAM | d_v42b_switch_min | 0x8a0 | 0xFFD008A0 | 0x0c50 | 0 | 0 | 0 |
| PARAM | d_v42b_switch_max | 0x8a2 | 0xFFD008A2 | 0x0c51 | 0 | 0 | 0 |
| PARAM | d_v42b_reset_delay | 0x8a4 | 0xFFD008A4 | 0x0c52 | 0 | 0 | 0 |
| PARAM | d_ldT_hr | 0x8a6 | 0xFFD008A6 | 0x0c53 | 0 | 0 | 0 |
| PARAM | d_maccthresh_hr | 0x8a8 | 0xFFD008A8 | 0x0c54 | 0 | 0 | 0 |
| PARAM | d_maccthresh1_hr | 0x8aa | 0xFFD008AA | 0x0c55 | 0 | 0 | 0 |
| PARAM | d_gu_hr | 0x8ac | 0xFFD008AC | 0x0c56 | 0 | 0 | 0 |
| PARAM | d_go_hr | 0x8ae | 0xFFD008AE | 0x0c57 | 0 | 0 | 0 |
| PARAM | d_b_hr | 0x8b0 | 0xFFD008B0 | 0x0c58 | 0 | 0 | 0 |
| PARAM | d_sm_hr | 0x8b2 | 0xFFD008B2 | 0x0c59 | 0 | 0 | 0 |
| PARAM | d_attmax_hr | 0x8b4 | 0xFFD008B4 | 0x0c5a | 0 | 0 | 0 |
| PARAM | c_mldt_efr | 0x8b6 | 0xFFD008B6 | 0x0c5b | 0 | 0 | 0 |
| PARAM | c_maccthresh_efr | 0x8b8 | 0xFFD008B8 | 0x0c5c | 0 | 0 | 0 |
| PARAM | c_maccthresh1_efr | 0x8ba | 0xFFD008BA | 0x0c5d | 0 | 0 | 0 |
| PARAM | c_gu_efr | 0x8bc | 0xFFD008BC | 0x0c5e | 0 | 0 | 0 |
| PARAM | c_go_efr | 0x8be | 0xFFD008BE | 0x0c5f | 0 | 0 | 0 |
| PARAM | c_b_efr | 0x8c0 | 0xFFD008C0 | 0x0c60 | 0 | 0 | 0 |
| PARAM | c_sm_efr | 0x8c2 | 0xFFD008C2 | 0x0c61 | 0 | 0 | 0 |
| PARAM | c_attmax_efr | 0x8c4 | 0xFFD008C4 | 0x0c62 | 0 | 0 | 0 |
| PARAM | d_sd_min_thr_tchfs | 0x8c6 | 0xFFD008C6 | 0x0c63 | 2619, a31d | 0 | 0 |
| PARAM | d_ma_min_thr_tchfs | 0x8c8 | 0xFFD008C8 | 0x0c64 | 5238, a33b | 0 | 0 |
| PARAM | d_md_max_thr_tchfs | 0x8ca | 0xFFD008CA | 0x0c65 | 0 | 0 | 0 |
| PARAM | d_md1_max_thr_tchfs | 0x8cc | 0xFFD008CC | 0x0c66 | 2619, a356 | 0 | 0 |
| PARAM | d_sd_min_thr_tchhs | 0x8ce | 0xFFD008CE | 0x0c67 | 0 | 0 | 0 |
| PARAM | d_ma_min_thr_tchhs | 0x8d0 | 0xFFD008D0 | 0x0c68 | 0 | 0 | 0 |
| PARAM | d_sd_av_thr_tchhs | 0x8d2 | 0xFFD008D2 | 0x0c69 | 0 | 0 | 0 |
| PARAM | d_md_max_thr_tchhs | 0x8d4 | 0xFFD008D4 | 0x0c6a | 0 | 0 | 0 |
| PARAM | d_md1_max_thr_tchhs | 0x8d6 | 0xFFD008D6 | 0x0c6b | 0 | 0 | 0 |
| PARAM | d_sd_min_thr_tchefs | 0x8d8 | 0xFFD008D8 | 0x0c6c | 0 | 0 | 0 |
| PARAM | d_ma_min_thr_tchefs | 0x8da | 0xFFD008DA | 0x0c6d | 0 | 0 | 0 |
| PARAM | d_md_max_thr_tchefs | 0x8dc | 0xFFD008DC | 0x0c6e | 0 | 0 | 0 |
| PARAM | d_md1_max_thr_tchefs | 0x8de | 0xFFD008DE | 0x0c6f | 0 | 0 | 0 |
| PARAM | d_wed_fil_ini | 0x8e0 | 0xFFD008E0 | 0x0c70 | 0 | 0 | 0 |
| PARAM | d_wed_fil_tc | 0x8e2 | 0xFFD008E2 | 0x0c71 | 0 | 0 | 0 |
| PARAM | d_x_min | 0x8e4 | 0xFFD008E4 | 0x0c72 | 0 | 0 | 0 |
| PARAM | d_x_max | 0x8e6 | 0xFFD008E6 | 0x0c73 | 0 | 0 | 0 |
| PARAM | d_slope | 0x8e8 | 0xFFD008E8 | 0x0c74 | 0 | 0 | 0 |
| PARAM | d_y_min | 0x8ea | 0xFFD008EA | 0x0c75 | 0 | 0 | 0 |
| PARAM | d_y_max | 0x8ec | 0xFFD008EC | 0x0c76 | 0 | 0 | 0 |
| PARAM | d_wed_diff_threshold | 0x8ee | 0xFFD008EE | 0x0c77 | 0 | 0 | 0 |
| PARAM | d_mabfi_min_thr_tchhs | 0x8f0 | 0xFFD008F0 | 0x0c78 | 0 | 0 | 0 |
| PARAM | d_facch_thr | 0x8f2 | 0xFFD008F2 | 0x0c79 | 2654, 91e3 | 0 | 0 |
| PARAM | d_max_ovsp_ul | 0x8f4 | 0xFFD008F4 | 0x0c7a | 0 | 0 | 0 |
| PARAM | d_sync_thres | 0x8f6 | 0xFFD008F6 | 0x0c7b | 0 | 0 | 0 |
| PARAM | d_idle_thres | 0x8f8 | 0xFFD008F8 | 0x0c7c | 0 | 0 | 0 |
| PARAM | d_m1_thres | 0x8fa | 0xFFD008FA | 0x0c7d | 0 | 0 | 0 |
| PARAM | d_max_ovsp_dl | 0x8fc | 0xFFD008FC | 0x0c7e | 0 | 0 | 0 |
| PARAM | d_gsm_bgd_mgt | 0x8fe | 0xFFD008FE | 0x0c7f | 0 | 0 | 0 |
| PARAM | a_fir_holes[4] | 0x900 | 0xFFD00900 | 0x0c80..0x0c83 | 0 | 0 | 0 |
| PARAM | a_fir31_uplink[31] | 0x908 | 0xFFD00908 | 0x0c84..0x0ca2 | 0 | 0 | 0 |
| PARAM | a_fir31_downlink[31] | 0x946 | 0xFFD00946 | 0x0ca3..0x0cc1 | 0 | 0 | 0 |

Bits utiles (`include/calypso/l1_environment.h`, **positions** de bit) : B_BLUD 15 (bloc présent),
B_AF 14, B_EMPTY_BLOCK 10, B_ECRC 9, B_SCH_CRC 8, B_FIRE1 6, B_FIRE0 5, B_BFI 2, B_UFI 0 ;
d_dsp_page : B_GSM_PAGE 1<<0, B_GSM_TASK 1<<1 (masques) ; d_ctrl_system : B_TSQ 0..2,
B_BCCH_FREQ_IND 3, B_TASK_ABORT 15 ; d_ctrl_tch : B_CHAN_MODE 0, B_CHAN_TYPE 4, B_RESET_SACCH 6,
B_VOCODER_ON 7, B_SYNC_TCH_UL 8, B_SYNC_TCH_DL 9, B_STOP_TCH_UL 10, B_STOP_TCH_DL 11, B_TCH_LOOP 12,
B_SUBCHANNEL 15. Tâches : NO 0, FB 5, SB 6, TCH_FB 8, TCH_SB 9, RACH 10, AUL 11, DUL 12, TCHT 13,
TCHA 14, NBN 17, EBN 18, NBS 19, EBS 20, NP 21, EP 22, ALLC 24, CB 25, DDL 26, ADL 27, TCHD 28,
CHECKSUM 33 (bit 15 = inversion I/Q, `dsp_task_iq_swap`).

PARAM (valeurs de `dsp_params`, lues dans l'ELF : `print/x dsp_params`) : d_transfer_rate 0x6666,
d_lat_mcu_bridge 15, d_lat_mcu_hom2sam 12, d_lat_mcu_bef_fast_access 5, d_lat_dsp_after_sam 4,
d_gprs_install_address 0x7002, d_misc_config 1, d_cn_sw_workaround 0xe, d_fb_margin_beg 24,
d_fb_margin_end 22, d_nsubb_idle 296, d_nsubb_dedic 30, d_fb_thr_det_iacq 0x3333,
d_fb_thr_det_track 0x28f6, d_dc_off_thres 0x7fff, d_dummy_thres 0x4400, d_dem_pond_gewl 0x6800,
d_dem_pond_red 0x4eb8, d_maccthresh1 0x1ec0, d_mldt −4, … (suite : `firmware/calypso/dsp_params.c`).

## 3. Faits mesurés sur l'interface (rejeu)

* **d_dsp_page (0x08d4)** : l'ARM l'alterne 0x0002/0x0003 (13234 modifications) ; la ROM le lit
  13218 fois (1er PC 0xa51c) ; chaque modification est lue au tick suivant (latence 1,00).
* **a_cu[0] (0x0a06)** : l'ARM pose B_BLUD (0x8000) 315 fois ; la ROM le lit 315 fois et le
  réécrit 315 fois (PC 0xaa5a) : c'est la ROM qui consomme B_BLUD (le `w[0] &= ~B_BLUD` de
  `montant.c:262` imite donc le bon comportement).
* **a_du_1[0] (0x096e)** : l'ARM pose B_BLUD 2488 fois (parole montante TCH/F) ; la ROM le lit
  5144 fois et l'écrit 2656 fois (PC 0xa925). **a_du_0 (0x0a24..)** : jamais modifié par l'ARM,
  mais lu 5314 / écrit 7829 fois par la ROM. Voir incohérence n° 6.
* **a_cd[0] (0x09d2)** : la ROM l'écrit 624 fois ; l'ARM y remet 0x0040 (B_FIRE1, `prim_tch.c:700`).
* **a_dd_0[0] (0x09f0)** : ROM lit 7892 / écrit 7892 ; l'ARM y remet 0 après lecture (2654 fois).
* **d_version_number1/2 (0x08da/0x08db)** : jamais touchés pendant le rejeu (écrits au boot).

## 4. Mémoire interne du DSP (établie le 2026-10-03 + mesure du rejeu)

| adresse | signification | ROM lit (n, 1er PC) | ROM écrit (n, 1er PC) | source |
|---|---|---|---|---|
| 0x4280..0x429f | tampon montant entrelacé xCCH, 4 bursts × 8 mots, circulaire BK=32 | 1578, 8dbd | 1579, 8869 | mémoire rom-full-reste ; MAILBOX.md 2026-10-03 |
| 0x3f8a..0x3f91 | burst montant émis (114 bits + hl/hu dans les 2 bits bas du mot 7), copié par PROM0 0x8900..0x891b | 1441, 85d3 | 13324, 8916 | idem |
| 0x3f9b..0x3fa2 | flux A5 montant XORé sur 0x3f8a (PROM0 0x85d1) | 1441, 85d4 | 37225, b185 | `src/pont.c:570` |
| 0x3f93..0x3f9a | flux A5 descendant | 11875, 8575 | 37225, b185 | brief 2026-10-03 |
| 0x3f92 | bit 0x1000 = flux A5 prêt | 168751, b04c | 99947, b04c | brief |
| 0x3d9b | pointeur de lecture du circulaire 0x4280 | 1573, 8bd5 | 1577, 8820 | `src/pont.c:571` |
| 0x2bfa.. | codé natural-order / tampon de travail | 305283, a2c4 | 61427, 8ba8 | mémoire rom-montant-codage |
| 0x2bf8 | drapeau parité classe 1a | 338033, 8df0 | 84547, 8daf | brief |
| 0x3d90 | drapeaux qualité CHED (0x01 parité, 0x02 MA, 0x08 SD-min, 0x10 …) | 37455, 8758 | 29527, 8758 | brief |
| 0x2d42.. (50 mots) | valeurs SD | 16195, 9abe | 16164, 9aad | brief |
| 0x2c00..0x2c1f | métriques Viterbi | 2679012, a2c4 | 398814, 8ba8 | brief |
| 0x3fad / 0x3fae | bits de mode (0x3fad bit 15 = verrou du répartiteur FB, PC 0x8753) | 343904 / 92658 | 27812 / 22517 | brief ; `c54x_mem.c` PROBE-3FAD-GATE |
| 0x3d89 | pointeur d'écriture du circulaire 232 mots en 0x4d00 (init 0x90c0) | 13281, 9169 | 10634, 90c0 | `c54x_mem.c` garde-3d89 |
| 0x43d8 | slot du gestionnaire courant (`ld *(0x43d8),A ; cala A` en 0xb01c) | 13218, b01c | 0 | `c54x_mem.c` HANDLER-WATCH |

Récolte plus large (MAILBOX.md, README.fr.md, sources) : `recolte_c54x.tsv` (brouillon, non
encore fusionné ici).

## 5. Ports I/O (mesurés : PORTR/PORTW 0x74xx/0x75xx du rejeu)

| port | sens | n | 1er PC | rôle | source |
|---|---|---|---|---|---|
| 0x0002 | W | 26436 | b572 | RIF SPCX (contrôle émission) | `calypso_rif.h:19` |
| 0x0003 | R/W | 81949 / 88202 | a675 / a67b | RIF SPCR (contrôle réception) | `calypso_rif.h:20` |
| 0x2800 | W | 147204 | b142 | A5 : commande (0 arrêt, 0x16/0x17) | `calypso_a5.c:39` |
| 0x2801, 0x2802 | R | 36801 | b1a0, b16c | A5 : trace, état | `calypso_a5.c:40-41` |
| 0x2803..0x2806 | W | 36801 | b145.. | A5 : Kc (= a_kc[0..3]) | `calypso_a5.c:42` |
| 0x2807, 0x2808 | W | 36801 | b151, b154 | A5 : a_a5fn[0] (T3<<5\|T2), a_a5fn[1] (T1) | `calypso_a5.c:13-15` |
| 0x2809..0x2818 | R | 36801 | b1ab.. | A5 : flux de clé (descendant puis montant) | `calypso_a5.c:45` |
| 0xf900 | W | 232373 | a6c3 | API Control (CAL207 7.2.2) | `calypso_xio.c:38` |
| 0xfa01 | W | 18809 | a658 | INTH du DSP | `calypso_xio.c` |
| 0xfc24..0xfc28 | R/W | ~12500..31315 | a5e8.. | fenêtre DMA RHEA (AAD 0x099c = tampon 0x0cce) | `calypso_rhea_dma.c` |

Vecteurs d'interruption (`calypso_c54x.h:162-191`) : TPU frame vec 28 / IMR bit 12, API vec 25 /
bit 9, DMA vec 30 / bit 14, RIF RX vec 16 / bit 0, A5 (CRYPT) vec 27 / bit 11.

## 6. Incohérences relevées (layer1 ↔ qosmo ↔ c54x_exe)

1. **`calypso_bsp_tx_burst()` lit `data[0x0900 + i]`** (`qosmo/.../calypso_bsp.c:3572`) : 0x0900 est
   dans le NDB (a_sch26[2..]), pas un tampon d'émission. Le burst émis par la ROM est en 0x3f8a
   (déjà connu ; c54x_exe le prend là via `pont.c tx_rom_publier`).
2. **Même nom, unités différentes** : `NDB_D_DSP_PAGE`/`NDB_D_FB_DET` = offsets d'**octets** dans le
   NDB (0x000/0x048, `include/hw/arm/calypso/calypso_api.h`) et adresses **mot DSP absolues**
   (0x08D4/0x08F8, `l1-dsp/calypso_fbsb.h:56-63`). Valeurs justes chacune dans sa convention ;
   une unité qui inclurait les deux en-têtes aurait une redéfinition silencieuse (à vérifier).
3. **B_BLUD** : position de bit (15) côté layer1, masque (1u<<15) côté qosmo — même nom.
4. **CALYPSO_API_SIZE = 64 Kio** (32768 mots, `calypso_api.h`) contre 16 Kio réels (API_SIZE 0x2000
   mots, `firmware/calypso/dsp.c:45`) ; l'enregistrement `S` fait 32768 mots, le DSP n'en voit que
   0x2000. Aucune écriture ARM au-delà de 0x2000 dans le rejeu (0 mot).
5. **d_ctrl_tch** : `dsp.c:503` documente b_chan_type sur les bits 4..7, `dsp_api.h` sur 4..5 avec
   bit 6 = reset SACCH et bit 7 = vocodeur ON ; `dsp_load_tch_param(chan_type=SDCCH_8=4)` pose donc
   le bit 6. Valeurs réellement écrites dans le rejeu : à analyser.
6. **Parole montante TCH/F dans a_du_1** : `prim_tch.c:485` `tch_sub ? a_du_0 : a_du_1` (alors que la
   descente prend `tch_sub ? a_dd_1 : a_dd_0`, `prim_tch.c:322`) ; l'ARM écrit 0x096e (a_du_1), la ROM
   le lit, mais elle lit et écrit aussi a_du_0 (0x0a24) que l'ARM ne touche jamais. `montant.c:314`
   lit NDB_A_DU_1 : cohérent avec la layer1. Rôle exact de a_du_0 pour la ROM : à confirmer.
7. Tailles : sizeof ARM ≠ hôte (T_DB_MCU_TO_DSP 36/34, T_NDB 6356, T_PARAM 292 : bourrage de fin) ;
   offsets identiques, sans effet.
8. Recoupements **vérifiés justes** (à la main, contre l'ELF) : tous les WP_*/RP_*/NDB_* de
   `calypso_api.h`, API_VERSION/API_VERSION2 (= d_version_number1/2), API_BL_STATUS 0x0FFE,
   `calypso_fbsb.h` (d_dsp_page, d_dsp_state 0x08E2, d_fb_det, d_fb_mode, a_sync_demod[0..3]),
   `calypso_arm2dsp.c` A2D_DSP_PAGE_OFF 0x01A8, `calypso_bsp.c` D_RACH_DEFAULT_OFFSET 0x023A (= d_rach),
   `src/rejouer.c` W_PAGE/R_PAGE/NDB/PARAM/BL_*, `tools/rejeu_banc.c` (a_cd 0x1FC, a_fd 0x21A,
   a_dd_0 0x238, a_kc 0x2CE, d_a5mode 0x1CE), `src/montant.c` NDB_D_TCH_MODE 0x006, `dsp.c`
   SC_CHKSUM_VER (0x08DB = d_version_number2).

## 7. Reste à faire

* Finir `tools/layer1_tester.py` (seule la partie extraction est écrite) : recoupement automatique,
  balayage statique des fonctions layer1, jugements PASS/FAIL par champ et par fonction, génération
  de cette page.
* Analyser les valeurs ARM (d_ctrl_tch, d_fn, a_a5fn, page W vs d_dsp_page, écho W→R) — données
  prêtes dans la mesure et le journal.
* Fusionner la récolte (MAILBOX.md, README.fr.md, sources qosmo) dans la section 4.
