# c54x_exe - the Calypso DSP outside QEMU.
#
# The sources come from /opt/GSM/qosmo and are NOT copied here.
QOSMO   ?= /opt/GSM/qosmo
CAL     := $(QOSMO)/hw/arm/calypso
L1DSP   := $(CAL)/l1-dsp
HORS    := $(QOSMO)/contrib/hors-qemu

CC      ?= gcc
CFLAGS  ?= -O3 -march=native -g -Wall -Werror=format -Werror=format-extra-args -Wno-unused-function -Wno-unused-variable \
           -Wno-unused-but-set-variable -Wno-sign-compare
CPPFLAGS := -D_GNU_SOURCE -I$(HORS)/doublures -I$(L1DSP) -I$(CAL) -I$(QOSMO)/include -I$(QOSMO)
# calypso_bsp.c encodes the RACH/NB bursts: gsm0503_rach_ext_encode
OSMO    := $(shell pkg-config --cflags --libs libosmocoding libosmocore 2>/dev/null)
LDLIBS  := -lpthread -lm $(OSMO)

SRC := src/main.c src/rejouer.c src/pcb-minimal.c src/verbosite.c src/pont.c src/cellule.c src/montant.c src/tsp_tx.c src/multi.c $(HORS)/cales-qemu.c \
       $(L1DSP)/calypso_gmsk.c \
       $(L1DSP)/calypso_c54x.c \
       $(L1DSP)/c54x_exec.c \
       $(L1DSP)/c54x_decode.c \
       $(L1DSP)/c54x_mem.c \
       $(L1DSP)/c54x_irq.c \
       $(L1DSP)/c54x_probes.c \
       $(L1DSP)/calypso_bsp.c \
       $(L1DSP)/calypso_arm2dsp.c \
       $(L1DSP)/calypso_mailbox.c \
       $(L1DSP)/calypso_fbsb.c \
       $(L1DSP)/calypso_dma.c \
       $(L1DSP)/calypso_rhea_dma.c \
       $(L1DSP)/calypso_rif.c \
       $(L1DSP)/calypso_a5.c \
       $(L1DSP)/calypso_twl3025.c \
       $(L1DSP)/calypso_vsp.c \
       $(CAL)/calypso_xio.c \
       $(CAL)/calypso_iota.c \
       $(CAL)/calypso_trf6151.c \
       $(CAL)/calypso_debug.c \
       $(CAL)/calypso_invariants.c

all: c54x_exe

# Headers are dependencies too, so that a change to a .h alone
# (calypso_c54x.h, calypso_bsp.h...) rebuilds the binary.
HDR := $(wildcard src/*.h $(L1DSP)/*.h $(CAL)/*.h $(QOSMO)/include/hw/arm/calypso/*.h)

# REBUILT ON EVERY make. The sources come from another repository (qosmo),
# so c54x_exe is rebuilt on each call, as after a make clean, rather than
# trusting "make: Nothing to be done".
# A single cc command, no .o: nothing else to clean.
.PHONY: c54x_exe
c54x_exe: $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $(SRC) $(LDLIBS)

# ISA conformance: the SPRU172C worked examples replayed on the core.
#   make isa_test && ./isa_test tools/isa_tests.txt 2>/dev/null
COEUR := $(filter-out src/%,$(SRC))
tools/isa_tests.txt: tools/isa_examples.py
	python3 tools/isa_examples.py > $@

# Off-bench replay of a recorded TCH (tools/rejeu_banc.c): /dev/shm/calypso_rejeu_tch.bin
#   make tools/rejeu_banc && ./tools/rejeu_banc [file] [max ticks]
tools/rejeu_banc: tools/rejeu_banc.c src/tsp_tx.c src/pcb-minimal.c src/verbosite.c $(COEUR) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Isrc -o $@ tools/rejeu_banc.c src/tsp_tx.c src/pcb-minimal.c src/verbosite.c $(COEUR) $(LDLIBS)

isa_test: tools/isa_test.c src/pcb-minimal.c $(COEUR) tools/isa_tests.txt
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ tools/isa_test.c src/pcb-minimal.c $(COEUR) $(LDLIBS)

# layer1_tester: addresses and functions of the layer1 <-> ROM interface, map
# docs/carte-memoire.md. Does NOT rebuild c54x_exe. The core is linked WITHOUT
# calypso_mailbox.c: tools/layer1_tester.c provides calypso_mbx_evt, the hook
# of every DSP access; fopen/open/socket/sendto are wrapped (nothing is
# written outside the output file, no socket).
#   make layer1_tester && python3 tools/layer1_tester.py [--rejeu FILE] [--md docs/carte-memoire.md]
#   make api_carte [REJEU=FILE]         (alias: regenerates docs/carte-memoire.md)
L1T_COEUR := $(filter-out $(L1DSP)/calypso_mailbox.c,$(COEUR))
L1T_WRAP  := -Wl,--wrap=fopen,--wrap=open,--wrap=socket,--wrap=sendto,--wrap=calypso_a5_portw,--wrap=calypso_a5_portr
REJEU     ?=
tools/layer1_tester: tools/layer1_tester.c src/pcb-minimal.c $(L1T_COEUR) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ tools/layer1_tester.c src/pcb-minimal.c $(L1T_COEUR) $(LDLIBS) $(L1T_WRAP)

layer1_tester: tools/layer1_tester

api_carte: tools/layer1_tester
	python3 tools/layer1_tester.py $(if $(REJEU),--rejeu $(REJEU)) --md docs/carte-memoire.md

# DSP function tester (tools/dsp_tester.c): the osmocom-bb ARM and the BTS
# are simulated (tools/dsp_banc_commun.c), one PASS/FAIL/NON-IMPL line per
# DSP task. Does NOT rebuild c54x_exe.
#   make dsp_tester && ./dsp_tester --all
# The API RAM structures and the parameter table are those of the osmocom-bb
# firmware (dsp_api.h, dsp_params.c), read from its tree.
OSMOBB  ?= /opt/GSM/osmocom-bb/src/target/firmware
BANC_CPPFLAGS := -idirafter $(OSMOBB)/include/calypso -idirafter $(OSMOBB)/calypso
BANC    := tools/dsp_banc_commun.c src/pcb-minimal.c src/verbosite.c
dsp_tester: tools/dsp_tester.c $(BANC) tools/dsp_banc_commun.h $(COEUR) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Isrc $(BANC_CPPFLAGS) -o $@ tools/dsp_tester.c $(BANC) $(COEUR) $(LDLIBS)

clean:
	rm -f c54x_exe isa_test tools/layer1_tester dsp_tester

.PHONY: all clean layer1_tester api_carte
