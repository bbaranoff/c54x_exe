# c54x_exe - le DSP Calypso hors QEMU.
#
# Les sources viennent de /opt/GSM/qosmo et ne sont PAS recopiees ici.
QOSMO   ?= /opt/GSM/qosmo
CAL     := $(QOSMO)/hw/arm/calypso
L1DSP   := $(CAL)/l1-dsp
HORS    := $(QOSMO)/contrib/hors-qemu

CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Werror=format -Werror=format-extra-args -Wno-unused-function -Wno-unused-variable \
           -Wno-unused-but-set-variable -Wno-sign-compare
CPPFLAGS := -D_GNU_SOURCE -I$(HORS)/doublures -I$(L1DSP) -I$(CAL) -I$(QOSMO)/include -I$(QOSMO)
# calypso_bsp.c encode les bursts RACH/NB : gsm0503_rach_ext_encode
OSMO    := $(shell pkg-config --cflags --libs libosmocoding libosmocore 2>/dev/null)
LDLIBS  := -lpthread -lm $(OSMO)

SRC := src/main.c src/rejouer.c src/pcb-minimal.c src/verbosite.c src/pont.c src/cellule.c src/montant.c $(HORS)/cales-qemu.c \
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
       $(L1DSP)/calypso_twl3025.c \
       $(CAL)/calypso_xio.c \
       $(CAL)/calypso_iota.c \
       $(CAL)/calypso_trf6151.c \
       $(CAL)/calypso_debug.c \
       $(CAL)/calypso_invariants.c

all: c54x_exe

c54x_exe: $(SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $(SRC) $(LDLIBS)

# ISA conformance: the SPRU172C worked examples replayed on the core.
#   make isa_test && ./isa_test tools/isa_tests.txt 2>/dev/null
COEUR := $(filter-out src/%,$(SRC))
tools/isa_tests.txt: tools/isa_examples.py
	python3 tools/isa_examples.py > $@

isa_test: tools/isa_test.c src/pcb-minimal.c $(COEUR) tools/isa_tests.txt
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ tools/isa_test.c src/pcb-minimal.c $(COEUR) $(LDLIBS)

clean:
	rm -f c54x_exe isa_test

.PHONY: all clean
