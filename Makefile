# c54x_exe - le DSP Calypso hors QEMU.
#
# Les sources viennent de /opt/GSM/qosmo et ne sont PAS recopiees ici.
QOSMO   ?= /opt/GSM/qosmo
CAL     := $(QOSMO)/hw/arm/calypso
L1DSP   := $(CAL)/l1-dsp
HORS    := $(QOSMO)/contrib/hors-qemu

CC      ?= gcc
CFLAGS  ?= -O3 -march=native -g -Wall -Werror=format -Werror=format-extra-args -Wno-unused-function -Wno-unused-variable \
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
       $(L1DSP)/calypso_a5.c \
       $(L1DSP)/calypso_twl3025.c \
       $(CAL)/calypso_xio.c \
       $(CAL)/calypso_iota.c \
       $(CAL)/calypso_trf6151.c \
       $(CAL)/calypso_debug.c \
       $(CAL)/calypso_invariants.c

all: c54x_exe

# [2026-09-23] Les en-tetes aussi : sans eux, une modification d un .h seul
# (calypso_c54x.h, calypso_bsp.h...) laissait un binaire perime.
HDR := $(wildcard src/*.h $(L1DSP)/*.h $(CAL)/*.h $(QOSMO)/include/hw/arm/calypso/*.h)

# [2026-09-23] RECOMPILATION A CHAQUE make. Les sources viennent d un autre
# depot (qosmo) et « make: Nothing to be done » laissait douter du binaire
# lance : c54x_exe est reconstruit a chaque appel, comme apres un make clean.
# Une seule commande cc, pas de .o : rien d autre a nettoyer.
.PHONY: c54x_exe
c54x_exe: $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $(SRC) $(LDLIBS)

# ISA conformance: the SPRU172C worked examples replayed on the core.
#   make isa_test && ./isa_test tools/isa_tests.txt 2>/dev/null
COEUR := $(filter-out src/%,$(SRC))
tools/isa_tests.txt: tools/isa_examples.py
	python3 tools/isa_examples.py > $@

isa_test: tools/isa_test.c src/pcb-minimal.c $(COEUR) tools/isa_tests.txt
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ tools/isa_test.c src/pcb-minimal.c $(COEUR) $(LDLIBS)

# layer1_tester : adresses et fonctions de l'interface layer1 <-> ROM, carte
# docs/carte-memoire.md. Ne reconstruit PAS c54x_exe. Le coeur est lie SANS
# calypso_mailbox.c : tools/layer1_tester.c fournit calypso_mbx_evt, le crochet
# de chaque acces DSP ; fopen/open/socket/sendto sont enveloppes (rien n'est
# ecrit hors du fichier de sortie, aucune socket).
#   make layer1_tester && python3 tools/layer1_tester.py [--rejeu FICHIER] [--md docs/carte-memoire.md]
#   make api_carte [REJEU=FICHIER]      (alias : regenere docs/carte-memoire.md)
L1T_COEUR := $(filter-out $(L1DSP)/calypso_mailbox.c,$(COEUR))
L1T_WRAP  := -Wl,--wrap=fopen,--wrap=open,--wrap=socket,--wrap=sendto,--wrap=calypso_a5_portw,--wrap=calypso_a5_portr
REJEU     ?=
tools/layer1_tester: tools/layer1_tester.c src/pcb-minimal.c $(L1T_COEUR) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ tools/layer1_tester.c src/pcb-minimal.c $(L1T_COEUR) $(LDLIBS) $(L1T_WRAP)

layer1_tester: tools/layer1_tester

api_carte: tools/layer1_tester
	python3 tools/layer1_tester.py $(if $(REJEU),--rejeu $(REJEU)) --md docs/carte-memoire.md

# Testeur de fonctions du DSP (tools/dsp_tester.c) : l'ARM osmocom-bb et la BTS
# sont simules (tools/dsp_banc_commun.c), une ligne PASS/FAIL/NON-IMPL par
# tache DSP. Ne reconstruit PAS c54x_exe.
#   make dsp_tester && ./dsp_tester --all
# Les structures de l'API RAM et la table de parametres sont celles du firmware
# osmocom-bb (dsp_api.h, dsp_params.c), lues dans son arbre.
OSMOBB  ?= /opt/GSM/osmocom-bb/src/target/firmware
BANC_CPPFLAGS := -idirafter $(OSMOBB)/include/calypso -idirafter $(OSMOBB)/calypso
BANC    := tools/dsp_banc_commun.c src/pcb-minimal.c src/verbosite.c
dsp_tester: tools/dsp_tester.c $(BANC) tools/dsp_banc_commun.h $(COEUR) $(HDR)
	$(CC) $(CFLAGS) $(CPPFLAGS) -Isrc $(BANC_CPPFLAGS) -o $@ tools/dsp_tester.c $(BANC) $(COEUR) $(LDLIBS)

clean:
	rm -f c54x_exe isa_test tools/layer1_tester dsp_tester

.PHONY: all clean layer1_tester api_carte
