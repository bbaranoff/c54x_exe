/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * isa_test.c - replay the SPRU172C worked examples against the C54x core.
 *
 * Reads tools/isa_tests.txt (written by tools/isa_examples.py): for every
 * example the assembled words, the state before and the state after. Each
 * test runs on a fresh C54xState: registers and memory are set, the words are
 * placed at PC, ONE c54x_run(s, 1) executes the instruction, and every
 * register/memory word the manual lists in "After Instruction" is compared.
 *
 *     make isa_test && ./isa_test tools/isa_tests.txt 2>/dev/null
 *     ./isa_test tools/isa_tests.txt -v 2>/dev/null      # print every test
 *
 * The verdict is a scorecard per instruction, not a debugging aid: a FAIL
 * names the register that differs and both values, then the core's handler is
 * read against the manual page. Registers the manual does not list are not
 * compared, so a handler that corrupts an unlisted register can still pass.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "qemu/thread.h"
#include "calypso_c54x.h"

/* what main.c normally provides */
uint32_t g_c54x_exe_fn;
uint32_t calypso_trx_get_fn(void) { return g_c54x_exe_fn; }
void calypso_inth_arm_ack(void) { }
QemuMutex calypso_pcb_daram_lock;
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool wr)
{ (void)addr; if (!wr) memset(buf, 0, len); }

extern uint16_t data_read(C54xState *s, uint16_t addr);
extern void     data_write(C54xState *s, uint16_t addr, uint16_t val);

#define ST0_TC_B  (1u << 12)
#define ST0_C_B   (1u << 11)
#define ST0_OVA_B (1u << 10)
#define ST0_OVB_B (1u << 9)

static struct { const char *nom; int reg; uint16_t bit; } BITS[] = {
    { "TC", 0, ST0_TC_B }, { "C", 0, ST0_C_B }, { "OVA", 0, ST0_OVA_B }, { "OVB", 0, ST0_OVB_B },
    { "BRAF", 1, 1u << 15 }, { "CPL", 1, 1u << 14 }, { "XF", 1, 1u << 13 }, { "HM", 1, 1u << 12 },
    { "INTM", 1, 1u << 11 }, { "OVM", 1, 1u << 9 }, { "SXM", 1, 1u << 8 }, { "C16", 1, 1u << 7 },
    { "FRCT", 1, 1u << 6 }, { "CMPT", 1, 1u << 5 },
};

static bool set_reg(C54xState *s, const char *r, uint64_t v)
{
    if (!strcmp(r, "A")) { s->a = (int64_t)(v & 0xFFFFFFFFFFULL); if (s->a & 0x8000000000LL) s->a |= ~0xFFFFFFFFFFLL; return true; }
    if (!strcmp(r, "B")) { s->b = (int64_t)(v & 0xFFFFFFFFFFULL); if (s->b & 0x8000000000LL) s->b |= ~0xFFFFFFFFFFLL; return true; }
    if (!strcmp(r, "T"))   { s->t = v; return true; }
    if (!strcmp(r, "TS"))  { s->t = v; return true; }
    if (!strcmp(r, "TRN")) { s->trn = v; return true; }
    if (!strcmp(r, "SP"))  { s->sp = v; return true; }
    if (!strcmp(r, "BK"))  { s->bk = v; return true; }
    if (!strcmp(r, "BRC")) { s->brc = v; return true; }
    if (!strcmp(r, "RSA")) { s->rsa = v; return true; }
    if (!strcmp(r, "REA")) { s->rea = v; return true; }
    if (!strcmp(r, "PC"))  { s->pc = v; return true; }
    if (!strcmp(r, "XPC")) { s->xpc = v; return true; }
    if (!strcmp(r, "PMST")) { s->pmst = v; return true; }
    if (!strcmp(r, "ST0")) { s->st0 = v; return true; }
    if (!strcmp(r, "ST1")) { s->st1 = v; return true; }
    if (!strcmp(r, "IMR")) { s->imr = v; return true; }
    if (!strcmp(r, "IFR")) { s->ifr = v; return true; }
    if (!strcmp(r, "ASM")) { s->st1 = (s->st1 & ~0x1F) | (v & 0x1F); return true; }
    if (!strcmp(r, "DP"))  { s->st0 = (s->st0 & ~0x1FF) | (v & 0x1FF); return true; }
    if (!strcmp(r, "ARP")) { s->st0 = (s->st0 & ~0xE000) | ((v & 7) << 13); return true; }
    if (r[0] == 'A' && r[1] == 'R' && r[2] >= '0' && r[2] <= '7' && !r[3]) { s->ar[r[2] - '0'] = v; return true; }
    for (unsigned i = 0; i < sizeof BITS / sizeof BITS[0]; i++)
        if (!strcmp(r, BITS[i].nom)) {
            uint16_t *st = BITS[i].reg ? &s->st1 : &s->st0;
            if (v) *st |= BITS[i].bit; else *st &= ~BITS[i].bit;
            return true;
        }
    return false;
}

static bool get_reg(C54xState *s, const char *r, uint64_t *v)
{
    if (!strcmp(r, "A")) { *v = (uint64_t)s->a & 0xFFFFFFFFFFULL; return true; }
    if (!strcmp(r, "B")) { *v = (uint64_t)s->b & 0xFFFFFFFFFFULL; return true; }
    if (!strcmp(r, "T") || !strcmp(r, "TS")) { *v = s->t; return true; }
    if (!strcmp(r, "TRN")) { *v = s->trn; return true; }
    if (!strcmp(r, "SP"))  { *v = s->sp; return true; }
    if (!strcmp(r, "BK"))  { *v = s->bk; return true; }
    if (!strcmp(r, "BRC")) { *v = s->brc; return true; }
    if (!strcmp(r, "RSA")) { *v = s->rsa; return true; }
    if (!strcmp(r, "REA")) { *v = s->rea; return true; }
    if (!strcmp(r, "PC"))  { *v = s->pc & 0xFFFF; return true; }
    if (!strcmp(r, "XPC")) { *v = s->xpc; return true; }
    if (!strcmp(r, "PMST")) { *v = s->pmst; return true; }
    if (!strcmp(r, "ST0")) { *v = s->st0; return true; }
    if (!strcmp(r, "ST1")) { *v = s->st1; return true; }
    if (!strcmp(r, "IMR")) { *v = s->imr; return true; }
    if (!strcmp(r, "IFR")) { *v = s->ifr; return true; }
    if (!strcmp(r, "ASM")) { *v = s->st1 & 0x1F; return true; }
    if (!strcmp(r, "DP"))  { *v = s->st0 & 0x1FF; return true; }
    if (!strcmp(r, "ARP")) { *v = (s->st0 >> 13) & 7; return true; }
    if (r[0] == 'A' && r[1] == 'R' && r[2] >= '0' && r[2] <= '7' && !r[3]) { *v = s->ar[r[2] - '0']; return true; }
    for (unsigned i = 0; i < sizeof BITS / sizeof BITS[0]; i++)
        if (!strcmp(r, BITS[i].nom)) {
            uint16_t st = BITS[i].reg ? s->st1 : s->st0;
            *v = (st & BITS[i].bit) ? 1 : 0;
            return true;
        }
    return false;
}

typedef struct { char reg[8]; uint64_t val; int mem; uint16_t addr; } Item;

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s tools/isa_tests.txt [-v] [-k MOT]\n", argv[0]); return 2; }
    bool verbeux = false; const char *filtre = NULL;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "-v")) verbeux = true;
        else if (!strcmp(argv[i], "-k") && i + 1 < argc) filtre = argv[++i];
    }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 2; }
    qemu_mutex_init(&calypso_pcb_daram_lock);

    char ligne[1024], titre[1024] = "";
    uint16_t mots[4]; int nmots = 0;
    Item avant[64], apres[64]; int na = 0, np = 0;
    int total = 0, ok = 0, ko = 0, ignores = 0;
    char bilan_ko[65536] = "";

    while (fgets(ligne, sizeof ligne, f)) {
        ligne[strcspn(ligne, "\n")] = 0;
        if (ligne[0] == 'T') {
            snprintf(titre, sizeof titre, "%s", ligne + 2);
            nmots = na = np = 0;
        } else if (ligne[0] == 'W') {
            char *p = ligne + 2; nmots = 0;
            while (*p && nmots < 4) { mots[nmots++] = (uint16_t)strtoul(p, &p, 16); while (*p == ' ') p++; }
        } else if (ligne[0] == 'B' || ligne[0] == 'A') {
            Item *it = ligne[0] == 'B' ? &avant[na] : &apres[np];
            int *n = ligne[0] == 'B' ? &na : &np;
            if (*n >= 64) continue;
            char k[8], r[8]; unsigned long long a, v;
            if (sscanf(ligne + 2, "%7s %llx %llx", k, &a, &v) == 3 && (k[0] == 'M' || k[0] == 'P') && !k[1]) {
                it->mem = k[0] == 'M' ? 1 : 2; it->addr = (uint16_t)a; it->val = v; it->reg[0] = 0; (*n)++;
            } else if (sscanf(ligne + 2, "%7s %llx", r, &v) == 2) {
                it->mem = 0; snprintf(it->reg, sizeof it->reg, "%s", r); it->val = v; (*n)++;
            }
        } else if (ligne[0] == 'E') {
            if (filtre && !strstr(titre, filtre)) continue;
            total++;
            C54xState *s = c54x_init();
            s->running = true;
            s->pc = 0x9000;
            bool prob = false;
            for (int i = 0; i < na; i++) {
                if (avant[i].mem == 1) { if (avant[i].addr < 0x60) data_write(s, avant[i].addr, (uint16_t)avant[i].val); else s->data[avant[i].addr] = (uint16_t)avant[i].val; }
                else if (avant[i].mem == 2) s->prog[avant[i].addr] = (uint16_t)avant[i].val;
                else if (!set_reg(s, avant[i].reg, avant[i].val)) prob = true;
            }
            uint16_t pc0 = (uint16_t)s->pc;
            for (int i = 0; i < nmots; i++) s->prog[(uint16_t)(pc0 + i)] = mots[i];
            /* the instruction after, so a delayed branch has something to execute */
            s->prog[(uint16_t)(pc0 + nmots)] = 0xF495; s->prog[(uint16_t)(pc0 + nmots + 1)] = 0xF495;
            c54x_run(s, 1);
            /* RPT arms a counter and returns 0 executed words: run the repeated
             * instruction as well so the example's "After" is observable. */
            if (s->rpt_active) c54x_run(s, 1);
            char detail[2048] = ""; int d = 0; bool echec = false;
            for (int i = 0; i < np; i++) {
                uint64_t got;
                if (apres[i].mem == 1) got = apres[i].addr < 0x60 ? data_read(s, apres[i].addr) : s->data[apres[i].addr];
                else if (apres[i].mem == 2) got = s->prog[apres[i].addr];
                else if (!get_reg(s, apres[i].reg, &got)) { prob = true; continue; }
                if (got != apres[i].val) {
                    echec = true;
                    if (apres[i].mem) d += snprintf(detail + d, sizeof detail - d, "  %s[%04x]: attendu %04llx, obtenu %04llx",
                                                    apres[i].mem == 1 ? "data" : "prog", apres[i].addr,
                                                    (unsigned long long)apres[i].val, (unsigned long long)got);
                    else d += snprintf(detail + d, sizeof detail - d, "  %s: attendu %llx, obtenu %llx",
                                       apres[i].reg, (unsigned long long)apres[i].val, (unsigned long long)got);
                }
            }
            if (echec) {
                ko++;
                printf("FAIL  %-60.60s  [%04x%s%04x%s]%s\n", titre, mots[0], nmots > 1 ? " " : "", nmots > 1 ? mots[1] : 0, prob ? " ?" : "", detail);
                size_t l = strlen(bilan_ko);
                snprintf(bilan_ko + l, sizeof bilan_ko - l, "  %s\n", titre);
            } else {
                ok++;
                if (verbeux) printf("ok    %-60.60s  [%04x]\n", titre, mots[0]);
            }
            free(s);
        } else if (ligne[0] == 'S') {
            ignores++;
        }
    }
    printf("\n%d exemples : %d ok, %d FAIL, %d non assembles (S)\n", total, ok, ko, ignores);
    return ko ? 1 : 0;
}
