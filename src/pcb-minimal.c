/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Les quatre helpers DARAM de calypso_full_pcb.c, repris a l'identique.
 *
 * [2026-09-16] calypso_full_pcb.c entier n'est pas compilable hors QEMU : il
 * inclut hw/core/cpu.h, et tout QEMU derriere. Mais les helpers dont le DSP a
 * besoin n'en dependent pas - ce sont un verrou et deux acces tableau. Copies
 * mot pour mot depuis qosmo-dsp/hw/arm/calypso/calypso_full_pcb.c:163-181.
 *
 * C'est la SEULE copie de ce binaire, et elle est signalee comme telle : si
 * l'original change, cette copie ment. Elle disparaitra le jour ou
 * calypso_full_pcb.c sera decouple de C54xState - c'est le meme travail que
 * celui qui rendra les 9 devices de la carte E88 disponibles a la L1 gr-gsm.
 */
#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "calypso_c54x.h"

extern QemuMutex calypso_pcb_daram_lock;

uint16_t calypso_dsp_daram_read(void *dsp_void, uint16_t addr)
{
    C54xState *dsp = (C54xState *)dsp_void;
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    uint16_t v = dsp->data[addr];
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
    return v;
}

void calypso_dsp_daram_write(void *dsp_void, uint16_t addr, uint16_t val)
{
    C54xState *dsp = (C54xState *)dsp_void;
    qemu_mutex_lock(&calypso_pcb_daram_lock);
    dsp->data[addr] = val;
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}

void calypso_pcb_daram_lock_acquire(void)
{
    qemu_mutex_lock(&calypso_pcb_daram_lock);
}

void calypso_pcb_daram_lock_release(void)
{
    qemu_mutex_unlock(&calypso_pcb_daram_lock);
}
