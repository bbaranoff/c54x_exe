/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * The four DARAM helpers of calypso_full_pcb.c, copied verbatim.
 *
 * calypso_full_pcb.c as a whole does not build outside QEMU: it includes
 * hw/core/cpu.h and all of QEMU behind it. The helpers the DSP needs do not
 * depend on any of that - one mutex and two array accesses - so they are copied
 * word for word from qosmo-dsp/hw/arm/calypso/calypso_full_pcb.c.
 *
 * This is the only copy in this binary: if the original changes, this file lies.
 * It goes away once calypso_full_pcb.c is decoupled from C54xState.
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
