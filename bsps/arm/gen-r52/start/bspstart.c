/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/bootcard.h>
#include <bsp/fatal.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>

#include <bsp/hwinfo.h>

#define R52_TIMER_INTID (GIC_NR_SGIS + PPI_IRQ__TIMER_PHYS)

#define GPT_FREQ_HZ 1000000000
void arm_generic_timer_get_config(
  uint32_t *frequency,
  uint32_t *irq
)
{
  *frequency = GPT_FREQ_HZ;
  *irq = R52_TIMER_INTID;
}

void bsp_start(void)
{
  bsp_interrupt_initialize();
}
