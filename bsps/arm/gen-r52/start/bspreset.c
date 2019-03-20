/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/bootcard.h>

void bsp_reset(void)
{
  /* TODO(kmoore) For the time being, there is no way to cause the CPU to reset
   * This causes a MPU fault which is buggy in QEMU and causes it to crash */
  __asm__ __volatile__("b #0x110000\n");
  /*__asm__ __volatile__("mov r1, #2\n"
                       "mcr p15, 4, r1, c12, c0, 2\n");*/
}
