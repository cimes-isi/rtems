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
  __asm__ __volatile__("mov r1, #2\n"
                       "mcr p15, 4, r1, c12, c0, 2\n");
}
