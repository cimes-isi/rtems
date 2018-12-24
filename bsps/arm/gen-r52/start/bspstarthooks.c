/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include <bsp/start.h>
#include <bsp/arm-cp15-start.h>
#include <bsp/arm-a9mpcore-start.h>
#include <bsp/linker-symbols.h>
#include "../../shared/cache/cacheimpl.h"

BSP_START_TEXT_SECTION void bsp_start_hook_0( void )
{
  // XXX TCM enabling goes here
}

BSP_START_TEXT_SECTION void bsp_start_hook_1( void )
{
  arm_cp15_set_vector_base_address(bsp_vector_table_begin);

  uint32_t ctrl = arm_cp15_get_control();
  ctrl &= ~ARM_CP15_CTRL_V;
  arm_cp15_set_control(ctrl);

  bsp_start_copy_sections();
  bsp_start_clear_bss();
}
