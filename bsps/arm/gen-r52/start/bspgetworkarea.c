/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/bootcard.h>
#include <bsp/arm-cp15-start.h>
#include <bsp/linker-symbols.h>

#include <libcpu/arm-cp15.h>


#define AREA_COUNT_MAX 16

void bsp_work_area_initialize(void)
{
  Heap_Area areas[AREA_COUNT_MAX];
  size_t area_count;

  areas[0].begin = bsp_section_work_begin;
  areas[0].size = (uintptr_t) bsp_section_work_size;
  area_count = 1;

  bsp_work_area_initialize_with_table(areas, area_count);
}
