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
#include <bsp/fatal.h>
#include <bsp/arm-cp15-start.h>
#include <bsp/linker-symbols.h>
#include "../../shared/cache/cacheimpl.h"

#include <bsp/hwinfo.h>
#include <bsp/irq.h>
#include <bsp/hpsc-arch.h>
#include <bsp/fdt.h>
#include <libfdt.h>

static int32_t get_tcm_availability( void )
{
  int32_t value;
  __asm__ volatile(""
    "mrc    p15,0,%0,c0,c0,2"
    : "=r" (value)
  );
  return value;
}

static int32_t get_tcm_region_register( int which_tcm )
{
  int32_t value;
  switch (which_tcm)
  {
  case 0:
    __asm__ volatile(""
      "mrc    p15,0,%0,c9,c1,0"
      : "=r" (value)
    );
    break;
  case 1:
    __asm__ volatile(""
      "mrc    p15,0,%0,c9,c1,1"
      : "=r" (value)
    );
    break;
  case 2:
    __asm__ volatile(""
      "mrc    p15,0,%0,c9,c1,2"
      : "=r" (value)
    );
    break;
  default:
    return 0;
  }
  return value;
}

#define TCM_ADDRESS_MASK 0xfffff000
static void tcm_region_value_set_address( uint32_t *region_value, void *address )
{
  (*region_value) &= ~TCM_ADDRESS_MASK;
  (*region_value) |= (((uint32_t)address) & TCM_ADDRESS_MASK);
}

#define TCM_EL10_MASK 0x00000001
static void tcm_region_value_set_el10( uint32_t *region_value, bool el10_enabled )
{
  (*region_value) &= ~TCM_EL10_MASK;
  (*region_value) |= (el10_enabled * TCM_EL10_MASK);
}

#define TCM_EL2_MASK 0x00000002
static void tcm_region_value_set_el2( uint32_t *region_value, bool el2_enabled )
{
  (*region_value) &= ~TCM_EL2_MASK;
  (*region_value) |= (el2_enabled * TCM_EL2_MASK);
}

#define TCM_SIZE_BITS_MASK 0x7c
#define TCM_SIZE_NONE 0x0
#define TCM_SIZE_8K 0x4
#define TCM_SIZE_16K 0x5
#define TCM_SIZE_32K 0x6
#define TCM_SIZE_64K 0x7
#define TCM_SIZE_128K 0x8
#define TCM_SIZE_256K 0x9
#define TCM_SIZE_512K 0xa
#define TCM_SIZE_1M 0xb
static uint32_t tcm_region_value_get_size( uint32_t region_value )
{
  region_value &= TCM_SIZE_BITS_MASK;
  uint32_t size_bits = region_value >> 2;
/*  switch (size_bits) {
  case TCM_SIZE_NONE:
    return 0;
  case TCM_SIZE_8K:
    return 0x2000;
  case TCM_SIZE_16K:
    return 0x4000;
  case TCM_SIZE_32K:
    return 0x8000;
  case TCM_SIZE_64K:
    return 0x10000;
  case TCM_SIZE_128K:
    return 0x20000;
  case TCM_SIZE_256K:
    return 0x40000;
  case TCM_SIZE_512K:
    return 0x80000;
  case TCM_SIZE_1M:
    return 0x100000;
  default:
    return 0;
  }*/
  return 1 << (size_bits + 9);
}

static void set_tcm_region_register( int which_tcm, uint32_t register_value )
{
  switch (which_tcm)
  {
  case 0:
    __asm__ volatile(""
      "mcr    p15,0,%0,c9,c1,0  \n"
      :
      : "r" (register_value)
    );
    break;
  case 1:
    __asm__ volatile(""
      "mcr    p15,0,%0,c9,c1,1  \n"
      :
      : "r" (register_value)
    );
    break;
  case 2:
    __asm__ volatile(""
      "mcr    p15,0,%0,c9,c1,2  \n"
      :
      : "r" (register_value)
    );
    break;
  default:
    break;
  }
}

static void set_tcm_address ( int which_tcm, void *tcm_address )
{
  uint32_t region_register = get_tcm_region_register(which_tcm);
  tcm_region_value_set_address(&region_register, tcm_address);
  tcm_region_value_set_el10(&region_register, true);
  tcm_region_value_set_el2(&region_register, true);
  set_tcm_region_register(which_tcm, region_register);
}


#define TCM_ANY 0x80000000
#define TCMA_ENABLED 0x1
#define TCMB_ENABLED 0x2
#define TCMC_ENABLED 0x4
static void setup_tcms( void )
{
  uint32_t tcm_enabled = get_tcm_availability();
  /* Check which TCMs need to be enabled */
  if (!(tcm_enabled & TCM_ANY))
  {
    return;
  }

  void *tcm_address = 0x00000000;
  if (tcm_enabled & TCMA_ENABLED)
  {
    /* enable TCMA */
    uint32_t tcma_size = tcm_region_value_get_size(get_tcm_region_register(0));
    if (tcma_size)
    {
      set_tcm_address(0, tcm_address);
      tcm_address += tcma_size;
    }
  }
  if (tcm_enabled & TCMB_ENABLED)
  {
    /* enable TCMB */
    uint32_t region_register = get_tcm_region_register(1);
    uint32_t tcmb_size = tcm_region_value_get_size(region_register);
    if (tcmb_size)
    {
      set_tcm_address(1, tcm_address);
      tcm_address += tcmb_size;
    }
  }
  if (tcm_enabled & TCMC_ENABLED)
  {
    /* enable TCMC */
    uint32_t region_register = get_tcm_region_register(2);
    uint32_t tcmc_size = tcm_region_value_get_size(region_register);
    if (tcmc_size)
    {
      set_tcm_address(2, tcm_address);
      tcm_address += tcmc_size;
    }
  }
}

#include <libchip/ns16550.h>
extern ns16550_context gen_r52_uart_context_0;
void set_uart_params(void)
{
  const void *fdt = hpsc_arch_bin;
  int status;
  uint32_t size;
  int root;
  int nic4;
  int lsio_uart_0;
  int lsio_uart_1;
  fdt32_t *uart_1_reg;
  fdt32_t *uart_1_interrupt_map;
  int len;
  uint32_t uart_1_address;
  uint32_t uart_1_int_ppi;
  uint32_t uart_1_int_offset;

  status = fdt_check_header(fdt);
  if (status != 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  size = fdt_totalsize(fdt);
  if (size != hpsc_arch_bin_size) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  root = fdt_path_offset(fdt, "/");
  if (root < 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  nic4 = fdt_subnode_offset(fdt, root, "nic4");
  if (nic4 < 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  /* Searching for labels doesn't work because they don't get mapped in the device tree blob
   * Searching for serial@<address> requires prior knowledge of the address
   * Assume the 2nd serial entry in nic4 is what is desired
   * There is no way to search for the nth entry
   * Find the first serial entry
   */
  lsio_uart_0 = fdt_subnode_offset(fdt, nic4, "serial");
  if (lsio_uart_0 < 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  /* Jump to the next node */
  lsio_uart_1 = fdt_next_node(fdt, lsio_uart_0, NULL);
  if (lsio_uart_1 < 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }
  /* Ensure that the node is also a serial node */
  if (strncmp("serial", fdt_get_name(fdt, lsio_uart_1, NULL), strlen("serial")) != 0) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }

  uart_1_reg = (fdt32_t *) fdt_getprop(fdt, lsio_uart_1, "reg", &len);
  if (!uart_1_reg || len != 16) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }
  uart_1_address = fdt32_to_cpu(uart_1_reg[0]);
  gen_r52_uart_context_0.port = uart_1_address;

  uart_1_interrupt_map = (fdt32_t *) fdt_getprop(fdt, lsio_uart_1, "interrupt-map", &len);
  if (!uart_1_interrupt_map || len != 28) {
    bsp_fatal(GEN_R52_FATAL_UART_ADDRESS);
  }
  uart_1_int_ppi = fdt32_to_cpu(uart_1_interrupt_map[4]);
  uart_1_int_offset = fdt32_to_cpu(uart_1_interrupt_map[5]);
#define GIC_EDGE_RISE 1
#define GIC_EDGE_FALL 2
#define GIC_EDGE_BOTH 3
#define GIC_LVL_HI    4
#define GIC_LVL_LO    8
  /*uart_int_trigger = fdt32_to_cpu(uart_1_interrupt_map[6]);*/
  gen_r52_uart_context_0.irq = (uart_1_int_ppi ? GIC_NR_SGIS : GIC_INTERNAL) + uart_1_int_offset;
}

BSP_START_TEXT_SECTION void bsp_start_hook_0( void )
{
  /* Enable TCMs as necessary */
  setup_tcms();
  set_uart_params();
}

BSP_START_TEXT_SECTION void bsp_start_hook_1( void )
{
  arm_cp15_set_vector_base_address(bsp_start_vector_table_begin);

  uint32_t ctrl = arm_cp15_get_control();
  ctrl &= ~ARM_CP15_CTRL_V;
  arm_cp15_set_control(ctrl);

  bsp_start_copy_sections();
  bsp_start_clear_bss();
}
