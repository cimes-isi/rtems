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
#include <bsp/fdt.h>
#include <bsp/irq-generic.h>
#include <bsp/linker-symbols.h>

#include <bsp/hwinfo.h>

#define MAGIC_IRQ_OFFSET 32

uint32_t bsp_fdt_map_intr(const uint32_t *intr, size_t icells)
{
  return intr[1] + 32;
}

static void set_clock(
  const void *fdt,
  int parent,
  ALT_CLK_t clk,
  const char *name
)
{
  int node;
  int len;
  const uint32_t *val;

  node = fdt_subnode_offset(fdt, parent, name);
  val = fdt_getprop(fdt, node, "clock-frequency", &len);

  if (val != NULL && len >= 4) {
    alt_clk_ext_clk_freq_set(clk, fdt32_to_cpu(val[0]));
  }
}

static void set_clock_by_output_name(
  const void *fdt,
  ALT_CLK_t clk,
  const char *clock_output_name
)
{
  int node;
  int len;
  const uint32_t *val;

  node = fdt_node_offset_by_prop_value(
    fdt,
    -1,
    "clock-output-names",
    clock_output_name,
    strlen(clock_output_name) + 1
  );
  val = fdt_getprop(fdt, node, "clock-frequency", &len);

  if (val != NULL && len >= 4) {
    alt_clk_ext_clk_freq_set(clk, fdt32_to_cpu(val[0]));
  }
}

static void update_clocks(void)
{
  const void *fdt;
  int parent;

  fdt = bsp_fdt_get();

  /* Try to set by node name */
  parent = fdt_node_offset_by_compatible(fdt, -1, "altr,clk-mgr");
  parent = fdt_subnode_offset(fdt, parent, "clocks");
  set_clock(fdt, parent, ALT_CLK_OSC1, "osc1");
  set_clock(fdt, parent, ALT_CLK_IN_PIN_OSC2, "osc2");
  set_clock(fdt, parent, ALT_CLK_F2H_PERIPH_REF, "f2s_periph_ref_clk");
  set_clock(fdt, parent, ALT_CLK_F2H_SDRAM_REF, "f2s_sdram_ref_clk");

  /* Try to set by "clock-output-names" property value */
  set_clock_by_output_name(fdt, ALT_CLK_OSC1, "hps_0_eosc1-clk");
  set_clock_by_output_name(fdt, ALT_CLK_IN_PIN_OSC2, "hps_0_eosc2-clk");
  set_clock_by_output_name(fdt, ALT_CLK_F2H_PERIPH_REF, "hps_0_f2s_periph_ref_clk-clk");
  set_clock_by_output_name(fdt, ALT_CLK_F2H_SDRAM_REF, "hps_0_f2s_sdram_ref_clk-clk");
}

rtems_vector_number gen_r52_get_irq_of_node(
  const void *fdt,
  int node,
  size_t index
)
{
  int len;
  const uint32_t *val;

  val = fdt_getprop(fdt, node, "interrupts", &len);
  if (val == NULL || len < (int) ((index + 1) * 12)) {
    return UINT32_MAX;
  }

  return fdt32_to_cpu(val[index * 3 + 1]) + MAGIC_IRQ_OFFSET;
}

void arm_generic_timer_get_config(
  uint32_t *frequency,
  uint32_t *irq
)
{
  const void *fdt;
  int node;
  int len;
  const uint32_t *val;

  fdt = bsp_fdt_get();
  node = fdt_path_offset(fdt, "/timer");

  val = fdt_getprop(fdt, node, "clock-frequency", &len);
  if (val != NULL && len >= 4) {
    *frequency = fdt32_to_cpu(val[0]);
  } else {
    bsp_fatal(GEN_R52_FATAL_GENERIC_TIMER_FREQUENCY);
  }

  /* FIXME: Figure out how Linux gets a proper IRQ number */
  *irq = gen_r52_get_irq_of_node(fdt, node, 0) - 16;
}

void bsp_start(void)
{
  update_clocks();
  bsp_interrupt_initialize();
}
