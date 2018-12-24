/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <libchip/ns16550.h>

#include <rtems/bspIo.h>

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/console-termios.h>

#ifdef BSP_USE_UART_INTERRUPTS
  #define UART_HANDLER &ns16550_handler_interrupt
#else
  #define UART_HANDLER &ns16550_handler_polled
#endif

static uint8_t gen_r52_uart_get_register(uintptr_t addr, uint8_t i)
{
  volatile uint32_t *reg = (volatile uint32_t *) addr;

  return (uint8_t) reg [i];
}

static void gen_r52_uart_set_register(uintptr_t addr, uint8_t i, uint8_t val)
{
  volatile uint32_t *reg = (volatile uint32_t *) addr;

  reg [i] = val;
}

ns16550_context gen_r52_uart_context_0 = {
  .base = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART 0"),
  .get_reg = gen_r52_uart_get_register,
  .set_reg = gen_r52_uart_set_register,
  .port = (uintptr_t) LSIO_UART1_BASE,
  .irq = RTPS_IRQ__LSIO_UART1_0 + GIC_INTERNAL,
  .clock = 115200,
  .initial_baud = GEN_R52_UART_BAUD
};

#ifdef GEN_R52_CONFIG_UART_1
ns16550_context gen_r52_uart_context_1 = {
  .base = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("UART 1"),
  .get_reg = gen_r52_uart_get_register,
  .set_reg = gen_r52_uart_set_register,
  .port = (uintptr_t) LSIO_UART0_BASE,
  .clock = 115200,
  .initial_baud = GEN_R52_UART_BAUD
};
#endif

const console_device console_device_table[] = {
    {
      .device_file = "/dev/ttyS0",
      .probe = ns16550_probe,
      .handler = UART_HANDLER,
      .context = &gen_r52_uart_context_0.base
    },
  #ifdef GEN_R52_CONFIG_UART_1
    {
      .device_file = "/dev/ttyS1",
      .probe = ns16550_probe,
      /* QEMU Device Tree does not support interrupt-driven secondary UART */
      .handler = &ns16550_handler_polled,
      .context = &gen_r52_uart_context_1.base
    },
  #endif
};

const size_t console_device_count = RTEMS_ARRAY_SIZE(console_device_table);

static void output_char(char c)
{
  rtems_termios_device_context *ctx = console_device_table[0].context;

  ns16550_polled_putchar( ctx, c );
}

BSP_output_char_function_type BSP_output_char = output_char;

BSP_polling_getchar_function_type BSP_poll_char = NULL;
