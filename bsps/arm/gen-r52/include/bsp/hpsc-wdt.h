/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (C) 2019 On-Line Applications Research Corporation (OAR)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __WDT_H__
#define __WDT_H__

#include <stdint.h>
#include <stdbool.h>
#include <bsp/irq.h>

/*!
 * \addtogroup WDT_CSR Watchdog Timer API for Configuration, Control, and Status
 *
 * This API provides functions for configuration, control, and status queries
 * of the Watchdog Timer.
 *
 * @{
 */

struct WDT_Base;

struct HPSC_WDT_Config {
  volatile struct WDT_Base *base;
  const char *name;
  rtems_vector_number vec;
  bool monitor;
  uint32_t clk_freq_hz;
  unsigned max_div;
  unsigned counter_width;
};

/*!
 * Initialize the watchdog monitor.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be initialized.
 *
 * \param       name
 *              A string describing the watchdog instance. Assumed static and will
 *              not be freed.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the watchdog
 *              timer.
 *
 * \param       vec
 *              The IRQ vector number for this watchdog timer.
 *
 * \param       clk_freq_hz
 *              The watchdog timer clock frequency in hertz.
 *
 * \param       max_div
 *              The watchdog timer maximum clock divider.
 */
void wdt_init_monitor(struct HPSC_WDT_Config *wdt_config, const char *name, volatile uint32_t *base, rtems_vector_number vec, uint32_t clk_freq_hz, unsigned max_div);

/*!
 * Configure the watchdog monitor.
 *
 * Note: Only monitors are configured.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be initialized.
 *
 * \param       freq
 *              The operating frequency of this WDT. Must be an integer divisor of the specified monitor clock frequency.
 *
 * \param       num_stages
 *              The number of stages to use on this watchdog timer.
 *
 * \param       timeouts
 *              The timeouts to be used for each stage of the timer.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 * \retval      RTEMS_INVALID_NUMBER The provided frequency, number of stages, or timeouts were invalid.
 */
rtems_status_code wdt_configure(struct HPSC_WDT_Config *wdt_config, unsigned freq, unsigned num_stages, uint64_t *timeouts);

/*!
 * Disable a watchdog timer.
 *
 * Note: Only a monitor can disable a timer once enabled.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be disabled.
 */
void wdt_disable(struct HPSC_WDT_Config *wdt_config);

/*!
 * Initialize the watchdog target.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be initialized.
 *
 * \param       name
 *              A string describing the watchdog instance. Assumed static and will
 *              not be freed.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the watchdog
 *              timer.
 *
 * \param       vec
 *              The IRQ vector number for this watchdog timer.
 */
void wdt_init_target(struct HPSC_WDT_Config *wdt_config, const char *name, volatile uint32_t *base, rtems_vector_number vec);

/*!
 * Uninitialize the watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be uninitialized.
 */
void wdt_uninit(struct HPSC_WDT_Config *wdt_config);

/*!
 * Check status of a watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be checked.
 *
 * \retval      true  Watchdog timer is enabled.
 * \retval      false Watchdog timer is disabled.
 */
bool wdt_is_enabled(struct HPSC_WDT_Config *wdt_config);

/*!
 * Enable a watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure to be enabled.
 */
void wdt_enable(struct HPSC_WDT_Config *wdt_config);

/*!
 * @}
 */

/*!
 * \addtogroup WDT_CSR Watchdog Timer API for Configuration, Control, and Status
 *
 * This API provides functions for configuration, control, and status queries
 * of the Watchdog Timer.
 *
 * @{
 */

/*!
 * Get the current count for the specified stage of a watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 *
 * \param       stage
 *              The stage for which to retrieve the timeout.
 *
 * \retval      The count for the given stage.
 */
uint64_t wdt_count(struct HPSC_WDT_Config *wdt_config, unsigned stage);

/*!
 * Get the configured terminal timeout for the specified stage of a watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 *
 * \param       stage
 *              The stage for which to retrieve the timeout.
 *
 * \retval      The configured timeout for the given stage.
 */
uint64_t wdt_timeout(struct HPSC_WDT_Config *wdt_config, unsigned stage);

/*!
 * Clear the timeout interrupt for the specified stage of a watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 *
 * \param       stage
 *              The stage for which to retrieve the timeout.
 */
void wdt_timeout_clear(struct HPSC_WDT_Config *wdt_config, unsigned stage);

/*!
 * Install the given interrupt handler for the given watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 *
 * \param       cb
 *              The interrupt handler.
 *
 * \param       cb_arg
 *              The argument to be provided to the interrupt handler.
 *
 * \retval      RTEMS_SUCCESSFUL        Successful operation.
 * \retval      RTEMS_CALLED_FROM_ISR   If this function is called from interrupt context this shall be returned.
 * \retval      RTEMS_INVALID_ADDRESS   If the handler address is NULL this shall be returned.
 * \retval      RTEMS_INVALID_ID        If the vector number is out of range this shall be returned.
 * \retval      RTEMS_INVALID_NUMBER    If an option is not applicable this shall be returned.
 * \retval      RTEMS_RESOURCE_IN_USE   If the vector is already occupied with a unique handler this shall be returned. If a unique handler should be installed and there is already a handler installed this shall be returned.
 * \retval      RTEMS_TOO_MANY          If a handler with this argument is already installed for the vector this shall be returned.
 * \retval      RTEMS_UNSATISFIED       If no handler exists to replace with the specified argument and vector this shall be returned.
 * \retval      RTEMS_IO_ERROR          Reserved for board support package specific error conditions.
 */
rtems_status_code wdt_handler_install(struct HPSC_WDT_Config *wdt_config, rtems_interrupt_handler cb, void *cb_arg);

/*!
 * Remove the given interrupt handler for the given watchdog timer.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 *
 * \param       cb
 *              The interrupt handler.
 *
 * \param       cb_arg
 *              The argument to be provided to the interrupt handler.
 *
 * \retval      RTEMS_SUCCESSFUL        Successful operation.
 * \retval      RTEMS_CALLED_FROM_ISR   If this function is called from interrupt context this shall be returned.
 * \retval      RTEMS_INVALID_ADDRESS   If the handler address is NULL this shall be returned.
 * \retval      RTEMS_INVALID_ID        If the vector number is out of range this shall be returned.
 * \retval      RTEMS_UNSATISFIED       If the handler with its argument is not installed for the vector this shall be returned.
 * \retval      RTEMS_IO_ERROR          Reserved for board support package specific error conditions.
 */
rtems_status_code wdt_handler_remove(struct HPSC_WDT_Config *wdt_config, rtems_interrupt_handler cb, void *cb_arg);

/*!
 * Clear the watchdog timer stages.
 *
 * \param       wdt_config
 *              A pointer to an allocated HPSC_WDT_Config structure.
 */
void wdt_kick(struct HPSC_WDT_Config *wdt_config);

/*!
 * @}
 */

#endif /* __WDT_H__ */
