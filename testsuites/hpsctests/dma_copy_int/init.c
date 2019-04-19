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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define CONFIGURE_INIT
#include "system.h"
#include "tmacros.h"
#include <stdio.h>
#include <bsp/hpsc-irqs.dtsh>
#include <bsp/dma.h>
#include <bsp/irq.h>

const char rtems_test_name[] = "HPSC DMA Copy with Interrupts";

volatile bool stopped = false;
DMA_Channel_t dma_channel;
volatile rtems_status_code failed_in_isr = RTEMS_SUCCESSFUL;
static void DMA_isr(void *arg)
{
  rtems_status_code status;

  /* Mark the transfer as complete */
  stopped = true;

  /* Check interrupt status */
  bool channel_int_status = false;
  status = dma_channel_int_status_get(&dma_channel, &channel_int_status);
  if (status != RTEMS_SUCCESSFUL) {
    failed_in_isr = status;
    return;
  }
  /* Expect interrupt high here */
  if (channel_int_status != 1) {
    failed_in_isr = RTEMS_INTERNAL_ERROR;
    return;
  }

  /* Reset interrupt status */
  status = dma_channel_int_clear(&dma_channel);
  failed_in_isr = status;
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

  status = rtems_interrupt_handler_install(
    BSP_DMA_CHANNEL_0_IRQ,
    "DMA Channel 0",
    RTEMS_INTERRUPT_UNIQUE,
    (rtems_interrupt_handler) DMA_isr,
    NULL
  );
  directive_failed(status, "DMA Channel 0 ISR Installation");

  /* Verify DMA controller supports any channels at all */
  if ( BSP_DMA_MAX_CHANNELS == 0 ) {
    printf( "\nBSP_DMA_MAX_CHANNELS Check FAILED -- expected (>0) got (0)\n");
    rtems_test_exit( RTEMS_INVALID_NUMBER );
  }

  /* Initialize DMA controller */
  DMA_Config_t dma_config;
  status = dma_init(&dma_config, BSP_DMA_BASE);
  directive_failed(status, "DMA Initialization");

  /* Allocate DMA channel resource */
  status = dma_channel_alloc(&dma_config, &dma_channel, 0);
  directive_failed(status, "DMA Channel 0 Allocation");

  /* Check interrupt status */
  bool channel_int_status = false;
  status = dma_channel_int_status_get(&dma_channel, &channel_int_status);
  directive_failed(status, "DMA Channel 0 Interrupt Status Retrieval");
  fatal_int_service_status(channel_int_status, 0, "DMA Channel 0 Interrupt status");

  /* Execute memory copy */
  uint32_t inbuf = 0x12345678, outbuf = 0xffffffff;
  status = dma_copy_memory_to_memory(&dma_channel, &outbuf, &inbuf, sizeof(inbuf));
  directive_failed(status, "DMA Channel 0 Copy");

  /* Wait for the channel to finish executing */
  bool faulted = false;
  while (!stopped && !faulted) {
    /* Get the current faulted status of the DMA channel */
    status = dma_channel_is_faulted(&dma_channel, &faulted);
    directive_failed(status, "DMA Channel 0 Status Retrieval");
    directive_failed(failed_in_isr, "DMA Channel 0 ISR Failure");
  }
  
  /* Check whether there was a fault during program execution */
  fatal_int_service_status(faulted, 0, "DMA Channel 0 Program Execution Fault");

  /* Check interrupt status, should be reset to 0 */
  status = dma_channel_int_status_get(&dma_channel, &channel_int_status);
  directive_failed(status, "DMA Channel 0 Interrupt Status Retrieval");
  fatal_int_service_status(channel_int_status, 0, "DMA Channel 0 Interrupt status");

  /* Check that outbuf was set correctly by the DMA controller */
  fatal_int_service_status(outbuf, 0x12345678, "DMA Channel 0 Memory Destination Value");

  /* Free the DMA channel */
  status = dma_channel_free(&dma_channel);
  directive_failed(status, "DMA Channel 0 Deallocation");

  /* Uninitialize DMA controller */
  status = dma_uninit(&dma_config);
  directive_failed(status, "DMA Uninitialization");
  TEST_END();
  rtems_test_exit( 0 );
}
