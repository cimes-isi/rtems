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
#include <bsp/dma.h>

const char rtems_test_name[] = "HPSC DMA Copy To Register";

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

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
  DMA_Channel_t dma_channel;
  status = dma_channel_alloc(&dma_config, &dma_channel, 0);
  directive_failed(status, "DMA Channel 0 Allocation");

  /* Execute memory copy */
  uint64_t inbuf = 0x1234567890abcdef;
  uint32_t outreg = 0xffffffff;
  status = dma_copy_memory_to_register(&dma_channel, &outreg, &inbuf, sizeof(inbuf), 8 /* bits wide */);
  directive_failed(status, "DMA Channel 0 Copy");

  /* Wait for the channel to finish executing */
  bool stopped = false;
  bool faulted = false;
  while (!stopped && !faulted) {
    /* Get the current execution status of the DMA channel */
    status = dma_channel_is_stopped(&dma_channel, &stopped);
    directive_failed(status, "DMA Channel 0 Status Retrieval");
    status = dma_channel_is_faulted(&dma_channel, &faulted);
    directive_failed(status, "DMA Channel 0 Status Retrieval");
  }
  
  /* Check whether there was a fault during program execution */
  fatal_int_service_status(faulted, 0, "DMA Channel 0 Program Execution Fault");

  /* Check that outreg was set correctly by the DMA controller */
  fatal_int_service_status(outreg, 0xffffff12, "DMA Channel 0 Register Destination Value");

  /* Free the DMA channel */
  status = dma_channel_free(&dma_channel);
  directive_failed(status, "DMA Channel 0 Deallocation");

  /* Uninitialize DMA controller */
  status = dma_uninit(&dma_config);
  directive_failed(status, "DMA Uninitialization");
  TEST_END();
  rtems_test_exit( 0 );
}
