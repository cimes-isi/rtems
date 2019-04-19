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

const char rtems_test_name[] = "HPSC DMA Copy HPSC Memory Bounds Check";

/* Checks read of a 64bit value from the provided address */
static rtems_status_code read_check_memory64(DMA_Config_t * dma_config, uint64_t * address)
{
  rtems_status_code status;

  /* Allocate DMA channel resource */
  DMA_Channel_t dma_channel;
  status = dma_channel_alloc(dma_config, &dma_channel, 0);
  if (status != RTEMS_SUCCESSFUL) {
    return status;
  }

  /* Execute memory copy */
  uint64_t outbuf = 0xaaaaaaaa;
  status = dma_copy_memory_to_memory(&dma_channel, &outbuf, address, sizeof(outbuf));
  if (status != RTEMS_SUCCESSFUL) {
    return status;
  }

  /* Wait for the channel to finish executing */
  bool stopped = false;
  bool faulted = false;
  while (!stopped && !faulted) {
    /* Get the current execution status of the DMA channel */
    status = dma_channel_is_stopped(&dma_channel, &stopped);
    if (status != RTEMS_SUCCESSFUL) {
      return status;
    }
    status = dma_channel_is_faulted(&dma_channel, &faulted);
    if (status != RTEMS_SUCCESSFUL) {
      return status;
    }
  }
  
  /* Check whether there was a fault during program execution */
  if (faulted) {
    return RTEMS_IO_ERROR;
  }

  /* Check that outbuf was set correctly by the DMA controller */
  if (outbuf != *address) {
    return RTEMS_IO_ERROR;
  }

  /* Free the DMA channel */
  status = dma_channel_free(&dma_channel);
  return status;
}

static rtems_status_code read_check_memory(DMA_Config_t * dma_config, char * address)
{
  return read_check_memory64(dma_config, (uint64_t *)address);
}

static rtems_status_code read_check_memory_end(DMA_Config_t * dma_config, char * address)
{
  address -= 8;
  return read_check_memory64(dma_config, (uint64_t *)address);
}

extern char bsp_section_text_begin[];
extern char bsp_section_text_end[];
extern char bsp_section_start_begin[];
extern char bsp_section_start_end[];
extern char bsp_section_data_begin[];
extern char bsp_section_data_end[];
extern char bsp_section_bss_begin[];
extern char bsp_section_bss_end[];
extern char bsp_section_stack_begin[];
extern char bsp_section_bss_end[];
extern char bsp_section_stack_begin[];
extern char bsp_section_stack_end[];
extern char __window_start__[];
extern char __window_end__[];
extern char __tcm_a_start__[];
extern char __tcm_a_end__[];
extern char __tcm_b_start__[];
extern char __tcm_b_end__[];
extern char __hpps_ddr_low_start__[];
extern char __hpps_ddr_low_end__[];
extern char __rtps_ddr_low_0_start__[];
extern char __rtps_ddr_low_0_end__[];
extern char __hpps_mbox_start__[];
extern char __hpps_mbox_end__[];

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

  /* Check RTPS Low DDR Block 0 */
  status = read_check_memory(&dma_config, (char *)0x40000000);
  directive_failed(status, "DMA Access to RTPS Low DDR Block 0 Bottom");
  status = read_check_memory_end(&dma_config, (char *)0x70000000);
  directive_failed(status, "DMA Access to RTPS Low DDR Block 0 Top");

  /* Check RTPS Low DDR Block 1 */
  status = read_check_memory(&dma_config, (char *)0x70000000);
  directive_failed(status, "DMA Access to RTPS Low DDR Block 1 Bottom");
  status = read_check_memory_end(&dma_config, (char *)0x7E000000);
  directive_failed(status, "DMA Access to RTPS Low DDR Block 1 Top");

  /* Check HPPS Low DDR Block 0 */
  status = read_check_memory(&dma_config, (char *)0x80000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 0 Bottom");
  status = read_check_memory(&dma_config, (char *)0x90000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 0 Middle");
  status = read_check_memory_end(&dma_config, (char *)0xA0000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 0 Top");

  /* Check HPPS Low DDR Block 1 */
  status = read_check_memory(&dma_config, (char *)0xA0000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 1 Bottom");
  status = read_check_memory(&dma_config, (char *)0xB0000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 1 Middle");
  /* The top of this memory reagion is not mapped by default by TRCH
  status = read_check_memory_end(&dma_config, (char *)0xC0000000);
  directive_failed(status, "DMA Access to HPPS Low DDR Block 1 Top");
   */

  /* Check other memory boundaries */
  status = read_check_memory(&dma_config, bsp_section_text_begin);
  directive_failed(status, "DMA Access to bsp_section_text_begin");
  status = read_check_memory_end(&dma_config, bsp_section_text_end);
  directive_failed(status, "DMA Access to bsp_section_text_end");
  status = read_check_memory(&dma_config, bsp_section_start_begin);
  directive_failed(status, "DMA Access to bsp_section_start_begin");
  status = read_check_memory_end(&dma_config, bsp_section_start_end);
  directive_failed(status, "DMA Access to bsp_section_start_end");
  status = read_check_memory(&dma_config, bsp_section_data_begin);
  directive_failed(status, "DMA Access to bsp_section_data_begin");
  status = read_check_memory_end(&dma_config, bsp_section_data_end);
  directive_failed(status, "DMA Access to bsp_section_data_end");
  status = read_check_memory(&dma_config, bsp_section_bss_begin);
  directive_failed(status, "DMA Access to bsp_section_bss_begin");
  status = read_check_memory_end(&dma_config, bsp_section_bss_end);
  directive_failed(status, "DMA Access to bsp_section_bss_end");
  status = read_check_memory_end(&dma_config, bsp_section_stack_end);
  directive_failed(status, "DMA Access to bsp_section_stack_end");

  /* This window is undefined other than being a view into 40-bit memory space */
  /*status = read_check_memory(&dma_config, __window_start__);
  directive_failed(status, "DMA Access to __window_start__");
  status = read_check_memory(&dma_config, __window_end__);
  directive_failed(status, "DMA Access to __window_end__");*/

  /* TCM accesses from DMA must use the external interface */
  /*status = read_check_memory(&dma_config, __tcm_a_start__);
  directive_failed(status, "DMA Access to __tcm_a_start__");
  status = read_check_memory(&dma_config, __tcm_a_end__);
  directive_failed(status, "DMA Access to __tcm_a_end__");
  status = read_check_memory(&dma_config, __tcm_b_start__);
  directive_failed(status, "DMA Access to __tcm_b_start__");
  status = read_check_memory(&dma_config, __tcm_b_end__);
  directive_failed(status, "DMA Access to __tcm_b_end__");*/

  status = read_check_memory(&dma_config, __hpps_ddr_low_start__);
  directive_failed(status, "DMA Access to __hpps_ddr_low_start__");
  /* The top of this memory reagion is not mapped by default by TRCH
  status = read_check_memory_end(&dma_config, __hpps_ddr_low_end__);
  directive_failed(status, "DMA Access to __hpps_ddr_low_end__");
   */
  status = read_check_memory(&dma_config, __rtps_ddr_low_0_start__);
  directive_failed(status, "DMA Access to __rtps_ddr_low_0_start__");
  status = read_check_memory_end(&dma_config, __rtps_ddr_low_0_end__);
  directive_failed(status, "DMA Access to __rtps_ddr_low_0_end__");
  status = read_check_memory(&dma_config, __hpps_mbox_start__);
  directive_failed(status, "DMA Access to __hpps_mbox_start__");
  status = read_check_memory_end(&dma_config, __hpps_mbox_end__);
  directive_failed(status, "DMA Access to __hpps_mbox_end__");

  /* Uninitialize DMA controller */
  status = dma_uninit(&dma_config);
  directive_failed(status, "DMA Uninitialization");
  TEST_END();
  rtems_test_exit( 0 );
}
