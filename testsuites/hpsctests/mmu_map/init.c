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
#include <bsp/mmu_500.h>
#include <bsp/dma.h>
#include <bsp/mpu.h>

const char rtems_test_name[] = "HPSC MMU Map";

/* DMA Microcode buffer: in RTPS DRAM (cannot be in TCM) */
#define RTPS_DMA_MCODE_ADDR (uint32_t*)0x40000000
#define RTPS_DMA_MCODE_SIZE 0x00001000

/* Buffers for DMA test: in RTPS DRAM */
#define RTPS_DMA_SRC_ADDR       (uint32_t*)0x40001000
#define RTPS_DMA_SIZE           0x00000200
#define RTPS_DMA_SIZE_12BIT     0x00001000
/* align to page */
#define RTPS_DMA_DST_ADDR       (uint32_t*)0x40002000
/* MMU test maps this to DST_ADDR */
#define RTPS_DMA_DST_REMAP_ADDR (uint32_t*)0x40003000

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

  /* Initialize MMU controller */
  MMU_Config_t mmu_config;
  status = mmu_init(&mmu_config, RTPS_SMMU_BASE);
  directive_failed(status, "MMU Initialization");

  /* Disable MMU mappings */
  status = mmu_disable(&mmu_config);
  directive_failed(status, "MMU Disable");

  /* Define a memory region for allocations */
  rtems_id region_id;
  status = rtems_region_create(rtems_build_name('M', 'M', 'U', ' '), (void*)0x50000000,
		               0x10000000, 0x1000 /* 4KB page size */, RTEMS_PRIORITY,
			       &region_id);
  directive_failed(status, "Region Creation");

  /* Create MMU context */
  MMU_Context_t *mmu_context;
  status = mmu_context_create(&mmu_context, &mmu_config, region_id, MMU_PAGESIZE_4KB);
  directive_failed(status, "MMU Context Creation");

  /* Create MMU stream */
  MMU_Stream_t *mmu_stream;
  status = mmu_stream_create(&mmu_stream, MASTER_ID_RTPS_DMA, mmu_context);
  directive_failed(status, "MMU Stream Creation");

  /* MMU maps RTPS_DMA_DST_REMAP_ADDR to RTPS_DMA_DST_ADDR and passthrough for
   * other required memory regions */
  status = mmu_map(mmu_context, RTPS_DMA_MCODE_ADDR, RTPS_DMA_MCODE_ADDR, RTPS_DMA_MCODE_SIZE);
  directive_failed(status, "MMU Map Creation");
  status = mmu_map(mmu_context, RTPS_DMA_SRC_ADDR, RTPS_DMA_SRC_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Creation");
  status = mmu_map(mmu_context, RTPS_DMA_DST_ADDR, RTPS_DMA_DST_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Creation");
  status = mmu_map(mmu_context, RTPS_DMA_DST_REMAP_ADDR, RTPS_DMA_DST_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Creation");

  /* Enable MMU mappings */
  status = mmu_enable(&mmu_config);
  directive_failed(status, "MMU Enable");

  /* Verify DMA controller supports any channels at all */
  if ( BSP_DMA_MAX_CHANNELS == 0 ) {
    printf( "\nBSP_DMA_MAX_CHANNELS Check FAILED -- expected (>0) got (0)\n");
    rtems_test_exit( RTEMS_INVALID_NUMBER );
  }

  /* Initialize DMA controller */
  DMA_Config_t dma_config;
  status = dma_init(&dma_config, BSP_DMA_BASE);
  directive_failed(status, "DMA Initialization");

  /* Allocate DMA channel resource where the DMA engine will be able to access it */
  DMA_Channel_t *dma_channel = (DMA_Channel_t*)RTPS_DMA_MCODE_ADDR;
  status = dma_channel_alloc(&dma_config, dma_channel, 0);
  directive_failed(status, "DMA Channel 0 Allocation");

  /* Setup buffer */
  uint32_t *src_buf = (uint32_t *)RTPS_DMA_SRC_ADDR;
  uint32_t *dst_buf = (uint32_t *)RTPS_DMA_DST_ADDR;

  for (unsigned i = 0; i < (RTPS_DMA_SIZE / sizeof(uint32_t)); ++i) {
    src_buf[i] = 0xf00d0000 | i;
  }

  /* Execute memory copy */
  status = dma_copy_memory_to_memory(dma_channel, (uint32_t *)RTPS_DMA_DST_REMAP_ADDR, src_buf, RTPS_DMA_SIZE);
  directive_failed(status, "DMA Channel 0 Copy");

  /* Wait for the channel to finish executing */
  bool stopped = false;
  bool faulted = false;
  while (!stopped && !faulted) {
    /* Get the current execution status of the DMA channel */
    status = dma_channel_is_stopped(dma_channel, &stopped);
    directive_failed(status, "DMA Channel 0 Status Retrieval");
    status = dma_channel_is_faulted(dma_channel, &faulted);
    directive_failed(status, "DMA Channel 0 Status Retrieval");
  }
  
  /* Check whether there was a fault during program execution */
  fatal_int_service_status(faulted, 0, "DMA Channel 0 Program Execution Fault");

  /* Check that the virtual access via mapped memory was successful */
  bool transferred = memcmp(src_buf, dst_buf, RTPS_DMA_SIZE);
  fatal_int_service_status(transferred, 0, "DMA Channel 0 Memory Destination Value");

  /* Free the DMA channel */
  status = dma_channel_free(dma_channel);
  directive_failed(status, "DMA Channel 0 Deallocation");

  /* Uninitialize DMA controller */
  status = dma_uninit(&dma_config);
  directive_failed(status, "DMA Uninitialization");

  /* Destroy MMU context mapping */
  status = mmu_unmap(mmu_context, RTPS_DMA_MCODE_ADDR, RTPS_DMA_MCODE_SIZE);
  directive_failed(status, "MMU Map Destruction");
  status = mmu_unmap(mmu_context, RTPS_DMA_SRC_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Destruction");
  status = mmu_unmap(mmu_context, RTPS_DMA_DST_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Destruction");
  status = mmu_unmap(mmu_context, RTPS_DMA_DST_REMAP_ADDR, RTPS_DMA_SIZE_12BIT);
  directive_failed(status, "MMU Map Destruction");

  /* Destroy the MMU stream */
  status = mmu_stream_destroy(mmu_stream);
  directive_failed(status, "MMU Stream Destruction");

  /* Destroy the MMU context */
  status = mmu_context_destroy(mmu_context);
  directive_failed(status, "MMU Context Destruction");

  /* Uninitialize MMU controller */
  status = mmu_uninit(&mmu_config);
  directive_failed(status, "MMU Uninitialization");

  TEST_END();
  rtems_test_exit( 0 );
}
