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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <bsp.h>
#include <bsp/hwinfo.h>
#include <bsp/sramfs.h>

#ifdef SRAMFS_DEBUG
# define SRAMFS_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
# define SRAMFS_DEBUG_PRINTF(...)
#endif

#define FILE_NAME_LENGTH 200
#define ECC_512_SIZE  3
#define PAGE_SIZE (1 << 14) // 16KB (used for progress display only)

typedef struct {
    uint32_t valid;
    uint32_t offset;	/* offset in SRAM image */
    uint32_t size;
    uint32_t load_addr;		/* 32bit load address in DRAM at run-time */
    uint32_t load_addr_high;	/* high 32bit of 64 bit load address in DRAM at run-time */
    char  name[FILE_NAME_LENGTH];
    uint32_t entry_offset;  /* the offset of the entry point in the image */
    uint8_t chcksum[32];    /* SHA-256 checksum */
    uint8_t ecc[ECC_512_SIZE];  /* ecc of the struct */
} file_descriptor;

typedef struct SRAMFS_Global_Table_s {
    uint32_t low_mark_data;	/* low mark of data */
    uint32_t high_mark_fd;	/* high mark of file descriptors */
    uint32_t n_files;		/* number of files */
    uint32_t fsize;		/* sram file size */
    uint8_t ecc[ECC_512_SIZE];   /* ecc of the struct */
    file_descriptor fds[0];
} SRAMFS_Global_Table_t;

static rtems_status_code load_dma(DMA_Config_t *dma_config, uint32_t *sram_addr, uint32_t *load_addr, unsigned size)
{
  rtems_status_code status;
  SRAMFS_DEBUG_PRINTF("SMC: initiating DMA transfer\r\n");

  DMA_Channel_t dma_channel;
  status = dma_channel_alloc_any(dma_config, &dma_channel);
  if (status) {
    return status;
  }

  /* Execute memory copy */
  status = dma_copy_memory_to_memory(&dma_channel, load_addr, sram_addr, size);
  if (status) {
    goto channel_cleanup;
  }

  /* Wait for the channel to finish executing */
  bool stopped = false;
  bool faulted = false;
  while (!stopped && !faulted) {
    /* Get the current execution status of the DMA channel */
    status = dma_channel_is_stopped(&dma_channel, &stopped);
    if (status) {
      goto channel_cleanup;
    }
    status = dma_channel_is_faulted(&dma_channel, &faulted);
    if (status) {
      goto channel_cleanup;
    }
  }

  /* Check whether there was a fault during program execution */
  if (faulted) {
    status = RTEMS_IO_ERROR;
    goto channel_cleanup;
  }

  rtems_status_code free_status;
channel_cleanup:
  /* Free the DMA channel */
  free_status = dma_channel_free(&dma_channel);
  if (status) {
    SRAMFS_DEBUG_PRINTF("SMC: DMA transfer failed: %u\r\n", status);
  } else {
    SRAMFS_DEBUG_PRINTF("SMC: DMA transfer succesful\r\n");
  }
  if (free_status) {
    return free_status;
  }
  return status;
}

static rtems_status_code load_memcpy(uint32_t *sram_addr, uint32_t *load_addr, unsigned size)
{
  unsigned p, w, b;

  uint32_t pages = size / PAGE_SIZE;
  uint32_t rem_words = (size % PAGE_SIZE) / sizeof(uint32_t);
  uint32_t rem_bytes = (size % PAGE_SIZE) % sizeof(uint32_t);

  // Split into pages, in order to print progress not too frequently and
  // without having to add a conditional (for whether to print progress
  // on every iteration of in the inner loop over words.
  for (p = 0; p < pages; p++) {
    for (w = 0; w < PAGE_SIZE / sizeof(uint32_t); w++) {
      *load_addr = *sram_addr;
      load_addr++;
      sram_addr++;
    }
    SRAMFS_DEBUG_PRINTF("SMC: loading... %3u%%\r", p * 100 / pages);
  }
  for (w = 0; w < rem_words; w++) {
    *load_addr = *sram_addr;
    load_addr++;
    sram_addr++;
  }
  uint8_t *load_addr_8 = (uint8_t *) load_addr;
  uint8_t *sram_addr_8 = (uint8_t *) sram_addr;
  for (b = 0; b < rem_bytes; b++) {
    *load_addr_8 = *sram_addr_8;
    load_addr_8++;
    sram_addr_8++;
  }
  SRAMFS_DEBUG_PRINTF("SMC: loading... 100%%\r\n");
  return RTEMS_SUCCESSFUL;
}

static rtems_status_code sramfs_load_internal(SRAMFS_Config_t *sramfs_config, const char *fname, bool use_file_load_address, uint32_t **addr)
{
  volatile SRAMFS_Global_Table_t *gt;
  unsigned i;
  uint32_t *load_addr_32;
  uint32_t *sram_addr_32;
  file_descriptor fd_buf;

  /* If not using the internal address, one must be provided */
  assert(use_file_load_address || addr);

  gt = sramfs_config->global_table;
  SRAMFS_DEBUG_PRINTF("SMC: SRAM: #files : %u, low_mark_data(0x%x), high_mark_fd(0x%x)\r\n",
         gt->n_files, gt->low_mark_data, gt->high_mark_fd);

  for (i = 0; i < gt->n_files ; i++) {
    fd_buf = gt->fds[i];
    if (!fd_buf.valid) {
      continue;
    }
    if (!strcmp(fd_buf.name, fname)) {
      break;
    }
  }
  if (i == gt->n_files) {
    SRAMFS_DEBUG_PRINTF("SMC: ERROR: file not found in SRAM: %s\r\n", fname);
    return RTEMS_NOT_DEFINED;
  }

  uint32_t offset = fd_buf.offset;
  sram_addr_32 = (uint32_t *) (((unsigned char *)gt) + offset);
  if (use_file_load_address) {
    load_addr_32 = (uint32_t *)fd_buf.load_addr;
  } else {
    load_addr_32 = *addr;
  }

  SRAMFS_DEBUG_PRINTF("SMC: loading file #%u: %s: 0x%p -> 0x%x (%u KB)\r\n",
         i, fd_buf.name, sram_addr_32,
         fd_buf.load_addr, fd_buf.size / 1024);

  int rc;
  if (sramfs_config->dma_config) {
    rc = load_dma(sramfs_config->dma_config, sram_addr_32, load_addr_32, fd_buf.size);
  } else {
    rc = load_memcpy(sram_addr_32, load_addr_32, fd_buf.size);
  }

  /* only write over the provided address if the embedded address is being used */
  if (addr && use_file_load_address) {
    *addr = load_addr_32;
  }
  return rc;
}

rtems_status_code sramfs_load_addr(SRAMFS_Config_t *sramfs_config, const char *fname, uint32_t *addr)
{
  return sramfs_load_internal(sramfs_config, fname, false, &addr);
}

rtems_status_code sramfs_load(SRAMFS_Config_t *sramfs_config, const char *fname, uint32_t **addr)
{
  return sramfs_load_internal(sramfs_config, fname, true, addr);
}

rtems_status_code sramfs_init(SRAMFS_Config_t *sramfs_config, volatile void *base, DMA_Config_t *dma)
{
  sramfs_config->global_table = (volatile SRAMFS_Global_Table_t *)base;
  sramfs_config->dma_config = dma;
  return RTEMS_SUCCESSFUL;
}

rtems_status_code sramfs_uninit(SRAMFS_Config_t *sramfs_config)
{
  sramfs_config->global_table = NULL;
  sramfs_config->dma_config = NULL;
  return RTEMS_SUCCESSFUL;
}
