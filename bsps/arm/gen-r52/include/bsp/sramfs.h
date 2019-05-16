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

#ifndef SRAMFS_H
#define SRAMFS_H

#include <bsp/dma.h>

struct SRAMFS_Global_Table_s;
typedef struct SRAMFS_Global_Table_s SRAMFS_Global_Table_t;

typedef struct {
  volatile SRAMFS_Global_Table_t *global_table;
  DMA_Config_t *dma_config;
} SRAMFS_Config_t;

/*!
 * Initialize the SMC-353.
 *
 * \param       sramfs_config
 *              A pointer to an allocated SRAMFS_Config_t structure to be initialized.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the SMC-353
 *              timer.
 *
 * \param       dma_config
 *              An optional DMA configuration object to use for memory copies.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code sramfs_init(SRAMFS_Config_t *sramfs_config, volatile void *base, DMA_Config_t *dma_config);

/*!
 * Uninitialize the SMC-353.
 *
 * \param       sramfs_config
 *              A pointer to an allocated SRAMFS_Config_t structure to be initialized.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code sramfs_uninit(SRAMFS_Config_t *sramfs_config);

/*!
 * Load file data from the SMC-353 SRAM.
 *
 * \param       sramfs_config
 *              A pointer to an allocated SRAMFS_Config_t structure to be initialized.
 *
 * \param       fname
 *              The name of the file to be retrieved.
 *
 * \param       addr
 *              [out] The address to which the data is loaded. The address is chosen
 *              by the SMC SRAM Filesystem Driver.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed. An unallocated DMA channel
 *                                   may not be available at the time of the API
 *                                   call. The DMA channel may not be in the STOPPED state.
 * \retval      RTEMS_INVALID_NUMBER The memory regions specified are overlapping.
 */
rtems_status_code sramfs_load(SRAMFS_Config_t *sramfs_config, const char *fname, uint32_t **addr);

/*!
 * Load file data from the SMC-353 SRAM at a specified address.
 *
 * \param       sramfs_config
 *              A pointer to an allocated SRAMFS_Config_t structure to be initialized.
 *
 * \param       fname
 *              The name of the file to be retrieved.
 *
 * \param       addr
 *              The address to which the data will be loaded. This is most useful when using
 *              DMA since the DMA engine may have a different view of memory than the CPU.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed. An unallocated DMA channel
 *                                   may not be available at the time of the API
 *                                   call. The DMA channel may not be in the STOPPED state.
 * \retval      RTEMS_INVALID_NUMBER The memory regions specified are overlapping.
 */
rtems_status_code sramfs_load_addr(SRAMFS_Config_t *sramfs_config, const char *fname, uint32_t *addr);

#endif // SRAMFS_H
