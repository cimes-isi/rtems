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
#include <bsp/smc_353.h>
#include <bsp/xnandps.h>
#include <bsp/xnandps_bbm.h>
#include <stdio.h>

/* Test parameters */
#define NAND_TEST_START_BLOCK	64	/**< Starting block to test */
#define NAND_TEST_NUM_BLOCKS	16	/**< Number of blocks to test */
#define NAND_TEST_BLOCK_SIZE	0x20000	/**< Test Block Size, must be same as the flash block size */

XNandPs NandInstance; /* XNand Instance. */
u8 WriteBuffer[NAND_TEST_BLOCK_SIZE];	/**< Block sized write buffer */

/* Prepare the write buffer. Fill in the data need to be written into Flash Device. */
static void fill_write_buffer(u8 write_buffer[NAND_TEST_BLOCK_SIZE])
{
  u32 Index;

  for (Index = 0; Index < NAND_TEST_BLOCK_SIZE; Index++) {
    write_buffer[Index] = Index % 256;
  }
}

/* Erase the blocks in the flash */
static void erase_blocks(XNandPs *NandInstPtr, u32 EndBlock)
{
  u32 BlockIndex;
  rtems_status_code status;

  for (BlockIndex = NAND_TEST_START_BLOCK; BlockIndex < EndBlock; BlockIndex++) {
    /* Don't erase bad blocks. */
    if (XNandPs_IsBlockBad(NandInstPtr, BlockIndex) == XST_SUCCESS) {
      continue;
    }
    /* Perform erase operation. */
    status = XNandPs_EraseBlock(NandInstPtr, BlockIndex);
    directive_failed(status, "Block Data Erase");
  }
}

static void check_block(XNandPs *NandInstPtr, int BlockIndex, u8 write_buffer[NAND_TEST_BLOCK_SIZE])
{
  rtems_status_code status;
  u8 ReadBuffer[NAND_TEST_BLOCK_SIZE];	/**< Block sized Read buffer */
  u64 Offset = BlockIndex * NandInstPtr->Geometry.BlockSize;

  /* Clear the Receive buffer for next iteration */
  memset(ReadBuffer, 0, NAND_TEST_BLOCK_SIZE);

  /* Perform the write operation. */
  status = XNandPs_Write(NandInstPtr, Offset, NAND_TEST_BLOCK_SIZE, write_buffer, NULL);
  directive_failed(status, "Block Data Write");

  /* Perform the read operation. */
  status = XNandPs_Read(NandInstPtr, Offset, NAND_TEST_BLOCK_SIZE, ReadBuffer, NULL);
  directive_failed(status, "Block Data Read");

  /* Compare the data read against the data Written. */
  int result = memcmp(ReadBuffer, write_buffer, NAND_TEST_BLOCK_SIZE);
  fatal_int_service_status(result, 0, "Block Data Verification");
}

const char rtems_test_name[] = "HPSC SMC-353 NAND Write";

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;
  XNandPs_Config nand_config =
  {
    0,       /**< Device ID of device */
    (uint32_t)SMC_BASE,  /**< SMC Base address */
    (uint32_t)TRCH_NAND_BASE,        /**< NAND flash Base address */
    8      /**< Flash data width */
  };

  TEST_BEGIN();

  fill_write_buffer(WriteBuffer);

  /* Run the NAND read write test */
  u32 BlockIndex;
  u32 EndBlock;

  /* Initialize the flash driver. */
  status = XNandPs_CfgInitialize(&NandInstance, &nand_config,
      nand_config.SmcBase,nand_config.FlashBase);
  directive_failed(status, "NAND Initialization");

  EndBlock = NAND_TEST_START_BLOCK + NAND_TEST_NUM_BLOCKS;
  erase_blocks(&NandInstance, EndBlock);
  /* Perform the read/write operation */
  for (BlockIndex = NAND_TEST_START_BLOCK; BlockIndex < EndBlock; BlockIndex++) {
    /* Don't program bad blocks. */
    if (XNandPs_IsBlockBad(&NandInstance, BlockIndex) == XST_SUCCESS) {
      continue;
    }

    check_block(&NandInstance, BlockIndex, WriteBuffer);
  }

  TEST_END();
  rtems_test_exit( 0 );
}
