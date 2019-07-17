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
#include <bsp/xnandps_blkdev.h>
#include <bsp/xnandps_bbm.h>
#include <bsp/xnandps.h>
#include <errno.h>
#include <rtems/blkdev.h>
#include <rtems/bdbuf.h>

#ifdef NAND_BLKDEV_DEBUG
# define NAND_BLKDEV_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
# define NAND_BLKDEV_DEBUG_PRINTF(...)
#endif

static uint32_t get_num_bad_blocks(XNandPs *NandInstPtr)
{
  uint32_t block_out = 0;
  uint32_t block_index = 0;
  uint32_t max_blocks = NandInstPtr->Geometry.NumBlocks;
  /* Walk the flash device, counting bad blocks */
  while (block_index < max_blocks) {
    if (XNandPs_IsBlockBad(NandInstPtr, block_index) == XST_SUCCESS) {
      block_out++;
    }
    block_index++;
  }
  return block_out;
}

static rtems_status_code get_block_index(XNandPs *NandInstPtr, uint32_t target_index, uint32_t *block_out)
{
  *block_out = 0;
  uint32_t max_blocks = NandInstPtr->Geometry.NumBlocks;
  /* Walk the flash device, skipping bad blocks */
  while (*block_out < max_blocks) {
    if (XNandPs_IsBlockBad(NandInstPtr, *block_out) != XST_SUCCESS) {
      /* Block is good */
      if (!target_index) {
        break;
      }
      target_index--;
    } else {
    }
    (*block_out)++;
  }
  return target_index ? RTEMS_IO_ERROR : RTEMS_SUCCESSFUL;
}

static bool
XNandPs_read_block (XNandPs *NandInstPtr, uint32_t block, uint8_t* buffer)
{
  /* Map the requested block over available blocks ignoring bad blocks */
  rtems_status_code sc = get_block_index(NandInstPtr, block, &block);
  if (sc != RTEMS_SUCCESSFUL) {
    return EIO;
  }
  u64 Offset = block * NandInstPtr->Geometry.BlockSize;
  int status = XNandPs_Read(NandInstPtr, Offset, NandInstPtr->Geometry.BlockSize, buffer, NULL);
  if (status != XST_SUCCESS) {
    return EIO;
  }
  return 0;
}

static int
XNandPs_read (XNandPs *NandInstPtr, rtems_blkdev_request* req)
{
  rtems_blkdev_sg_buffer* sg = req->bufs;
  uint32_t buf;
  int ret = 0;

  for (buf = 0; (ret == 0) && (buf < req->bufnum); buf++, sg++)
  {
    uint8_t* data;
    uint32_t fb;
    uint32_t b;
    fb = sg->length / NandInstPtr->Geometry.BlockSize;
    data = sg->buffer;
    for (b = 0; b < fb; b++, data += NandInstPtr->Geometry.BlockSize)
    {
      ret = XNandPs_read_block (NandInstPtr, sg->block + b, data);
      if (ret)
        break;
    }
  }

  rtems_blkdev_request_done (req, ret ? RTEMS_IO_ERROR : RTEMS_SUCCESSFUL);

  return 0;
}

static bool
XNandPs_write_block (XNandPs *NandInstPtr, uint32_t block, uint8_t* buffer)
{
  /* Map the requested block over available blocks ignoring bad blocks */
  rtems_status_code sc = get_block_index(NandInstPtr, block, &block);
  if (sc != RTEMS_SUCCESSFUL) {
    return EIO;
  }
  u64 Offset = block * NandInstPtr->Geometry.BlockSize;
  int status = XNandPs_Write(NandInstPtr, Offset, NandInstPtr->Geometry.BlockSize, buffer, NULL);
  if (status != XST_SUCCESS) {
    return EIO;
  }
  return 0;
}

static int
XNandPs_write (XNandPs *NandInstPtr, rtems_blkdev_request* req)
{
  rtems_blkdev_sg_buffer* sg = req->bufs;
  uint32_t buf;
  int ret = 0;

  for (buf = 0; (ret == 0) && (buf < req->bufnum); buf++, sg++)
  {
    uint8_t* data;
    uint32_t fb;
    uint32_t b;
    fb = sg->length / NandInstPtr->Geometry.BlockSize;
    data = sg->buffer;
    for (b = 0; b < fb; b++, data += NandInstPtr->Geometry.BlockSize)
    {
      ret = XNandPs_write_block (NandInstPtr, sg->block + b, data);
      if (ret)
        break;
    }
  }

  rtems_blkdev_request_done (req, ret ? RTEMS_IO_ERROR : RTEMS_SUCCESSFUL);

  return 0;
}

static int
XNandPs_ioctl(rtems_disk_device *dd, uint32_t req, void *argp)
{
  rtems_blkdev_request *r = argp;
  XNandPs *NandInstPtr = rtems_disk_get_driver_data(dd);

  rtems_mutex_lock(&NandInstPtr->lock);

  switch (req) {
    case RTEMS_BLKIO_REQUEST:
      switch (r->req) {
        case RTEMS_BLKDEV_REQ_READ:
          errno = XNandPs_read(NandInstPtr, r);
          break;
        case RTEMS_BLKDEV_REQ_WRITE:
          errno = XNandPs_write(NandInstPtr, r);
          break;
        default:
          errno = EINVAL;
          break;
      }
      break;

    default:
      errno = rtems_blkdev_ioctl(dd, req, argp);
      break;
  }

  rtems_mutex_unlock(&NandInstPtr->lock);

  return errno == 0 ? 0 : -1;
}

rtems_status_code
XNandPs_blkdev_create(XNandPs *NandInstPtr, const char *dev)
{
  rtems_status_code status;
  status = rtems_blkdev_create(
    dev,
    NandInstPtr->Geometry.BlockSize,
    NandInstPtr->Geometry.NumBlocks - get_num_bad_blocks(NandInstPtr),
    XNandPs_ioctl,
    NandInstPtr
  );
  return status;
}
