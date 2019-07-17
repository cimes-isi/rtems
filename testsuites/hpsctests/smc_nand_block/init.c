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
#include <bsp/xnandps_blkdev.h>
#include <bsp/xnandps.h>
#include <stdio.h>
#include <rtems/blkdev.h>
#include <fcntl.h>

const char rtems_test_name[] = "HPSC SMC-353 NAND Block Write";
static rtems_disk_device *dd;
XNandPs NandInstance; /* XNand Instance. */

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;
  const char *dev = "/dev/nand";
  XNandPs_Config nand_config =
  {
    0,       /**< Device ID of device */
    (uint32_t)SMC_BASE,  /**< SMC Base address */
    (uint32_t)TRCH_NAND_BASE,        /**< NAND flash Base address */
    8      /**< Flash data width */
  };

  TEST_BEGIN();

  /* Initialize the flash driver. */
  status = XNandPs_CfgInitialize(&NandInstance, &nand_config,
      nand_config.SmcBase,nand_config.FlashBase);
  directive_failed(status, "NAND Initialization");

  status = XNandPs_blkdev_create(&NandInstance, dev);
  directive_failed(status, "NAND Block Device Creation");

  int fd = open(dev, O_RDWR);
  rtems_test_assert(fd >= 0);

  int rv = rtems_disk_fd_get_disk_device(fd, &dd);
  rtems_test_assert(rv == 0);

  ssize_t num_bytes = write(fd, rtems_test_name, strlen(rtems_test_name));
  rtems_test_assert(num_bytes == strlen(rtems_test_name));

  rv = lseek(fd, NandInstance.Geometry.BlockSize * 65, SEEK_SET);
  rtems_test_assert(rv == NandInstance.Geometry.BlockSize * 65);

  num_bytes = write(fd, rtems_test_name, strlen(rtems_test_name));
  rtems_test_assert(num_bytes == strlen(rtems_test_name));

  rv = close(fd);
  rtems_test_assert(rv == 0);

  fd = open(dev, O_RDWR);
  rtems_test_assert(fd >= 0);

  char read_buffer[100];
  num_bytes = read(fd, read_buffer, strlen(rtems_test_name));
  rtems_test_assert(num_bytes == strlen(rtems_test_name));
  rtems_test_assert(memcmp(read_buffer, rtems_test_name, num_bytes) == 0);

  rv = lseek(fd, NandInstance.Geometry.BlockSize * 65, SEEK_SET);
  rtems_test_assert(rv == NandInstance.Geometry.BlockSize * 65);

  num_bytes = read(fd, read_buffer, strlen(rtems_test_name));
  rtems_test_assert(num_bytes == strlen(rtems_test_name));
  rtems_test_assert(memcmp(read_buffer, rtems_test_name, num_bytes) == 0);

  rv = close(fd);
  rtems_test_assert(rv == 0);

  TEST_END();
  rtems_test_exit( 0 );
}
