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

#ifndef __XNANDPS_BLKDEV_H__
#define __XNANDPS_BLKDEV_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#include <bsp/xnandps.h>

/*!
 * Initialize the SMC-353.
 *
 * \param       NandInstPtr
 *              A pointer to an initialized NAND Instance structure for block device overlay.
 *
 * \param       dev
 *              The location at which the NAND should be attached to the file system.
 *
 * \retval      RTEMS_SUCCESSFUL      The configuration completed successfully.
 * \retval      RTEMS_INVALID_NUMBER  Media block size or count is not positive.
 * \retval      RTEMS_NO_MEMORY	      Not enough memory.
 * \retval      RTEMS_UNSATISFIED     Cannot create generic device node.
 * \retval      RTEMS_INCORRECT_STATE Cannot initialize bdbuf.
 */
rtems_status_code XNandPs_blkdev_create(XNandPs *NandInstPtr, const char *dev);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __XNANDPS_BLKDEV_H__ */
