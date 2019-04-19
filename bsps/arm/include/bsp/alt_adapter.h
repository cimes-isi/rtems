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

#ifndef LIBBSP_ARM_GEN_R52_BSP_ALT_ADAPTER_H
#define LIBBSP_ARM_GEN_R52_BSP_ALT_ADAPTER_H

#define ALT_STATUS_CODE rtems_status_code

/*! Definitions of status codes returned by the HWLIB. */

/*! The operation was successful. */
#define ALT_E_SUCCESS               RTEMS_SUCCESSFUL

/*! The operation failed. */
#define ALT_E_ERROR                 RTEMS_IO_ERROR
/*! An argument violates a range constraint. */
#define ALT_E_ARG_RANGE             RTEMS_INVALID_NUMBER
/*! A bad argument value was passed. */
#define ALT_E_BAD_ARG               RTEMS_INVALID_NUMBER
/*! The operation is invalid or illegal. */
#define ALT_E_BAD_OPERATION         RTEMS_INCORRECT_STATE
/*! An operation or response timeout period expired. */
#define ALT_E_TMO                   RTEMS_TIMEOUT
/*! The buffer does not contain enough free space for the operation. */
#define ALT_E_BUF_OVF               RTEMS_NO_MEMORY

/*!
 * Indicates a FALSE condition.
 */
#define ALT_E_FALSE                 false
/*!
 * Indicates a TRUE condition.
 */
#define ALT_E_TRUE                  true

/* Note, additional positive status codes may be defined to return
 * a TRUE condition with additional information */

#define ALT_CAST(type, ptr)  ((type) (ptr))
#define ALT_CACHE_LINE_SIZE         32

/*! Write the 32 bit word to the destination address in device memory.
 *  \param dest - Write destination pointer address
 *  \param src  - 32 bit data word to write to memory
 */
#define alt_write_word(dest, src)       (*ALT_CAST(volatile uint32_t *, (dest)) = (src))

/*! Read and return the 32 bit word from the source address in device memory.
 *  \param src    Read source pointer address
 *  \returns      32 bit data word value
 */
#define alt_read_word(src)              (*ALT_CAST(volatile uint32_t *, (src)))

/*! Set selected bits in the 32 bit word at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to set in destination word
 */
#define     alt_setbits_word(dest, bits)        (alt_write_word(dest, alt_read_word(dest) | (bits)))

/*! Clear selected bits in the 32 bit word at the destination address in device memory.
 *  \param dest - Destination pointer address
 *  \param bits - Bits to clear in destination word
 */
#define     alt_clrbits_word(dest, bits)        (alt_write_word(dest, alt_read_word(dest) & ~(bits)))

#define ALT_DMA_PERIPH_PROVISION_16550_SUPPORT 0
#define ALT_DMA_PERIPH_PROVISION_QSPI_SUPPORT 0

#define ALT_DMASECURE_ADDR BSP_DMA_BASE

#endif /* LIBBSP_ARM_GEN_R52_BSP_ALT_ADAPTER_H */
