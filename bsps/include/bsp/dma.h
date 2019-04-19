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

#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

typedef struct DMA_Channel {
  uint8_t channel;
  BSP_DMA_CHANNEL_PVT dma_channel_pvt;
} DMA_Channel_t;

/*!
 * \addtogroup DMA_CSR DMA API for Configuration, Control, and Status
 *
 * This API provides functions for configuration, control, and status queries
 * of the DMA controller.
 *
 * @{
 */

/*!
 * Initialize the DMA controller.
 *
 * Initializes the DMA controller by setting the necessary control values to
 * establish the security state and MUXed peripheral request interface selection
 * configurations before taking the DMA controller out of reset.
 *
 * After the DMA is initialized, the following conditions hold true:
 *  * All DMA channel threads are in the Stopped state.
 *  * All DMA channel threads are available for allocation.
 *  * DMA Manager thread is waiting for an instruction from either APB
 *    interface.
 *  * The security state configurations of the DMA Manager, interrupt outputs,
 *    and peripheral request interfaces are established and immutable until the
 *    DMA is reset.
 *  * The MUXed peripheral request interface selection configurations are
 *    established and immutable until the DMA is reset.
 *
 * \param       dma_cfg
 *              A pointer to a DMA_Config_t structure containing the desired
 *              DMA controller security state and peripheral request interface
 *              MUX selections.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the DMA
 *              controller
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed.
 */
rtems_status_code dma_init(DMA_Config_t *dma_cfg, volatile uint32_t *base);

/*!
 * Uninitializes the DMA controller.
 *
 * Uninitializes the DMA controller by killing any running channel threads and
 * putting the DMA controller into reset.
 *
 * \param       dma_cfg
 *              A DMA controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed.
 */
rtems_status_code dma_uninit(DMA_Config_t *dma_cfg);

/*!
 * Allocate a DMA channel resource for use.
 *
 * \param       dma_cfg
 *              A DMA controller configuration structure.
 *
 * \param       channel
 *              A DMA controller channel structure.
 *
 * \param       channel_number
 *              The 0-based channel resource to allocate.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed.
 */
rtems_status_code dma_channel_alloc(DMA_Config_t *dma_cfg, DMA_Channel_t *channel, uint8_t channel_number);

/*!
 * Allocate a free DMA channel resource for use if there are any.
 *
 * \param       dma_cfg
 *              A DMA controller configuration structure.
 *
 * \param       channel
 *              A DMA controller channel structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed. An unallocated channel
 *                              may not be available at the time of the API
 *                              call.
 */
rtems_status_code dma_channel_alloc_any(DMA_Config_t *dma_cfg, DMA_Channel_t *channel);

/*!
 * Free a DMA channel resource for reuse.
 *
 * \param       channel
 *              The DMA controller channel resource to free.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed. The channel may not be in
 *                              the STOPPED state.
 */
rtems_status_code dma_channel_free(DMA_Channel_t *channel);

/*!
 * Abort any activity on the specified DMA channel.
 *
 * Terminates the channel thread of execution by issuing a DMAKILL instruction
 * using the DMA APB slave interface.
 *
 * \param       channel
 *              The DMA channel to abort
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_IO_ERROR  The operation failed.
 * \retval      RTEMS_TIMEOUT   Timeout waiting for the channel to change into
 *                              KILLING or STOPPED state.
 */
rtems_status_code dma_channel_reset(DMA_Channel_t *channel);

/*!
 * Checks whether the specified DMA channel is quiescent.
 *
 * If the DMA channel is quiescent, a new DMA transfer may be started.
 *
 * \param       channel
 *              The DMA channel to check.
 *
 * \param       stopped
 *              [out] Pointer to an output parameter to contain the DMA
 *              channel execution state.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel identifier is invalid.
 */
rtems_status_code dma_channel_is_stopped(DMA_Channel_t *channel,
                                                       bool *stopped);

/*!
 * Checks whether the specified DMA channel is faulted.
 *
 * If the DMA channel is faulted, it must be reset before a new DMA
 * transfer may be started.
 *
 * \param       channel
 *              The DMA channel to return the operational state of.
 *
 * \param       faulted
 *              [out] Pointer to an output parameter to contain the DMA
 *              channel faulted state.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel identifier is invalid.
 */
rtems_status_code dma_channel_is_faulted(DMA_Channel_t *channel,
                                                       bool *faulted);

/*!
 * Returns the interrupt status of the specified DMA channel.
 *
 * \param       channel
 *              The channel the interrupt is associated with.
 *
 * \param       channel_interrupt_high
 *              [out] Pointer to an output parameter to contain the DMA
 *              channel interrupt state.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 */
rtems_status_code dma_channel_int_status_get(DMA_Channel_t *channel, bool *channel_interrupt_high);

/*!
 * Returns the abort status.
 *
 * This will fill the outbound pointer argument with true if the DMA Controller has signaled an abort on any of the DMA channels.
 *
 * \param       abort_interrupt_high
 *              [out] Pointer to an output parameter to contain the DMA
 *              channel interrupt state.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 */
rtems_status_code dma_abort_int_status_get(bool *abort_interrupt_high);

/*!
 * Clear the active (HIGH) interrupt status of the specified DMA channel.
 *
 * Does nothing if the channel interrupt is inactive (LOW).
 *
 * \param       channel
 *              The channel the interrupt is associated with.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 */
rtems_status_code dma_channel_int_clear(DMA_Channel_t *channel);

/*!
 * Clear the active (HIGH) abort interrupt status.
 *
 * Does nothing if the abort interrupt is inactive (LOW).
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 */
rtems_status_code dma_abort_int_clear(void);

/*!
 * @}
 */

/*!
 * \addtogroup DMA_STD_OPS DMA API for Standard Operations
 *
 * The functions in this group provide common DMA operations for common bulk
 * data transfers between:
 *  * Memory to Memory
 *  * Zero to Memory
 *  * Keyhole Register to Memory
 *  * Memory to Keyhole Register
 *
 * All DMA operations are asynchronous. The following are the ways to receive
 * notification of a DMA transfer complete operation:
 *  * Use dma_channel_state_get() and poll for the 
 *    executing status.
 *  * Watch the associated interrupt for the given channel.
 *
 * Cache related maintenance on the source and/or destination buffer are not
 * handled the DMA API and are the responsibility of the programmer. This is
 * because the DMA API does not have visibility into the current configuration
 * of the MMU or know about any special considerations regarding the source
 * and/or destination memory. The following are some example scenarios and
 * cache maintenance related precautions that may need to be taken:
 *  * dma_copy_memory_to_memory(): Source buffer should be cleaned or purged,
 *    destination buffer should be invalidated.
 *  * dma_copy_zero_to_memory(): Destination buffer should be invalidated.
 *  * dma_copy_memory_to_register(): Source buffer should be cleaned or purged.
 *  * dma_copy_register_to_memory(): Destination buffer should be invalidated.
 *
 * @{
 */

/*!
 * Uses the DMA engine to asynchronously copy the specified memory from the
 * given source address to the given destination address.
 *
 * Overlapping memory regions are not supported.
 *
 * \param       channel
 *              The DMA channel structure containing details to use for the transfer.
 *
 * \param       dest
 *              The destination memory address to copy to.
 *
 * \param       src
 *              The source memory address to copy from.
 *
 * \param       size
 *              The size of the transfer in bytes.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel or event identifier (if
 *                              used) is invalid, or the memory regions
 *                              specified are overlapping.
 */
rtems_status_code dma_copy_memory_to_memory(DMA_Channel_t *channel,
                                                        void *dest,
                                                        const void *src,
                                                        size_t size);

/*!
 * Uses the DMA engine to asynchronously zero out the specified memory buffer.
 *
 * \param       channel
 *              The DMA channel structure containing details to use for the transfer.
 *
 * \param       buf
 *              The buffer memory address to zero out.
 *
 * \param       size
 *              The size of the buffer in bytes.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel information is invalid.
 */
rtems_status_code dma_copy_zero_to_memory(DMA_Channel_t *channel,
                                                        void *buf,
                                                        size_t size);

/*!
 * Uses the DMA engine to asynchronously transfer the contents of a memory
 * buffer to a keyhole register.
 *
 * \param       channel
 *              The DMA channel structure containing details to use for the transfer.
 *
 * \param       dst_reg
 *              The address of the register to write buffer to.
 *
 * \param       src_buf
 *              The address of the memory buffer for the data.
 *
 * \param       count
 *              The number of transfers to make.
 *
 * \param       register_width_bits
 *              The width of the register to transfer to in bits. Valid values
 *              are 8, 16, 32, and 64.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel, event identifier (if used),
 *                              or register width are invalid, or if the
 *                              destination register or source buffer is
 *                              unaligned to the register width.
 */
rtems_status_code dma_copy_memory_to_register(DMA_Channel_t *channel,
                                           void *dst_reg,
                                           const void *src_buf,
                                           size_t count,
                                           uint32_t register_width_bits);

/*!
 * Uses the DMA engine to asynchronously transfer the contents of a keyhole
 * register to a memory buffer.
 *
 * \param       channel
 *              The DMA channel structure containing details to use for the transfer.
 *
 * \param       dst_buf
 *              The address of the memory buffer to copy to.
 *
 * \param       src_reg
 *              The address of the keyhole register to read from.
 *
 * \param       count
 *              The number of transfers to make.
 *
 * \param       register_width_bits
 *              The width of the register to transfer to in bits. Valid values
 *              are 8, 16, 32, and 64.
 *
 * \retval      RTEMS_SUCCESS        The operation was successful.
 * \retval      RTEMS_IO_ERROR       The operation failed.
 * \retval      RTEMS_INVALID_NUMBER The given channel, event identifier (if used),
 *                              or register width are invalid, or if the
 *                              destination buffer or source register is
 *                              unaligned to the register width.
 */
rtems_status_code dma_copy_register_to_memory(DMA_Channel_t *channel,
                                           void *dst_buf,
                                           const void *src_reg,
                                           size_t count,
                                           uint32_t register_width_bits);

/*!
 * @}
 */

/*!
 * @}
 */

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __DMA_H__ */
