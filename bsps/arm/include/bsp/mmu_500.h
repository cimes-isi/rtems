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

#ifndef __MMU_H__
#define __MMU_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define MAX_LEVEL_INDEX 3

enum mmu_pagesize {
  MMU_PAGESIZE_4KB = 0,
  MMU_PAGESIZE_16KB,
  MMU_PAGESIZE_64KB,
};

struct MMU_Config_t;
typedef struct MMU_Config MMU_Config_t;

/* Contains constant metadata about a level */
typedef struct MMU_Level {
    unsigned page_table_size;
    unsigned page_table_entries;
    /* size of block or page */
    unsigned block_size;
    unsigned idx_bits;
    unsigned lsb_bit;
} MMU_Level_t;

typedef struct MMU_Context {
  MMU_Config_t *mmu;
  rtems_id region_id;
  const struct granule *granule;
  /* Top-level page table (always allocated) */
  uint64_t *page_table;
  MMU_Level_t levels[MAX_LEVEL_INDEX+1];
} MMU_Context_t;

typedef struct MMU_Stream {
  MMU_Context_t *context;
} MMU_Stream_t;

struct MMU_Config {
  volatile uint32_t *base;
  MMU_Context_t contexts[MMU_500_MAX_CONTEXTS];
  MMU_Stream_t streams[MMU_500_MAX_STREAMS];
};

/*!
 * \addtogroup MMU_CSR MMU API for Configuration, Control, and Status
 *
 * This API provides functions for configuration, control, and status queries
 * of the MMU controller.
 *
 * @{
 */

/*!
 * Initialize the MMU controller.
 *
 * Initializes the MMU controller by setting the necessary control values to
 * establish the security state before taking the MMU controller out of reset.
 *
 * \param       mmu_config
 *              A pointer to a MMU_Config_t structure containing the desired
 *              MMU controller security state and peripheral request interface
 *              MUX selections.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the MMU
 *              controller
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 */
rtems_status_code mmu_init(MMU_Config_t *mmu_config, volatile uint32_t *base);

/*!
 * Uninitializes the MMU controller.
 *
 * Uninitializes the MMU controller by killing any running channel threads and
 * putting the MMU controller into reset.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 */
rtems_status_code mmu_uninit(MMU_Config_t *mmu_config);

/*!
 * Enables the MMU controller.
 *
 * Activates the MMU so that it can perform memory mappings based on the
 * currently set configuration.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 */
rtems_status_code mmu_enable(MMU_Config_t *mmu_config);

/*!
 * Disables the MMU controller.
 *
 * Disables the MMU such that existing mappings are no longer applied, but
 * are not removed from the MMU.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 */
rtems_status_code mmu_disable(MMU_Config_t *mmu_config);

/*!
 * Enables interrupts for the MMU controller.
 *
 * Enables global and per-context fault interrupts.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS     The operation was successful.
 * \retval      RTEMS_UNSATISFIED Interrupts are not supported on this MMU.
 */
rtems_status_code mmu_interrupts_enable(MMU_Config_t *mmu_config);

/*!
 * Disables interrupts for the MMU controller.
 *
 * Disables global and per-context fault interrupts.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \retval      RTEMS_SUCCESS     The operation was successful.
 * \retval      RTEMS_UNSATISFIED Interrupts are not supported on this MMU.
 */
rtems_status_code mmu_interrupts_disable(MMU_Config_t *mmu_config);

/*!
 * Checks whether a MMU is faulted.
 *
 * \param       mmu_config
 *              A MMU controller structure.
 *
 * \param       faulted
 *              [out] True after call if the MMU has faulted.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 */
rtems_status_code mmu_is_faulted(MMU_Config_t *mmu_config, bool *faulted);

/*!
 * Clears the MMU controller fault.
 *
 * \param       mmu_config
 *              A MMU controller structure.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 */
rtems_status_code mmu_clear_faults(MMU_Config_t *mmu_config);

/*!
 * Creates a MMU controller context.
 *
 * Creates a context used for creating new memory maps.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \param       mmu_config
 *              A MMU controller configuration structure.
 *
 * \param       region_id
 *              A RTEMS ID that identifies the Region to be
 *              used for allocations.
 *
 * \param       page_size
 *              A pointer to the RTEMS ID that identifies the Region to be
 *              used for allocations.
 *
 * \retval      RTEMS_SUCCESS            The operation was successful.
 * \retval      RTEMS_NO_MEMORY          There are no remaining contexts to allocate.
 * \retval      RTEMS_INVALID_ADDRESS    The requested segment was not aligned.
 * \retval      RTEMS_INVALID_SIZE       The requested segment size is too large
 *                                       for the region.
 * \retval      RTEMS_INVALID_ID         The region ID is no longer valid.
 * \retval      RTEMS_UNSATISFIED        The requested segment size could not be
 *                                       found in the region.
 * \retval      RTEMS_TIMEOUT            The memory allocation request timed out.
 * \retval      RTEMS_OBJECT_WAS_DELETED The memory region was destroyed during
 *                                       the allocation.
 */
rtems_status_code mmu_context_create(MMU_Context_t **mmu_context, MMU_Config_t *mmu_config, rtems_id region_id, enum mmu_pagesize page_size);

/*!
 * Destroys the MMU controller context.
 *
 * Destroys a MMU controller context and associated mappings.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 * \retval      RTEMS_IO_ERROR        Existing mappings do not appear as expected.
 * \retval      RTEMS_INVALID_ID      The region ID is no longer valid.
 * \retval      RTEMS_INVALID_ADDRESS Memory to be freed is NULL or not in the memory region.
 */
rtems_status_code mmu_context_destroy(MMU_Context_t *mmu_context);

/*!
 * Checks whether a MMU Context is faulted.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \param       faulted
 *              [out] True after call if the context has faulted.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 */
rtems_status_code mmu_context_is_faulted(MMU_Context_t *mmu_context, bool *faulted);

/*!
 * Clears the MMU controller context fault.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 */
rtems_status_code mmu_context_clear_faults(MMU_Context_t *mmu_context);

/*!
 * Creates a MMU controller stream.
 *
 * Creates a stream used for creating new memory maps.
 *
 * \param       mmu_stream
 *              A MMU controller stream structure.
 *
 * \param       master
 *              The memory master ID of the entity making the request.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 * \retval      RTEMS_NO_MEMORY There are no remaining streams to allocate.
 */
rtems_status_code mmu_stream_create(MMU_Stream_t **mmu_stream, uint32_t master, MMU_Context_t *mmu_context);

/*!
 * Destroys a MMU controller stream.
 *
 * \param       mmu_stream
 *              A MMU controller stream structure.
 *
 * \retval      RTEMS_SUCCESS   The operation was successful.
 */
rtems_status_code mmu_stream_destroy(MMU_Stream_t *mmu_stream);


/*!
 * @}
 */

/*!
 * \addtogroup MMU_STD_OPS MMU API for Standard Operations
 *
 * The functions in this group provide common MMU operations for mapping memory spaces.
 *
 * @{
 */

/*!
 * Uses the MMU to map a memory block of a given size to another memory block
 *
 * Overlapping memory regions are not supported.
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \param       vaddr
 *              The virtual memory address to map from.
 *
 * \param       paddr
 *              The physical memory address to map to.
 *
 * \param       size
 *              The size of the mapped block.
 *
 * \retval      RTEMS_SUCCESS            The operation was successful.
 * \retval      RTEMS_INVALID_ADDRESS    The virtual address mapping overlaps with
 *                                       existing mappings or the allocated segment
 *                                       is not aligned.
 * \retval      RTEMS_INVALID_SIZE       The size is not aligned or the requested
 *                                       segment size is too large for the region.
 * \retval      RTEMS_INVALID_ID         The region ID is no longer valid.
 * \retval      RTEMS_UNSATISFIED        The requested segment size could not be
 *                                       found in the region.
 * \retval      RTEMS_TIMEOUT            The memory allocation request timed out.
 * \retval      RTEMS_OBJECT_WAS_DELETED The memory region was destroyed during
 *                                       the allocation.
 */
rtems_status_code mmu_map(MMU_Context_t *mmu_context, void *vaddr, void *paddr, unsigned size);

/*!
 * Uses the MMU to remove a memory mapping
 *
 * \param       mmu_context
 *              A MMU controller context structure.
 *
 * \param       vaddr
 *              The virtual memory address to map from.
 *
 * \param       size
 *              The size of the mapped block.
 *
 * \retval      RTEMS_SUCCESS         The operation was successful.
 * \retval      RTEMS_INVALID_SIZE    The size is not aligned.
 * \retval      RTEMS_IO_ERROR        Existing mappings do not appear as expected.
 * \retval      RTEMS_INVALID_ID      The region ID is no longer valid.
 * \retval      RTEMS_INVALID_ADDRESS Memory to be freed is NULL or not in the memory region.
 */
rtems_status_code mmu_unmap(MMU_Context_t *mmu_context, void *vaddr, unsigned size);

/*!
 * @}
 */

/*!
 * @}
 */

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __MMU_H__ */
