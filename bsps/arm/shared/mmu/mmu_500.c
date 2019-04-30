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
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include <bsp.h>
#include <bsp/mmu_500.h>
#include <bsp/utility.h>

/* References refer to either
 *    ARM System MMU Architecture Specification (IHI0062D), or
 *    ARM Architecture Reference Manual ARMv8 (DDI0487C) (Ch D4)
 */

#define NUMPAGENDXB   0x3
#define PAGESIZE      0x1000

#ifdef MMU_500_DEBUG
# define MMU_500_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
# define MMU_500_DEBUG_PRINTF(...)
#endif

#if MMU_500_QEMU
#  define NUMPAGES      1
#else
#  define NUMPAGES      (2 * (NUMPAGENDXB + 1))
#endif

#define SMMU_CB_SIZE   (NUMPAGES * PAGESIZE)

struct MMU_Context_Base {
  uint32_t SCTLR;
#define SMMU_SCTLR_CFIE BSP_BIT32(6)
#define SMMU_SCTLR_CFRE BSP_BIT32(5)
#define SMMU_SCTLR_M BSP_BIT32(0)
  uint32_t unused_4[3];
  uint32_t TCR2;
#define SMMU_TCR2_PAGESIZE_40BIT BSP_BIT32(1)
  uint32_t unused_14[3];
  uint64_t TTBR0;
  uint32_t unused_28[2];
  uint32_t TCR;
#define SMMU_TCR_T0SZ(val) BSP_FLD32(val, 0, 5)
#define SMMU_TCR_T0SZ_GET(reg) BSP_FLD32GET(reg, 0, 5)
#define SMMU_TCR_T0SZ_SET(reg, val) BSP_FLD32SET(reg, val, 0, 5)
  uint32_t unused_34[8];
  uint32_t FSR;
#define SMMU_FSR_FAULTS(val) BSP_FLD32(val, 1, 8)
#define SMMU_FSR_FAULTS_GET(reg) BSP_FLD32GET(reg, 1, 8)
#define SMMU_FSR_FAULTS_SET(reg, val) BSP_FLD32SET(reg, val, 1, 8)
  uint8_t unused_padding[SMMU_CB_SIZE - 0x5C];
};

struct MMU_Base {
  uint32_t SCR0;
#define SMMU_SCR0_GFIE BSP_BIT32(2)
#define SMMU_SCR0_GFRE BSP_BIT32(1)
#define SMMU_SCR0_CLIENTPD BSP_BIT32(0)
  uint32_t unused_4[7];
  uint32_t IDR0;
#define SMMU_IDR0_NUMIRPT(val) BSP_FLD32(val, 16, 23)
#define SMMU_IDR0_NUMIRPT_GET(reg) BSP_FLD32GET(reg, 16, 23)
#define SMMU_IDR0_NUMIRPT_SET(reg, val) BSP_FLD32SET(reg, val, 16, 23)
  uint32_t IDR1;
#define SMMU_IDR1_PAGESIZE BSP_BIT32(31)
#define SMMU_IDR1_NUMPAGENDXB(val) BSP_FLD32(val, 28, 30)
#define SMMU_IDR1_NUMPAGENDXB_GET(reg) BSP_FLD32GET(reg, 28, 30)
#define SMMU_IDR1_NUMPAGENDXB_SET(reg, val) BSP_FLD32SET(reg, val, 28, 30)
#define SMMU_IDR1_NUMS2CB(val) BSP_FLD32(val, 16, 23)
#define SMMU_IDR1_NUMS2CB_GET(reg) BSP_FLD32GET(reg, 16, 23)
#define SMMU_IDR1_NUMS2CB_SET(reg, val) BSP_FLD32SET(reg, val, 16, 23)
#define SMMU_IDR1_SMCD BSP_BIT32(15)
#define SMMU_IDR1_SSDTP(val) BSP_FLD32(val, 12, 13)
#define SMMU_IDR1_SSDTP_GET(reg) BSP_FLD32GET(reg, 12, 13)
#define SMMU_IDR1_SSDTP_SET(reg, val) BSP_FLD32SET(reg, val, 12, 13)
#define SMMU_IDR1_NUMSSDNDXB(val) BSP_FLD32(val, 8, 11)
#define SMMU_IDR1_NUMSSDNDXB_GET(reg) BSP_FLD32GET(reg, 8, 11)
#define SMMU_IDR1_NUMSSDNDXB_SET(reg, val) BSP_FLD32SET(reg, val, 8, 11)
#define SMMU_IDR1_NUMCB(val) BSP_FLD32(val, 0, 7)
#define SMMU_IDR1_NUMCB_GET(reg) BSP_FLD32GET(reg, 0, 7)
#define SMMU_IDR1_NUMCB_SET(reg, val) BSP_FLD32SET(reg, val, 0, 7)
  uint32_t IDR2;
  uint32_t IDR3;
  uint32_t IDR4;
  uint32_t IDR5;
  uint32_t IDR6;
  uint32_t IDR7;
  uint64_t unused_40;
  uint32_t SGFSR;
#define SMMU_SGFSR_FAULTS(val) BSP_FLD32(val, 0, 8)
#define SMMU_SGFSR_FAULTS_GET(reg) BSP_FLD32GET(reg, 0, 8)
#define SMMU_SGFSR_FAULTS_SET(reg, val) BSP_FLD32SET(reg, val, 0, 8)

#if MMU_500_QEMU
  uint8_t unused_before_contexts[0x10000 - 0x4C];
#else
  uint8_t unused_before_contexts[SMMU_CB_SIZE - 0x4C];
#endif
  struct MMU_Context_Base contexts_base[16];
};

struct MMU_Stream_Base {
  uint32_t unused_0[512];
  uint32_t SMR;
#define SMMU_SMR_VALID BSP_BIT32(31)
  uint32_t unused_804[255];
  uint32_t S2CR;
#define SMMU_S2CR_EXIDVALID BSP_BIT32(10)
  uint32_t unused_c04[255];
  uint32_t CBAR;
#define SMMU_CBAR_IRPTNDX(val) BSP_FLD32(val, 24, 31)
#define SMMU_CBAR_IRPTNDX_GET(reg) BSP_FLD32GET(reg, 24, 31)
#define SMMU_CBAR_IRPTNDX_SET(reg, val) BSP_FLD32SET(reg, val, 24, 31)
#define SMMU_CBAR_TYPE_STAGE1_WITH_STAGE2_BYPASS BSP_BIT32(16)
  uint32_t unused_1004[511];
  uint32_t CBA2R;
#define SMMU_CBA2R_VA64 BSP_BIT32(0)
};

#define SMMU__CB_TCR__VMSA8__TG0_4KB   (0b00 << 14)
#define SMMU__CB_TCR__VMSA8__TG0_16KB  (0b10 << 14)
#define SMMU__CB_TCR__VMSA8__TG0_64KB  (0b01 << 14)

/* VMSAv8 Long-descriptor fields */
#define VMSA8_DESC__BITS   12
#define VMSA8_DESC__VALID 0b1
#define VMSA8_DESC__TYPE_MASK   0b10
#define VMSA8_DESC__TYPE__BLOCK 0b00
#define VMSA8_DESC__TYPE__TABLE 0b10
#define VMSA8_DESC__TYPE__PAGE  0b10
#define VMSA8_DESC__UNPRIV 0x40
#define VMSA8_DESC__RO     0x80
#define VMSA8_DESC__AF     0x400
#define VMSA8_DESC__AP__EL0    0x40
#define VMSA8_DESC__AP__RO     0x80
#define VMSA8_DESC__AP__RW     0x00
#define VMSA8_DESC__NG     0x800
#define VMSA8_DESC__PXN    (1LL << 53)
#define VMSA8_DESC__XN     (1LL << 54)
#define VMSA8_DESC_BYTES 8

/* Fig D4-15: desc & ~(~0ULL << 48) & (MASK(granule->page_bits))
 * but since this code runs on Aarch32 and can only address 32-bit pointers (not
 * 48-bit), we can just truncate the MSB part with a cast. Fig D4-15
 */
#define NEXT_TABLE_PTR(desc, granule) \
        (uint64_t *)((uint32_t)desc & ~((1 << granule->page_bits)-1))

/* Use Page Table 0 for 0x0 to 0x0000_0000_FFFF_FFFF
 * NOTE: This the only config supported by this driver
 */
#define T0SZ 32

#define GL_BASE(mmu)        ((volatile struct MMU_Base *)(mmu)->base)

#define CB_INDEX(context)       ((((uint8_t*)context) - (uint8_t*)(context)->mmu->contexts) / sizeof(*context))
#define CBn_BASE(base, n)   ((volatile struct MMU_Context_Base *)&GL_BASE(base)->contexts_base[n])
#define CTX_BASE(context)       CBn_BASE((context)->mmu, CB_INDEX(context))

#define ST_INDEX(stream)    ((((uint8_t*)stream) - (uint8_t*)(stream)->context->mmu->streams) / sizeof(*stream))
#define ST_BASE(stream)     ((volatile struct MMU_Stream_Base *)((volatile uint8_t *)(stream)->context->mmu->base + ST_INDEX(stream)))

#define ALIGNED(x, bits) !((uint32_t)x & ((1 << (bits)) - 1))

/* Constant metadata about a translation granule option */
struct granule {
  unsigned tg;
  unsigned bits_per_level;
  unsigned bits_in_first_level;
  unsigned page_bits;
  unsigned start_level;
};

/* Reference meta-info about HW that we have to index at runtime */
static const struct granule granules[] = {
        [MMU_PAGESIZE_4KB]  = {
                .tg = SMMU__CB_TCR__VMSA8__TG0_4KB,
		/* Fig D4-15 */
                .page_bits = 12,
		/* Table D4-9 */
                .bits_per_level = 9,
#if (25 <= T0SZ) && (T0SZ <= 33)
		/* Table D4-26 (function of T0SZ) */
                .start_level = 1,
		/* Table D4-26 (function of T0SZ): len([y:lsb]) */
                .bits_in_first_level = ((37 - T0SZ) + 26) - 30 + 1,
#else
#error Driver does not support given value of T0SZ
#endif /* T0SZ */
        },
        [MMU_PAGESIZE_16KB]  = {
                .tg = SMMU__CB_TCR__VMSA8__TG0_16KB,
                .page_bits = 14,
                .bits_per_level = 11,
#if (28 <= T0SZ) && (T0SZ <= 38)
		/* Table D4-26 (function of T0SZ) */
                .start_level = 2,
		/* Table D4-26 (function of T0SZ): len([y:lsb]) */
                .bits_in_first_level = ((42 - T0SZ) + 21) - 25 + 1,
#else
#error Driver does not support given value of T0SZ
#endif /* T0SZ */
        },
        [MMU_PAGESIZE_64KB] = {
                .tg = SMMU__CB_TCR__VMSA8__TG0_64KB,
                .page_bits = 16,
                .bits_per_level = 13,
#if (22 <= T0SZ) && (T0SZ <= 34)
		/* Table D4-26 (function of T0SZ) */
                .start_level = 2,
		/* Table D4-26 (function of T0SZ): len([y:lsb]) */
                .bits_in_first_level = ((38 - T0SZ) + 25) - 29 + 1,
#else
#error Driver does not support given value of T0SZ
#endif /* T0SZ */
        },
};

static inline unsigned page_table_index(MMU_Level_t *levp, uint64_t vaddr) {
  return ((vaddr >> levp->lsb_bit) & ~(~0 << levp->idx_bits));
}

static rtems_status_code page_table_alloc(uint64_t **page_table, MMU_Context_t *context, unsigned level)
{
  assert(context);
  assert(context->mmu);

  MMU_Level_t *levp = &context->levels[level];
  MMU_500_DEBUG_PRINTF("MMU: page_table_alloc: ctx %p level %u size 0x%x align 0x%x\r\n",
         context, level, levp->page_table_size, context->granule->page_bits);

  *page_table = NULL;
  rtems_status_code sc = rtems_region_get_segment(context->region_id, levp->page_table_size, RTEMS_WAIT, RTEMS_NO_TIMEOUT, (void**)page_table);

  if (!*page_table || sc) {
    return sc;
  }
  if (!ALIGNED(*page_table, context->granule->page_bits)) {
    return RTEMS_INVALID_ADDRESS;
  }

  for (uint32_t i = 0; i < levp->page_table_entries; ++i) {
    (*page_table)[i] = ~VMSA8_DESC__VALID;
  }

  MMU_500_DEBUG_PRINTF("MMU: page_table_alloc: allocated level %u pt at %p, cleared %u entries\r\n",
         level, *page_table, levp->page_table_entries);
  return sc;
}

static rtems_status_code page_table_free(uint64_t *page_table, MMU_Context_t *context, unsigned level)
{
  assert(context);
  assert(context->mmu);
  MMU_500_DEBUG_PRINTF("MMU: page_table_free: ctx %p level %u pt %p\r\n",
         context, level, page_table);

  int rc;
  MMU_Level_t *levp = &context->levels[level];
  uint64_t *next_page_table;
  for (unsigned i = 0; i < levp->page_table_entries; ++i) {
    uint64_t desc = page_table[i];
    if (level < MAX_LEVEL_INDEX && (page_table[i] & VMSA8_DESC__VALID) &&
      ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__TABLE)) {
      next_page_table = NEXT_TABLE_PTR(desc, context->granule);
      rc = page_table_free(next_page_table, context, level + 1);
      if (rc) {
        return rc;
      }
    }
  }
  return rtems_region_return_segment(context->region_id, page_table);
}

#ifdef MMU_500_DEBUG
static void dump_page_table(MMU_Context_t *context, uint64_t *page_table, unsigned level)
{
  assert(context);
  assert(context->mmu);

  MMU_Level_t *levp = &context->levels[level];
  for (unsigned index = 0; index < levp->page_table_entries; ++index) {
    uint64_t desc = page_table[index];
    uint64_t *next_page_table = NULL;
    if (desc & VMSA8_DESC__VALID) {
      const char *type;
      if (level < MAX_LEVEL_INDEX &&
        (desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__TABLE) {
        type = "TABLE";
        next_page_table = NEXT_TABLE_PTR(desc, context->granule);
      } else if ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__PAGE) {
        type = "PAGE";
      } else if ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__BLOCK) {
        type = "BLOCK";
      } else {
        type = "INVALID";
      }
      printf("MMU:");
      for (unsigned j = 0; j < level; ++j) {
        printf("    ");
      }
      printf("L%u: %u: %p: %08x%08x [%s]\r\n",
              level, index, &page_table[index],
              (uint32_t)(desc >> 32), (uint32_t)desc,
              type);
      if (next_page_table) {
        dump_page_table(context, next_page_table, level + 1);
      }
    }
  }
}
#endif /* MMU_500_DEBUG */

rtems_status_code mmu_init(MMU_Config_t *mmu_config, volatile uint32_t *base)
{
  assert(mmu_config);
  memset(mmu_config, 0, sizeof(MMU_Config_t));
  mmu_config->base = base;
  /* Enable global fault reporting */
  GL_BASE(mmu_config)->SCR0 |= SMMU_SCR0_GFRE;
  MMU_500_DEBUG_PRINTF("MMU: initialized\r\n");
  return RTEMS_SUCCESSFUL;
}

#define FIND_ALLOCATED_ITEM(type, array, max, accessor) \
({ \
  type tmp = NULL; \
  for (int i = 0; i < max; i++) { \
    if (array[i].accessor) { \
      tmp = &(array[i]); \
      break; \
    } \
  } \
  tmp; \
})

#define CLEAR_ITEM(item) memset(item, 0, sizeof(*item))

rtems_status_code mmu_uninit(MMU_Config_t *mmu_config)
{
  assert(mmu_config);
  MMU_500_DEBUG_PRINTF("MMU: uninitialized\r\n");
  memset(&mmu_config->streams, 0, sizeof(mmu_config->streams));
  MMU_Context_t *mmu_context;
  while ((mmu_context = FIND_ALLOCATED_ITEM(MMU_Context_t *, mmu_config->contexts, MMU_500_MAX_CONTEXTS, mmu))) {
    mmu_context_destroy(mmu_context);
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_enable(MMU_Config_t *mmu_config)
{
  assert(mmu_config);
  MMU_500_DEBUG_PRINTF("MMU: enable\r\n");
  GL_BASE(mmu_config)->SCR0 &= ~SMMU_SCR0_CLIENTPD;
  for (int i = 0; i < MMU_500_MAX_CONTEXTS; i++) {
    volatile struct MMU_Context_Base *context_base = CBn_BASE(mmu_config, i);
    /* Enable context fault reporting */
    context_base->SCTLR |= SMMU_SCTLR_CFRE;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_disable(MMU_Config_t *mmu_config)
{
  assert(mmu_config);
  MMU_500_DEBUG_PRINTF("MMU: disable\r\n");
  GL_BASE(mmu_config)->SCR0 |= SMMU_SCR0_CLIENTPD;
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_interrupts_enable(MMU_Config_t *mmu_config)
{
  assert(mmu_config);
  MMU_500_DEBUG_PRINTF("MMU: interrupts enable\r\n");
  volatile struct MMU_Base *mmu_base = GL_BASE(mmu_config);
  uint32_t num_interrupts = SMMU_IDR0_NUMIRPT_GET(mmu_base->IDR0);
  if (!num_interrupts) {
    return RTEMS_UNSATISFIED;
  }
  /* Enable global fault interrupts */
  mmu_base->SCR0 |= SMMU_SCR0_GFIE;
  for (int i = 0; i < MMU_500_MAX_CONTEXTS; i++) {
    volatile struct MMU_Context_Base *context_base = CBn_BASE(mmu_config, i);
    /* Interrupt number is set on the stream */
    /* Enable context fault interrupts */
    context_base->SCTLR |= SMMU_SCTLR_CFIE;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_interrupts_disable(MMU_Config_t *mmu_config)
{
  assert(mmu_config);
  MMU_500_DEBUG_PRINTF("MMU: interrupts disable\r\n");
  volatile struct MMU_Base *mmu_base = GL_BASE(mmu_config);
  uint32_t num_interrupts = SMMU_IDR0_NUMIRPT_GET(mmu_base->IDR0);
  if (!num_interrupts) {
    return RTEMS_UNSATISFIED;
  }
  /* Disable global fault interrupts */
  mmu_base->SCR0 &= ~SMMU_SCR0_GFIE;
  for (int i = 0; i < MMU_500_MAX_CONTEXTS; i++) {
    volatile struct MMU_Context_Base *context_base = CBn_BASE(mmu_config, i);
    /* Disable context fault interrupt */
    context_base->SCTLR &= ~SMMU_SCTLR_CFIE;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_is_faulted(MMU_Config_t *mmu_config, bool *faulted)
{
  assert(mmu_config);
  assert(faulted);

  volatile struct MMU_Base *mmu_base = GL_BASE(mmu_config);
  *faulted = false;
  if (SMMU_SGFSR_FAULTS_GET(mmu_base->SGFSR)) {
    *faulted = true;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_clear_faults(MMU_Config_t *mmu_config)
{
  assert(mmu_config);

  /* Clear all faults */
  volatile struct MMU_Base *mmu_base = GL_BASE(mmu_config);
  mmu_base->SGFSR = SMMU_SGFSR_FAULTS_SET(mmu_base->SGFSR, 0x1ff);
  return RTEMS_SUCCESSFUL;
}


#define FIND_FREE_ITEM(type, array, max, accessor) \
({ \
  type tmp = NULL; \
  for (int i = 0; i < max; i++) { \
    if (!array[i].accessor) { \
      tmp = &(array[i]); \
      break; \
    } \
  } \
  tmp; \
})

rtems_status_code mmu_context_create(MMU_Context_t **mmu_context, MMU_Config_t *m, rtems_id region_id, enum mmu_pagesize page_size)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  assert(m);

  assert(page_size < sizeof(granules) / sizeof(granules[0]));
  const struct granule *g = &granules[page_size];

  MMU_500_DEBUG_PRINTF("MMU: context_create: page_size enum %u\r\n", page_size);

  /* find first unused context */
  *mmu_context = FIND_FREE_ITEM(MMU_Context_t *, m->contexts, MMU_500_MAX_CONTEXTS, mmu);
  if (!*mmu_context) {
    return RTEMS_NO_MEMORY;
  }
  (*mmu_context)->mmu = m;
  (*mmu_context)->granule = g;
  (*mmu_context)->region_id = region_id;

  MMU_500_DEBUG_PRINTF("MMU: context_create: alloced *mmu_context %p start level %u\r\n",
         *mmu_context, g->start_level);

  for (unsigned level = g->start_level; level <= MAX_LEVEL_INDEX; ++level) {
    MMU_Level_t *levp = &(*mmu_context)->levels[level];

    levp->idx_bits = (level == g->start_level) ?
            g->bits_in_first_level : g->bits_per_level;
    levp->lsb_bit = g->page_bits +
        (MAX_LEVEL_INDEX - g->start_level - (level - g->start_level)) * g->bits_per_level;

    levp->page_table_entries = 1 << levp->idx_bits;
    levp->page_table_size = levp->page_table_entries * VMSA8_DESC_BYTES;
    levp->block_size = 1 << levp->lsb_bit;

    MMU_500_DEBUG_PRINTF("MMU: context_create: level %u: pt entries %u pt size 0x%x "
           "block size 0x%x idx bits %u lsb_bit %u\r\n", level,
           levp->page_table_entries, levp->page_table_size, levp->block_size,
           levp->idx_bits, levp->lsb_bit);
  }

  sc = page_table_alloc(&(*mmu_context)->page_table, *mmu_context, (*mmu_context)->granule->start_level);
  if (sc) {
    CLEAR_ITEM(*mmu_context);
    return sc;
  }

  volatile struct MMU_Context_Base *context_base = CTX_BASE(*mmu_context);
  context_base->TCR = SMMU_TCR_T0SZ(T0SZ) | g->tg;
  context_base->TCR2 = SMMU_TCR2_PAGESIZE_40BIT;
  context_base->TTBR0 = (uint32_t)(*mmu_context)->page_table;
  context_base->SCTLR = SMMU_SCTLR_M;

  MMU_500_DEBUG_PRINTF("MMU: created *mmu_context %p pt %p entries %u size 0x%x\r\n",
         *mmu_context, (*mmu_context)->page_table,
         (*mmu_context)->levels[(*mmu_context)->granule->start_level].page_table_entries,
         (*mmu_context)->levels[(*mmu_context)->granule->start_level].page_table_size);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_context_destroy(MMU_Context_t *context)
{
  int rc;

  assert(context);
  assert(context->mmu);

  MMU_500_DEBUG_PRINTF("MMU: context_destroy: ctx %p\r\n", context);

  volatile struct MMU_Context_Base *context_base = CTX_BASE(context);
  context_base->TCR = 0;
  context_base->TCR2 = 0;
  context_base->TTBR0 = 0;
  context_base->SCTLR = 0;

  rc = page_table_free(context->page_table, context, context->granule->start_level);
  CLEAR_ITEM(context);
  return rc;
}

rtems_status_code mmu_context_is_faulted(MMU_Context_t *mmu_context, bool *faulted)
{
  assert(mmu_context);
  assert(mmu_context->mmu);
  assert(faulted);

  volatile struct MMU_Context_Base *context_base = CTX_BASE(mmu_context);
  *faulted = false;
  if (SMMU_FSR_FAULTS_GET(context_base->FSR)) {
    *faulted = true;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_context_clear_faults(MMU_Context_t *mmu_context)
{
  assert(mmu_context);
  assert(mmu_context->mmu);

  /* Clear all faults */
  volatile struct MMU_Context_Base *context_base = CTX_BASE(mmu_context);
  context_base->FSR = SMMU_FSR_FAULTS_SET(context_base->FSR, 0xff);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_map(MMU_Context_t *context, void *vaddr_page_tabler, void *paddr_page_tabler, unsigned size)
{
  assert(context);
  assert(context->mmu);
  uint64_t vaddr = (uint64_t)(uint32_t)vaddr_page_tabler;
  uint64_t paddr = (uint64_t)(uint32_t)paddr_page_tabler;

  MMU_500_DEBUG_PRINTF("MMU: map: ctx %p: 0x%08x%08x -> 0x%08x%08x size = %x\r\n",
          context, (uint32_t)(vaddr >> 32), (uint32_t)vaddr,
          (uint32_t)(paddr >> 32), (uint32_t)paddr, size);

  if (!ALIGNED(size, context->granule->page_bits)) {
    MMU_500_DEBUG_PRINTF("ERROR: MMU map: region size (0x%x) not aligned to TG of %u bits\r\n",
            size, context->granule->page_bits);
    return RTEMS_INVALID_SIZE;
  }

  /* Iterate over pages or blocks */
  do {
    MMU_500_DEBUG_PRINTF("MMU: map: vaddr %08x%08x paddr %08x%08x size 0x%x\r\n",
            (uint32_t)(vaddr >> 32), (uint32_t)vaddr,
            (uint32_t)(paddr >> 32), (uint32_t)paddr, size);

    unsigned level = context->granule->start_level;
    MMU_Level_t *levp = &context->levels[level];
    uint64_t *page_table = context->page_table;
    uint64_t *next_page_table;
    unsigned index;
    uint64_t desc;

    while (level < MAX_LEVEL_INDEX) {
      index = page_table_index(levp, vaddr);
      desc = page_table[index];
      next_page_table = NULL;

      MMU_500_DEBUG_PRINTF("MMU: map: walk: level %u pt %p: %u: %p: desc %08x%08x\r\n",
              level, page_table, index, &page_table[index],
              (uint32_t)(desc >> 32), (uint32_t)desc);

      if (ALIGNED(vaddr, levp->lsb_bit) && size >= levp->block_size) {
        MMU_500_DEBUG_PRINTF("MMU: map: walk: level %u: vaddr %08x%08x block of size 0x%x\r\n",
                level, (uint32_t)(vaddr >> 32), (uint32_t)vaddr,
                levp->block_size);
        break;
      }

      if  ((desc & VMSA8_DESC__VALID) &&
            ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__TABLE)) {
        next_page_table = NEXT_TABLE_PTR(desc, context->granule);
        MMU_500_DEBUG_PRINTF("MMU: map: walk: level %u: next pt %p\r\n", level, next_page_table);
      }

      if (!next_page_table) {
        rtems_status_code sc = page_table_alloc(&next_page_table, context, level + 1);
        if (sc) {
          return sc;
        }
        page_table[index] = (uint64_t)(uint32_t)next_page_table |
            VMSA8_DESC__VALID | VMSA8_DESC__TYPE__TABLE;
        MMU_500_DEBUG_PRINTF("MMU: map: walk: level %u: pt pointer desc: %u: %p: <- %08x%08x\r\n",
                level, index, &page_table[index],
                (uint32_t)(page_table[index] >> 32), (uint32_t)page_table[index]);
      }

      page_table = next_page_table;
      ++level;
      levp = &context->levels[level];
    }

    index = page_table_index(levp, vaddr);

    if ((page_table[index] & VMSA8_DESC__VALID)) {
      MMU_500_DEBUG_PRINTF("ERROR: MMU: overlapping/overwriting mappings not supported\r\n");
      return RTEMS_INVALID_ADDRESS;
    }

    page_table[index] = (paddr & (~0 << VMSA8_DESC__BITS)) |
        VMSA8_DESC__VALID |
        (level == MAX_LEVEL_INDEX ? VMSA8_DESC__TYPE__PAGE : VMSA8_DESC__TYPE__BLOCK) |
        VMSA8_DESC__PXN | VMSA8_DESC__XN |
        VMSA8_DESC__AP__EL0 |
        VMSA8_DESC__AP__RW |
        VMSA8_DESC__AF;

    MMU_500_DEBUG_PRINTF("MMU: map: level %u: desc: %04u: 0x%p: <- 0x%08x%08x\r\n", level, index,
            &page_table[index], (uint32_t)(page_table[index] >> 32), (uint32_t)page_table[index]);

    vaddr += levp->block_size;
    paddr += levp->block_size;
    size -= levp->block_size;
  } while (size > 0);

#ifdef MMU_500_DEBUG
  dump_page_table(context, context->page_table, context->granule->start_level);
#endif /* MMU_500_DEBUG */
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_unmap(MMU_Context_t *context, void *vaddr_page_tabler, unsigned size)
{
  assert(context);
  assert(context->mmu);
  uint64_t vaddr = (uint64_t)(uint32_t)vaddr_page_tabler;

  MMU_500_DEBUG_PRINTF("MMU: unmap: ctx %p: 0x%08x%08x size 0x%x\r\n",
          context, (uint32_t)(vaddr >> 32), (uint32_t)vaddr, size);

  if (!ALIGNED(size, context->granule->page_bits)) {
    MMU_500_DEBUG_PRINTF("ERROR: MMU unmap: region size (0x%x) not aligned to TG of %u bits\r\n",
            size, context->granule->page_bits);
    return RTEMS_INVALID_SIZE;
  }

  struct walk_step {
    uint64_t *page_table;
    unsigned index;
  };
  struct walk_step walk[MAX_LEVEL_INDEX + 1];

  /* Iterate over pages or blocks */
  do { 
    int level = context->granule->start_level;
    MMU_Level_t *levp = &context->levels[level];
    uint64_t *page_table = context->page_table;
    unsigned index;
    uint64_t desc;

    while (level < MAX_LEVEL_INDEX) {
      index = page_table_index(levp, vaddr);
      desc = page_table[index];

      MMU_500_DEBUG_PRINTF("MMU: unmap: walk: level %u pt %p %u: %p: %08x%08x\r\n",
              level, page_table, index, &page_table[index],
              (uint32_t)(desc >> 32), (uint32_t)desc);

      walk[level].page_table = page_table;
      walk[level].index = index;

      if (ALIGNED(vaddr, levp->lsb_bit) && size >= levp->block_size) {
        MMU_500_DEBUG_PRINTF("MMU: unmap: walk: level %u: pt %p: block of size 0x%x\r\n",
                level, page_table, levp->block_size);
        break;
      }

      if  ((desc & VMSA8_DESC__VALID) &&
           ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__TABLE)) {
        page_table = NEXT_TABLE_PTR(desc, context->granule);
        MMU_500_DEBUG_PRINTF("MMU: unmap: walk: level %u: next pt %p\r\n", level, page_table);
      } else {
        MMU_500_DEBUG_PRINTF("ERROR: MMU: expected page descriptor does not exist\r\n");
        return RTEMS_IO_ERROR;
      }

      ++level;
      levp = &context->levels[level];
    }

    index = page_table_index(levp, vaddr);

    if (!(page_table[index] & VMSA8_DESC__VALID)) {
      MMU_500_DEBUG_PRINTF("ERROR: MMU: expected page or block descriptor does not exist\r\n");
      return RTEMS_IO_ERROR;
    }

    MMU_500_DEBUG_PRINTF("MMU: unmap: level %u: desc: %04u: 0x%p: 0x%08x%08x\r\n", level, index,
            &page_table[index], (uint32_t)(page_table[index] >> 32), (uint32_t)page_table[index]);
    page_table[index] = ~VMSA8_DESC__VALID;
    MMU_500_DEBUG_PRINTF("MMU: unmap: level %u: desc: %04u: 0x%p: <- 0x%08x%08x\r\n", level, index,
            &page_table[index], (uint32_t)(page_table[index] >> 32), (uint32_t)page_table[index]);

    /* Walk the levels backwards and destroy empty tables */
    while (--level >= context->granule->start_level) {
      struct walk_step *step = &walk[level];

      desc = step->page_table[step->index];

      assert((desc & VMSA8_DESC__VALID) &&
              ((desc & VMSA8_DESC__TYPE_MASK) == VMSA8_DESC__TYPE__TABLE));

      uint64_t *next_page_table = NEXT_TABLE_PTR(desc, context->granule);

      MMU_500_DEBUG_PRINTF("MMU: unmap: backwalk: level %u: %04u: 0x%p: 0x%08x%08x\r\n",
              level, step->index, &step->page_table[index],
              (uint32_t)(desc >> 32), (uint32_t)desc);

      /* Child level checked for deletion */
      MMU_Level_t *back_levp = &context->levels[level + 1];
      bool page_table_empty = true;
      for (unsigned i = 0; i < back_levp->page_table_entries; ++i) {
        if (next_page_table[i] & VMSA8_DESC__VALID) {
          page_table_empty = false;
          break;
        }
      }
      if (!page_table_empty) {
        break;
      }

      step->page_table[step->index] = ~VMSA8_DESC__VALID;

      MMU_500_DEBUG_PRINTF("MMU: unmap: backwalk: level %u: %04u: 0x%p: <- 0x%08x%08x\r\n",
              level, step->index, &step->page_table[index],
              (uint32_t)(step->page_table[step->index] >> 32),
              (uint32_t)step->page_table[step->index]);

      rtems_status_code sc = page_table_free(next_page_table, context, level + 1);
      if (sc) {
        return sc;
      }
    }

    vaddr += levp->block_size;
    size -= levp->block_size;
  } while (size > 0);
#ifdef MMU_500_DEBUG
  dump_page_table(context, context->page_table, context->granule->start_level);
#endif /* MMU_500_DEBUG */
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_stream_create(MMU_Stream_t **mmu_stream, uint32_t master, MMU_Context_t *context)
{
  assert(context);
  assert(context->mmu);

  MMU_Config_t *m = context->mmu;
  MMU_Stream_t *s = FIND_FREE_ITEM(MMU_Stream_t *, m->streams, MMU_500_MAX_STREAMS, context);
  if (!s) {
    return RTEMS_NO_MEMORY;
  }
  *mmu_stream = s;
  s->context = context;

  volatile struct MMU_Stream_Base *stream_base = ST_BASE(s);
  stream_base->SMR = SMMU_SMR_VALID | master;
  stream_base->S2CR = SMMU_S2CR_EXIDVALID | CB_INDEX(context);
  stream_base->CBAR = SMMU_CBAR_TYPE_STAGE1_WITH_STAGE2_BYPASS | SMMU_CBAR_IRPTNDX(CB_INDEX(context)+1);
  stream_base->CBA2R = SMMU_CBA2R_VA64;

  MMU_500_DEBUG_PRINTF("MMU: created stream %p: 0x%x -> ctx %p\r\n",
         s, master, context);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code mmu_stream_destroy(MMU_Stream_t *s)
{
  assert(s);
  assert(s->context);
  assert(s->context->mmu);

  MMU_500_DEBUG_PRINTF("MMU: destroy stream %p\r\n", s);

  volatile struct MMU_Stream_Base *stream_base = ST_BASE(s);
  stream_base->SMR = 0;
  stream_base->S2CR = 0;
  stream_base->CBAR = 0;
  stream_base->CBA2R = 0;

  CLEAR_ITEM(s);
  return RTEMS_SUCCESSFUL;
}


