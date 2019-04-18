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

#ifndef __MPU_H__
#define __MPU_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define DISABLE_MPU() \
  { \
    uint32_t ctrl = arm_cp15_get_control(); \
    ctrl &= ~ARM_CP15_CTRL_M; \
    arm_cp15_set_control(ctrl); \
  }

#define ENABLE_MPU() \
  { \
    uint32_t ctrl = arm_cp15_get_control(); \
    ctrl |= ARM_CP15_CTRL_M; \
    arm_cp15_set_control(ctrl); \
  }

#define ATTR_RO_Access     (0x6)
#define ATTR_RW_Access     (0x2)
#define ATTR_Execute_Never (0x1)
#define MMU_Region_Enable  (0x1)

#define MPU_REGION_01(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs,  c8, 0, 1)
#define MPU_REGION_02(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs,  c8, 4, 5)
#define MPU_REGION_03(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs,  c9, 0, 1)
#define MPU_REGION_04(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs,  c9, 4, 5)
#define MPU_REGION_05(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c10, 0, 1)
#define MPU_REGION_06(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c10, 4, 5)
#define MPU_REGION_07(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c11, 0, 1)
#define MPU_REGION_08(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c11, 4, 5)
#define MPU_REGION_09(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c12, 0, 1)
#define MPU_REGION_10(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c12, 4, 5)
#define MPU_REGION_11(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c13, 0, 1)
#define MPU_REGION_12(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c13, 4, 5)
#define MPU_REGION_13(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c14, 0, 1)
#define MPU_REGION_14(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c14, 4, 5)
#define MPU_REGION_15(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c15, 0, 1)
#define MPU_REGION_16(_start, _end, _attrs) MPU_REGION(_start, _end, _attrs, c15, 4, 5)

#define MPU_REGION(_start, _end, _attrs, _CRm, _op2_b, _op2_l) \
  { \
    uint32_t _base = ((uint32_t)_start) | _attrs; \
    __asm__ volatile("" \
      "mcr    p15,0,%0,c6," #_CRm "," #_op2_b "  \n" \
      : \
      : "r" (_base) \
    ); \
    uint32_t _limit = ((uint32_t)_end); \
    /* Clear the lowest 6 bits */ \
    _limit &= ~0x3f; \
    /* Set MMU Region Enable bit */ \
    _limit |= MMU_Region_Enable; \
    __asm__ volatile("" \
      "mcr    p15,0,%0,c6," #_CRm "," #_op2_l "  \n" \
      : \
      : "r" (_limit) \
    ); \
  }

#define MPU_REGION_01_DISABLE() MPU_REGION_DISABLE( c8, 1)
#define MPU_REGION_02_DISABLE() MPU_REGION_DISABLE( c8, 5)
#define MPU_REGION_03_DISABLE() MPU_REGION_DISABLE( c9, 1)
#define MPU_REGION_04_DISABLE() MPU_REGION_DISABLE( c9, 5)
#define MPU_REGION_05_DISABLE() MPU_REGION_DISABLE(c10, 1)
#define MPU_REGION_06_DISABLE() MPU_REGION_DISABLE(c10, 5)
#define MPU_REGION_07_DISABLE() MPU_REGION_DISABLE(c11, 1)
#define MPU_REGION_08_DISABLE() MPU_REGION_DISABLE(c11, 5)
#define MPU_REGION_09_DISABLE() MPU_REGION_DISABLE(c12, 1)
#define MPU_REGION_10_DISABLE() MPU_REGION_DISABLE(c12, 5)
#define MPU_REGION_11_DISABLE() MPU_REGION_DISABLE(c13, 1)
#define MPU_REGION_12_DISABLE() MPU_REGION_DISABLE(c13, 5)
#define MPU_REGION_13_DISABLE() MPU_REGION_DISABLE(c14, 1)
#define MPU_REGION_14_DISABLE() MPU_REGION_DISABLE(c14, 5)
#define MPU_REGION_15_DISABLE() MPU_REGION_DISABLE(c15, 1)
#define MPU_REGION_16_DISABLE() MPU_REGION_DISABLE(c15, 5)

#define MPU_REGION_DISABLE(_CRm, _op2_l) \
  { \
    uint32_t end = 0; \
    __asm__ volatile("" \
      "mcr    p15,0,%0,c6," #_CRm "," #_op2_l "  \n" \
      : \
      : "r" (end) \
    ); \
  }

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __MPU_H__ */
