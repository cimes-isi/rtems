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

#ifndef LIBBSP_ARM_GEN_R52_BSP_H
#define LIBBSP_ARM_GEN_R52_BSP_H

#include <bspopts.h>

#define BSP_FEATURE_IRQ_EXTENSION

#define BSP_FDT_IS_SUPPORTED

#ifndef ASM

#include <rtems.h>

#include <bsp/default-initial-extension.h>
#include <bsp/hwinfo.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void arm_generic_timer_get_config(uint32_t *frequency, uint32_t *irq);

#define BSP_ARM_GIC_DIST_BASE ( RTPS_GIC_BASE + 0x00000000 )
#define BSP_ARM_GIC_REDIST_BASE ( RTPS_GIC_BASE + 0x00040000 )

#include <bsp/dma_330.h>
#define BSP_DMA_MAX_CHANNELS	8
#define BSP_DMA_BASE		RTPS_DMA_BASE
#define BSP_DMA_ABORT_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_ABORT )
#define BSP_DMA_CHANNEL_0_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV0 )
#define BSP_DMA_CHANNEL_1_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV1 )
#define BSP_DMA_CHANNEL_2_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV2 )
#define BSP_DMA_CHANNEL_3_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV3 )
#define BSP_DMA_CHANNEL_4_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV4 )
#define BSP_DMA_CHANNEL_5_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV5 )
#define BSP_DMA_CHANNEL_6_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV6 )
#define BSP_DMA_CHANNEL_7_IRQ	( GIC_INTERNAL + RTPS_IRQ__RTPS_DMA_EV7 )

#define MMU_500_QEMU 1
#define MMU_500_MAX_CONTEXTS 8
#define MMU_500_MAX_STREAMS 8

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

#endif /* LIBBSP_ARM_GEN_R52_BSP_H */
