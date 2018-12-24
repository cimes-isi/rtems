/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
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

// XXX This BSP is currently setup with a GICv1 GIC
// Give it something so it compiles
#define BSP_ARM_GIC_CPUIF_BASE ( 0x00000000 )
#define BSP_ARM_GIC_DIST_BASE ( 0x00000000 )

// XXX This BSP is currently setup for ARMv7 memory-mapped timer
// Give it something so it compiles
#define BSP_ARM_A9MPCORE_GT_BASE 0x30A05000
#define BSP_ARM_A9MPCORE_SCU_BASE 0x30A05000

// Addresses of the timer instances
#define BSP_ARM_A9MPCORE_GT_BASE_0 0x30A05000
#define BSP_ARM_A9MPCORE_GT_BASE_1 0x30A06000


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

#endif /* LIBBSP_ARM_GEN_R52_BSP_H */
