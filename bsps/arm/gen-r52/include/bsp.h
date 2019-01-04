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

void arm_generic_timer_get_config(uint32_t *frequency, uint32_t *irq);

// XXX This BSP is currently setup with a GICv1 GIC
// Give it something so it compiles
#define BSP_ARM_GIC_CPUIF_BASE ( 0x00000000 )
#define BSP_ARM_GIC_DIST_BASE ( 0x00000000 )

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ASM */

#endif /* LIBBSP_ARM_GEN_R52_BSP_H */
