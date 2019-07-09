##
#
# @file
#
# @brief Makefile of LibBSP for the Cortex-R52 platform.
#

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/gen-r52/start/bsp_specs

start.$(OBJEXT): ../../../../../../bsps/arm/shared/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
project_lib_DATA = start.$(OBJEXT)

project_lib_DATA += linkcmds
dist_project_lib_DATA += ../../../../../../bsps/arm/gen-r52/start/linkcmds


hpsc-arch.c: ../../../../../../bsps/arm/gen-r52/start/hpsc-arch.dts
	arm-rtems5-gcc -E -nostdinc -I$(<D)/../include/bsp -x assembler-with-cpp -o - $< |dtc ${DTC_ALIGN} -O asm > hpsc-arch.s
	arm-rtems5-as hpsc-arch.s -o hpsc-arch.o
	arm-rtems5-objcopy -O binary hpsc-arch.o hpsc-arch.bin
	rtems-bin2c hpsc-arch.bin hpsc-arch.c
project_lib_DATA += hpsc-arch.c

project_lib_LIBRARIES = librtemsbsp.a

librtemsbsp_a_SOURCES =
librtemsbsp_a_CPPFLAGS = $(AM_CPPFLAGS)

CFLAGS += -Wno-missing-prototypes

# Shared
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/btimer/btimer-stub.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bsp-fdt.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/stackalloc.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-memcpy.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cp15/arm-cp15-set-exception-handler.c

# Startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspclean.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspgetworkarea.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspreset.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspstart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspstarthooks.c
librtemsbsp_a_SOURCES += hpsc-arch.c
if HAS_SMP
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/arm-a9mpcore-smp.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/start/bspsmp.c
endif

# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/irq/irq-default-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/irq/irq-gicv3.c

# Console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/console-termios-init.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/console-termios.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/console/console-config.c

# Clock
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/clock/clock-generic-timer.c

# DMA
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/dma/dma_330.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/dma/alt_dma.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/dma/alt_dma_program.c

# MMU
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/mmu/mmu_500.c

# SMC
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/smc/smc_353.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xnandps_bbm.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xnandps.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xnandps_onfi.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xil_io.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xil_assert.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/xilinx_smc/xil_printf.c

# WDT
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/wdt/hpsc-wdt.c

# SRAM FS Driver
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/gen-r52/smc/sramfs.c

###############################################################################
#                  Special Rules                                              #
###############################################################################


include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/gen-r52/headers.am
