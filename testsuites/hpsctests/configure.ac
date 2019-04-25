## Process this file with autoconf to produce a configure script.

AC_PREREQ([2.69])
AC_INIT([rtems-c-src-tests-hpsctests],[_RTEMS_VERSION],[https://devel.rtems.org/newticket])
AC_CONFIG_SRCDIR([dma_copy_from_zero])
AC_CONFIG_HEADER([config.h])
RTEMS_TOP([../..],[..])
RTEMS_SOURCE_TOP
RTEMS_BUILD_TOP

RTEMS_CANONICAL_TARGET_CPU

AM_INIT_AUTOMAKE([no-define foreign subdir-objects 1.12.2])
AM_MAINTAINER_MODE

RTEMS_ENABLE_CXX

RTEMS_ENV_RTEMSBSP

RTEMS_PROJECT_ROOT

RTEMS_PROG_CC_FOR_TARGET

RTEMS_PROG_CXX_FOR_TARGET
RTEMS_CANONICALIZE_TOOLS

# BSP Test configuration
RTEMS_TEST_CHECK([dma_copy_from_zero])
RTEMS_TEST_CHECK([dma_copy])
RTEMS_TEST_CHECK([dma_copy_int])
RTEMS_TEST_CHECK([dma_any_channel])
RTEMS_TEST_CHECK([dma_copy_to_register])
RTEMS_TEST_CHECK([dma_copy_from_register])
RTEMS_TEST_CHECK([dma_copy_hpsc_bounds])
RTEMS_TEST_CHECK([mmu_map])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
