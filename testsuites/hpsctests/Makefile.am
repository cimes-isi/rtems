#
# Samples
#

ACLOCAL_AMFLAGS = -I ../aclocal

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am

hpsctests =
hpsc_screens =
hpsc_docs =

support_includes = -I$(top_srcdir)/../support/include

if TEST_dma_copy_from_zero
hpsctests += dma_copy_from_zero
hpsc_screens += dma_copy_from_zero/dma_copy_from_zero.scn
hpsc_docs += dma_copy_from_zero/dma_copy_from_zero.doc
dma_copy_from_zero_SOURCES = dma_copy_from_zero/init.c dma_copy_from_zero/system.h
dma_copy_from_zero_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy_from_zero) \
	$(support_includes)
endif

if TEST_dma_copy
hpsctests += dma_copy
hpsc_screens += dma_copy/dma_copy.scn
hpsc_docs += dma_copy/dma_copy.doc
dma_copy_SOURCES = dma_copy/init.c dma_copy/system.h
dma_copy_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy) \
	$(support_includes)
endif

if TEST_dma_copy_int
hpsctests += dma_copy_int
hpsc_screens += dma_copy_int/dma_copy_int.scn
hpsc_docs += dma_copy_int/dma_copy_int.doc
dma_copy_int_SOURCES = dma_copy_int/init.c dma_copy_int/system.h
dma_copy_int_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy_int) \
	$(support_includes)
endif

if TEST_dma_any_channel
hpsctests += dma_any_channel
hpsc_screens += dma_any_channel/dma_any_channel.scn
hpsc_docs += dma_any_channel/dma_any_channel.doc
dma_any_channel_SOURCES = dma_any_channel/init.c dma_any_channel/system.h
dma_any_channel_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_any_channel) \
	$(support_includes)
endif

if TEST_dma_copy_to_register
hpsctests += dma_copy_to_register
hpsc_screens += dma_copy_to_register/dma_copy_to_register.scn
hpsc_docs += dma_copy_to_register/dma_copy_to_register.doc
dma_copy_to_register_SOURCES = dma_copy_to_register/init.c dma_copy_to_register/system.h
dma_copy_to_register_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy_to_register) \
	$(support_includes)
endif

if TEST_dma_copy_from_register
hpsctests += dma_copy_from_register
hpsc_screens += dma_copy_from_register/dma_copy_from_register.scn
hpsc_docs += dma_copy_from_register/dma_copy_from_register.doc
dma_copy_from_register_SOURCES = dma_copy_from_register/init.c dma_copy_from_register/system.h
dma_copy_from_register_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy_from_register) \
	$(support_includes)
endif

if TEST_dma_copy_hpsc_bounds
hpsctests += dma_copy_hpsc_bounds
hpsc_screens += dma_copy_hpsc_bounds/dma_copy_hpsc_bounds.scn
hpsc_docs += dma_copy_hpsc_bounds/dma_copy_hpsc_bounds.doc
dma_copy_hpsc_bounds_SOURCES = dma_copy_hpsc_bounds/init.c dma_copy_hpsc_bounds/system.h
dma_copy_hpsc_bounds_CPPFLAGS = $(AM_CPPFLAGS) $(TEST_FLAGS_dma_copy_hpsc_bounds) \
	$(support_includes)
endif

noinst_PROGRAMS = $(hpsctests)
