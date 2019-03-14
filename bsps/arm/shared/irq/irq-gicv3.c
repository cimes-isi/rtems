/*
 *  COPYRIGHT (c) 1989-2019.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/arm-gic.h>

#include <rtems/score/armv4.h>

#include <libcpu/arm-cp15.h>

#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/start.h>

#define PRIORITY_DEFAULT 127

/* cpuif->iccicr */
#define ICC_CTLR    "p15, 0, %0, c12, c12, 4"

/* cpuif->iccpmr */
#define ICC_PMR     "p15, 0, %0,  c4,  c6, 0"

/* cpuif->iccbpr */
#define ICC_BPR0    "p15, 0, %0, c12,  c8, 3"
#define ICC_BPR1    "p15, 0, %0, c12, c12, 3"

/* cpuif->icciar */
#define ICC_IAR0    "p15, 0, %0, c12,  c8, 0"
#define ICC_IAR1    "p15, 0, %0, c12, c12, 0"

/* cpuif->icceoir */
#define ICC_EOIR0   "p15, 0, %0, c12,  c8, 1"
#define ICC_EOIR1   "p15, 0, %0, c12, c12, 1"

#define ICC_SRE     "p15, 0, %0, c12, c12, 5"

#define ICC_IGRPEN0 "p15, 0, %0, c12, c12, 6"
#define ICC_IGRPEN1 "p15, 0, %0, c12, c12, 7"

#define READ_SR(SR_NAME) \
({ \
  uint32_t value; \
  __asm__ volatile("mrc    " SR_NAME : "=r" (value) ); \
  value; \
})

#define WRITE_SR(SR_NAME, VALUE) \
    __asm__ volatile("mcr    " SR_NAME "  \n" : : "r" (VALUE) );

#define ARM_GIC_REDIST ((volatile gic_redist *) BSP_ARM_GIC_REDIST_BASE)
#define ARM_GIC_SGI_PPI (((volatile gic_sgi_ppi *) ((char*)BSP_ARM_GIC_REDIST_BASE + (1 << 16))))

void bsp_interrupt_dispatch(void)
{
  uint32_t icciar = READ_SR(ICC_IAR1);
  rtems_vector_number vector = GIC_CPUIF_ICCIAR_ACKINTID_GET(icciar);
  rtems_vector_number spurious = 1023;

  if (vector != spurious) {
    uint32_t psr = _ARMV4_Status_irq_enable();
    bsp_interrupt_handler_dispatch(vector);

    _ARMV4_Status_restore(psr);

    WRITE_SR(ICC_EOIR1, icciar);
  }
}

void bsp_interrupt_vector_enable(rtems_vector_number vector)
{
  volatile gic_dist *dist = ARM_GIC_DIST;
  volatile gic_sgi_ppi *sgi_ppi = ARM_GIC_SGI_PPI;

  bsp_interrupt_assert(bsp_interrupt_is_valid_vector(vector));

  /* TODO(kmoore) This could use some cleanup and integration
   * Vectors below 32 are currently routed through the redistributor */
  if (vector >= 32) {
    gic_id_enable(dist, vector);
  } else {
    sgi_ppi->icspiser[0] = 1 << (vector % 32);
  }
}

void bsp_interrupt_vector_disable(rtems_vector_number vector)
{
  volatile gic_dist *dist = ARM_GIC_DIST;
  volatile gic_sgi_ppi *sgi_ppi = ARM_GIC_SGI_PPI;

  bsp_interrupt_assert(bsp_interrupt_is_valid_vector(vector));

  if (vector >= 32) {
    gic_id_disable(dist, vector);
  } else {
    sgi_ppi->icspicer[0] = 1 << (vector % 32);
  }
}

static inline uint32_t get_id_count(volatile gic_dist *dist)
{
  uint32_t id_count = GIC_DIST_ICDICTR_IT_LINES_NUMBER_GET(dist->icdictr);

  id_count = 32 * (id_count + 1);
  id_count = id_count <= 1020 ? id_count : 1020;

  return id_count;
}

static void init_cpu_interface(void)
{
  uint32_t sre_value = 0x7;
  WRITE_SR(ICC_SRE, sre_value);
  WRITE_SR(ICC_PMR, GIC_CPUIF_ICCPMR_PRIORITY(0xff));
  WRITE_SR(ICC_BPR0, GIC_CPUIF_ICCBPR_BINARY_POINT(0x0));

  volatile gic_redist *redist = ARM_GIC_REDIST;
  uint32_t waker = redist->icrwaker;
  uint32_t waker_mask = GIC_DIST_ICRWAKER_PROCESSOR_SLEEP;
  waker &= ~waker_mask;
  redist->icrwaker = waker;

  /* Set interrupt group to 1NS for SGI/PPI interrupts routed through the redistributor */
  volatile gic_sgi_ppi *sgi_ppi = ARM_GIC_SGI_PPI;
  sgi_ppi->icspigrpr[0] = 0xffffffff;
  sgi_ppi->icspigrpmodr[0] = 0;

  /* Enable interrupt groups 0 and 1 */
  WRITE_SR(ICC_IGRPEN0, 0x1);
  WRITE_SR(ICC_IGRPEN1, 0x1);
  WRITE_SR(ICC_CTLR, 0x0);
}

rtems_status_code bsp_interrupt_facility_initialize(void)
{
  volatile gic_dist *dist = ARM_GIC_DIST;
  uint32_t id_count = get_id_count(dist);
  uint32_t id;

  arm_cp15_set_exception_handler(
    ARM_EXCEPTION_IRQ,
    _ARMV4_Exception_interrupt
  );

  dist->icddcr = GIC_DIST_ICDDCR_ARE_NS | GIC_DIST_ICDDCR_ARE_S
               | GIC_DIST_ICDDCR_ENABLE_GRP1S | GIC_DIST_ICDDCR_ENABLE_GRP1NS
               | GIC_DIST_ICDDCR_ENABLE_GRP0;

  for (id = 0; id < id_count; id += 32) {
    /* Disable all interrupts */
    dist->icdicer[id / 32] = 0xffffffff;

    /* Set interrupt group to 1NS for all interrupts */
    dist->icdigr[id / 32] = 0xffffffff;
    dist->icdigmr[id / 32] = 0;
  }

  for (id = 0; id < id_count; ++id) {
    gic_id_set_priority(dist, id, PRIORITY_DEFAULT);
  }

  for (id = 32; id < id_count; ++id) {
    gic_id_set_targets(dist, id, 0x01);
  }

  init_cpu_interface();
  return RTEMS_SUCCESSFUL;
}

#ifdef RTEMS_SMP
BSP_START_TEXT_SECTION void arm_gic_irq_initialize_secondary_cpu(void)
{
  volatile gic_dist *dist = ARM_GIC_DIST;

  while ((dist->icddcr & GIC_DIST_ICDDCR_ENABLE) == 0) {
    /* Wait */
  }

#error modify init_cpu_interface to use correct offsets for each CPU
  init_cpu_interface();
}
#endif

rtems_status_code arm_gic_irq_set_priority(
  rtems_vector_number vector,
  uint8_t priority
)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if (bsp_interrupt_is_valid_vector(vector)) {
    volatile gic_dist *dist = ARM_GIC_DIST;

    gic_id_set_priority(dist, vector, priority);
  } else {
    sc = RTEMS_INVALID_ID;
  }

  return sc;
}

rtems_status_code arm_gic_irq_get_priority(
  rtems_vector_number vector,
  uint8_t *priority
)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if (bsp_interrupt_is_valid_vector(vector)) {
    volatile gic_dist *dist = ARM_GIC_DIST;

    *priority = gic_id_get_priority(dist, vector);
  } else {
    sc = RTEMS_INVALID_ID;
  }

  return sc;
}

void bsp_interrupt_set_affinity(
  rtems_vector_number vector,
  const Processor_mask *affinity
)
{
  volatile gic_dist *dist = ARM_GIC_DIST;
  uint8_t targets = (uint8_t) _Processor_mask_To_uint32_t(affinity, 0);

  gic_id_set_targets(dist, vector, targets);
}

void bsp_interrupt_get_affinity(
  rtems_vector_number vector,
  Processor_mask *affinity
)
{
  volatile gic_dist *dist = ARM_GIC_DIST;
  uint8_t targets = gic_id_get_targets(dist, vector);

  _Processor_mask_From_uint32_t(affinity, targets, 0);
}
