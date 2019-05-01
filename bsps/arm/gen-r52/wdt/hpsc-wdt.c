#define DEBUG 0

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <bsp.h>
#include <bsp/hpsc-wdt.h>
#include <bsp/utility.h>

#ifdef HPSC_WDT_DEBUG
# define HPSC_WDT_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
# define HPSC_WDT_DEBUG_PRINTF(...)
#endif

/* re-configurable by a macro in Qemu model (requires Qemu rebuild) */
#define MAX_STAGES 2

struct WDT_Stage_Base {
  uint64_t TERMINAL;
  uint64_t COUNT;
};

struct WDT_Base {
  struct WDT_Stage_Base stages[MAX_STAGES];
  uint32_t CONFIG;
#define HPSC_WDT_CONFIG_TICKDIV(val) BSP_FLD32(val, 2, 8)
#define HPSC_WDT_CONFIG_TICKDIV_GET(reg) BSP_FLD32GET(reg, 2, 8)
#define HPSC_WDT_CONFIG_TICKDIV_SET(reg, val) BSP_FLD32SET(reg, val, 2, 8)
#define HPSC_WDT_CONFIG_ENABLE BSP_BIT32(0)
  /* Each bit of status is the timeout enable for that stage */
  uint32_t STATUS;
  uint32_t CMD_ARM;
  uint32_t CMD_FIRE;
};

/* The split by command type is for driver implementation convenience, in
 * hardware, there is only one command type. */

enum cmd {
  CMD_DISABLE = 0,
  NUM_CMDS,
};

enum stage_cmd {
  SCMD_CAPTURE = 0,
  SCMD_LOAD,
  SCMD_CLEAR,
  NUM_SCMDS,
};

struct cmd_code {
  uint32_t arm;
  uint32_t fire;
};

/* Store the codes in RO memory section. From fault tolerance perspective,
 * there is not a big difference with storing them in instruction memory (i.e.
 * by using them as immediate values) -- and the compiler is actually free to
 * do so. Both are protected by the MPU, and an upset in both will be detected
 * by ECC. In both cases, the value can end up in the cache (either I or D,
 * both of which have ECC). Also, in either case, the code will end up in a
 * register (which is protected by TMR, in case of TRCH). */


static const struct cmd_code stage_cmd_codes[][NUM_SCMDS] = {
  [0] = {
    [SCMD_CAPTURE] = { 0xCD01, 0x01CD },
    [SCMD_LOAD] =    { 0xCD03, 0x03CD },
    [SCMD_CLEAR] =   { 0xCD05, 0x05CD },
  },
  [1] = {
    [SCMD_CAPTURE] = { 0xCD02, 0x02CD },
    [SCMD_LOAD] =    { 0xCD04, 0x04CD },
    [SCMD_CLEAR] =   { 0xCD06, 0x06CD },
  },
};

static const struct cmd_code cmd_codes[NUM_CMDS] = {
  [CMD_DISABLE] = { 0xCD07, 0x07CD },
};

static void exec_cmd(struct HPSC_WDT_Config *wdt, const struct cmd_code *code)
{
  wdt->base->CMD_ARM = code->arm;
  wdt->base->CMD_FIRE = code->fire;
}

static void exec_global_cmd(struct HPSC_WDT_Config *wdt, enum cmd cmd)
{
  HPSC_WDT_DEBUG_PRINTF("WDT %s: exec cmd: %u\r\n", wdt->name, cmd);
  assert(cmd < NUM_CMDS);
  exec_cmd(wdt, &cmd_codes[cmd]);
}
static void exec_stage_cmd(struct HPSC_WDT_Config *wdt, enum stage_cmd scmd, unsigned stage)
{
  HPSC_WDT_DEBUG_PRINTF("WDT %s: stage %u: exec stage cmd: %u\r\n", wdt->name, stage, scmd);
  assert(stage < MAX_STAGES);
  assert(scmd < NUM_SCMDS);
  exec_cmd(wdt, &stage_cmd_codes[stage][scmd]);
}

static void wdt_init(struct HPSC_WDT_Config *wdt, const char *name, volatile uint32_t *base,
                     rtems_vector_number vec)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: init base %p\r\n", name, base);

  wdt->base = (volatile struct WDT_Base *)base;
  wdt->name = name;
  wdt->monitor = false;
  wdt->vec = vec;
}

static unsigned log2_of_pow2(unsigned long v)
{
    assert(v);
    int b = 1;
    while ((v & 0x1) == 0x0) {
        v >>= 1;
        b++;
    }
    /* power of 2 */
    assert(v == 0x1);
    return b;
}

void wdt_init_monitor(struct HPSC_WDT_Config *wdt, const char *name, volatile uint32_t *base,
                      rtems_vector_number vec, uint32_t clk_freq_hz, unsigned max_div)
{
  wdt_init(wdt, name, base, vec);
  wdt->monitor = true;
  wdt->clk_freq_hz = clk_freq_hz;
  wdt->max_div = max_div;
  wdt->counter_width = (64 - log2_of_pow2(wdt->max_div) - 1);
}

void wdt_init_target(struct HPSC_WDT_Config *wdt, const char *name, volatile uint32_t *base,
                     rtems_vector_number vec)
{
  wdt_init(wdt, name, base, vec);
}

rtems_status_code wdt_configure(struct HPSC_WDT_Config *wdt, unsigned freq,
                unsigned num_stages, uint64_t *timeouts)
{
  assert(wdt);
  assert(wdt->monitor);
  /* not strict requirement, but for sanity */
  assert(!wdt_is_enabled(wdt));
  if (num_stages > MAX_STAGES) {
    HPSC_WDT_DEBUG_PRINTF("ERROR: WDT: more stages than supported: %u >= %u\r\n",
           num_stages, MAX_STAGES);
    return RTEMS_INVALID_NUMBER;
  }
  if (!(freq <= wdt->clk_freq_hz && wdt->clk_freq_hz % freq == 0)) {
    HPSC_WDT_DEBUG_PRINTF("ERROR: WDT: freq is larger than or not a divisor of clk freq: %u > %u\r\n",
           freq, wdt->clk_freq_hz);
    return RTEMS_INVALID_NUMBER;
  }
  for (unsigned stage = 0; stage < num_stages; ++stage) {
    if (timeouts[stage] & (~0ULL << wdt->counter_width)) {
      HPSC_WDT_DEBUG_PRINTF("ERROR: WDT: timeout for stage %u exceeds counter width (%u bits): %08x%08x\r\n",
             stage, wdt->counter_width,
            (uint32_t)(timeouts[stage] >> 32), (uint32_t)(timeouts[stage] & 0xffffffff));
      return RTEMS_INVALID_NUMBER;
    }
  }

  /* Set divider and zero other fields */
  assert(wdt->clk_freq_hz % freq == 0);
  unsigned div = wdt->clk_freq_hz / freq;
  if (div > wdt->max_div) {
    HPSC_WDT_DEBUG_PRINTF("ERROR: WDT: divider too large: %u > %u\r\n",
           div, wdt->max_div);
    return RTEMS_INVALID_NUMBER;
  }

  HPSC_WDT_DEBUG_PRINTF("WDT %s: set divider to %u\r\n", wdt->name, div);
  wdt->base->CONFIG = HPSC_WDT_CONFIG_TICKDIV(div);

  for (unsigned stage = 0; stage < num_stages; ++stage) {
    /* Loading alone does not clear the current count (which may have been
     * left over if timer previously enabled and disabled). */
    exec_stage_cmd(wdt, SCMD_CLEAR, stage);

    wdt->base->stages[stage].TERMINAL = timeouts[stage];
    exec_stage_cmd(wdt, SCMD_LOAD, stage);
  }
  return RTEMS_SUCCESSFUL;
}

void wdt_uninit(struct HPSC_WDT_Config *wdt)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: destroy\r\n", wdt->name);
  if (wdt->monitor) {
    assert(!wdt_is_enabled(wdt));
  }
  memset(wdt, 0, sizeof(*wdt));
}

uint64_t wdt_count(struct HPSC_WDT_Config *wdt, unsigned stage)
{
  assert(wdt);
  exec_stage_cmd(wdt, SCMD_CAPTURE, stage);
  uint64_t count = wdt->base->stages[stage].COUNT;
  HPSC_WDT_DEBUG_PRINTF("WDT %s: count -> 0x%08x%08x\r\n", wdt->name,
         (uint32_t)(count >> 32), (uint32_t)(count & 0xffffffff));
  return count;
}

uint64_t wdt_timeout(struct HPSC_WDT_Config *wdt, unsigned stage)
{
  assert(wdt);
  /* NOTE: not going to be the right value if it wasn't not loaded via cmd */
  uint64_t terminal = wdt->base->stages[stage].TERMINAL;
  HPSC_WDT_DEBUG_PRINTF("WDT %s: terminal -> 0x%08x%08x\r\n", wdt->name,
         (uint32_t)(terminal >> 32), (uint32_t)(terminal & 0xffffffff));
  return terminal;
}

void wdt_timeout_clear(struct HPSC_WDT_Config *wdt, unsigned stage)
{
  assert(wdt);
  /* TODO: spec unclear: if we are not allowed to clear the int source, then
   * we have to disable the interrupt via the interrupt controller, and
   * re-enable it in wdt_enable. */
  wdt->base->STATUS &= ~(1 << stage);
}

bool wdt_is_enabled(struct HPSC_WDT_Config *wdt)
{
  bool enabled = wdt->base->CONFIG & HPSC_WDT_CONFIG_ENABLE;
  HPSC_WDT_DEBUG_PRINTF("WDT %s: is enabled -> %u\r\n", wdt->name, enabled);
  return enabled;
}

rtems_status_code wdt_handler_install(struct HPSC_WDT_Config *wdt, rtems_interrupt_handler cb, void *cb_arg)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: Install ISR\r\n", wdt->name);
  return rtems_interrupt_handler_install(wdt->vec, wdt->name, RTEMS_INTERRUPT_UNIQUE, cb, cb_arg);
}

void wdt_enable(struct HPSC_WDT_Config *wdt)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: Enable\r\n", wdt->name);
  wdt->base->CONFIG |= HPSC_WDT_CONFIG_ENABLE;
}

rtems_status_code wdt_handler_remove(struct HPSC_WDT_Config *wdt, rtems_interrupt_handler cb, void *cb_arg)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: Remove ISR\r\n", wdt->name);
  return rtems_interrupt_handler_remove(wdt->vec, cb, cb_arg);
}

void wdt_disable(struct HPSC_WDT_Config *wdt)
{
  assert(wdt);
  assert(wdt->monitor);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: Disable\r\n", wdt->name);
  exec_global_cmd(wdt, CMD_DISABLE);
}

void wdt_kick(struct HPSC_WDT_Config *wdt)
{
  assert(wdt);
  HPSC_WDT_DEBUG_PRINTF("WDT %s: kick\r\n", wdt->name);
  /* In Concept A variant, there is only a clear for stage 0.  In Concept B
   * variant, there's a clear for each stage, but it is suffient to clear the
   * first stage, because that action has to stop the timers for downstream
   * stages in HW, according to the current interpretation of the HW spec. */
  exec_stage_cmd(wdt, SCMD_CLEAR, /* stage */ 0);
}
