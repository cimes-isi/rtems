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
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>

#include <bsp.h>
#include <bsp/hwinfo.h>
#include <bsp/smc_353.h>
#include <bsp/utility.h>

#ifdef SMC_353_SRAM_DEBUG
# define SMC_353_SRAM_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
# define SMC_353_SRAM_DEBUG_PRINTF(...)
#endif

/* Must be 0x20 bytes */
typedef struct SMC_353_Block_Base_s {
  uint32_t cycles;
  uint32_t opmode;
  uint32_t unused_8[6];
} SMC_353_Block_Base_t;

/* Must be 0x100 bytes */
typedef struct SMC_353_ECC_Base_s {
  uint32_t status;
#define SMC_353_ECC_STATUS_BUSY BSP_BIT32(6)
  uint32_t config;
#define SMC_353_ECC_CONFIG_ECC_EXTRA_BLOCK_SIZE(val) BSP_FLD32(val, 11, 12)
#define SMC_353_ECC_CONFIG_ECC_EXTRA_BLOCK_SIZE_GET(reg) BSP_FLD32GET(reg, 11, 12)
#define SMC_353_ECC_CONFIG_ECC_EXTRA_BLOCK_SIZE_SET(reg, val) BSP_FLD32SET(reg, val, 11, 12)
#define SMC_353_ECC_CONFIG_ECC_EXTRA_BLOCK BSP_BIT32(10)
#define SMC_353_ECC_CONFIG_ECC_INT_ABORT BSP_BIT32(9)
#define SMC_353_ECC_CONFIG_ECC_INT_PASS BSP_BIT32(8)
#define SMC_353_ECC_CONFIG_ECC_IGNORE_ADD_EIGHT BSP_BIT32(7)
#define SMC_353_ECC_CONFIG_ECC_JUMP(val) BSP_FLD32(val, 5, 6)
#define SMC_353_ECC_CONFIG_ECC_JUMP_GET(reg) BSP_FLD32GET(reg, 5, 6)
#define SMC_353_ECC_CONFIG_ECC_JUMP_SET(reg, val) BSP_FLD32SET(reg, val, 5, 6)
#define SMC_353_ECC_CONFIG_ECC_READ_END BSP_BIT32(4)
#define SMC_353_ECC_CONFIG_ECC_MODE(val) BSP_FLD32(val, 2, 3)
#define SMC_353_ECC_CONFIG_ECC_MODE_GET(reg) BSP_FLD32GET(reg, 2, 3)
#define SMC_353_ECC_CONFIG_ECC_MODE_SET(reg, val) BSP_FLD32SET(reg, val, 2, 3)
#define SMC_353_ECC_CONFIG_PAGE_SIZE(val) BSP_FLD32(val, 0, 1)
#define SMC_353_ECC_CONFIG_PAGE_SIZE_GET(reg) BSP_FLD32GET(reg, 0, 1)
#define SMC_353_ECC_CONFIG_PAGE_SIZE_SET(reg, val) BSP_FLD32SET(reg, val, 0, 1)
  uint32_t memcmd0;
#define SMC_353_ECC_MEMCMD0_RD_END_CMD_V BSP_BIT32(24)
#define SMC_353_ECC_MEMCMD0_RD_END_CMD(val) BSP_FLD32(val, 16, 23)
#define SMC_353_ECC_MEMCMD0_RD_END_CMD_GET(reg) BSP_FLD32GET(reg, 16, 23)
#define SMC_353_ECC_MEMCMD0_RD_END_CMD_SET(reg, val) BSP_FLD32SET(reg, val, 16, 23)
#define SMC_353_ECC_MEMCMD0_RD_CMD(val) BSP_FLD32(val, 8, 15)
#define SMC_353_ECC_MEMCMD0_RD_CMD_GET(reg) BSP_FLD32GET(reg, 8, 15)
#define SMC_353_ECC_MEMCMD0_RD_CMD_SET(reg, val) BSP_FLD32SET(reg, val, 8, 15)
#define SMC_353_ECC_MEMCMD0_WR_CMD(val) BSP_FLD32(val, 0, 7)
#define SMC_353_ECC_MEMCMD0_WR_CMD_GET(reg) BSP_FLD32GET(reg, 0, 7)
#define SMC_353_ECC_MEMCMD0_WR_CMD_SET(reg, val) BSP_FLD32SET(reg, val, 0, 7)
  uint32_t memcmd1;
#define SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE_V BSP_BIT32(24)
#define SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE(val) BSP_FLD32(val, 16, 23)
#define SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE_GET(reg) BSP_FLD32GET(reg, 16, 23)
#define SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE_SET(reg, val) BSP_FLD32SET(reg, val, 16, 23)
#define SMC_353_ECC_MEMCMD1_RD_COL_CHANGE(val) BSP_FLD32(val, 8, 15)
#define SMC_353_ECC_MEMCMD1_RD_COL_CHANGE_GET(reg) BSP_FLD32GET(reg, 8, 15)
#define SMC_353_ECC_MEMCMD1_RD_COL_CHANGE_SET(reg, val) BSP_FLD32SET(reg, val, 8, 15)
#define SMC_353_ECC_MEMCMD1_WR_COL_CHANGE(val) BSP_FLD32(val, 0, 7)
#define SMC_353_ECC_MEMCMD1_WR_COL_CHANGE_GET(reg) BSP_FLD32GET(reg, 0, 7)
#define SMC_353_ECC_MEMCMD1_WR_COL_CHANGE_SET(reg, val) BSP_FLD32SET(reg, val, 0, 7)
  uint32_t addr0;
  uint32_t addr1;
  // XXX ==value0
  uint32_t block[4];
/* These definitions are valid for each array entry in block */
#define SMC_353_ECC_BLOCK_INT BSP_BIT32(31)
#define SMC_353_ECC_BLOCK_VALID BSP_BIT32(30)
#define SMC_353_ECC_BLOCK_READ BSP_BIT32(29)
#define SMC_353_ECC_BLOCK_FAIL BSP_BIT32(28)
#define SMC_353_ECC_BLOCK_CORRECT BSP_BIT32(27)
#define SMC_353_ECC_BLOCK_VAL(val) BSP_FLD32(val, 0, 23)
#define SMC_353_ECC_BLOCK_VAL_GET(reg) BSP_FLD32GET(reg, 0, 23)
#define SMC_353_ECC_BLOCK_VAL_SET(reg, val) BSP_FLD32SET(reg, val, 0, 23)
  uint32_t extra_block;
#define SMC_353_ECC_EXTRA_BLOCK_VAL(val) BSP_FLD32(val, 0, 15)
#define SMC_353_ECC_EXTRA_BLOCK_VAL_GET(reg) BSP_FLD32GET(reg, 0, 15)
#define SMC_353_ECC_EXTRA_BLOCK_VAL_SET(reg, val) BSP_FLD32SET(reg, val, 0, 15)
  uint32_t reserved_2c[(0x100-0x2c)/4];
} SMC_353_ECC_Base_t;

typedef struct SMC_353_Base_s {
    uint32_t memc_status;
#define SMC_353_MEMC_STATUS_RAW_ECC_INT1 BSP_BIT32(12)
#define SMC_353_MEMC_STATUS_RAW_ECC_INT0 BSP_BIT32(11)
#define SMC_353_MEMC_STATUS_ECC_INT1 BSP_BIT32(10)
#define SMC_353_MEMC_STATUS_ECC_INT0 BSP_BIT32(9)
#define SMC_353_MEMC_STATUS_ECC_INT_EN1 BSP_BIT32(8)
#define SMC_353_MEMC_STATUS_ECC_INT_EN0 BSP_BIT32(7)
#define SMC_353_MEMC_STATUS_RAW_INT_STATUS1 BSP_BIT32(6)
#define SMC_353_MEMC_STATUS_RAW_INT_STATUS0 BSP_BIT32(5)
#define SMC_353_MEMC_STATUS_INT_STATUS1 BSP_BIT32(4)
#define SMC_353_MEMC_STATUS_INT_STATUS0 BSP_BIT32(3)
#define SMC_353_MEMC_STATUS_INT_EN1 BSP_BIT32(2)
#define SMC_353_MEMC_STATUS_INT_EN0 BSP_BIT32(1)
#define SMC_353_MEMC_STATUS_STATE BSP_BIT32(0)
    uint32_t memif_cfg;
    uint32_t mem_cfg_set;
#define SMC_353_MEM_CFG_SET_ECC_INT_ENABLE1 BSP_BIT32(6)
#define SMC_353_MEM_CFG_SET_ECC_INT_ENABLE0 BSP_BIT32(5)
#define SMC_353_MEM_CFG_SET_LOW_POWER_REQ BSP_BIT32(2)
#define SMC_353_MEM_CFG_SET_INT_ENABLE1 BSP_BIT32(1)
#define SMC_353_MEM_CFG_SET_INT_ENABLE0 BSP_BIT32(0)
    uint32_t mem_cfg_clr;
#define SMC_353_MEM_CFG_CLR_ECC_INT_DISABLE1 BSP_BIT32(6)
#define SMC_353_MEM_CFG_CLR_ECC_INT_DISABLE0 BSP_BIT32(5)
#define SMC_353_MEM_CFG_CLR_INT_CLEAR1 BSP_BIT32(4)
#define SMC_353_MEM_CFG_CLR_INT_CLEAR0 BSP_BIT32(3)
#define SMC_353_MEM_CFG_CLR_LOW_POWER_EXIT BSP_BIT32(2)
#define SMC_353_MEM_CFG_CLR_INT_DISABLE1 BSP_BIT32(1)
#define SMC_353_MEM_CFG_CLR_INT_DISABLE0 BSP_BIT32(0)
    uint32_t direct_cmd;
#define SMC_353_DIRECT_CMD_CHIP_NMBR(val) BSP_FLD32(val, 23, 25)
#define SMC_353_DIRECT_CMD_CHIP_NMBR_GET(reg) BSP_FLD32GET(reg, 23, 25)
#define SMC_353_DIRECT_CMD_CHIP_NMBR_SET(reg, val) BSP_FLD32SET(reg, val, 23, 25)
#define SMC_353_DIRECT_CMD_CMD_TYPE(val) BSP_FLD32(val, 21, 22)
#define SMC_353_DIRECT_CMD_CMD_TYPE_GET(reg) BSP_FLD32GET(reg, 21, 22)
#define SMC_353_DIRECT_CMD_CMD_TYPE_SET(reg, val) BSP_FLD32SET(reg, val, 21, 22)
#define SMC_353_DIRECT_CMD_SET_CRE BSP_BIT32(20)
#define SMC_353_DIRECT_CMD_ADDR(val) BSP_FLD32(val, 0, 19)
#define SMC_353_DIRECT_CMD_ADDR_GET(reg) BSP_FLD32GET(reg, 0, 19)
#define SMC_353_DIRECT_CMD_ADDR_SET(reg, val) BSP_FLD32SET(reg, val, 0, 19)
    uint32_t set_cycles;
#define SMC_353_SET_CYCLES_SET_T6(val) BSP_FLD32(val, 20, 23)
#define SMC_353_SET_CYCLES_SET_T6_GET(reg) BSP_FLD32GET(reg, 20, 23)
#define SMC_353_SET_CYCLES_SET_T6_SET(reg, val) BSP_FLD32SET(reg, val, 20, 23)
#define SMC_353_SET_CYCLES_SET_T5(val) BSP_FLD32(val, 17, 19)
#define SMC_353_SET_CYCLES_SET_T5_GET(reg) BSP_FLD32GET(reg, 17, 19)
#define SMC_353_SET_CYCLES_SET_T5_SET(reg, val) BSP_FLD32SET(reg, val, 17, 19)
#define SMC_353_SET_CYCLES_SET_T4(val) BSP_FLD32(val, 14, 16)
#define SMC_353_SET_CYCLES_SET_T4_GET(reg) BSP_FLD32GET(reg, 14, 16)
#define SMC_353_SET_CYCLES_SET_T4_SET(reg, val) BSP_FLD32SET(reg, val, 14, 16)
#define SMC_353_SET_CYCLES_SET_T3(val) BSP_FLD32(val, 11, 13)
#define SMC_353_SET_CYCLES_SET_T3_GET(reg) BSP_FLD32GET(reg, 11, 13)
#define SMC_353_SET_CYCLES_SET_T3_SET(reg, val) BSP_FLD32SET(reg, val, 11, 13)
#define SMC_353_SET_CYCLES_SET_T2(val) BSP_FLD32(val, 8, 10)
#define SMC_353_SET_CYCLES_SET_T2_GET(reg) BSP_FLD32GET(reg, 8, 10)
#define SMC_353_SET_CYCLES_SET_T2_SET(reg, val) BSP_FLD32SET(reg, val, 8, 10)
#define SMC_353_SET_CYCLES_SET_T1(val) BSP_FLD32(val, 4, 7)
#define SMC_353_SET_CYCLES_SET_T1_GET(reg) BSP_FLD32GET(reg, 4, 7)
#define SMC_353_SET_CYCLES_SET_T1_SET(reg, val) BSP_FLD32SET(reg, val, 4, 7)
#define SMC_353_SET_CYCLES_SET_T0(val) BSP_FLD32(val, 0, 3)
#define SMC_353_SET_CYCLES_SET_T0_GET(reg) BSP_FLD32GET(reg, 0, 3)
#define SMC_353_SET_CYCLES_SET_T0_SET(reg, val) BSP_FLD32SET(reg, val, 0, 3)
    uint32_t set_opmode;
#define SMC_353_SET_OPMODE_SET_MW(val) BSP_FLD32(val, 0, 1)
#define SMC_353_SET_OPMODE_SET_MW_GET(reg) BSP_FLD32GET(reg, 0, 1)
#define SMC_353_SET_OPMODE_SET_MW_SET(reg, val) BSP_FLD32SET(reg, val, 0, 1)
    uint32_t unused_1c;
    uint32_t refresh_0;
    uint32_t refresh_1;
    uint32_t unused_28[54];
    SMC_353_Block_Base_t blocks[8];
    uint32_t user_status;
    uint32_t user_config;
    uint32_t reserved_208[(0x300-0x208)/4];
    /* Interface 0 is SRAM for SMC-353, interface 1 is NAND */
    SMC_353_ECC_Base_t ecc[2];
    uint32_t reserved_500[(0xE00-0x500)/4];
    uint32_t int_cfg;
    uint32_t int_inputs;
    uint32_t int_outputs;
    uint32_t reserved_e0c[(0xFE0-0xE0C)/4];
    uint32_t periph_id_n;
    uint32_t pcell_id_n;
} SMC_353_Base_t;

rtems_status_code smc_353_ecc_set_mode(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Mode ecc_mode)
{
  assert(smc_353_config);

  smc_353_config->base->ecc[interface].config = SMC_353_ECC_CONFIG_ECC_MODE_SET(smc_353_config->base->ecc[interface].config, ecc_mode);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_ecc_set_page_size(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Page_Size page_size)
{
  assert(smc_353_config);

  smc_353_config->base->ecc[interface].config = SMC_353_ECC_CONFIG_PAGE_SIZE_SET(smc_353_config->base->ecc[interface].config, page_size);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_ecc_set_jump(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Jump jump)
{
  assert(smc_353_config);

  smc_353_config->base->ecc[interface].config = SMC_353_ECC_CONFIG_ECC_JUMP_SET(smc_353_config->base->ecc[interface].config, jump);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_ecc_set_read_end(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Read_End read_end)
{
  assert(smc_353_config);

  if (read_end) {
    smc_353_config->base->ecc[interface].config |= SMC_353_ECC_CONFIG_ECC_READ_END;
  } else {
    smc_353_config->base->ecc[interface].config &= ~SMC_353_ECC_CONFIG_ECC_READ_END;
  }
  return RTEMS_SUCCESSFUL;
}

bool smc_353_int_status_raw(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface)
{
  assert(smc_353_config);
  if (interface == NAND) {
    return smc_353_config->base->memc_status & SMC_353_MEMC_STATUS_RAW_INT_STATUS1;
  }
  return smc_353_config->base->memc_status & SMC_353_MEMC_STATUS_RAW_INT_STATUS0;
}

void smc_353_int_clear(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface)
{
  assert(smc_353_config);
  if (interface == NAND) {
    smc_353_config->base->mem_cfg_clr = SMC_353_MEM_CFG_CLR_INT_CLEAR1;
  }
  smc_353_config->base->mem_cfg_clr = SMC_353_MEM_CFG_CLR_INT_CLEAR0;
}

bool smc_353_ecc_busy(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface)
{
  assert(smc_353_config);
  return smc_353_config->base->ecc[interface].status & SMC_353_ECC_STATUS_BUSY;
}

uint32_t smc_353_ecc_block_value(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Block ecc_block)
{
  assert(smc_353_config);
  return smc_353_config->base->ecc[interface].block[ecc_block];
}

void smc_353_int_enable(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, bool enabled)
{
  assert(smc_353_config);
  if (interface == NAND) {
    if (enabled) {
      smc_353_config->base->mem_cfg_set =
        SMC_353_MEM_CFG_SET_ECC_INT_ENABLE1
        | SMC_353_MEM_CFG_SET_INT_ENABLE1;
    } else {
      smc_353_config->base->mem_cfg_clr =
        SMC_353_MEM_CFG_CLR_ECC_INT_DISABLE1
        | SMC_353_MEM_CFG_CLR_INT_CLEAR1
        | SMC_353_MEM_CFG_CLR_INT_DISABLE1;
    }
  } else {
    if (enabled) {
      smc_353_config->base->mem_cfg_set =
        SMC_353_MEM_CFG_SET_ECC_INT_ENABLE0
        | SMC_353_MEM_CFG_SET_INT_ENABLE0;
    } else {
      smc_353_config->base->mem_cfg_clr =
        SMC_353_MEM_CFG_CLR_ECC_INT_DISABLE0
        | SMC_353_MEM_CFG_CLR_INT_CLEAR0
        | SMC_353_MEM_CFG_CLR_INT_DISABLE0;
    }
  }
}

rtems_status_code smc_353_ecc_set_memcmd0(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface,
  bool use_end_cmd, uint8_t read_end_cmd, uint8_t read_cmd, uint8_t write_cmd)
{
  uint32_t memcmd = 0;
  if (use_end_cmd) {
    memcmd |= SMC_353_ECC_MEMCMD0_RD_END_CMD_V;
  }
  memcmd |= SMC_353_ECC_MEMCMD0_RD_END_CMD(read_end_cmd);
  memcmd |= SMC_353_ECC_MEMCMD0_RD_CMD(read_cmd);
  memcmd |= SMC_353_ECC_MEMCMD0_WR_CMD(write_cmd);
  smc_353_config->base->ecc[interface].memcmd0 = memcmd;
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_ecc_set_memcmd1(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface,
  bool use_end_cmd, uint8_t read_end_cmd, uint8_t read_cmd, uint8_t write_cmd)
{
  uint32_t memcmd = 0;
  if (use_end_cmd) {
    memcmd |= SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE_V;
  }
  memcmd |= SMC_353_ECC_MEMCMD1_RD_END_COL_CHANGE(read_end_cmd);
  memcmd |= SMC_353_ECC_MEMCMD1_RD_COL_CHANGE(read_cmd);
  memcmd |= SMC_353_ECC_MEMCMD1_WR_COL_CHANGE(write_cmd);
  smc_353_config->base->ecc[interface].memcmd1 = memcmd;
  return RTEMS_SUCCESSFUL;
}

static void update_nand_regs(SMC_353_Config_t *smc_353_config)
{
  /* select NAND and UpdateRegs */
  smc_353_config->base->direct_cmd = SMC_353_DIRECT_CMD_CHIP_NMBR(4) | SMC_353_DIRECT_CMD_CMD_TYPE(2);
}

rtems_status_code smc_353_set_bus_width(SMC_353_Config_t *smc_353_config, enum SMC_353_Opmode_Bus_Width bus_width)
{
  assert(smc_353_config);

  smc_353_config->base->set_opmode = SMC_353_SET_OPMODE_SET_MW(bus_width);
  update_nand_regs(smc_353_config);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_set_cycles(SMC_353_Config_t *smc_353_config, uint32_t timing0,
                                     uint32_t timing1, uint32_t timing2, uint32_t timing3,
				     uint32_t timing4, uint32_t timing5, uint32_t timing6)
{
  assert(smc_353_config);

  /* These fields are only 4 bits wide */
  if (timing0 < 2 || timing0 > 15) {
    return RTEMS_INVALID_NUMBER;
  }
  if (timing1 < 2 || timing1 > 15) {
    return RTEMS_INVALID_NUMBER;
  }
  /* These fields are only 3 bits wide */
  if (timing2 < 1 || timing2 > 7) {
    return RTEMS_INVALID_NUMBER;
  }
  if (timing3 < 1 || timing3 > 7) {
    return RTEMS_INVALID_NUMBER;
  }
  if (timing4 > 7) {
    return RTEMS_INVALID_NUMBER;
  }
  if (timing5 > 7) {
    return RTEMS_INVALID_NUMBER;
  }
  if (timing6 > 7) {
    return RTEMS_INVALID_NUMBER;
  }
  uint32_t cycles = 0;
  cycles |= SMC_353_SET_CYCLES_SET_T0(timing0);
  cycles |= SMC_353_SET_CYCLES_SET_T1(timing1);
  cycles |= SMC_353_SET_CYCLES_SET_T2(timing2);
  cycles |= SMC_353_SET_CYCLES_SET_T3(timing3);
  cycles |= SMC_353_SET_CYCLES_SET_T4(timing4);
  cycles |= SMC_353_SET_CYCLES_SET_T5(timing5);
  cycles |= SMC_353_SET_CYCLES_SET_T6(timing6);
  smc_353_config->base->set_cycles = cycles;
  update_nand_regs(smc_353_config);
  return RTEMS_SUCCESSFUL;
}

static rtems_status_code wait_nand_ecc_busy(SMC_353_Config_t *smc_353_config)
{
  uint32_t wait_time_ms = 0;
  /* wait 500ms for the ECC to become ready */
  while (wait_time_ms < 500) {
    if (!smc_353_ecc_busy(smc_353_config, NAND)) {
      break;
    }
    usleep(1000);
    wait_time_ms++;
  }
  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_init(SMC_353_Config_t *smc_353_config, volatile void *base)
{
  smc_353_config->base = (volatile SMC_353_Base_t *)base;
  /* clear and disable interrupts */
  smc_353_int_enable(smc_353_config, NAND, false);
  smc_353_set_bus_width(smc_353_config, BW_8BITS);
  update_nand_regs(smc_353_config);
  if (wait_nand_ecc_busy(smc_353_config)) {
    return RTEMS_TIMEOUT;
  }

  /* From ARM SMC-35x manual */
  smc_353_ecc_set_memcmd0(smc_353_config, NAND, true, 0x30, 0x0, 0x80);
  smc_353_ecc_set_memcmd1(smc_353_config, NAND, true, 0xE0, 0x5, 0x85);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code smc_353_uninit(SMC_353_Config_t *smc_353_config)
{
  smc_353_config->base = NULL;
  return RTEMS_SUCCESSFUL;
}
