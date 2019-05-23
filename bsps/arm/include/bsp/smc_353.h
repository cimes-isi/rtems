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

#ifndef SMC_353_H
#define SMC_353_H

struct SMC_353_Base_s;
typedef struct SMC_353_Base_s SMC_353_Base_t;

typedef struct SMC_353_Config_s {
  volatile SMC_353_Base_t *base;
} SMC_353_Config_t;

/*!
 * Initialize the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an allocated SMC_353_Config_t structure to be initialized.
 *
 * \param       base
 *              A pointer to the base of the memory space occupied by the SMC-353
 *              timer.
 *
 * \retval      RTEMS_SUCCESSFUL The configuration completed successfully.
 * \retval      RTEMS_TIMEOUT    ECC was busy during initialization.
 */
rtems_status_code smc_353_init(SMC_353_Config_t *smc_353_config, volatile void *base);

/*!
 * Uninitialize the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_uninit(SMC_353_Config_t *smc_353_config);

enum SMC_353_Interface {
  SRAM = 0,
  NAND = 1,
};

enum SMC_353_ECC_Config_Mode {
  BYPASSED = 0,
  CALCULATE_APB = 1,
  CALCULATE_WRITE = 2,
};

/*!
 * Set the ECC mode for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       ecc_mode
 *              The desired ECC mode.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_mode(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Mode ecc_mode);

enum SMC_353_ECC_Config_Jump {
  NO_JUMP = 0,
  JUMP_COL = 1,
  JUMP_FULL = 2,
};

/*!
 * Set the jump mode for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       jump
 *              The type of column change supported.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_jump(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Jump jump);

enum SMC_353_ECC_Config_Read_End {
  READ_AFTER_BLOCK = 0,
  READ_AFTER_PAGE = 1,
};

/*!
 * Set the read end mode for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       read_end
 *              Where the read should end.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_read_end(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Read_End read_end);

enum SMC_353_ECC_Config_Page_Size {
  PS_512 = 1,
  PS_1024 = 2,
  PS_2048 = 3,
};

/*!
 * Set the ECC page size for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       page_size
 *              The desired ECC page size.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_page_size(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Config_Page_Size page_size);

/*!
 * Set the ECC memcmd0 (ECC operation start) for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       use_end_cmd
 *              Whether the read end command is used.
 *
 * \param       read_end_cmd
 *              The read end command.
 *
 * \param       read_cmd
 *              The read command.
 *
 * \param       write_cmd
 *              The write command.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_memcmd0(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, bool use_end_cmd, uint8_t read_end_cmd, uint8_t read_cmd, uint8_t write_cmd);

/*!
 * Set the ECC memcmd1 (column change) for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       use_end_cmd
 *              Whether the read end command is used.
 *
 * \param       read_end_cmd
 *              The read end command.
 *
 * \param       read_cmd
 *              The read command.
 *
 * \param       write_cmd
 *              The write command.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_ecc_set_memcmd1(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, bool use_end_cmd, uint8_t read_end_cmd, uint8_t read_cmd, uint8_t write_cmd);

/*!
 * Get the NAND interrupt status for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \retval      True     The interrupt has fired.
 * \retval      False    The interrupt has not fired.
 */
bool smc_353_int_status_raw(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface);

/*!
 * Clear the interrupt status for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 */
void smc_353_int_clear(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface);

/*!
 * Check whether the NAND ECC is busy.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \retval      True     The SMC-353 is busy with ECC calculations.
 * \retval      False    The SMC-353 is not busy with ECC calculations.
 */
bool smc_353_ecc_busy(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface);

enum SMC_353_ECC_Block {
  BLOCK_0 = 0,
  BLOCK_1 = 1,
  BLOCK_2 = 2,
  BLOCK_3 = 3,
};

/*!
 * Retrieve the NAND ECC Block information for the given block.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       ecc_block
 *              The index of the desired ECC block information.
 *
 * \retval      The ECC block value.
 */
uint32_t smc_353_ecc_block_value(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, enum SMC_353_ECC_Block ecc_block);

/*!
 * Retrieve the NAND ECC Block information for the given block.
 *
 * \param       smc_353_config
 *              A pointer to an initialized SMC_353_Config_t structure.
 *
 * \param       interface
 *              The interface to manipulate.
 *
 * \param       enabled
 *              Whether the interrupts are enabled.
 */
void smc_353_int_enable(SMC_353_Config_t *smc_353_config, enum SMC_353_Interface interface, bool enabled);

enum SMC_353_Opmode_Bus_Width {
  BW_8BITS = 0,
  BW_16BITS = 1,
  BW_32BITS = 2,
};

/*!
 * Set the bus width for the NAND.
 *
 * \param       smc_353_config
 *              A pointer to an allocated SMC_353_Config_t structure to be initialized.
 *
 * \param       bus_width
 *              The width of the NAND chip's bus.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 */
rtems_status_code smc_353_set_bus_width(SMC_353_Config_t *smc_353_config, enum SMC_353_Opmode_Bus_Width bus_width);

/*!
 * Set the NAND timings for the SMC-353.
 *
 * \param       smc_353_config
 *              A pointer to an allocated SMC_353_Config_t structure to be initialized.
 *
 * \param       timing0
 *              The NAND's T0 value.
 *
 * \param       timing1
 *              The NAND's T1 value.
 *
 * \param       timing2
 *              The NAND's T2 value.
 *
 * \param       timing3
 *              The NAND's T3 value.
 *
 * \param       timing4
 *              The NAND's T4 value.
 *
 * \param       timing5
 *              The NAND's T5 value.
 *
 * \param       timing6
 *              The NAND's T6 value.
 *
 * \retval      RTEMS_SUCCESSFUL     The configuration completed successfully.
 * \retval      RTEMS_INVALID_NUMBER One or more of the timings is out of bounds.
 */
rtems_status_code smc_353_set_cycles(SMC_353_Config_t *smc_353_config, uint32_t timing0,
                                     uint32_t timing1, uint32_t timing2, uint32_t timing3,
				     uint32_t timing4, uint32_t timing5, uint32_t timing6);

#endif // SMC_353_H
