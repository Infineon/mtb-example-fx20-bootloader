/***************************************************************************//**
* \file cy_smif_memslot_bl.h
* \version 2.130
*
* \brief
*  This file provides the constants and parameter values for the memory-level
*  APIs of the SMIF driver.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2016-2024 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#ifndef _CY_SMIF_MEMSLOT_H_
#define _CY_SMIF_MEMSLOT_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_syslib.h"
#include "cy_smif.h"
#include "cy_smif_memslot.h"


/*******************************************************************************
* Function Name: Cy_SMIF_MemInit_NoXIP
****************************************************************************//**
*
* This function initializes the slots of the memory device in the SMIF
* configuration.
* This function must be called when a memory device is required to be used
* in memory-mapped (XIP) mode. This function can also be called instead of
* calling \ref Cy_SMIF_MemSfdpDetect when SFDP auto-discovery is enabled.
* Note that this function performs SFDP on all the external memories whereas
* \ref Cy_SMIF_MemSfdpDetect performs it only on one memory that is specified
* through the arguments. This function configures the SMIF device slot registers
* with the configuration from \ref cy_stc_smif_mem_config_t structure which is 
* a member of the \ref cy_stc_smif_block_config_t structure. If SFDP discovery
* is enabled in the configuration structure through autoDetectSfdp field,
* this function calls \ref Cy_SMIF_MemSfdpDetect function for each memory,
* fills the structures with the discovered parameters, and configures the
* SMIF device slot registers accordingly. \ref Cy_SMIF_Init must have been 
* called prior to calling this function.
* The \ref cy_stc_smif_context_t context structure returned from \ref Cy_SMIF_Init
* is passed as a parameter to this function.
*
* \note 4-byte addressing mode is set when the memory device supports 
*       3- or 4-byte addressing mode.
*
* \param base
* The address of the slave-slot device register to initialize.
*
* \param blockConfig
* The configuration structure array that configures the SMIF memory device to be
* mapped into the PSoC memory map. \ref cy_stc_smif_mem_config_t
*
* \param context
* This is the pointer to the context structure \ref cy_stc_smif_context_t
* allocated by the user. The structure is used during the SMIF
* operation for internal configuration and data retention. The user must not
* modify anything in this structure.
*
* \return The memory slot initialization status.
*       - \ref CY_SMIF_SUCCESS
*       - \ref CY_SMIF_BAD_PARAM
*       - \ref CY_SMIF_SFDP_SS0_FAILED
*       - \ref CY_SMIF_SFDP_SS1_FAILED
*       - \ref CY_SMIF_SFDP_SS2_FAILED
*       - \ref CY_SMIF_SFDP_SS3_FAILED
*
*******************************************************************************/
cy_en_smif_status_t Cy_SMIF_MemInit_NoXIP(SMIF_Type *base,
                            cy_stc_smif_block_config_t const * blockConfig,
                            cy_stc_smif_context_t *context);



/*******************************************************************************
* Function Name: Cy_SMIF_MemInitSfdpMode_Lite
****************************************************************************//**
*
* This function can be used for any preferred data width based command instruction set from SFDP Buffer.
*
* \param base
* Holds the base address of the SMIF block registers.
*
* \param memCfg
* The memory configuration structure that configures the SMIF memory device to
*  map into the device memory map. \ref cy_stc_smif_mem_config_t
*
* \param maxdataWidth
* maximum data width available on physical interface.
*
* \param qer_id
* Quad enable requirement ID specifically used for SFDP 1.0 compliant devices where
* Quad mode is available for use, but SFDP basic flash parameter table does not specify
* quad mode enable instruction. In other cases, this can be passed as CY_SMIF_SFDP_QER_0.
*
* \param context
* This is the pointer to the context structure \ref cy_stc_smif_context_t
* allocated by the user. The structure is used during the SMIF
* operation for internal configuration and data retention. The user must not
* modify anything in this structure.
*
* \return A status of the transmission.
*       - \ref CY_SMIF_SUCCESS
*       - \ref CY_SMIF_CMD_NOT_FOUND
*       - \ref CY_SMIF_SFDP_CORRUPTED_TABLE
*       - \ref CY_SMIF_EXCEED_TIMEOUT
*
* \snippet smif/snippet/main.c SMIF_INIT: SFDP
*******************************************************************************/

cy_en_smif_status_t Cy_SMIF_MemInitSfdpMode_Lite(SMIF_Type *base,
                                    const cy_stc_smif_mem_config_t *memCfg,
                                    cy_en_smif_txfr_width_t maxdataWidth,
                                    cy_en_smif_qer_t qer_id,
                                    cy_stc_smif_context_t *context);



/*******************************************************************************
* Function Name: Cy_SMIF_MemSfdpDetect_Lite
****************************************************************************//**
*
* This function detects the device signature for SFDP devices.
* Refer to the SFDP spec (JESD216B) for details.
* The function asks the device using an SPI and then populates the relevant
* parameters for \ref cy_stc_smif_mem_device_cfg_t.
*
* \note This function is a blocking function and blocks until the structure data
* is read and returned. This function uses \ref Cy_SMIF_TransmitCommand()
* If there is no support for SFDP in the memory device, the API returns an
* error condition. The function requires:
*   - SMIF initialized and enabled to work in the normal mode;
*   - readSfdpCmd field of \ref cy_stc_smif_mem_device_cfg_t is enabled.
*
* \note The SFDP detect takes into account the types of the SPI supported by the
* memory device and also the dataSelect option selected to choose which SPI mode
* (SPI, DSPI, QSPI) to load into the structures. The algorithm prefers
* QSPI>DSPI>SPI, provided there is support for it in the memory device and the
* dataSelect selected by the user.
*
* \note 4-byte addressing mode is set when the memory device supports
*       3- or 4-byte addressing mode.
*
* \note When the Erase command is not found the width of the command
*  transfer (cmdWidth) is set to CY_SMIF_WIDTH_NA. When the Program command
*  is not found for 4 byte addressing mode the Program command instruction
*  is set for 1S-1S-1S Protocol mode and 3-byte addressing mode.
*
* \param base
* Holds the base address of the SMIF block registers.
*
* \param device
* The device structure instance declared by the user. This is where the detected
* parameters are stored and returned.
*
* \param slaveSelect
* The slave select line for the device.
*
* \param dataSelect
* The data line selection options for a slave device.
*
* \param context
* This is the pointer to the context structure \ref cy_stc_smif_context_t
* allocated by the user. The structure is used during the SMIF
* operation for internal configuration and data retention. The user must not
* modify anything in this structure.
*
* \return A status of the transmission.
*       - \ref CY_SMIF_SUCCESS
*       - \ref CY_SMIF_CMD_FIFO_FULL
*       - \ref CY_SMIF_NO_SFDP_SUPPORT
*       - \ref CY_SMIF_EXCEED_TIMEOUT
*
*******************************************************************************/
cy_en_smif_status_t Cy_SMIF_MemSfdpDetect_Lite(SMIF_Type *base,
                                    cy_stc_smif_mem_device_cfg_t *device,
                                    cy_en_smif_slave_select_t slaveSelect,
                                    cy_en_smif_data_select_t dataSelect,
                                    cy_stc_smif_context_t *context);
#endif
