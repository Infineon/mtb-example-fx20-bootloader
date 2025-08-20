/***************************************************************************//**
* \file cy_smif_memslot_bl.c
* \version 2.60
*
* \brief
*  This file provides the source code for the memory-level APIs of the SMIF driver.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2016-2022 Cypress Semiconductor Corporation
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

#include "cy_device.h"

#if defined (CY_IP_MXSMIF)

#include "cy_smif_memslot_bl.h"
#include "cy_gpio.h"
#include "../cy_spi_smif.h"
#if defined(__cplusplus)
extern "C" {
#endif


/** \cond INTERNAL */
/***************************************
*     Internal Constants
***************************************/
#define SMIF_MAX_RX_COUNT           (65536UL)
#define ONE_MILLI_IN_MICRO          (1000UL)
#define TIMEOUT_SLICE_MAX           (1000000UL)    /* The maximum timeout slice (microseconds)
                                                    * while polling the memory */
#define TIMEOUT_SLICE_DIV           (4U)           /* The division factor to use for slicing the timeout
                                                    * while polling the memory
                                                    */
#if (CY_IP_MXSMIF_VERSION>=2)
static cy_en_smif_status_t cy_smif_octalddrenable(SMIF_Type *base,
                                    cy_stc_smif_mem_config_t const *memDevice,
                                    cy_stc_smif_context_t const *context);
#endif
/** \endcond */

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
                            cy_stc_smif_context_t *context)
{
    SMIF_DEVICE_Type volatile * device;
    cy_stc_smif_mem_config_t const * memCfg;
    uint32_t result = (uint32_t)CY_SMIF_BAD_PARAM;
    uint32_t sfdpRes =(uint32_t)CY_SMIF_SUCCESS;
    uint32_t idx;

    if ((NULL != base) && (NULL != blockConfig) && (NULL != blockConfig->memConfig) 
        && (NULL != context) && (0U != blockConfig->memCount))
    {
        uint32_t size = blockConfig->memCount;
        cy_stc_smif_mem_config_t** extMemCfg = blockConfig->memConfig;

        result = (uint32_t)CY_SMIF_SUCCESS;
        for(idx = 0UL; idx < size; idx++)
        {
            memCfg = extMemCfg[idx];
            if (NULL != memCfg)
            {
                /* Check smif memory slot configuration */
                CY_ASSERT_L3(CY_SMIF_SLAVE_SEL_VALID(memCfg->slaveSelect));
                CY_ASSERT_L3(CY_SMIF_DATA_SEL_VALID(memCfg->dataSelect));

                #if (CY_IP_MXSMIF_VERSION >= 2)
                context->flags = memCfg->flags;
                #endif /* (CY_IP_MXSMIF_VERSION>=2) */

                /* SPI(deviceCfg) and Hyperbus(hbdeviceCfg) are mutually exclusive and if both are initialized, priority would be for SPI(deviceCfg) */
                if(memCfg->deviceCfg != NULL)
                {
                    device = Cy_SMIF_GetDeviceBySlot(base, memCfg->slaveSelect);
                    if (NULL != device)
                    {
                        /* The slave-slot initialization of the device control register.
                         * Cy_SMIF_MemSfdpDetect() must work */
                        SMIF_DEVICE_CTL(device)  = _CLR_SET_FLD32U(SMIF_DEVICE_CTL(device),
                                                                   SMIF_DEVICE_CTL_DATA_SEL,
                                                                  (uint32_t)memCfg->dataSelect);

                        /* Before SFDP Enumeration, configure SMIF dedicated Clock and RWDS lines */
                        #if (CY_IP_MXSMIF_VERSION >= 5)
                        SMIF_CLK_HSIOM(base) = ((uint32_t)(HSIOM_SEL_ACT_15)) | (((uint32_t)HSIOM_SEL_ACT_15) << 8U);
                        SMIF_RWDS_HSIOM(base) = (uint32_t)HSIOM_SEL_ACT_15;
                        SMIF_CLK_DRIVEMODE(base) = CY_GPIO_DM_STRONG | (CY_GPIO_DM_STRONG << 4U);
                        SMIF_RWDS_DRIVEMODE(base) = CY_GPIO_DM_PULLDOWN;
                        #endif

                        uint32_t sfdpRet = (uint32_t)CY_SMIF_SUCCESS;
                        if (0U != (memCfg->flags & CY_SMIF_FLAG_DETECT_SFDP))
                        {
                            sfdpRet = (uint32_t)Cy_SMIF_MemSfdpDetect_Lite(base,
                                                    memCfg->deviceCfg,
                                                    memCfg->slaveSelect,
                                                    memCfg->dataSelect,
                                                    context);
                            if((uint32_t)CY_SMIF_SUCCESS != sfdpRet)
                            {
                                sfdpRes |=  ((uint32_t)CY_SMIF_SFDP_FAIL << idx);
                            }
                        }
                        /* Check the size of the smif memory slot address */
                        CY_ASSERT_L2(MEM_ADDR_SIZE_VALID(memCfg->deviceCfg->numOfAddrBytes));
                    }
                    else
                    {
                        result = (uint32_t)CY_SMIF_BAD_PARAM;
                        break;
                    }
                }
    #if (CY_IP_MXSMIF_VERSION>=2) && defined (SMIF_HYPERBUS_DEVICE_SUPPORT)
                else if(memCfg->hbdeviceCfg != NULL)
                {
                    result = (uint32_t)Cy_SMIF_HyperBus_InitDevice(base, memCfg, context);
                }
    #endif /* (CY_IP_MXSMIF_VERSION>=2) && defined (SMIF_HYPERBUS_DEVICE_SUPPORT) */
                else
                {
                    result = (uint32_t)CY_SMIF_BAD_PARAM;
                    break;
                }
            }
        }
    }
    if((uint32_t)CY_SMIF_SUCCESS != sfdpRes)
    {
        result = (((uint32_t)CY_SMIF_ID) | ((uint32_t)CY_PDL_STATUS_ERROR) | sfdpRes);
    }
    return (cy_en_smif_status_t) result;
}

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXSMIF */

/* [] END OF FILE */
