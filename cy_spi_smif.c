/***************************************************************************//**
 * \file cy_spi_smif.c
 * \version 1.0
 *
 * \brief
 * SPI Driver implementation based on SMIF PDL
 *
 *******************************************************************************
 * \copyright
 * (c) (2021-2025), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 *
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
#include "cy_spi_smif.h"
#include "cycfg_qspi_memslot.h"
#include "cy_smif_memslot_bl.h"
#include <string.h>


cy_stc_smif_context_t qspiContext;
static uint32_t glSfdpSupported = false;

static const cy_stc_smif_config_t qspiConfig =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = 0u, /*Minimum duration SPI_SS is held high between SPI transfers
                           (Default: 0 -> 1 interface cycle)*/
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,/* Source selection for receiver clock.
                                                            MISO is sampled on rising edge of this clock */
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR
};


static cy_stc_cfi_flash_map_t cfiFlashMap;
#if BL_DEBUG
/**
 * \name Cy_App_PrintBuffer
 * \brief Prints a buffer in hex format for debug purposes.
 * \param buffer Pointer to the buffer.
 * \param len Length of the buffer.
 * \retval None
 */
static void Cy_App_PrintBuffer(uint8_t *buffer, uint32_t len)
{
    ASSERT(buffer != NULL, 0);
    uint32_t looper = 0;

    Cy_Debug_AddToLog(1, "\r\n");
    for (looper = 0; looper < len; looper++)
    {
        if (!(looper % 8)) {
            Cy_Debug_AddToLog(1, "\r\n");
        }
        Cy_Debug_AddToLog(1, "0x%x ", buffer[looper]);
    }

    Cy_Debug_AddToLog(1, "\r\n");
}
#endif

/**
 * \name Cy_SPI_WaitTillMemNotBusy
 * \brief Waits until the memory is not busy or timeout occurs.
 * \param slaveSelect Slave select line.
 * \param timeoutUs Timeout in microseconds.
 * \retval cy_en_smif_status_t SMIF status.
 */
static cy_en_smif_status_t Cy_SPI_WaitTillMemNotBusy(cy_en_smif_slave_select_t slaveSelect, uint32_t timeoutUs) {
    uint32_t busyWait = 0;
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    while(Cy_SPI_IsMemBusy(slaveSelect))
    {
        if(busyWait++ >= timeoutUs)
        {
            status =  CY_SMIF_EXCEED_TIMEOUT;
            break;
        }
        else
        {
            Cy_SysLib_DelayUs(1);
        }
    }
    return status;
}

/**
 * \name AddressToArray
 * \brief Converts a 32-bit address to a byte array.
 * \param value 32-bit value to convert.
 * \param byteArray Output byte array.
 * \param size Number of bytes to convert.
 * \retval cy_en_smif_status_t SMIF status.
 */
static cy_en_smif_status_t AddressToArray(uint32_t value, uint8_t *byteArray, uint8_t size)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    if(byteArray == NULL)
    {
        return CY_SMIF_BAD_PARAM;
    }

    do
    {
        size--;
        byteArray[size] = (uint8_t)(value & 0x000000FF);
        value >>= 8U; /* Shift to get the next byte */
    } while (size > 0U);

    return status;
}

/**
 * \name Cy_SPI_WriteEnable
 * \brief Enables write operation on the SPI flash.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
static cy_en_smif_status_t Cy_SPI_WriteEnable(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    status = Cy_SMIF_MemCmdWriteEnable(SMIF_HW, smifMemConfigs[0], &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/**
 * \name Cy_SPI_DeinitSMIFPins
 * \brief Deinitializes the SMIF pins
 * \retval cy_en_gpio_status_t GPIO status.
 */
static cy_en_gpio_status_t Cy_SPI_DeinitSMIFPins()
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Create a local structure to store the SMIF GPIO configuration */
    const struct {
        GPIO_PRT_Type * port;
        uint32_t pin;
        en_hsiom_sel_t hsiomDeinit;
        uint32_t driveModeDeinit;
    } smifPinConfigs[] = {
        {SMIF_DATA0_PORT,   SMIF_DATA0_PIN,   HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
        {SMIF_DATA1_PORT,   SMIF_DATA1_PIN,   HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
        {SMIF_DATA2_PORT,   SMIF_DATA2_PIN,   HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
        {SMIF_DATA3_PORT,   SMIF_DATA3_PIN,   HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
        {SMIF_SELECT0_PORT, SMIF_SELECT0_PIN, HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
        {SMIF_CLK_PORT,     SMIF_CLK_PIN, 	HSIOM_SEL_GPIO,   CY_GPIO_DM_HIGHZ},
    };

    for (uint32_t i = 0; i < sizeof(smifPinConfigs) / sizeof(smifPinConfigs[0]); i++) {
        pinCfg.driveMode = smifPinConfigs[i].driveModeDeinit;
        pinCfg.hsiom = smifPinConfigs[i].hsiomDeinit;
        status = Cy_GPIO_Pin_Init(smifPinConfigs[i].port, smifPinConfigs[i].pin, &pinCfg);
        ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);
    }

    return status;
}

/**
 * \name Cy_SPI_ValidFlashDetected
 * \brief Checks if a valid flash device is detected.
 * \retval uint32_t 1 if valid, 0 otherwise.
 */
uint32_t Cy_SPI_ValidFlashDetected(void)
{
    return (cfiFlashMap.isValid || glSfdpSupported) ;
}

/**
 * \name Cy_SPI_VerifyCFIParams
 * \brief Verifies the CFI parameters in the buffer.
 * \param rxBuffer Buffer containing CFI data.
 * \retval bool true if valid, false otherwise.
 */
static bool Cy_SPI_VerifyCFIParams(uint8_t *rxBuffer)
{
    bool isQRYPresent = false;

    if(rxBuffer[CY_CFI_QRY_STRING_OFFSET] == 'Q' && rxBuffer[CY_CFI_QRY_STRING_OFFSET + 1] == 'R' && rxBuffer[CY_CFI_QRY_STRING_OFFSET + 2] == 'Y')
    {
        isQRYPresent = true;
        cfiFlashMap.isValid = 1;
    }
    else
    {
        cfiFlashMap.isValid = 0;
    }

    return isQRYPresent;
}

/**
 * \name Cy_SPI_UpdateMemoryCfg
 * \brief Updates the memory configuration from the CFI flash map.
 * \param cfiFlashMap Pointer to the CFI flash map.
 * \retval None
 */
static void Cy_SPI_UpdateMemoryCfg (const cy_stc_cfi_flash_map_t *cfiFlashMap)
{
    if(!glSfdpSupported) {
        deviceCfg_SFDP_SlaveSlot_0.memSize = cfiFlashMap->deviceSize;
        deviceCfg_SFDP_SlaveSlot_0.numOfAddrBytes = SPI_ADDRESS_BYTE_COUNT;
        deviceCfg_SFDP_SlaveSlot_0.readCmd->command = CY_SPI_4READ_COMMAND;
        deviceCfg_SFDP_SlaveSlot_0.writeEnCmd->command = CY_SMIF_WRITE_ENABLE_CMD;
        deviceCfg_SFDP_SlaveSlot_0.programCmd->command = CY_SPI_4PROGRAM_CMD;
        deviceCfg_SFDP_SlaveSlot_0.readStsRegWipCmd->command = CY_SPI_STATUS_READ_CMD;
        deviceCfg_SFDP_SlaveSlot_0.stsRegBusyMask = CY_SMIF_STATUS_REG_BUSY_MASK;
        deviceCfg_SFDP_SlaveSlot_0.programTime = CY_APP_SPI_PROGRAM_TIMEOUT_US ;
    }
}

/**
 * \name Cy_SPI_ReadCFIMap
 * \brief Reads the CFI map from the flash device.
 * \param rxBuffer Buffer to store the CFI data.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
static cy_en_smif_status_t Cy_SPI_ReadCFIMap(uint8_t *rxBuffer, cy_en_smif_slave_select_t slaveSelect)
{
    uint8_t sectorIndex = 0;
    uint8_t eraseRegionIndex = 0;
    uint8_t writeSizeFactor = 0;

    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_READ_ID_CMD,
            CY_SMIF_WIDTH_SINGLE,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)slaveSelect,
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, CY_CFI_TABLE_LENGTH, CY_SMIF_WIDTH_SINGLE, &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
        if((CY_SMIF_SUCCESS == status) && (Cy_SPI_VerifyCFIParams(rxBuffer)))
        {
            cfiFlashMap.deviceSizeFactor = rxBuffer[CY_CFI_DEVICE_SIZE_OFFSET];
            cfiFlashMap.deviceSize = (uint32_t)(1u << cfiFlashMap.deviceSizeFactor);
            writeSizeFactor = rxBuffer[CY_CFI_PROGRAM_SIZE_OFFSET];
            if (writeSizeFactor) {
                cfiFlashMap.programSize = (uint32_t) (1u << writeSizeFactor);
            }
#if BL_DEBUG
            DBG_APP_INFO("DeviceSize = 0x%x[%d]\r\nProgramSize=%d\r\n", (cfiFlashMap.deviceSize), (cfiFlashMap.deviceSize),
                    cfiFlashMap.programSize);
#endif

            //Parse the CFI buffer and understand possible memory array layouts
            cfiFlashMap.numEraseRegions = rxBuffer[CY_CFI_NUM_ERASE_REGION_OFFSET];
#if BL_DEBUG
            DBG_APP_INFO("numEraseRegions = %d\r\n", cfiFlashMap.numEraseRegions);
#endif

            if(cfiFlashMap.numEraseRegions < CY_CFI_TABLE_LENGTH)
            {
                //The part has multiple erase layouts, possibly because it supports hybrid layout
                for(eraseRegionIndex = 0 , sectorIndex = 0;
                        eraseRegionIndex < (cfiFlashMap.numEraseRegions);
                        eraseRegionIndex++, sectorIndex += CY_CFI_ERASE_REGION_SIZE_INFO_SIZE)
                {
                    cfiFlashMap.memoryLayout[eraseRegionIndex].numSectors = 1 + (rxBuffer[sectorIndex + CY_CFI_ERASE_NUM_SECTORS_OFFSET] |
                            (rxBuffer[sectorIndex + CY_CFI_ERASE_NUM_SECTORS_OFFSET + 1] << 8));

                    cfiFlashMap.memoryLayout[eraseRegionIndex].sectorSize = 256 * (rxBuffer[sectorIndex + CY_CFI_ERASE_SECTOR_SIZE_OFFSET] |
                            (rxBuffer[sectorIndex + CY_CFI_ERASE_SECTOR_SIZE_OFFSET + 1] << 8));
                    if(eraseRegionIndex)
                    {
                        cfiFlashMap.memoryLayout[eraseRegionIndex].startingAddress = (cfiFlashMap.memoryLayout[eraseRegionIndex - 1].startingAddress +
                                cfiFlashMap.memoryLayout[eraseRegionIndex - 1].sectorSize *
                                cfiFlashMap.memoryLayout[eraseRegionIndex - 1].numSectors);
                    }
                    else
                    {
                        cfiFlashMap.memoryLayout[eraseRegionIndex].startingAddress = 0;
                    }

                    cfiFlashMap.memoryLayout[eraseRegionIndex].lastAddress = cfiFlashMap.memoryLayout[eraseRegionIndex].startingAddress +
                        (cfiFlashMap.memoryLayout[eraseRegionIndex].numSectors * cfiFlashMap.memoryLayout[eraseRegionIndex].sectorSize) - 1;

                    if(cfiFlashMap.memoryLayout[eraseRegionIndex].sectorSize == 0x1000)
                    {
                        cfiFlashMap.num4KBParameterRegions++;
                    }

#if BL_DEBUG
                    DBG_APP_INFO("Erase region:%d, numSectors=%d, sectorSize=0x%x, startingAddress=0x%x lastAddress=0x%x\r\n",eraseRegionIndex,
                            cfiFlashMap.memoryLayout[eraseRegionIndex].numSectors,
                            cfiFlashMap.memoryLayout[eraseRegionIndex].sectorSize,
                            cfiFlashMap.memoryLayout[eraseRegionIndex].startingAddress,
                            cfiFlashMap.memoryLayout[eraseRegionIndex].lastAddress);
#endif
                }
            }
        }
    }
    return status;
}

/**
 * \name Cy_SPI_IsMemBusy
 * \brief Checks if the SPI memory is busy.
 * \param slaveSelect Slave select line.
 * \retval bool true if busy, false otherwise.
 */
bool Cy_SPI_IsMemBusy(cy_en_smif_slave_select_t slaveSelect)
{
    return Cy_SMIF_MemIsBusy(SMIF_HW, smifMemConfigs[0], &qspiContext);
}

/**
 * \name Cy_SPI_Start
 * \brief Initializes the SPI interface and flash device.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
cy_en_smif_status_t Cy_SPI_Start(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t CFIMap[CY_CFI_TABLE_LENGTH]={0};
    
    status = Cy_SMIF_Init(SMIF_HW, &qspiConfig, 10000u, &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);

    Cy_SMIF_Enable(SMIF_HW, &qspiContext);
   
    status = Cy_SMIF_MemInit_NoXIP(SMIF_HW, &smifBlockConfig, &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    if(status == CY_SMIF_SUCCESS) {
        glSfdpSupported = true;
#if BL_DEBUG
        DBG_APP_INFO("Num HybridReg: %d\r\n", deviceCfg_SFDP_SlaveSlot_0.hybridRegionCount);
        for (int looper = 0; looper < deviceCfg_SFDP_SlaveSlot_0.hybridRegionCount; looper++) {
            DBG_APP_INFO("0x%x 0x%x 0x%x 0x%x 0x%x\r\n",deviceCfg_SFDP_SlaveSlot_0.hybridRegionInfo[looper]->regionAddress,
                    deviceCfg_SFDP_SlaveSlot_0.hybridRegionInfo[looper]->sectorsCount,
                    deviceCfg_SFDP_SlaveSlot_0.hybridRegionInfo[looper]->eraseCmd,
                    deviceCfg_SFDP_SlaveSlot_0.hybridRegionInfo[looper]->eraseSize,
                    deviceCfg_SFDP_SlaveSlot_0.hybridRegionInfo[looper]->eraseTime);
        }
#endif
    }

    status = Cy_SPI_ReadCFIMap(CFIMap, slaveSelect);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /* Update memslot params from CFI table if SFDP is not supported */
    Cy_SPI_UpdateMemoryCfg(&cfiFlashMap);

    /*
     * Added as a workaround as it was seen that the program size was set to
     * 512 Bytes in SFDP table for SFS256 flash, wheres as the hardware had 256B
     * as default program size. CFI table had 256B as page size, hence using the
     * value from CFI table for program size.
     * */
    if((cfiFlashMap.programSize > 0) &&
            (smifBlockConfig.memConfig[0]->deviceCfg->programSize != cfiFlashMap.programSize)) {
        DBG_APP_INFO("Program size changed to %d\r\n",cfiFlashMap.programSize);
        smifBlockConfig.memConfig[0]->deviceCfg->programSize = cfiFlashMap.programSize;
    }

#if BL_DEBUG
    Cy_Debug_AddToLog(1,"SPI Initialization Done\r\n");
    DBG_APP_INFO("SPI Clock = %d\r\n",Cy_SysClk_ClkHfGetFrequency(CY_SYSCLK_SPI_CLK_HF1));
    if(CY_SMIF_SUCCESS == status)
    {
        DBG_APP_INFO("CFI Map:\r\n");
        Cy_App_PrintBuffer(CFIMap, CY_CFI_TABLE_LENGTH);
    }
#endif
    return status;
}

/**
 * \name Cy_SPI_Stop
 * \brief Deinitializes the SPI interface and flash device.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
cy_en_smif_status_t Cy_SPI_Stop(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    /* Exit 4-byte addressing mode using soft reset.
     * If soft reset method is not supported, user application should either use
     * 4 Byte addressing mode in their application, or the application
     * should use other methods (hard reset/custom reset commands/config register updates)
     * to exit out of 4 Byte addressing mode.*/

    Cy_SFDP_ExitFourByteAddressingMode(SMIF_HW, slaveSelect, &qspiContext);
    Cy_SysClk_ClkHfDisable(CY_SYSCLK_SPI_CLK_HF1);
    Cy_SPI_DeinitSMIFPins();
    Cy_SMIF_MemDeInit(SMIF_HW);
    Cy_SMIF_Disable(SMIF_HW);
    return status;
}

/**
 * \name Cy_SPI_SectorEraseCFI
 * \brief Erases a sector using CFI information.
 * \param slaveSelect Slave select line.
 * \param eraseCmd Erase command.
 * \param address Address to erase.
 * \retval cy_en_smif_status_t SMIF status.
 */
static cy_en_smif_status_t Cy_SPI_SectorEraseCFI(cy_en_smif_slave_select_t slaveSelect, uint32_t eraseCmd,
        uint32_t address) {
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT];
    AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT);

    status = Cy_SPI_WriteEnable(slaveSelect);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if (status == CY_SMIF_SUCCESS)
    {
        status =  Cy_SMIF_TransmitCommand(SMIF_HW,
                eraseCmd,
                CY_SMIF_WIDTH_SINGLE,
                addrArray,
                SPI_ADDRESS_BYTE_COUNT,
                CY_SMIF_WIDTH_SINGLE,
                slaveSelect,
                CY_SMIF_TX_LAST_BYTE,
                &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}

/**
 * \name Cy_SPI_SectorErase
 * \brief Erases a sector or block in the SPI flash.
 * \param slaveSelect Slave select line.
 * \param blockAddress Block address to erase.
 * \param eraseLength Length to erase.
 * \retval cy_en_smif_status_t SMIF status.
 */
cy_en_smif_status_t Cy_SPI_SectorErase(cy_en_smif_slave_select_t slaveSelect, uint32_t blockAddress,
        uint32_t eraseLength)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t index = 0;
    uint32_t hybridIndex = 0;
    uint32_t numEraseRegions = cfiFlashMap.numEraseRegions;

    blockAddress = blockAddress * CY_APP_SPI_FLASH_ERASE_SIZE;
    if(glSfdpSupported) {
        status = Cy_SMIF_MemEraseSector(SMIF_HW, smifMemConfigs[0], blockAddress,eraseLength, &qspiContext);
    }
    else
    {
        if(blockAddress >= cfiFlashMap.deviceSize)
        {
            return CY_SMIF_BAD_PARAM;
        }

        //Check if memory has any 4KB sector regions
        if(cfiFlashMap.num4KBParameterRegions)
        {
            for(index = 0; index < numEraseRegions; index++)
            {
                //Check if the blockAddress to be erased is in a 4KB Hybrid region
                if((blockAddress >= cfiFlashMap.memoryLayout[index].startingAddress)  &&
                        (blockAddress <= cfiFlashMap.memoryLayout[index].lastAddress) &&
                        (cfiFlashMap.memoryLayout[index].sectorSize == 0x1000))
                {
                    //The blockAddress is present in a 4 KB sector region. Erase the entire 4KB region.
                    for(hybridIndex = 0; hybridIndex < cfiFlashMap.memoryLayout[index].numSectors; hybridIndex++)
                    {
                        status = Cy_SPI_SectorEraseCFI(slaveSelect, CY_SPI_4HYBRID_SECTOR_ERASE_CMD,
                                blockAddress + (hybridIndex * 0x1000));
                        if(status == CY_SMIF_SUCCESS)
                        {
                            status = Cy_SPI_WaitTillMemNotBusy(slaveSelect, CY_APP_SPI_HYBRID_ERASE_TIMEOUT_US);
                            if(status != CY_SMIF_SUCCESS) {
                                break;
                            }
                        }
                    }
                }
            }
        }

        //Do a uniform sector erase command to erase all non-4KB sector areas. This command has no effect on the 4KB regions.
        status = Cy_SPI_SectorEraseCFI(slaveSelect, CY_SPI_4SECTOR_ERASE_CMD, blockAddress);
        if(status == CY_SMIF_SUCCESS)
        {
            status = Cy_SPI_WaitTillMemNotBusy(slaveSelect, CY_APP_SPI_HYBRID_ERASE_TIMEOUT_US);
        }
    }
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/**
 * \name Cy_SPI_WriteOperation
 * \brief Writes data to the SPI flash.
 * \param startingAddress Start address to write.
 * \param txBuffer Data buffer to write.
 * \param length Number of bytes to write.
 * \param numPages Number of pages to write.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t startingAddress, uint8_t *txBuffer, uint32_t length, uint32_t numPages, cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    status = Cy_SMIF_MemWrite(SMIF_HW, smifMemConfigs[0], startingAddress,
            txBuffer, length, &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(!status)
    {
#if BL_DEBUG
        Cy_Debug_AddToLog(1, "Write %d done\r\n", startingAddress);
#endif
    }
    return status;
}

/**
 * \name Cy_SPI_ReadOperation
 * \brief Reads data from the SPI flash.
 * \param address Start address to read.
 * \param rxBuffer Buffer to store read data.
 * \param length Number of bytes to read.
 * \param slaveSelect Slave select line.
 * \retval cy_en_smif_status_t SMIF status.
 */
cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_MemRead(SMIF_HW, smifMemConfigs[0], address,
            rxBuffer, length, &qspiContext);

    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/**
 * \name Cy_App_CheckStatus
 * \brief Checks the status of an operation and optionally blocks on error.
 * \param function Name of the function.
 * \param line Line number.
 * \param condition Condition to check.
 * \param value Value to report.
 * \param isBlocking If true, block on error.
 * \retval None
 */
void Cy_App_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
#if BL_DEBUG
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
#endif
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

