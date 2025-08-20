/***************************************************************************//**
 * \file cy_spi_smif.h
 * \version 1.0
 *
 * \brief Macros and definitions for SPI drievr for bootloader
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

#ifndef _SPI_SMIF_H_
#define _SPI_SMIF_H_

#include "cy_pdl.h"
#include "cy_debug.h"


#define ASSERT(condition, value) Cy_App_CheckStatus(__func__, __LINE__, condition, value, true);

#if BL_DEBUG
#define ASSERT_NON_BLOCK(condition, value) Cy_App_CheckStatus(__func__, __LINE__, condition, value, false);
#else
#define ASSERT_NON_BLOCK(condition, value) 
#endif

#define SMIF_HW (SMIF0)
#define CY_SYSCLK_SPI_CLK_HF1 (1)

#define SMIF_CLK_HSIOM (P6_0_SMIF_SPI_CLK)
#define SMIF_CLK_PORT (P6_0_PORT)
#define SMIF_CLK_PIN (P6_0_PIN)

#define SMIF_SELECT0_HSIOM (P6_1_SMIF_SPI_SELECT0)
#define SMIF_SELECT0_PORT (P6_1_PORT)
#define SMIF_SELECT0_PIN (P6_1_PIN)

#define SMIF_SELECT1_HSIOM (P6_2_SMIF_SPI_SELECT1)
#define SMIF_SELECT1_PORT (P6_2_PORT)
#define SMIF_SELECT1_PIN (P6_2_PIN)

#define SMIF_DATA0_HSIOM (P7_0_SMIF_SPI_DATA0)
#define SMIF_DATA0_PORT (P7_0_PORT)
#define SMIF_DATA0_PIN (P7_0_PIN)

#define SMIF_DATA1_HSIOM (P7_1_SMIF_SPI_DATA1)
#define SMIF_DATA1_PORT (P7_1_PORT)
#define SMIF_DATA1_PIN (P7_1_PIN)

#define SMIF_DATA2_HSIOM (P7_2_SMIF_SPI_DATA2)
#define SMIF_DATA2_PORT (P7_2_PORT)
#define SMIF_DATA2_PIN (P7_2_PIN)

#define SMIF_DATA3_HSIOM (P7_3_SMIF_SPI_DATA3)
#define SMIF_DATA3_PORT (P7_3_PORT)
#define SMIF_DATA3_PIN (P7_3_PIN)

#define SMIF_DATA4_HSIOM                (P7_4_SMIF_SPI_DATA4)
#define SMIF_DATA4_PORT                 (P7_4_PORT)
#define SMIF_DATA4_PIN                  (P7_4_PIN)

#define SMIF_DATA5_HSIOM                (P7_5_SMIF_SPI_DATA5)
#define SMIF_DATA5_PORT                 (P7_5_PORT)
#define SMIF_DATA5_PIN                  (P7_5_PIN)

#define SMIF_DATA6_HSIOM                (P7_6_SMIF_SPI_DATA6)
#define SMIF_DATA6_PORT                 (P7_6_PORT)
#define SMIF_DATA6_PIN                  (P7_6_PIN)

#define SMIF_DATA7_HSIOM                (P7_7_SMIF_SPI_DATA7)
#define SMIF_DATA7_PORT                 (P7_7_PORT)
#define SMIF_DATA7_PIN                  (P7_7_PIN)


#define CY_SPI_CONFIG_REG_READ_CMD        (0x35)
#define CY_SPI_WRITE_REGISTER_CMD         (0x01)
#define CY_SPI_WRITE_ENABLE_CMD           (0x06)
#define CY_SPI_READ_ID_CMD                (0x9F)
#define CY_SPI_STATUS_READ_CMD            (0x05)
#define CY_SPI_PROGRAM_CMD                (0x02)
#define CY_SPI_4PROGRAM_CMD               (0x12)
#define CY_SPI_READ_CMD                   (0x03)
#define CY_SPI_4READ_COMMAND              (0x13)
#define CY_SPI_SECTOR_ERASE_CMD           (0xD8)
#define CY_SPI_4SECTOR_ERASE_CMD          (0xDC)
#define CY_SPI_HYBRID_SECTOR_ERASE_CMD    (0x20)
#define CY_SPI_4HYBRID_SECTOR_ERASE_CMD   (0x21)
#define CY_SPI_READ_STATUS_REG_1_CMD      (0x05)
#define CY_SPI_READ_STATUS_REG_2_CMD      (0x07)
#define CY_APP_SPI_CHIP_ERASE_CMD         (0xC7)
#define CY_APP_SPI_READ_ANY_REG_CMD       (0x65)
#define CY_APP_SPI_WRITE_ANY_REGISTER_CMD (0x71)


#define CY_SPI_STATUS_REG_1 (0x01)
#define CY_SPI_STATUS_REG_2 (0x02)

#define SPI_ADDRESS_BYTE_COUNT              (4)
#define CY_FLASH_ID_LENGTH                  (0x08)
#define CY_CFI_TABLE_LENGTH                 (0x56)
#define CY_SFDP_TABLE_LENGTH                (0x80)
#define CY_SPI_CONFIG_REGISTER_NV3          (0x04)
#define CY_SPI_DISABLE_HYBRID_SECTOR        (1 << 3) /* Disable 4-KB Erase (Uniform Sector Architecture) */


/**
 * \enum cy_en_flash_programmer_vendor_cmd_t
 * \brief Vendor commands sent by USB Host application (e.g., control center) for SPI flash operations.
 * These commands are used to communicate with the bootloader for various flash operations such as checking SPI support,
 * reading/writing/erasing flash, and getting device ID.
 */
typedef enum cy_en_flash_programmer_vendor_cmd_t
{
    FLASH_CMD_CHECK_SPI_SUPPORT     = 0xB0, /**< Check if SPI is supported. */
    FLASH_CMD_CHECK_STATUS          = 0xB5, /**< Check status of the flash device. */
    FLASH_CMD_FLASH_WRITE           = 0xB2, /**< Write data to flash. */
    FLASH_CMD_FLASH_READ            = 0xB3, /**< Read data from flash. */
    FLASH_CMD_FLASH_SECTOR_ERASE    = 0xB4, /**< Erase a sector in flash. */
} cy_en_flash_programmer_vendor_cmd_t;


#define CY_APP_SPI_FLASH_ERASE_SIZE        (0x0010000)
#define CY_APP_SPI_FLASH_PAGE_SIZE         (0x100)
#define CY_APP_SPI_MAX_USB_TRANSFER_SIZE   (0x800)
#define CY_APP_SPI_PROGRAM_TIMEOUT_US      (2000)
#define CY_APP_SPI_HYBRID_ERASE_TIMEOUT_US (650000)

#define CY_CFI_DEVICE_SIZE_OFFSET           (0x27)
#define CY_CFI_ERASE_NUM_SECTORS_OFFSET     (0x2D)
#define CY_CFI_ERASE_REGION_SIZE_INFO_SIZE  (0x04)
#define CY_CFI_ERASE_SECTOR_SIZE_OFFSET     (0x2F)
#define CY_CFI_MAX_SIZE_NUM_ERASE_SECTORS   (0xFF)
#define CY_CFI_NUM_ERASE_REGION_OFFSET      (0x2C)
#define CY_CFI_PROGRAM_SIZE_OFFSET          (0x2A)
#define CY_CFI_QRY_STRING_OFFSET            (0x10)

#define CY_APP_READ_4_BYTES_CMD_1S_1S_1S    (0x13)

typedef struct cy_stc_cfi_erase_block_info_t
{
    uint32_t numSectors;
    uint32_t sectorSize;
    uint32_t startingAddress;
    uint32_t lastAddress;
} cy_stc_cfi_erase_block_info_t;


typedef struct cy_stc_cfi_flash_map_t
{
    uint32_t isValid;
    uint32_t deviceSizeFactor; /*0x27h*/
    uint32_t deviceSize;
    uint32_t programSize;        /*0x2A*/
    uint32_t numEraseRegions;    /*0x2c*/
    cy_stc_cfi_erase_block_info_t memoryLayout[CY_CFI_MAX_SIZE_NUM_ERASE_SECTORS]; /* 0x2D onwards*/
    uint32_t num4KBParameterRegions;

} cy_stc_cfi_flash_map_t;



extern cy_stc_smif_context_t qspiContext;

/**
 * \brief Stops the SPI interface for the specified slave select.
 * \param ss The slave select to stop.
 * \retval cy_en_smif_status_t Status of the stop operation.
 */
cy_en_smif_status_t Cy_SPI_Stop(cy_en_smif_slave_select_t ss);

/**
 * \brief Starts the SPI interface for the specified slave select.
 * \param ss The slave select to start.
 * \retval cy_en_smif_status_t Status of the start operation.
 */
cy_en_smif_status_t Cy_SPI_Start(cy_en_smif_slave_select_t ss);

/**
 * \brief Checks if the memory device is busy for the given slave select.
 * \param slaveSelect The slave select to check.
 * \retval bool True if memory is busy, false otherwise.
 */
bool Cy_SPI_IsMemBusy(cy_en_smif_slave_select_t slaveSelect);

/**
 * \brief Performs a write operation to the SPI memory.
 * \param address The starting address to write to.
 * \param txBuffer Pointer to the data buffer to write.
 * \param length Number of bytes to write.
 * \param numPages Number of pages to write.
 * \param ss The slave select to use.
 * \retval cy_en_smif_status_t Status of the write operation.
 */
cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t address, uint8_t *txBuffer, uint32_t length, uint32_t numPages, cy_en_smif_slave_select_t ss);

/**
 * \brief Performs a read operation from the SPI memory.
 * \param address The starting address to read from.
 * \param rxBuffer Pointer to the buffer to store read data.
 * \param length Number of bytes to read.
 * \param ss The slave select to use.
 * \retval cy_en_smif_status_t Status of the read operation.
 */
cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_smif_slave_select_t ss);

/**
 * \brief Erases a sector or block in the SPI memory.
 * \param slaveSelect The slave select to use.
 * \param blockAddress The starting address of the block to erase.
 * \param eraseLength The length of memory to erase.
 * \retval cy_en_smif_status_t Status of the erase operation.
 */
cy_en_smif_status_t Cy_SPI_SectorErase(cy_en_smif_slave_select_t slaveSelect, uint32_t blockAddress,
        uint32_t eraseLength);

/**
 * \brief Checks the status and handles errors for application code.
 * \param function Name of the function calling this check.
 * \param line Line number in the source file.
 * \param condition Condition to check (nonzero is pass).
 * \param value Value to report if the condition fails.
 * \param isBlocking If true, blocks on error; if false, does not block.
 */
void Cy_App_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

/**
 * \brief Checks if a valid SPI flash device is detected.
 * \retval uint32_t Nonzero if a valid flash is detected, zero otherwise.
 */
uint32_t Cy_SPI_ValidFlashDetected(void);

/**
 * \brief Exits 4-byte addressing mode for SFDP-compatible SPI flash devices.
 * \param base Pointer to the SMIF hardware base.
 * \param slaveSelect The slave select to use.
 * \param context Pointer to the SMIF context structure.
 * \retval cy_en_smif_status_t Status of the operation.
 */
cy_en_smif_status_t Cy_SFDP_ExitFourByteAddressingMode(SMIF_Type *base, cy_en_smif_slave_select_t slaveSelect,
        cy_stc_smif_context_t const *context);

#endif
