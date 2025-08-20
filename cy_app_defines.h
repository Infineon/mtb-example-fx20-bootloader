/***************************************************************************//**
* \file cy_app_defines.h
* \version 1.0
*
* \brief Macros and definitions for the application
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

#ifndef _CY_APP_DEFINES_H_
#define _CY_APP_DEFINES_H_

#include "cy_pdl.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_crypto_common.h"
#include "cy_debug.h"
#include "cy_spi_smif.h"
#include "cybsp.h"
#if BL_DEBUG
#include "cy_usbfs_cdc.h"
#endif /* BL_DEBUG */

#if defined(__cplusplus)
extern "C" {
#endif

/* Added for debug purposes */
#define ENABLE_JUMP_TO_APPLN        (1)
#define ENABLE_FLASH_PROGRAMMER     (1)

#define MS_VENDOR_CODE               (0xF0)


/* Stack pointer in the vector table should be within bounds of the System RAM. */
#define FX3G2_BL_VECTOR_SPTR_VALID(sp)                  \
    (((sp) >= 0x08000000UL) && ((sp) <= 0x08020000UL))

/* Reset vector in the vector table should be within the allowed flash bounds and should have THUMB bit set. */
#define FX3G2_BL_VECTOR_START_VALID(rv)                 \
    (((rv) >= 0x10008000UL) && ((rv) <= 0x1007E000UL) && (((rv) & 0x01UL) != 0))


#define MIN(a,b)               ((a) > (b)) ? (b) : (a)


#if BL_DEBUG
#define FX3G2_BL_SIZE                (0xB000u)
#else
/* Size of flash reserved for the bootloader. */
#define FX3G2_BL_SIZE                (0x8000u)
#endif

/* Base address of flash on FX3G2 devices. */
#define FX3G2_FLASH_BASE_ADDR        (0x10000000u)

/* First FX3G2_BL_SIZE bytes of flash is reserved for BL and application is expected to start immediately after this. */
#define JUMP_ADDRESS                 (FX3G2_FLASH_BASE_ADDR + FX3G2_BL_SIZE)

/* User flash size on CYUSB3083-FCAXI, CYUSB3084-FCAXI, CYUSB4013-FCAXI and CYUSB4014-FCAXI parts. */
#define FX3G2_MAX_FLASH_SIZE         (0x80000u)

#define FX3G2_BL_END_ADDRESS        (FX3G2_FLASH_BASE_ADDR + FX3G2_BL_SIZE)

/* User flash size on CYUSB3081-FCAXI, CYUSB3082-FCAXI, CYUSB4011-FCAXI and CYUSB4012-FCAXI parts. */
#define FX3G2_LTD_FLASH_SIZE         (0x40000u)

/* Address of register that reports device flash size. */
#define CPUSS_FLASH_SIZE_ADDR        (0x402020C0u)

/* Size of one flash row on FX3G2 device. */
#define FX3G2_FLASH_ROW_SIZE         (0x200u)

#define DFU_BM_REQUEST_TYPE_0        (0x21)
#define DFU_BM_REQUEST_TYPE_1        (0xA1)
#define CHARMAP(c)                   (((c) >= 10) ? ('A' + (c) - 10) : ('0' + (c)))

/** Retrieves byte 0 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE0(d)        ((uint8_t)((d) & 0xFF))

/** Retrieves byte 1 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE1(d)        ((uint8_t)(((d) >>  8) & 0xFF))

/** Retrieves byte 2 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE2(d)        ((uint8_t)(((d) >> 16) & 0xFF))

/** Retrieves byte 3 from a 32 bit number */
#define CY_USB_DWORD_GET_BYTE3(d)        ((uint8_t)(((d) >> 24) & 0xFF))


/*macros for DFU requests */
#define DFU_REQ_DETACH                  (0x0)
#define DFU_REQ_DNLOAD                  (0x1)
#define DFU_REQ_UPLOAD                  (0x2)
#define DFU_REQ_GETSTATUS               (0x3)
#define DFU_REQ_CLRSTATUS               (0x4)
#define DFU_REQ_GETSTATE                (0x5)
#define DFU_REQ_ABORT                   (0x6)

/**
 * \brief DFU state machine states for USB Device Firmware Upgrade process.
 */
typedef enum
{
    DFU_APP_IDLE = 0,           /**< Application idle state. */
    DFU_APP_DETACH,             /**< Application detach state. */
    DFU_IDLE,                   /**< DFU idle state. */
    DFU_DNLOAD_SYNC,            /**< DFU download sync state. */
    DFU_DNBUSY,                 /**< DFU download busy state. */
    DFU_DNLOAD_IDLE,            /**< DFU download idle state. */
    DFU_MANIFEST_SYNC,          /**< DFU manifest sync state. */
    DFU_MANIFEST,               /**< DFU manifest state. */
    DFU_MANIFEST_WAIT_RESET,    /**< DFU manifest wait reset state. */
    DFU_UPLOAD_IDLE,            /**< DFU upload idle state. */
    DFU_ERROR                   /**< DFU error state. */
}cy_en_dfu_state_t;

/**
 * \brief Structure representing the DFU_GETSTATUS response as per USB DFU specification.
 */
typedef struct __attribute__ ((aligned (4)))
{
    uint8_t bStatus;            /**< Status code. */
    uint8_t bwPollTimeout[3];   /**< Minimum time in milliseconds before the device is ready to accept a new request. */
    uint8_t bState;             /**< Current state of the device. */
    uint8_t iString;            /**< Index of status description string. */
} cy_stc_dfu_get_status_response_t;

/**
 * \brief Structure holding device information such as flash size and silicon ID.
 */
typedef struct __attribute__ ((aligned (4)))
{
    uint32_t flashSize;         /**< Size of the internal flash in KB. */
    uint32_t siliconId;         /**< Unique silicon identifier. */
} cy_stc_device_info_t;

#if defined(__cplusplus)
}
#endif
#endif /*_CY_APP_DEFINES_H_*/
