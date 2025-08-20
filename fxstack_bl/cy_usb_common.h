/***************************************************************************//**
* \file cy_usb_common.h
* \version 1.0
*
* \brief Provides common definitions used in the USB Device Stack. 
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

#ifndef _CY_USB_COMMON_H_
#define _CY_USB_COMMON_H_

#include <stdbool.h>
//#include <stdint.h>

/* This file will be shared across all the layers ie CAL/USBD/APPLICATION. */
#ifndef NULL
#define NULL 0
#endif /* NULL */

#define FALSE 0
#define TRUE 1

#define CY_USBHS_EGR_EPM_BASE_ADDR  0x30000000UL
#define CY_USBHS_ING_EPM_BASE_ADDR  0x30004000UL

#define CY_USB_MAX_ENDP_NUMBER     1
#define CY_USB_ENDP_0              0
#define CY_USBD_MAX_STR_DSCR_INDX  16

#define CY_USB_ARG_ZLP 0x01
#define CY_USB_ARG_SLP 0x00

#define CY_USB_MIN(arg1,arg2) (((arg1) <= (arg2)) ? (arg1): (arg2))
#define CY_USB_MAX(arg1,arg2) (((arg1) >= (arg2)) ? (arg1): (arg2))

/* From uchar array create world */
#define CY_USB_BUILD_2B_WORD(byte1, byte0)  \
                             ((uint16_t)((byte1 << 8) | byte0))

#define CY_USB_BUILD_4B_WORD(byte3,byte2,byte1,byte0) \
             ((uint32_t)((((byte3) << 24) | ((byte2) << 16) | \
                         ((byte1) << 8) | (byte0)))

typedef bool (* cy_usb_cal_msg_callback_t) (void *pUsbdCtxt, void *pMsg);

/* 
 * CAL layer return this code when it's API is called.
 * This return code is shared between CAL and USBD layer.
 */
typedef enum cy_en_usbss_cal_ret_code_ {
    CY_USB_CAL_STATUS_SUCCESS=0,
    CY_USB_CAL_STATUS_FAILURE,
    CY_USB_CAL_STATUS_BAD_PARAM,
    CY_USB_CAL_STATUS_CAL_CTXT_NULL,
    CY_USB_CAL_STATUS_MALLOC_FAILED,
    CY_USB_CAL_STATUS_MSG_SEND_FAIL,
    CY_USB_CAL_STATUS_CAL_BASE_NULL,
    CY_USB_CAL_STATUS_TOGGLE_FAILED,
    CY_USB_CAL_STATUS_CAL_PROT_BASE_NULL,
    CY_USB_CAL_STATUS_CAL_LNK_BASE_NULL,
    CY_USB_CAL_STATUS_CAL_EPM_BASE_NULL,
    CY_USB_CAL_STATUS_CAL_MAIN_BASE_NULL
}cy_en_usb_cal_ret_code_t;


/* These message prepared and sent by CAL layer to USBD layer. */
typedef enum cy_en_usb_cal_msg_type_ {
    CY_USB_CAL_MSG_INVALID=0,
    CY_USB_CAL_MSG_RESET,
    CY_USB_CAL_MSG_HSGRANT,
    CY_USB_CAL_MSG_RESET_DONE,
    CY_USB_CAL_MSG_SUDAV,
    CY_USB_CAL_MSG_STATUS_STAGE,
    CY_USB_CAL_MSG_SETADDR,
    CY_USB_CAL_MSG_PROT_SETADDR_0,
    CY_USB_CAL_MSG_SUSP,
    CY_USB_CAL_MSG_RESUME_START,
    CY_USB_CAL_MSG_RESUME_END,
    CY_USB_CAL_MSG_DEEP_SLEEP_EXIT,
    CY_USB_CAL_MSG_L1_SLEEP,
    CY_USB_CAL_MSG_L1_URESUME,
    CY_USB_CAL_MSG_SOF,
    CY_USB_CAL_MSG_PROT_SUTOK,
    CY_USB_CAL_MSG_ERRLIMIT,
    CY_USBSS_CAL_MSG_LNK_RESET,
    CY_USBSS_CAL_MSG_LNK_CONNECT,
    CY_USBSS_CAL_MSG_LNK_DISCONNECT,
    CY_USBSS_CAL_MSG_EP_INT,
    CY_USBSS_CAL_MSG_LNK_INT,
    CY_USBSS_CAL_MSG_PROT_INT,
    CY_USBSS_CAL_MSG_PROT_EP_INT,
    CY_USBSS_CAL_MSG_VBUS_CHANGE,
    CY_USBSS_CAL_MSG_USB3_COMPLIANCE,
    CY_USBSS_CAL_MSG_USB3_WARM_RESET,
    CY_USBSS_CAL_MSG_USB3_U3_SUSPEND,
    CY_USBSS_CAL_MSG_USB3_U0_RESUME,
    CY_USBSS_CAL_MSG_UX_REENABLE,
    CY_USBSS_CAL_MSG_TRY_UX_EXIT,
    CY_USBSS_CAL_MSG_LNK_ERR_LIMIT,
    CY_USBSS_CAL_MSG_EPM_UNDERRUN,
    CY_USBSS_CAL_MSG_USB3_LNKFAIL,
    CY_USBSS_CAL_MSG_USB3_LMP_FAIL,
    CY_USB_CAL_MSG_IN_ZLP,
    CY_USB_CAL_MSG_IN_SLP,
    CY_USB_CAL_MSG_OUT_ZLP,
    CY_USB_CAL_MSG_OUT_SLP,
    CY_USB_CAL_MSG_PROT_HOST_ERR,
    CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CAP,
    CY_USB_CAL_MSG_PROT_TIMEOUT_PORT_CFG,
    CY_USB_CAL_MSG_PROT_TIMEOUT_PING,
    CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CAP,
    CY_USB_CAL_MSG_PROT_LMP_INVALID_PORT_CFG,
    CY_USB_CAL_MSG_OUT_DONE,
    CY_USB_CAL_MSG_MAX
}cy_en_usb_cal_msg_type_t;

typedef struct cy_stc_usbhs_cal_msg_ {
    cy_en_usb_cal_msg_type_t type;
    uint32_t data[2];
}cy_stc_usb_cal_msg_t;

/* defines supported speed in FX3G2. */
typedef enum cy_en_usb_speed_
{
    CY_USBD_USB_DEV_NOT_CONNECTED = 0x00,
    CY_USBD_USB_DEV_FS,
    CY_USBD_USB_DEV_HS,
    CY_USBD_USB_DEV_SS_GEN1,
    CY_USBD_USB_DEV_SS_GEN1X2,
    CY_USBD_USB_DEV_SS_GEN2,
    CY_USBD_USB_DEV_SS_GEN2X2,
} cy_en_usb_speed_t;

/* Control request on endpoint 0 */
typedef struct cy_stc_usb_setup_req_ {
    uint8_t bmRequest;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}cy_stc_usb_setup_req_t;

/* defines type of endpoint.  */
typedef enum cy_en_usb_endp_type_ {
    CY_USB_ENDP_TYPE_CTRL=0,
    CY_USB_ENDP_TYPE_ISO,
    CY_USB_ENDP_TYPE_BULK,
    CY_USB_ENDP_TYPE_INTR,
    CY_USB_ENDP_TYPE_INVALID
}cy_en_usb_endp_type_t;

/* defines direction of endpoint. */
typedef enum cy_en_usb_endp_dir_ {
    CY_USB_ENDP_DIR_OUT=0,
    CY_USB_ENDP_DIR_IN,
    CY_USB_ENDP_DIR_INVALID
}cy_en_usb_endp_dir_t;


/* structure to configure endpoint registers. */
typedef struct cy_stc_usb_endp_config_
{
    cy_en_usb_endp_type_t endpType;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpNumber;
    uint32_t maxPktSize;
    uint32_t isoPkts;
    uint32_t burstSize;
    uint32_t streamID;
    bool allowNakTillDmaRdy; /* Applicable only for out endp */
} cy_stc_usb_endp_config_t;



/*******************************************************************************
* Function Name: Cy_USBHS_CalculateEpmAddr
****************************************************************************//**
*
* Returns the base address of the Endpoint Memory region corresponding
* to a USBHSDEV endpoint.
*
* \param endpNum
* Endpoint index (valid range is 0 to 15).
*
* \param endpDirection
* Direction of the endpoint
*
* \return
* The base address of the endpoint memory region for the endpoint.
*
*******************************************************************************/
static inline uint32_t *
Cy_USBHS_CalculateEpmAddr (uint32_t endpNum, cy_en_usb_endp_dir_t endpDirection)
{
    if (endpNum >= CY_USB_MAX_ENDP_NUMBER)
    {
        return NULL;
    }

    if (endpDirection == CY_USB_ENDP_DIR_IN)
    {
        return((uint32_t *)(CY_USBHS_EGR_EPM_BASE_ADDR + (endpNum << 10U)));
    }

    return((uint32_t *)(CY_USBHS_ING_EPM_BASE_ADDR + (endpNum << 10U)));
}

#endif /* _CY_USB_COMMON_H_ */

