/***************************************************************************//**
* \file cy_usb_usbd.h
* \version 1.0
*
* \brief This file contains all the declaration related to USBD layer.
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

#if !defined(CY_USB_USBD_H)
#define CY_USB_USBD_H


#include "cy_usbhs_cal_drv.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#endif /* FREERTOS_ENABLE */

/* Various Descriptor types as per stansdard */
#define CY_USB_DSCR_TYPE_DEVICE             (0x01u)
#define CY_USB_DSCR_TYPE_CFG                (0x02u)
#define CY_USB_DSCR_TYPE_STR                (0x03u)
#define CY_USB_DSCR_TYPE_INTF               (0x04u)
#define CY_USB_DSCR_TYPE_ENDP               (0x05u)
#define CY_USB_DSCR_TYPE_DEVICE_QUALIFIER   (0x06u)


#define CY_USB_DSCR_TYPE_INTF_ASSOC         (0x0Bu)
#define CY_USB_DSCR_TYPE_DEVICE_CAP         (0x10u)
#define CY_USB_DSCR_TYPE_SS_ENDP_COMP       (0x30u)
#define CY_USB_DSCR_TYPE_SSP_ISO_ENDP_COMP  (0x31u)


/* Various common dscr Index */
#define CY_USB_DSCR_OFFSET_LEN               (0x00)
#define CY_USB_DSCR_OFFSET_TYPE              (0x01)

/* Various descriptor length as per Standard. */
#define CY_USB_DEVICE_DSCR_LEN              (18)
#define CY_USB_DEVICE_QUAL_DSCR_LEN         (10)

/* define to decode setup packet from SETUP0 and SETUP1. */
#define CY_USB_BMREQUEST_SETUP0_MASK      (0x000000FFu)
#define CY_USB_BMREQUEST_SETUP0_POS       (0u)
#define CY_USB_BREQUEST_SETUP0_MASK       (0x0000FF00u)
#define CY_USB_BREQUEST_SETUP0_POS        (8u)
#define CY_USB_WVALUE_SETUP0_MASK         (0xFFFF0000u)
#define CY_USB_WVALUE_SETUP0_POS          (16u)
#define CY_USB_WINDEX_SETUP1_MASK         (0x0000FFFFu)
#define CY_USB_WINDEX_SETUP1_POS          (0u)
#define CY_USB_WLENGTH_SETUP1_MASK         (0xFFFF0000u)
#define CY_USB_WLENGTH_SETUP1_POS          (16u)


/* defines to decode USB Request types from bmRequest type */
#define CY_USB_CTRL_REQ_TYPE_MASK              (0x60)
#define CY_USB_CTRL_REQ_TYPE_POS               (5u)
#define CY_USB_CTRL_REQ_STD                    (0x00u)
#define CY_USB_CTRL_REQ_CLASS                  (0x01u)
#define CY_USB_CTRL_REQ_VENDOR                 (0x02u)
#define CY_USB_CTRL_REQ_RESERVED               (0x03u)


/* defines to decode recipient from bmRequest type. */
#define CY_USB_CTRL_REQ_RECIPENT_MASK          (0x1Fu)
#define CY_USB_CTRL_REQ_RECIPENT_POS           (0u)
#define CY_USB_CTRL_REQ_RECIPENT_DEVICE        (0u)
#define CY_USB_CTRL_REQ_RECIPENT_INTF          (1u)
#define CY_USB_CTRL_REQ_RECIPENT_ENDP          (2u)
#define CY_USB_CTRL_REQ_RECIPENT_OTHERS        (3u)
#define CY_USB_CTRL_REQ_RECIPENT_RSVD          (4u)

/* Extracting descriptor type and index from wValue. */
#define CY_USB_CTRL_REQ_DSCR_TYPE_MASK          (0xFF00u)
#define CY_USB_CTRL_REQ_DSCR_TYPE_POS           (8u)
#define CY_USB_CTRL_REQ_DSCR_INDEX_MASK         (0x00FFu)
#define CY_USB_CTRL_REQ_DSCR_INDEX_POS          (0u)


/* defines for GET STATUS command  */
#define CY_USB_GET_STATUS_DEV_SELF_POWER_POS          (0u)
#define CY_USB_GET_STATUS_DEV_SELF_POWER              (0x01u)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP_MASK      (0x0002)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP_POS       (1u)
#define CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP           (0x02u)

#define CY_USB_GET_STATUS_ENDP_HALT_POS               (0u)
#define CY_USB_GET_STATUS_ENDP_HALT                   (1u)

/* various offset in device descriptor */
#define CY_USB_DEVICE_DSCR_OFFSET_MAX_PKT_SIZE      (0x07)
#define CY_USB_DEVICE_DSCR_OFFSET_NUM_CONFIG        (0x08)

/* various offset in config descriptor */
#define CY_USB_CFG_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_CFG_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_CFG_DSCR_OFFSET_TOTAL_LEN            (0x02)
#define CY_USB_CFG_DSCR_OFFSET_NUM_INTF             (0x04)
#define CY_USB_CFG_DSCR_OFFSET_CFG_VALUE            (0x05)
#define CY_USB_CFG_DSCR_OFFSET_ATTRIBUTE            (0x07)
#define CY_USB_CFG_DSCR_OFFSET_MAX_POWER            (0x08)

#define CY_USB_CFG_DSCR_SELF_POWER_MASK             ((uint8_t)0x40)
#define CY_USB_CFG_DSCR_REMOTE_WAKEUP_MASK          ((uint8_t)0x20)

/* various offset in interface descriptor  */
#define CY_USB_INTF_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_INTF_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_INTF_DSCR_OFFSET_NUMBER               (0x02)
#define CY_USB_INTF_DSCR_OFFSET_ALT_SETTING          (0x03)
#define CY_USB_INTF_DSCR_OFFSET_NUM_ENDP             (0x04)

/* various offset in endpoint descriptor  */
#define CY_USB_ENDP_DSCR_OFFSET_LEN                  (0x00)
#define CY_USB_ENDP_DSCR_OFFSET_TYPE                 (0x01)
#define CY_USB_ENDP_DSCR_OFFSET_ADDRESS              (0x02)
#define CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE            (0x03)
#define CY_USB_ENDP_DSCR_OFFSET_MAX_PKT              (0x04)
#define CY_USB_ENDP_DSCR_OFFSET_INTERVAL             (0x06)

/* various offset in SS endpoint Companion descriptor  */
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_LEN                (0x00)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_TYPE               (0x01)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMAX_BURST         (0x02)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_BMATTRIBUTE        (0x03)
#define CY_USB_SS_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL (0x04)

/* various offset in SS PLUS ISO endpoint Companion descriptor  */
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_LEN                (0x00)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_TYPE               (0x01)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_RES                (0x02)
#define CY_USB_SSP_ISO_ENDP_COMPN_DSCR_OFFSET_WBYTE_PER_INTERVAL (0x04)


/* various Device capability code  */
#define CY_USB_DEVICE_CAP_TYPE_WUSB                 (0x01u)
#define CY_USB_DEVICE_CAP_TYPE_USB2_EXT             (0x02u)
#define CY_USB_DEVICE_CAP_TYPE_SS_USB               (0x03u)
#define CY_USB_DEVICE_CAP_TYPE_CONTAINTER_ID        (0x04u)
#define CY_USB_DEVICE_CAP_TYPE_SS_PLUS              (0x0Au)
#define CY_USB_DEVICE_CAP_TYPE_PTM                  (0x0Bu)

/* various offset in USB2.0 EXT descriptor  */
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_USB2_EXT_OFFSET_ATTRIBUTE            (0x03)


/* Various offset in SS capability descriptor */
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_ATTRIBUTE            (0x03)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_SPEED_SUPPORT        (0x04)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_FUNC_SUPPORT         (0x06)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_U1_EXIT_LAT          (0x07)
#define CY_USB_CAP_DSCR_SSUSB_OFFSET_U2_EXIT_LAT          (0x08)

/* Container ID OFFSET */
#define CY_USB_CAP_DSCR_CID_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_CID_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_CID_OFFSET_CAP_TYPE             (0x02)
#define CY_USB_CAP_DSCR_CID_OFFSET_RESERVE              (0x03)
#define CY_USB_CAP_DSCR_CID_OFFSET_ID                   (0x04)

/* various offset in SS PLUS Device capability descriptor */
#define CY_USB_CAP_DSCR_SSP_OFFSET_LEN                  (0x00)
#define CY_USB_CAP_DSCR_SSP_OFFSET_TYPE                 (0x01)
#define CY_USB_CAP_DSCR_SSP_OFFSET_CAP_TYPE             (0x02)

/* QUEUE related define  */
#define CY_USB_USBD_MSG_QUEUE_SIZE 32
#define CY_USB_USBD_MSG_SIZE (sizeof(cy_stc_usb_cal_msg_t))

/* Number of interface supported in a configuration. */
#define CY_USB_MAX_INTF 2

/* wMaxPacketSize mask. */
#define CY_USB_ENDP_MAX_PKT_SIZE_MASK   (0x07FFU)

/* Additional transactions per microframe mask. */
#define CY_USB_ENDP_ADDL_XN_MASK        (0x18U)

/* Additional transactions per microframe field bit position. */
#define CY_USB_ENDP_ADDL_XN_POS         (3U)

typedef struct cy_stc_usb_usbd_ctxt_ cy_stc_usb_usbd_ctxt_t;
typedef void (* cy_usb_usbd_callback_t) (void *pAppCtxt, 
                                         cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_stc_usb_cal_msg_t *pMsg);

/* typdedef to register varius descriptors. */
typedef enum cy_en_usb_set_dscr_type_ {
    CY_USB_SET_SS_DEVICE_DSCR,
    CY_USB_SET_HS_DEVICE_DSCR,
    CY_USB_SET_DEVICE_QUAL_DSCR,
    CY_USB_SET_FS_CONFIG_DSCR,
    CY_USB_SET_HS_CONFIG_DSCR,
    CY_USB_SET_SS_CONFIG_DSCR,
    CY_USB_SET_STRING_DSCR,
    CY_USB_SET_SS_BOS_DSCR,
} cy_en_usb_set_dscr_type_t;

/* typedef to register various callbacks */
typedef enum cy_en_usb_usbd_cb_ {
    CY_USB_USBD_CB_RESET,
    CY_USB_USBD_CB_RESET_DONE,
    CY_USB_USBD_CB_BUS_SPEED,
    CY_USB_USBD_CB_SETUP,
    CY_USB_USBD_CB_SUSPEND,
    CY_USB_USBD_CB_RESUME,
    CY_USB_USBD_CB_SET_CONFIG,
    CY_USB_USBD_CB_SET_INTF,
    CY_USB_USBD_CB_STATUS_STAGE_COMP,
    CY_USB_USBD_CB_L1_SLEEP,
    CY_USB_USBD_CB_L1_RESUME,
    CY_USB_USBD_CB_ZLP,
    CY_USB_USBD_CB_SLP,
    CY_USB_USBD_CB_DONE,
    CY_USB_USBD_CB_DISCONNECT,
    CY_USB_USBD_CB_SET_INVALID
} cy_en_usb_usbd_cb_t;

typedef enum cy_en_usb_device_state_
{
    CY_USB_DEVICE_STATE_DISABLE = 0x00, /* When device is not visible on BUS */
    CY_USB_DEVICE_STATE_ENABLE,         /* When device is visible on BUS. */
    CY_USB_DEVICE_STATE_ATTACHED,       /* When Device visible and VBUS detected. */
    CY_USB_DEVICE_STATE_POWER,          /* When VBUS visible. */
    CY_USB_DEVICE_STATE_RESET,          /* when Bus reset applied. */
    CY_USB_DEVICE_STATE_DEFAULT,        /* when RESET complete. */
    CY_USB_DEVICE_STATE_ADDRESS,        /* When SET_ADDRESS complete. */
    CY_USB_DEVICE_STATE_CONFIGURED,     /* When device is in configured state. */
    CY_USB_DEVICE_STATE_SUSPEND,        /* When USB link is suspended (L2/U3) */
    CY_USB_DEVICE_STATE_HS_L1,          /* When USBHS link is in L1 state. */
    CY_USB_DEVICE_STATE_INVALID,        /* Unused state. */
}cy_en_usb_device_state_t;

typedef enum cy_en_usb_endp0_state_
{
    CY_USB_ENDP0_STATE_IDLE = 0x00,
    CY_USB_ENDP0_STATE_SETUP,
    CY_USB_ENDP0_STATE_DATAIN,
    CY_USB_ENDP0_STATE_DATAOUT,
    CY_USB_ENDP0_STATE_STATUS,
    CY_USB_ENDP0_STATE_STALL
}cy_en_usb_endp0_state_t;

/* Various standard request as per USB specification. */
typedef enum cy_en_usb_std_req_
{
    CY_USB_SC_GET_STATUS = 0x00,
    CY_USB_SC_CLEAR_FEATURE,
    CY_USB_SC_RESERVED,
    CY_USB_SC_SET_FEATURE,
    CY_USB_SC_SET_ADDRESS = 0x05,
    CY_USB_SC_GET_DESCRIPTOR,
    CY_USB_SC_SET_DESCRIPTOR,
    CY_USB_SC_GET_CONFIGURATION,
    CY_USB_SC_SET_CONFIGURATION,
    CY_USB_SC_GET_INTERFACE,
    CY_USB_SC_SET_INTERFACE,
    CY_USB_SC_SYNC_FRAME,
    CY_USB_SC_SET_SEL = 0x30,       /**< Set system exit latency. */
    CY_USB_SC_SET_ISOC_DELAY        /**< Set isochronous delay. */
} cy_en_usb_std_req_t;

/* Various standard descriptor as per USB secification */
typedef enum cy_en_usb_dscr_type_ {
    CY_USB_DEVICE_DSCR = 0x01,
    CY_USB_CONFIG_DSCR,
    CY_USB_STRING_DSCR,
    CY_USB_INTR_DSCR,
    CY_USB_ENDP_DSCR,
    CY_USB_DEVICE_QUAL_DSCR,
    CY_USB_OTHERSPEED_DSCR,
    CY_USB_BOS_DSCR = 0x0F,
    CY_DEVICE_CAPB_DSCR,
    CY_SS_ENDP_COMPN_DSCR = 0x30,
    CY_SSPLUS_ISO_ENDP_COMPN_DSCR = 0x31
} cy_en_usb_dscr_type_t;

/* defines used in set/clear feature command */
typedef enum cy_en_usb_feature_selector_ {
    CY_USB_FEATURE_ENDP_HALT = 0,
    CY_USB_FEATURE_DEVICE_REMOTE_WAKE = 1,
    CY_USB_FEATURE_DEVICE_TEST_MODE = 2,
    CY_USB_FEATURE_U1_ENABLE   = 48,            /**< USB 3.0 U1 Enable. */
    CY_USB_FEATURE_U2_ENABLE   = 49,            /**< USB 3.0 U2 Enable. */
    CY_USB_FEATURE_LTM_ENABLE  = 50             /**< USB 3.0 LTM Enable. */
}cy_en_usb_feature_selector_t;

/* define for enumeration method. */
typedef enum cy_en_usb_enum_method_ {
    CY_USB_ENUM_METHOD_FAST = 0,    /* USBD layer handles enumeration process. */
    CY_USB_ENUM_METHOD_APPLICATION, /* Application layer will handle enumeration. */
}cy_en_usb_enum_method_t;

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usbd_ret_code_ {
    CY_USBD_STATUS_SUCCESS=0,
    CY_USBD_STATUS_FAILURE,
    CY_USBD_STATUS_BAD_PARAM,
    CY_USBD_STATUS_CTXT_NULL,
    CY_USBD_STATUS_PTR_NULL,
    CY_USBD_STATUS_INVALID_CALLBACK_TYPE,
    CY_USBD_STATUS_INVALID_DSCR_TYPE,
    CY_USBD_STATUS_INVALID_INDEX,
    CY_USBD_STATUS_INVALID_CONFIG_NUMBER,
    CY_USBD_STATUS_MALLOC_FAILED,
    CY_USBD_STATUS_MSG_SEND_FAIL,
    CY_USBD_STATUS_CTRL_REQ_HANDLE_FAIL,
    CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM,
    CY_USBD_STATUS_ENDP_CONFIG_FAIL,
    CY_USBD_STATUS_TIMEOUT,
    CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED,
}cy_en_usbd_ret_code_t;


/*
 *  cy_stc_usb_set_dscr_ptrs_t:
 *  Application registers required descriptors through given API and descriptors
 *  will be stored in this data structure.
 */
typedef struct cy_stc_usb_set_dscr_ptrs_
{
    uint8_t     *pUsbDevDscr;
    uint8_t     *pUsbDevQualDscr;
    uint8_t     *pUsbCfgDscr;
    uint8_t     *pUsbOtherSpeedCfgDscr;
    uint8_t     *pUsbFsCfgDscr;
    uint8_t     *pUsbHsCfgDscr;
    uint8_t     *pUsbStringDescr[CY_USBD_MAX_STR_DSCR_INDX + 1];
}cy_stc_usb_set_dscr_ptrs_t;

/*
 * endpoint related info/configuration stored here.
 * Tried to map endp_config_t used in CAL layer and
 * endp_info used in USBD layer. This is the reason
 * some of the variable you seen as uint32 instead of
 * uint8_t.
 */
typedef struct cy_stc_usb_endp_info_
{
    bool valid;
    bool mapped;
    bool halt;
    bool allowNakTillDmaRdy;
    cy_en_usb_endp_type_t endpType;
    uint32_t maxPktSize;
    uint32_t mult;
    uint32_t burstSize;
    uint32_t streamID;
}cy_stc_usb_endp_info_t;

/* Map USBD DMA types and function names to DMAC types and functions. */
typedef cy_stc_dmac_descriptor_t cy_stc_usbd_dma_descr_t;
typedef cy_stc_dmac_descriptor_config_t cy_stc_usbd_dma_descr_conf_t;
typedef cy_stc_dmac_channel_config_t cy_stc_usbd_dma_chn_conf_t;

#define CY_USBD_DMA_CHN_DISABLED CY_DMAC_CHANNEL_DISABLED
#define CY_USBD_DMA_CHN_ENABLED CY_DMAC_CHANNEL_ENABLED
#define CY_USBD_DMA_RETRIG_IM CY_DMAC_RETRIG_IM
#define CY_USBD_DMA_WAIT_FOR_REACT CY_DMAC_WAIT_FOR_REACT
#define CY_USBD_DMA_DESCR_CHAIN CY_DMAC_DESCR_CHAIN
#define CY_USBD_DMA_X_LOOP CY_DMAC_X_LOOP
#define CY_USBD_DMA_BYTE CY_DMAC_BYTE
#define CY_USBD_DMA_WORD CY_DMAC_WORD
#define CY_USBD_DMA_XFER_SIZE_DATA CY_DMAC_TRANSFER_SIZE_DATA
#define CY_USBD_DMA_1D_XFER CY_DMAC_1D_TRANSFER
#define CY_USBD_DMA_2D_XFER CY_DMAC_2D_TRANSFER

#define Cy_USBD_DMADesc_SetSrcAddress Cy_DMAC_Descriptor_SetSrcAddress
#define Cy_USBD_DMADesc_SetDstAddress Cy_DMAC_Descriptor_SetDstAddress
#define Cy_USBD_DMADesc_SetXloopDataCount Cy_DMAC_Descriptor_SetXloopDataCount
#define Cy_USBD_DMADesc_SetYloopDataCount Cy_DMAC_Descriptor_SetYloopDataCount
#define Cy_USBD_DMADesc_SetYloopSrcIncrement Cy_DMAC_Descriptor_SetYloopSrcIncrement
#define Cy_USBD_DMADesc_SetYloopDstIncrement Cy_DMAC_Descriptor_SetYloopDstIncrement
#define Cy_USBD_DMADesc_SetNextDescriptor Cy_DMAC_Descriptor_SetNextDescriptor
#define Cy_USBD_DMADesc_SetChannelState Cy_DMAC_Descriptor_SetChannelState
#define Cy_USBD_DMADesc_Init Cy_DMAC_Descriptor_Init
#define Cy_USBD_DMAChn_Init Cy_DMAC_Channel_Init
#define Cy_USBD_DMAChn_SetDesc Cy_DMAC_Channel_SetDescriptor
#define Cy_USBD_DMAChn_Enable Cy_DMAC_Channel_Enable
#define Cy_USBD_DMAChn_Disable Cy_DMAC_Channel_Disable
#define Cy_USBD_DMAChn_GetIntrStatus Cy_DMAC_Channel_GetInterruptStatus
#define Cy_USBD_DMAChn_ClearIntr(dmabase, channel, value) Cy_DMAC_Channel_ClearInterrupt((dmabase), (channel), (value))

#define Cy_USBD_EP0In_DmaBase(pUsbdCtxt)        pUsbdCtxt->pCpuDmacBase
#define Cy_USBD_EP0Out_DmaBase(pUsbdCtxt)       pUsbdCtxt->pCpuDmacBase

#define CY_USBD_EGREP_OUT_TRIG          TRIG_IN_MUX_5_USBHSDEV_TR_OUT16
#define CY_USBD_EGREP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN16
#define CY_USBD_EGREP_DMA_TRIG_BASE     TRIG_OUT_MUX_5_MDMA_TR_IN0
#define CY_USBD_EGREP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_MDMA_TR_OUT0
#define CY_USBD_INGEP_OUT_TRIG          TRIG_IN_MUX_5_USBHSDEV_TR_OUT0
#define CY_USBD_INGEP_IN_TRIG           TRIG_OUT_MUX_8_USBHSDEV_TR_IN0
#define CY_USBD_INGEP_DMA_TRIG_BASE     TRIG_OUT_MUX_5_MDMA_TR_IN0
#define CY_USBD_INGEP_DMAOUT_TRIG_BASE  TRIG_IN_MUX_8_MDMA_TR_OUT0

/* main data structure which represent device. */
struct cy_stc_usb_usbd_ctxt_
{
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    uint8_t devAddr;
    cy_en_usb_enum_method_t enumMethod;
    cy_en_usb_endp0_state_t endp0State;

    cy_stc_usb_set_dscr_ptrs_t dscrs;
    uint8_t *pActiveCfgDscr;
    uint8_t activeCfgNum;
    uint16_t altSettings[CY_USB_MAX_INTF];

    uint8_t selfPowered;
    uint8_t remoteWakeupAbility;
    uint8_t remoteWakeupEnable;

    cy_stc_usb_setup_req_t setupReq;
    /*
     * waiting for Data makes it active. In active state if one more
     * request comes then need to clear ep0 buffer and DMA channels.
     */
    bool setupReqActive;
    /*
     * flag indicating whether this is the first data being sent on
     * EP0-IN. We need to trigger DMA transfer manually in this case.
     */
    bool ep0SendDone;

    /* Flag indicating whether LPM entry has been disabled by application. */
    bool lpmDisabled;

    cy_stc_usb_endp_info_t endpInfoIn[CY_USB_MAX_ENDP_NUMBER];
    cy_stc_usb_endp_info_t endpInfoOut[CY_USB_MAX_ENDP_NUMBER];

    /* EP0 DMA related fields. */
    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr0;
    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr1;
    cy_stc_usbd_dma_descr_t dmaCh0InXferDscr2;
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr0;
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr1;
    cy_stc_usbd_dma_descr_t dmaCh1OutXferDscr2;

    uint32_t channel0;
    uint32_t channel1;

    /** Bus reset callback notification */
    cy_usb_usbd_callback_t busResetCb;
    /** Bus reset done callback notification */
    cy_usb_usbd_callback_t busResetDoneCb;
    /** Bus speed callback notification */
    cy_usb_usbd_callback_t busSpeedCb;
    /** Bus setup callback notification */
    cy_usb_usbd_callback_t setupCb;
    /** Bus suspend callback notification */
    cy_usb_usbd_callback_t suspendCb;
    /** Bus resume callback notification */
    cy_usb_usbd_callback_t resumeCb;
    /** setConfig callback notification */
    cy_usb_usbd_callback_t setConfigCb;
    /** setInt callback notification */
    cy_usb_usbd_callback_t setIntfCb;
    /** status stage completion callback notification */
    cy_usb_usbd_callback_t statusStageComplCb;

    cy_usb_usbd_callback_t zlpCb;
    cy_usb_usbd_callback_t slpCb;
    cy_usb_usbd_callback_t doneCb;
    cy_usb_usbd_callback_t DisconnectCb;

    /* DMA related fields. */
    DMAC_Type *pCpuDmacBase;

    cy_stc_usb_cal_ctxt_t *pCalCtxt;
    void *pAppCtxt;

    /* Copy of USB Other Speed configuration descriptor. */
    uint8_t otherSpeedCfgDscrBuf[256U];
    bool EnumerationDone;
    bool strDscrAvailable;
};

/* API Provided by USB layer  */
uint32_t Cy_USBD_GetVersion(void);
cy_en_usbd_ret_code_t Cy_USBD_RegisterCallback
                                        (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_en_usb_usbd_cb_t callBackType,
                                         cy_usb_usbd_callback_t callBackFunc);
cy_en_usbd_ret_code_t Cy_USBD_SetDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         cy_en_usb_set_dscr_type_t dscrType,
                                         uint8_t dscrIndex, uint8_t *pDscr);


/* Device descriptor related functions. */
cy_en_usb_speed_t Cy_USBD_GetDeviceSpeed(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
void Cy_USBD_SetDeviceSpeed(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_en_usb_speed_t speed);
void Cy_USBD_SetDeviceSpeedAtUSBDOnly(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                      cy_en_usb_speed_t speed);

cy_en_usb_device_state_t Cy_USBD_GetDeviceState
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
cy_en_usbd_ret_code_t Cy_USBD_FindEndp0MaxPktSize(uint8_t *pDevDscr,
                                                  cy_en_usb_speed_t devSpeed,
                                                  uint32_t *pMaxPktSize);

/* Config descriptor related functions. */
bool Cy_USBD_isCfgValid(uint8_t cfgNum, const uint8_t *pCfgDscr);
cy_en_usbd_ret_code_t Cy_USB_USBD_GetActiveCfgNum
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint8_t *pCfgNum);
uint8_t *Cy_USB_USBD_GetActiveCfgDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
uint8_t Cy_USBD_FindNumOfIntf(const uint8_t *pCfgDscr);
bool Cy_USBD_FindSelfPower(const uint8_t *pCfgDscr);
bool Cy_USBD_FindRemoteWakeupAbility(uint8_t *pCfgDscr);
bool Cy_USBD_GetRemoteWakeupStatus(cy_stc_usb_usbd_ctxt_t *pUsbdCtx);
void Cy_USBD_SignalRemoteWakeup(cy_stc_usb_usbd_ctxt_t *pUsbdCtx,
                                bool startEnd);

/* Interface descriptor related functions. */
uint8_t *Cy_USBD_GetIntfDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t intfNum,
                            uint8_t altSetting);
bool Cy_USBD_isIntfValid(uint16_t intf, uint8_t *pCfgDscr);
uint8_t Cy_USBD_FindNumOfEndp(uint8_t *pIntfDscr);
uint8_t *Cy_USBD_GetEndpDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pIntfDscr);
uint8_t *Cy_USBD_GetSsEndpCompDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    uint8_t *pEndpDscr);
uint8_t *Cy_USBD_GetSspIsoEndpCompDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pEndpDscr);
cy_en_usbd_ret_code_t Cy_USB_USBD_GetActiveAltSetting
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint8_t intfNum,
                                           uint8_t *pAltSetting);
uint8_t Cy_USBD_FindAltSetting(uint8_t *pIntfDscr);

/* Endpoint repated functions. */
bool Cy_USBD_EndpDscrValid(uint8_t *pEndpDscr);
cy_en_usbd_ret_code_t Cy_USBD_GetEndpNumMaxPktDir(uint8_t *pEndpDscr,
	                                          uint32_t *pEndpNum,
                                                  uint16_t *pMaxPktSize,
						  uint32_t *pDir);
cy_en_usbd_ret_code_t Cy_USBD_GetEndpMaxPktSize(uint8_t *pEndpDscr,
                                                uint16_t *pMaxPktSize);
cy_en_usbd_ret_code_t Cy_USBD_GetEndpAttribute(uint8_t *pEndpDscr,
                                               uint8_t *pAttribute);
cy_en_usbd_ret_code_t Cy_USBD_GetEndpType(uint8_t *pEndpDscr,
		                          uint32_t *pEndpType);
cy_en_usbd_ret_code_t Cy_USBD_GetEndpInterval(uint8_t *pEndpDscr,
                                              uint8_t *pInterval);

cy_en_usbd_ret_code_t Cy_USBD_EnableEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                        uint32_t endpNumber,
                                         cy_en_usb_endp_dir_t endpDirection,
                                                                 bool enable);
cy_en_usbd_ret_code_t Cy_USBD_SetEpBurstMode
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        bool enable);
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpConfig(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_endp_config_t endpConfig);
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpSetClearStall
                                        (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                         uint32_t endpNumber,
                                         cy_en_usb_endp_dir_t endpDirection,
                                                                bool setClear);
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpSetClearNakNrdyAll
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                                bool setClear);
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearNakNrdy (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                                bool setClear);

bool Cy_USBD_EndpIsNakNrdySet
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection);
bool Cy_USBD_EndpIsStallSet(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            uint32_t endpNumber,
                            cy_en_usb_endp_dir_t endpDirection);
cy_en_usbd_ret_code_t Cy_USBD_EndpMapStream
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        uint16_t streamId,
                                        uint32_t socketNum);
void Cy_USBD_FlushEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection);

void Cy_USBD_ResetEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                   cy_en_usb_endp_dir_t endpDir, bool preserveSeqNo);

void Cy_USBD_FlushResetEndpAll(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_en_usb_endp_dir_t endpDirection);

void Cy_USBD_UpdateXferCount(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint32_t endpNumber,
                             cy_en_usb_endp_dir_t endpDirection,
                             uint32_t bufferSize);
void Cy_USBD_SendAckSetupDataStatusStage(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0Data
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);
                                       
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0DataSs
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);
                                     
cy_en_usbd_ret_code_t Cy_USB_USBD_SendEndp0DataHs
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);

cy_en_usbd_ret_code_t Cy_USB_USBD_RecvEndp0Data
                                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       uint8_t *pBuffer, uint32_t bufferSize);

cy_en_usbd_ret_code_t 
Cy_USB_USBD_RecvEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                                    uint8_t *pBuffer, uint32_t bufferSize);
                                    
cy_en_usbd_ret_code_t 
Cy_USB_USBD_RecvEndp0DataSs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                                    uint8_t *pBuffer, uint32_t bufferSize);

void Cy_USBD_ConnectDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_en_usb_speed_t usbSpeed);
void Cy_USBD_ResetUsbdCommonDs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
void Cy_USBD_DisconnectDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

cy_en_usbd_ret_code_t Cy_USBD_HandleSsDisconnect
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                             cy_stc_usb_cal_msg_t *pMsg);

cy_en_usbd_ret_code_t Cy_USBD_HandleMsg(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);

cy_en_usbd_ret_code_t Cy_USBD_HandleReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleSsReset(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleHsGrant(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleResetDone
                                       (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                        cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleCtrlXfrSetupStage
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleStatusStage
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleSuspend(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleResume(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleZlp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleSlp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                                 cy_stc_usb_cal_msg_t *pMsg);

cy_en_usbd_ret_code_t Cy_USBD_HandleSetInterface
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq,
                                            cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleGetInterface
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq);
cy_en_usbd_ret_code_t Cy_USBD_HandleSetConfiguration
                                          (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           cy_stc_usb_setup_req_t setupReq,
                                           cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleGetConfiguration
                                            (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq);
cy_en_usbd_ret_code_t Cy_USBD_HandleClearFeature
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq,
                                              cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleSetFeature
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            cy_stc_usb_setup_req_t setupReq,
                                            cy_stc_usb_cal_msg_t *pMsg);
cy_en_usbd_ret_code_t Cy_USBD_HandleGetStatus
                                           (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq);

cy_en_usbd_ret_code_t Cy_USBD_HandleGetDscr(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                              cy_stc_usb_setup_req_t setupReq,
                                              cy_stc_usb_cal_msg_t *pMsg);


/* Set of initialization functions related to EP0-DMA */
cy_en_usbd_ret_code_t Cy_USB_USBD_ResetController(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
void Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig(cy_stc_usbd_dma_descr_conf_t
                                             *pEndp0InCpuDmaDscrConfig,
                                             bool first);
void Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig(cy_stc_usbd_dma_descr_conf_t
                                              *pEndp0OutdscrConfig, bool first);
void Cy_USB_USBD_InitCpuDmaChannelCfg(cy_stc_usbd_dma_chn_conf_t *pDmaChCfg,
                                            cy_stc_usbd_dma_descr_t *pDmaDscr);
cy_en_usbd_ret_code_t Cy_USB_USBD_cpuDmaInit(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

/* Set of initialization functions related to USB */
void  Cy_USBD_InitUsbDscrPtrs(cy_stc_usb_set_dscr_ptrs_t *pDscr);

cy_en_usbd_ret_code_t Cy_USB_USBD_Init(void *pAppCtxt,
                                       cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                       DMAC_Type *pCpuDmacBase, 
                                       cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USB_USBD_DisableHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
void Cy_USB_USBD_EnableHsDevice(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);
cy_en_usbd_ret_code_t Cy_USB_USBD_EndpInit(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);



cy_en_usbd_ret_code_t Cy_USB_USBD_SendNrdy(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint32_t endpNumber,
                                           cy_en_usb_endp_dir_t endpDirection,
                                           uint16_t bulkStream);
cy_en_usbd_ret_code_t Cy_USB_USBD_SendErdy(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                           uint32_t endpNumber,
                                           cy_en_usb_endp_dir_t endpDirection,
                                           uint8_t numP, uint16_t bulkStream);

void Cy_USBD_TaskHandler(void *pTaskParam);
bool Cy_USBD_ProcessMsg(void *pUsbd, void *pCalMgs);
cy_en_usbd_ret_code_t Cy_USBD_SendEgressZLP(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                            uint32_t endpNumber);
cy_en_usbd_ret_code_t Cy_USBD_ClearZlpSlpIntrEnableMask
                      (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNumber,
                       cy_en_usb_endp_dir_t endpDirection, bool zlpSlp);
cy_en_usbd_ret_code_t Cy_USBD_GetUSBLinkActive (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

void Cy_USB_USBD_RetireRecvEndp0DataHs(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

cy_en_usbd_ret_code_t Cy_USB_USBD_RetireRecvEndp0Data(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt);

cy_en_usbd_ret_code_t Cy_USBD_DebugRegisterCallback (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
        uint8_t debugOutEp, cy_en_usb_usbd_cb_t callBackType, cy_usb_usbd_callback_t callBackFunc);

#endif /* (CY_USB_USBD_H) */

/* [EOF] */

