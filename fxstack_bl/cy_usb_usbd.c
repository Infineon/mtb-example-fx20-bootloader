/***************************************************************************//**
* \file cy_usb_usbd.c
* \version 1.0
*
* \brief USBD implementation for bootloader
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
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usbd_version.h"
#include <string.h>
#include "cy_debug.h"
/*
 * Function: Cy_USB_USBD_Init()
 * Description: This function initializes USBD layer and
 *              calls CAL layer initialization function.
 * Parameter: none.
 * return: None
 * Note:
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_Init (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                  DMAC_Type *pCpuDmacBase, cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    cy_en_usbd_ret_code_t  retCode = CY_USBD_STATUS_SUCCESS;
    uint32_t intfNum;
#if BL_DEBUG
    DBG_USBD_INFO("Cy_USB_USBD_Init  >>\r\n");
#endif
    pUsbdCtxt->pCalCtxt = pCalCtxt;
    pUsbdCtxt->pAppCtxt = pAppCtxt;
    pUsbdCtxt->pCpuDmacBase = pCpuDmacBase;
    pUsbdCtxt->channel0 = 0x00;
    pUsbdCtxt->channel1 = 0x01;

    /* Make HS device Invisible on BUS */
    Cy_USB_USBD_DisableHsDevice(pUsbdCtxt);

    /* Initialize dscr pointers so that application can set required dscr */
    Cy_USBD_InitUsbDscrPtrs(&(pUsbdCtxt->dscrs));

    /* Reset all callbacks */
    pUsbdCtxt->busResetCb = NULL;
    pUsbdCtxt->busResetDoneCb = NULL;
    pUsbdCtxt->busSpeedCb = NULL;
    pUsbdCtxt->setupCb = NULL;
    pUsbdCtxt->suspendCb = NULL;
    pUsbdCtxt->resumeCb = NULL;
    pUsbdCtxt->setConfigCb = NULL;
    pUsbdCtxt->setIntfCb = NULL;
    pUsbdCtxt->statusStageComplCb = NULL;

    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
    }

    /* initialize USBD data structure.
     * Call Controller initialization function.
     *  - Call FS PHY Initialization function.
     *  - Call HS PHY initialization function.
     *  - HS Clock will be disable in reset_done if required.
     */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->selfPowered = 0x00;
    pUsbdCtxt->remoteWakeupAbility = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;

    /* Initialize the USBHS and USBSS IP */
    Cy_USBHS_Cal_InitUsbController(pUsbdCtxt->pCalCtxt, pUsbdCtxt, Cy_USBD_ProcessMsg);
#if BL_DEBUG
    DBG_USBD_INFO("InitControllerCalled......\r\n");
#endif
    Cy_USB_USBD_EndpInit(pUsbdCtxt);
    retCode = Cy_USB_USBD_cpuDmaInit(pUsbdCtxt);
#if BL_DEBUG
    DBG_USBD_INFO("CPUDmaInitCalled............\r\n");
#endif

    /* Make State Enable, enable interrupt, and then make device visible. */
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ENABLE;
    pUsbdCtxt->strDscrAvailable = FALSE;
    pUsbdCtxt->EnumerationDone = false;

#if BL_DEBUG
    DBG_USBD_INFO("Cy_USB_USBD_Init  <<\r\n");
#endif
    return(retCode);
}   /* end of function Cy_USB_USBD_Init() */

/*
 * Function: Cy_USBD_RegisterCallback()
 * Description: This API will be used by application to register required
 *              callback.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_en_usb_usbd_cb_t,
 *            cy_usb_usbd_callback_t
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_RegisterCallback (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_en_usb_usbd_cb_t callBackType,
                          cy_usb_usbd_callback_t callBackFunc)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("RegCal:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    switch (callBackType) {
        case CY_USB_USBD_CB_RESET:
            pUsbdCtxt->busResetCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_RESET_DONE:
            pUsbdCtxt->busResetDoneCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_BUS_SPEED:
            pUsbdCtxt->busSpeedCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SETUP:
            pUsbdCtxt->setupCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SUSPEND:
            pUsbdCtxt->suspendCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_RESUME:
            pUsbdCtxt->resumeCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SET_CONFIG:
            pUsbdCtxt->setConfigCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SET_INTF:
            pUsbdCtxt->setIntfCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_STATUS_STAGE_COMP:
            pUsbdCtxt->statusStageComplCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_ZLP:
            pUsbdCtxt->zlpCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_SLP:
            pUsbdCtxt->slpCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_DONE:
            pUsbdCtxt->doneCb = callBackFunc;
            break;

        case CY_USB_USBD_CB_DISCONNECT:
            pUsbdCtxt->DisconnectCb = callBackFunc;
            break;

        default:
#if BL_DEBUG
            DBG_USBD_ERR("RegCal:default\r\n");
#endif
            retCode = CY_USBD_STATUS_INVALID_CALLBACK_TYPE;
            break;
    }
    return(retCode);
}   /* end of function  */


/*
 * Function: Cy_USBD_SetDscr()
 * Description: This API will be used by application to provide various
 *              standard descriptors to USBD layer and USBD layer will use
 *              this descriptor in "fast enumeration". It is callers
 *              responsibility to provide proper descriptor with right index.
 * Parameter: pUsbdCtxt, dscrType, dscrIndex, pDscr
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_SetDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                 cy_en_usb_set_dscr_type_t dscrType,
                 uint8_t dscrIndex,
                 uint8_t *pDscr)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;
    uint8_t numIntf;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("SetDscr:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (NULL == pDscr) {
#if BL_DEBUG
        DBG_USBD_ERR("SetDscr:pDscrNull\r\n");
#endif
        return(CY_USBD_STATUS_BAD_PARAM);
    }

    switch (dscrType) {

        case CY_USB_SET_HS_DEVICE_DSCR:
            pUsbdCtxt->dscrs.pUsbDevDscr = pDscr;
            break;

        case CY_USB_SET_DEVICE_QUAL_DSCR:
            pUsbdCtxt->dscrs.pUsbDevQualDscr = pDscr;
            break;

        case CY_USB_SET_FS_CONFIG_DSCR:
            numIntf = *(pDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
            if (numIntf <= CY_USB_MAX_INTF) {
                pUsbdCtxt->dscrs.pUsbFsCfgDscr = pDscr;
            } else {
                pUsbdCtxt->dscrs.pUsbFsCfgDscr = NULL;
                retCode = CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED;
            }
            break;

        case CY_USB_SET_HS_CONFIG_DSCR:
            numIntf = *(pDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF);
            if (numIntf <= CY_USB_MAX_INTF) {
                pUsbdCtxt->dscrs.pUsbHsCfgDscr = pDscr;
            } else {
                pUsbdCtxt->dscrs.pUsbHsCfgDscr = NULL;
                retCode = CY_USBD_STATUS_DSCR_FIELD_NOT_SUPPORTED;
            }
            break;

        case CY_USB_SET_STRING_DSCR:
            if (dscrIndex > CY_USBD_MAX_STR_DSCR_INDX) {
                return(CY_USBD_STATUS_INVALID_INDEX);
            }

            pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex] = pDscr;
            /*
             * If string descriptor is not register (other than index 0)
             * then device should return stall and strDscrAvailable will
             * be used for the same.
             */
            if ((dscrIndex != 0x00) &&
                (pUsbdCtxt->strDscrAvailable == FALSE)) {
                pUsbdCtxt->strDscrAvailable = TRUE;
            }
            break;

        default:
#if BL_DEBUG
            DBG_USBD_ERR("SetDscr:default\r\n");
#endif
            retCode = CY_USBD_STATUS_INVALID_DSCR_TYPE;
            break;
    }
    return(retCode);
}   /* end of function   */


/*
 * Function: Cy_USBD_GetDeviceSpeed()
 * Description: This API returns present device speed.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: cy_en_usb_speed_t.
 */
cy_en_usb_speed_t
Cy_USBD_GetDeviceSpeed (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("GetDevSpe:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_USB_DEV_NOT_CONNECTED);
    }
    return(pUsbdCtxt->devSpeed);
}   /* end of function */


/*
 * Function: Cy_USBD_FindEndp0MaxPktSize()
 * Description: This API finds endpoint 0 max packet size from device
 *              descriptor. whoever define device should write device
 *              dscr properly.
 * Parameter: device descriptor, devSpeed and pMaxPacketsize.
 * return: CY_USBD_STATUS_SUCCESS if function able to retrieve endpoint0 size.
 *         CY_USBD_STATUS_BAD_PARAM in all other case.
 */
cy_en_usbd_ret_code_t
Cy_USBD_FindEndp0MaxPktSize (uint8_t *pDevDscr, cy_en_usb_speed_t devSpeed,
                             uint32_t *pMaxPktSize)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_BAD_PARAM;

    if (NULL != pDevDscr) {
        *pMaxPktSize =
        (*(uint8_t *)(pDevDscr + CY_USB_DEVICE_DSCR_OFFSET_MAX_PKT_SIZE));
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        *pMaxPktSize = 0x00;
    }
    return(retCode);
}   /* end of function */

/*
 * Function: Cy_USBD_GetEndpNumMaxPktDir()
 * Description: This API gets endpoint number, maxPktSize and Direction.
 * Parameter: pEndpDscr, pEndpNum, pMaxPktSize, pDir
 * return: cy_en_usbd_ret_code_t.
 */
cy_en_usbd_ret_code_t
Cy_USBD_GetEndpNumMaxPktDir (uint8_t *pEndpDscr, uint32_t *pEndpNum,
                             uint16_t *pMaxPktSize, uint32_t *pDir)
{
    if (NULL == pEndpDscr) {
        *pMaxPktSize = 0x00;
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    if ((pEndpNum == NULL) || (pMaxPktSize == NULL)) {
#if BL_DEBUG
        DBG_USBD_ERR("GetEndpNumMaxPktDir: NullPointer\r\n");
#endif
        return (CY_USBD_STATUS_PTR_NULL);
    }

    *pEndpNum = ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

    *pMaxPktSize = (*(uint8_t *)(pEndpDscr +CY_USB_ENDP_DSCR_OFFSET_MAX_PKT)) +
        ((*(uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) << 8);
    *pMaxPktSize &= CY_USB_ENDP_MAX_PKT_SIZE_MASK;

    *pDir = (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80);

    return(CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*
 * Function: Cy_USBD_EnableEndp()
 * Description: This function enables/disable endpoint.
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber, endpDirection, enable
 * return: return code
 */
cy_en_usbd_ret_code_t
Cy_USBD_EnableEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNumber,
                    cy_en_usb_endp_dir_t endpDirection,  bool enable)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("EnaEnd:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    calRetCode = Cy_USBHS_Cal_EnableEndp(pUsbdCtxt->pCalCtxt, endpNumber,
                                             endpDirection, enable);

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function */

/*
 * Function: Cy_USB_USBD_EndpConfig()
 * Description: This function takes care of endpoint configuration.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_endp_config_t.
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpConfig (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                        cy_stc_usb_endp_config_t endpConfig)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_endp_info_t *pEndpInfo;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("EndCon:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /*
     * If endpoint number > 15 then error.
     * Endpoint 0 should be configured before reset Done.
     * Other endpoint should be configured only after device in
     * configured state.
     */
    if ((endpConfig.endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        ((endpConfig.endpNumber == CY_USB_ENDP_0) &&
        (pUsbdCtxt->devState >= CY_USB_DEVICE_STATE_DEFAULT)) ||
        ((endpConfig.endpNumber > CY_USB_ENDP_0) &&
        (pUsbdCtxt->devState < CY_USB_DEVICE_STATE_CONFIGURED))) {
        return(CY_USBD_STATUS_ENDP_CONFIG_INVALID_PARAM);
    }

    endpNumber = endpConfig.endpNumber;
    endpDirection = endpConfig.endpDirection;

    if (endpDirection == CY_USB_ENDP_DIR_OUT) {
        pEndpInfo = &(pUsbdCtxt->endpInfoOut[endpNumber]);
    } else {
        pEndpInfo = &(pUsbdCtxt->endpInfoIn[endpNumber]);
    }

    pEndpInfo->maxPktSize = endpConfig.maxPktSize;
    pEndpInfo->endpType = endpConfig.endpType;
    pEndpInfo->valid = endpConfig.valid;
    pEndpInfo->burstSize = endpConfig.burstSize;
    pEndpInfo->streamID = endpConfig.streamID;
    pEndpInfo->valid = endpConfig.valid;
    pEndpInfo->allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;

    /* for endpoint 0  IN/OUT taken care in single call. */
    if (endpDirection == CY_USB_ENDP_DIR_OUT) {
        /*
         * if allowNakTillDmaRdy set then explicitly set NAK bit for that
         * endpoint. With this fist packet will be responded with NAK.
         */
        if (endpConfig.allowNakTillDmaRdy) {
            Cy_USBHS_Cal_EndpSetClearNak(pUsbdCtxt->pCalCtxt,endpNumber,
                                         CY_USB_ENDP_DIR_OUT, true);
        }
    }
    calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function  */


/*
 * Function: Cy_USB_USBD_EndpSetClearStall()
 * Description: This function enable or disable STALL condition.
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber, cy_en_usb_endp_dir_t
 *            setClear.
 * return: cy_en_usbd_ret_code_t
 * Note: It is caller's responsibility to call this function for various endpoint
 * based on required device state.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearStall (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               uint32_t endpNum,
                               cy_en_usb_endp_dir_t endpDirection,
                               bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("EndSetCleSta:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    calRetStatus = Cy_USBHS_Cal_EndpSetClearStall(pUsbdCtxt->pCalCtxt,
                                                  endpNum, endpDirection,
                                                  setClear);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        /* for non endpoint 0, update halt Variable */
        if (endpNum != 0x00) {
            if (setClear) {
                if (CY_USB_ENDP_DIR_IN == endpDirection) {
                    pUsbdCtxt->endpInfoIn[endpNum].halt = TRUE;
                } else {
                    pUsbdCtxt->endpInfoOut[endpNum].halt = TRUE;
                }
            } else {
                if (CY_USB_ENDP_DIR_IN == endpDirection) {
                    pUsbdCtxt->endpInfoIn[endpNum].halt = FALSE;
                } else {
                    pUsbdCtxt->endpInfoOut[endpNum].halt = FALSE;
                }
            }
        }
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* end of function. */



/*
 * Function: Cy_USBD_FlushEndp()
 * Description: This function flush/reset endpoint based on speed of device
 *              controller.
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber, cy_en_usb_endp_dir_t
 * return: None
 */
void
Cy_USBD_FlushEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                        uint32_t endpNumber,
                        cy_en_usb_endp_dir_t endpDirection)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("FluResEnd:usbdCtxtNull\r\n");
#endif
        return;
    }

    if (endpNumber ==  0x00) {
        Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNumber,
                               CY_USB_ENDP_DIR_IN);
        Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNumber,
                               CY_USB_ENDP_DIR_OUT);
    } else {
        Cy_USBHS_Cal_FlushEndp(pUsbdCtxt->pCalCtxt, endpNumber,
                               endpDirection);
    }
    return;
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USBD_ResetEndp
****************************************************************************//**
*
*  This function resets endpoint related paramters by calling appropriate function
*  at HW level. This clears sticky bits e.g retry bit, flowcontrol bit.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param preserveSeqNo
* true to preserve sequence number and false to reset sequence number.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBD_ResetEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNum,
                   cy_en_usb_endp_dir_t endpDir, bool preserveSeqNo)
{
    /* HS does not have endpoint reset implementation.
     * Function added to fix compiler errors
     */
}



/*
 * Function: Cy_USB_USBD_EndpSetClearNakNrdyAll()
 * Description: This function enable or disable NAK/NRDY condition on all
 *              endpoints based on speed of device.
 * Parameter: cy_stc_usb_usbd_ctxt_t, setClear.
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearNakNrdyAll (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("EndSetCleNakNrdAll:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    calRetStatus = Cy_USBHS_Cal_SetClearNakAll(pUsbdCtxt->pCalCtxt,
                                               setClear);

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function   */

/*******************************************************************************
* Function name: Cy_USB_USBD_EndpSetClearNakNrdy
****************************************************************************//**
*
*  This function enable or disable NAK/NRDY condition for the endpoint.
*
* \param pUsbdCtxt
* USBD layer context pointer.
*
* \param endpNum
* endpoint number.
*
* \param endpDir
* endpoint direction.
*
* \param setClear
* true for set and false for clear NAK condition.
*
* \return
* CY_USBD_STATUS_SUCCESS if operation is successful.
* CY_USBD_STATUS_CTXT_NULL if usbd context is NULL.
* CY_USBD_STATUS_FAILURE in other cases.
*
*******************************************************************************/
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpSetClearNakNrdy (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                uint32_t endpNum, cy_en_usb_endp_dir_t endpDir,
                                bool setClear)
{
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_FAILURE;

    if (NULL == pUsbdCtxt) {
        DBG_USBD_ERR("EndSetCleNakNrd:usbdCtxt Null\r\n");
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((endpNum > 0x00) &&
        (pUsbdCtxt->devState < CY_USB_DEVICE_STATE_CONFIGURED)) {
        return(CY_USBD_STATUS_FAILURE);
    }

    if ((pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) ||
         (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS)) {

        calRetStatus = Cy_USBHS_Cal_EndpSetClearNak(pUsbdCtxt->pCalCtxt,
                                                    endpNum, endpDir,
                                                    setClear);
    }

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* end of function. */


/*
 * Function: Cy_USBD_EndpIsNakNrdySet()
 * Description: This function checks whether the specified endpoint currently has the NAK bit set.
 * Parameter:
 *      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt: USBD stack context
 *      uint32_t endpNumber: Endpoint index
 *      cy_en_usb_endp_dir_t endpDirection: Endpoint direction.
 * return: boolean status indicating whether EP NAK/NRDY bit is set.
 */
bool Cy_USBD_EndpIsNakNrdySet (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               uint32_t endpNumber,
                               cy_en_usb_endp_dir_t endpDirection)
{
    bool epNaked = false;

    if ((pUsbdCtxt != NULL) && (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        epNaked = Cy_USBHS_Cal_EndpIsNakNrdySet(pUsbdCtxt->pCalCtxt,
                                                endpNumber, endpDirection);
    }
    return epNaked;
}

/*
 * Function: Cy_USBD_EndpIsStallSet()
 * Description: This function checks whether the specified endpoint
 *              currently has the STALL bit set.
 * Parameter:
 *      cy_stc_usb_usbd_ctxt_t *pUsbdCtxt: USBD stack context
 *      uint32_t endpNumber: Endpoint index
 *      cy_en_usb_endp_dir_t endpDirection: Endpoint direction.
 * return: boolean status indicating whether EP STALL bit is set.
 */
bool Cy_USBD_EndpIsStallSet (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint32_t endpNumber,
                             cy_en_usb_endp_dir_t endpDirection)
{
    bool epStalled = false;

    if ((pUsbdCtxt != NULL) && (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        epStalled = Cy_USBHS_Cal_EndpIsStallSet(pUsbdCtxt->pCalCtxt,
                                                endpNumber, endpDirection);
    }
    return epStalled;
}   /* End of function */

/*
 * Function: Cy_USBD_UpdateXferCount()
 * Description: This function will update xfer count based on speed of device.
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber, endpDirection, bufferSize
 * return: none
 */
void
Cy_USBD_UpdateXferCount (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         uint32_t endpNumber,
                         cy_en_usb_endp_dir_t endpDirection,
                         uint32_t bufferSize)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("UpdXfrCnt:usbdCtxtNull\r\n");
#endif
        return;
    }
    Cy_USBHS_Cal_UpdateXferCount(pUsbdCtxt->pCalCtxt, endpNumber,
                                 endpDirection, bufferSize);
    return;
}   /* End of function */


/*
 * Function: Cy_USBD_SendAckSetupDataStatusStage()
 * Description: Allow chipset to send ACK to complete control transfer.
 * Parameter: cy_stc_usb_usbd_ctxt_t
 * return: none
 */
void
Cy_USBD_SendAckSetupDataStatusStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("SndAckSet:usbdCtxtNull\r\n");
#endif
        return;
    }
    /*
     * We need to make sure that the Link is in L0 and does not go back into L1 until
     * the request handling is complete.
     */
    Cy_USBHS_Cal_GetLinkActive(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_SendAckSetupDataStatusStage(pUsbdCtxt->pCalCtxt);

    return;
}   /* End of function */


static uint32_t Cy_USB_USBD_WaitForXferComplete (
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        DMAC_Type *base,
        uint32_t channel, cy_en_usb_endp_dir_t direction) 
{

    uint32_t dmaIntr = 0;
    uint32_t loopCnt = 2500;

    /* Wait until DMA transfer is completed. */
    dmaIntr = Cy_USBD_DMAChn_GetIntrStatus(base,channel);

    while (dmaIntr == 0)
    {
        loopCnt--;
        if (loopCnt == 0UL) {
            /* Timeout. */
            break;
        }
        Cy_SysLib_Delay(1);
        if(direction == CY_USB_ENDP_DIR_OUT) {
            if(pUsbdCtxt->pCalCtxt->pCalBase->DEV_EP_INGRS_INTR == 0x00010000)
            {
                pUsbdCtxt->pCalCtxt->pCalBase->DEV_EP_INGRS_INTR = 0x00010000;
                Cy_TrigMux_SwTrigger(CY_USBD_INGEP_OUT_TRIG, CY_TRIGGER_TWO_CYCLES);
            }
        }
        dmaIntr = Cy_USBD_DMAChn_GetIntrStatus(base,channel);
    }
    
    return dmaIntr;
}




static void Cy_USB_USBD_CfgEndp0Dma(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        cy_en_usb_endp_dir_t direction, uint8_t *pBuffer,
        uint32_t bytesToXfer) 

{
    cy_stc_usbd_dma_descr_conf_t dscr_config;
    cy_stc_usbd_dma_descr_t *dmaDscr;
    DMAC_Type *dmaBase;
    uint32_t dmaChannel;
    
    uint32_t slpLength = bytesToXfer & 0x3FUL; 
    uint32_t remainingBytes = bytesToXfer;
    cy_stc_usbd_dma_descr_t *dmaDscr2d;
    cy_stc_usbd_dma_descr_t *dmaDscr1d_1;
    cy_stc_usbd_dma_descr_t *dmaDscr1d_2;
    uint8_t *srcAddrBase; 
    uint8_t *dstAddrBase;

    bool needDscr2 = false;
    bool enByteTransfers = false;
    
    if(direction == CY_USB_ENDP_DIR_OUT)
    {
        dmaBase     = Cy_USBD_EP0Out_DmaBase(pUsbdCtxt);
        dmaChannel  = pUsbdCtxt->channel1;
        srcAddrBase = (uint8_t *)0x30004000;
        dstAddrBase = (uint8_t *)pBuffer;
        dmaDscr2d   = &(pUsbdCtxt->dmaCh1OutXferDscr0);
        dmaDscr1d_1 = &(pUsbdCtxt->dmaCh1OutXferDscr1);
        dmaDscr1d_2 = &(pUsbdCtxt->dmaCh1OutXferDscr2);
    }
    else
    {
        dmaBase     = Cy_USBD_EP0In_DmaBase(pUsbdCtxt);
        dmaChannel  = pUsbdCtxt->channel0;
        srcAddrBase = (uint8_t *)pBuffer;
        dstAddrBase = (uint8_t *)0x30000000;
        dmaDscr2d   = &(pUsbdCtxt->dmaCh0InXferDscr0);
        dmaDscr1d_1 = &(pUsbdCtxt->dmaCh0InXferDscr1);
        dmaDscr1d_2 = &(pUsbdCtxt->dmaCh0InXferDscr2);
    }


    /* Do max packet size transfers using 2D DMA transfers */
    if (bytesToXfer >= 64U) {
        /* Need to set src, dst address and update size of transfer ie xCount. */
        dmaDscr = dmaDscr2d;

        Cy_USBD_DMADesc_SetSrcAddress(dmaDscr, srcAddrBase);
        Cy_USBD_DMADesc_SetDstAddress(dmaDscr, dstAddrBase);
        /*Complete 16 x WORD transfers to the nearest multiple of 64 */
        Cy_USBD_DMADesc_SetXloopDataCount(dmaDscr, 0x10UL);
        Cy_USBD_DMADesc_SetYloopDataCount(dmaDscr, (bytesToXfer >> 6U));

        if (direction == CY_USB_ENDP_DIR_IN) {
            Cy_USBD_DMADesc_SetYloopSrcIncrement(dmaDscr, 0x10UL);
            Cy_USBD_DMADesc_SetYloopDstIncrement(dmaDscr, 0x00UL);
        }

        /* Point to the next descriptor if transfer involves SLP */
        Cy_USBD_DMADesc_SetNextDescriptor(dmaDscr, (slpLength)? dmaDscr1d_1 : NULL);
        Cy_USBD_DMADesc_SetChannelState  (dmaDscr, (slpLength)? CY_USBD_DMA_CHN_ENABLED : CY_USBD_DMA_CHN_DISABLED);
        remainingBytes = bytesToXfer % 64;
    } else {
        /* DMA will start with descriptor 1 in this case (no 2D xfers needed) */
        dmaDscr = dmaDscr1d_1;
    }

    if(remainingBytes < 4) {
        enByteTransfers  = true;
    }
    /* 64x transfers done. Now use 1D WORD size transfers till possible */
    if (remainingBytes) 
    {
        dscr_config.retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
        dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.dataPrefetch = false;
        dscr_config.dataSize = enByteTransfers? CY_USBD_DMA_BYTE : CY_USBD_DMA_WORD;
        dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
        if(direction == CY_USB_ENDP_DIR_OUT)
        {
            dscr_config.srcAddress = (uint8_t *)srcAddrBase;
            dscr_config.dstAddress = (uint8_t *)(dstAddrBase + (bytesToXfer & 0xFFC0UL)); //Skip over the completed 64x transfers
        }
        else
        {
            dscr_config.srcAddress = (uint8_t *)(srcAddrBase + (bytesToXfer & 0xFFC0UL));
            dscr_config.dstAddress = (uint8_t *)dstAddrBase;
        }

        dscr_config.srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
        dscr_config.dstXincrement = 1; /* Destination is RAM address */
        dscr_config.xCount = enByteTransfers ? remainingBytes : (remainingBytes >> 2U);
        dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
        dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
        dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
        
        dscr_config.nextDescriptor = NULL;
        dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
       
        if(!enByteTransfers) {
            remainingBytes = remainingBytes % 4;
            if (remainingBytes) {
                dscr_config.nextDescriptor = dmaDscr1d_2;
                dscr_config.channelState = CY_USBD_DMA_CHN_ENABLED;
                needDscr2 = true;
            }
        }
        Cy_USBD_DMADesc_Init(dmaDscr1d_1, &dscr_config);
    }

    /*64x and 4x transfers completed. Configure DMA for the remaining byte level transfers (if any) */
    if(remainingBytes)
    {
        dscr_config.retrigger = (needDscr2)? CY_USBD_DMA_RETRIG_IM:CY_USBD_DMA_WAIT_FOR_REACT;
        dscr_config.interruptType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.triggerInType = CY_USBD_DMA_DESCR_CHAIN;
        dscr_config.dataPrefetch = false;
        dscr_config.dataSize = CY_USBD_DMA_BYTE;
        dscr_config.srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
        dscr_config.descriptorType = CY_USBD_DMA_1D_XFER;
        if(direction == CY_USB_ENDP_DIR_OUT)
        {
            dscr_config.srcAddress = (uint8_t *)srcAddrBase + ((needDscr2) ?(bytesToXfer & 0x3CUL): 0);
            dscr_config.dstAddress = (uint8_t *)(dstAddrBase+ ((needDscr2) ?(bytesToXfer & 0xFFFCUL) : (bytesToXfer & 0xFFC0UL)));
        }
        else
        {
            dscr_config.srcAddress = (uint8_t *)(srcAddrBase + ((needDscr2) ?(bytesToXfer & 0xFFFCUL) : (bytesToXfer & 0xFFC0UL)));
            dscr_config.dstAddress = (uint8_t *)(dstAddrBase + ((needDscr2) ?(bytesToXfer & 0x3CUL): 0));
        }
        dscr_config.srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
        dscr_config.dstXincrement = 1; /* Destination is RAM address */
        dscr_config.xCount = remainingBytes;
        dscr_config.srcYincrement = 0; /* Not required for 1D transfer */
        dscr_config.dstYincrement = 0; /* Not required for 1D tranfer */
        dscr_config.yCount = 0;        /* For 1D transfer we dont need this */
        dscr_config.nextDescriptor = NULL;
        dscr_config.channelState = CY_USBD_DMA_CHN_DISABLED;
        Cy_USBD_DMADesc_Init((needDscr2) ? dmaDscr1d_2 : dmaDscr1d_1, &dscr_config);
    }

    Cy_USBD_DMAChn_SetDesc(dmaBase, dmaChannel, dmaDscr);
    Cy_USBD_DMAChn_Enable(dmaBase,dmaChannel);

}

/*
 * Function: Cy_USB_USBD_SendEndp0DataHs()
 * Description: HS device sends data through DMA for endpoint 0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pBuffer, bufferSize.
 * return: cy_en_usbd_ret_code_t.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_SendEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retval = CY_USBD_STATUS_SUCCESS;
    uint32_t dmaIntr;

#if BL_DEBUG
    DBG_USBD_INFO("Cy_USB_USBD_SendEndp0DataHs\r\n");
#endif

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("SndEnd0DataHs:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
#if BL_DEBUG
        DBG_USBD_ERR("SndEnd0DataHs:bufferOrSizeNull\r\n");
#endif
        return (CY_USBD_STATUS_BAD_PARAM);
    }
    Cy_USBD_UpdateXferCount(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, bufferSize);
    
    Cy_USB_USBD_CfgEndp0Dma(pUsbdCtxt, CY_USB_ENDP_DIR_IN, pBuffer,bufferSize);

    if (pUsbdCtxt->ep0SendDone == false) {
        /* Force the trigger to DMA channel input trigger to 1. */
        Cy_TrigMux_SwTrigger(CY_USBD_EGREP_OUT_TRIG,
                             CY_TRIGGER_TWO_CYCLES);
        pUsbdCtxt->ep0SendDone = true;
    }

    /* Clear busy bit to get the data moving. */
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

    /* Make sure USB link is in L0 until the transfer is completed. */
    Cy_SysLib_DelayUs(10);
    Cy_USBHS_Cal_GetLinkActive(pUsbdCtxt->pCalCtxt);

    dmaIntr = Cy_USB_USBD_WaitForXferComplete(pUsbdCtxt, 
            Cy_USBD_EP0In_DmaBase(pUsbdCtxt), 
            pUsbdCtxt->channel0, CY_USB_ENDP_DIR_IN);

    if (dmaIntr != 0) {
        /* Clear the DMA channel interrupt status. */
        Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0, dmaIntr);
#if BL_DEBUG
        DBG_USBD_TRACE("%x DONE\r\n", dmaIntr);
#endif /* BL_DEBUG */
    } else {
#if BL_DEBUG
        DBG_USBD_WARN("Ep0Send TIMEOUT\r\n");
#endif

        /* Disable the DMA channel. */
        Cy_USBD_DMAChn_Disable(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0);

        /*
         * Flush any data in the EPM buffer. Trigger will need to be set for
         * next DMA transfer.
         */
        Cy_USBD_FlushEndp(pUsbdCtxt, 0, CY_USB_ENDP_DIR_IN);
        pUsbdCtxt->ep0SendDone = false;

        retval = CY_USBD_STATUS_TIMEOUT;
    }

    return  retval;
}   /* End of function */






/*
 * Function: Cy_USB_USBD_RecvEndp0DataHs()
 * Description: HS device recieves data through DMA for endpoint 0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pBuffer, bufferSize.
 * return: cy_en_usbd_ret_code_t.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_RecvEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             uint8_t *pBuffer, uint32_t bufferSize)
{
    cy_en_usbd_ret_code_t retval = CY_USBD_STATUS_SUCCESS;
    uint32_t dmaIntrStatus;
        
    if (NULL == pUsbdCtxt) {
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if ((pBuffer == NULL) || (bufferSize == 0)) {
        return (CY_USBD_STATUS_BAD_PARAM);
    }

    /* Allow control transfer handshake. */
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    
    Cy_USB_USBD_CfgEndp0Dma(pUsbdCtxt, CY_USB_ENDP_DIR_OUT, pBuffer,bufferSize);

    if (bufferSize & 0x3F) {
        /*
         * Enable the SLP_RCVD interrupt for EP0-OUT if a short packet is expected.
         * The interrupt will be cleared and disabled in the ISR so that the short packet DMA is
         * triggered immediately.
         */
        Cy_USBHS_Cal_EnableCtrlSlpIntr(pUsbdCtxt->pCalCtxt);
    }

    /* Make sure USB link is in L0 until the transfer is completed. */
    Cy_USBHS_Cal_GetLinkActive(pUsbdCtxt->pCalCtxt);
    dmaIntrStatus = Cy_USB_USBD_WaitForXferComplete(pUsbdCtxt, Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1,
                CY_USB_ENDP_DIR_OUT);
    if(dmaIntrStatus)
    {
        /* Clear the DMA channel interrupt status. */
        Cy_USBD_DMAChn_ClearIntr(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, dmaIntrStatus);
    }
    else
    {
        /* Disable the DMA channel. */
        Cy_USBD_DMAChn_Disable(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1);
        retval = CY_USBD_STATUS_TIMEOUT;
    }
    return retval;
}   /* End of function */

/*
 * Function: Cy_USBD_HandleGetDscr()
 * Description: This function handles all get descriptor command coming
 *              from host.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t, pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetDscr (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_setup_req_t setupReq,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    bool isReqHandledByApp = false;
    uint16_t wValue, wLength, wTotalLength;
    uint8_t *pBuffer = NULL;
    uint32_t bufferSize = 0x00;
    uint8_t dscrType, dscrIndex;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlGetDsc:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /*
     * if descriptor is available then descriptor will be sent otherwise
     * STALL will be sent.
     */
    wValue = setupReq.wValue;
    wLength = setupReq.wLength;

    dscrType = ((wValue & CY_USB_CTRL_REQ_DSCR_TYPE_MASK)
                                      >> CY_USB_CTRL_REQ_DSCR_TYPE_POS);
    dscrIndex = ((wValue & CY_USB_CTRL_REQ_DSCR_INDEX_MASK)
                                      >> CY_USB_CTRL_REQ_DSCR_INDEX_POS);
    switch (dscrType) {

        case  CY_USB_DEVICE_DSCR:
#if BL_DEBUG
            DBG_USBD_INFO("DEVDSCR\r\n");
#endif
                pBuffer = pUsbdCtxt->dscrs.pUsbDevDscr;
            if (pBuffer) {
                bufferSize =  CY_USB_MIN(wLength, CY_USB_DEVICE_DSCR_LEN);
            }
            break;

        case CY_USB_CONFIG_DSCR:
#if BL_DEBUG
            DBG_USBD_INFO("CFGDSCR\r\n");
#endif
            if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
                /* device implementing maximum two descriptors. */
                pBuffer = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
            } else if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
            }

            if (pBuffer) {
                wTotalLength =  CY_USB_BUILD_2B_WORD(pBuffer[3],pBuffer[2]);
                bufferSize = CY_USB_MIN(wLength, wTotalLength);
            }
            break;

        case CY_USB_STRING_DSCR:
#if BL_DEBUG
            DBG_USBD_INFO("STRDSCR\r\n");
#endif
           /*
            * If string index is greater than CY_USBD_MAX_STR_DSCR_INDX OR if
            * the string descriptor is not registered, then pass the request to
            * application
            */
            if (pUsbdCtxt->strDscrAvailable == TRUE) {
                if ((dscrIndex > CY_USBD_MAX_STR_DSCR_INDX) ||
                    (pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex] == NULL)) {
                    isReqHandledByApp = true;
                } else {
                    pBuffer = pUsbdCtxt->dscrs.pUsbStringDescr[dscrIndex];
                    bufferSize = CY_USB_MIN(wLength, pBuffer[0]);
                }
            } else {
                /* if string dscr not registered (index 0 does not matter) */
                pBuffer = NULL;
            }
            break;

        case CY_USB_DEVICE_QUAL_DSCR:
#if BL_DEBUG
            DBG_USBD_INFO("QUALDSCR\r\n");
#endif
            if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_HS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbDevQualDscr;
                bufferSize = CY_USB_DEVICE_QUAL_DSCR_LEN;
            }

            if (pBuffer) {
                bufferSize = CY_USB_MIN(wLength, CY_USB_DEVICE_QUAL_DSCR_LEN);
            }
            break;

        case CY_USB_OTHERSPEED_DSCR:
#if BL_DEBUG
            DBG_USBD_INFO("OTHERsPEEDdscr\r\n");
#endif
           /*
            * if device is operating at full speed then other speed config
            * means high speed config. if device operating at high speed
            * then other speed means full speed config descriptor.
            */
            if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
                pBuffer = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
            } else {
                pBuffer = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
            }

            if (pBuffer) {
                wTotalLength =  CY_USB_BUILD_2B_WORD(pBuffer[3],pBuffer[2]);
                bufferSize = CY_USB_MIN(wLength, wTotalLength);
                /*
                 * Copy the descriptor into local buffer and change
                 * descriptor type.
                */
                memcpy((uint8_t *)pUsbdCtxt->otherSpeedCfgDscrBuf, pBuffer,
                       bufferSize);
                pUsbdCtxt->otherSpeedCfgDscrBuf[1U] = 0x07U;
                pBuffer = (uint8_t *)(pUsbdCtxt->otherSpeedCfgDscrBuf);
            }
            break;

        default:
            /* Control should not reach here. */
#if BL_DEBUG
            DBG_USBD_INFO("UNKNOWN\r\n");
#endif
            isReqHandledByApp = true;
            break;
    }   /* end of switch */

    if (isReqHandledByApp) {
        if (pUsbdCtxt->setupCb) {
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            Cy_USBHS_Cal_EndpSetClearStall(pUsbdCtxt->pCalCtxt, 0x00,
                                            CY_USB_ENDP_DIR_IN, TRUE);
        }
        return CY_USBD_STATUS_SUCCESS;
    }

    if (pBuffer) {
        Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, pBuffer, bufferSize);

            /*
             * If (wLength > bufferSize) and bufferSize is a multiple of max.
             * pkt. size; we need to send a ZLP to complete the transfer.
             */
            if ((wLength > bufferSize) && ((bufferSize & 0x3FUL) == 0)) {
                Cy_USBHS_Cal_SendEgressZLP(pUsbdCtxt->pCalCtxt, 0x00);
            }
    } else {
#if BL_DEBUG
        DBG_USBD_INFO("GetDscrSndSTALL\r\n");
#endif
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                                     CY_USB_ENDP_DIR_IN, TRUE);
    }
    return(retStatus);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleGetStatus()
 * Description: Get status request is handled by the function.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetStatus (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    cy_stc_usb_setup_req_t setupReq)
{
    uint8_t recipient;
    uint8_t *pBuffer = NULL;
    uint16_t bufferSize;
    uint16_t endpNum = 0x00;
    uint32_t getStatusData = 0x00;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlGetSta:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }

#if BL_DEBUG
    DBG_USBD_INFO("GETSTAT\r\n");
#endif

    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);

    switch (recipient) {

        case CY_USB_CTRL_REQ_RECIPENT_DEVICE:
            getStatusData = 0x00;
            if (pUsbdCtxt->selfPowered) {
                getStatusData |= (uint16_t)(CY_USB_GET_STATUS_DEV_SELF_POWER);
            }
            /*
             * If device support remote wakeup then inform to host.
             * host will decide to enable or not.
             */
            if (pUsbdCtxt->remoteWakeupEnable) {
                getStatusData |= (uint16_t)CY_USB_GET_STATUS_DEV_REMOTE_WAKEUP;
            }
            pBuffer = (uint8_t *)&getStatusData;
            break;

        case CY_USB_CTRL_REQ_RECIPENT_INTF:
            getStatusData = 0x00;
            pBuffer = (uint8_t *)&getStatusData;
            break;


        case CY_USB_CTRL_REQ_RECIPENT_ENDP:
            endpNum = setupReq.wIndex & 0x7FU;
            if (((setupReq.wIndex) & 0x80U) != 0) {
                if (pUsbdCtxt->endpInfoIn[endpNum].halt) {
                    getStatusData = (uint16_t)CY_USB_GET_STATUS_ENDP_HALT;
                } else {
                    getStatusData = 0x00;
                }
            } else {
                if (pUsbdCtxt->endpInfoOut[endpNum].halt) {
                    getStatusData = (uint16_t)CY_USB_GET_STATUS_ENDP_HALT;
                } else {
                    getStatusData = 0x00;
                }
            }
            pBuffer = (uint8_t *)&getStatusData;
            break;

        default:
            pBuffer = NULL;
            break;
    }

    if (pBuffer) {
        /* Get status always returns two byte of data. */
        bufferSize = 0x02;
        Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, pBuffer, bufferSize);
    } else {
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                                     CY_USB_ENDP_DIR_IN, TRUE);
    }   /* end of else pBuffer */
    return(retStatus);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleSetFeature()
 * Description: SetFeature request is handled by the function.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 * set feature does not have data stage.
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetFeature (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_setup_req_t setupReq,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    uint8_t recipient;
    uint16_t endpNum;
    cy_en_usb_endp_dir_t dir;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;
    bool handled = false;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlSetFea:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    /*
     * In case of set feature, direction will be available
     * with endpoint number.
     */
    if ((setupReq.wIndex) & (uint16_t)0x80U) {
        dir = CY_USB_ENDP_DIR_IN;
    } else {
        dir = CY_USB_ENDP_DIR_OUT;
    }

    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);
    endpNum = setupReq.wIndex & 0x7FU;

    if ((CY_USB_CTRL_REQ_RECIPENT_ENDP == recipient) &&
        (CY_USB_FEATURE_ENDP_HALT == setupReq.wValue) &&
        (endpNum < CY_USB_MAX_ENDP_NUMBER)) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
#if BL_DEBUG
            DBG_USBD_INFO("setFeatureSetupClbEndp\r\n");
#endif
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                                                                         TRUE);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_INTF == recipient) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
#if BL_DEBUG
            DBG_USBD_INFO("setFeatureSetupClbIntf\r\n");
#endif
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                    TRUE);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (!handled) {
        retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
    return (retStatus);
}   /* end of function  */

/*
 * Function: Cy_USBD_HandleClearFeature()
 * Description: ClearFeature request is handled by the function.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 * Clear feature does not have data stage.
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleClearFeature (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq,
                            cy_stc_usb_cal_msg_t *pMsg)
{

    uint8_t recipient;
    uint16_t endpNum;
    cy_en_usb_endp_dir_t dir;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;
    bool handled = false;

    if ((setupReq.wIndex) & (uint16_t)0x80U) {
        dir = CY_USB_ENDP_DIR_IN;
    } else {
        dir = CY_USB_ENDP_DIR_OUT;
    }

    recipient = ((setupReq.bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK) >>
                 CY_USB_CTRL_REQ_RECIPENT_POS);
    endpNum = setupReq.wIndex & 0x7FU;

    if ((CY_USB_CTRL_REQ_RECIPENT_ENDP == recipient) &&
        (CY_USB_FEATURE_ENDP_HALT == setupReq.wValue) &&
        (endpNum < CY_USB_MAX_ENDP_NUMBER)) {

        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
#if BL_DEBUG
            DBG_USBD_INFO("clearFeatureSetupClbEndp\r\n");
#endif
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /*
             * Call CAL function to clear stall for given endpoint
             * and send ACK during status stage.
             */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum,
                                                  dir, FALSE);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (CY_USB_CTRL_REQ_RECIPENT_INTF == recipient) {
        /* Trigger callback */
        if (pUsbdCtxt->setupCb) {
#if BL_DEBUG
            DBG_USBD_INFO("setFeatureSetupClbIntf\r\n");
#endif
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            /* if callback is not register then USBD handles the same. */
            retStatus = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, endpNum, dir,
                    TRUE);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        }
        handled = true;
    }

    if (!handled) {
        /* Stall EP0 if the request has not been handled. */
        retStatus= Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }

    return(retStatus);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleSetConfiguration()
 * Description: This function handles set configuration command.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t,
 *            cy_stc_usb_cal_msg_t
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetConfiguration (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_setup_req_t setupReq,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    uint8_t *pCfgDscr;
    uint8_t configNum;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    /*
     * Check config value proper or not.
     * Make required configuration as active configuration.
     * Configure endpoints.
     * Call application call back related to set_config.
     */

#if BL_DEBUG
    DBG_USBD_INFO("HNDLSETCFG\r\n");
#endif
    configNum = (uint8_t)((setupReq.wValue) & (0x00FF));

    if (pUsbdCtxt->devSpeed == CY_USBD_USB_DEV_FS) {
        pCfgDscr = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
    } else {
            pCfgDscr = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
    }

    if (0x00 == configNum) {
        /*
         * If host sends setCfg with config 0 then device should move to
         * Address state and send ACK to host.
         */
        pUsbdCtxt->pActiveCfgDscr = NULL;
        pUsbdCtxt->activeCfgNum = 0x00;
        pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
        pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ADDRESS;
        pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
        pUsbdCtxt->pActiveCfgDscr = NULL;
        pUsbdCtxt->activeCfgNum = 0x00;
        pUsbdCtxt->selfPowered = 0x00;
        pUsbdCtxt->remoteWakeupAbility = 0x00;
        pUsbdCtxt->remoteWakeupEnable = 0x00;
        pUsbdCtxt->EnumerationDone = false;

        /* Trigger callback */
        if (pUsbdCtxt->setConfigCb) {
#if BL_DEBUG
           DBG_USBD_INFO("SetCfg0Clb\r\n");
#endif
           pUsbdCtxt->setConfigCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        return(retStatus);
    }

    pUsbdCtxt->pActiveCfgDscr = pCfgDscr;
    pUsbdCtxt->activeCfgNum = configNum;
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbdCtxt->remoteWakeupAbility = 0x00;
    pUsbdCtxt->selfPowered = 0x00;
    pUsbdCtxt->EnumerationDone = true;

    /* Trigger callback */
    if (pUsbdCtxt->setConfigCb) {
#if BL_DEBUG
       DBG_USBD_INFO("SETCFGclbNt\r\n");
#endif
       pUsbdCtxt->setConfigCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    /* No data stage so enable HW to send ACK. */
    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    return(retStatus);
}   /* End of function  */


/*
 * Function: Cy_USBD_HandleGetConfiguration()
 * Description: This function returns configuration value.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetConfiguration (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    cy_stc_usb_setup_req_t setupReq)
{
    uint8_t *pBuffer;
    uint16_t bufferSize = 0x00;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    pBuffer = &(pUsbdCtxt->activeCfgNum);
    bufferSize =  0x01;
    Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, pBuffer, bufferSize);
    return(retStatus);
}   /* end of function. */


/*
 * Function: Cy_USBD_HandleSetInterface()
 * Description: This function set alternate setting send by host.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 * Set Interface does not have data stage.
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSetInterface (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;
    uint8_t *pCfgDscr = pUsbdCtxt->pActiveCfgDscr;
    uint32_t intfNum, altSetting;

    intfNum = setupReq.wIndex;
    altSetting = setupReq.wValue;

    if ((pCfgDscr == NULL) ||
         (intfNum >= (*(pCfgDscr + CY_USB_CFG_DSCR_OFFSET_NUM_INTF)))) {
        /* Corruption in memory so just send STALL. */
        retCode = Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                                    CY_USB_ENDP_DIR_IN, TRUE);
    } else {
        /* things are fine so update info, trigger callback, initiate ACK. */
        pUsbdCtxt->altSettings[intfNum] = altSetting;
        if (pUsbdCtxt->setIntfCb) {
            pUsbdCtxt->setIntfCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
        /* No data stage so enable HW to send ACK. */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
    }
    return(retCode);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleGetInterface()
 * Description: This function returns alternate setting.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_setup_req_t
 * return: cy_en_usbd_ret_code_t
 * TBD: any discussion require wrt dataStructure for altSetting?
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleGetInterface (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_setup_req_t setupReq)
{

    uint8_t *pBuffer;
    uint32_t bufferSize = 0x00;
    uint16_t intf;
    uint8_t altSetting;
    cy_en_usb_cal_ret_code_t calRetStatus = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    /*
     * altSetting is always updated in deviceInfo so it can be used as it is.
     * Only Interface number should be checked.
     * Code can be written without pBuffer also.
     */
    intf = setupReq.wIndex;
    altSetting = pUsbdCtxt->altSettings[intf];
    pBuffer = &altSetting;
    bufferSize =  0x01;
    calRetStatus = Cy_USBHS_Cal_UpdateXferCount(pUsbdCtxt->pCalCtxt, 0x00,
                                                CY_USB_ENDP_DIR_IN, bufferSize);
    Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, pBuffer, bufferSize);

    if (CY_USB_CAL_STATUS_FAILURE == calRetStatus) {
        retStatus = CY_USBD_STATUS_FAILURE;
    }
    return(retStatus);
}   /* End of function */


/*
 * Function: Cy_USBD_HandleReset()
 * Description: This function takes care of all RESET related activity.
 * Parameter: pMsg
 * return: cy_en_usbd_ret_code_t, cy_stc_usb_cal_msg_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleReset (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    cy_stc_usb_cal_msg_t *pMsg)
{
    uint32_t endp0MaxPktSize;
    cy_stc_usb_endp_config_t endpConfig;
    uint32_t intfNum;
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t retCode = CY_USBD_STATUS_SUCCESS;

    /*
     * This section assumes Fast enumeation enabled.
     * Some of the data structure needs to be reinitialized.
     * Configure like device working in FS mode.
     * Configure H/W register related to endpoint and take endp size from
     * available descriptor.
     */

    /* Initialize usb device info structure. */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;

    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;

    /* During reset make sure that all altesetting should be 0 */
    for (intfNum = 0x00; intfNum < CY_USB_MAX_INTF; intfNum++) {
        pUsbdCtxt->altSettings[intfNum] = 0x00;
    }
    /*
     * Configure endpoint 0x00 and make it valid.
     * As off now ignoring retCode because endp0MaxPktSize will be 0
     * in case of error condition.
     */
    retCode = Cy_USBD_FindEndp0MaxPktSize(pUsbdCtxt->dscrs.pUsbDevDscr,
                                          pUsbdCtxt->devSpeed,
                                          &endp0MaxPktSize);
    endpConfig.valid = 0x01;
    endpConfig.endpNumber = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_CTRL;
    endpConfig.maxPktSize = endp0MaxPktSize;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;
    endpConfig.allowNakTillDmaRdy = false;      /* Allow NAK function should not be enabled for EP0. */

    pUsbdCtxt->endpInfoIn[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoIn[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoIn[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;


    pUsbdCtxt->endpInfoOut[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoOut[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoOut[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].allowNakTillDmaRdy = endpConfig.allowNakTillDmaRdy;

    calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    /* Connect trigger from EP0 egress EPM to DMA channel. */
    Cy_TrigMux_Connect(CY_USBD_EGREP_OUT_TRIG,
                        CY_USBD_EGREP_DMA_TRIG_BASE + pUsbdCtxt->channel0,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from DMA channel to EP0 egress EPM. */
    Cy_TrigMux_Connect(CY_USBD_EGREP_DMAOUT_TRIG_BASE + pUsbdCtxt->channel0,
                        CY_USBD_EGREP_IN_TRIG,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from EP0 ingress EPM to DMA channel. */
    Cy_TrigMux_Connect(CY_USBD_INGEP_OUT_TRIG,
                        CY_USBD_INGEP_DMA_TRIG_BASE + pUsbdCtxt->channel1,
                        false, TRIGGER_TYPE_LEVEL);

    /* Connect trigger from DMA channel to EP0 ingress EPM. */
    Cy_TrigMux_Connect(CY_USBD_INGEP_DMAOUT_TRIG_BASE + pUsbdCtxt->channel1,
                       CY_USBD_INGEP_IN_TRIG,
                       false, TRIGGER_TYPE_EDGE);

    /* Clear the flag indicating with any EP0 data send has been done. */
    pUsbdCtxt->ep0SendDone = false;

    Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt, CY_USB_ENDP_DIR_IN);
    Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt, CY_USB_ENDP_DIR_OUT);
    Cy_USBHS_Cal_HandleReset (pUsbdCtxt->pCalCtxt);

    if (pUsbdCtxt->busResetCb) {
        /* Inform about Speed */
        pUsbdCtxt->busResetCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (retCode);
}   /* end of function  */

/*
 * Function: Cy_USBD_HandleHsGrant()
 * Description: High speed related switching is handled here.
 * Parameter: pMsg
 * return: cy_en_usbd_ret_code_t, cy_stc_usb_cal_msg_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleHsGrant (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlHsGra:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    /*
     * high speed config descriptor will be more relavent.
     * FS descriptor will become other descriptor.
     * speed become HighSpeed.
     */
    pUsbdCtxt->dscrs.pUsbCfgDscr = pUsbdCtxt->dscrs.pUsbHsCfgDscr;
    pUsbdCtxt->dscrs.pUsbOtherSpeedCfgDscr = pUsbdCtxt->dscrs.pUsbFsCfgDscr;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_HS;
    if (pUsbdCtxt->busSpeedCb) {
        /* Inform about Speed */
        pUsbdCtxt->busSpeedCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* End of function  */


/*
 * Function: Cy_USBD_HandleResetDone()
 * Description: Reset done signaling message is handled here.
 * Parameter: pMsg
 * return: cy_en_usbd_ret_code_t, cy_stc_usb_usbd_ctxt_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleResetDone (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        return (CY_USBD_STATUS_CTXT_NULL);
    }
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DEFAULT;
    if (pUsbdCtxt->busResetDoneCb) {
        /* Inform about Speed */
        pUsbdCtxt->busResetDoneCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleCtrlXfrSetupStage()
 * Description: This function will handle setup stage of control transfer and
 *               prepare for data/ACK stage based on request.
 * Parameter: cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: cy_en_usbd_ret_code_t, cy_stc_usb_cal_msg_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleCtrlXfrSetupStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                 cy_stc_usb_cal_msg_t *pMsg)
{
    uint32_t  setupData0;
    uint32_t  setupData1;
    uint8_t   bmRequest, bRequest;
    uint8_t   reqType;

    cy_stc_usb_setup_req_t setupReq;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_FAILURE;

    setupData0 = pMsg->data[0];
    setupData1 = pMsg->data[1];

    /* Make sure EP0 stall condition is cleared before handling the request. */
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, false);

    /* Decode the fields from the setup request. */
    pUsbdCtxt->setupReq.bmRequest = bmRequest = setupReq.bmRequest =
            (uint8_t)((setupData0 & CY_USB_BMREQUEST_SETUP0_MASK) >>
                       CY_USB_BMREQUEST_SETUP0_POS);
    pUsbdCtxt->setupReq.bRequest = bRequest = setupReq.bRequest =
            (uint8_t)((setupData0 & CY_USB_BREQUEST_SETUP0_MASK) >>
                        CY_USB_BREQUEST_SETUP0_POS);

    pUsbdCtxt->setupReq.wValue = setupReq.wValue =
            (uint16_t)((setupData0 & CY_USB_WVALUE_SETUP0_MASK) >>
                        CY_USB_WVALUE_SETUP0_POS);

    pUsbdCtxt->setupReq.wIndex = setupReq.wIndex =
            (uint16_t)((setupData1 & CY_USB_WINDEX_SETUP1_MASK) >>
                        CY_USB_WINDEX_SETUP1_POS);
    pUsbdCtxt->setupReq.wLength = setupReq.wLength =
            (uint16_t)((setupData1 & CY_USB_WLENGTH_SETUP1_MASK) >>
                        CY_USB_WLENGTH_SETUP1_POS);

    /* reqType can be STD, CLASS, VENDOR or OTHERS */
    reqType = ((bmRequest & CY_USB_CTRL_REQ_TYPE_MASK) >>
                                                CY_USB_CTRL_REQ_TYPE_POS);

    if (CY_USB_CTRL_REQ_STD == reqType) {
        switch (bRequest) {

            case CY_USB_SC_GET_STATUS:
                retStatus = Cy_USBD_HandleGetStatus(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_CLEAR_FEATURE:
                retStatus = Cy_USBD_HandleClearFeature(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SET_FEATURE:
                retStatus = Cy_USBD_HandleSetFeature(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SET_ADDRESS:
#if BL_DEBUG
                DBG_USBD_INFO("CY_USB_SC_SET_ADDRESS\r\n");
#endif
                /*
                 * SET ADDRESS command handle by hw so control will not
                 * come here.
                 */
                break;

            case CY_USB_SC_GET_DESCRIPTOR:
                /* Due to bmRequest and bRequest control reached here */
                retStatus = Cy_USBD_HandleGetDscr(pUsbdCtxt, setupReq,pMsg);
                break;

            case CY_USB_SC_SET_DESCRIPTOR:
                break;

            case CY_USB_SC_GET_CONFIGURATION:
#if BL_DEBUG
                DBG_USBD_INFO("CY_USB_SC_GET_CONFIGURATION\r\n");
#endif
                retStatus = Cy_USBD_HandleGetConfiguration(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_SET_CONFIGURATION:
                /*
                 * TBD: Following things will be done here.
                 * state change to Configured, All endpoints will be
                 * initialized and enabdled. callback will be called
                 * and information will be passed to application.
                 */
                retStatus = Cy_USBD_HandleSetConfiguration(pUsbdCtxt,setupReq,
                                                           pMsg);
                break;

            case CY_USB_SC_GET_INTERFACE:
                retStatus = Cy_USBD_HandleGetInterface(pUsbdCtxt,setupReq);
                break;

            case CY_USB_SC_SET_INTERFACE:
                retStatus = Cy_USBD_HandleSetInterface(pUsbdCtxt,setupReq,pMsg);
                break;

            case CY_USB_SC_SYNC_FRAME:
                /* Just send acknowledge */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                break;
        }   /* endof switch (bRequest) */
    } else {
        /*
         * For non standard request, if setup callback is not registered
         * then send STALL.
         */
        if (pUsbdCtxt->setupCb) {
            pUsbdCtxt->setupReq = setupReq;
            pUsbdCtxt->setupCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        } else {
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                           CY_USB_ENDP_DIR_IN, TRUE);
        }
    }   /* end of ELSE for standatd request */
    return(retStatus);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleStatusStage()
 * Description: Status stage completion will be done here.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleStatusStage (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlStaStage:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;

    if (pUsbdCtxt->statusStageComplCb) {
        pUsbdCtxt->statusStageComplCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*
 * Function: Cy_USBD_HandleSuspend()
 * Description: Suspend message hsndled here.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSuspend (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlSuspend:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devState <= CY_USB_DEVICE_STATE_RESET) {
        /* Before reset comes, dont do anything. */
#if BL_DEBUG
        DBG_USBD_INFO("Suspend came when DevState < RESET\r\n");
#endif
        return(CY_USBD_STATUS_SUCCESS);
    }

    /* Now handle Suspend */
    pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_SUSPEND;
    /* Nothing is required here. */

    if (pUsbdCtxt->suspendCb) {
        pUsbdCtxt->suspendCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}


/*
 * Function: Cy_USBD_HandleResume()
 * Description: Resume message handled here.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleResume (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                      cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlResume:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }
    pUsbdCtxt->devState = pUsbdCtxt->prevDevState;
    pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_SUSPEND;

    if (pUsbdCtxt->resumeCb) {
        pUsbdCtxt->resumeCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */


/*
 * Function: Cy_USBD_HandleZlp()
 * Description: Handle ZLP for an endpoint.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleZlp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlZlp:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }
    if (pUsbdCtxt->zlpCb) {
        pUsbdCtxt->zlpCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }

    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*
 * Function: Cy_USBD_HandleDone()
 * Description: Handle DONE interrupt from an endpoint.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleDone (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlDone:usbdCtxtNull\r\n");
#endif
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->doneCb) {
        pUsbdCtxt->doneCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);

        /* Re-enable the DONE interrupt for this endpoint. */
        Cy_USBHS_Cal_UpdateEpIntrMask(pUsbdCtxt->pCalCtxt,
                                      pMsg->data[0], CY_USB_ENDP_DIR_OUT,
                                      USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk, true);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function */

/*
 * Function: Cy_USBD_HandleSlp()
 * Description: Handle SLP for an endpoint.
 * Parameter: usbd ctxt and pMsg
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSlp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    if (NULL == pUsbdCtxt) {
        return(CY_USBD_STATUS_CTXT_NULL);
    }

    /* Handle SLP interrupt on EP0-OUT here itself. For other endpoints, pass on to the callback. */
    if ((pMsg->type == CY_USB_CAL_MSG_OUT_SLP) && (pMsg->data[0] == 0x00)) {
        /*
         * RecvEndp0Data function would have unmasked SLP interrupt on EP0 only
         * after configuring the DMA channel correctly. Just assert the trigger
         * output from the USB block to ensure that the data gets read out by
         * the DMA channel.
         */
        Cy_TrigMux_SwTrigger(CY_USBD_INGEP_OUT_TRIG, CY_TRIGGER_TWO_CYCLES);
    } else {
        if (pUsbdCtxt->slpCb) {
            pUsbdCtxt->slpCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
        }
    }
    return (CY_USBD_STATUS_SUCCESS);
}


/*
 * Function: Cy_USBD_HandleSsDisconnect()
 * Description: This function handles SS-Disconnect comes as
 *              CY_USBSS_CAL_MSG_LNK_DISCONNECT message from cal layer.
 * Parameter: pUsbdCtxt, pMsg
 * return: cy_en_usbd_ret_code_t, cy_stc_usb_cal_msg_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleSsDisconnect (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{

    Cy_USBD_DisconnectDevice(pUsbdCtxt);

    if (pUsbdCtxt->DisconnectCb) {
        pUsbdCtxt->DisconnectCb(pUsbdCtxt->pAppCtxt, pUsbdCtxt, pMsg);
    }
    return (CY_USBD_STATUS_SUCCESS);
}   /* end of function  */


/*
 * Function: Cy_USBD_HandleMsg()
 * Description: This function handles message coming from CAL layer.
 * Parameter: message
 * return: None
 */
cy_en_usbd_ret_code_t
Cy_USBD_HandleMsg (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                   cy_stc_usb_cal_msg_t *pMsg)
{
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("HndlMsg:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    switch (pMsg->type) {

        case CY_USB_CAL_MSG_SOF:
            /* Nothing for SoF message. Just print if required. */
            break;

        case CY_USB_CAL_MSG_RESET:
#if BL_DEBUG
            DBG_USBD_INFO("CY_USB_CAL_MSG_RESET\r\n");
#endif
            retStatus = Cy_USBD_HandleReset(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_HSGRANT:
#if BL_DEBUG
            DBG_USBD_INFO("CY_USB_CAL_MSG_HSGRANT\r\n");
#endif
             Cy_USBD_HandleHsGrant(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_RESET_DONE:
#if BL_DEBUG
            DBG_USBD_INFO("CY_USB_CAL_MSG_RESET_DONE\r\n");
#endif
            Cy_USBD_HandleResetDone(pUsbdCtxt, pMsg);
            break;

            /* CASE fall through here is intentional. */
        case CY_USB_CAL_MSG_SUDAV:
            /*
             * Control request came. Handle as per requirement.
             * 1. For fast enumeration standrard request will be handled here.
             * 2. For application-enumeration, trigger callback register by
             *    application.
             * 3. All Non standard requst, trigger callback register by
             *    application.
             */
            retStatus = Cy_USBD_HandleCtrlXfrSetupStage(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_STATUS_STAGE:
            Cy_USBD_HandleStatusStage(pUsbdCtxt,pMsg);
            break;

        case CY_USB_CAL_MSG_SETADDR:
#if BL_DEBUG
            DBG_USBD_INFO("CY_USB_CAL_MSG_SETADDR\r\n");
#endif
            pUsbdCtxt->prevDevState = pUsbdCtxt->devState;
            pUsbdCtxt->devState = CY_USB_DEVICE_STATE_ADDRESS;
            Cy_USBHS_Cal_GetDevAddress(pUsbdCtxt->pCalCtxt,
                           &(pUsbdCtxt->devAddr));
            break;

        case CY_USB_CAL_MSG_PROT_SETADDR_0:
            /* device should come to default state */
#if BL_DEBUG
            DBG_USBD_INFO("Msg:SetAddr0\r\n");
#endif
            break;

        case CY_USB_CAL_MSG_SUSP:
            /*
             * ISR takes care of PHY and controller related settings. use
             * this message to inform upper layer about SUSPEND.
             */
            Cy_USBD_HandleSuspend(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_L1_SLEEP:
#if BL_DEBUG
            DBG_USBD_ERR("Msg:L1_SLEEPNotHandled.\r\n");
#endif
            break;

        case CY_USB_CAL_MSG_RESUME_START:
#if BL_DEBUG
            DBG_USBD_INFO("Msg:RESUME_START\r\n");
#endif
            Cy_USBD_HandleResume(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_RESUME_END:
#if BL_DEBUG
            DBG_USBD_INFO("Msg:RESUME_END\r\n");
#endif
            break;

        case CY_USB_CAL_MSG_L1_URESUME:
#if BL_DEBUG
            DBG_USBD_ERR("Msg:L1_URESUMENotHandled.\r\n");
#endif
            break;

        case CY_USB_CAL_MSG_IN_ZLP:
        case CY_USB_CAL_MSG_OUT_ZLP:
            Cy_USBD_HandleZlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_IN_SLP:
        case CY_USB_CAL_MSG_OUT_SLP:
            Cy_USBD_HandleSlp(pUsbdCtxt, pMsg);
            break;

        case CY_USB_CAL_MSG_ERRLIMIT:
#if BL_DEBUG
          DBG_USBD_TRACE("Msg:ErrorLimit\r\n");
#endif
            break;

        case CY_USBSS_CAL_MSG_EPM_UNDERRUN:
#if BL_DEBUG
            DBG_USBD_INFO("Msg:EpmUnderrun\r\n");
#endif
            break;

        case CY_USB_CAL_MSG_OUT_DONE:
            Cy_USBD_HandleDone(pUsbdCtxt, pMsg);
            break;

        default:
#if BL_DEBUG
            DBG_USBD_ERR("HndlMsg:default(%x)\r\n", pMsg->type);
#endif
            break;
    }
    return(retStatus);
}   /* end of function */


/*
 * Function: Cy_USBD_InitUsbDscrPtrs()
 * Description: This function initializes all descriptor pointers to NULL.
 * Parameter: pointer to descriptors structure.
 * return: None
 */
void
Cy_USBD_InitUsbDscrPtrs (cy_stc_usb_set_dscr_ptrs_t *pDscr)
{
    uint32_t dscrIndex = 0x00;
    pDscr->pUsbDevDscr = NULL;
    pDscr->pUsbDevQualDscr = NULL;
    pDscr->pUsbCfgDscr = NULL;
    pDscr->pUsbOtherSpeedCfgDscr = NULL;
    pDscr->pUsbFsCfgDscr = NULL;
    pDscr->pUsbHsCfgDscr = NULL;
    for (dscrIndex = 0x00; dscrIndex <CY_USBD_MAX_STR_DSCR_INDX; dscrIndex++) {
        pDscr->pUsbStringDescr[dscrIndex] = NULL;
    }
    return;
}


/*
 * Function: Cy_USB_USBD_ConnectDevice()
 * Description: Connect device to BUS and make it visible.
 * Parameter: cy_stc_usb_usbd_ctxt_t, usbSpeed
 * return: None
 */
void
Cy_USBD_ConnectDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_en_usb_speed_t usbSpeed)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("Connect:NullCtxt\r\n");
#endif
        return;
    }

        /* clear all dev control interrupt and enable required interrupt */
        Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
        Cy_USBHS_Cal_EnableReqDevCtrlIntr(pUsbdCtxt->pCalCtxt);
        Cy_USBHS_Cal_ConnUsbPins(pUsbdCtxt->pCalCtxt);

    return;
}   /* End of function */


/*
 * Function: Cy_USBD_ResetUsbdCommonDs()
 * Description: Reset USBD data structure which are not speed dependent.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None
 * NOTE: Any speed specific leanup should have beed done before calling
 *       this function.
 */
void
Cy_USBD_ResetUsbdCommonDs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    /*
     * This is not API. It is internal function where caller should have
     * already checked NULL ptr.
     * Common cleanup required at USBD layer during disconnect.
     */
    pUsbdCtxt->devAddr = 0x00;
    pUsbdCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pUsbdCtxt->devSpeed = CY_USBD_USB_DEV_NOT_CONNECTED;
    pUsbdCtxt->endp0State = CY_USB_ENDP0_STATE_IDLE;
    pUsbdCtxt->pActiveCfgDscr = NULL;
    pUsbdCtxt->activeCfgNum = 0x00;
    pUsbdCtxt->selfPowered = 0x00;
    pUsbdCtxt->remoteWakeupAbility = 0x00;
    pUsbdCtxt->remoteWakeupEnable = 0x00;
    pUsbdCtxt->EnumerationDone = false;
    pUsbdCtxt->ep0SendDone = false;
    return;
}   /* End of function Cy_USBD_ResetUsbdCommonDs() */


/*
 * Function: Cy_USBD_DisconnectDevice()
 * Description: Disconnect device from BUS. This function should be called
 *              by module/layer who detects vbus insertion/removal.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None
 */
void
Cy_USBD_DisconnectDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("DisConDev:usbdCtxtNull\r\n");
#endif
        return;
    }
        /* disable interrupts */
        Cy_USBHS_Cal_DisableAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
        Cy_USBHS_Cal_DisconUsbPins(pUsbdCtxt->pCalCtxt);
        /* Since device is disconnected so Flush all endpoint */
        Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt, CY_USB_ENDP_DIR_IN);
        Cy_USBHS_Cal_FlushAllEndp(pUsbdCtxt->pCalCtxt, CY_USB_ENDP_DIR_OUT);
    Cy_USBD_ResetUsbdCommonDs(pUsbdCtxt);
    return;
}   /* End of function. */


/*
 * Function: Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig()
 * Description: It initializes DMA descriptor for endpoint 0 IN transfer.
 * Parameter: cy_stc_usbd_dma_descr_conf_t.
 * return: None.
 */
void
Cy_USB_USBD_InitEndp0InCpuDmaDscrConfig (cy_stc_usbd_dma_descr_conf_t
					 *pEndp0InCpuDmaDscrConfig,
					 bool first)
{
  pEndp0InCpuDmaDscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
  pEndp0InCpuDmaDscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN;
  pEndp0InCpuDmaDscrConfig->dataPrefetch = false;
  pEndp0InCpuDmaDscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
  pEndp0InCpuDmaDscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
  pEndp0InCpuDmaDscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
  pEndp0InCpuDmaDscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
  pEndp0InCpuDmaDscrConfig->xCount = 1;     /* TBD: Need to update as size of descriptor */
  pEndp0InCpuDmaDscrConfig->nextDescriptor = NULL;
  pEndp0InCpuDmaDscrConfig->srcXincrement = 1;
  pEndp0InCpuDmaDscrConfig->dstXincrement = 1;
  pEndp0InCpuDmaDscrConfig->dstYincrement = 0x00; /* Not required for 1D tranfer. */

  if (first)
    {
      pEndp0InCpuDmaDscrConfig->triggerOutType = CY_USBD_DMA_X_LOOP;
      pEndp0InCpuDmaDscrConfig->triggerInType = CY_USBD_DMA_X_LOOP;
      pEndp0InCpuDmaDscrConfig->channelState = CY_USBD_DMA_CHN_ENABLED;
      pEndp0InCpuDmaDscrConfig->dataSize = CY_USBD_DMA_WORD;
      pEndp0InCpuDmaDscrConfig->descriptorType = CY_USBD_DMA_2D_XFER;
      pEndp0InCpuDmaDscrConfig->srcYincrement = 0x10; /* Increment by 16 for 64 bytes. */
      pEndp0InCpuDmaDscrConfig->yCount = 1;        /* TBD: To be updated based on transfer size. */
    }
  else
    {
      pEndp0InCpuDmaDscrConfig->triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
      pEndp0InCpuDmaDscrConfig->triggerInType = CY_USBD_DMA_DESCR_CHAIN;
      pEndp0InCpuDmaDscrConfig->channelState = CY_USBD_DMA_CHN_DISABLED;
      pEndp0InCpuDmaDscrConfig->dataSize = CY_USBD_DMA_BYTE;
      pEndp0InCpuDmaDscrConfig->descriptorType = CY_USBD_DMA_1D_XFER;
      pEndp0InCpuDmaDscrConfig->srcYincrement = 0; /* Not required for 1D transfer. */
      pEndp0InCpuDmaDscrConfig->yCount = 0;        /* For 1D trasnfer we dont need this. */
    }
}   /* End of function  */

/*
 * Function: Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig()
 * Description: It initializes DMA descriptor for EP0-OUT transfers.
 * Parameter: cy_stc_usbd_dma_descr_conf_t, bool
 * return: None.
 */
void
Cy_USB_USBD_InitEndp0OutCpuDmaDscrConfig (cy_stc_usbd_dma_descr_conf_t *pEndp0OutdscrConfig,
					  bool first)
{
  pEndp0OutdscrConfig->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;  /* Wait for input trigger after each packet. */
  pEndp0OutdscrConfig->interruptType = CY_USBD_DMA_DESCR_CHAIN; /* Assert interrupt at end of descriptor chain. */
  pEndp0OutdscrConfig->dataPrefetch = false;
  pEndp0OutdscrConfig->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
  pEndp0OutdscrConfig->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
  pEndp0OutdscrConfig->srcAddress = NULL;  /* TBD: Need to be dscr address in IN */
  pEndp0OutdscrConfig->dstAddress = NULL;  /* TBD: Need to be FIFO address in IN */
  pEndp0OutdscrConfig->srcXincrement = 1; /* Address increment is required when reading from EPM as well. */
  pEndp0OutdscrConfig->dstXincrement = 1; /* Destination is RAM address */
  pEndp0OutdscrConfig->nextDescriptor = NULL;
  pEndp0OutdscrConfig->srcYincrement = 0;    /* Go back to EP base address for each packet. */
  if (first) {
      pEndp0OutdscrConfig->triggerOutType = CY_USBD_DMA_X_LOOP;     /* Trigger to be generated after each packet. */
      pEndp0OutdscrConfig->triggerInType = CY_USBD_DMA_X_LOOP;      /* Input trigger starts transfer of one packet. */
      pEndp0OutdscrConfig->channelState = CY_USBD_DMA_CHN_ENABLED;
      pEndp0OutdscrConfig->dataSize = CY_USBD_DMA_WORD;             /* Transfer 4 bytes on each AHB cycle. */
      pEndp0OutdscrConfig->descriptorType = CY_USBD_DMA_2D_XFER;
      pEndp0OutdscrConfig->xCount = 0x10;     /* In each loop, transfer 0x10 * 4 = 0x40 bytes. */
      pEndp0OutdscrConfig->dstYincrement = 0x10; /* Increment by 0x10 * 4 = 64 bytes after each packet. */
      pEndp0OutdscrConfig->yCount = 1;        /* TBD: Update based on number of packets to be transferred. */
  } else {
      pEndp0OutdscrConfig->triggerOutType = CY_USBD_DMA_DESCR_CHAIN;
      pEndp0OutdscrConfig->triggerInType = CY_USBD_DMA_DESCR_CHAIN;
      pEndp0OutdscrConfig->channelState = CY_USBD_DMA_CHN_DISABLED;
      pEndp0OutdscrConfig->dataSize = CY_USBD_DMA_BYTE;
      pEndp0OutdscrConfig->descriptorType = CY_USBD_DMA_1D_XFER;
      pEndp0OutdscrConfig->xCount = 1;        /* TBD: Update based on actual packet size. */
      pEndp0OutdscrConfig->dstYincrement = 0; /* Not required for 1D tranfer */
      pEndp0OutdscrConfig->yCount = 0;        /* For 1D transfer we dont need this */
  }
}   /* end of function. */

/*
 * Function: Cy_USB_USBD_InitCpuDmaChannelCfg()
 * Description: It initializes DMA channel config.
 * Parameter: cy_stc_usbd_dma_chn_conf_t, cy_stc_usbd_dma_descr_t.
 * return: None.
 * Note: Other three config are same. if require then it can be passed
 *       as parameter.
 */
void
Cy_USB_USBD_InitCpuDmaChannelCfg (cy_stc_usbd_dma_chn_conf_t *pDmaChCfg,
                                          cy_stc_usbd_dma_descr_t *pDmaDscr)
{
    pDmaChCfg->descriptor = pDmaDscr;
    pDmaChCfg->priority = 1;
    pDmaChCfg->enable = false;
    pDmaChCfg->bufferable = false;
    return;
}   /* end of function  */



/*
 * IN transfers from EP0 : Need to move data from SRAM buffer to EP0 EPM.
 *
 * OUT transfers to EP0 : Need to move data from EP0 EPM to SRAM buffer.
 *
 * */

static void Cy_USB_USBD_InitEndp0CpuDmaDscr (cy_stc_usbd_dma_descr_conf_t *dmaDscr,
        cy_en_usb_endp_dir_t direction, bool enable2DXfer)
{

    dmaDscr->retrigger = CY_USBD_DMA_WAIT_FOR_REACT;
    dmaDscr->interruptType = CY_USBD_DMA_DESCR_CHAIN;


    dmaDscr->triggerOutType = (enable2DXfer) ? CY_USBD_DMA_X_LOOP : CY_USBD_DMA_DESCR_CHAIN;
    dmaDscr->triggerInType  = (enable2DXfer) ? CY_USBD_DMA_X_LOOP : CY_USBD_DMA_DESCR_CHAIN;
    dmaDscr->channelState   = (enable2DXfer) ? CY_USBD_DMA_CHN_ENABLED: CY_USBD_DMA_CHN_DISABLED;
    dmaDscr->dataSize       = (enable2DXfer) ? CY_USBD_DMA_WORD: CY_USBD_DMA_BYTE;
    dmaDscr->descriptorType = (enable2DXfer) ? CY_USBD_DMA_2D_XFER: CY_USBD_DMA_1D_XFER;
    
    /* Y Loop present only for 2D transfers */
    dmaDscr->yCount         = (enable2DXfer) ? 1 : 0;
    
    dmaDscr->dataPrefetch   = false;
    dmaDscr->srcTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
    dmaDscr->dstTransferSize = CY_USBD_DMA_XFER_SIZE_DATA;
    dmaDscr->srcAddress = NULL;  
    dmaDscr->dstAddress = NULL; 
    dmaDscr->srcXincrement = 1;
    dmaDscr->dstXincrement  = 1;
    
    if(direction == CY_USB_ENDP_DIR_IN) 
    {
        /* Minimum 1 X loop present to move data from SRAM to EPM */
        dmaDscr->xCount         = 0x01UL;     
        /* Increment by 16 (will result in 16*4 = 64 for 2D transfers*/
        dmaDscr->srcYincrement  = (enable2DXfer) ? 0x10UL : 0x00UL; 
        /* No increment needed for EPM */
        dmaDscr->dstYincrement  = 0x00; 
    }
    else 
    {
        /* Move 16 * WORD bytes for 2D Xfers. Minimum 1 XLoop for 1D Xfer*/
        dmaDscr->xCount         = (enable2DXfer) ? 0x10UL : 0x01UL;     
        /* No increment needed as EPM is SRC */
        dmaDscr->srcYincrement  = 0x00UL;
        /* Move destination pointer by 16*WORDs for 2D Xfers, NA for 1D xfers */
        dmaDscr->dstYincrement  = (enable2DXfer) ? 0x10UL : 0x00UL;
    }
    dmaDscr->nextDescriptor = NULL;
}


/*
 * Function: Cy_USB_USBD_cpuDmaInit()
 * Description: It initializes Cupu DMA functionality.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None.
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_cpuDmaInit (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_usbd_dma_descr_conf_t endp0InCpuDmaDscrConfig;
    cy_stc_usbd_dma_descr_conf_t endp0OutCpuDmaDscrConfig;

    cy_stc_usbd_dma_chn_conf_t cpuDmaCh0InConfig;
    cy_stc_usbd_dma_chn_conf_t cpuDmaCh1OutConfig;

    Cy_DMAC_Enable(pUsbdCtxt->pCpuDmacBase);
    /*
     * Prepare DMA channel for IN tranfer.  This includes prepare DMA dscr
     * config, then initialize dma descriptor, prepare dma channel config
     * and then initialize DMA channel.
     */
    Cy_USB_USBD_InitEndp0CpuDmaDscr(&endp0InCpuDmaDscrConfig, CY_USB_ENDP_DIR_IN, true);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr0), &endp0InCpuDmaDscrConfig);

    Cy_USB_USBD_InitEndp0CpuDmaDscr(&endp0InCpuDmaDscrConfig, CY_USB_ENDP_DIR_IN, false);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr1), &endp0InCpuDmaDscrConfig);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh0InXferDscr2), &endp0InCpuDmaDscrConfig);

    Cy_USB_USBD_InitCpuDmaChannelCfg(&cpuDmaCh0InConfig, &(pUsbdCtxt->dmaCh0InXferDscr0));
    Cy_USBD_DMAChn_Init(Cy_USBD_EP0In_DmaBase(pUsbdCtxt), pUsbdCtxt->channel0, &cpuDmaCh0InConfig);

    /* Similar way prepare DMA for Out channel. */
    Cy_USB_USBD_InitEndp0CpuDmaDscr(&endp0OutCpuDmaDscrConfig, CY_USB_ENDP_DIR_OUT, true);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr0), &endp0OutCpuDmaDscrConfig);

    Cy_USB_USBD_InitEndp0CpuDmaDscr(&endp0OutCpuDmaDscrConfig, CY_USB_ENDP_DIR_OUT, true);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr1), &endp0OutCpuDmaDscrConfig);
    Cy_USBD_DMADesc_Init(&(pUsbdCtxt->dmaCh1OutXferDscr2), &endp0OutCpuDmaDscrConfig);

    Cy_USB_USBD_InitCpuDmaChannelCfg(&cpuDmaCh1OutConfig, &(pUsbdCtxt->dmaCh1OutXferDscr0));
    Cy_USBD_DMAChn_Init(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt), pUsbdCtxt->channel1, &cpuDmaCh1OutConfig);
   /*
    * TBD: Later DMA interrupt can be registered from here but it is not
    * required as of now.
    */
    return CY_USBD_STATUS_SUCCESS;
}   /* end of function  */

/*
 * Function: Cy_USB_USBD_EndpInit()
 * Description: This function initializes HighSpeed and FullSpeed section of
 *              USBD layer and calls HS CAL layer initialization function.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_EndpInit (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_cal_ret_code_t calRetCode = CY_USB_CAL_STATUS_SUCCESS;
    cy_en_usbd_ret_code_t  retCode = CY_USBD_STATUS_SUCCESS;

    /*
     * Make sure endp_info and endp_config in sync.
     * Configure endpoint 0x00 first.
     * Disable all endpoint.
     * enable endpoint 0 during reset and others during set Config.
     */
    endpConfig.valid = 0x00;
    endpConfig.endpNumber = 0x00;
    endpConfig.endpDirection = CY_USB_ENDP_DIR_IN;
    endpConfig.endpType = CY_USB_ENDP_TYPE_CTRL;
    endpConfig.maxPktSize = 64;
    endpConfig.isoPkts = 0x00;
    endpConfig.burstSize = 0x00;
    endpConfig.streamID = 0x00;
    endpConfig.allowNakTillDmaRdy = 0x00;
    
    pUsbdCtxt->endpInfoIn[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoIn[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoIn[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoIn[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoIn[0].valid = endpConfig.valid;

    pUsbdCtxt->endpInfoOut[0].maxPktSize = endpConfig.maxPktSize;
    pUsbdCtxt->endpInfoOut[0].endpType = endpConfig.endpType;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;
    pUsbdCtxt->endpInfoOut[0].burstSize = endpConfig.burstSize;
    pUsbdCtxt->endpInfoOut[0].streamID = endpConfig.streamID;
    pUsbdCtxt->endpInfoOut[0].valid = endpConfig.valid;

    /* for endpoint 0  IN/OUT taken care in single call. */
    calRetCode = Cy_USBHS_Cal_EndpConfig(pUsbdCtxt->pCalCtxt, endpConfig);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }

    return retCode;
}   /* end of function  */


/*
 * Function: Cy_USB_USBD_DisableHsDevice()
 * Description: This function disable all HS device interrupt and
 *              make device invisible n BUS.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None
 */
void
Cy_USB_USBD_DisableHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        return;
    }
    /* Disconnect USB device from BUS, Clear and disable HS interrupt. */
    Cy_USBHS_Cal_DisconUsbPins(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_DisableAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    return;
}    /*  End of function Cy_USB_USBD_DisableHsDevice() */

/*
 * Function: Cy_USB_USBD_EnableHsDevice()
 * Description: This function enable HS device and make it visible on BUS.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None
 */
void
Cy_USB_USBD_EnableHsDevice (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
        return;
    }
    /*
     * Clear all interrupt, Enable required interrupt and
     * then make device visible on BUS.
     */
    Cy_USBHS_Cal_ClearAllDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_EnableReqDevCtrlIntr(pUsbdCtxt->pCalCtxt);
    Cy_USBHS_Cal_ConnUsbPins(pUsbdCtxt->pCalCtxt);
    return;
}    /*  End of function Cy_USB_USBD_EnableHsDevice() */

/*
 * Function: Cy_USB_USBD_ResetController()
 * Description: This function will call resetController based on current speed.
 * Parameter: cy_stc_usb_usbd_ctxt_t.
 * return: None
 * Note:
 */
cy_en_usbd_ret_code_t
Cy_USB_USBD_ResetController (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("ResetController:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

    if (pUsbdCtxt->devSpeed <= CY_USBD_USB_DEV_HS) {
        Cy_USBHS_Cal_ResetHsController(pUsbdCtxt->pCalCtxt);
    } else {
        /* TBD: Need to code for SS */
    }
    return(CY_USBD_STATUS_SUCCESS);
}   /* End of function Cy_USB_USBD_ResetController() */




/*
 * Function: Cy_USBD_ProcessMsg()
 * Description: Initialize This function sends msg to USBD thread.
 * Parameter: pUsbd, pCalMgs
 * return: true if context switch is to be initiated.
 */
bool
Cy_USBD_ProcessMsg (void *pUsbd, void *pCalMgs)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_usb_cal_msg_t *pMsg;
    bool retval = false;

    pUsbdCtxt = (cy_stc_usb_usbd_ctxt_t *)pUsbd;
    pMsg = (cy_stc_usb_cal_msg_t *)pCalMgs;

    /* Directly call the handle message function. */
    Cy_USBD_HandleMsg(pUsbdCtxt, pMsg);
    return retval;
}   /* end of function */

/*
 * Function: Cy_USBD_SendEgressZLP()
 * Description: This function sends ZLP for given endpoint.
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber
 * return: return code
 */
cy_en_usbd_ret_code_t
Cy_USBD_SendEgressZLP (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t endpNumber)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("SntEgrZlp:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

        calRetCode =
            Cy_USBHS_Cal_SendEgressZLP(pUsbdCtxt->pCalCtxt, endpNumber);

    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}


/*
 * Function: Cy_USBD_ClearZlpSlpIntrEnableMask()
 * Description: This function clears endpoint interrupt for ingr and egrs
 *              intr and enables respective mask register..
 * Parameter: cy_stc_usb_usbd_ctxt_t, endpNumber, cy_en_usb_endp_dir_t, zlpSlp
 * return: return code
 */
cy_en_usbd_ret_code_t
Cy_USBD_ClearZlpSlpIntrEnableMask (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                   uint32_t endpNumber,
                                   cy_en_usb_endp_dir_t endpDirection,
                                   bool zlpSlp)
{
    cy_en_usb_cal_ret_code_t calRetCode;
    cy_en_usbd_ret_code_t retCode;

    if (NULL == pUsbdCtxt) {
#if BL_DEBUG
        DBG_USBD_ERR("ClrZlpSlpEnaMas:usbdCtxtNull\r\n");
#endif
        return (CY_USBD_STATUS_CTXT_NULL);
    }

        calRetCode =
            Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask(pUsbdCtxt->pCalCtxt,
                                                   endpNumber,
                                                   endpDirection, zlpSlp);
    if (CY_USB_CAL_STATUS_SUCCESS == calRetCode) {
        retCode = CY_USBD_STATUS_SUCCESS;
    } else {
        retCode = CY_USBD_STATUS_FAILURE;
    }
    return(retCode);
}   /* end of function */


/*
 * Function: Cy_USBD_GetUSBLinkActive()
 * Description: This function ensures that the USB link is brought into the L0 (USB2) or U0 (USB3) state
 * when the respective low power states.
 * Parameter: cy_stc_usb_usbd_ctxt_t
 * return: cy_en_usbd_ret_code_t
 */
cy_en_usbd_ret_code_t
Cy_USBD_GetUSBLinkActive (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    cy_en_usb_cal_ret_code_t calRetStatus;

        calRetStatus = Cy_USBHS_Cal_GetLinkActive(pUsbdCtxt->pCalCtxt);

    if (CY_USB_CAL_STATUS_SUCCESS == calRetStatus) {
        return(CY_USBD_STATUS_SUCCESS);
    } else {
        return(CY_USBD_STATUS_FAILURE);
    }
}   /* End of function   */


/*
 * Function: Cy_USB_USBD_RetireRecvEndp0DataHs()
 * Description: This function will disable dma channel for HS which was
 *              submitted to recieve data.
 * Parameter: cy_stc_usb_usbd_ctxt_t
 * return: cy_en_usbd_ret_code_t
 */
void
Cy_USB_USBD_RetireRecvEndp0DataHs (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt)
{
    /* Disable the DMA channel. */
    Cy_USBD_DMAChn_Disable(Cy_USBD_EP0Out_DmaBase(pUsbdCtxt),
                           pUsbdCtxt->channel1);

    /* Flush any data in the EPM buffer. */
    Cy_USBD_FlushEndp(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_OUT);
    return;
}   /* End of function */

