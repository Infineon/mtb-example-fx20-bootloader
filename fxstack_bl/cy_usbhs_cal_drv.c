/***************************************************************************//**
* \file cy_usbhs_cal_drv.c
* \version 1.0
*
* \brief USB CAL implementation for bootloader
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
#include "cy_debug.h"
#include <string.h>

/* Non available bits used as TBD */
#define TBD_BIT_POSITION 0x00


/*******************************************************************************
 * Function name: Cy_USBHS_ConfigExtClkPin
 ****************************************************************************//**
 *
 * Function used to enable selection of external 24 MHz clock input provided
 * on external clock pin as reference for the PLL in the USB block.
 *
 * \param init - Set TRUE to initialize pin for external clock input
 *                Set FALSE to remove external clock input capability for the IO.
 *
 * \return Pass/fail status of configuration update.
 *******************************************************************************/
static cy_en_gpio_status_t Cy_USBHS_ConfigExtClkPin (bool init)
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;

    memset ((void *)&pinCfg, 0, sizeof(pinCfg));
    if(init)
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        pinCfg.hsiom     = HSIOM_SEL_ACT_4;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_ANALOG;
        pinCfg.hsiom     = HSIOM_SEL_GPIO;
    }
    status = Cy_GPIO_Pin_Init(GPIO_PRT5, CY_EXTERNAL_CLK_PIN, &pinCfg);
    return status;
}

/*******************************************************************************
 * Function name:Cy_USBHS_WaitForPllLock 
 ****************************************************************************//**
 *  Wait till timeout for USB HS PLL to get locked. 
 *
 * \param timeoutMs - Timeout value in milliseconds. 
 *                    Set timeout as 0 for infinite wait.
 *
 * \return True - PLL Lock, False - PLL not locked (timeout)
 *******************************************************************************/

static bool Cy_USBHS_WaitForPllLock (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t timeoutMs)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    uint32_t pllLockWait = 0;
    
    /* wait for PLL_LOCK interrupt and then clear the interrupt*/
    while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK))) 
    {
        if ((timeoutMs) && (pllLockWait++ > timeoutMs))
        {
            DBG_HSCAL_INFO("PLL Lock timedout\r\n");
            return false;
        }
        Cy_SysLib_Delay(1);
    }
    pPhyBase->INTR0  |= USBHSPHY_INTR0_PLL_LOCK;
    return true;
}

/*
 * Function: Cy_USBHS_Cal_SendMsg()
 * Description: This function send message to upper layer ie USBD layer.
 * Parameter: pMsg
 * return: bool
 * Note: This should call ISR safe routine only.
 */
bool
Cy_USBHS_Cal_SendMsg (cy_stc_usb_cal_ctxt_t *pCalCtxt, void *pMsg)
{
    return (pCalCtxt->msgCb(pCalCtxt->pUsbdCtxt, (void *)pMsg));
}

/*
 * Function: Cy_USBHS_Cal_UpdateXferCount()
 * Description: This function will update register with xfer count.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpointNumber, endpDirection, xferCount
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_UpdateXferCount (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNumber,
                              cy_en_usb_endp_dir_t endpDirection,
                              uint32_t xferCount)
{
    USBHSDEV_Type  *pCalBase;

    if ((endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDirection) {
        pCalBase->DEV_EPO_XFER_CNT[endpNumber] = xferCount;
    } else {
        pCalBase->DEV_EPI_XFER_CNT[endpNumber] = xferCount;
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */

/*
 * Function: Cy_USBHS_Cal_ConnUsbPins()
 * Description: This function will update register so that device will
 *               be visible on USB BUS.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ConnUsbPins (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DISCON);
    return(CY_USB_CAL_STATUS_SUCCESS);
}

/*
 * Function: Cy_USBHS_Cal_DisconUsbPins()
 * Description: This function will update register so that device will
 *               not be visible on USB BUS.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DisconUsbPins (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_DISCON;
    return(CY_USB_CAL_STATUS_SUCCESS);
}


/*
 * Function: Cy_USBHS_Cal_GetRemoteWakeupStatus()
 * Description: This function returns status of L2 remote wakeup in HW.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: TRUE if L2 remote wakeup is enabled otherwise FLASE.
 */
bool
Cy_USBHS_Cal_GetRemoteWakeupStatus (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(FALSE);
    }

    /* Check whether remote wakeup is allowed. */
    if ((pCalBase->DEV_LPM_ATTR & USBHSDEV_DEV_LPM_ATTR_L2_SUSP_RMT_WAKEUP_EN) != 0) {
        return(TRUE);
    } else {
        return(FALSE);
    }
}   /* end of function  */

/*
 * Function: Cy_USBHS_Cal_SignalRemotWakup()
 * Description: This function will update register so that device will
 *               initiate remote wakeup signaling ie it will try to comeout
 *               from L2_SUSPEND.
 * Parameter: cy_stc_usb_cal_ctxt_t, signal start/end
 * return: cy_en_usb_cal_ret_code_t
 * Note: It is callers responsibility to maintain 15ms time between Signal
 *       start and signal end.
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SignalRemotWakup (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                               bool startEndSignal)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* Device initiated L2 SUSPEND exit sequence. */
    if (startEndSignal) {
        pCalBase->DEV_PWR_CS &= (USBHSDEV_DEV_PWR_CS_DEV_SUSPEND | USBHSDEV_DEV_PWR_CS_FORCE_FS);
        pCalBase->DEV_LPM_TIM_1 =  0x33450;
        pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_SIGRSUME;
    } else {
        pCalBase->DEV_PWR_CS &= ~USBHSDEV_DEV_PWR_CS_SIGRSUME;
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*
 * Function: Cy_USBHS_Cal_HsHandleResume()
 * Description: Resume related functionality will be handled here.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: void
 */
void
Cy_USBHS_Cal_HsHandleResume (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    (void)pCalCtxt;
    (void)pCalBase;
    /* Handle L2 PHY sequence for HS and then disable suspend bit. */
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);

    /* TBD: How to bringup PHY Internal PLL - Generte 480MHz */
}   /* end of function */

/*
 * Function: Cy_USBHS_Cal_FsHandleResume()
 * Description: Resume related functionality will be handled here.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: void
 */
void
Cy_USBHS_Cal_FsHandleResume (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    /* Handle L2 PHY sequence for FS and then disable suspend bit. */
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);

    return;
}


/*
 * Function: Cy_USBHS_Cal_SetTestMode()
 * Description: This function will update register to setup required test mode.
 * Parameter: cy_stc_usb_cal_ctxt_t, cy_en_usbhs_cal_test_mode_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetTestMode (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              cy_en_usbhs_cal_test_mode_t testMode)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CS |= (testMode << USBHSDEV_DEV_CS_TEST_MODE_Pos);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function */


/*
 * Function: Cy_USBHS_Cal_GetDevAddress()
 * Description: This function will get device address assigned by Host.
 * Parameter: cy_stc_usb_cal_ctxt_t, poointer to device address.
 * return: return code
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_GetDevAddress (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint8_t *pDevAddr)
{

    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (pDevAddr == NULL) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    *pDevAddr = (uint8_t)((pCalBase->DEV_CS & USBHSDEV_DEV_CS_DEVICEADDR_Msk)
                          >> (USBHSDEV_DEV_CS_DEVICEADDR_Pos));
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function Cy_USBHS_Cal_GetDevAddress() */


/*
 * Function: Cy_USBHS_Cal_EndpSetClearNak()
 * Description: This function enable or disable NAK condition in hw.
 *              By setting NAK bit, endpoint will keep sending NAK till
 *              NAK bit is cleared.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpNumber, cy_en_usb_endp_dir_t, setClear
 * return: return code
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpSetClearNak (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                              uint32_t endpNumber,
                              cy_en_usb_endp_dir_t endpDirection,
                              bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if ((endpNumber != 0) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }
    retCode = CY_USB_CAL_STATUS_SUCCESS;

    /* Endpoint 0 is special case */
    switch (endpNumber) {
        case 0:
            if (setClear) {
                pCalBase->DEV_EPI_CS[0] |= USBHSDEV_DEV_EPI_CS_NAK;
                pCalBase->DEV_EPO_CS[0] |= USBHSDEV_DEV_EPO_CS_NAK;
            } else {
                pCalBase->DEV_EPI_CS[0] &= (~USBHSDEV_DEV_EPI_CS_NAK);
                pCalBase->DEV_EPO_CS[0] &= (~USBHSDEV_DEV_EPO_CS_NAK);
            }
            break;
        }

    return(retCode);
}   /* end of function  */


/*
 * Function: Cy_USBHS_Cal_SetClearNakAll()
 * Description: This function either set or clear NAK for all endpoint.
 * Parameter:
 * return: return code
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetClearNakAll (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (setClear) {
        /* Set CLR bit and set NAKALL bit */
        pCalBase->DEV_CS &=
             ((pCalBase->DEV_CS & (~USBHSDEV_DEV_CS_SETUP_CLR_BUSY)) |
              (USBHSDEV_DEV_CS_NAKALL));
    } else {
        pCalBase->DEV_CS &=
                (~(USBHSDEV_DEV_CS_SETUP_CLR_BUSY | USBHSDEV_DEV_CS_NAKALL));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */


/*
 * Function: Cy_USBHS_Cal_EndpIsNakNrdySet()
 * Description: This function checks whether the specified endpoint is currently in the NAKed state.
 * Parameter:
 *     cy_stc_usb_cal_ctxt_t *pCalCtxt: USBHS-CAL context structure.
 *     uint32_t endpNumber: Endpoint index.
 *     cy_en_usb_endp_dir_t endpDirection: Endpoint direction.
 * return: Whether endpoint is currently NAKed.
 */
bool Cy_USBHS_Cal_EndpIsNakNrdySet (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                    uint32_t endpNumber,
                                    cy_en_usb_endp_dir_t endpDirection)
{
    USBHSDEV_Type *pCalBase;
    bool endpNaked = false;

    if ((endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpNaked);
    }

    if ((pCalCtxt != NULL) && (pCalCtxt->pCalBase != NULL)) {
        pCalBase = pCalCtxt->pCalBase;

        if (endpDirection == CY_USB_ENDP_DIR_OUT) {
            if ((pCalBase->DEV_EPO_CS[endpNumber] & USBHSDEV_DEV_EPO_CS_NAK) ||
                (pCalBase->DEV_CS & USBHSDEV_DEV_CS_NAKALL)) {
                endpNaked = true;
            }
        } else {
            if ((pCalBase->DEV_EPI_CS[endpNumber] & USBHSDEV_DEV_EPI_CS_NAK) ||
                (pCalBase->DEV_CS & USBHSDEV_DEV_CS_NAKALL)) {
                endpNaked = true;
            }
        }
    }
    return endpNaked;
}   /* End of function() */

/*
 * Function: Cy_USBHS_Cal_EndpIsStallSet()
 * Description: This function checks whether the specified endpoint is
 *              currently in the STALLed state.
 * Parameter:
 *     cy_stc_usb_cal_ctxt_t *pCalCtxt: USBHS-CAL context structure.
 *     uint32_t endpNumber: Endpoint index.
 *     cy_en_usb_endp_dir_t endpDirection: Endpoint direction.
 * return: Whether endpoint is currently Stalled.
 */
bool Cy_USBHS_Cal_EndpIsStallSet (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                  uint32_t endpNumber,
                                  cy_en_usb_endp_dir_t endpDirection)
{
    USBHSDEV_Type *pCalBase;
    bool endpStalled = false;

    if ((endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return(endpStalled);
    }

    if ((pCalCtxt != NULL) && (pCalCtxt->pCalBase != NULL)) {
        pCalBase = pCalCtxt->pCalBase;

        if (endpDirection == CY_USB_ENDP_DIR_OUT) {
            endpStalled =
            (bool)((pCalBase->DEV_EPO_CS[endpNumber] & USBHSDEV_DEV_EPO_CS_STALL) != 0);
        } else {
            endpStalled =
            (bool)((pCalBase->DEV_EPI_CS[endpNumber] & USBHSDEV_DEV_EPI_CS_STALL) != 0);
        }
    }
    return endpStalled;
}   /* end of function */


/*
 * Function: Cy_USBHS_Cal_EndpSetClearStall()
 * Description: This function enable or disable STALL condition in hw.
 * Parameter:
 *     cy_stc_usb_cal_ctxt_t *pCalCtxt: USBHS-CAL context structure.
 *     uint32_t endpNumber: Endpoint index.
 *     cy_en_usb_endp_dir_t endpDirection: Endpoint direction
 *     bool setClear: 1 for set and 0 for clear.
 * return: return code
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpSetClearStall (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                uint32_t endpNumber,
                                cy_en_usb_endp_dir_t endpDirection,
                                bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if ((endpNumber != 0) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    retCode = CY_USB_CAL_STATUS_SUCCESS;

    /* Only Endpoint 0 is supported by bootloader */
    switch (endpNumber) {
        case 0:
            if (setClear) {
                pCalBase->DEV_EPI_CS[0] |= USBHSDEV_DEV_EPI_CS_STALL;
                pCalBase->DEV_EPO_CS[0] |= USBHSDEV_DEV_EPO_CS_STALL;

                /* For endp 0 there is extra instruction to hw to come out
                 * of NAK state.
                 */
                pCalBase->DEV_CS |= (USBHSDEV_DEV_CS_SETUP_CLR_BUSY);

            } else {
                pCalBase->DEV_EPI_CS[0] &= (~USBHSDEV_DEV_EPI_CS_STALL);
                pCalBase->DEV_EPO_CS[0] &= (~USBHSDEV_DEV_EPO_CS_STALL);
            }
            break;
    }

    return(retCode);
}   /* end of function     */


/*
 * Function: Cy_USBHS_Cal_EnableEndp()
 * Description: This function enables/disable endpoint and set/reset
 *               respective interrupt.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpNumber, endpDirection, enable
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t endpNumber,
                         cy_en_usb_endp_dir_t endpDirection,  bool enable)
{
    USBHSDEV_Type  *pCalBase;

    if ((endpNumber != CY_USB_ENDP_0) ||
	(endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
	return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
	return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /*
     * Enable: set valid bit and enable interrupt for this endpoint.
     * Disable: clear valid bit and disable interrupt for this endpoint.
     */
    if (enable) {
	pCalBase->DEV_EPO_CS[endpNumber] |= USBHSDEV_DEV_EPO_CS_VALID;
	pCalBase->DEV_EP_INTR_MASK |=
	    (0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER));
	pCalBase->DEV_EPI_CS[endpNumber] |= USBHSDEV_DEV_EPI_CS_VALID;

    } else {
	pCalBase->DEV_EPO_CS[endpNumber] &= (~(USBHSDEV_DEV_EPO_CS_VALID));
	pCalBase->DEV_EP_INTR_MASK &=
	    (~(0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER)));
	pCalBase->DEV_EPI_CS[endpNumber] &= (~(USBHSDEV_DEV_EPI_CS_VALID));

    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function Cy_USBHS_Cal_EnableEndp () */


/*
 * Function: Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask()
 * Description: This function clears endpoint interrupt for ingr and egrs
 *              intr and enables respective mask register.
 *              writing 1 in intr register will clear the interrupt.
 *              writting 1 in mask register will enable that register.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpNumber, cy_en_usb_endp_dir_t, zlpSlp
 * return: cy_en_usb_cal_ret_code_t
 * zlpSlp = 1 means zlp and 0 means slp
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                        uint32_t endpNumber,
                                        cy_en_usb_endp_dir_t endpDirection,
                                        bool zlpSlp)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();

    /* zlp: 0-15 bits and slp: 16-31 bits for ingr and egrs register. */
    if (CY_USB_ENDP_DIR_IN == endpDirection) {
        if (zlpSlp) {
            pCalBase->DEV_EP_EGRS_INTR = (0x01 << (endpNumber));
            pCalBase->DEV_EP_EGRS_INTR_MASK |= (0x01 << (endpNumber));
        } else {
            pCalBase->DEV_EP_EGRS_INTR =
                               (0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER));
            pCalBase->DEV_EP_EGRS_INTR_MASK |=
                               (0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER));
        }
    } else {
        if (zlpSlp) {
            pCalBase->DEV_EP_INGRS_INTR = 0x01 << (endpNumber);
            pCalBase->DEV_EP_INGRS_INTR_MASK |= 0x01 << (endpNumber);
        } else {
            pCalBase->DEV_EP_INGRS_INTR =
                                (0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER));
            pCalBase->DEV_EP_INGRS_INTR_MASK |=
                                (0x01 << (endpNumber + CY_USB_MAX_ENDP_NUMBER));
        }
    }

    Cy_SysLib_ExitCriticalSection(intMask);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function */


/*
 * Function: Cy_USBHS_Cal_HandleCtrlOutSlp()
 * Description: This function checks for the presence of and clears the EP_INGRS_INTR
 *              corresponding to SLP for the control endpoint.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_HandleCtrlOutSlp (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (0 != ((pCalBase->DEV_EP_INGRS_INTR) & (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos))) {
        pCalBase->DEV_EP_INGRS_INTR = (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos);
    }

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*
 * Function: Cy_USBHS_Cal_EnableCtrlSlpIntr()
 * Description: This function enables the SLP_RCVD interrupt for EP0-OUT.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableCtrlSlpIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    pCalBase->DEV_EP_INGRS_INTR_MASK |= (1U << USBHSDEV_DEV_EP_INGRS_INTR_MASK_EP_INGRS_SLP_RCVD_Pos);
    Cy_SysLib_ExitCriticalSection(intMask);

    return CY_USB_CAL_STATUS_SUCCESS;
}

/*
 * Function: Cy_USBHS_Cal_FlushEndp()
 * Description: This function will flush data available in perticular
 *              endpoint FIFO.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpNumber, endpDirection
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_FlushEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                       uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if ((endpNumber >= CY_USB_MAX_ENDP_NUMBER) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDirection) {
    } else {
        pCalBase->EEPM_ENDPOINT[endpNumber] |=
                                        (USBHSDEV_EEPM_ENDPOINT_EGRS_FLUSH_EP);
        /* wait for ms as per hw recommentation. */
        pCalBase->EEPM_ENDPOINT[endpNumber] &=
                                     (~(USBHSDEV_EEPM_ENDPOINT_EGRS_FLUSH_EP));
        pCalBase->DEV_EP_INTR_MASK |= (0x01 << (endpNumber));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */


/*
 * Function: Cy_USBHS_Cal_FlushAllEndp()
 * Description: This function will flush data available in all Ingress or
 *              egress endpoint FIFO.
 * Parameter: cy_stc_usb_cal_ctxt_t, endpDirection
 * return: cy_en_usb_cal_ret_code_t
 * Note: Clarified from H/W team that no delay is required in FlushAll.
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_FlushAllEndp (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                           cy_en_usb_endp_dir_t endpDirection)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (endpDirection >= CY_USB_ENDP_DIR_INVALID) {
        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

    if (CY_USB_ENDP_DIR_OUT == endpDirection) {
        pCalBase->EPM_CS |= (USBHSDEV_EPM_CS_EGRS_FORCE_FLUSH_ALL);
        pCalBase->EPM_CS &= (~(USBHSDEV_EPM_CS_EGRS_FORCE_FLUSH_ALL));
    } else {
        pCalBase->EPM_CS |= (USBHSDEV_EPM_CS_IGRS_FORCE_FLUSH_ALL);
        pCalBase->EPM_CS &= (~(USBHSDEV_EPM_CS_IGRS_FORCE_FLUSH_ALL));
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function() */


/*
 * Function: Cy_USBHS_Cal_EndpConfig()
 * Description: This function handles configuration of endpoint.
 * Parameter: cy_stc_usb_cal_ctxt_t and cy_usbhs_cal_endp_config_t.
 * return: cy_en_usb_cal_ret_code_t
 * TBD: As off now Only OUT endpoint interrupts are enabled.
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EndpConfig (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                            cy_stc_usb_endp_config_t configParam)
{
    cy_en_usb_endp_dir_t endpDirection = configParam.endpDirection;
    cy_en_usb_endp_type_t endpType = configParam.endpType;
    uint32_t endpNumber = configParam.endpNumber;
    uint32_t maxPktSize = configParam.maxPktSize;
    uint32_t dev_ep_cs = 0x00;
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* check all possibility of wrong input parameter.
     * BL update - limit to support only EP0 */
    if ((endpNumber != CY_USB_ENDP_0) ||
        (endpType >= CY_USB_ENDP_TYPE_INVALID) ||
        (endpDirection >= CY_USB_ENDP_DIR_INVALID)) {

        return (CY_USB_CAL_STATUS_BAD_PARAM);
    }

        /* Start by disabling the endpoint. */
        pCalBase->DEV_EPO_CS[endpNumber] &= ~USBHSDEV_DEV_EPO_CS_VALID;

        dev_ep_cs = pCalBase->DEV_EPO_CS[endpNumber];
        /* first clear payload related bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk);
        dev_ep_cs |= ((maxPktSize & USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk) <<
                                        (USBHSDEV_DEV_EPI_CS_PAYLOAD_Pos));
        /* First clear type bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_TYPE_Msk);
        dev_ep_cs |= ((endpType & 0x03) << USBHSDEV_DEV_EPO_CS_TYPE_Pos);

        /* for non ISO/INTR two bits are 0 so dont need else condition. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_ISOINPKS_Msk);
        pCalBase->DEV_EPO_CS[endpNumber] = dev_ep_cs;

        if (configParam.allowNakTillDmaRdy) {
            pCalBase->IEPM_ENDPOINT[endpNumber] |=
                         USBHSDEV_IEPM_ENDPOINT_ALLOW_NAK_TILL_DMA_RDY_Msk;
        } else {
            pCalBase->IEPM_ENDPOINT[endpNumber] &=
                         (~USBHSDEV_IEPM_ENDPOINT_ALLOW_NAK_TILL_DMA_RDY_Msk);
        }

        if (configParam.valid) {
            /* Enable the endpoint after other configurations have been set. */
            pCalBase->DEV_EPO_CS[endpNumber] |= USBHSDEV_DEV_EPO_CS_VALID;
        } else {
            /* Disable the endpoint first and then clear interrupt settings. */
            pCalBase->DEV_EPO_CS[endpNumber] &= (~(USBHSDEV_DEV_EPO_CS_VALID));
        }


        /* Start by disabling the endpoint. */
        pCalBase->DEV_EPI_CS[endpNumber] &= ~USBHSDEV_DEV_EPI_CS_VALID;

        /* Direction is IN */
        dev_ep_cs = pCalBase->DEV_EPI_CS[endpNumber];
        /* first clear payload related bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk);
        dev_ep_cs |= ((maxPktSize & USBHSDEV_DEV_EPI_CS_PAYLOAD_Msk) <<
                                        (USBHSDEV_DEV_EPI_CS_PAYLOAD_Pos));
        /* First clear type bits and then set to right value. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_TYPE_Msk);
        dev_ep_cs |= ((endpType & 0x03) << USBHSDEV_DEV_EPO_CS_TYPE_Pos);

        /* for non ISO/INTR two bits are 0 so dont need else condition. */
        dev_ep_cs &= (~USBHSDEV_DEV_EPO_CS_ISOINPKS_Msk);
        pCalBase->DEV_EPI_CS[endpNumber] = dev_ep_cs;

        /* if endp need to enable then interrupt also need to be enabled. */
        if (configParam.valid) {
            pCalBase->DEV_EPI_CS[endpNumber] |= USBHSDEV_DEV_EPI_CS_VALID;
        } else {
            pCalBase->DEV_EPI_CS[endpNumber] &= (~(USBHSDEV_DEV_EPI_CS_VALID));
        }

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function   */


/*
 * Function: Cy_USBHS_Cal_UpdateEpIntrMask()
 * Description: This function updates the interrupt mask for an endpoint with the desired values.
 * Parameter:
 *      cy_stc_usb_cal_ctxt_t *pCalCtxt: HS-CAL context structure
 *      uint32_t endpNumber: Endpoint index
 *      cy_en_usb_endp_dir_t endpDirection: Direction of the endpoint
 *      uint32_t epIntrMask: Interrupt mask bits to be set or cleared.
 *      bool setClear: Whether the Interrupt Mask bits are to be set or cleared.
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_UpdateEpIntrMask (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                               uint32_t endpNumber,
                               cy_en_usb_endp_dir_t endpDirection,
                               uint32_t epIntrMask,
                               bool setClear)
{
    USBHSDEV_Type  *pCalBase;
    uint32_t intMask;
    uint32_t epCs;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    intMask = Cy_SysLib_EnterCriticalSection();
    if (endpDirection == CY_USB_ENDP_DIR_OUT) {
        epCs = pCalCtxt->pCalBase->DEV_EPO_CS[endpNumber];
        if (setClear) {
            pCalCtxt->pCalBase->DEV_EPO_CS[endpNumber] = ((epCs & 0xFF01FFFFUL) | epIntrMask);
        } else {
            pCalCtxt->pCalBase->DEV_EPO_CS[endpNumber] = (epCs & (0xFF01FFFFUL & ~epIntrMask));
        }
    } else {
        epCs = pCalCtxt->pCalBase->DEV_EPI_CS[endpNumber];
        if (setClear) {
            pCalCtxt->pCalBase->DEV_EPI_CS[endpNumber] = ((epCs & 0xFF01FFFFUL) | epIntrMask);
        } else {
            pCalCtxt->pCalBase->DEV_EPI_CS[endpNumber] = (epCs & (0xFF01FFFFUL & ~epIntrMask));
        }
    }
    Cy_SysLib_ExitCriticalSection(intMask);

    return(CY_USB_CAL_STATUS_SUCCESS);
}


/*
 * Function: Cy_USBHS_Cal_SendAckSetupDataStatusStage()
 * Description: This function update register so that device will send ACK
 *              to complete control transfer.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SendAckSetupDataStatusStage (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;
    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CS |= (USBHSDEV_DEV_CS_SETUP_CLR_BUSY);
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */

/*
 * Function: Cy_USBHS_Cal_ClearAllDevCtlIntr()
 * Description: This function will clear all interrupt bit in devCtrl register.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_ClearAllDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;

    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CTL_INTR = 0xFFFFFFFF;
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function() */


/*
 * Function: Cy_USBHS_Cal_DisableAllDevCtrlIntr()
 * Description: This function will disable all possible interrupt generated
 *               by USB device.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_DisableAllDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    /* All interrupts under dev_ctl_reg is disabled. */
    pCalBase->DEV_CTL_INTR_MASK &=
        ~(USBHSDEV_DEV_CTL_INTR_MASK_SETADDR | USBHSDEV_DEV_CTL_INTR_MASK_SOF |
         USBHSDEV_DEV_CTL_INTR_MASK_SUSP | USBHSDEV_DEV_CTL_INTR_MASK_URESET |
         USBHSDEV_DEV_CTL_INTR_MASK_HSGRANT |
         USBHSDEV_DEV_CTL_INTR_MASK_SUTOK |
         USBHSDEV_DEV_CTL_INTR_MASK_SUDAV |
         USBHSDEV_DEV_CTL_INTR_MASK_ERRLIMIT |
         USBHSDEV_DEV_CTL_INTR_MASK_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_SLEEP_REQ |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */

/*
 * Function: Cy_USBHS_Cal_EnableReqDevCtrlIntr()
 * Description: This function will enable required interrupt under
 *              dev ctrl reister.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_EnableReqDevCtrlIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase->DEV_CTL_INTR_MASK |=
        (USBHSDEV_DEV_CTL_INTR_MASK_SETADDR |
         USBHSDEV_DEV_CTL_INTR_MASK_SUSP |
         USBHSDEV_DEV_CTL_INTR_MASK_URESET |
         USBHSDEV_DEV_CTL_INTR_MASK_HSGRANT |
         USBHSDEV_DEV_CTL_INTR_MASK_SUDAV |
         USBHSDEV_DEV_CTL_INTR_MASK_ERRLIMIT |
         USBHSDEV_DEV_CTL_INTR_MASK_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_SLEEP_REQ |
         USBHSDEV_DEV_CTL_INTR_MASK_L1_URESUME |
         USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE);
         //USBHSDEV_DEV_CTL_INTR_MASK_SOF);

    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function()  */


/* Function: cy_usbhs_intr_handler()
 * Description: This function handle USBHS interrupt.
 * Note: RESET, HSGRANT then RESET_DONE is the sequence of interrupt.
 * SoF is not required so no need to allocate CPU time to SoF.
 * SUTOK is not used in fw since SUDAV is more relavent.
 * ERRORLIMIT also not used. If require then it can be implemented.
 */
bool
Cy_USBHS_Cal_IntrHandler (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    cy_stc_usb_cal_msg_t msg = {CY_USB_CAL_MSG_INVALID,{0,0}};
    uint32_t activeIntr;
    bool yield_reqd = false;
    volatile uint32_t count;

    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    /* get active interrupt list and address them one by one */
    activeIntr = ((pCalBase->DEV_CTL_INTR) & (pCalBase->DEV_CTL_INTR_MASK));

    if (activeIntr != 0) {

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SUSP) {
            /* When Host Suspend the Bus. Handle suspend mode */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SUSP;
            msg.type = CY_USB_CAL_MSG_SUSP;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
#if BL_DEBUG
            DBG_HSCAL_INFO("SUSPEND INTR\r\n");
#endif
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_URESET) {
           /* When Host initiates USB reset. Handle start of reset */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_URESET;
            msg.type = CY_USB_CAL_MSG_RESET;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
#if BL_DEBUG
            DBG_HSCAL_INFO("RESET INTR\r\n");
#endif
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_HSGRANT) {
           /* When Host Grant high speed communication. Handle HS mode */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_HSGRANT;
            msg.type = CY_USB_CAL_MSG_HSGRANT;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
#if BL_DEBUG
            DBG_HSCAL_INFO("HSGNT INTR\r\n");
#endif
        }

        /* Handle end of reset done */
        if (activeIntr & USBHSDEV_DEV_CTL_INTR_RESETDONE) {
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_RESETDONE;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_RESET_DONE;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_RESETDONE;
#if BL_DEBUG
            DBG_HSCAL_INFO("RESET DONE INTR\r\n");
#endif
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_STATUS_STAGE) {
            /* Handle completion of status stage */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_STATUS_STAGE;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_STATUS_STAGE;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_STATUS_STAGE;

        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SUDAV) {
            /* Handle setup with data */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SUDAV;
            /* Get setup data, prepare message and send message to upper layer. */
            msg.type = CY_USB_CAL_MSG_SUDAV;
            msg.data[0] = pCalBase->DEV_SETUPDAT_0;
            msg.data[1] = pCalBase->DEV_SETUPDAT_1;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SETADDR) {
            /* Handle completion of set address  */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_SETADDR;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SETADDR;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_SETADDR;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_SETADDR;
#if BL_DEBUG
            DBG_HSCAL_INFO("SET ADDR INTR\r\n");
#endif
        }


        if (activeIntr & USBHSDEV_DEV_CTL_INTR_L1_SLEEP_REQ) {
            /* Handle L1 sleep request */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_L1_SLEEP_REQ;
        }

        /* It is URESUME START interrupt arrived when L2 or L1 resume starts. */
        if (activeIntr & USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED) {
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_HOST_URSUME_ARRIVED;
            msg.type = CY_USB_CAL_MSG_RESUME_START;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
#if BL_DEBUG
            DBG_HSCAL_INFO("HOST_URSUME INTR\r\n");
#endif
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_URESUME) {
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_URESUME;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_RESUME_END;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
#if BL_DEBUG
            DBG_HSCAL_INFO("RESUME INTR\r\n");
#endif
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_L1_URESUME) {
            /* Handle L1 resume */
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_L1_URESUME;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_L1_URESUME;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_SOF) {
            /* Handle Sof */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_SOF;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_SOF;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_SOF;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_SOF;
        }

        if (activeIntr & USBHSDEV_DEV_CTL_INTR_ERRLIMIT) {
            /* Handle Error Limit Interrupt */
            pCalBase->DEV_CTL_INTR_MASK &= ~USBHSDEV_DEV_CTL_INTR_MASK_ERRLIMIT;
            pCalBase->DEV_CTL_INTR = USBHSDEV_DEV_CTL_INTR_ERRLIMIT;
            /* fetch count value  and clear it by writting 0x00. */
            count = ((pCalBase->DEV_CS) & (USBHSDEV_DEV_CS_COUNT_Msk));
            count = (count >> (USBHSDEV_DEV_CS_COUNT_Pos));
            msg.data[0] = count;
            /* Here Prepare ISR SAFE message and send to upper layer. */
            msg.type = CY_USB_CAL_MSG_ERRLIMIT;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            pCalBase->DEV_CS &= (~(USBHSDEV_DEV_CS_COUNT_Msk));
            pCalBase->DEV_CTL_INTR_MASK |= USBHSDEV_DEV_CTL_INTR_MASK_ERRLIMIT;
        }

    }

    /* Handling Endpoint interrupts  */
    volatile uint32_t actDevEpIntr;
    volatile uint32_t epIngrActIntr;
    volatile uint32_t dev_ep_cs;
    uint32_t endpNum;

    /* Check for EP_INGRS interrupts. */
    epIngrActIntr = ((pCalBase->DEV_EP_INGRS_INTR) & (pCalBase->DEV_EP_INGRS_INTR_MASK));
    endpNum = 0;
    while (epIngrActIntr != 0) {
        /* Check if ZLP interrupt is asserted for any OUT endpoint. */
        if ((epIngrActIntr & (1 << endpNum)) != 0) {
            /* Disable the interrupt instead of clearing it. Interrupt shall be cleared and re-enabled by the APP. */
            pCalBase->DEV_EP_INGRS_INTR_MASK &= ~(0x00000001UL << endpNum);

            msg.type = CY_USB_CAL_MSG_OUT_ZLP;
            msg.data[0] = endpNum;
            msg.data[1] = 0x00;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);

#if (!OUT_SLP_ZLP_DELAYED_HANDLER)
            /* Clear and re-enable the interrupt. */
            pCalBase->DEV_EP_INGRS_INTR = (0x0001UL << endpNum);
            pCalBase->DEV_EP_INGRS_INTR_MASK |= (0x0001UL << endpNum);
#endif /* (!OUT_SLP_ZLP_DELAYED_HANDLER) */
        }

        /* Check if SLP interrupt is asserted for any OUT endpoint. */
        if ((epIngrActIntr & (1 << (endpNum + USBHSDEV_DEV_EP_INGRS_INTR_EP_INGRS_SLP_RCVD_Pos))) != 0) {

#if (OUT_SLP_ZLP_DELAYED_HANDLER)
            /* In case of EP0, clear and disable the interrupt.
             * For other endpoints, just disable the interrupt. Interrupt shall be cleared and re-enabled by the APP. */
            pCalBase->DEV_EP_INGRS_INTR_MASK &= ~(0x00010000UL << endpNum);

            if (endpNum == 0) {
                pCalBase->DEV_EP_INGRS_INTR = (1 << USBHSDEV_DEV_EP_INGRS_INTR_EP_INGRS_SLP_RCVD_Pos);
            } else {
                msg.type = CY_USB_CAL_MSG_OUT_SLP;
                msg.data[0] = endpNum;
                /*
                 *  0-9 bit in the register gives size of slp.
                 *  value 0x00 means 1 byte and 0x01 means 2 bytes so
                 *  while sending message add 1.
                 */
                msg.data[1] = ((pCalBase->IEPM_ENDPOINT[endpNum]) & USBHSDEV_IEPM_ENDPOINT_INGRS_SLP_BYTE_COUNT_Msk) + 1;
                yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            }
#else
            /* Disable the interrupt first. */
            pCalBase->DEV_EP_INGRS_INTR_MASK &= ~(0x00010000UL << endpNum);

            /* Notify USBD for handling. */
            /*
             *  0-9 bit in the register gives size of slp.
             *  value 0x00 means 1 byte and 0x01 means 2 bytes so
             *  while sending message add 1.
             */
            msg.type    = CY_USB_CAL_MSG_OUT_SLP;
            msg.data[0] = endpNum;
            msg.data[1] = ((pCalBase->IEPM_ENDPOINT[endpNum]) & USBHSDEV_IEPM_ENDPOINT_INGRS_SLP_BYTE_COUNT_Msk) + 1;
            yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);

            /* Clear the interrupt. Re-enable it for endpoints other than 0. */
            pCalBase->DEV_EP_INGRS_INTR = (0x00010000UL << endpNum);
            if (endpNum != 0) {
                pCalBase->DEV_EP_INGRS_INTR_MASK |= (0x00010000UL << endpNum);
            }
#endif /* OUT_SLP_ZLP_DELAYED_HANDLER */
        }

        epIngrActIntr &= ~(0x10001UL << endpNum);
        endpNum++;
    }

    /* Check for other EP interrupts. */
    actDevEpIntr = (pCalBase->DEV_EP_INTR & pCalBase->DEV_EP_INTR_MASK);

    /* Clear the interrupts we have read. */
    pCalBase->DEV_EP_INTR = actDevEpIntr;

    endpNum = 0;
    while (actDevEpIntr != 0) {
        /* Check for interrupt on IN endpoint. */
        if ((actDevEpIntr & (1UL << endpNum)) != 0) {
            dev_ep_cs =  pCalBase->DEV_EPI_CS[endpNum];

            /* Currently, we only need to handle the ZERO interrupt. */
            if ((dev_ep_cs & USBHSDEV_DEV_EPI_CS_ZERO) != 0) {
                msg.type = CY_USB_CAL_MSG_IN_ZLP;
                msg.data[0] = endpNum;
                msg.data[1] = 0x00;
                yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Clear the interrupts that are active by writing 1 into the status bits. */
            pCalBase->DEV_EPI_CS[endpNum] = dev_ep_cs;

            actDevEpIntr &= ~(1UL << endpNum);
        }

        /* Check for interrupt on OUT endpoint. */
        if ((actDevEpIntr & (0x10000UL << endpNum)) != 0) {
            dev_ep_cs = pCalBase->DEV_EPO_CS[endpNum];

            /* Currently, we only need to handle the DONE interrupt. */
            if ((dev_ep_cs & USBHSDEV_DEV_EPO_CS_DONE_Msk) != 0) {

                /* Leave the DONE interrupt disabled until the message is handled by USBD. */
                dev_ep_cs &= ~USBHSDEV_DEV_EPO_CS_DONE_MASK_Msk;

                msg.type = CY_USB_CAL_MSG_OUT_DONE;
                msg.data[0] = endpNum;
                msg.data[1] = 0x00;
                yield_reqd |= Cy_USBHS_Cal_SendMsg(pCalCtxt, &msg);
            }

            /* Clear the interrupts that are active by writing 1 into the status bits. */
            pCalBase->DEV_EPO_CS[endpNum] = dev_ep_cs;

            actDevEpIntr &= ~(0x10000UL << endpNum);
        }

        endpNum++;
    }

    return yield_reqd;
}   /* end of function cy_usbhs_intr_handler() */


/*
 * Function: Cy_USBHS_Cal_HsHandleL1Sleep()
 * Description: L1-SLEEP request during device in high speed mode will
 *              be handled here.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_HsHandleL1Sleep (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    return;
}   /* end of function () */


/*
 * Function: Cy_USBHS_Cal_HsHandleL1Wakeup()
 * Description: L1-Wakeup request for HS is handled in this function.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_HsHandleL1Wakeup (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
#if PHY_INIT_ENABLE
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
    /* wait for 8us  */
    Cy_SysLib_DelayUs(8);
    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;


    while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK)));
    /* RW1C */
    pPhyBase->INTR0  |= USBHSPHY_INTR0_PLL_LOCK;
#endif /* PHY_INIT_ENABLE */
    return;
}

/*
 * Function: Cy_USBHS_Cal_DevInitiateddResumeL1Sleep()
 * Description: Device initiated resume from L1Speep.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 * Note: As per HW BROS, duration should be 0x5DC. Must
 */
void
Cy_USBHS_Cal_DevInitiateddResumeL1Sleep (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t duration)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    pCalBase->DEV_LPM_TIM_1 =  duration;
    pCalBase->DEV_PWR_CS |=  USBHSDEV_DEV_PWR_CS_SIGRSUME;
    return;
}   /* end of function */


/*
 * Function: Cy_USBHS_Cal_DevInitiateddResumeL2Sleep()
 * Description: Device initiated resume from L1Speep.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 * NOTE: Resume from L1 or L2, Duration has some trick otherwise we can combine
 *       both the function.
 */
void
Cy_USBHS_Cal_DevInitiateddResumeL2Sleep (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                         uint32_t duration)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    pCalBase->DEV_LPM_TIM_1 =  duration;
    pCalBase->DEV_PWR_CS |=  USBHSDEV_DEV_PWR_CS_SIGRSUME;
    return;
}   /* end of function () */

/*
 * Function: Cy_USBHS_Cal_HsHandleSuspend()
 * Description: Handling suspend in high speed mode.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_HsHandleSuspend (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    (void)pPhyBase;
    (void)pCalBase;

    return;
}   /* end of function() */

/*
 * Function: Cy_USBHS_Cal_FsHandleSuspend()
 * Description: Handling suspend in full speed mode.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_FsHandleSuspend (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    (void)pPhyBase;
    (void)pCalBase;
    return;
}   /* end of function */


/* Function: Cy_USBHS_Cal_InitHSMode()
 * Description: This function takes care of CAL layer initialization in
 *              High speed mode.
 * Parameter: cy_stc_usb_cal_ctxt_t.
 * return: none
 */
void
Cy_USBHS_Cal_InitUsbControllerHSMode (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

    (void)pCalBase;

    /* Program CLK_ECO_CONFIG as per BROS. */
    Cy_USBHS_Cal_InitHsPhy(pCalCtxt);
    return;
}

/*
 * Function: Cy_USBHS_Cal_InitHsPhy()
 * Description: This function Initialize PHY in high speed mode.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_InitHsPhy (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
    /*
     * TBD:
     * 1. Find out anything needs to UNDO related to FS mode.
     * 2. Find out ECO Enable CONFIG REGISTER and wait for clock to stablize.
     */
    pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_SWITCH_EN;
    pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_ENABLE_LV;
    while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_HS_VCCD));
    pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_HS_VCCD);


    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
    /* wait for 8us  */
    Cy_SysLib_DelayUs(8);

    pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;


    while (!(pPhyBase->INTR0 & (USBHSPHY_INTR0_PLL_LOCK)));
    /* RW1C */
    pPhyBase->INTR0  |= USBHSPHY_INTR0_PLL_LOCK;
    Cy_SysLib_DelayUs(1);
    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD;
    pPhyBase->CDR_CONTROL |=  USBHSPHY_CDR_CONTROL_CDR_ENABLE;
    return;
}

/*
 * Function: Cy_USBHS_Cal_CommonInitPhy()
 * Description: This function Initialize common registers which common to
 *              full and high speed mode.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: None
 */
void
Cy_USBHS_Cal_CommonInitPhy (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

    /* Current reference initialization */
    pPhyBase->IREFGEN_CONTROL |= USBHSPHY_IREFGEN_CONTROL_ENABLE_LV;
    /* Voltage reference initialization */
    pPhyBase->VREFGEN_CONTROL |= USBHSPHY_VREFGEN_CONTROL_ENABLE_LV;
    /* wait for 20us  */
    Cy_SysLib_DelayUs(20);

    /*
     * During this time interrupts are not enabled so keep checking
     * required bit in interrupt register as polling mechanism.
     */
    pPhyBase->REG_2P5_CONTROL |= USBHSPHY_REG_2P5_CONTROL_ENABLE_LV;

    /* check for enable vccd interrupt here only */
    while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_VCCD));
    /* Clear interrupt bit by writing "1" */
    pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_VCCD);

    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_VCCD;
    /* wait for 40us  */
    Cy_SysLib_DelayUs(40);
    return;
}

/* Function: Cy_USBHS_Cal_InitUsbController()
 * Description: This function takes care of CAL layer initialization which
 *              include FS and HS PHY.
 * Parameter: cal layer context structure.
 * return: none
 */
void
Cy_USBHS_Cal_InitUsbController (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                void *pUsbdCtxt,
                                cy_usb_cal_msg_callback_t callBackFunc)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;
    /*
     * This function assumes that pCalBase and pPhyBase allready
     * populated by initial caller.
     */
    pCalCtxt->pUsbdCtxt = pUsbdCtxt;
    pCalCtxt->msgCb = callBackFunc;


    pCalBase->POWER |= USBHSDEV_POWER_RESETN;
    Cy_SysLib_DelayUs(1);
    pCalBase->POWER |= USBHSDEV_POWER_VBUS_VALID;
    pCalBase->POWER &= ~(USBHSDEV_POWER_EPM_DCG_ENABLE);


    /*
     * Clear the CONT_TO_DATA bit by default. This has to be set only when we
     * are handling the EP0-OUT transfers.
     */
    pCalBase->DEV_CS &= (~(USBHSDEV_DEV_CS_CONT_TO_DATA_Msk));

    /* Enable setting to reduce round-trip latency by passing flop in EPM. */
    pCalBase->LEGACY_FEATURE_ENABLE = USBHSDEV_LEGFEAT_BYPASS_FLOP_EN;

    /*
     * Initialize FS and HS PHY together and then disable HS clock
     * if it is not required.
     */
#if BL_DEBUG
    DBG_HSCAL_TRACE("Calling PhyCommonInit\r\n");
#endif
    Cy_USBHS_Cal_PhyCommonInit(pCalCtxt);
    Cy_USBHS_Cal_FsHsModePhyInit(pCalCtxt);

#if BL_DEBUG
    DBG_HSCAL_TRACE("Cy_USBHS_Cal_Init  <<\r\n");
#endif

#if OUT_SLP_ZLP_DELAYED_HANDLER
    /* Enable DMA trigger generation for Short Packets. */
    pCalBase->EPM_CS = USBHSDEV_EPM_CS_ALLOW_TRIG_ON_SLP_Msk;
#endif /* OUT_SLP_ZLP_DELAYED_HANDLER */
    return;
}

/*******************************************************************************
* Function name: Cy_USBHS_Cal_FsHsModePhyInit
****************************************************************************//**
*
* Initializes high speed PHY.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_FsHsModePhyInit (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
  USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;
  bool pllLocked = 0;
  pCalCtxt->clkSrcType = USB2REF_CLK_SRC_NA;
  USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

#if BL_DEBUG
  DBG_HSCAL_INFO("Cy_USBHS_Cal_FsHsModePhyInit >>\r\n");
#endif
  /* TBD: Find out anything needs to be done to UNDO FS mode settings? */

  pPhyBase->REG_1P1_CONTROL |= USBHSPHY_REG_1P1_CONTROL_ENABLE_LV;

  /* wait for VCCD interrupt and then clear the interrupt*/
#if BL_DEBUG
  DBG_HSCAL_INFO("before ENABLE_HS_VCCD loop\r\n");
#endif
  while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_HS_VCCD)) {
      Cy_SysLib_DelayUs(1);
#if BL_DEBUG
      DBG_HSCAL_INFO("Inside ENABLE_HS_VCCD loop. \r\n");
#endif
  }
  pPhyBase->INTR0 =  (USBHSPHY_INTR0_ENABLE_HS_VCCD);

  /* If ECO has already been enabled and is showing OK status, go ahead with existing config. */
  if (Cy_SysClk_EcoGetStatus() == CY_SYSCLK_ECOSTAT_STABLE)
    {
      /* Enable PLL supply, wait for at least 8 us and then enable PLL. */
      pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
      Cy_SysLib_DelayUs(10);
      pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;

      DBG_HSCAL_INFO("ECO: Waiting for USBHS PLL Lock\r\n");
      pllLocked = Cy_USBHS_WaitForPllLock(pCalCtxt, 0);
      if (pllLocked) {
	  pCalCtxt->clkSrcType = USB2REF_CLK_SRC_ECO;
      } else {
	  /* Disable PLL and ECO. */
	  pPhyBase->PLL_CONTROL_1 &= ~(USBHSPHY_PLL_CONTROL_1_SUPPLY_EN | USBHSPHY_PLL_CONTROL_1_PLL_EN);
	  Cy_SysClk_EcoDisable();
      }
    }

  /* USBHS PLL is not locked, check if clock is coming from the ECO */
  if(!pllLocked)
    {
      /* Check whether the active clock input is from EXT_CLK.If not, fallback to ECO input */
      Cy_USBHS_ConfigExtClkPin(TRUE);
      pCalBase->POWER |= (USBHSDEV_POWER_REFCLK_SEL_Msk);

      /* Enable PLL supply, wait for at least 8 us and then enable PLL. */
      pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
      /* wait for 8us  */
      Cy_SysLib_DelayUs(10);
      pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;

      DBG_HSCAL_INFO("EXT_CLK:Waiting for USBHS PLL Lock\r\n");
      pllLocked = Cy_USBHS_WaitForPllLock(pCalCtxt, CY_USBHS_PLL_LOCK_TIMEOUT_MS);

      /* USBHS PLL is not locked, check if clock is coming from the ECO */
      if (!pllLocked) {
#if BL_DEBUG
	  DBG_HSCAL_INFO("Enabling ECO\r\n");
#endif
	  /* Disable settings for external clock */
	  pCalBase->POWER &= ~(USBHSDEV_POWER_REFCLK_SEL_Msk);
	  Cy_USBHS_ConfigExtClkPin(FALSE);

	  /*
	   * Temporary for QB part support: Apply a typical ECO trim register value if the WDTRIM field
	   * is set greater than 1 (75 mV).
	   */
	  if ((SRSS->CLK_TRIM_ECO_CTL & SRSS_CLK_TRIM_ECO_CTL_WDTRIM_Msk) > 1UL) {
	      SRSS->CLK_TRIM_ECO_CTL = 0x2127F0UL;
	  }

	  /* Enable the ECO and wait for ECO stability for a maximum of 10 ms. */
	  if (Cy_SysClk_EcoEnable(10000UL) != CY_SYSCLK_SUCCESS) {
#if BL_DEBUG
	      DBG_HSCAL_INFO("ECO stability timeout\r\n");
#endif
	      return;
	  }

	  /*TODO: Check whether we should timeout here or wait infinitely?*/
          /* Enable PLL supply, wait for at least 8 us and then enable PLL. */
          pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_SUPPLY_EN;
          Cy_SysLib_DelayUs(10);
          pPhyBase->PLL_CONTROL_1 |= USBHSPHY_PLL_CONTROL_1_PLL_EN;

	  DBG_HSCAL_INFO("ECO:Waiting for USBHS PLL Lock\r\n");
	  pllLocked = Cy_USBHS_WaitForPllLock(pCalCtxt, 0);
	  if(pllLocked)
	    {
	      pCalCtxt->clkSrcType = USB2REF_CLK_SRC_ECO;
	    }
      }
      else
	{
	  DBG_HSCAL_INFO("EXT_CLK:PLL lock done\r\n");
	  pCalCtxt->clkSrcType = USB2REF_CLK_SRC_EXT_CLK;
	}
    }
      /* wait for 1 us */
      Cy_SysLib_DelayUs(1);
      pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_HS_VCCD;
      pPhyBase->CDR_CONTROL |=  USBHSPHY_CDR_CONTROL_CDR_ENABLE;
      /* Update PHY TX settings for initial connection.
       * AFE_CONTROL_1.HS_AMP_SEL = 5
       */
      pPhyBase->AFE_CONTROL_1 = (
              (pPhyBase->AFE_CONTROL_1 & ~(USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Msk | USBHSPHY_AFE_CONTROL_1_HS_PREE_SEL_Msk)) |
              (0x05UL << USBHSPHY_AFE_CONTROL_1_HS_AMP_SEL_Pos));

      /* PHY RX update for compliance. */
      pPhyBase->VREFGEN_CONTROL = (
              (pPhyBase->VREFGEN_CONTROL & ~USBHSPHY_VREFGEN_CONTROL_TED_SEL_0_Msk) |
              (0x07UL << USBHSPHY_VREFGEN_CONTROL_TED_SEL_0_Pos));
#if BL_DEBUG
      DBG_HSCAL_INFO("Clock source type = %d\r\n",pCalCtxt->clkSrcType);
      DBG_HSCAL_INFO("Cy_USBHS_Cal_FsHsModePhyInit <<\r\n");
#endif
      return;
    }    /* end of function */


/*******************************************************************************
* Function name: Cy_USBHS_Cal_PhyCommonInit
****************************************************************************//**
*
* Initializes registers common to full and high speed PHY configuration.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* None.
*
*******************************************************************************/
void
Cy_USBHS_Cal_PhyCommonInit (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSPHY_Type  *pPhyBase = pCalCtxt->pPhyBase;

#if BL_DEBUG
    DBG_HSCAL_INFO("Cy_USBHS_Cal_PhyCommonInit >>\r\n");
#endif

    /* Clear all interrupts */
    pPhyBase->INTR0 = 0xFFFFFFFF;

    /* Current reference initialization */
    pPhyBase->IREFGEN_CONTROL |= USBHSPHY_IREFGEN_CONTROL_ENABLE_LV;
    /* Voltage reference initialization */
    pPhyBase->VREFGEN_CONTROL |= USBHSPHY_VREFGEN_CONTROL_ENABLE_LV;
    /* wait for 20us  */
    Cy_SysLib_DelayUs(20);

    /*
     * During this time interrupts are not enabled so keep checking
     * required bit in interrupt register as polling mechanism.
     */
    pPhyBase->REG_2P5_CONTROL |= USBHSPHY_REG_2P5_CONTROL_ENABLE_LV;

#if BL_DEBUG
    DBG_HSCAL_INFO("before USBHSPHY_INTR0_ENABLE_VCCD loop\r\n");
#endif
    /* check for enable vccd interrupt here only */
    while (!(pPhyBase->INTR0 & USBHSPHY_INTR0_ENABLE_VCCD)) {
        Cy_SysLib_DelayUs(1);
#if BL_DEBUG
        DBG_HSCAL_INFO("Inside ENABLE_VCCD loop. \r\n");
#endif
    }
    /* Clear interrupt bit by writing "1" */
    pPhyBase->INTR0 |=  (USBHSPHY_INTR0_ENABLE_VCCD);

    pPhyBase->AFE_CONTROL_1 |= USBHSPHY_AFE_CONTROL_1_CPU_DELAY_ENABLE_VCCD;
    /* wait for 40us  */
    Cy_SysLib_DelayUs(40);
#if BL_DEBUG
    DBG_HSCAL_INFO("Cy_USBHS_Cal_PhyCommonInit <<\r\n");
#endif
    return;
}


/* Function: Cy_USBHS_Cal_ResetHsController()
 * Description: This function reset HS controller.
 * Parameter: cy_stc_usb_cal_ctxt_t.
 * return: none
 */
void
Cy_USBHS_Cal_ResetHsController (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase = pCalCtxt->pCalBase;

	/*
	 * first clear RESETN bit so that hw will go in reset mode
	 * then bringout hw from reset by writting 1 at resetn bit.
	 */
    pCalBase->POWER &= (~(USBHSDEV_POWER_RESETN | USBHSDEV_POWER_VBUS_VALID | USBHSDEV_POWER_REFCLK_SEL_Msk));

    pCalBase->POWER |= (USBHSDEV_POWER_RESETN | USBHSDEV_POWER_VBUS_VALID);

    pCalBase->POWER &= ~(USBHSDEV_POWER_EPM_DCG_ENABLE);
    pCalBase->DEV_CS &= (~(USBHSDEV_DEV_CS_CONT_TO_DATA_Msk));

    /* Enable DMA trigger generation for Short Packets. */
    pCalBase->EPM_CS = (USBHSDEV_EPM_CS_ALLOW_TRIG_ON_SLP_Msk);

    /* TBD: If anything required at PHY level then need to include here. */

    return;
}


/* Function: Cy_USBHS_Cal_SendEgressZLP()
 * Description: This function triggers sending of a ZLP on an Egress endpoint.
 * Parameter: pCalCtxt, endpNumber
 * return: none
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SendEgressZLP (cy_stc_usb_cal_ctxt_t *pCalCtxt,
        uint32_t endpNumber)
{
    USBHSDEV_Type  *pCalBase;
    cy_en_usb_cal_ret_code_t retCode;

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pCalBase)) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase = pCalCtxt->pCalBase;
    if (endpNumber >= CY_USB_MAX_ENDP_NUMBER) {
        return(CY_USB_CAL_STATUS_BAD_PARAM);
    }

    retCode = CY_USB_CAL_STATUS_SUCCESS;

    /* We can set the ZLP request bit immediately. EPM takes care of sending ZLP only after any data that is
     * in the buffer is gone. Application will need to ensure that it waits for ZLP completion before next
     * packet is queued.
     */
    pCalBase->DEV_EP_EGRS_REQ = (1UL << endpNumber);

    return retCode;
}


/*
 * Function: Cy_USBHS_Cal_LpmSetClearNYET()
 * Description: This function either set or clear NYET bit for LPM response.
 * Parameter: cy_stc_usb_cal_ctxt_t, setClear
 * return: return code
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_LpmSetClearNYET (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (setClear) {
        /* Set CLR bit and set NYET bit */
        pCalBase->DEV_LPM_ATTR |= (USBHSDEV_DEV_LPM_ATTR_NYET);
    } else {
        pCalBase->DEV_LPM_ATTR &= (~USBHSDEV_DEV_LPM_ATTR_NYET);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* End of function   */

/*
 * Function: Cy_USBHS_Cal_GetLinkActive()
 * Description: This function makes sure that the USB 2.x link gets into the L0 state if it is in L1.
 * Parameter: cy_stc_usb_cal_ctxt_t
 * return: return code
 */
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_GetLinkActive(
        cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    if ((NULL == pCalCtxt) || (NULL == pCalCtxt->pCalBase)) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    pCalBase = pCalCtxt->pCalBase;

    /* If link is in L1 state, initiate a remote wakeup. */
    if ((pCalBase->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0) {
        Cy_USBHS_Cal_DevInitiateddResumeL1Sleep(pCalCtxt, 0x05DCUL);
#if BL_DEBUG
        DBG_HSCAL_INFO("L1Wake\r\n");
#endif
    }

    return(CY_USB_CAL_STATUS_SUCCESS);
}

/*
 * Function: Cy_USBHS_Cal_SetControllerSpeed()
 * Description: This function set controller speed to FS or HS. If explicitly
 *              FS is not mentioned then function will set speed to HS.
 * Parameter: cy_stc_usb_cal_ctxt_t and speed.
 * return: cy_en_usb_cal_ret_code_t
 */
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_SetControllerSpeed (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                 cy_en_usb_speed_t speed)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }

    if (CY_USBD_USB_DEV_FS == speed) {
        pCalBase->DEV_PWR_CS |= USBHSDEV_DEV_PWR_CS_FORCE_FS;
    } else {
        pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_FORCE_FS);
    }
    return(CY_USB_CAL_STATUS_SUCCESS);
}   /* end of function. */

/*******************************************************************************
* Function name: Cy_USBHS_Cal_HandleReset
****************************************************************************//**
*
* Some of controller register will be brought to default state.
*
* \param pCalCtxt
* CAL layer context pointer.
*
* \return
* CY_USB_CAL_STATUS_SUCCESS if the operation is successful.
* CY_USB_CAL_STATUS_CAL_BASE_NULL if register base pointer is NULL.
*
*******************************************************************************/
cy_en_usb_cal_ret_code_t
Cy_USBHS_Cal_HandleReset (cy_stc_usb_cal_ctxt_t *pCalCtxt)
{
    USBHSDEV_Type  *pCalBase;

    pCalBase = pCalCtxt->pCalBase;
    if (NULL == pCalBase) {
        return(CY_USB_CAL_STATUS_CAL_BASE_NULL);
    }
    pCalBase->DEV_PWR_CS &= (~USBHSDEV_DEV_PWR_CS_DEV_SUSPEND_Msk);

    return(CY_USB_CAL_STATUS_SUCCESS);
}

