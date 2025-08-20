/***************************************************************************//**
* \file cy_usbhs_cal_drv.h
* \version 1.0
*
* \brief This file contains all the declaration related to CAL layer for USB 2.x controller.
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
*/

#if !defined(CY_USBHS_CAL_DRV_H)
#define CY_USBHS_CAL_DRV_H

#include <stdbool.h>
/* Control bit to be set in LEGACY_FEATURE_EN register to reduce round trip latency. */
#define USBHSDEV_LEGFEAT_BYPASS_FLOP_EN (0x0008UL)

#define CY_USBHS_PLL_LOCK_TIMEOUT_MS (10) /*10 ms*/
#define CY_EXTERNAL_CLK_PIN     (P5_1_PIN) /*XTALIN: P5_0, XTALOUT: P5_1*/


/* Enumeration of possible clock sources for USB2 PLL REFCLK */
typedef enum cy_en_usb2_ref_clk_src_t
{
    USB2REF_CLK_SRC_ECO     = 0x00,
    USB2REF_CLK_SRC_EXT_CLK = 0x01,
    USB2REF_CLK_SRC_NA      = 0xFF
} cy_en_usb2_ref_clk_src_t;

typedef enum cy_en_usbhs_cal_test_mode_ {
    CY_USBHS_CAL_TEST_MODE_NORMAL=0,
    CY_USBHS_CAL_TEST_MODE_TEST_J,
    CY_USBHS_CAL_TEST_MODE_TEST_K,
    CY_USBHS_CAL_TEST_MODE_TEST_SE0_NAK,
    CY_USBHS_CAL_TEST_MODE_TEST_PACKET,
    CY_USBHS_CAL_TEST_MODE_TEST_RESERVED
}cy_en_usbhs_cal_test_mode_t;


/* main data structure which represent CAL layer */
typedef struct cy_stc_usb_cal_ctxt_
{
    USBHSDEV_Type  *pCalBase;
    USBHSPHY_Type  *pPhyBase;
    void *pUsbdCtxt;
    void *queueCtxt;
    cy_usb_cal_msg_callback_t msgCb;
    cy_en_usb2_ref_clk_src_t clkSrcType;
}cy_stc_usb_cal_ctxt_t;

/* 
 * List of API provided by CAL layer. While calling API, USBD layer has to pass
 * cal_ctxt alongwith other para meters.
 */
void Cy_USBHS_Cal_PhyCommonInit(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_FsHsModePhyInit(cy_stc_usb_cal_ctxt_t *pCalCtxt);
/* Controller and PHY related initialization function */
void Cy_USBHS_Cal_InitUsbController(cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                    void *pUsbdCtxt, 
                                    cy_usb_cal_msg_callback_t callBackFunc);
void Cy_USBHS_Cal_InitUsbControllerHSMode (cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_InitHsPhy(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_CommonInitPhy(cy_stc_usb_cal_ctxt_t *pCalCtxt);


/* LPM Related Functions  */
void Cy_USBHS_Cal_HsHandleL1Sleep(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_HsHandleL1Wakeup(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_DevInitiateddResumeL1Sleep(cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                                            uint32_t duration);
void Cy_USBHS_Cal_DevInitiateddResumeL2Sleep(cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                                            uint32_t duration);
void Cy_USBHS_Cal_HsHandleSuspend(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_FsHandleSuspend(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_HsHandleResume(cy_stc_usb_cal_ctxt_t *pCalCtxt);
void Cy_USBHS_Cal_FsHandleResume(cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EnableReqDevCtrlIntr
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_DisableAllDevCtrlIntr
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_ClearAllDevCtrlIntr
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_UpdateXferCount
                                           (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                            uint32_t endpNumber,
                                            cy_en_usb_endp_dir_t endpDirection,
                                            uint32_t xferCount);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_ConnUsbPins
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_DisconUsbPins
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);

bool Cy_USBHS_Cal_GetRemoteWakeupStatus(cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SignalRemotWakup
                                              (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                               bool startEndSignal);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SetTestMode
                                        (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                         cy_en_usbhs_cal_test_mode_t testMode);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_GetDevAddress
                                            (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                             uint8_t *pDevAddr);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EndpSetClearNak
                                         (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                          uint32_t endpNumber,
                                          cy_en_usb_endp_dir_t endpDirection,
                                          bool setClear);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SetClearNakAll
                                              (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                               bool setClear);
bool Cy_USBHS_Cal_EndpIsNakNrdySet
                                        (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                         uint32_t endpNumber,
                                         cy_en_usb_endp_dir_t endpDirection);

bool Cy_USBHS_Cal_EndpIsStallSet(cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                 uint32_t endpNumber,
                                 cy_en_usb_endp_dir_t endpDirection);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EndpSetClearStall
                                    (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                     uint32_t endpNumber,
                                     cy_en_usb_endp_dir_t endpDirection,
                                     bool setClear);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EnableEndp
                                            (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                             uint32_t endpNumber,
                                             cy_en_usb_endp_dir_t endpDirection,
                                             bool enable);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_UpdateEpIntrMask
                                (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                 uint32_t endpNumber,
                                 cy_en_usb_endp_dir_t endpDirection,
                                 uint32_t epIntrMask,
                                 bool setClear);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_ClearZlpSlpIntrEnableMask
                                           (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                            uint32_t endpNumber,
                                            cy_en_usb_endp_dir_t endpDirection,
                                            bool zlpSlp);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_FlushEndp
                                          (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                           uint32_t endpNumber,
                                           cy_en_usb_endp_dir_t endpDirection);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_FlushAllEndp
                                          (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                           cy_en_usb_endp_dir_t endpDirection);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EndpConfig
                                        (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                         cy_stc_usb_endp_config_t configParam);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SendAckSetupDataStatusStage
                                             (cy_stc_usb_cal_ctxt_t *pCalCtxt);
bool Cy_USBHS_Cal_IntrHandler (cy_stc_usb_cal_ctxt_t  *pCalCtxt);

void Cy_USBHS_Cal_ResetHsController(cy_stc_usb_cal_ctxt_t *pCalCtxt);
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SendEgressZLP
                        (cy_stc_usb_cal_ctxt_t *pCalCtxt, uint32_t endpNumber);
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_LpmSetClearNYET
                              (cy_stc_usb_cal_ctxt_t *pCalCtxt, bool setClear);
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_GetLinkActive
                              (cy_stc_usb_cal_ctxt_t *pCalCtxt);
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_HandleCtrlOutSlp (cy_stc_usb_cal_ctxt_t *pCalCtxt);
cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_EnableCtrlSlpIntr (cy_stc_usb_cal_ctxt_t *pCalCtxt);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_SetControllerSpeed
                                              (cy_stc_usb_cal_ctxt_t *pCalCtxt,
                                               cy_en_usb_speed_t speed);

cy_en_usb_cal_ret_code_t Cy_USBHS_Cal_HandleReset(cy_stc_usb_cal_ctxt_t *pCalCtxt);

#endif /* (!defined(CY_USBHS_CAL_DRV_H)) */

