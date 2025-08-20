/***************************************************************************//**
 * \file main.c
 * \version 1.0
 *
 * Bootloader implementation for CY FX (FX20, FX10, FX5N and FX5) devices.
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
#include "cy_app_defines.h"
#include <string.h>

#define FX_BL_BUILD_NUMBER              (0xCE000001)

#ifdef FX_BL_VERSION
const char __attribute__ ((section(".bl_version"), used)) __attribute__ ((aligned (4))) fx_bl_vers_str[] = FX_BL_VERSION;
#else
const char __attribute__ ((section(".bl_version"), used)) __attribute__ ((aligned (4))) fx_bl_vers_str[] = "00000000";
#endif /* FX_BL_VERSION */

const uint32_t __attribute__ ((section(".bl_version"), used)) fx_bl_build_number = FX_BL_BUILD_NUMBER;



cy_stc_device_info_t deviceInfo;
cy_stc_usb_usbd_ctxt_t usbdCtxt;
cy_stc_usb_cal_ctxt_t  calCtxt;
DMAC_Type *pCpuDmacBase;
DW_Type *pCpuDw0Base, *pCpuDw1Base;

static uint8_t Ep0TestBuffer[4096U] __attribute__ ((aligned (4)));

__attribute__ ((section(".bl_stat"))) volatile uint8_t isBootActive[8];

const uint8_t bootMode[8] = "BOOTMODE";
const uint8_t appMode[8] = "APPLMODE";
volatile cy_en_dfu_state_t dfuState = DFU_APP_IDLE;
cy_stc_dfu_get_status_response_t getStatusResponse;
uint32_t downloadDone = 0;
bool selfOverwriteDetected = false;
uint32_t hfclkFreq = BCLK__BUS_CLK__HZ;


#if ENABLE_FLASH_PROGRAMMER
cy_en_smif_slave_select_t glSlaveSelect = CY_SMIF_SLAVE_SELECT_0;
#endif /* ENABLE_FLASH_PROGRAMMER */



#if BL_DEBUG
/* Select SCB interface used for UART based logging. */
#define LOGGING_SCB             (SCB1)
#define LOGGING_SCB_IDX         (1)
#define DEBUG_LEVEL             (3u)

/* Debug log related initilization */
#define LOGBUF_SIZE (1024u)
uint8_t logBuff[LOGBUF_SIZE];

cy_stc_debug_config_t dbgCfg = {
    .pBuffer         = logBuff,
    .traceLvl        = DEBUG_LEVEL,
    .bufSize         = LOGBUF_SIZE,
#if ENABLE_USBFS_DEBUG
    .dbgIntfce       = CY_DEBUG_INTFCE_USBFS_CDC,
#else
    .dbgIntfce       = CY_DEBUG_INTFCE_UART_SCB1,
#endif
    .printNow        = true
};
#endif /* BL_DEBUG */



extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxLangString[];
extern const uint8_t CyFxMfgString[];
extern const uint8_t CyFxProdString[];
extern const uint8_t CyFxSerialNoString[];
extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function called by USBD when a USB bus reset is detected.
 * \param pAppCtxt Application context pointer (unused).
 * \param pUsbdCtxt USB device context pointer.
 * \param pMsg USB message context pointer (unused).
 * \retval None
 */
void Cy_USB_AppBusResetCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        cy_stc_usb_cal_msg_t *pMsg)
{
    if(dfuState == DFU_MANIFEST_WAIT_RESET)
    {
        Cy_SysLib_DelayUs(1000);
        Cy_USBD_DisconnectDevice (pUsbdCtxt);
    }

    if(dfuState == DFU_APP_DETACH)
    {
        Cy_SysLib_DelayUs(1000);
        Cy_USBD_DisconnectDevice (pUsbdCtxt);
        CyFxUSBHSConfigDscr[16] = 0x02; //Change the interface protocol code
        Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
        Cy_SysLib_DelayUs(1000);
        Cy_USBD_ConnectDevice(&usbdCtxt, CY_USBD_USB_DEV_HS);
        dfuState = DFU_IDLE;
    }
}

static cy_en_usbd_ret_code_t Cy_DFU_SendStatusResponse(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        uint8_t status, uint32_t pollTimeout, uint8_t state, uint8_t strIndex, uint16_t len)
{ 
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    getStatusResponse.bStatus = status;
    getStatusResponse.bwPollTimeout[0] = CY_USB_DWORD_GET_BYTE0(pollTimeout);
    getStatusResponse.bwPollTimeout[1] = CY_USB_DWORD_GET_BYTE1(pollTimeout);
    getStatusResponse.bwPollTimeout[2] = CY_USB_DWORD_GET_BYTE2(pollTimeout);
    getStatusResponse.bState = state;
    getStatusResponse.iString = strIndex;
    retStatus =  Cy_USB_USBD_SendEndp0DataHs (pUsbdCtxt,
            (uint8_t *)&getStatusResponse, len);
    return retStatus;
}


/**
 * \name Cy_DFU_StateMachine
 * \brief Implements the DFU state machine for handling DFU requests.
 * \param pUsbdCtxt USB device context pointer.
 * \param bRequest DFU request code.
 * \param wLength Length of data for the request.
 * \retval None
 */
static void Cy_DFU_StateMachine(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t bRequest, uint16_t wLength)
{
    uint32_t i;
    static uint32_t address = (uint32_t)(JUMP_ADDRESS);
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    bool reqHandled  = false;
    uint32_t statusPollTimeout = 0;
        
    /* Prevent compiler warning. */
    (void)retStatus;

    if ((dfuState == DFU_DNBUSY) || (dfuState == DFU_MANIFEST_WAIT_RESET))
    {
        /* DUT should not respond to any command while in DNBUSY/WAIT_RESET state */
        reqHandled = false;
    }
    else if((bRequest == DFU_REQ_DETACH) && (dfuState == DFU_APP_IDLE))
    {
        dfuState = DFU_APP_DETACH;
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        Cy_SysLib_DelayUs(1000);
        Cy_USBD_DisconnectDevice (pUsbdCtxt);
        CyFxUSBHSConfigDscr[16] = 0x02; //Change the interface protocol code
        Cy_USBD_SetDscr(pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
        Cy_SysLib_DelayUs(1000);
        Cy_USBD_ConnectDevice(&usbdCtxt, CY_USBD_USB_DEV_HS);
        dfuState = DFU_IDLE;
        reqHandled = true;
    }
    else if(bRequest == DFU_REQ_GETSTATE)
    {
        retStatus = Cy_USB_USBD_SendEndp0DataHs (pUsbdCtxt, (uint8_t *)&dfuState, 1);
        reqHandled = true;
    }
    else if (bRequest == DFU_REQ_GETSTATUS)
    { 
        switch(dfuState)
        {
            case DFU_DNLOAD_SYNC:
                if(downloadDone)
                {
                    dfuState = DFU_DNLOAD_IDLE;
                }
                else
                {
                    dfuState = DFU_DNBUSY;
                }
                /* no break */

            case DFU_IDLE:
            case DFU_UPLOAD_IDLE:
                statusPollTimeout = 0x64;
                break;

            case DFU_MANIFEST:
                dfuState = DFU_MANIFEST_WAIT_RESET;
                statusPollTimeout = 0x64;
                break;

            case DFU_DNLOAD_IDLE:
                if(!downloadDone)
                {
                    dfuState = DFU_DNBUSY;
                }
                statusPollTimeout = 0x1770;
                break;

            case DFU_MANIFEST_SYNC:
                dfuState = DFU_MANIFEST;
                /* no break */
            case DFU_APP_IDLE:
            case DFU_APP_DETACH:
            case DFU_ERROR:
                statusPollTimeout = 0x1770;
                break;
            default: break;
        }
        Cy_DFU_SendStatusResponse(pUsbdCtxt, 0x00, statusPollTimeout, dfuState, 0x0, wLength);
        reqHandled = true;
    }
    else if (bRequest == DFU_REQ_DNLOAD)
    { 
        if ((dfuState == DFU_IDLE) || (dfuState == DFU_DNLOAD_IDLE))
        {
            if(wLength > 0)
            {
                dfuState = DFU_DNLOAD_SYNC;
                retStatus = Cy_USB_USBD_RecvEndp0DataHs(pUsbdCtxt, ((uint8_t *)(Ep0TestBuffer)), wLength);
                downloadDone = 0;
                for(i = 0; i < wLength; i = i + 512)
                {
                    Cy_Flash_EraseRow(address);
                    Cy_Flash_ProgramRow(address, (uint32_t *)(Ep0TestBuffer + i));
                    address += 512;
                }
                downloadDone = 1;
            }
            else
            {
                if(dfuState == DFU_IDLE)
                {
                    dfuState = DFU_ERROR;
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
                }
                else
                {
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                    dfuState = DFU_MANIFEST_SYNC;
                }
            }

            reqHandled = true;
        }
    }
    else if (((bRequest == DFU_REQ_CLRSTATUS) && (dfuState == DFU_ERROR)) ||
            ((bRequest == DFU_REQ_ABORT)  && (dfuState == DFU_UPLOAD_IDLE || dfuState == DFU_IDLE)))
    { 
        dfuState = DFU_IDLE;
        reqHandled = true;
    }

    /* Unhandled command */
    if(!reqHandled)
    {
        switch (dfuState)
        {
            case DFU_DNLOAD_IDLE:
            case DFU_MANIFEST_SYNC:
            case DFU_UPLOAD_IDLE:
            case DFU_ERROR:
                dfuState = DFU_ERROR;
                break;

            case DFU_APP_IDLE:
            case DFU_APP_DETACH:
            case DFU_IDLE:
            case DFU_DNLOAD_SYNC:
            case DFU_DNBUSY:
                dfuState = DFU_APP_IDLE;
                break;
            case DFU_MANIFEST:
            case DFU_MANIFEST_WAIT_RESET:
                /* No action needed in MANIFEST states for unhandled commands */
                return;
        }
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
}

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback for USB setup requests.
 * \param pAppCtxt Application context pointer
 * \param pUsbdCtxt USB device context pointer.
 * \param pMsg USB message context pointer
 * \retval None
 */
void Cy_USB_AppSetupCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
        cy_stc_usb_cal_msg_t *pMsg)
{
    bool isReqHandled = false;
    uint32_t address = 0;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    uint8_t bRequest, bReqType;
    uint8_t bTarget;
    uint16_t wValue, wIndex, wLength;

#if ENABLE_FLASH_PROGRAMMER
    uint8_t fx10FlashProg[] = {'F','X','1','0','P','R','O','G'};
    uint32_t qspiAddress = 0;
    uint32_t sector = 0;
    uint32_t numPages = (CY_APP_SPI_MAX_USB_TRANSFER_SIZE) / (CY_APP_SPI_FLASH_PAGE_SIZE);
    cy_en_smif_status_t qspiStatus = CY_SMIF_SUCCESS;
    uint8_t busyStat = 0;
#endif /* ENABLE_FLASH_PROGRAMMER */

    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_MASK);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    /* Handle Microsoft OS String Descriptor request. */
    if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
            (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
            (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE)))
    {
        /* Make sure we do not send more data than requested. */
        if (wLength > glOsString[0])
        {
            wLength = glOsString[0];
        }
        retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, glOsString, wLength);
        if(retStatus != CY_USBD_STATUS_SUCCESS)
        {
        }
        isReqHandled = true;
    }

    /* Handle OS Compatibility and OS Feature requests */
    if (bRequest == MS_VENDOR_CODE)
    {
        if (wIndex == 0x04)
        {
            if (wLength > glOsCompatibilityId[0])
            {
                wLength = glOsCompatibilityId[0];
            }
            retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, glOsCompatibilityId, wLength);
            if(retStatus != CY_USBD_STATUS_SUCCESS)
            {
            }
            isReqHandled = true;
        }
        else if (wIndex == 0x05)
        {
            if (wLength > glOsFeature[0])
            {
                wLength = glOsFeature[0];
            }
            retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, glOsFeature, wLength);
            if(retStatus != CY_USBD_STATUS_SUCCESS)
            {
            }
            isReqHandled = true;
        }
    }

    if((bReqType == DFU_BM_REQUEST_TYPE_0) || (bReqType == DFU_BM_REQUEST_TYPE_1))
    {
        Cy_DFU_StateMachine(pUsbdCtxt, bRequest, wLength);
        isReqHandled = true;
    }

    if (((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS)
            == CY_USB_CTRL_REQ_VENDOR)
    {
#if BL_DEBUG
        Cy_Debug_AddToLog(1, "BRequest = 0x%x\r\n",bRequest);
#endif
        /* Request to program the flash with the received data */
        if((bRequest == 0xA0) && (wLength != 0) && (!selfOverwriteDetected))
        {
            retStatus = Cy_USB_USBD_RecvEndp0DataHs(pUsbdCtxt, ((uint8_t *)Ep0TestBuffer), wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS)
            {
                address = (wIndex << 16) + wValue;
                /* Prevent self-overwrite */
                if(address >= FX3G2_BL_END_ADDRESS)
                {
                    selfOverwriteDetected = false;
                    Cy_Flash_EraseRow(address);
                    Cy_Flash_ProgramRow(address, (uint32_t *)Ep0TestBuffer);
                    isReqHandled = true;
                }
                else
                {
                    selfOverwriteDetected = true;
#if BL_DEBUG
                    Cy_Debug_AddToLog(1, "Self-overwrite detected at address 0x%x %d\r\n",address,selfOverwriteDetected);
#endif
                    isReqHandled = false;
                }
            }
        }

        if((bRequest == 0xA1) && (wLength != 0))
        {
            address = (wIndex << 16) + wValue;
            retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, ((uint8_t *)address), wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS)
            {
                isReqHandled = true;
            }
        }

        if((bRequest == 0xE0) && (wLength == 0))
        {
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            Cy_SysLib_Delay(1000);
            Cy_USBD_DisconnectDevice (&usbdCtxt);
            Cy_SysLib_Delay(1000);
            memcpy((uint8_t *)isBootActive, (uint8_t *)appMode, 8);
            NVIC_SystemReset();
        }
       
        /* Return internal flash size and silicon-ID */
        if(bRequest == 0xE4)
        {
            DBG_BLOAD_INFO("Flash Size = 0x%x, Silicon ID=0x%x\r\n",deviceInfo.flashSize, deviceInfo.siliconId);
            wLength = MIN(wLength, sizeof(cy_stc_device_info_t));
            retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, ((uint8_t *)&deviceInfo), wLength);
            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
            isReqHandled = (retStatus == CY_USBD_STATUS_SUCCESS);
        }

        if (bRequest == 0xE5)
        {
            wLength = MIN(wLength, 16);
            retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, (uint8_t *)fx_bl_vers_str, wLength);
            ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
            isReqHandled = (retStatus == CY_USBD_STATUS_SUCCESS);
        }

        /* Handle SPI vendor commands */
#if ENABLE_FLASH_PROGRAMMER
        if(Cy_SPI_ValidFlashDetected())
        {
            if (bRequest == FLASH_CMD_CHECK_SPI_SUPPORT)
            {
                retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, fx10FlashProg, sizeof(fx10FlashProg));
                ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
                isReqHandled = !retStatus;
#if BL_DEBUG
                if(!retStatus)
                {
                    Cy_Debug_AddToLog(1, "Check flash prog done\r\n");
                }
#endif
            }

            /* Check memory busy status */
            if (bRequest == FLASH_CMD_CHECK_STATUS)
            {
                busyStat = Cy_SPI_IsMemBusy(glSlaveSelect);
                retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt,&busyStat, 1);
                ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
                isReqHandled = !retStatus;
#if BL_DEBUG
                if(!retStatus)
                {
                    Cy_Debug_AddToLog(1, "Check status done (%d)\r\n",busyStat);
                }
#endif
            }

            /* Write to SPI flash */
            if ((bRequest == FLASH_CMD_FLASH_WRITE) && (wLength == CY_APP_SPI_MAX_USB_TRANSFER_SIZE))
            {
                qspiAddress = wIndex * CY_APP_SPI_MAX_USB_TRANSFER_SIZE;;
                retStatus = Cy_USB_USBD_RecvEndp0DataHs(pUsbdCtxt, Ep0TestBuffer, wLength);
                ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
                if (retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    qspiStatus = Cy_SPI_WriteOperation(qspiAddress, Ep0TestBuffer, wLength, numPages, glSlaveSelect);
                    isReqHandled = !qspiStatus;
#if BL_DEBUG
                    if(!qspiStatus)
                    {
                        Cy_Debug_AddToLog(1, "numPages = %d.Program %d->%d done\r\n",numPages, qspiAddress,qspiAddress+(CY_APP_SPI_MAX_USB_TRANSFER_SIZE-1));
                    }
#endif
                }
            }

            /* Read from SPI flash */
            if ((bRequest == FLASH_CMD_FLASH_READ) && (wLength == CY_APP_SPI_MAX_USB_TRANSFER_SIZE))
            {
                qspiAddress = wIndex * CY_APP_SPI_MAX_USB_TRANSFER_SIZE;
                qspiStatus = Cy_SPI_ReadOperation(qspiAddress, Ep0TestBuffer , wLength, glSlaveSelect);
#if BL_DEBUG
                if(!qspiStatus)
                {
                    Cy_Debug_AddToLog(1, "Read %d->%d done\r\n",qspiAddress, qspiAddress + wLength-1);
                }
#endif
                retStatus = Cy_USB_USBD_SendEndp0DataHs(pUsbdCtxt, Ep0TestBuffer, wLength);
                ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
                isReqHandled = !qspiStatus;
            }

            /* Sector Erase to SPI flash */
            if (bRequest == FLASH_CMD_FLASH_SECTOR_ERASE)
            {
                sector = wIndex & 0xFFFF;
                if(Cy_SPI_SectorErase (glSlaveSelect, sector, CY_APP_SPI_FLASH_ERASE_SIZE) == CY_SMIF_SUCCESS)
                {
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
#if BL_DEBUG
                    Cy_Debug_AddToLog(1, "Erase sector %d done\r\n",sector);
#endif
                    isReqHandled = true;
                }
            }
        }
        else
        {
#if BL_DEBUG
            Cy_Debug_AddToLog(1, "CFI param (QRY) not detected. Flash not supported\r\n");
#endif
        }
#endif /* ENABLE_FLASH_PROGRAMMER*/
    }

    /* If Request is not handled by the callback, Stall the command. */
    if(!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
}   /* end of function. */


/*****************************************************************************
 * Function Name: UsbHS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void UsbHS_ISR(void)
{
    Cy_USBHS_Cal_IntrHandler(&calCtxt);
}

/**
 * \name Cy_Bl_JumpToApplication
 * \brief Jumps to the user application if valid.
 * \retval None
 */
void Cy_Bl_JumpToApplication(void)
{
    void (*appResetHandler)(void) = (void (*)())((uint32_t)*(volatile uint32_t *)(JUMP_ADDRESS+4));

    /* Make sure ECO is disabled before transferring control to the application. */
    Cy_SysClk_EcoDisable();

#if BL_DEBUG
    /* Need to disable the debug interface. */
#if ENABLE_USBFS_DEBUG
    CyUsbFsCdc_Disable();
#else
    CY_SCB_UART_Disable(LOGGING_SCB);
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 0);
#endif /* ENABLE_USBFS_DEBUG */
#endif /* BL_DEBUG */

    CPUSS->CM0_VECTOR_TABLE_BASE = (uint32_t)JUMP_ADDRESS;

    __set_MSP((uint32_t)*(volatile uint32_t *)JUMP_ADDRESS);

    appResetHandler();
}


/**
 * \name Cy_Bl_AppValidCheck
 * \brief Checks if the application image is valid by verifying vector table and hash.
 * \param isFlash256K True if device has 256K flash, false for 512K.
 * \retval uint8_t 1 if valid, 0 otherwise.
 */
uint8_t Cy_Bl_AppValidCheck (bool isFlash256K)
{
    volatile uint32_t *pAppVectorTable = (volatile uint32_t *)JUMP_ADDRESS;
    uint32_t appSize;
    uint32_t hashAddr;
    uint8_t hashValue[32];
    uint8_t isAppValid = 0;

    if (isFlash256K) {
        appSize = FX3G2_LTD_FLASH_SIZE - FX3G2_BL_SIZE - FX3G2_FLASH_ROW_SIZE;
    } else {
        appSize = FX3G2_MAX_FLASH_SIZE - FX3G2_BL_SIZE - FX3G2_FLASH_ROW_SIZE;
    }

#if BL_DEBUG
    DBG_BLOAD_INFO("App size: %x\r\n", appSize);
#endif

    /*
     * Start with a basic format check on the application:
     * Check for valid stack pointer and reset vector entries in the vector table.
     */
    if (
            (FX3G2_BL_VECTOR_SPTR_VALID(pAppVectorTable[0])) &&
            (FX3G2_BL_VECTOR_START_VALID(pAppVectorTable[1]))
       ) {
        /*
         * Compute SHA256 checksum over the application binary and compare with the checksum stored
         * in the last flash row.
         */
        Cy_Crypto_Core_Enable(CRYPTO);
        Cy_Crypto_Core_V2_Sha(CRYPTO, (uint8_t const *)(JUMP_ADDRESS),
                appSize, hashValue, CY_CRYPTO_MODE_SHA256);
        Cy_Crypto_Core_Disable(CRYPTO);

        hashAddr = FX3G2_FLASH_BASE_ADDR + FX3G2_BL_SIZE + appSize;
        if ((memcmp((uint8_t *)hashAddr, hashValue, 32) == 0)) {
            isAppValid = true;
        } else {
#if BL_DEBUG
            DBG_BLOAD_ERR("App hash compare failed\r\n");
#endif
        }
    } else {
#if BL_DEBUG
        DBG_BLOAD_ERR("App Framing invalid: %x %x\r\n", pAppVectorTable[0], pAppVectorTable[1]);
#endif
    }

    return isAppValid;
}

/**
 * \name CheckIfBootModeRequested
 * \brief Checks if boot mode is requested via GPIO pin.
 * \retval bool true if boot mode is requested, false otherwise.
 */
static bool CheckIfBootModeRequested (void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    uint32_t i, val;

    /* Configure P13.0 pin as input to sense the boot mode request. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    Cy_GPIO_Pin_Init(P13_0_PORT, P13_0_PIN, &pinCfg);

    val = Cy_GPIO_Read(P13_0_PORT, P13_0_PIN);
    i = 0;

    /* Wait until we read the same value from the BOOTMODE pin for 10 times in a row. */
    while (i < 10) {
        Cy_SysLib_DelayUs(10);
        if (val == Cy_GPIO_Read(P13_0_PORT, P13_0_PIN)) {
            i++;
        } else {
            val = !val;
            i = 0;
        }
    }

    /* Disable input buffer on BOOTMODE pin before we return. */
    pinCfg.driveMode = CY_GPIO_DM_ANALOG;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    Cy_GPIO_Pin_Init(P13_0_PORT, P13_0_PIN, &pinCfg);

    return (val != 0);
}


cy_rslt_t cybsp_register_custom_sysclk_pm_callback()
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    return result;
}



/*****************************************************************************
 * Function Name: main(void)
 ******************************************************************************
 * Summary:
 *  Entry to the application.
 *
 * Parameters:
 *  void

 * Return:
 *  Does not return.
 *****************************************************************************/
int main (void)
{
    cy_stc_sysint_t intrCfg;
    uint8_t isAppValid;
    uint8_t isFlash256K = true;
    bool bootModeRqt = false;

    /* Initialize the PDL driver library and set the clock variables. */
    Cy_PDL_Init(&cy_deviceIpBlockCfgFX3G2);

    cybsp_init();

    /* Disable the watchdog reset function. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /* Enable interrupts. */
    __enable_irq ();

    bootModeRqt = CheckIfBootModeRequested();

#if BL_DEBUG
#if ENABLE_USBFS_DEBUG
    Cy_Debug_LogInit(&dbgCfg);
    Cy_SysLib_Delay(1000);
#else
    /* Initialize SCB for debug log output. */
    InitUart(LOGGING_SCB_IDX);
    Cy_Debug_LogInit(&dbgCfg);
#endif
    DBG_BLOAD_INFO("DEBUG INFRA ENABLED!!\r\n");
#endif /* BL_DEBUG */

    memset((void *)&usbdCtxt, 0, sizeof(cy_stc_usb_usbd_ctxt_t));
    memset((void *)&calCtxt, 0, sizeof(cy_stc_usb_cal_ctxt_t));

    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base  = ((DW_Type *)DW0_BASE);
    pCpuDw1Base  = ((DW_Type *)DW1_BASE);

    calCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    calCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;

#if ENABLE_JUMP_TO_APPLN
#if BL_DEBUG
    DBG_BLOAD_INFO("Checking AppValid\r\n");
#endif

    /* Find the size of flash memory supported by the controller. This could be either 512 KB or 256 KB. */
    if (((*(volatile uint32_t*)CPUSS_FLASH_SIZE_ADDR) & 0x3000U) == 0)
    {
        isFlash256K = false;
        deviceInfo.flashSize = CPUSS_FLASH_SIZE;
    }
    else
    {
        deviceInfo.flashSize = 256;
    }
    deviceInfo.siliconId = SFLASH_SILICON_ID;

    isAppValid = Cy_Bl_AppValidCheck (isFlash256K);
#if BL_DEBUG
    DBG_BLOAD_INFO("App valid=%d\r\n", isAppValid);
#endif

    /* Set BOOTMODE string in the application state array. */
    if (isAppValid == 0)
    {
        memcpy ((uint8_t*) isBootActive, (uint8_t*) bootMode, 8);
    }
    else
    {
        /* Transfer control to the application if BOOTMODE is not requested. */
        if ((!bootModeRqt)
                && (memcmp ((uint8_t*) isBootActive, (uint8_t*) bootMode, 8) != 0))
        {
#if ENABLE_FLASH_PROGRAMMER
            Cy_SPI_Stop (glSlaveSelect);
#endif /* ENABLE_FLASH_PROGRAMMER */

            /* This function will not return. */
            Cy_Bl_JumpToApplication ();
        }
        else
        {
#if BL_DEBUG
            DBG_BLOAD_INFO("Staying in Bootloader mode on APP request\r\n");
#endif
            memcpy ((uint8_t*) isBootActive, (uint8_t*) bootMode, 8);
        }
    }
#endif /* ENABLE_JUMP_TO_APPLN */

#if ENABLE_FLASH_PROGRAMMER
    /* Enable QSPI interface and check for presence of a supported flash memory device.
     * If this is present, we can enable the commands that read/write the external flash.
     */
    Cy_SPI_Start(glSlaveSelect);
#endif /* ENABLE_FLASH_PROGRAMMER */

    /* Register the ISR and enable the interrupt. */
    intrCfg.intrSrc      = NvicMux0_IRQn;
    intrCfg.intrPriority = 0;
    intrCfg.cm0pSrc = usbhsdev_interrupt_u2d_active_o_IRQn;
    Cy_SysInt_Init(&intrCfg, UsbHS_ISR);
    NVIC_EnableIRQ(NvicMux0_IRQn);

    /* USBDINIT will take care USBD and CAL layer */
    Cy_USB_USBD_Init(NULL, &usbdCtxt, pCpuDmacBase, &calCtxt);

    Cy_Flash_Init();

    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxLangString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxMfgString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxProdString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 3, (uint8_t *)CyFxSerialNoString);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);

    Cy_USBD_RegisterCallback(&usbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(&usbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);

    Cy_USBD_ConnectDevice(&usbdCtxt, CY_USBD_USB_DEV_HS);
#if BL_DEBUG
    DBG_BLOAD_INFO("Enabled connection to USBHS\r\n");
#endif
    while (1)
    {
        if(dfuState == DFU_MANIFEST_WAIT_RESET)
        {
            memcpy((uint8_t *)isBootActive, (uint8_t *)appMode, 8);
            Cy_SysLib_DelayUs(1000);
            Cy_USBD_DisconnectDevice (&usbdCtxt);
            NVIC_SystemReset();
        }
    }
    return 0;
}

/* [] END OF FILE */
