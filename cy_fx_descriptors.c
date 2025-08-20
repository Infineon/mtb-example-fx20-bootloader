/***************************************************************************//**
* \file cy_fx_descriptors.c
* \version 1.0
*
* USB descriptors used in application
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


/**
 * \name CyFxUSB20DeviceDscr
 * \brief Standard USB 2.0 device descriptor for the bootloader device.
 */
const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (4))) =
{
    0x12, /* Descriptor size */
    0x01, /* Device descriptor type */
    0x00,0x02, /* USB 2.00 */
    0x00, /* Device class */
    0x00, /* Device sub-class */
    0x00, /* Device protocol */
    0x40, /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04, /* Vendor ID */
    0xE1,0x00, /* FX3G2 Bootloader Vendor ID. */
    0x00,0x00, /* Device release number */
    0x01, /* Manufacture string index */
    0x02, /* Product string index */
    0x03, /* Serial number string index */
    0x01 /* Number of configurations */
};

/**
 * \name CyFxUSBDeviceQualDscr
 * \brief Standard USB device qualifier descriptor for high-speed operation.
 */
const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (4))) =
{
    0x0A, /* Descriptor size */
    0x06, /* Device qualifier descriptor type */
    0x00,0x02, /* USB 2.0 */
    0x00, /* Device class */
    0x00, /* Device sub-class */
    0x00, /* Device protocol */
    0x40, /* Maxpacket size for EP0 : 64 bytes */
    0x01, /* Number of configurations */
    0x00 /* Reserved */
};

/**
 * \name CyFxUSBHSConfigDscr
 * \brief Standard high-speed configuration descriptor including DFU interface and functional descriptors.
 */
uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (4))) =
{
    0x09, /* Configuration descriptor size */
    0x02, /* Configuration descriptor type */
    0x1B, 0x00, /* Total length */
    0x01, /* Number of interfaces */
    0x01, /* Configuration number */
    0x00, /* Configuration string index */
    0x80, /* Bus powered */
    0x32, /* Max power (100mA) */
    0x09, /* Interface descriptor size */
    0x04, /* Interface descriptor type */
    0x00, /* Interface number */
    0x00, /* Alternate setting */
    0x00, /* Number of endpoints */
    0xFE, /* Interface class */
    0x01, /* Interface sub class */
    0x01, /* Interface protocol */
    0x00, /* Interface string index */
    0x09, /* DFU Functional descriptor size */
    0x21, /* DFU Functional descriptor type */
    0x01, /* bmAttributes - bitDownload */
    0xF4, 0x01, /* wDetachTimeOut: 500ms */
    0x00, 0x10, /* Max size of payload */
    0x01, 0x01 /* bcdRevision */
};

/**
 * \name CyFxLangString
 * \brief USB language ID string descriptor (US English).
 */
const uint8_t CyFxLangString[] __attribute__ ((aligned (4))) =
{
    0x04,
    0x03,
    0x09,
    0x04
};

/**
 * \name CyFxMfgString
 * \brief USB manufacturer string descriptor.
 */
const uint8_t CyFxMfgString[] __attribute__ ((aligned (4))) =
{
    0x12, 0x03,
    'I',  0x00,
    'N',  0x00,
    'F',  0x00,
    'I',  0x00,
    'N',  0x00,
    'E',  0x00,
    'O',  0x00,
    'N',  0x00
};

/**
 * \name CyFxProdString
 * \brief USB product string descriptor.
 */
const uint8_t CyFxProdString[] __attribute__ ((aligned (4))) =
{
    0x2A, 0x03,
    'E',  0x00,
    'Z',  0x00,
    '-',  0x00,
    'U',  0x00,
    'S',  0x00,
    'B',  0x00,
    ' ',  0x00,
    'F',  0x00,
    'X',  0x00,
    ' ',  0x00,
    'B',  0x00,
    'O',  0x00,
    'O',  0x00,
    'T',  0x00,
    'L',  0x00,
    'O',  0x00,
    'A',  0x00,
    'D',  0x00,
    'E',  0x00,
    'R',  0x00
};

/**
 * \name CyFxSerialNoString
 * \brief USB serial number string descriptor.
 */
const uint8_t CyFxSerialNoString[] __attribute__ ((aligned (4))) =
{
  0x08,
  0x03,
  '0', 0x00,
  '0', 0x00,
  '1', 0x00,
};

/**
 * \name glOsString
 * \brief Microsoft OS String Descriptor for WinUSB compatibility.
 */
uint8_t glOsString[] __attribute__ ((aligned (4))) =
{
    0x12, /* Length. */
    CY_USB_STRING_DSCR, /* Type - string. */
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00, /* Signature. */
    MS_VENDOR_CODE, /* MS vendor code. */
    0x00 /* Padding. */
};

/**
 * \name glOsCompatibilityId
 * \brief Microsoft OS Compatibility ID Descriptor for WinUSB binding.
 */
uint8_t glOsCompatibilityId[] __attribute__ ((aligned (4))) =
{
    0x28, 0x00, 0x00, 0x00, /* length */
    0x00, 0x01, /* BCD version */
    0x04, 0x00, /* Index: 4 - compatibility ID */
    0x01, /* count. Need to be updated based on number of interfaces. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved. */
    /* First Interface */
    0x00, /* Interface number */
    0x01, /* reserved: Must be 1 */
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, /* comp ID: WinUSB */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* sub-compatibility ID: NONE */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved */
};

/**
 * \name glOsFeature
 * \brief Microsoft OS Feature Descriptor for device interface GUID.
 */
uint8_t glOsFeature[] __attribute__ ((aligned (4))) =
{
    0x8E, 0x00, 0x00, 0x00, /* Length */
    0x00, 0x01, /* BCD version */
    0x05, 0x00, /* Index */
    0x01, 0x00, /* count */
    0x84, 0x00, 0x00, 0x00, /* length */
    0x01, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x28, 0x00, /* wPropertyNameLength */
    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6E, 0x00,
    0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00,
    0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00, /* bPropertyName: DeviceInterfaceGUID */
    0x4E, 0x00, 0x00, 0x00, /* dwPropertyDataLength */
    '{', 0x00, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00, '6', 0x00,
    '7', 0x00, '-', 0x00, '2', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'E', 0x00, 'E', 0x00, '-', 0x00, '8', 0x00, 'D', 0x00, 'D', 0x00, '3', 0x00,
    '-', 0x00, 'F', 0x00, 'A', 0x00, 'D', 0x00, 'E', 0x00, 'A', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00
    /* bPropertyData: {01234567-2A4F-49EE-8DD3-FADEA377234A} */
};


/* [] END OF FILE */
