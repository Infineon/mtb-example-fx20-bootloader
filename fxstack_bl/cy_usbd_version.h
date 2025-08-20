/***************************************************************************//**
* \file cy_usbd_version.h
* \version 1.0
*
* \brief This file contains version information for the USBD for bootloader
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




#ifndef _CY_USBD_VERSION_H_
#define _CY_USBD_VERSION_H_

#define USBD_VERSION_MAJOR      (0u)
#define USBD_VERSION_MINOR      (2u)
#define USBD_VERSION_PATCH      (0u)
#define USBD_VERSION_BUILD      (58u)

#define USBD_VERSION_NUM        ((USBD_VERSION_MAJOR << 28u) | (USBD_VERSION_MINOR << 24u) | \
        (USBD_VERSION_PATCH << 16u) | (USBD_VERSION_BUILD))

#endif /* _CY_USBD_VERSION_H_ */
