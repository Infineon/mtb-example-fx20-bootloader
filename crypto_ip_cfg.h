/***************************************************************************//**
* \file crypto_ip_cfg.h
* \version 1.0
*
* \brief Crypto driver configuration settings used in this application.
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

#ifndef _CRYPTO_IP_CFG_H_
#define _CRYPTO_IP_CFG_H_

#if defined(__cplusplus)
extern "C" {
#endif

/* Use MPN specific hardware definitions. */
#define CY_CRYPTO_CFG_HW_USE_MPN_SPECIFIC

/* EZ-USB FX devices have Crypto hardware block version 2. */
#define CY_CRYPTO_CFG_HW_V2_ENABLE

/* Custom configuration: Enable only SHA256 function. */
#define CY_CRYPTO_CFG_SHA_C
#define CY_CRYPTO_CFG_SHA2_256_ENABLED

#if defined(__cplusplus)
}
#endif

#endif /* _CRYPTO_IP_CFG_H_ */

/*[]*/

