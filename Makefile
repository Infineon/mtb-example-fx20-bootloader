################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright (2025), Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=KIT_FX20_FMC_001

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-fx20-bootloader

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Clang Compiler (must be installed separately)
#
# To use the ARM toolchain, ensure the CY_COMPILER_ARM_DIR environment variable is set to the compiler's directory (absolute path).
# For example, the default path for ARMCLANG from a Keil installation is typically C:/Keil_v5/ARM/ARMCLANG.
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Release


# If set to "true" or "1", display full command-lines when building.
VERBOSE=

# Name of CORE to use: Bootloader is designed to run on the CM0P core only.
# CM0  -- Cortex M0
CORE=CM0P

################################################################################
# Advanced Configuration
################################################################################

# Include the bsp makefile so that we can get the MPN selection details.
-include bsps/TARGET_$(TARGET)/bsp.mk

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=FX20_$(CORE)

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=


# Add additional defines to the build process (without a leading -D).
DEFINES= \
        BCLK__BUS_CLK__HZ=75000000 \
        FREERTOS_ENABLE=0 \
        CYBSP_CUSTOM_SYSCLK_PM_CALLBACK=1 \
        CY_CRYPTO_USER_CONFIG_FILE=\"crypto_ip_cfg.h\"
	
DEBUG?=no
ifeq ($(DEBUG),yes)
$(info Debug enabled in this build. Disable to save code space)
    DEFINES += \
        DEBUG_INFRA_EN=1 \
        DEBUG_BLOCKING_PRINT_EN=1 \
        BL_DEBUG=1 \
        ENABLE_USBFS_DEBUG=1
endif

# Add a version string indicating the Bootloader version.
DEFINES += FX_BL_VERSION=\"1234abcd\"

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(TOOLCHAIN), GCC_ARM)
    CFLAGS= -Os -Og -flto -ffat-lto-objects -mcpu=cortex-m0plus -mthumb -ffunction-sections -fdata-sections -g
else ifeq ($(TOOLCHAIN), ARM)
    CFLAGS= -Os -Og -mcpu=Cortex-M0plus -mthumb -ffunction-sections -fdata-sections -g \
            --target=arm-arm-none-eabi -fno-rtti -fno-exceptions -Wno-error -fshort-wchar -fshort-enums
endif

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
ifeq ($(TOOLCHAIN), GCC_ARM)
    LDFLAGS=-Wl,--start-group -mcpu=cortex-m0plus -mthumb --entry=Reset_Handler -Wl,--gc-sections -g -ffunction-sections -finline-functions -flto -Os -Wl,--end-group
else ifeq ($(TOOLCHAIN), ARM)
    LDFLAGS=--cpu=Cortex-M0plus --entry=Reset_Handler --diag_suppress=L6329W,L6314W
endif

# Additional / custom libraries to link in to the application.
LDLIBS=

# Default device selection in case information is not provided by the BSP.
DEVICE ?= CYUSB4024-BZXI

# Check if the selected target is a supported part for this bootloader.
ifeq ($(filter $(DEVICE),CYUSB4024-BZXI CYUSB4014-FCAXI CYUSB3084-FCAXI CYUSB3284-FCAXI),)
    $(error Unsupported MPN: $(DEVICE))
endif

# Path to the linker script to use (if empty, use the default linker script).
# Use linker script for CM0P core
ifeq ($(DEBUG),yes)
$(warning Flash area limit increased to support DEBUG=yes. Make appropriate changes in user application's flash area to enable switch to user application)
LINKER_SCRIPT = $(if $(filter GCC_ARM,$(TOOLCHAIN)),fx_cm0plus_debug.ld,fx_cm0plus_debug.sct)
else
LINKER_SCRIPT = $(if $(filter GCC_ARM,$(TOOLCHAIN)),fx_cm0plus.ld,fx_cm0plus.sct)
endif
# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
# Post build to merge bootloader and application
ifeq ($(TOOLCHAIN), GCC_ARM)
POSTBUILD=\
    $(OBJCOPY) -O ihex build/$(TARGET)/$(CONFIG)/$(APPNAME).elf build/$(TARGET)/$(CONFIG)/$(APPNAME).hex
else ifeq ($(TOOLCHAIN), ARM)
POSTBUILD=\
    $(FROMELF) --i32combined --base=0x10000000 -o build/$(TARGET)/$(CONFIG)/$(APPNAME).hex build/$(TARGET)/$(CONFIG)/$(APPNAME).elf
endif

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory. The variable name depends on the
# toolchain used for the build. Refer to the ModusToolbox user guide to get the correct
# variable name for the toolchain used in your build.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# software provided compiler by default).
CY_COMPILER_GCC_ARM_DIR=

# Locate ModusToolbox helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox software in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
    $(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

# Path to FROMELF tool.
FROMELF=$(MTB_TOOLCHAIN_ARM__ELF2BIN)

# Path to OBJCOPY tool.
OBJCOPY=$(MTB_TOOLCHAIN_GCC_ARM__ELF2BIN)

include $(CY_TOOLS_DIR)/make/start.mk
