# EZ-USB&trade; FX bootloader

This code example demonstrates the implementation of USB bootloader for EZ-USB&trade; FX family of devices.

The USB bootloader occupies the first 32 KB of the on-chip flash memory and leaves the remaining area for application-specific usage. This bootloader supports programming of firmware applications to the application-specific area in the device's internal flash without having to make use of a dedicated Serial Wire Debug (SWD) programmer.

The last 512 bytes of the on-chip flash is expected to store an SHA256 checksum calculated over the rest of the application-specific flash contents. The bootloader makes use of this checksum to verify the integrity of the application before transferring control to it. The checksum is calculated and appended to the application HEX file by the post build commands provided in every EZ-USB&trade; FX code example.

If a valid application is not found in the flash, the bootloader proceeds to enumerate as a High-Speed USB vendor class device. The EZ-USB&trade; FX Control Center application can be used to exercise the programming features of the bootloader. It supports programming a provided application hex file onto the internal flash. It also supports programming FPGA binaries to the external SPI flash module available on the EZ-USB&trade; FX development kits.

For more details, see the Help on the EZ-USB&trade; FX Control Center application.

> **Note:** This bootloader project is also applicable to EZ-USB&trade; FX10, EZ-USB&trade; FX5N, and EZ-USB&trade; FX5 devices.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-fx20-bootloader)

[Provide feedback on this code example.](https://yourvoice.infineon.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDE2MTgiLCJTcGVjIE51bWJlciI6IjAwMi00MTYxOCIsIkRvYyBUaXRsZSI6IkVaLVVTQiZ0cmFkZTsgRlggYm9vdGxvYWRlciIsInJpZCI6ImFzaHdpbi5uYWlyQGluZmluZW9uLmNvbSIsIkRvYyB2ZXJzaW9uIjoiMS4wLjEiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiV0lSRUQiLCJEb2MgRmFtaWx5IjoiU1NfVVNCIn0=)


## Requirements

- [ModusToolbox&trade; v3.5 or later](https://www.infineon.com/modustoolbox?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=fxbootloader&uid=ci0108940001&aid=ai010894) (tested with v3.5)
- Board support package (BSP) minimum required version: 4.3.3
- Programming language: C


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)


## Supported kits (make variable 'TARGET')

- [EZ-USB&trade; FX20 DVK](https://www.infineon.com/evaluation-board/KIT-FX20-FMC-001?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_boardpage&utm_term=fxbootloader&uid=ci0108940002&aid=ai010894) (`KIT_FX20_FMC_001`) – Default value of `TARGET`


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.
You need a Miniprog4 to program the bootloader hex file to the device.


## Software setup

- See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/document-promo/infineon-modustoolbox-software-installation-guide-gettingstarted-en_6c67405e-0877-40e9-ab99-18759b12b102?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=fxbootloader&uid=ci0108940013&aid=ai010894) for information about installing and configuring the tools package

- Install the [EZ-USB&trade; FX Control Center](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfxcontrolcenter?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=fxbootloader&uid=ci0108940003&aid=ai010894) application to help with device programming


## Using the code example


### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/document-promo/infineon-modustoolbox-project-creator-user-guide-usermanual-en_83d9e435-643c-4353-9d40-fa7bca5e7720?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940004&aid=ai010894) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*)

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target)

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**

      > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you

   b. Select this code example from the list by enabling its check box

      > **Note:** You can narrow the list of displayed examples by typing in the filter box

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**

   d. Click **Create** to complete the application creation process

</details>


<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-fx20-bootloader](https://github.com/Infineon/mtb-example-fx20-bootloader)" application with the desired name "FX20_BOOTLOADER" configured for the *KIT_FX20_FMC_001* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_FX20_FMC_001 --app-id mtb-example-fx20-bootloader --user-app-name FX20_BOOTLOADER --target-dir "C:/mtb_projects"
   ```

The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/document-promo/infineon-modustoolbox-tools-package-user-guide-gettingstarted-en_231b37fd-6dc2-4e85-b587-2542043c8b8d?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940005&aid=ai010894) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/document-promo/infineon-modustoolbox-eclipse-ide-user-guide-usermanual-en_cc5e5dff-54e7-48f8-82fc-d0157f732a67?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940006&aid=ai010894) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/document-promo/infineon-visual-studio-code-user-guide-usermanual-en_60d79bb8-6e50-4d28-9ef3-f14b1a546046?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940007&aid=ai010894) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/document-promo/infineon-modustoolbox-tools-package-user-guide-gettingstarted-en_231b37fd-6dc2-4e85-b587-2542043c8b8d?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940005&aid=ai010894) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


### Using this code example with specific manufacturer part numbers (MPNs)

By default, the code example build is targeted for the `CYUSB4024-BZXI`, which has 512 KB of flash memory. Use the BSP Assistant tool to modify the application to target different EZ-USB&trade; FX20, EZ-USB&trade; FX10, EZ-USB&trade; FX5N, or EZ-USB&trade; FX5 family MPNs as follows.

> **Note:** This application makes use of the Crypto hardware block to do SHA checksum computation and the Serial Memory Interface block to access external flash devices. Therefore, it is only supported on the EZ-USB&trade; MPNs listed in **Table 1**.

**Table 1. MPNs supported by this code example**

MPN       | Family | Flash size (KB)
:---------------- | :----- | :--------------
CYUSB4024-BZXI    | EZ-USB&trade; FX20   | 512
CYUSB4014-FCAXI   | EZ-USB&trade; FX10   | 512
CYUSB3284-FCAXI   | EZ-USB&trade; FX5N   | 512
CYUSB3084-FCAXI   | EZ-USB&trade; FX5    | 512

<br>


#### Setup for a different MPN

Perform the following steps to modify the code example to work on a different MPN as listed in **Table 1**

1. Launch the BSP Assistant tool:

   a. **Eclipse IDE:** Launch the BSP Assistant tool by navigating to **Quick Panel** > **Tools**

   b. **Visual Studio Code:** Select the ModusToolbox&trade; extension from the left menu bar and launch the BSP Assistant tool, which is available in the **Application** menu of the **MODUSTOOLBOX TOOLS** section

2. In BSP Assistant, select **Devices** from the tree view on the left

3. Choose the desired part from the drop down menu on the right

4. Click **Save**. This closes the BSP Assistant tool

5. Build the application and proceed with programming


## Operation

1. Connect the board (J2) to your PC using the provided USB cable

2. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**
   </details>


   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE.

   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

<details><summary><b>Programming application hex file to internal flash</b></summary>

1. Open the **EZ-USB&trade; FX Control Center** application <br> 
    The EZ-USB&trade; FX20 device enumerates as **EZ-USB&trade; FX Bootloader**<br>
2. Select the **EZ-USB&trade; FX Bootloader** device in **EZ-USB&trade; FX Control Center**<br>
3. Click **Program** > **Internal Flash**<br>
4. Navigate to the folder with the *.hex* file and program. Confirm if the programming is successful in the log window of the **EZ-USB&trade; FX Control Center** application<br>
5. After programming, the application starts automatically<br>
</details>

<details><summary><b>Programming an application hex file to external flash</b></summary>

1. Select **Program** > **External SPI Flash**<br>
2. Browse and select the file to program <br>
3. The **EZ-USB&trade; FX Control Center** application checks the file size, erases the required number of sectors, and writes the file in 2048 byte chunks to the external SPI flash <br> After every 2048 bytes of write, the Control Center application reads back and verifies if the write is successful
</details>

<details><summary><b>Using DFU application to program application to internal flash</b></summary>

1. Convert the application hex file to binary format using the [SRecord tool](https://srecord.sourceforge.net/)<br>
For example, use the following command to convert application hex generated with an offset to support the bootloader region to binary file<br>
```srec_cat.exe application.hex -Intel -offset -0x10008000 -o application_dfu.bin -Binary``` <br>

2. Use the [DFU utility](https://dfu-util.sourceforge.net/) to download the binary file to the internal flash <br>
```dfu-util.exe -D application_dfu.bin --serial 001``` <br>
</details>

> **Note:** All above sections require the device to be in bootloader mode of operation.

<details><summary><b>Change the device into bootloader mode</b></summary>

Follow these steps to get the device into bootloader mode of operation: <br>
1. Press and hold the **PMODE (SW2)** switch <br>
2. Press and release the **RESET (SW3)** switch <br>
3. Finally, release the **PMODE** switch<br>
</details>


## Debugging

You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/document-promo/infineon-modustoolbox-eclipse-ide-user-guide-usermanual-en_cc5e5dff-54e7-48f8-82fc-d0157f732a67?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_userguide&utm_term=fxbootloader&uid=ci0108940006&aid=ai010894).

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.

</details>


## Design and implementation

The EZ-USB&trade; FX bootloader occupies the first 32 KB of internal flash and is responsible for validating, programming, and launching user applications, as well as supporting external SPI flash programming.

The application region follows, with the last 512 bytes reserved for an SHA256 checksum. The bootloader verifies this checksum before launching the application. On reset, the bootloader checks the application vector table for valid stack pointer and reset vector values, and verifies the SHA256 checksum over the application region. If validation fails, the device remains in bootloader mode. The PMOD or boot pin is sampled at startup and this is used to determine whether to stay in bootloader mode or attempt to launch the user application.

The bootloader implements a USB 2.0 High-Speed device using a vendor-specific interface and supports Device Firmware Upgrade (DFU) class requests. It can receive firmware images over USB over these interfaces and program them into internal flash.

The bootloader also supports programming external SPI flash devices using EZ-USB&trade; FX device's SMIF interface. It detects and configures supported flash devices at runtime using SFDP or CFI tables, and provides vendor commands for erase, read, and write operations.


### Using logs from UART or USB FS 

The bootloader does not support debug prints over UART or USB FS by default due to code-size constraints. If you need to enable debug prints, set `DEBUG=yes` in *Makefile*. This will make the linker use a separate linker/scatter file with increased flash size allocation (44 KB instead of 32 KB) to accommodate the increase in code size due to this change. <br>
User applications that will be loaded by the bootloader must be compiled to start at this new flash address (0x1000B000). To ensure this, the application's linker or scatter file must be modified to configure the correct flash start address and length.


## Compile-time configurations

Application functionality can be customized by setting variables in *Makefile* or by configuring them through `make` CLI arguments.

- Run the `make` command or build the project in your IDE to compile the application and generate a USB bootloader-compatible binary. This binary can be programmed onto the EZ-USB&trade; FX20 device using the EZ-USB&trade; Control Center application

- Choose between the **Arm&reg; Compiler** or the **GNU Arm&reg; Embedded Compiler** build toolchains by setting the `TOOLCHAIN` variable in *Makefile* to `ARM` or `GCC_ARM` respectively. If you set it to `ARM`, ensure to set `CY_ARM_COMPILER_DIR` as a make variable or environment variable, pointing to the path of the compiler's root directory

- Set `ADD_VERSION` to yes to enable adding version information to the bootloader

- Set `DEBUG` to yes to enable debug prints from the bootloader. See [Debugging](#debugging) section for more details

<br>


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN237841](https://www.infineon.com/document-promo/infineon-getting-started-with-ez-usb-fx20-fx10-fx5n-fx5-applicationnotes-en_9fbbff93-43d1-4aec-804f-b0dd911e3d10?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_applicationnote&utm_term=fxbootloader&uid=ci0108940008&aid=ai010894) – Getting started with EZ-USB&trade; FX20/FX10/FX5N/FX5
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [EZ-USB&trade; FX20 datasheets](https://www.infineon.com/document-promo/infineon-cyusb402x-ez-usb-fx20-usb-20-gbps-peripheral-controller-datasheet-en_1bf09735-a0df-42c1-86c1-335cbaf4452c?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=docs_datasheet&utm_term=fxbootloader&uid=ci0108940009&aid=ai010894)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=fxbootloader&uid=ci0108940010&aid=ai010894)
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – Peripheral Driver Library (PDL)
Middleware on GitHub  | [usbfxstack](https://github.com/Infineon/usbfxstack) – USBFX Stack middleware library and docs
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=fxbootloader&uid=ci0108940001&aid=ai010894) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.

[Peripheral controllers for USB 3.2 homepage](https://www.infineon.com/products/universal-serial-bus/usb-3-2-peripheral-controllers?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_productpage&utm_term=fxbootloader&uid=ci0108940011&aid=ai010894)<br>
[Peripheral controllers for USB 3.2 webinar](https://www.infineon.com/event/webinar/2025/usb1020gbps?utm_campaign=202602_glob_en_pss.p.supsp_usb&utm_medium=3rdptmedia&utm_source=github_repository&utm_content=website_eventpage&utm_term=fxbootloader&uid=ci0108940012&aid=ai010894)


## Document history

Document title: *CE241618* – *EZ-USB&trade; FX bootloader*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
 1.0.1   | Updated to use the correct linker script when building for debug configuration

<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

© Cypress Semiconductor Corporation, 2026. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.


