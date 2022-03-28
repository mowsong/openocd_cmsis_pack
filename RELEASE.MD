### Cypress OpenOCD v4.3.1 Release Notes

Please refer to the README.md for a complete description of the Cypress customized version of OpenOCD.

### New Features

- Adds support of WLC1 devices
- Access to the SFlash User Rows is blocked on CCGx, PMG1 and WLCx devices

### Known Issues

| ID                                | Known Issue                       | Workaround                          |
|-----------------------------------|-----------------------------------|-----------------------------------|
|  PROGTOOLS-1425 | A “JTAG-DP STICKY ERROR” message may appear in the IDE when connected via the JTAG interface of a MiniProg4 probe in CMSIS-DAP HID mode.  |  You can safely ignore this error, or switch the MiniProg4 to CMSIS-DAP bulk mode. |
| PROGTOOLS-1429  | JTAG performance on MiniProg4 may be significantly slower than SWD.  |  Use the SWD interface if JTAG performance is not acceptable. |
|  PROGTOOLS-1869 | In Linux OS, with KitProg3 in CMSIS-DAP HID mode, a debug session in ModusToolbox can be destroyed if you use the Firmware Loader --device-list command while debugging. This is limitation of hidapi library used on Linux. MacOS and Windows OSs are not impacted. | If you have a debug session running, don't use the firmware loader tool.  |
|  PROGTOOLS-2388 | PSoC 64 Secure MCUs do not support the programming and debugging via JTAG interface.  It is because of the specifics of PSoC 64 MCUs - JTAG pins are configured after unpredictable amount of time after what causes a lot of warnings during programming via JTAG. | Use SWD interface with PSoC 64 Secure MCUs.  |
|  PROGTOOLS-2238 | Unable to acquire PSoC4 target with J-Link probe if target is in PROTECTED state. J-Link probe does not support device acquisition in Test Mode. This prevents invocation of SROM API making it impossible to re-program or erase the chip in PROTECTED state. | The workaround is to unprotect/erase the chip using MiniProg4 or KitProg3 probe.  |
|  PROGTOOLS-2099 | A “Failed to read memory at <address>” message may appear in “Memory View” of IDE when connected via the MiniProg4 probe and try to read memory from invalid address. | You can safely ignore this error.  |
|  PROGTOOLS-912 | Programming the S25FS512S external memory device mounted on the CYW9P62S1-43012EVB-01 kit might not work if SFDP auto-detection is selected in the QSPI Configurator. This is caused by corrupted SFDP table in this memory. | Select "S25FS512S" instead of "Auto detect SFDP" for the memory slot in the QSPI Configurator.  |

### More information

-   [openOCD
    README.md](https://github.com/cypresssemiconductorco/openocd/blob/cypress/README.MD)

-   [openOCD User Guide](https://www.cypress.com/file/495061/download)

-   [ModusToolbox Software Environment, Quick Start Guide, Documentation, and
    Videos](https://www.cypress.com/products/modustoolbox-software-environment)

-   [ModusToolbox Device Configurator Tool
    Guide](https://www.cypress.com/ModusToolboxDeviceConfig)

-   [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)

© Cypress Semiconductor Corporation, 2020-2022. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").
