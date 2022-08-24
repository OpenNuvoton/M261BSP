# M261 Series CMSIS BSP

This BSP folder

## .\Document\


- CMSIS.html<br>
	Introduction of CMSIS version 5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro M261 Series CMSIS BSP Revision History.pdf<br>
	The revision history of M261 Series BSP.

- NuMicro M261 Series Driver Reference Guide.chm<br>
	The usage of drivers in M261 Series BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V5.0 definitions by ARM® Corp.

- Device<br>
	CMSIS compliant device header file.

- NuMaker<br>
	Specific libraries for M261 NuMaker board.

- SmartcardLib<br>
	Library for accessing a smartcard.

- StdDriver<br>
	All peripheral driver header and source files.

- UsbHostLib<br>
	USB host library source code.

## .\Sample Code\


- AttackDetection<br>
	Sample codes for non-invasive physical attack detection.

- CardReader<br>
	USB CCID Smartcard Reader sample code.

- FreeRTOS<br>
	Simple FreeRTOS demo code.

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened. The hard fault handler show some information included program counter, which is the address where the processor was executing when the hard fault occur. The listing file (or map file) can show what function and instruction that was. It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- MKROM<br>
	Demonstratethe usage of M261 MKROM libraries, and show how to generate a secure boot image for Secure Boot Verification.

- NuMaker<br>
	Sample codes for NuMaker-PFM-M261 board.

- PowerManagement<br>
	Power management sample code.

- Semihost<br>
	Show how to print and get character through IDE console window.

- StdDriver<br>
	Demonstrate the usage of M261 series MCU peripheral driver APIs.

- XOM<br>
	Demonstrate how to create XOM library and use it.


## .\ThirdParty\

- FatFs<br>
	An open source FAT/exFAT filesystem library.


# Licesne

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
M261 BSP files are provided under the Apache-2.0 license.

