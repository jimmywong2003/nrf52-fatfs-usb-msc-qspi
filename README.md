# nrf52-fatfs-usb-msc-qspi

Example of USB MSC with Filesystem on nRF52840 DK 

## This example is to modify the original SDK example (USB-MSC) with QSPI.  It modified the 4 button functions
* Button 1: toggle USB on/off
* Button 2: create the random files
* Button 3: List the directory
* Button 4: Create the file system

[Note]:  Manipulating the file system from the development kit will not be possible while USB is connected.

## Requirement
* SDK 17.0
* nRF52840 DK Board
* Segger Embedded Studio 4.51 or later
