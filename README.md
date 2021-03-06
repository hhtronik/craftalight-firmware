Craftalight
===========

This repository contains the firmware for Craftalight, the open source hardware USB-powered LED light for your (hand)bag.

The hardware repository for Craftalight is over there: [craftalight-hardware on Github](https://github.com/hhtronik/craftalight-hardware)

> NOTE: this document is a work-in-progress


Development environment setup
-----------------------------

You need an up-to-date version of Visual Studio Code. Then go to [PlatformIO.org](https://platformio.org/) and follow the instructions to install the VS Code extension.

Next, clone this repository and open it in Visual Studio Code

Debugging and flashing
----------------------

To connect the debug probe (you will need an STLink V2 or another SWIM-Compatible debug probe PlatformIO can deal with) connect the pins labeled as `SWIM`-Port on the PCB as follow (from left to right):

1. Ground (square pad)
2. SWIM
3. NRST
4. VCC Ref (!! not meant as supply for the board, do not connect on STLink clones when using a USB supply at the same time !!)

The PlatformIO enviromnent enables you to do real debugging on the Craftalight. If you just want to flash a new firmware to the Craftalight, you can you can use other tools (like `ST STM8 Visual Programmer` or `stm8flash`) to do so.

Licence
-------

This software is under GPLv3 licence. See [LICENCE](./LICENCE) file for a full copy of the terms.

