# Transputer Picoputer

This project is a port of 
[blackjetrock's Picoputer](https://github.com/blackjetrock/picoputer), which itself is based on Julian Highfield's T4 Transputer emulator, available from the 
[pahihu/t4 repository](https://github.com/pahihu/t4).

Julian's T4 emulator is an impressive piece of software. It emulates a Transputer and can run programs compiled with the original INMOS toolset.

This port runs the emulator on a Raspberry Pi Pico 2. One of the emulated Transputer links is connected to the Pico's PIO hardware. The PIO state machines implement the Transputer link protocol at a link speed of 20 Mbit/s.

The physical link interface currently used with the project is based on one of the Teensy Link Interface boards from the 
[Transputer Teensy Link project](https://github.com/dg1vs/Transputer-Teensy-Link).

## Goals

One of the main goals of this project is to improve my understanding of the Transputer architecture, its link protocol, and the interaction between Transputer software and external hardware.

In the longer term, I would like to build Transputer-compatible I/O devices that use the Raspberry Pi Pico's PIO hardware to implement Transputer link interfaces.

The project is also intended as a learning platform for:

- understanding the internal operation of the T4 emulator;
- running software compiled with the original INMOS toolchain;
- experimenting with the Transputer link protocol;
- connecting the emulator to real Transputer hardware; 
- and developing new Transputer-compatible peripherals.

## Related Projects

A similar project is the [DevZendo Transputer Emulator](https://github.com/devzendo/transputer-emulator/tree/master), which also uses a Raspberry Pi Pico to emulate a Transputer.

For this project, however, I decided to continue using Julian Highfield's emulator. One important reason is its close relationship with the Transputer validation suite created by Mike Brüstle and available through [transputer.net](https://www.transputer.net/).




# Transputer-Picoputer

This is a port of blackjetrock's picoputer 
(via [https://github.com/blackjetrock/picoputer](https://github.com/blackjetrock/picoputer)) 
which is port of Julian Highfield's transputer emulator 
(via [https://github.com/pahihu/t4](https://github.com/pahihu/t4)). 
The t4 transputer emulator from Julian is a nice piece of software, which can be used to compile transputer software using the old INMOS-toolset. 
The emulator runs now on a Raspberry Pi Pico2 and talks down one link that are attached to PIO hardware. 
The PIOs run code that uses the 20MHz transputer link protocol. 
The link-interface I'm using is one off then Teensy-Link-Interfaces (TLI) 
(see [https://github.com/dg1vs/Transputer-Teensy-Link](https://github.com/dg1vs/Transputer-Teensy-Link))

## Goal
One of my main goals of this coding experince is to understand the Transputer in mor detail and on the long run build I/O-devices using the PIO as an Transputer-inteface

## Other similar projects

I found a similar project
(see [https://github.com/devzendo/transputer-emulator/tree/master](https://github.com/devzendo/transputer-emulator/tree/master)),
using an Raspberry Pi Pico for simulating an Transputer. Nevertheless I'm sticked to Julian emulator since he runs the Transputer Validation suite from Mike 
[transputer.net](transputer.net)



