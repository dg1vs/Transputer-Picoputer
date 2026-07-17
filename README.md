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

## Next steps


