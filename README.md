# BlastOS
WIP Arduino (and eventually RP2040) firmware for modern brushless foam dart blasters.
Field testing still WIP, use at your own risk.

## Features
- Simple firing modes: Semi/Burst/Auto. No plans to implement any others. Burst ranges 2 to 9, ends on trigger release.
- PWM only. Support for RP2040 and bDshot300/600 is planned.
- Requires separate rev control switch, potentially subject to change.
- Tight and robust firing control loop with no unintended behaviors out of user control.
- DPS control.
- Stealth functionality - applies motor braking (if ESC supports it) and turns off screen.
- Fractional velocity - temporarily reduces velocity by a set amount for arcing/mortaring darts. Experimental feature.
- Configuration menu.
- Simple competition lock, accessible from configuration menu.
- Controlled update logic only updates screen at slow intervals, or during selector changes, minimizing control disruption. Uses non-blocking nI2C library.

## Other
Some settings are not exposed to config yet, such as per-cell min and max voltages.
Battle testing still TBD.
Requires sufficient controls on the blaster for boot velocity and firing mode selections, as well as menu navigation.

Developed on Spirit/Phanta hardware. More specific blaster platform implementations TBD.
