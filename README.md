MWL Twister 2
================

Code and hardware designs for the MWL's twister 2, a simple stepper-based
trode twister.

Code requires a couple Arduino libraries
- AccelStepper
- Liquid Crystal
- Teensyduino add-on.

This repository (should eventually) contain circuit board files, firmware code,
and a parts list.

## Usage
Twister 2 is simple. It turns your trodes in two directions for a set number of
turns per direction. The speed of the twister can also be controlled.

- Power the device.
- Place your finger on the control knob. This is the device's only user input
  mechanism.
![Control Knob](./resources/protoype.jpg)
    - __Pressing it__ will cycle through different settings (forward turns,
      backward turns, turn speed)
    - __Turning it__ will increment or decrement the selected setting depending
      on turn direction.
    - __Pressing and holding it__ for 500 msec will execute the twist sequence
      using the current settings
    - __Pressing it__ during a twist will cancel the twist and stop the motor
      immediately.

- The actual "twister" is just a 200-detent stepper motor with one of the old
  twister's armature glued to it. You should leave the rubber mat under the
  stepper to prevent vibrations.i
