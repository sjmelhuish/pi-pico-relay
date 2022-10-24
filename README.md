# pi-pico-relay

This project uses a Pi Pico (original or W) micro-controller board to control a coaxial relay.

The relay used is the "transfer switch" type, so the two switch states are presented as through and cross-over.
User control is by two push-buttons. This triggers a relay control pulse from either of two output pins.
Relay state is sensed using two input pins. This state is then presented to the user by means of an OLED display.
The display is addressed using I<sup>2</sup>C.
