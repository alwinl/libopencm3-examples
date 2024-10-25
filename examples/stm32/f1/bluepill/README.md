# Bluepill examples

These examples target a generic STM32F103C8T6 based development board commonly
known as the "Bluepill".

These boards are readily available from sources like AliExpress or E-Bay for a 
couple of dollars (search for stm32 bluepill or stm32f103c8t6). Flashing these
projects are set up to use OpenOCD and a ST-link programmer or clone.
OpenOCD is free software and an ST-link programmer clone is available from the 
same sources, again for a couple of dollars (search for "stlink v2")
Some sellers even sell a combo (a bluepill and a stlink).

## Caveats

These examples show the implementation of the OpenCM3 library. I presume you
are able to code in C, have some rudimentary microcontroller knowledge and
know your way around your own computer. Specifically I leave it to you to set
up your toolchain, flasher software, editor/IDE and the like. If you are
familiar with the "make" command and have some way of executing "make" using 
an arm toolchain and know how to set up and run your flasher software
(ie OpenOCD or st-flash) you should be fine.

Along with these examples it will pay to have the ST Reference Manual (RM0008)
and the libopencm3 documentation at your fingertips. Download the RM0008 from
the ST website and build the libopencm3 documentation in the libopencm3 
subdirectory.

Also usefull is a schematic of the dev board and a pinout diagram. Both can be
found on the web (search terms "blue pill pinout" and "blue pill schematics").


## Programmer and board setup

Navigate to [Roger Clarks Wiki on github](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Programming-an-STM32F103XXX-with-a-generic-%22ST-Link-V2%22-programmer-from-Linux)
for a description (with photos) on how to hook the board and the programmer together.

## Usage

I have tried to create a tutorial-like progression in the examples. You might
get the most out of these examples if you study and try these examples in order.

To flash your program to the board use the **bp-flash** target (make bp-flash).

## Example list

### Fancyblink

When you apply power to an STM32F103 it will use the internal 8Mhz clock.
This example shows how to configure the Reset and Clock Control (RCC) to
use the blue pill's external 8Mhz crystal.

By default the pins used by the programmer to access the STM32F103
(SWDCLK (PA14) and SWDIO (PA13)) are set to be general purpose IO.

This means that, without the proper precautions, we can only flash our program 
once. In order to reflash a new program we have to play with the BOOT0 pin.
It can be done but is inconvenient.

The solution is to remap these pins and configure them properly, so we can take
control of the board without having to change jumpers.
As an added bonus we will be able to debug our program on the board.

So the fancyblink program remaps the pins and configures the AFIO block by
reprogramming the AFIO_MAPR register. An added gotcha is that, by default,
all peripherals are disabled. Thus we need to enable the AFIO clock.

The rest is relatively simple. To blink the led (attached to port C, pin 13) we
need to configure the appropriate GPIO port, ie turn on the port's clock and
configure the pin to be an output.

In the main function we toggle the LED and wait a while. Repeat ad infinitum.

### TimedBlink

In this example we use a general purpose timer peripheral to better control
the timing of the led toggle.

Additional steps required are enabling the timer peripheral, configuring and
starting it and then check the underflow event in the main loop.

### InterruptBlink

This example uses the general purpose timer 2 again, but in this instance the
timer is programmed to fire an interrupt on underflow.
We toggle the led in the interrupt routine so no work is required in the main
function.

In the file *libopencm3/include/libopencmsis/stm32/f1/irqhandlers.h* is a list
of all the ISR functions that are weakly bound by libopencm3. By redefining
these functions you implement your custom interrupt service routine.

In this example we have implemented `void tim2_isr(void)` to service TIM2 interrupts.

### SystickBlink

One of the core Cortex-M peripherals is the SysTick timer. This block is commonly
used to drive an operating system tick or, more generally, to provide an internal
time base for your firmware to use.
The easiest way to use the systick timer is to set the frequency, enable interrupts
and set it running.

The ISR we have defined here is `void sys_tick_handler(void)` which is defined in 
*libopencm3/lib/cm3/vector.c*

### ControlledBlink

This is the first example that uses an input. The blue pill features a pin header block next
to the reset switch with two jumper links. The link closest to the edge of the board
is connected to the BOOT0 pin and the link next to the reset switch is connected to
the BOOT1 pin. Section 3.4 in the RM0008 explains the various boot up options but
the main take away for this example is that we keep the BOOT0 pin tied low.

In this instance we have BOOT1 free to do whatever we want. Depending on the jumper
setting we can tie this pin high or low. The BOOT1 pin is also known as PB2, ie the
second pin on port B.

### SerialBlink

To be able to communicate with the microprocessor from a host computer is definitely handy.
In the good old days we used RS232 communications. This is still supported on the STM32
but less so on PC's. The world has moved towards USB so we better move with it.
To keep this example simple, we require character transfers between host and blue pill
RS232 style. Fortunately the USB consortium has defined a device class that is specifically
taylored to provide us with this type of communication.

The STM32 has an USB device peripheral block build in. As with any peripheral
this block will perform the correct functionality if configured properly. Libopencm3 implements
the USB stack on top of this block, so all we have to do is to send our configuration data
to libopencm3. We do this by calling `usbd_init`

The function `usb_standard_get_descriptor` in the file *usb_standard.c* is the function that
assembles the reponse packets based on information we have provided.
The first query/response concerns the device descriptor. All libopencm3 does is to copy the
device configuration structure we provided to the `usbd_init` call to the packet buffer.
The second query (the configuration) requires a bit more work. libopencm3 assembles the response
in the function `build_config_descriptor`. We provide libopencm3 with a configuration
descriptor, interface descriptor(s) and endpoint descriptor(s). Study the `build_config_descriptor`
function to learn how libopencm3 assembles the response from this information.
For the last query libopencm3 sends the strings we passed to `usbd_init`.

Apart from the generic USB configuration we add the device class specific information. In
this case it is the CDC class ACM information we need to define and handle V.25 (AT) commands.

Once all is setup you can send characters to the host using `usb_write` and receive characters
from the host using `usb_read`.

To use this in your own project you can copy the *usbserial.c* and *usbserial.h* files to
your project tree, ensure you have the USB clock divider setup correctly (`rcc_set_usbpre` function),
call `usb_setup` and start communicating using `usb_read` and `usb_write`.

### adc_temp
