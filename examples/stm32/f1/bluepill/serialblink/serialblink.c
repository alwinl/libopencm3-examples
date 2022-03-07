/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2022 Alwin Leerling <dna.leerling@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "./usbserial.h"

#include <string.h>

uint32_t sys_ticks = 0;

static void clock_setup(void)
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_AFIO);	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_GPIOC); /* Enable GPIOC clock. */
	
	/* The system clock is running at 72MHz, so we divide the clock by 1.5 to get a USB speed of 48MHz */
	rcc_set_usbpre( RCC_CFGR_USBPRE_PLL_CLK_DIV1_5 );
}

static void gpio_setup(void)
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST, 0 );

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
	/* Preconfigure the LED. */
	gpio_clear(GPIOC, GPIO13);	/* Switch on LED. */
}

static void systick_setup(void)
{
#define tick_frequency (1000)		/* 1ms period equals 1 kHz */
#define AHB_frequency (72000000)	/* system running at 72 Mhz */

	systick_set_frequency( tick_frequency, AHB_frequency );
	systick_counter_enable();
	systick_interrupt_enable();
}

static void setup( void )
{
	clock_setup();
	systick_setup();
	gpio_setup();
	usb_setup();
}

void sys_tick_handler( void )
{
	++sys_ticks;
}

/* LED functionality */

typedef enum { ON = 0, OFF, SLOW, NORMAL, FAST } mode_t;

static void led_loop( mode_t mode )
{
	static uint32_t start_counter = 0;
	uint16_t timeout = 0;
	
	switch( mode ) {
	case ON:
		gpio_clear(GPIOC, GPIO13);	/* Switch on LED. */
		break;
	case OFF:
		gpio_set(GPIOC, GPIO13);	/* Switch off LED. */
		break;
	case SLOW:
		timeout = 1000;
		break;
	case NORMAL:
		timeout = 500;
		break;
	case FAST:
		timeout = 250;
		break;
	}
	
	if( (timeout == 0) || ( sys_ticks - start_counter < timeout ) )
		return;
		
	gpio_toggle( GPIOC, GPIO13 );	/* LED on/off */
		
	start_counter = sys_ticks;
}



static char * messages[] = 
{
	"Turned the LED on\n\r",
	"Turned the LED off\n\r",
	"Blinking the LED slowly\n\r",
	"Blinking the LED\n\r",
	"Blinking the LED fast\n\r"
};

static char * startup_message = 
{
	"\n\n\n\rWelcome to SerialBlink\n\n\r"
	"Please select from the following options:\n\r"
	"o or O : Turn the LED on\n\r"
	"x or X : Turn the LED off\n\r"
	"s or S : Blink the LED slowly\n\r"
	"n or N : Blink the LED normally\n\r"
	"f or F : Blink the LED fast\n\r"
	"h or H : This help message\n\r"
	"\n\r"
};

static mode_t usb_loop( mode_t mode )
{
	char usb_buffer[64];
	int rx_bytes;

	rx_bytes = usb_read( usb_buffer, 64 );
	if( rx_bytes == 0 )
		return mode;
		
	for( int i=0; i<rx_bytes; ++i ) {
		switch( usb_buffer[i] ) {
		case 'O':
		case 'o': mode = ON; break;
		case 'X':
		case 'x': mode = OFF; break;
		case 'S':
		case 's': mode = SLOW; break;
		case 'N':
		case 'n': mode = NORMAL; break;
		case 'F':
		case 'f': mode = FAST; break;
		case 'h':
		case 'H':
		default:
			usb_write( startup_message, strlen(startup_message)  );
		}
		usb_write( messages[mode], strlen(messages[mode])  );
	}

	return mode;
}


int main(void)
{
	mode_t mode = NORMAL;
	
	setup();
	
	while (1) {
		led_loop( mode );
		mode = usb_loop( mode );
	}

	return 0;
}
