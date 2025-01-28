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

/* Set STM32 to 72 MHz. */
static void clock_setup( void )
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll( &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ] );

	rcc_periph_clock_enable( RCC_AFIO );	/* Enable AFIO clock. */
	rcc_periph_clock_enable( RCC_GPIOC );	/* Enable GPIOC clock. */
}

static void gpio_setup( void )
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0 );

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode( GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13 );

	/* Preconfigure the LED. */
	gpio_clear( GPIOC, GPIO13 );	/* Switch on LED. */
}

int main( void )
{
	clock_setup();
	gpio_setup();

	/* Blink the LED (PC13) on the board. */
	while( 1 ) {
		
		gpio_toggle( GPIOC, GPIO13 );	/* LED on/off */
		
		for( int i = 0; i < 800000; i++ )	/* Wait a bit. */
			__asm__( "nop" );
	}

	return 0;
}
