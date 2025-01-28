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

uint32_t sys_ticks = 0;

/* Set STM32 to 72 MHz. */
static void clock_setup(void)
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_AFIO);	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_GPIOC);	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_GPIOB);	/* Enable GPIOB clock */
}

static void gpio_setup(void)
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0 );

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
	/* Set GPIO2 (in GPIO port B) to 'floating input'.
	 * NB. This is not strictly nessesay since this is the default start up state
	 */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);

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

void sys_tick_handler( void )
{
	++sys_ticks;
}

static void led_toggler( void )
{
	static uint32_t start_counter = 0;
	
	if( sys_ticks - start_counter < 500 ) /* still waiting for half a second to pass */
		return;
		
	gpio_toggle( GPIOC, GPIO13 );	/* LED on/off */
		
	start_counter = sys_ticks;
}

int main(void)
{
	sys_ticks = 0;
	
	clock_setup();
	gpio_setup();
	systick_setup();

	/* do nothing */
	while (1) {
	
		if( ( gpio_port_read( GPIOB ) & GPIO2 ) )
			led_toggler();
	}

	return 0;
}
