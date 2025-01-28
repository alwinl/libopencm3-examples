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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

/* Set STM32 to 72 MHz. */
static void clock_setup( void )
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll( &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ] );

	rcc_periph_clock_enable( RCC_AFIO );	/* Enable AFIO clock. */
	rcc_periph_clock_enable( RCC_GPIOC );	/* Enable GPIOC clock. */
	rcc_periph_clock_enable( RCC_TIM2 );	/* Enable TIM2 clock */
}

static void gpio_setup(void)
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0 );

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode( GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13 );

	/* Preconfigure the LED. */
	gpio_clear( GPIOC, GPIO13 );	/* Switch on LED. */
}

/*
 * We want the led to flash at a frequency of 1 Hz with a 50% duty cycle.
 * Thus we need to toggle the led every 500ms.
 * The clock runs at 72MHz which means that we have to toggle every 36 Mega ticks
 * If we set the prescaler to 36000 and count up from 0 to 1000 we can toggle
 * every time the clock times out, ie generates an underflow event.
 */
static void timer_setup( void )
{
	nvic_enable_irq( NVIC_TIM2_IRQ );
	nvic_set_priority( NVIC_TIM2_IRQ, 0 );

	timer_set_prescaler( TIM2, 36000 - 1 );
	timer_set_period( TIM2, 1000 );	/* auto reload register */

	timer_enable_irq( TIM2, TIM_DIER_UIE );

	timer_enable_counter( TIM2 );
}

static void setup( void )
{
	clock_setup();
	gpio_setup();
	timer_setup();
}

void tim2_isr( void )
{
	gpio_toggle( GPIOC, GPIO13 );	/* LED on/off */

	/* Note: We cannot use timer_clear_flag here
	 * timer_clear_flag sets the status register to the (inverted) passed parameter,
	 * it does not AND the register with the  (inverted) passed parameter.
	 *
	 * ie, libopencm3 does
	 * 		TIM_SR(handle) = ~parameter;
	 * instead of
	 * 		TIM_SR(handle) &= ~parameter;
	 *
	 * This does make a difference to the other bits
	 */
	//timer_clear_flag( TIM2, TIM_SR_UIF );
	TIM_SR(TIM2) &= ~TIM_SR_UIF;	/* it is our responsibility to clear the flag */
}

int main(void)
{
	setup();

	while( 1 ) {
		/* do nothing */
	}

	return 0;
}
