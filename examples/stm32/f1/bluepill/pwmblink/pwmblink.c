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

/* Set STM32 to 72 MHz. */
static void clock_setup( void )
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll( &rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ] );

	rcc_periph_clock_enable( RCC_AFIO );	/* Enable AFIO clock. */
	rcc_periph_clock_enable( RCC_GPIOA );	/* Enable GPIOA clock. */
	rcc_periph_clock_enable( RCC_TIM2 );	/* Enable TIM2 clock */
}

static void gpio_setup( void )
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0 );

	/*
		We want the led to be controlled by the output of timer 2 channel 1 output pin.
		So, to not double drive the led, we must ensure that PC13 is a floating
		input pin. By default this is so, so we do not need to configure anything.
		The output of Timer 2, channel 1 is available on PA0.
		In order for the pin to actually output this signal we need to connect this pin
		to the timer output. This is done by setting the alternate function configuration
		on this pin.
		Hardware wise we need to connect PA0 with PC13.
	*/
	gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
}

/*
 * We want to control the brightness of the led. We do this by altering the duty cycle
 * of a PWM signal. A low duty cycle results in a dim led, a high duty cycle in a bright led
 * Like in timed blink we want to do something on a under/over flow. This means
 * that we have to set the prescaler up appropriately. With a prescaler at 360 we change
 * the duty cycle every 5ms.
 *
 * Now for the new bit. In order to use channel 1 of TIM2 as a PWM signal accessable on
 * an external pin, we have to specify that TIM2, Channel 1 is in output compare mode,
 * responding as PWM on a match (timer_set_oc_mode).
 * For the external output to appear we have to call timer_enable_oc_output.
 * We need to set the match value, timer_set_oc_value, and optionally the polarity of
 * the output.
 * The final output depends on both the PWM mode (PWM1 or PWM2) and the set polarity
 * There are four combinations:
 *
 * PWM1 and polarity low => pin is "0" when counter < compare match else pin is "1"
 * PWM2 and polarity high => pin is "0" when counter < compare match else pin is "1"
 *
 * PWM1 and polarity high => pin is "1" when counter < compare match else pin is "0"
 * PWM2 and polarity low => pin is "1" when counter < compare match else pin is "0"
 *
 * Note: make sure that the match value is less than the period, else the compare
 * will never match.
 *
 * An optional call is the timer_enable_oc_preload call. When preload is enabled and you
 * set a new match value (timer_set_oc_value) this new match value will be used on the
 * next cycle, ie when a match occurs using the old value. If preload is disabled matching
 * switches immediately to the new match value.
 */
static void timer_setup( void )
{
	//timer_set_mode( TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_DOWN );
	timer_set_oc_mode( TIM2, TIM_OC1, TIM_OCM_PWM2 );
	timer_set_oc_value( TIM2, TIM_OC1, 10 );
	timer_enable_oc_output( TIM2, TIM_OC1 );
	timer_set_oc_polarity_low( TIM2, TIM_OC1 );
//	timer_enable_oc_preload( TIM2, TIM_OC1 );

	timer_set_prescaler( TIM2, 360 - 1 );
	timer_set_period( TIM2, 1000 );	/* auto reload register */

	timer_enable_counter( TIM2 );
}

static void setup( void )
{
	clock_setup();
	gpio_setup();
	timer_setup();
}

static void timer_loop( void )
{
	static uint32_t current_dc = 10;
	static int32_t dir = 1;

	if( ! timer_get_flag( TIM2, TIM_SR_UIF ) )
		return;

	timer_clear_flag( TIM2, TIM_SR_UIF );	/* it is our responsibility to clear the flag */

	current_dc += dir;

#if 0
	if( current_dc == 1000 )
		dir = -1;

	if( current_dc == 10 )
		dir = 1;
#else
	if( current_dc == 1000 )
		current_dc = 10;
#endif

	timer_set_oc_value( TIM2, TIM_OC1, current_dc );	/* Compare value */
}

int main( void )
{
	setup();

	while( 1 ) {

		timer_loop();
	}

	return 0;
}
