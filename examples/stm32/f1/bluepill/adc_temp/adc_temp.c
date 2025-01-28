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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "./usbserial.h"

#include <stdio.h>
#include <string.h>

uint32_t sys_ticks = 0;

/*
 * Setup functions
 */
static void clock_setup(void)
{
	/* Use a High Speed External 8Mhz crystal and run at 72Mhz */
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_AFIO);	/* Enable AFIO clock. */
	rcc_periph_clock_enable(RCC_GPIOC);	/* Enable GPIOC clock. */
	rcc_periph_clock_enable(RCC_ADC1);	/* Enable ADC clock. */

	/* The system clock is running at 72MHz, so we divide the clock by 1.5 to get a USB speed of 48MHz */
	rcc_set_usbpre( RCC_CFGR_USBPRE_PLL_CLK_DIV1_5 );
	
	/* The ADC maximum speed is 14Mhz, we set the divider to 6 to get the clock running at 12Mhz */
	rcc_set_adcpre( RCC_CFGR_ADCPRE_DIV6 );
}

static void systick_setup(void)
{
#define tick_frequency (1000)		/* 1ms period equals 1 kHz */
#define AHB_frequency (72000000)	/* system running at 72 Mhz */

	systick_set_frequency( tick_frequency, AHB_frequency );
	systick_counter_enable();
	systick_interrupt_enable();
}

static void gpio_setup(void)
{
	/* Ensure we can use SWD after we have flashed this binary */
	gpio_primary_remap( AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0 );

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
	/* Preconfigure the LED. */
	gpio_clear(GPIOC, GPIO13);	/* Switch on LED. */
}

static void adc_setup( void )
{
	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	/* We configure everything for one single injected conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	/* We can only use discontinuous mode on either the regular OR injected channels, not both */
	adc_disable_discontinuous_mode_regular(ADC1);
	adc_enable_discontinuous_mode_injected(ADC1);
	/* We want to start the injected conversion in software */
	adc_enable_external_trigger_injected(ADC1,ADC_CR2_JEXTSEL_JSWSTART);
	adc_set_right_aligned(ADC1);
	
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC to start up. */
	for( uint32_t start = sys_ticks; sys_ticks == start; )	/* wait one milli second, we only have to wait 2 ADC clock ticks so this is more than enough */
		__asm__("nop");
		
	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);

	/* Select the channel(s) we want to convert. 16 = temperature_sensor. */
	uint8_t channel_array[16] = {16};

	adc_set_injected_sequence( ADC1, 1, channel_array );	/* Set the injected sequence here, with number of channels */
}

static void setup( void )
{
	clock_setup();
	systick_setup();
	gpio_setup();
	usb_setup();
	adc_setup();
}

/* 
 * interrrupt handlers
 */
 
void sys_tick_handler( void )
{
	++sys_ticks;
}

/* LED functionality */

typedef enum { ON = 0, OFF, SLOW, NORMAL, FAST } led_mode_t;

static void led_loop( led_mode_t mode )
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

/*
 * ADC functionality
 */
static void adc_start_loop( void )
{
	static uint32_t adc_counter = 0;
	
	if( sys_ticks - adc_counter < 1000 ) /* still waiting for timeout to pass */
		return;
		
	adc_start_conversion_injected( ADC1 );
		
	adc_counter = sys_ticks;
}

static void adc_process_loop( void )
{
	if( !adc_eoc_injected( ADC1 ) )	/* Wait for end of conversion. */
		return;
		
	adc_clear_flag( ADC1, ADC_SR_JEOC );	 // clear injected end of conversion
	
	/* STM32F103x8_xB Datasheet page 79 */
#define AVG_SLOPE 0.0043		/* Volts per degree Celcius */
#define V25 1.43

#define VOLTS_PER_ADCCODE (3.3 / 4096)

	uint8_t pos = 0;
	char buffer[50] = "                ";

	uint16_t adc_reading = adc_read_injected( ADC1, 1 ); // get the result from ADC_JDR1 on ADC1 (only bottom 12bits)
	float Vsense = (float)adc_reading * VOLTS_PER_ADCCODE;
	
	if(  Vsense < 2.0 ) {
		pos = snprintf( buffer, 50, " - Under voltage %d %f\r", adc_reading, Vsense );
		usb_write( buffer, pos );
	} else if( Vsense > 3.3 ) {
		pos = snprintf( buffer, 50, " - Over voltage %f\r", Vsense );
		usb_write( buffer, pos );
	} else {
	
		float temperature = 25 + ( V25 - Vsense ) / AVG_SLOPE;	/* RM0008 page 236 */
		
		pos = snprintf( buffer, 50, " - %f Celcius\r", temperature );
		
		usb_write( buffer, pos );
	}
}

/*
 * USB functionality
 */
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

static led_mode_t usb_loop( led_mode_t mode )
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

/*
 * Outer program loop
 */
int main( void )
{
	led_mode_t mode = NORMAL;
	
	setup();
	
	while( 1 ) {
		adc_start_loop();
		led_loop( mode );
		adc_process_loop();
		mode = usb_loop( mode );
	}

	return 0;
}
