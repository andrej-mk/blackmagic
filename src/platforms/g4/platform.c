/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2017 Uwe Bonnes bon@elektron,ikp,physik.tu-darmstadt.de
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32G4-IF
 * implementation.
 */

#include "general.h"
#include "usb.h"
#include "aux_serial.h"

#include <libopencm3/stm32/g4/adc.h>
#include <libopencm3/stm32/g4/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/flash.h>

extern uint32_t _ebss[];

static void adc_init(void);

void platform_init(void)
{
//	volatile uint32_t *magic = (uint32_t *) &_ebss;
	/* If RCC_CFGR is not at it's reset value, the bootloader was executed
	 * and SET_ADDRESS got us to this place. On G4, without further efforts,
	 * application does not start in that case.
	 * So issue an reset to allow a clean start!
	 */
	/*if (RCC_CFGR)
		scb_reset_system();
	SYSCFG_MEMRM &= ~3;*/
	/* Buttom is BOOT0, so buttom is already evaluated!*/
/*	if (((magic[0] == BOOTMAGIC0) && (magic[1] == BOOTMAGIC1))) {
		magic[0] = 0;
		magic[1] = 0;*/
		/* Jump to the built in bootloader by mapping System flash.
		   As we just come out of reset, no other deinit is needed!*/
/*		SYSCFG_MEMRM |=  1;
		scb_reset_core();
	}*/
	rcc_clock_setup_pll(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_96MHZ]);
	rcc_set_clock48_source(RCC_CCIPR_CLK48_PLLQ);
	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_CRC);
	adc_init();
	/* Disconnect USB after reset:
	 * Pull USB_DP low. Device will reconnect automatically
	 * when USB is set up later, as Pull-Up is hard wired*/
 	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO12);
	rcc_periph_reset_pulse(RST_USB); 

	// JTAG/SWD gpio setup
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TMS_PIN | TCK_PIN | TDI_PIN | TMS_DIR_PIN);
	gpio_set_output_options(JTAG_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_MED, TMS_PIN | TCK_PIN | TDI_PIN);
	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TDO_PIN);
	// NRST
	gpio_mode_setup(NRST_SENSE_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, NRST_SENSE_PIN);
	gpio_mode_setup(NRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, NRST_PIN);
	gpio_set_output_options(NRST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, NRST_PIN);
	gpio_set(NRST_PORT, NRST_PIN);
	// Power
	gpio_set_output_options(PWR_BR_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, PWR_BR_PIN);
	gpio_set(PWR_BR_PORT, PWR_BR_PIN);
	gpio_mode_setup(PWR_BR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_BR_PIN);
	// LED
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
					LED_UART | LED_IDLE_RUN | LED_ERROR | LED_BOOTLOADER);

	platform_timing_init();
	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	blackmagic_usb_init();
	aux_serial_init();
}

void platform_nrst_set_val(bool assert)
{
	gpio_set_val(NRST_PORT, NRST_PIN, assert);
}

bool platform_nrst_get_val(void)
{
	return (gpio_get(NRST_SENSE_PORT, NRST_SENSE_PIN)) != 0;
}

uint32_t platform_target_voltage_sense(void)
{
	const uint8_t channel = 2;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);
	adc_start_conversion_regular(ADC1);
	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));
	uint32_t val = adc_read_regular(ADC1); /* 0-4095 */
	return (val * 97) / 8190;
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	uint32_t val = platform_target_voltage_sense();
	if (val > 60) {
		ret[0] = 'x';
		ret[2] = 'x';
	} else {
		ret[0] = '0' + val / 10;
		ret[2] = '0' + val % 10;
	}
	return ret;
}

void platform_request_boot(void)
{
	/* Bootloader cares for reenumeration */
	scb_reset_system();
}

void platform_target_clk_output_enable(bool enable)
{
	(void)enable;
}

bool platform_target_get_power(void) 
{
	return !gpio_get(PWR_BR_PORT, PWR_BR_PIN);
}

void platform_target_set_power(bool power)
{
	if (power) {
		gpio_clear(PWR_BR_PORT, PWR_BR_PIN);
	} else {
		gpio_set(PWR_BR_PORT, PWR_BR_PIN);
	}
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);
	RCC_CCIPR |= RCC_CCIPR_ADC12_SYS << RCC_CCIPR_ADC12_SHIFT;
	adc_disable_deeppwd(ADC1);
	adc_enable_regulator(ADC1);
	for (int i = 0; i < 200000; i++)
		__asm__("nop");
	adc_power_off(ADC1);
	adc_calibrate(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_92DOT5CYC);
	adc_power_on(ADC1);
}
