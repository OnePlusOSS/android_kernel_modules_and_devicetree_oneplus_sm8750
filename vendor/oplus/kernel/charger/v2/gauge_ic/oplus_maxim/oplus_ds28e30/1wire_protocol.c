// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */
#define pr_fmt(fmt) "[1WIRE_PROTOCOL]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <oplus_chg_ic.h>

#include "1wire_protocol.h"
#include "ds28e30.h"

#define ONE_WIRE_CONFIG_OUT		writel_relaxed(g_onewire_data->onewire_gpio_cfg_out_val, g_onewire_data->gpio_cfg_out_reg)
#define ONE_WIRE_CONFIG_IN		writel_relaxed(g_onewire_data->onewire_gpio_cfg_in_val, g_onewire_data->gpio_cfg_in_reg)
#define ONE_WIRE_OUT_HIGH		writel_relaxed(g_onewire_data->onewire_gpio_level_high_val, g_onewire_data->gpio_out_high_reg)
#define ONE_WIRE_OUT_LOW		writel_relaxed(g_onewire_data->onewire_gpio_level_low_val, g_onewire_data->gpio_out_low_reg)

/* #define WIRE_TIMEING_DEBUG */ 	/* if defined, then print 1-wire timing variants to the log file */
static struct onewire_gpio_data *g_onewire_data;

int write_get_pin(void);
void delay_us(unsigned int delay_us);
void delay_ms(unsigned int delay_ms);

/* Basic 1-Wire functions */
int ow_reset(void);
void write_byte(unsigned char byte_value);
void write_bit(unsigned char bit_value);
unsigned char read_bit(void);
unsigned char read_byte(void);

/*******************************************************************************
 get 1-wire GPIO input
*******************************************************************************/
int write_get_pin(void)
{
	int ret = 0;
	ret = gpio_get_value(g_onewire_data->ow_gpio);
	return ret;
}
/*****************************************************************************
delay us subroutine
*****************************************************************************/
void delay_us(unsigned int delay_us)    /* 1US */
{
	udelay(delay_us);
}

static struct timespec get_current_time(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return ts;
}

void delay_ns(unsigned int delay_ns)
{
	ndelay(delay_ns);
}

/*****************************************************************************
delay ms subroutine
*****************************************************************************/
void delay_ms(unsigned int delay_ms)    /* 1US */
{
	mdelay(delay_ms);
}

/*****************************************************************************
Basic 1-Wire functions
Reset all of the devices on the 1-Wire Net and return the result.
Returns: TRUE(1):  presense pulse(s) detected, device(s) reset
      FALSE(0): no presense pulses detected
*****************************************************************************/
int ow_reset(void)
{
	unsigned int presence = 0;
	unsigned int value = 0;
	unsigned int gpio_value = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_LOW;
	delay_us(54);	/* wait 54 us */
	ONE_WIRE_CONFIG_IN;
	delay_us(9);	/* wait 9 us for presence */
	value = readl_relaxed(g_onewire_data->gpio_in_reg);
	presence = !(value >> g_onewire_data->gpio_addr_offset & 0x1);
	chg_info(" %s ++++gpio_value 0x%x value 0x%x presence 0x%x", __func__, gpio_value, value, presence);
	delay_us(50);	/* wait 50 us */
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_IN;
	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);

	return(presence); /* presence signal returned */
}

/* Send 1 bit of communication to the 1-Wire Net.
 The parameter 'sendbit' least significant bit is used.
 'bitvalue' - 1 bit to send (least significant byte)
 */
void write_bit(unsigned char bitval)
{
	struct timespec w_start_ns, w_end_ns;
	long w_diff_ns;
	unsigned char i;
	w_start_ns = get_current_time();

	if (g_onewire_data->maxim_romid_crc_support) {
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;			/* Output Low '0' */
		w_start_ns = get_current_time();
		ONE_WIRE_OUT_LOW;			/* Output Low '0' */
		for (i = 0; i < DELAY_CHECK_NUM; i++) {
			w_end_ns = get_current_time();
			w_diff_ns = w_end_ns.tv_nsec - w_start_ns.tv_nsec;
			if (w_diff_ns >= DELAY_WRITE_BIT_OUT_LOW)
				break;
		}
		/* delay_ns(g_onewire_data->write_begin_low_level_time); */
		if (bitval != 0) {
			ONE_WIRE_OUT_HIGH;	/* ONE_WIRE_OUT_HIGH; set 1-wire to logic high if bitval='1' */
			w_end_ns = get_current_time();
			w_diff_ns = w_end_ns.tv_nsec - w_start_ns.tv_nsec;
			check_womid_bit(w_diff_ns);
		}
		delay_us(12);		/*  waiting for 12us */
		ONE_WIRE_OUT_HIGH;
#ifdef WIRE_TIMEING_DEBUG
		if (bitval != 0)
			chg_info("%s: Write_1_Low_time=%ld\n", __func__, w_diff_ns);
		w_end_ns = get_current_time();
		w_diff_ns = w_end_ns.tv_nsec - w_start_ns.tv_nsec;
		chg_info("%s: Write_bit_time=%ld\n", __func__, w_diff_ns);
#endif
		delay_us(8);			/*  waiting for 6us to recover to logic high */
		/* delay_us(g_onewire_data->write_relese_ic_time); */
	} else {
		ONE_WIRE_OUT_LOW;
		delay_us(1);			/* keeping logic low for 1 us */
		if (bitval != 0)
			ONE_WIRE_OUT_HIGH;	/* ONE_WIRE_OUT_HIGH; set 1-wire to logic high if bitval='1' */
		delay_us(10);			/*  waiting for 10us */
		ONE_WIRE_OUT_HIGH;
		delay_us(5);			/*  waiting for 5us to recover to logic high */
	}
}

/* Send 1 bit of read communication to the 1-Wire Net and and return the
 result 1 bit read from the 1-Wire Net.
 Returns:  1 bits read from 1-Wire Net
 */
unsigned char read_bit(void)
{
	unsigned int vamm = 0;
	unsigned int value;
	unsigned char i;
	struct timespec r_start_ns;
	struct timespec r_end_ns;
	long r_diff_ns;
	long r_diff_ns_1;

	ONE_WIRE_CONFIG_OUT;
	if (g_onewire_data->maxim_romid_crc_support) {
		/* Execute output '0' 5 times*/
		ONE_WIRE_OUT_HIGH;
		ONE_WIRE_OUT_LOW;	/* output low '0' */
		r_start_ns = get_current_time();
		for (i = 0; i < DELAY_CHECK_NUM; i++) {
			r_end_ns = get_current_time();
			r_diff_ns_1 = r_end_ns.tv_nsec - r_start_ns.tv_nsec;
			if (r_diff_ns_1 >= DELAY_READ_BIT_OUT_LOW)
				break;
		}
		/* set 1-wire as input */
		ONE_WIRE_CONFIG_IN;
		for (i = 0; i < 7; i++) {
			value = readl_relaxed(g_onewire_data->gpio_in_reg);
			value = (value >> g_onewire_data->gpio_addr_offset) & 0x1;
			vamm += value;
			if (i == 1)
				r_end_ns = get_current_time();
		}
		r_diff_ns = r_end_ns.tv_nsec - r_start_ns.tv_nsec;
#ifdef WIRE_TIMEING_DEBUG
		chg_info("%s: vamm=%d, read bit low time tRL=%ld, read bit end sample time=%ld\n", __func__, vamm, r_diff_ns_1, r_diff_ns);
#endif
		/* set threshold to justify logic '1' or '0' */
		if (vamm > 6)
			vamm = 1;
		else
			vamm = 0;
	} else {
		ONE_WIRE_OUT_LOW;
		ONE_WIRE_OUT_LOW;
		if (g_onewire_data->maxim_trl_ndelay != 0) {
			delay_ns(g_onewire_data->maxim_trl_ndelay);
		}
		ONE_WIRE_CONFIG_IN;
		delay_ns(500);	/* wait 500 ns */
		value = readl_relaxed(g_onewire_data->gpio_in_reg);
		vamm = value >> g_onewire_data->gpio_addr_offset & 0x1;
	}
	delay_us(6);	/* waiting for 6us Keep GPIO at the input state */
	ONE_WIRE_OUT_HIGH;
	ONE_WIRE_CONFIG_OUT;
	if (g_onewire_data->maxim_romid_crc_support)
		check_romid_bit(r_diff_ns, vamm, i);
	delay_us(6);	/* waiting for 6us Keep GPIO at the output state */
	return(vamm);	/*  return value of 1-wire dat pin */
}


/* Send 8 bits of communication to the 1-Wire Net and verify that the
 8 bits read from the 1-Wire Net is the same (write operation).
 The parameter 'sendbyte' least significant 8 bits are used.
 'val' - 8 bits to send (least significant byte)
 Returns:  TRUE: bytes written and echo was the same
           FALSE: echo was not the same
 */
void write_byte(unsigned char val)
{
	unsigned char i;
	unsigned char temp;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);
	ONE_WIRE_CONFIG_OUT;
	for (i = 0; i < 8; i++) {	/*  writes byte, one bit at a time */
		temp = val >> i;		/*  shifts val right */
		temp &= 0x01;			/*  copy that bit to temp */
		write_bit(temp);		/*  write bit in temp into */
	}
	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);
}

/* Send 8 bits of read communication to the 1-Wire Net and and return the
 result 8 bits read from the 1-Wire Net.
 Returns:  8 bits read from 1-Wire Net
 */
unsigned char read_byte(void)
{
	unsigned char i;
	unsigned char value = 0;
	unsigned long flags;

	raw_spin_lock_irqsave(&g_onewire_data->lock, flags);
	for (i = 0; i < 8; i++)
		if (read_bit()) value |= 0x01 << i;	/* reads byte in, one byte at a time and then shifts it left */

	raw_spin_unlock_irqrestore(&g_onewire_data->lock, flags);

	return(value);
}

int onewire_init(struct onewire_gpio_data *onewire_data)
{
	chg_info("%s entry", __func__);
	g_onewire_data = (struct onewire_gpio_data *)kmalloc(sizeof(struct onewire_gpio_data), GFP_KERNEL);
	if (!g_onewire_data) {
		chg_err("Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (onewire_data == NULL) {
		kfree(g_onewire_data);
		chg_info("%s onewire_data is null return", __func__);
		return -1;
	}
	raw_spin_lock_init(&g_onewire_data->lock);

	g_onewire_data->gpio_cfg_out_reg = onewire_data->gpio_cfg_out_reg;
	g_onewire_data->gpio_cfg_in_reg = onewire_data->gpio_cfg_in_reg;
	g_onewire_data->gpio_out_high_reg = onewire_data->gpio_out_high_reg;
	g_onewire_data->gpio_out_low_reg = onewire_data->gpio_out_low_reg;
	g_onewire_data->gpio_in_reg = onewire_data->gpio_in_reg;
	g_onewire_data->gpio_addr_offset = onewire_data->gpio_addr_offset;
	g_onewire_data->onewire_gpio_cfg_out_val = onewire_data->onewire_gpio_cfg_out_val;
	g_onewire_data->onewire_gpio_cfg_in_val = onewire_data->onewire_gpio_cfg_in_val;
	g_onewire_data->onewire_gpio_level_high_val = onewire_data->onewire_gpio_level_high_val;
	g_onewire_data->onewire_gpio_level_low_val = onewire_data->onewire_gpio_level_low_val;
	g_onewire_data->write_begin_low_level_time = onewire_data->write_begin_low_level_time;
	g_onewire_data->write_relese_ic_time = onewire_data->write_relese_ic_time;
	g_onewire_data->maxim_romid_crc_support = onewire_data->maxim_romid_crc_support;
	g_onewire_data->maxim_trl_ndelay = onewire_data->maxim_trl_ndelay;
	ONE_WIRE_CONFIG_OUT;
	ONE_WIRE_OUT_HIGH;
	delay_ms(10);
	return 0;
}

void onewire_set_gpio_config_in(void)
{
	ONE_WIRE_CONFIG_IN;
	chg_info("%s set gpio in", __func__);
}

void onewire_set_gpio_config_out(void)
{
	ONE_WIRE_CONFIG_OUT;
	chg_info("%s set gpio out", __func__);
}

bool get_maxim_romid_crc_support(void)
{
	return g_onewire_data->maxim_romid_crc_support;
}
