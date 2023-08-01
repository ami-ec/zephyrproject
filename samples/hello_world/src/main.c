/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc_pins.h>

#define THREADS_STACK_SIZE 1000
char threads_stacks[2][THREADS_STACK_SIZE];
static struct k_thread threads_struct_data[2];

const struct device *gpio_dev_out;
const struct device *gpio_dev_in;

void thread_1_function_entry_point(void *arg1, void *arg2, void *arg3)
{

	gpio_dev_out = DEVICE_DT_GET(DT_NODELABEL(gpio_100_136));
	gpio_dev_in = DEVICE_DT_GET(DT_NODELABEL(gpio_200_236));

	int status = 0;
	int PinValue = 0;

	if(gpio_dev_out != NULL && gpio_dev_in != NULL)
	{
		status = gpio_pin_configure(gpio_dev_out, MCHP_GPIO_132, GPIO_OUTPUT);
		printk("[debug][thread_1] MCHP_GPIO_132 config = 0x%X\n", status);

		status = gpio_pin_configure(gpio_dev_in, MCHP_GPIO_222, GPIO_INPUT);
		printk("[debug][thread_1] MCHP_GPIO_222 config = 0x%X\n", status);

		while(1)
		{
			printk("[debug][thread_1] GPIO_132_Pin = 1\n");
			gpio_pin_set_raw(gpio_dev_out, MCHP_GPIO_132, 1);
			k_msleep(500);

			gpio_port_get_raw(gpio_dev_in, &PinValue);
			printk("[debug][thread_1] GPIO_222_Pin = 0x%X\n", PinValue);
			k_msleep(500);

			printk("[debug][thread_1] GPIO_132_Pin = 0\n");
			gpio_pin_set_raw(gpio_dev_out, MCHP_GPIO_132, 0);
			k_msleep(500);

			gpio_port_get_raw(gpio_dev_in, &PinValue);
			printk("[debug][thread_1] GPIO_222_Pin = 0x%X\n", PinValue);
			k_msleep(500);
		}
	}
	else
	{
		printk("[debug][thread_1] Not found GPIO device.\n");
	}	
}

void main(void)
{
	printk("\nCreate thread_1\n");
	
	k_tid_t thread_1_tid = NULL;
	thread_1_tid = k_thread_create(&threads_struct_data[0], 
							&threads_stacks[0][0],
                            THREADS_STACK_SIZE,
                            thread_1_function_entry_point,
                            NULL, NULL, NULL,
                            K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
}
