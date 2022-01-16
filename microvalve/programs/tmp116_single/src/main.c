/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE 	DT_ALIAS(led0)
#define LED0		DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)

#define OUT1IN1_NODE 	DT_NODELABEL(out1_in1)
#define OUT1IN1		DT_GPIO_LABEL(OUT1IN1_NODE, gpios)
#define OUT1IN1_PIN	DT_GPIO_PIN(OUT1IN1_NODE, gpios)
#define OUT1IN1_FLAGS	DT_GPIO_FLAGS(OUT1IN1_NODE, gpios)

#define OUT1IN2_NODE 	DT_NODELABEL(out1_in2)
#define OUT1IN2		DT_GPIO_LABEL(OUT1IN2_NODE, gpios)
#define OUT1IN2_PIN	DT_GPIO_PIN(OUT1IN2_NODE, gpios)
#define OUT1IN2_FLAGS	DT_GPIO_FLAGS(OUT1IN2_NODE, gpios)

/* Delay roughly in 100 ms */
#define T_ON 1            //0.5 sec
#define T_OFF 80  //10 sec

// Zephyr Sensor Def
#define TMP117_0x48 true
#define I2C_DEV DT_LABEL(DT_ALIAS(i2c))

// Custom Driver Addresses
#define TMP117_RESOLUTION 78125
#define TMP117_ADDR 	0x49
#define TEMP_RSLT 	0x00
#define TEMP_CFG  	0x01
#define TEMP_THighLimit 0x02
#define TEMP_TLowLimit  0x03
#define TEMP_EEPROM_UL  0x04
#define TEMP_EEPROM1  	0x05
#define TEMP_EEPROM2  	0x06
#define TEMP_OFFSET  	0x07
#define TEMP_EEPROM3  	0x08
#define TEMP_DEVID  	0x0F

// Custom Driver Commands
// -- Configuration
#define CC_MOD 		0 << 10
#define SD_MOD 		1 << 10
#define OS_MOD 		3 << 10
#define CONVERSION	0 << 7
#define AVG             0 << 5
#define Therm_Alert     1 << 4
#define POL             0 << 3
#define Alert_Sel       0 << 2
#define RST    		1 << 1
// -- High Limit
// -- Low Limit
// -- EEPROM
// -- OFFSET
#define OFFSET		0
uint8_t tx_byte;
uint8_t rx_bytes[6];
int16_t serial_num;
int16_t temp;
const struct device *i2c;
int code;
int counter;
struct i2c_slave_config tmp117_config;

void main(void)
{
	int length;
	int32_t tmp;
	int32_t heat_sig=0;
	int32_t millis_time;
	const struct device *dev;
	const struct device *led_pin;
	const struct device *out1_in1;
	const struct device *out1_in2;
	struct sensor_value temp_value;
	struct sensor_value offset_value;

        bool led_is_on = true;
        int ret;   
//-- LED
        led_pin = device_get_binding(LED0);
        if (led_pin == NULL) {
                return;
        }

        ret = gpio_pin_configure(led_pin, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
        if (ret < 0) {
                return;
        }
//-- out1_in1
	out1_in1 = device_get_binding(OUT1IN1);
	if (out1_in1 == NULL) {
		return;
	}

	ret = gpio_pin_configure(out1_in1, OUT1IN1_PIN, GPIO_OUTPUT_ACTIVE | OUT1IN1_FLAGS);
	if (ret < 0) {
		return;
	}
//-- out1_in2
	out1_in2 = device_get_binding(OUT1IN2);
	if (out1_in2 == NULL) {
		return;
	}

	ret = gpio_pin_configure(out1_in2, OUT1IN2_PIN, GPIO_OUTPUT_ACTIVE | OUT1IN2_FLAGS);
	if (ret < 0) {
		return;
	}
	gpio_pin_set(out1_in1, OUT1IN1_PIN, false);
	gpio_pin_set(out1_in2, OUT1IN2_PIN, false);

//-- I2C Device
	if(TMP117_0x48){
		dev = device_get_binding(DT_LABEL(DT_INST(0, ti_tmp116)));
		__ASSERT(dev != NULL, "Failed to get TMP116 device binding");
		printk("Device %s - %p is ready\n", dev->name, dev);
		offset_value.val1 = 0;
		offset_value.val2 = 0;
		ret = sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP,
			      	SENSOR_ATTR_OFFSET, &offset_value);
		if (ret) {
			printk("sensor_attr_set failed ret = %d\n", ret);
			printk("SENSOR_ATTR_OFFSET is only supported by TMP117\n");
		}
	}
	counter = 0;
	while (1) {
		if(counter == 1){
        		gpio_pin_set(led_pin, LED0_PIN, true);
			gpio_pin_set(out1_in2, OUT1IN2_PIN, true);
			heat_sig = 100;
		}
		else if(counter == T_ON+1){
        		gpio_pin_set(led_pin, LED0_PIN, false);
			gpio_pin_set(out1_in2, OUT1IN2_PIN, false);
			heat_sig = 0;
		}
		else if(counter == T_OFF+T_ON+1){
			counter = 0;
		}
		counter++;

		if(TMP117_0x48){
			ret = sensor_sample_fetch(dev);
			if (ret) {
				printk("Failed to fetch measurements (%d)\n", ret);
				return;
			}

			ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
					 &temp_value);
			if (ret) {
				printk("Failed to get measurements (%d)\n", ret);
				return;
			}
		}
		printk("%d.%d %d\n", temp_value.val1, temp_value.val2,heat_sig);
		k_msleep(100);
	}
}
