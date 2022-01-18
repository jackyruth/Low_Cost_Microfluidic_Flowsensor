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
#define T_ON  100   //0.5 sec
#define T_OFF 10000  //10 sec

// Zephyr Sensor Def
#define I2C_DEV DT_LABEL(DT_ALIAS(i2c))

// Register Values
#define TEMP_CFG_VAL 0x00 //0x220 is default
#define TEMP_RST     0x06

// Custom Driver Addresses
#define TMP117_RESOLUTION 78125
#define TMP117_ADDR 	0x48
#define I2C_BROADCAST   0x00
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
int16_t config;
int16_t data_rdy;
int16_t temp;
uint32_t prev_time;
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
	i2c = device_get_binding(I2C_DEV);

	/* === Setup TMP117 === */
	config = sys_cpu_to_be16(TEMP_CFG_VAL);
	code = i2c_burst_write(i2c, TMP117_ADDR, TEMP_CFG, &config, 2);
        __ASSERT(code == 0, "ERROR: i2c_read, exit with code %u\n", code);
	
	code = i2c_burst_read(i2c, TMP117_ADDR, TEMP_CFG, &config, 2);
        __ASSERT(code == 0, "ERROR: i2c_read, exit with code %u\n", code);
	config = sys_be16_to_cpu(config);
	printk("NEW CONFIG: %d\r\n", config);


	/* === Counter for Heater === */
	counter = 0;
	while (1) {
		if(counter == 1){
        		gpio_pin_set(led_pin, LED0_PIN, true);
			gpio_pin_set(out1_in2, OUT1IN2_PIN, true);
			heat_sig = 100;
		}
		else if(counter == (T_ON+1)/16){
        		gpio_pin_set(led_pin, LED0_PIN, false);
			gpio_pin_set(out1_in2, OUT1IN2_PIN, false);
			heat_sig = 0;
		}
		else if(counter == (T_OFF+T_ON+1)/16){
			counter = 0;
		}
		counter++;

		/* === Start Temperature Extraction === */
		prev_time = k_uptime_get_32();

		/* Reset TMP117 */
		config = sys_cpu_to_be16(TEMP_RST);
		code = i2c_burst_write(i2c, I2C_BROADCAST, TEMP_CFG, &config, 2);

		/* Setup TMP117 */
		config = sys_cpu_to_be16(TEMP_CFG_VAL);
		code = i2c_burst_write(i2c, TMP117_ADDR, TEMP_CFG, &config, 2);

		/* Check for Data Ready */
		while((data_rdy & 0x2000) == 0){
			code = i2c_burst_read(i2c, TMP117_ADDR, TEMP_CFG, &data_rdy, 2);
        		__ASSERT(code == 0, "ERROR: i2c_read, exit with code %u\n", code);
			data_rdy = sys_be16_to_cpu(data_rdy);
		}
		data_rdy = 0;

		/* Record Data */
        	code = i2c_burst_read(i2c, TMP117_ADDR, TEMP_RSLT, &temp, 2);
        	__ASSERT(code == 0, "ERROR: i2c_read, exit with code %u\n", code);
		temp = sys_be16_to_cpu(temp);
		tmp = ((int16_t)temp * (int32_t)TMP117_RESOLUTION) / 10;
		while(k_uptime_get_32()-prev_time<15);
		printk("%u %d.%d %d\n", k_uptime_get_32(), tmp/1000000, tmp%1000000, heat_sig);
		// printk("%d.%d %d\n", tmp/1000000, tmp%1000000, heat_sig);
	}
}