/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#define ADM1032_I2C_ADDR    0x4C
#define READ_CMD_LOCAL_THERM_LIMIT 0x20
#define READ_CMD_MANUFACTURER_ID 0xFE
#define WRITE_CMD_LOCAL_THERM_LIMIT 0x20

#define STACKSIZE   1024
#define PRIORITY_EVENT  7

/*void main(void)
{
    const struct device *const i2c_device = DEVICE_DT_GET(DT_ALIAS(i2c0));
    uint8_t LocalTherm = 0x55;

    read_bytes_adm1032(i2c_device, READ_CMD_MANUFACTURER_ID); //0xFE

    //write_bytes_adm1032(i2c_device, WRITE_CMD_LOCAL_THERM_LIMIT, LocalTherm); //0x20
    //read_bytes_adm1032(i2c_device, READ_CMD_LOCAL_THERM_LIMIT); //0x20
}*/


static int write_bytes(const struct device *i2c_dev, 
                       uint8_t slave_address,
                       uint8_t cmd,
                       uint8_t *data, 
					   uint32_t num_bytes)
{
    struct i2c_msg msgs[2];

    msgs[0].buf = &cmd;
    printk("[app][wb]msgs[0].buf=0x%X\n", msgs[0].buf);
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = data;
    printk("[app][wb]msgs[1].buf=0x%X\n", msgs[1].buf);
    msgs[1].len = num_bytes;
    msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    return i2c_transfer(i2c_dev, &msgs[0], 2, slave_address);
}

static int read_bytes(const struct device *i2c_dev, 
                       uint8_t slave_address,
                       uint8_t cmd,
                       uint8_t *data, 
					   uint32_t num_bytes)
{
    struct i2c_msg msgs[2];

    msgs[0].buf = &cmd;
    printk("[app][rb]msgs[0].buf=0x%X\n", msgs[0].buf);
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE;

    msgs[1].buf = data;
    printk("[app][rb]msgs[1].buf=0x%X\n", msgs[1].buf);
    msgs[1].len = num_bytes;
    msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

    return i2c_transfer(i2c_dev, &msgs[0], 2, slave_address);
}

void write_bytes_adm1032(const struct device *I2cDev, uint8_t write_cmd, uint8_t write_data)
{
    uint8_t data[1];
    int ret;

    printk("[app][wba] Verify I2C device is ");
    if (!device_is_ready(I2cDev))
    {
        printk("not ready.\n");
        return;
    }else{
        printk("ready.\n");
    }

    data[0] = write_data;
    ret = write_bytes(I2cDev, ADM1032_I2C_ADDR, write_cmd, &data[0], 1); //0x4C
    if (ret)
    {
        printk("[app][wba] data=0x%X, write_bytes fail, error code (%d)\n", data[0], ret);
    }else{
        printk("[app][wba] data=0x%X, write_bytes success\n", data[0]);
    }
}

void read_bytes_adm1032(const struct device *I2cDev, uint8_t read_cmd)
{
    uint8_t data[1];
    int ret;

    printk("[app][rba] Verify I2C device is ");
    if (!device_is_ready(I2cDev))
    {
        printk("not ready.\n");
        return;
    }else{
        printk("ready.\n");
    }

    data[0] = 0x00;
    ret = read_bytes(I2cDev, ADM1032_I2C_ADDR, read_cmd, &data[0], 1); //0x4C
    if (ret)
    {
        printk("[app][rba] read_bytes fail, error code (%d)\n", ret);
    }else{
        printk("[app][rba] read_bytes success, data = 0x%X\n", data[0]);
    }
}

void threadAFucn(void *dummy1, void *dummy2, void *dummy3)
{
    const struct device *const i2c_device = DEVICE_DT_GET(DT_ALIAS(i2c0));

    while(1) {
        printk("threadAFucn runing\n");
        read_bytes_adm1032(i2c_device, READ_CMD_MANUFACTURER_ID); //0xFE
        //k_sleep(K_MSEC(5000));
    }
}

K_THREAD_DEFINE(threadA_id, STACKSIZE, threadAFucn, NULL, NULL, NULL, 
        PRIORITY_EVENT, 0, 0); //K_NO_WAIT

 
void main(void)
{
    printk("\nmain function entry\n");
}
