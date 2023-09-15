/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdlib.h>

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>
#include <MEC1501hsz.h>

struct i2c_xec_config {
    uint32_t port_sel;
    uint32_t base_addr;
    uint8_t girq_id;
    uint8_t girq_bit;
};

#define DBG_I2C_REGISTER_DISPLAY_ENABLE 1
#define DBG_I2C_LOG_DISPLAY_ENABLE 1
#define MALLOC_ENABLE 0
#define TEST_TIMES 8
#define TOTAL_I2C_DATA_TO_SEND 32

const struct device *const i2c_dev_sequence[] = {
    DEVICE_DT_GET(DT_ALIAS(i2c1)),
    DEVICE_DT_GET(DT_ALIAS(i2c7))
};

#define SIZE_I2C_DEV_SEQ (sizeof(i2c_dev_sequence)/sizeof(i2c_dev_sequence[0]))

struct i2c_target_test_info {
    struct i2c_target_callbacks callbacks;
    const uint16_t address;
};

static struct i2c_target_test_info i2c_target_test_info[SIZE_I2C_DEV_SEQ] = {
    { .callbacks = { NULL },
        .address = 0x4C,
    },
    { .callbacks = { NULL },
        .address = 0x4C,
    },
};

void i2c_registers_disaplay(const struct device *const dev,
                            const char *pre_display_msg) {
#if DBG_I2C_REGISTER_DISPLAY_ENABLE == 0
    ARG_UNUSED(dev);
    ARG_UNUSED(pre_display_msg);
#else
    const uint8_t reg_offset_skip_check[] = {
        0x08, 0x1c, 0x3c, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c, 0x68
    };
    const uint8_t end_of_reg_offset = 0x68;
    const struct i2c_xec_config *mchp_config = dev->config;
    uint32_t base_address = (uint32_t)mchp_config->base_addr;
    uint32_t value = 0;

    if (pre_display_msg != NULL) {
        printk("'033[1;36m");
        printk("%s", pre_display_msg);
        printk("'033[0m\n");
    }

    printk("%s: registers\n", dev->name);
    printk("\033[1;33m");
    printk("| %7s | %12s |\n", "OFFSET", "VALUE");

    for (int offset = 0; offset <= end_of_reg_offset; offset = offset + 4) {
        for (int i = 0; i < sizeof(reg_offset_skip_check); i++) {
            if (offset == reg_offset_skip_check[i]) {
                goto skip_check;
            }
        }

        value = sys_read32(base_address + offset);
        printk("| 0x%-5x | 0x%-10X |\n", offset, value);
skip_check:
        continue;
    }
    printk("\033[0m\n");
#endif
}

void dump_unregister_i2c_target(void) {
    printk("ensure all device in master state," \
           "dump to unregister all i2c target device\n");

    for (int i = 0; i < SIZE_I2C_DEV_SEQ; i++) {
        i2c_target_unregister(i2c_dev_sequence[i], NULL);
    }
}


struct i2c_target_config dump_i2c_target_config[SIZE_I2C_DEV_SEQ] = { NULL };
uint8_t dump_buf[TOTAL_I2C_DATA_TO_SEND] = { 0 };

void i2c_target_and_source_test(void) {
    struct i2c_target_config \
    *i2c_target_config[SIZE_I2C_DEV_SEQ] = { NULL };
    size_t i2c_src_idx = 0;
    uint16_t i2c_src_addr = 0;
    size_t i2c_target_idx = 0;
    int ret = -1;
    uint8_t *buf = NULL;

    for (int i = 0; i < SIZE_I2C_DEV_SEQ; i++) {
        if (!device_is_ready(i2c_dev_sequence[i])) {
#if DBG_I2C_REGISTER_DISPLAY_ENABLE
            printk("\033[1;31m");
            printk("idx %d i2c: Device is not ready\n", i);
            printk("\033[0m\n");
#endif
            return;
        }

        printk("%d) %s: device is ready\n", i, i2c_dev_sequence[i]->name);
    }

    for (int i = 0; i < SIZE_I2C_DEV_SEQ; i++) {
#if MALLOC_ENABLE
        i2c_target_config[i] = (struct i2c_target_config *)malloc(
            sizeof(struct i2c_target_config));
        if (!i2c_target_config[i]) {
            printk("\033[1;31m");
            printk("%s: Error allocating memory for i2c_target_config\n",
                   i2c_dev_sequence[i]->name);
            printk("\033[0m\n");
            goto free;
        }
#else
        i2c_target_config[i] = &dump_i2c_target_config[i];
#endif
    }


#if MALLOC_ENABLE
    buf = (uint8_t *)malloc(TOTAL_I2C_DATA_TO_SEND);

    if (!buf) {
        printk("\033[1;31m");
        printk("Failed to allocating memory for i2c send buffer\n");
        printk("\033[0m\n");
        goto free;
    }
#else
    buf = &dump_buf[0];
#endif

    for (int i = 0; i < TOTAL_I2C_DATA_TO_SEND; i++) {
        buf[i] = i+1;
    }

    for (int test_seq = 0; test_seq < TEST_TIMES; test_seq++) {
        dump_unregister_i2c_target();
        size_t idx = test_seq % SIZE_I2C_DEV_SEQ;

        i2c_target_idx = (idx + 1) % SIZE_I2C_DEV_SEQ;
        i2c_target_config[i2c_target_idx]->callbacks = \
            &i2c_target_test_info[i2c_target_idx].callbacks;
        i2c_target_config[i2c_target_idx]->address = \
            i2c_target_test_info[i2c_target_idx].address;

        i2c_src_idx = idx;
        i2c_src_addr = i2c_target_config[i2c_target_idx]->address;

        ret = i2c_configure(i2c_dev_sequence[i2c_target_idx],
                            I2C_SPEED_SET(I2C_SPEED_FAST)
                            | I2C_MODE_CONTROLLER);
        if (ret) {
            printk("\033[1;31m");
            printk("%s: failed (%d) to configure as target i2c device\n",
                   i2c_dev_sequence[i2c_target_idx]->name, ret);
            printk("\033[0m\n");
            goto free;
        }

        ret = i2c_configure(i2c_dev_sequence[i2c_src_idx],
                            I2C_SPEED_SET(I2C_SPEED_FAST)
                            | I2C_MODE_CONTROLLER);

        if (ret) {
            printk("\033[1;31m");
            printk("%s: failed (%d) to configure as master i2c device\n",
                   i2c_dev_sequence[i2c_src_idx]->name, ret);
            printk("\033[0m\n");
            goto free;
        }
#if DBG_I2C_LOG_DISPLAY_ENABLE
        printk("\033[1;33m");
        printk("=====================================================\n");
        printk("\n\n[%2d] TESTS: source (%s) -> target (%s)\n\n",
               test_seq,
               i2c_dev_sequence[i2c_src_idx]->name,
               i2c_dev_sequence[i2c_target_idx]->name);
        printk("=====================================================\n");
        printk("\033[0m\n");
#endif

        i2c_registers_disaplay(i2c_dev_sequence[i2c_target_idx],
                               "[ before ] register target device");

        ret = i2c_target_register(i2c_dev_sequence[i2c_target_idx],
                                  i2c_target_config[i2c_target_idx]);

        i2c_registers_disaplay(i2c_dev_sequence[i2c_target_idx],
                               "[ after ] register target device");

        if (ret) {
            printk("%s: failed (%d) on register as target i2c device",
                   i2c_dev_sequence[idx]->name, ret);
            continue;
        }

        i2c_registers_disaplay(i2c_dev_sequence[i2c_src_idx],
                               "[ before ] master write");
        i2c_registers_disaplay(i2c_dev_sequence[i2c_target_idx],
                               "[ before ] master write of target device");

        ret = i2c_write(i2c_dev_sequence[i2c_src_idx],
                        buf, TOTAL_I2C_DATA_TO_SEND, i2c_src_addr);

        i2c_registers_disaplay(i2c_dev_sequence[i2c_src_idx],
                               "[ after ] master write");
        i2c_registers_disaplay(i2c_dev_sequence[i2c_target_idx],
                               "[ after ] master write of target device");

        if (ret) {
            printk("\033[1;31m");
            printk("%s: failed (%d) to send i2c data as address 0x%04X\n",
                   i2c_dev_sequence[i2c_src_idx]->name, ret, i2c_src_addr);
            printk("\033[0m\n");
            continue;
        }

        ret = i2c_target_unregister(i2c_dev_sequence[i2c_target_idx],
                                    i2c_target_config[i2c_target_idx]);

        i2c_registers_disaplay(
            i2c_dev_sequence[i2c_target_idx],
            "[ after ] UNregister target device");
        i2c_registers_disaplay(
            i2c_dev_sequence[i2c_src_idx],
            "[ after ] Unregister target device (SOURCE DEVICE)");

        if (ret) {
            printk("\033[1;31m");
            printk("%s: failed (%d) on UNregister target i2c device",
                   i2c_dev_sequence[idx]->name, ret);
            printk("\033[0m\n");
            continue;
        }

        printk("\033[1;32m");
        printk("TEST PASS [%d/%d]\n",
               test_seq + 1, SIZE_I2C_DEV_SEQ);
        printk("\033[0m\n");
    }

free:
    for (int i = 0; i < SIZE_I2C_DEV_SEQ; i++) {
        k_free(i2c_target_config[i]);
    }

    k_free(buf);

    return;
}

void main(void)
{
    printk("Hello World! %s\n", CONFIG_BOARD);
    i2c_target_and_source_test();
}
