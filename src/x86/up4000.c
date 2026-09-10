/*
 * Author: Javier Arteaga <javier@emutex.com>
 * Based on work from: Dan O'Donovan <dan@emutex.com>
 *                     Nicola Lunghi <nicola.lunghi@emutex.com>
 * Copyright (c) 2017 Emutex Ltd.
 * Copyright (c) 2014 Intel Corporation.
 *
 * SPDX-License-Identifier: MIT
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

#include "common.h"
#include "gpio.h"
#include "gpio/gpio_chardev.h"
#include "x86/up4000.h"

#define PLATFORM_NAME "UP4000"
#define PLATFORM_VERSION "1.0.0"

#define MRAA_UP4000_GPIOCOUNT 28

// All 40-pin header GPIOs on the UP 4000 are routed through the on-board
// FPGA, which identifies itself via this chardev chip label (as seen in
// gpiodetect), using line numbers that match the standard Raspberry Pi BCM
// GPIO numbering. Unlike the UP2/UP Xtreme, none of the header signals are
// wired directly to the SoC's native pinctrl gpiochips (INT3452:00-03).
#define MRAA_UP4000_FPGA_LABEL "Raspberry Pi compatible UP GPIO"

// The FPGA's /dev/gpiochipN index is assigned in kernel probe order, which
// is not stable across reboots (other gpiochips, e.g. USB GPIO expanders,
// can enumerate before or after it). Find it by chip label instead of
// trusting a fixed number.
static int
mraa_up4000_find_fpga_chip()
{
    mraa_gpiod_chip_info** cinfos;
    int num_chips = mraa_get_chip_infos(&cinfos);
    int found = -1;
    int i;

    if (num_chips < 0) {
        return -1;
    }

    for (i = 0; i < num_chips; i++) {
        if (cinfos[i] == NULL) {
            continue;
        }
        if (found == -1 &&
            strncmp(cinfos[i]->chip_info.label, MRAA_UP4000_FPGA_LABEL,
                    sizeof(cinfos[i]->chip_info.label)) == 0) {
            int chip_number;
            if (sscanf(cinfos[i]->chip_info.name, "gpiochip%d", &chip_number) == 1) {
                found = chip_number;
            }
        }
        close(cinfos[i]->chip_fd);
        free(cinfos[i]);
    }
    free(cinfos);

    return found;
}

// utility function to setup pin mapping of boards
static mraa_result_t
mraa_up4000_set_pininfo(mraa_board_t* board, int mraa_index, char* name,
        mraa_pincapabilities_t caps, int chip, int line)
{
    if (mraa_index < board->phy_pin_count) {
        mraa_pininfo_t* pin_info = &board->pins[mraa_index];
        strncpy(pin_info->name, name, MRAA_PIN_NAME_SIZE);
        pin_info->capabilities = caps;
        if (caps.gpio) {
            pin_info->gpio.pinmap = line;
            pin_info->gpio.mux_total = 0;
            pin_info->gpio.gpio_chip = chip;
            pin_info->gpio.gpio_line = line;
        }
        return MRAA_SUCCESS;
    }
    return MRAA_ERROR_INVALID_RESOURCE;
}

mraa_board_t*
mraa_up4000_board()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof (mraa_board_t));

    if (b == NULL) {
        return NULL;
    }

    b->platform_name = PLATFORM_NAME;
    b->platform_version = PLATFORM_VERSION;
    b->phy_pin_count = MRAA_UP4000_PINCOUNT;
    b->gpio_count = MRAA_UP4000_GPIOCOUNT;
    b->chardev_capable = 1;

    b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * MRAA_UP4000_PINCOUNT);
    if (b->pins == NULL) {
        goto error;
    }

    b->adv_func = (mraa_adv_func_t *) calloc(1, sizeof (mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b->pins);
        goto error;
    }

    int fpga_chip = mraa_up4000_find_fpga_chip();
    if (fpga_chip == -1) {
        syslog(LOG_CRIT, "up4000: could not find gpiochip labeled '%s'", MRAA_UP4000_FPGA_LABEL);
        free(b->adv_func);
        free(b->pins);
        goto error;
    }
    syslog(LOG_NOTICE, "up4000: FPGA header gpiochip resolved to gpiochip%d", fpga_chip);

    // NOTE: I2C/SPI/UART bus wiring (i2c_bus/spi_bus/uart_dev, pwm_dev) is
    // deliberately left unconfigured below. The header signals for those
    // buses are only exposed here as plain chardev GPIO lines on the FPGA
    // until the actual bus routing on this board has been verified.
    mraa_up4000_set_pininfo(b, 0, "INVALID",    (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 1, "3.3v",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 2, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 3, "I2C_SDA",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 2);
    mraa_up4000_set_pininfo(b, 4, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 5, "I2C_SCL",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 3);
    mraa_up4000_set_pininfo(b, 6, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 7, "GPIO4",      (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 4);
    mraa_up4000_set_pininfo(b, 8, "UART_TX",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 14);
    mraa_up4000_set_pininfo(b, 9, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 10, "UART_RX",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 15);
    mraa_up4000_set_pininfo(b, 11, "GPIO17",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 17);
    mraa_up4000_set_pininfo(b, 12, "GPIO18",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 18);
    mraa_up4000_set_pininfo(b, 13, "GPIO27",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 27);
    mraa_up4000_set_pininfo(b, 14, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // Solar disconnect / Thruster enable
    mraa_up4000_set_pininfo(b, 15, "GPIO22",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 22);
    // Reaction wheel RPM
    mraa_up4000_set_pininfo(b, 16, "GPIO23",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 23);
    mraa_up4000_set_pininfo(b, 17, "3.3v",      (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // Reaction wheel RPM
    mraa_up4000_set_pininfo(b, 18, "GPIO24",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 24);
    mraa_up4000_set_pininfo(b, 19, "SPI0_MOSI", (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 10);
    mraa_up4000_set_pininfo(b, 20, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 21, "SPI0_MISO", (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 9);
    // Reaction wheel RPM
    mraa_up4000_set_pininfo(b, 22, "GPIO25",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 25);
    mraa_up4000_set_pininfo(b, 23, "SPI0_CLK",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 11);
    mraa_up4000_set_pininfo(b, 24, "SPI0_CS0",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 8);
    mraa_up4000_set_pininfo(b, 25, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 26, "SPI0_CS1",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 7);
    mraa_up4000_set_pininfo(b, 27, "ID_SD",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 0);
    mraa_up4000_set_pininfo(b, 28, "ID_SC",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 1);
    // GPIO expander reset (PropBoard 1)
    mraa_up4000_set_pininfo(b, 29, "GPIO5",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 5);
    mraa_up4000_set_pininfo(b, 30, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // GPIO expander reset (PropBoard 2)
    mraa_up4000_set_pininfo(b, 31, "GPIO6",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 6);
    // Solar connect / Iso valve
    mraa_up4000_set_pininfo(b, 32, "GPIO12",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 12);
    // GPIO expander reset (IOBoard 1)
    mraa_up4000_set_pininfo(b, 33, "GPIO13",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 13);
    mraa_up4000_set_pininfo(b, 34, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // GPIO expander reset (IOBoard 2)
    mraa_up4000_set_pininfo(b, 35, "GPIO19",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 19);
    mraa_up4000_set_pininfo(b, 36, "GPIO16",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 16);
    mraa_up4000_set_pininfo(b, 37, "GPIO26",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 26);
    // Voltage control
    mraa_up4000_set_pininfo(b, 38, "GPIO20",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 20);
    mraa_up4000_set_pininfo(b, 39, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // Watchdog
    mraa_up4000_set_pininfo(b, 40, "GPIO21",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 21);

    b->i2c_bus_count = 0;
    b->def_i2c_bus = 0;

    b->pwm_dev_count = 0;
    b->def_pwm_dev = 0;

    b->spi_bus_count = 0;
    b->def_spi_bus = 0;

    b->uart_dev_count = 0;
    b->def_uart_dev = 0;

    b->aio_count = 0;

    return b;

error:
    syslog(LOG_CRIT, "up4000: Platform failed to initialise");
    free(b);
    return NULL;
}
