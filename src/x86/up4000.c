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
        if (caps.i2c) {
            pin_info->i2c.pinmap = 1;
            pin_info->i2c.mux_total = 0;
        }
        if (caps.spi) {
            pin_info->spi.mux_total = 0;
        }
        return MRAA_SUCCESS;
    }
    return MRAA_ERROR_INVALID_RESOURCE;
}

static mraa_result_t
mraa_up4000_get_pin_index(mraa_board_t* board, char* name, int* pin_index)
{
    int i;
    for (i = 0; i < board->phy_pin_count; ++i) {
        if (strncmp(name, board->pins[i].name, MRAA_PIN_NAME_SIZE) == 0) {
            *pin_index = i;
            return MRAA_SUCCESS;
        }
    }

    syslog(LOG_CRIT, "up4000: Failed to find pin name %s", name);

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

    // NOTE: UART TX (pin 8) is confirmed with a logic analyzer: while
    // continuously writing to /dev/ttyS5, a clean, error-free async serial
    // decode (115200 8N1) was captured on this pin. /dev/ttyS5 is the
    // native SoC UART reached via PCI 0000:00:18.1 (dw-apb-uart.4) - NOT
    // /dev/ttyS4 / 0000:00:18.0 (dw-apb-uart.3), which is the other LPSS
    // HSUART PCI function and was initially assumed to be the header UART
    // but produced no activity on any header pin under the same test.
    // RX/RTS/CTS (pins 10/11/36) are wired up on the matching FPGA line
    // numbers per the UP2's documented UART pin mapping, but have NOT been
    // individually confirmed with a loopback or real peripheral - only TX
    // has been directly observed. SPI is wired up on the same
    // native-PCI-function reasoning used for UART/I2C (no bitbang consumer
    // on the FPGA lines, a dedicated PCI SPI master, same pattern as
    // UP2), but as of this writing has NOT been empirically confirmed with
    // a real device or MOSI/MISO loopback, because /dev/spidev1.x was
    // root-only (crw------- root root, no group) with no sudo access
    // available to test with. I2C was confirmed by observing real
    // header-attached devices respond over i2cdetect on the native SoC I2C
    // controllers below. Verify SPI and UART RX/RTS/CTS with a loopback or
    // real peripheral before trusting them in the field.
    mraa_up4000_set_pininfo(b, 0, "INVALID",    (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 1, "3.3v",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 2, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 3, "I2C_SDA",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, fpga_chip, 2);
    mraa_up4000_set_pininfo(b, 4, "5v",         (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 5, "I2C_SCL",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, fpga_chip, 3);
    mraa_up4000_set_pininfo(b, 6, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 7, "GPIO4",      (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 4);
    mraa_up4000_set_pininfo(b, 8, "UART_TX",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, fpga_chip, 14);
    mraa_up4000_set_pininfo(b, 9, "GND",        (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 10, "UART_RX",   (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, fpga_chip, 15);
    mraa_up4000_set_pininfo(b, 11, "UART_RTS",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, fpga_chip, 17);
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
    mraa_up4000_set_pininfo(b, 19, "SPI0_MOSI", (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, fpga_chip, 10);
    mraa_up4000_set_pininfo(b, 20, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 21, "SPI0_MISO", (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, fpga_chip, 9);
    // Reaction wheel RPM
    mraa_up4000_set_pininfo(b, 22, "GPIO25",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 25);
    mraa_up4000_set_pininfo(b, 23, "SPI0_CLK",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, fpga_chip, 11);
    mraa_up4000_set_pininfo(b, 24, "SPI0_CS0",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, fpga_chip, 8);
    mraa_up4000_set_pininfo(b, 25, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    mraa_up4000_set_pininfo(b, 26, "SPI0_CS1",  (mraa_pincapabilities_t) {1, 1, 0, 0, 1, 0, 0, 0}, fpga_chip, 7);
    mraa_up4000_set_pininfo(b, 27, "ID_SD",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, fpga_chip, 0);
    mraa_up4000_set_pininfo(b, 28, "ID_SC",     (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 1, 0, 0}, fpga_chip, 1);
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
    mraa_up4000_set_pininfo(b, 36, "UART_CTS",  (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 1}, fpga_chip, 16);
    mraa_up4000_set_pininfo(b, 37, "GPIO26",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 26);
    // Voltage control
    mraa_up4000_set_pininfo(b, 38, "GPIO20",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 20);
    mraa_up4000_set_pininfo(b, 39, "GND",       (mraa_pincapabilities_t) {0, 0, 0, 0, 0, 0, 0, 0}, fpga_chip, -1);
    // Watchdog
    mraa_up4000_set_pininfo(b, 40, "GPIO21",    (mraa_pincapabilities_t) {1, 1, 0, 0, 0, 0, 0, 0}, fpga_chip, 21);

    b->i2c_bus_count = 0;
    b->def_i2c_bus = 0;
    int i2c_bus_num;

    // Configure I2C adaptor #0 (default)
    // Confirmed via i2cdetect: real header-attached devices (GPIO
    // expanders, sensors) respond on this native adaptor.
    i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:16.1", "i2c_designware.1");
    if (i2c_bus_num != -1) {
        int i = b->i2c_bus_count;
        b->i2c_bus[i].bus_id = i2c_bus_num;
        mraa_up4000_get_pin_index(b, "I2C_SDA", &(b->i2c_bus[i].sda));
        mraa_up4000_get_pin_index(b, "I2C_SCL", &(b->i2c_bus[i].scl));
        b->i2c_bus_count++;
    }

    // Configure I2C adaptor #1
    // (normally reserved for accessing HAT EEPROM)
    i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:16.0", "i2c_designware.0");
    if (i2c_bus_num != -1) {
        int i = b->i2c_bus_count;
        b->i2c_bus[i].bus_id = i2c_bus_num;
        mraa_up4000_get_pin_index(b, "ID_SD", &(b->i2c_bus[i].sda));
        mraa_up4000_get_pin_index(b, "ID_SC", &(b->i2c_bus[i].scl));
        b->i2c_bus_count++;
    }

    b->pwm_dev_count = 0;
    b->def_pwm_dev = 0;

    b->spi_bus_count = 0;
    b->def_spi_bus = 0;

    // Configure SPI #0 CS0 (default)
    // Header SPI pins trace to a dedicated native PCI SPI master
    // (0000:00:19.0, pxa2xx-spi.5 -> /sys/class/spi_master/spi1), the same
    // pattern used for I2C above and matching the UP2's wiring - there's no
    // spi-gpio bitbang consumer on the FPGA's SPI lines. mraa has no
    // generic PCI-based SPI bus resolver (unlike mraa_find_i2c_bus_pci; the
    // sysfs layout differs), so bus_id is hardcoded to 1, same as up2.c.
    b->spi_bus[0].bus_id = 1;
    b->spi_bus[0].slave_s = 0;
    mraa_up4000_get_pin_index(b, "SPI0_CS0",  &(b->spi_bus[0].cs));
    mraa_up4000_get_pin_index(b, "SPI0_MOSI", &(b->spi_bus[0].mosi));
    mraa_up4000_get_pin_index(b, "SPI0_MISO", &(b->spi_bus[0].miso));
    mraa_up4000_get_pin_index(b, "SPI0_CLK",  &(b->spi_bus[0].sclk));
    b->spi_bus_count++;

    // Configure SPI #0 CS1
    b->spi_bus[1].bus_id = 1;
    b->spi_bus[1].slave_s = 1;
    mraa_up4000_get_pin_index(b, "SPI0_CS1",  &(b->spi_bus[1].cs));
    mraa_up4000_get_pin_index(b, "SPI0_MOSI", &(b->spi_bus[1].mosi));
    mraa_up4000_get_pin_index(b, "SPI0_MISO", &(b->spi_bus[1].miso));
    mraa_up4000_get_pin_index(b, "SPI0_CLK",  &(b->spi_bus[1].sclk));
    b->spi_bus_count++;

    b->uart_dev_count = 0;
    b->def_uart_dev = 0;

    // Configure UART #0 (default)
    // Confirmed via logic analyzer: TX (pin 8) shows a clean async serial
    // decode while continuously writing to /dev/ttyS5. This is
    // 0000:00:18.1 / dw-apb-uart.4, not 0000:00:18.0 / dw-apb-uart.3
    // (/dev/ttyS4), which is the other native PCI HSUART and was ruled out
    // by the same test (no activity on any header pin).
    if (mraa_find_uart_bus_pci(
            "/sys/bus/pci/devices/0000:00:18.1/dw-apb-uart.4/dw-apb-uart.4:0/dw-apb-uart.4:0.0/tty/",
            &(b->uart_dev[0].device_path)) == MRAA_SUCCESS) {
        mraa_up4000_get_pin_index(b, "UART_RX", &(b->uart_dev[0].rx));
        mraa_up4000_get_pin_index(b, "UART_TX", &(b->uart_dev[0].tx));
        mraa_up4000_get_pin_index(b, "UART_CTS", &(b->uart_dev[0].cts));
        mraa_up4000_get_pin_index(b, "UART_RTS", &(b->uart_dev[0].rts));
        b->uart_dev_count++;
    }

    b->aio_count = 0;

    return b;

error:
    syslog(LOG_CRIT, "up4000: Platform failed to initialise");
    free(b);
    return NULL;
}
