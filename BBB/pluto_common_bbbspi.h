/*   This is a component of pluto_servo/step_bbbspi for BeagleBoneBlack to FPGA over SPI for linuxcnc.
 *    Copyright 2013 Matsche <tinker@play-pla.net>
 * 			based on GP Orcullo's picnc driver and
 * 			based on the pluto_common.h from Jeff Epler <jepler@unpythonic.net>
 *
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef HAL_BBBSPI_H
#define HAL_BBBSPI_H

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#define CONTROL_MODULE_START_ADDR        0x44E10000
#define CONTROL_MODULE_END_ADDR        0x44E11FFF
#define CONTROL_MODULE_SIZE        (CONTROL_MODULE_END_ADDR - CONTROL_MODULE_START_ADDR)

#define PIN_RX_DISABLED 0
#define PIN_RX_ENABLED (1<<5)

#define PIN_PULLUD_DISABLED 0
#define PIN_PULLUD_ENABLED (1<<3)

#define PIN_PULLUP (1<<4)
#define PIN_PULLDOWN 0

#define PIN_SLEW_FAST 0
#define PIN_SLEW_SLOW (1<<6)


#define PIN_MODE0 0
#define PIN_MODE1 1
#define PIN_MODE2 2
#define PIN_MODE3 3
#define PIN_MODE4 4
#define PIN_MODE5 5
#define PIN_MODE6 6
#define PIN_MODE7 7

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO_SIZE  0x00002000

// OE: 0 is output, 1 is input
#define GPIO_OE 0x134
#define GPIO_DATAIN 0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_CLRDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194


#define GPIO1_16_DATA0 (1<<16)
#define GPIO1_18_CLK (1<<18)
#define GPIO1_19_RST (1<<19)

#define GPIO1_16_OSET 0x840
#define GPIO1_18_OSET 0x848
#define GPIO1_19_OSET 0x84c

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

//static const char *device = "/dev/spidev1.1";
static const char *device = "/dev/spidev1.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 5000000;
static uint16_t delay;

				/* SPICLK = 250 MHz/(1 << SPICLKDIV) */
#define SPIBUFSIZE		20		/* SPI buffer size in bytes*/
#define BUFSIZE			(SPIBUFSIZE/4)

int mem_fd1, mem_fd2;
volatile void *gpio_map;
volatile void *ctrl_map;
// I/O access
volatile unsigned *gpio1base;
volatile unsigned *ctrl_mod;

static int gpio_setup(void)
{
	// get the slot#
	//system("cat /sys/devices/bone_capemgr.*/slots |grep BB-SPI0-AND-FPGA|cut -d: -f1 |cut -d' ' -f2")
	// enable gpio overlay/fragment; muss noch verbessert werden....
	system("echo BB-SPI0-AND-FPGA > /sys/devices/bone_capemgr.9/slots");
	
	//system("echo pluto_config > /sys/devices/ocp.*/spi0_and_gpio.11/state");
	
	system("echo 2 > /sys/class/gpio/export");
	system("echo 3 > /sys/class/gpio/export");
	system("echo 4 > /sys/class/gpio/export");
	system("echo 5 > /sys/class/gpio/export");

	/* open /dev/mem */
	if ((mem_fd1 = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: can't open /dev/mem \n");
		return (-1);
	}

	/* mmap GPIO */
	//gpio_map = (char *)mmap(
	gpio_map = mmap(NULL, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd1, GPIO1_BASE);

	if (gpio_map == MAP_FAILED) {
		rtapi_print_msg(RTAPI_MSG_ERR,"mmap ERROR %d\n", (int)gpio_map);
		return (-1);
	}
	// Always use the volatile pointer!
	gpio1base = (volatile unsigned *)gpio_map;

	// init control module
	mem_fd2 = open("/dev/mem", O_RDWR);
	ctrl_map = mmap(NULL, CONTROL_MODULE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd2, CONTROL_MODULE_START_ADDR);
	if(ctrl_map == MAP_FAILED) {
		rtapi_print_msg(RTAPI_MSG_ERR,"%s: ERROR: Unable to map Control Module: %d", ctrl_map);
		exit(-1);
	}
	//close(mem_fd2);
	// Always use the volatile pointer!
	ctrl_mod = (volatile unsigned *)ctrl_map;
	
	*(ctrl_mod+GPIO1_16_OSET/4) = (unsigned)(PIN_MODE7 | PIN_PULLUD_DISABLED | PIN_RX_DISABLED);
	*(ctrl_mod+GPIO1_18_OSET/4) = (unsigned)(PIN_MODE7 | PIN_PULLUD_DISABLED | PIN_RX_DISABLED);
	*(ctrl_mod+GPIO1_19_OSET/4) = (unsigned)(PIN_MODE7 | PIN_PULLUD_DISABLED | PIN_RX_DISABLED);
	
	// Set outputs
	*(gpio1base + GPIO_OE/4) &= (~GPIO1_18_CLK);
	*(gpio1base + GPIO_OE/4) &= (~GPIO1_16_DATA0);
	*(gpio1base + GPIO_OE/4) &= (~GPIO1_19_RST);
}


#endif //HAL_BBBSPI_H
