/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      mllite_test.c
 *       @brief     Test app for mllite.
 *       @details   The contents of this main are a good example of how a HAL
 *                  would be implemented.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "USB_eMPL/descriptors.h"

#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/usb.h"

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#include "F5xx_F6xx_Core_Lib/HAL_FLASH.h"

#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "usbConstructs.h"

#include "msp430.h"
#include "msp430_clock.h"
#include "msp430_i2c.h"
#include "msp430_interrupt.h"

#include "inv_gyro.h"
#include "inv_gyro_dmp_android.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "inv_driver_if.h"
#include "packet.h"
#include "msp430_uart.h"

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

/* 50 Hz has been tested to give good performance
 * on the testing code with mCLK set to 12Mhz.
 * Recommended to increase the system clock to
 * higher Hz from 12Mhz to increase the 50Hz update
 * any higher value.
 */
#define DEFAULT_MPU_HZ  (50)
#define DEFAULT_MAG_MS  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

/* Default used to indicate data ready*/
#define TOGGLE_RED_LED  (P1OUT ^=0x04)
#define SET_RED_LED     (P1OUT |=0x04)
#define CLEAR_RED_LED   (P1OUT &=~ 0x04)

/* Default used to indicate motion detect*/
#define TOGGLE_BLUE_LED  (P1OUT ^=0x08)
#define SET_BLUE_LED     (P1OUT |=0x08)
#define CLEAR_BLUE_LED   (P1OUT &=~ 0x08)

/* Default used to indicate Failure*/
#define TOGGLE_BOTH_LED (P1OUT ^=  (0x04 + 0x08))
#define SET_BOTH_LED    (P1OUT |=  (0x04 + 0x08))
#define CLEAR_BOTH_LED  (P1OUT &=~ (0x04 + 0x08))


struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};

struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	unsigned char new_gyro;
	unsigned char new_compass;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned short read_pedo_ms;
	unsigned long next_compass_ms;
	/* TODO: For this example, this value is never changed. Save a couple bytes. */
	unsigned short read_compass_ms;
	unsigned short report;
	unsigned char dmp_features;
	struct rx_s rx;
	struct int_param_s int_param;
};
static struct hal_s hal = { 0 };

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* Should be defined by your compass driver. */
extern struct driver_if_s compass_if;

//unsigned char *mpl_key = (unsigned char*)"Wrong_Key";
unsigned char *mpl_key = (unsigned char*) "eMPL 5.1";

// Global Variables used in the main and functions
unsigned char accel_fsr;
unsigned short gyro_rate, gyro_fsr, compass_fsr;
unsigned long timestamp;
unsigned long sensor_timestamp;
int new_data;
unsigned char check_uart_mode_enabled = 0;

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void) {
	long msg, data[9];
	int8_t accuracy;
	unsigned long timestamp;

	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*) &timestamp)) {
		/* Sends a quaternion packet to the PC. Since this is used by the Python
		 * test app to visually represent a 3D quaternion, it's sent each time
		 * the MPL has new data.
		 */
		eMPL_send_quat(data);

		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT)
			eMPL_send_data(PACKET_DATA_QUAT, data);
	}

	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_ACCEL, data);
	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_GYRO, data);
	}
	if (hal.report & PRINT_COMPASS) {
		if (inv_get_sensor_type_compass(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_COMPASS, data);
	}
	if (hal.report & PRINT_EULER) {
		if (inv_get_sensor_type_euler(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_EULER, data);
	}
	if (hal.report & PRINT_ROT_MAT) {
		if (inv_get_sensor_type_rot_mat(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_ROT, data);
	}
	if (hal.report & PRINT_HEADING) {
		if (inv_get_sensor_type_heading(data, &accuracy,
				(inv_time_t*) &timestamp))
			eMPL_send_data(PACKET_DATA_HEADING, data);
	}
	if (hal.report & PRINT_PEDO) {
		unsigned long timestamp;
		msp430_get_clock_ms(&timestamp);
		if (timestamp > hal.next_pedo_ms) {
			hal.next_pedo_ms = timestamp + hal.read_pedo_ms;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n",
					step_count, walk_time);
		}
	}

	/* Whenever the MPL detects a change in motion state, the application can
	 * be notified. For this example, we use an LED to represent the current
	 * motion state.
	 */
	msg = inv_get_message_level_0(
			INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
	if (msg) {
		if (msg & INV_MSG_MOTION_EVENT) {
			MPL_LOGI("Motion!\n");
			SET_BLUE_LED;
		} else if (msg & INV_MSG_NO_MOTION_EVENT) {
			MPL_LOGI("No motion!\n");
			CLEAR_BLUE_LED;
		} else if (msg & INV_MSG_NEW_GB_EVENT) {
			long gyro_bias[3], temp;
			inv_get_gyro_bias(gyro_bias, &temp);
			dmp_set_gyro_bias(gyro_bias);
		}
	}
}

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
	signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 * The present matrix is updated to match the MotionFit SDK board
 * according to the board definition of XYZ. (Z is coming out)
 *
 *    |------------------------------------|
 *    SW1              		[ BT ]		   |  (-Y Board)
 *    |                                    |    .
 *    |           ------                   |   /.\
 *    |          |[INV] |                  |    .
 *    | +x chip<-|     .|.pin1             |    .
 *    |           ------                   |    .
 *    |               					   |  -------> (-X Board)
 *    SW2					[ TI ]		   |    .
 *    SW4								   |
 *    |                                    |
 *    |O-------------[USB]----------------O|
 *
 *
 */
static struct platform_data_s gyro_pdata = { .orientation = { 1, 0, 0, 0, 1,
		0, 0, 0, 1 } };

static struct platform_data_s compass_pdata = { .orientation = { 0, 1, 0, 1, 0,
		0, 0, 0, -1 } };

static void setup_gyro(void) {
	unsigned char mask = 0, lp_accel_was_on = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON) {
		mask |= INV_XYZ_GYRO;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	if (hal.sensors & COMPASS_ON) {
		mask |= INV_XYZ_COMPASS;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	/* If you need a power transition, this function should be called with a
	 * mask of the sensors still enabled. The driver turns off any sensors
	 * excluded from this mask.
	 */
	gyro_set_sensors(mask);
	gyro_configure_fifo(mask);
	if (lp_accel_was_on) {
		unsigned short rate;
		hal.lp_accel_mode = 0;
		/* Switching out of LP accel, notify MPL of new accel sampling rate. */
		gyro_get_sample_rate(&rate);
		inv_set_accel_sample_rate(1000000L / rate);
	}
}

static void tap_cb(unsigned char direction, unsigned char count) {
	switch (direction) {
	case TAP_X_UP:
		MPL_LOGI("Tap X+ ");
		break;
	case TAP_X_DOWN:
		MPL_LOGI("Tap X- ");
		break;
	case TAP_Y_UP:
		MPL_LOGI("Tap Y+ ");
		break;
	case TAP_Y_DOWN:
		MPL_LOGI("Tap Y- ");
		break;
	case TAP_Z_UP:
		MPL_LOGI("Tap Z+ ");
		break;
	case TAP_Z_DOWN:
		MPL_LOGI("Tap Z- ");
		break;
	default:
		return;
	}
	MPL_LOGI("x%d\n", count);
	return;
}

static void display_orient_cb(unsigned char orientation) {
	/* These #define's assume that the chip is mounted on a handset, such that
	 * the Y-axis travels from the bottom to the top of the phone. Here is a
	 * lame attempt at ASCII art.
	 *  ________
	 * |        |
	 * |  /\    |
	 * |  ||    |
	 * |  ||y+  |
	 * |  ||    |
	 * |  ||    |
	 * |________|
	 */
	switch (orientation) {
	case DISPLAY_ORIENT_PORTRAIT:
		MPL_LOGI("Portrait.\n");
		break;
	case DISPLAY_ORIENT_LANDSCAPE:
		MPL_LOGI("Landscape.\n");
		break;
	case DISPLAY_ORIENT_REVERSE_PORTRAIT:
		MPL_LOGI("Reverse Portrait.\n");
		break;
	case DISPLAY_ORIENT_REVERSE_LANDSCAPE:
		MPL_LOGI("Reverse Landscape.\n");
		break;
	default:
		break;
	}
}

static void orient_cb(unsigned char orientation) {
	if (orientation & ORIENTATION_X_UP)
		MPL_LOGI("Orientation X+");
	else if (orientation & ORIENTATION_X_DOWN)
		MPL_LOGI("Orientation X-");
	else if (orientation & ORIENTATION_Y_UP)
		MPL_LOGI("Orientation Y+");
	else if (orientation & ORIENTATION_Y_DOWN)
		MPL_LOGI("Orientation Y-");
	else if (orientation & ORIENTATION_Z_UP)
		MPL_LOGI("Orientation Z+");
	else if (orientation & ORIENTATION_Z_DOWN)
		MPL_LOGI("Orientation Z-");

	if (orientation & ORIENTATION_FLIP)
		MPL_LOGI(" (FLIP!)\n");
	else
		MPL_LOGI("\n");
}
static void msp430_reset(void) {
	PMMCTL0 |= PMMSWPOR;
}

static void handle_input(void) {
	char c;
	int result;
	unsigned short accel_sens;
	float gyro_sens;
	size_t store_size;
	const unsigned char header[3] = "inv";
	long gyro_bias[3], accel_bias[3];

	/* Read incoming byte and check for header.
	 * Technically, the MSP430 USB stack can handle more than one byte at a
	 * time. This example allows for easily switching to UART if porting to a
	 * different microcontroller.
	 */
	rx_new = 0;
	/* modify the code here and choose the logic for UART or USB
	 * if USB call the cdcReceiveDataInBuffer((BYTE*) &c, 1, CDC0_INTFNUM);
	 * else call the msp430_get_uart_rx_data((BYTE*) &c);
	 * Looking for a command inva or invc etc
	 */

	if (check_uart_mode_enabled) {
		msp430_get_uart_rx_data((BYTE*) &c);
	} else {
		cdcReceiveDataInBuffer((BYTE*) &c, 1, CDC0_INTFNUM);
	}
	//cdcReceiveDataInBuffer((BYTE*)&c, 1, CDC0_INTFNUM);
	if (hal.rx.header[0] == header[0]) {
		if (hal.rx.header[1] == header[1]) {
			if (hal.rx.header[2] == header[2]) {
				memset(&hal.rx.header, 0, sizeof(hal.rx.header));
				hal.rx.cmd = c;
			} else if (c == header[2])
				hal.rx.header[2] = c;
			else
				memset(&hal.rx.header, 0, sizeof(hal.rx.header));
		} else if (c == header[1])
			hal.rx.header[1] = c;
		else
			memset(&hal.rx.header, 0, sizeof(hal.rx.header));
	} else if (c == header[0])
		hal.rx.header[0] = header[0];
	if (!hal.rx.cmd)
		return;

	switch (hal.rx.cmd) {
	/* These commands turn off individual sensors. */
	case '8':
		hal.sensors ^= ACCEL_ON;
		setup_gyro();
		if (!(hal.sensors & ACCEL_ON))
			inv_accel_was_turned_off();
		break;
	case '9':
		hal.sensors ^= GYRO_ON;
		setup_gyro();
		if (!(hal.sensors & GYRO_ON))
			inv_gyro_was_turned_off();
		break;
	case '0':
		hal.sensors ^= COMPASS_ON;
		setup_gyro();
		if (!(hal.sensors & COMPASS_ON))
			inv_compass_was_turned_off();
		break;
		/* The commands send individual sensor data or fused data to the PC. */
	case 'a':
		hal.report ^= PRINT_ACCEL;
		break;
	case 'g':
		hal.report ^= PRINT_GYRO;
		break;
	case 'c':
		hal.report ^= PRINT_COMPASS;
		break;
	case 'e':
		hal.report ^= PRINT_EULER;
		break;
	case 'r':
		hal.report ^= PRINT_ROT_MAT;
		break;
	case 'q':
		hal.report ^= PRINT_QUAT;
		break;
	case 'h':
		hal.report ^= PRINT_HEADING;
		break;
		/* This command prints out the value of each gyro register for debugging.
		 * If logging is disabled, this function has no effect.
		 */
	case 'd':
		gyro_reg_dump();
		break;
		/* Test out low-power accel mode. */
	case 'p':
		if (hal.dmp_on)
			/* LP accel is not compatible with the DMP. */
			break;
		gyro_lp_accel_mode(20);
		/* When LP accel mode is enabled, the driver automatically configures
		 * the hardware for latched interrupts. However, the MCU sometimes
		 * misses the rising/falling edge, and the hal.new_gyro flag is never
		 * set. To avoid getting locked in this state, we're overriding the
		 * driver's configuration and sticking to unlatched interrupt mode.
		 *
		 * TODO: The MCU supports level-triggered interrupts.
		 */
		gyro_set_int_latched(0);
		hal.sensors &= ~(GYRO_ON | COMPASS_ON);
		hal.sensors |= ACCEL_ON;
		hal.lp_accel_mode = 1;
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
		/* This snippet of code shows how to load and store calibration data from
		 * the MPL. For the MSP430, the flash segment must be unlocked before
		 * reading/writing and locked when no longer in use. When porting to a
		 * different microcontroller, flash memory might be accessible at anytime,
		 * or may not be available at all.
		 */
	case 'l':
		inv_get_mpl_state_size(&store_size);
		if (store_size > FLASH_SIZE) {
			MPL_LOGE("Calibration data exceeds available memory.\n");
			break;
		}
		FCTL3 = FWKEY;
		inv_load_mpl_states(FLASH_MEM_START, store_size);
		FCTL3 = FWKEY + LOCK;
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
	case 's':
		inv_get_mpl_state_size(&store_size);
		if (store_size > FLASH_SIZE) {
			MPL_LOGE("Calibration data exceeds available memory.\n");
			return;
		} else {
			unsigned char mpl_states[100], tries = 5, erase_result;
			inv_save_mpl_states(mpl_states, store_size);
			while (tries--) {
				/* Multiple attempts to erase current data. */
				Flash_SegmentErase((uint16_t*) FLASH_MEM_START);
				erase_result = Flash_EraseCheck((uint16_t*) FLASH_MEM_START,
						store_size >> 1);
				if (erase_result == FLASH_STATUS_OK)
					break;
			}
			if (erase_result == FLASH_STATUS_ERROR) {
				MPL_LOGE("Could not erase user page for calibration "
				"storage.\n");
				break;
			}
			FlashWrite_8(mpl_states, FLASH_MEM_START, store_size);
		}
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
		/* The hardware self test can be run without any interaction with the
		 * MPL since it's completely localized in the gyro driver. Logging is
		 * assumed to be enabled; otherwise, a couple LEDs could probably be used
		 * here to display the test results.
		 */
	case 't':
		result = gyro_run_self_test(gyro_bias, accel_bias);
		if (!result) {
			MPL_LOGI("Passed!\n");
			MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
					accel_bias[0]/65536.f, accel_bias[1]/65536.f, accel_bias[2]/65536.f);
			MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
					gyro_bias[0]/65536.f, gyro_bias[1]/65536.f, gyro_bias[2]/65536.f);
			/* MPL expects biases in hardware units << 16, but self test returns
			 * biases in g's << 16.
			 */
			gyro_get_accel_sens(&accel_sens);
			accel_bias[0] *= accel_sens;
			accel_bias[1] *= accel_sens;
			accel_bias[2] *= accel_sens;
			inv_set_accel_bias(accel_bias, 3);
			gyro_get_gyro_sens(&gyro_sens);
			gyro_bias[0] = (long) (gyro_bias[0] * gyro_sens);
			gyro_bias[1] = (long) (gyro_bias[1] * gyro_sens);
			gyro_bias[2] = (long) (gyro_bias[2] * gyro_sens);
			inv_set_gyro_bias(gyro_bias, 3);
		} else {
			if (result & 0x001)
				MPL_LOGE("Gyro X failed.\n");
			if (result & 0x002)
				MPL_LOGE("Gyro Y failed.\n");
			if (result & 0x004)
				MPL_LOGE("Gyro Z failed.\n");
			if (result & 0x008)
				MPL_LOGE("Accel X failed.\n");
			if (result & 0x010)
				MPL_LOGE("Accel Y failed.\n");
			if (result & 0x020)
				MPL_LOGE("Accel Z failed.\n");
			if (result & 0x040)
				MPL_LOGE("Compass X failed.\n");
			if (result & 0x080)
				MPL_LOGE("Compass Y failed.\n");
			if (result & 0x100)
				MPL_LOGE("Compass Z failed.\n");
		}
		/* Let MPL know that contiguity was broken. */
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		break;
		/* Depending on your application, sensor data may be needed at a faster or
		 * slower rate. These commands can speed up or slow down the rate at which
		 * the sensor data is pushed to the MPL.
		 *
		 * In this example, the compass rate is never changed.
		 */
	case '1':
		if (hal.dmp_on)
			dmp_set_fifo_rate(10);
		else
			gyro_set_sample_rate(10);
		inv_set_gyro_sample_rate(100000L);
		inv_set_accel_sample_rate(100000L);
		break;
	case '2':
		if (hal.dmp_on)
			dmp_set_fifo_rate(20);
		else
			gyro_set_sample_rate(20);
		inv_set_gyro_sample_rate(50000L);
		inv_set_accel_sample_rate(50000L);
		break;
	case '3':
		if (hal.dmp_on)
			dmp_set_fifo_rate(40);
		else
			gyro_set_sample_rate(40);
		inv_set_gyro_sample_rate(25000L);
		inv_set_accel_sample_rate(25000L);
		break;
	case '4':
		if (hal.dmp_on)
			dmp_set_fifo_rate(50);
		else
			gyro_set_sample_rate(50);
		inv_set_gyro_sample_rate(20000L);
		inv_set_accel_sample_rate(20000L);
		break;
	case '5':
		if (hal.dmp_on)
			dmp_set_fifo_rate(100);
		else
			gyro_set_sample_rate(100);
		inv_set_gyro_sample_rate(10000L);
		inv_set_accel_sample_rate(10000L);
		break;
	case ',':
		/* Set hardware to interrupt on gesture event only. */
		dmp_set_interrupt_mode(DMP_INT_GESTURE);
		break;
	case '.':
		/* Set hardware to interrupt periodically. */
		dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
		break;
	case '6':
		/* Toggle pedometer display. */
		hal.report ^= PRINT_PEDO;
		break;
	case '7':
		/* Reset pedometer. */
		dmp_set_pedometer_step_count(0);
		dmp_set_pedometer_walk_time(0);
		break;
	case 'f':
		if (hal.lp_accel_mode)
			/* LP accel is not compatible with the DMP. */
			return;
		/* Toggle DMP. */
		if (hal.dmp_on) {
			unsigned short dmp_rate;
			unsigned char mask = 0;
			hal.dmp_on = 0;
			gyro_set_dmp_state(0);
			/* Restore FIFO settings. */
			if (hal.sensors & ACCEL_ON)
				mask |= INV_XYZ_ACCEL;
			if (hal.sensors & GYRO_ON)
				mask |= INV_XYZ_GYRO;
			if (hal.sensors & COMPASS_ON)
				mask |= INV_XYZ_COMPASS;
			gyro_configure_fifo(mask);
			/* When the DMP is used, the hardware sampling rate is fixed at
			 * 200Hz, and the DMP is configured to downsample the FIFO output
			 * using the function dmp_set_fifo_rate. However, when the DMP is
			 * turned off, the sampling rate remains at 200Hz. This could be
			 * handled in inv_gyro.c, but it would need to know that
			 * inv_gyro_dmp_android.c exists. To avoid this, we'll just put the
			 * extra logic in the application layer.
			 * TODO: Figure out how to abstract this from the application.
			 */
			dmp_get_fifo_rate(&dmp_rate);
			gyro_set_sample_rate(dmp_rate);
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("DMP disabled.\n");
		} else {
			unsigned short sample_rate;
			hal.dmp_on = 1;
			/* Preserve current FIFO rate. */
			gyro_get_sample_rate(&sample_rate);
			dmp_set_fifo_rate(sample_rate);
			gyro_set_dmp_state(1);
			MPL_LOGI("DMP enabled.\n");
		}
		break;
	case 'x':
		msp430_reset();
		break;
	case 'v':
		/* Toggle LP quaternion. */
		hal.dmp_features ^= DMP_FEATURE_LP_QUAT;
		dmp_enable_feature(hal.dmp_features);
		if (!(hal.dmp_features & DMP_FEATURE_LP_QUAT)) {
			inv_quaternion_sensor_was_turned_off();
			MPL_LOGI("LP quaternion disabled.\n");
		} else
			MPL_LOGI("LP quaternion enabled.\n");
		break;
	default:
		break;
	}
	hal.rx.cmd = 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void) {
	hal.new_gyro = 1;
	TOGGLE_RED_LED;
}
/*Make sure the unused ports are properly initialized
 *    Below information is based on Motion Fit SDK
 *    LED Red  = P1.2
 *    LED Blue = P1.3
 *    P1.4 --> Make it output low: // Not for WSDK
 *    Switch 1 = P2.3
 *    Switch 2 = P2.4
 *    User is expected to update all the I/O port
 *    according to his applications on interconnection
 */
static void msp430_port_init(void) {

	 P1SEL &=~  0x1C; // standard I/o;
	 P1DIR |=   0x1C; // standard I/o - output;
	 P1OUT &=~  0x1C; // standard I/o - output =0;
	 P2SEL &=~  0x18; // standard I/o
	 P2DIR &=~  0x18; // standard I/o - input
	 P2OUT &=~  0x18; // standard I/o - input =0. does not matter

}

/* Set up MSP430 peripherals. */
static inline void platform_init(void) {
	WDTCTL = WDTPW | WDTHOLD;
	SetVCore(2);
	msp430_clock_init(12000000L, 2);
	if (USB_init() != kUSB_succeed)
		msp430_reset();
	msp430_i2c_enable();
	msp430_int_enable();
	msp430_uart_init();
	msp430_port_init();

	USB_setEnabledEvents(kUSB_allUsbEvents);
	if (USB_connectionInfo() & kUSB_vbusPresent) {
		if (USB_enable() == kUSB_succeed) {
			USB_reset();
			USB_connect();
		} else
			msp430_reset();
	}

	hal.int_param.cb = gyro_data_ready_cb;
	hal.int_param.port = 2;
	hal.int_param.pin = 0;
	hal.int_param.lp_exit = INT_EXIT_LPM0;

}

void main(void) {
	inv_error_t result;

	/* Set up MSP430 hardware. */
	platform_init();

	/* Set up gyro.
	 * Every function preceded by gyro_ is a driver function and can be found
	 * in inv_gyro.h.
	 */
	result = gyro_init(&hal.int_param);
	if (result) {
		MPL_LOGE("Could not initialize gyro.\n");
		msp430_reset();
	}

	/* If you're not using an MPU9150 AND you're not using DMP features, this
	 * function will place all slaves on the primary bus.
	 * gyro_set_bypass(1);
	 */

	result = inv_init_mpl();
	if (result) {
		MPL_LOGE("Could not initialize MPL.\n");
		msp430_reset();
	}

	/* Compute 6-axis and 9-axis quaternions. */
	inv_enable_quaternion();
	inv_enable_9x_sensor_fusion();
	/* The MPL expects compass data at a constant rate (matching the rate
	 * passed to inv_set_compass_sample_rate). If this is an issue for your
	 * application, call this function, and the MPL will depend on the
	 * timestamps passed to inv_build_compass instead.
	 *
	 * inv_9x_fusion_use_timestamps(1);
	 */

	/* This function has been deprecated.
	 * inv_enable_no_gyro_fusion();
	 */

	/* Update gyro biases when not in motion.
	 * WARNING: These algorithms are mutually exclusive.
	 */
	inv_enable_fast_nomot();
	/* inv_enable_motion_no_motion(); */
	/* inv_set_no_motion_time(1000); */

	/* Update gyro biases when temperature changes. */
	inv_enable_gyro_tc();

	/* This algorithm updates the accel biases when in motion. A more accurate
	 * bias measurement can be made when running the self-test (see case 't' in
	 * handle_input), but this algorithm can be enabled if the self-test can't
	 * be executed in your application.
	 *
	 * inv_enable_in_use_auto_calibration();
	 */

	/* Compass calibration algorithms. */
	inv_enable_vector_compass_cal();
	inv_enable_magnetic_disturbance();
	/* If you need to estimate your heading before the compass is calibrated,
	 * enable this algorithm. It becomes useless after a good figure-eight is
	 * detected, so we'll just leave it out to save memory.
	 * inv_enable_heading_from_gyro();
	 */

	/* Allows use of the MPL APIs in read_from_mpl. */
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) {
		while (1) {
			MPL_LOGE("Not authorized.\n");
			SET_BOTH_LED;
			msp430_delay_ms(5000);
			CLEAR_BOTH_LED;
			msp430_delay_ms(5000);
		}
	}
	if (result) {
		MPL_LOGE("Could not start the MPL.\n");
		msp430_reset();
	}

	/* Get/set hardware configuration. Start gyro. */
	/* Wake up all sensors. */
	gyro_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	/* Push both gyro and accel data into the FIFO. */
	gyro_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	gyro_set_sample_rate(DEFAULT_MPU_HZ);
	/* The compass sampling rate can be less than the gyro/accel sampling rate.
	 * Use this function for proper power management.
	 */
	gyro_set_compass_sample_rate(1000 / DEFAULT_MAG_MS);
	/* Read back configuration in case it was set improperly. */
	gyro_get_sample_rate(&gyro_rate);
	gyro_get_gyro_fsr(&gyro_fsr);
	gyro_get_accel_fsr(&accel_fsr);
	gyro_get_compass_fsr(&compass_fsr);

	/* set accel full scale range
	 * 2
	 * 4
	 * 8
	 * 16
	 * */
	//  gyro_set_accel_fsr();
	/* Sync driver configuration with MPL. */
	/* Sample rate expected in microseconds. */
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);
	/* The compass rate is independent of the gyro and accel rates. As long as
	 * inv_set_compass_sample_rate is called with the correct value, the 9-axis
	 * fusion algorithm's compass correction gain will work properly.
	 */
	inv_set_compass_sample_rate(DEFAULT_MAG_MS * 1000L);

	//TODO: set quat rate:
	//  inv_set_quat_sample_rate();

	/* Set chip-to-body orientation matrix.
	 * Set hardware units to dps/g's/degrees scaling factor.
	 */
	inv_set_gyro_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) gyro_fsr << 15);
	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) accel_fsr << 15);
	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long) compass_fsr << 15);

	/* Initialize HAL state variables. */
	hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
	hal.dmp_on = 0;
	hal.report = 0;
	hal.rx.cmd = 0;
	hal.next_pedo_ms = 0;
	hal.new_compass = 0;

	/* Compass reads are handled by scheduler. */
	msp430_get_clock_ms(&timestamp);
	hal.read_compass_ms = DEFAULT_MAG_MS;
	hal.next_compass_ms = timestamp + hal.read_compass_ms;

	/* To initialize the DMP:
	 * 1. Call dmp_load_android_firmware(). This pushes the DMP image in
	 *    inv_gyro_dmp_android.h into the MPU memory.
	 * 2. Push the gyro and accel orientation matrix to the DMP.
	 * 3. Register gesture callbacks. Don't worry, these callbacks won't be
	 *    executed unless the corresponding feature is enabled.
	 * 4. Call dmp_enable_feature(mask) to enable different features.
	 * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	 * 6. Call any feature-specific control functions.
	 *
	 * To enable the DMP, just call gyro_set_dmp_state(1). This function can
	 * be called repeatedly to enable and disable the DMP at runtime.
	 *
	 * The following is a short summary of the features supported in the DMP
	 * image provided in inv_gyro_dmp_android.c:
	 * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	 * 200Hz. Integrating the gyro data at higher rates reduces numerical
	 * errors (compared to integration on the MCU at a lower sampling rate).
	 * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	 * DMP_FEATURE_ORIENT: Notify the application when the device orientation
	 * has changed.
	 * DMP_FEATURE_DISPLAY_ORIENT: Google's screen rotation algorithm. Similar
	 * to the feature above, but only triggers an event at the four
	 * orientations where the screen should rotate.
	 */
	dmp_load_android_firmware();
	dmp_set_orientation(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	dmp_register_tap_cb(tap_cb);
	dmp_register_display_orient_cb(display_orient_cb);
	dmp_register_orient_cb(orient_cb);
	/* These features can be enabled/disabled at runtime. If you'd like to
	 * try it, here's an example:
	 * void toggle_display_orient(void) {
	 *     hal.dmp_features ^= DMP_FEATURE_DISPLAY_ORIENT;
	 *     dmp_enable_feature(hal.dmp_features);
	 * }
	 */
	hal.dmp_features = DMP_FEATURE_LP_QUAT | DMP_FEATURE_TAP
			| DMP_FEATURE_ORIENT | DMP_FEATURE_DISPLAY_ORIENT;
	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	hal.read_pedo_ms = 1000;

	__enable_interrupt();

	/* Wait for enumeration. */

	BYTE usb_state;
	while (1) {
		new_data = 0;
		usb_state = USB_connectionState();
		switch (USB_connectionState()) {
		/* This represents the main loop when USB is disconnected. Since the
		 * device is powered by VBUS, this case is unused.
		 */
		case ST_USB_DISCONNECTED:
			/* Idea here is to use this state to enable UART control
			 * Make sure the UART Rx and Tx interrupt is enabled once
			 * Modify the handle input based on the Uart Rx
			 */
			if (check_uart_mode_enabled == 0) {
				msp430_uart_enable();
				check_uart_mode_enabled = 1;
			}
			break;
			/* Unused. */
		case ST_USB_CONNECTED_NO_ENUM:
			if (check_uart_mode_enabled == 0) {
				msp430_uart_enable();
				check_uart_mode_enabled = 1;
			}
			break;
			/* This represents the main loop when USB is connected. */
		case ST_ENUM_ACTIVE:
			/* This represents the main loop when the MSP430 is suspended by the PC.
			 * If the application needs to do anything at this time, take care to
			 * draw as little current as possible.
			 */
			//This mode is enabled when USB is connected. Ideally disable UART mode.
			check_uart_mode_enabled = 0; // Repeated every time.
			break;
		case ST_ENUM_SUSPENDED:
			__bis_SR_register(LPM3_bits + GIE);
			break;
			/* This represents the short time period when the MSP430 is being
			 * enumerated on the PC. Flash an LED or something if you're trying
			 * to be cool.
			 */
		case ST_ENUM_IN_PROGRESS:
			/* The MSP430 was suspended before it could be enumerated. */
		case ST_NOENUM_SUSPENDED:
			break;
		case ST_ERROR:
			msp430_reset();
			break;
		default:
			break;
		}

		if (check_uart_mode_enabled || usb_state == ST_ENUM_ACTIVE) {
			if (rx_new) {
				/* A byte has been received via USB. See handle_input for a list of
				 * valid commands.
				 */
				handle_input();
			}
			msp430_get_clock_ms(&timestamp);

			/* We're not using a data ready interrupt for the compass, so we'll
			 * make our compass reads timer-based instead.
			 */
			if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode
					&& hal.new_gyro && (hal.sensors & COMPASS_ON)) {
				hal.next_compass_ms = timestamp + hal.read_compass_ms;
				hal.new_compass = 1;
			}

			if (!hal.sensors || !hal.new_gyro) {
				/* Put the MSP430 to sleep until a timer interrupt or data ready
				 * interrupt is detected.
				 */
				__bis_SR_register(LPM0_bits + GIE);
				continue;
			}

			if (hal.new_gyro && hal.lp_accel_mode) {
				short accel_short[3];
				long accel[3];
				gyro_get_accel_reg(accel_short, &sensor_timestamp);
				accel[0] = (long) accel_short[0];
				accel[1] = (long) accel_short[1];
				accel[2] = (long) accel_short[2];
				inv_build_accel(accel, 0, sensor_timestamp);
				new_data = 1;
				hal.new_gyro = 0;
			} else if (hal.new_gyro && hal.dmp_on) {
				short gyro[3], accel_short[3], sensors;
				unsigned char more;
				long accel[3], quat[4], temperature;
				/* This function gets new data from the FIFO when the DMP is in
				 * use. The FIFO can contain any combination of gyro, accel,
				 * quaternion, and gesture data. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
				 * the FIFO isn't being filled with accel data.
				 * The driver parses the gesture data to determine if a gesture
				 * event has occured; on an event, the application will be notified
				 * via a callback (assuming that a callback function was properly
				 * registered).
				 * The more parameter is non-zero if there are leftover packets in
				 * the FIFO. The HAL can use this information to increase the
				 * frequency at which this function is called.
				 */
				dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp,
						&sensors, &more);
				if (!more)
					hal.new_gyro = 0;
				if (sensors & INV_XYZ_GYRO) {
					/* Push the new data to the MPL. */
					inv_build_gyro(gyro, sensor_timestamp);
					new_data = 1;
					/* Temperature only used for gyro temp comp. */
					gyro_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
				if (sensors & INV_XYZ_ACCEL) {
					accel[0] = (long) accel_short[0];
					accel[1] = (long) accel_short[1];
					accel[2] = (long) accel_short[2];
					inv_build_accel(accel, 0, sensor_timestamp);
					new_data = 1;
				}
				if (sensors & INV_WXYZ_QUAT) {
					inv_build_quat(quat, 0, sensor_timestamp);
					new_data = 1;
				}
			} else if (hal.new_gyro) {
				short gyro[3], accel_short[3];
				unsigned char sensors, more;
				long accel[3], temperature;
				/* This function gets new data from the FIFO. The FIFO can contain
				 * gyro, accel, both, or neither. The sensors parameter tells the
				 * caller which data fields were actually populated with new data.
				 * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
				 * being filled with accel data. The more parameter is non-zero if
				 * there are leftover packets in the FIFO. The HAL can use this
				 * information to increase the frequency at which this function is
				 * called.
				 */
				gyro_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors,
						&more);
				if (!more)
					hal.new_gyro = 0;
				if (sensors & INV_XYZ_GYRO) {
					/* Push the new data to the MPL. */
					inv_build_gyro(gyro, sensor_timestamp);
					new_data = 1;
					/* Temperature only used for gyro temp comp. */
					gyro_get_temperature(&temperature, &sensor_timestamp);
					inv_build_temp(temperature, sensor_timestamp);
				}
				if (sensors & INV_XYZ_ACCEL) {
					accel[0] = (long) accel_short[0];
					accel[1] = (long) accel_short[1];
					accel[2] = (long) accel_short[2];
					inv_build_accel(accel, 0, sensor_timestamp);
					new_data = 1;
				}
			}
			if (hal.new_compass) {
				short compass_short[3];
				long compass[3];
				hal.new_compass = 0;
				/* For the MPU9150 (or the MPU6050 with an AKM on the auxiliary I2C
				 * bus), the raw magnetometer registers are copied to special gyro
				 * registers.
				 */
				if (!gyro_get_compass_reg(compass_short, &sensor_timestamp)) {
					compass[0] = (long) compass_short[0];
					compass[1] = (long) compass_short[1];
					compass[2] = (long) compass_short[2];
					/* NOTE: If using a third-party compass calibration library,
					 * pass in the compass data in uT * 2^16 and set the second
					 * parameter to INV_CALIBRATED | acc, where acc is the
					 * accuracy from 0 to 3.
					 */
					inv_build_compass(compass, 0, sensor_timestamp);
				}
				new_data = 1;
			}
			if (new_data) {
				inv_execute_on_data();
				/* This function reads bias-compensated sensor data and sensor
				 * fusion outputs from the MPL. The outputs are formatted as seen
				 * in eMPL_outputs.c. This function only needs to be called at the
				 * rate requested by the host.
				 */
				read_from_mpl();
			}
		}
	}
}

