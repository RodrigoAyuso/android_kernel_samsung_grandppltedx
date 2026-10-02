/* BT541_V8_RECOVERY_20260927: experimental confirmed-DELTA recovery. */
/*
 *
 * Zinitix bt541 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/async.h>

#include "bt541_ts.h"
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>

/* sec config */
#define TSP_VERBOSE_DEBUG
#define CONFIG_SEC_FACTORY_TEST
#define SUPPORTED_TOUCH_KEY
#define BT541_USE_INPUT_OPEN_CLOSE	1

#ifdef CONFIG_SEC_FACTORY_TEST
#include "linux/input/sec_cmd.h"
#include <linux/sec_class.h>
#endif
/* MTK DMA I2C*/
#define TPD_SUPPORT_I2C_DMA  1

/* PAT MODE */
#define PAT_CONTROL

#if TPD_SUPPORT_I2C_DMA
#include <linux/dma-mapping.h>
#define _ERROR(e)      ((0x01 << e) | (0x01 << (sizeof(s32) * 8 - 1)))
#define eRROR	       _ERROR(1)
#define GTP_ADDR_LENGTH 		0
#define I2C_MASTER_CLOCK	      400
#define IIC_DMA_MAX_TRANSFER_SIZE     250
#define ERROR_IIC      _ERROR(2)
#endif

#define CHECK_HWID				0

/*zinitix support define */
#define ZINITIX_DEBUG				1
#define ZINITIX_I2C_CHECKSUM			1
#define NOT_SUPPORTED_TOUCH_DUMMY_KEY

#ifdef SUPPORTED_PALM_TOUCH
#define TOUCH_POINT_MODE			2
#else
#define TOUCH_POINT_MODE			0
#endif

#define MAX_SUPPORTED_FINGER_NUM		2 /* max 10 */

#define TC_SECTOR_SZ				8

/* Touch Key define*/
#ifdef SUPPORTED_TOUCH_KEY
#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
#define MAX_SUPPORTED_BUTTON_NUM		2 /* max 8 */
#define SUPPORTED_BUTTON_NUM			2
#else
#define MAX_SUPPORTED_BUTTON_NUM		2 /* max 8 */
#define SUPPORTED_BUTTON_NUM			2
#endif
#endif

/* Upgrade Method*/
#define TOUCH_ONESHOT_UPGRADE			1
/* if you use isp mode, you must add i2c device :
   name = "zinitix_isp" , addr 0x50*/

/* resolution offset */
#define ABS_PT_OFFSET				(0)

#define TOUCH_FORCE_UPGRADE			1
#define USE_CHECKSUM				1

#define CHIP_OFF_DELAY				50 /*ms*/
#define CHIP_ON_DELAY				20 /*ms*/
#define FIRMWARE_ON_DELAY			60 /*ms*/

#define DELAY_FOR_SIGNAL_DELAY			30 /*us*/
#define DELAY_FOR_TRANSCATION			50
#define DELAY_FOR_POST_TRANSCATION		10
#define IUM_SET_TIMEOUT				64 /*1.7s*/
#define BT541_USEC_PER_MSEC	1000
/*PAT MODE*/

#ifdef PAT_CONTROL
/*------------------------------
	<<< apply to server >>>
	0x00 : no action
	0x01 : clear nv
	0x02 : pat magic
	0x03 : rfu

	<<< use for temp bin >>>
	0x05 : forced clear nv & f/w update before pat magic, eventhough same f/w
	0x06 : rfu
-------------------------------*/
#define PAT_CONTROL_NONE			0x00
#define PAT_CONTROL_CLEAR_NV			0x01
#define PAT_CONTROL_PAT_MAGIC			0x02
#define PAT_CONTROL_FORCE_UPDATE		0x05

#define PAT_MAX_LCIA				0x80
#define PAT_MAX_MAGIC				0xF5
#define PAT_MAGIC_NUMBER			0x83
/*addr*/
#define PAT_CAL_DATA				0x00
#define PAT_DUMMY_VERSION			0x02
#define PAT_FIX_VERSION 			0x04
#endif

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

/* Key Enum */
enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

enum fw_sequence {
	fw_false = 0,
	fw_true,
	fw_force,
};


/* ESD Protection */
/*second : if 0, no use. if you have to use, 3 is recommended*/
/*if H/W TSP_ta use, delete ESD */
#define ESD_TIMER_INTERVAL			1
#define SCAN_RATE_HZ				100
#define CHECK_ESD_TIMER				3

/* V8: hardware-dependent experimental detector; never reset on idle alone. */
#define BT541_V8_DEFAULT_AUTO 1
#define BT541_V8_INTERVAL_MS 30000U
#define BT541_V8_QUIET_MS 10000U
#define BT541_V8_CONFIRM_MS 2000U
#define BT541_V8_COOLDOWN_MS 120000U
#define BT541_V8_DELTA_THRESHOLD 500

/*Test Mode (Monitoring Raw Data) */
#define MAX_RAW_DATA_SZ				576 /* 32x18 */
#define MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)
/* preriod raw data interval */

#define RAWDATA_DELAY_FOR_HOST			100

struct raw_ioctl {
	u32 sz;
	u32 buf;
};

struct reg_ioctl {
	u32 addr;
	u32 val;
};

#define TOUCH_SEC_MODE				48
#define TOUCH_REF_MODE				10
#define TOUCH_NORMAL_MODE			5
#define TOUCH_DELTA_MODE			3
#define TOUCH_SDND_MODE				6
#define TOUCH_CNDDATA_MODE			7 /* current raw data */
#define TOUCH_REFERENCE_MODE			8 /* nv raw data */
#define TOUCH_DND_MODE				11
#define TOUCH_H_GAP_JITTER_MODE		16
#define TOUCH_REF_ABNORMAL_TEST_MODE	33

/*  Other Things */
#define INIT_RETRY_CNT				1
#define I2C_SUCCESS				0
#define I2C_FAIL				1

/*---------------------------------------------------------------------*/

/* Register Map*/
#define BT541_SWRESET_CMD			0x0000
#define BT541_WAKEUP_CMD			0x0001

#define BT541_IDLE_CMD				0x0004
#define BT541_SLEEP_CMD				0x0005

#define BT541_CLEAR_INT_STATUS_CMD		0x0003
#define BT541_CALIBRATE_CMD			0x0006
#define BT541_SAVE_STATUS_CMD			0x0007
#define BT541_SAVE_CALIBRATION_CMD		0x0008
#define BT541_RECALL_FACTORY_CMD		0x000f

#define BT541_THRESHOLD				0x0020

#define BT541_DEBUG_REG				0x0115 /* 0~7 */

#define BT541_TOUCH_MODE			0x0010
#define BT541_CHIP_REVISION			0x0011
#define BT541_FIRMWARE_VERSION			0x0012

#define BT541_MINOR_FW_VERSION			0x0121

#define BT541_VENDOR_ID				0x001C
#define BT541_MODULE_ID				0x001E
#define BT541_HW_ID				0x0014

#define BT541_DATA_VERSION_REG			0x0013
#define BT541_SUPPORTED_FINGER_NUM		0x0015
#define BT541_EEPROM_INFO			0x0018
#define BT541_INITIAL_TOUCH_MODE		0x0019

#define BT541_TOTAL_NUMBER_OF_X			0x0060
#define BT541_TOTAL_NUMBER_OF_Y			0x0061

#define BT541_DELAY_RAW_FOR_HOST		0x007f

#define BT541_BUTTON_SUPPORTED_NUM		0x00B0
#define BT541_BUTTON_SENSITIVITY		0x00B2
#define BT541_DUMMY_BUTTON_SENSITIVITY		0X00C8
#define BT541_BTN_WIDTH				0x016d
#define BT541_REAL_WIDTH			0x01c0


#define BT541_X_RESOLUTION			0x00C0
#define BT541_Y_RESOLUTION			0x00C1

#define BT541_POINT_STATUS_REG			0x0080
#define BT541_ICON_STATUS_REG			0x00AA

#define BT541_AFE_FREQUENCY			0x0100
#define BT541_DND_N_COUNT			0x0122
#define BT541_DND_U_COUNT			0x0135
#define BT541_SHIFT_VALUE			0x012B
#define BT541_ISRC_CTRL				0x014F

#define BT541_RAWDATA_REG			0x0200

#define BT541_EEPROM_INFO_REG			0x0018

#define BT541_INT_ENABLE_FLAG			0x00f0
#define BT541_PERIODICAL_INTERRUPT_INTERVAL	0x00f1

#define BT541_CHECKSUM_RESULT			0x012c

#define BT541_INIT_FLASH			0x01d0
#define BT541_WRITE_FLASH			0x01d1
#define BT541_READ_FLASH			0x01d2

#define ZINITIX_INTERNAL_FLAG_02		0x011e

#define BT541_OPTIONAL_SETTING			0x0116

/* Interrupt & status register flag bit
   -------------------------------------------------
 */
#define BIT_PT_CNT_CHANGE			0
#define BIT_DOWN				1
#define BIT_MOVE				2
#define BIT_UP					3
#define BIT_PALM				4
#define BIT_PALM_REJECT				5
#define RESERVED_0				6
#define RESERVED_1				7
#define BIT_WEIGHT_CHANGE			8
#define BIT_PT_NO_CHANGE			9
#define BIT_REJECT				10
#define BIT_PT_EXIST				11
#define RESERVED_2				12
#define BIT_MUST_ZERO				13
#define BIT_DEBUG				14
#define BIT_ICON_EVENT				15

/* button */
#define BIT_O_ICON0_DOWN			0
#define BIT_O_ICON1_DOWN			1
#define BIT_O_ICON2_DOWN			2
#define BIT_O_ICON3_DOWN			3
#define BIT_O_ICON4_DOWN			4
#define BIT_O_ICON5_DOWN			5
#define BIT_O_ICON6_DOWN			6
#define BIT_O_ICON7_DOWN			7

#define BIT_O_ICON0_UP				8
#define BIT_O_ICON1_UP				9
#define BIT_O_ICON2_UP				10
#define BIT_O_ICON3_UP				11
#define BIT_O_ICON4_UP				12
#define BIT_O_ICON5_UP				13
#define BIT_O_ICON6_UP				14
#define BIT_O_ICON7_UP				15


#define SUB_BIT_EXIST				0
#define SUB_BIT_DOWN				1
#define SUB_BIT_MOVE				2
#define SUB_BIT_UP				3
#define SUB_BIT_UPDATE				4
#define SUB_BIT_WAIT				5


#define zinitix_bit_set(val, n)		((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)		((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)		((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)		(((((s) & 0xff) << 8) | (((s) >> 8) & 0xff)))

/* end header file */

#ifdef CONFIG_SEC_FACTORY_TEST

#define MAX_FW_PATH 255
#define TSP_FW_FILENAME "zinitix_fw.bin"
#define TSP_CMD_X_NUM		19
#define TSP_CMD_Y_NUM		10
#define TSP_CMD_NODE_NUM	(TSP_CMD_X_NUM * TSP_CMD_Y_NUM)

struct tsp_raw_data {
	s16 reference_data_abnormal[TSP_CMD_NODE_NUM];
	s16 dnd_data[TSP_CMD_NODE_NUM];
	s16 hfdnd_data[TSP_CMD_NODE_NUM];
	s32 hfdnd_data_sum[TSP_CMD_NODE_NUM];
	s16 delta_data[TSP_CMD_NODE_NUM];
	s16 vgap_data[TSP_CMD_NODE_NUM];
	s16 hgap_data[TSP_CMD_NODE_NUM];
	s16 reference_data[TSP_CMD_NODE_NUM*2 + TSP_CMD_NODE_NUM];
	s16 gapjitter_data[TSP_CMD_NODE_NUM];
};

#if defined(CONFIG_SAMSUNG_LPM_MODE)
	extern unsigned int poweroff_charging;
#endif

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_threshold(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);

/* Vendor dependant command */
static void get_reference(void *device_data);
static void get_dnd(void * device_data);
static void get_hfdnd(void * device_data);
static void get_dnd_v_gap(void * device_data);
static void get_dnd_h_gap(void * device_data);
static void get_hfdnd_v_gap(void * device_data);
static void get_hfdnd_h_gap(void * device_data);
static void get_delta(void *device_data);
static void run_reference_read (void *device_data);
static void run_dnd_read(void *device_data);
static void run_hfdnd_read(void *device_data);
static void run_dnd_v_gap_read(void *device_data);
static void run_dnd_h_gap_read(void * device_data);
static void run_hfdnd_v_gap_read(void *device_data);
static void run_hfdnd_h_gap_read(void * device_data);
static void run_delta_read(void *device_data);
static void hfdnd_spec_adjust(void *device_data);
static void clear_reference_data(void *device_data);
static void run_gapjitter_read(void *device_data);
static void get_gapjitter(void *device_data);
static void run_force_calibration(void *device_data);
#ifdef PAT_CONTROL
static void get_pat_information(void *device_data);
static void get_calibration_nv_data(void *device_data);
static void get_tune_fix_ver_data(void *device_data);
static void set_calibration_nv_data(void *device_data);
static void set_tune_fix_ver_data(void *device_data);
#endif

static void dead_zone_enable(void *device_data);
static void run_mis_cal_read(void *device_data);
static void get_mis_cal(void *device_data);


static void force_recover(void *device_data);

static void get_watchdog_status(void *device_data);
static void set_auto_recover(void *device_data);
static void get_touch_health(void *device_data);
static void v8_fw_update(void *device_data);
static void v8_get_fw_ver_bin(void *device_data);
static void v8_get_fw_ver_ic(void *device_data);
static void v8_get_threshold(void *device_data);
static void v8_module_off_master(void *device_data);
static void v8_module_on_master(void *device_data);
static void v8_module_off_slave(void *device_data);
static void v8_module_on_slave(void *device_data);
static void v8_get_chip_vendor(void *device_data);
static void v8_get_chip_name(void *device_data);
static void v8_get_x_num(void *device_data);
static void v8_get_y_num(void *device_data);
static void v8_not_support_cmd(void *device_data);
static void v8_run_reference_read(void *device_data);
static void v8_get_reference(void *device_data);
static void v8_run_delta_read(void *device_data);
static void v8_get_delta(void *device_data);
static void v8_run_dnd_read(void *device_data);
static void v8_get_dnd(void *device_data);
static void v8_run_dnd_v_gap_read(void *device_data);
static void v8_get_dnd_v_gap(void *device_data);
static void v8_run_dnd_h_gap_read(void *device_data);
static void v8_get_dnd_h_gap(void *device_data);
static void v8_run_hfdnd_read(void *device_data);
static void v8_get_hfdnd(void *device_data);
static void v8_run_hfdnd_v_gap_read(void *device_data);
static void v8_get_hfdnd_v_gap(void *device_data);
static void v8_run_hfdnd_h_gap_read(void *device_data);
static void v8_get_hfdnd_h_gap(void *device_data);
static void v8_run_gapjitter_read(void *device_data);
static void v8_get_gapjitter(void *device_data);
static void v8_hfdnd_spec_adjust(void *device_data);
static void v8_clear_reference_data(void *device_data);
static void v8_run_force_calibration(void *device_data);
static void v8_get_pat_information(void *device_data);
static void v8_get_calibration_nv_data(void *device_data);
static void v8_get_tune_fix_ver_data(void *device_data);
static void v8_set_calibration_nv_data(void *device_data);
static void v8_set_tune_fix_ver_data(void *device_data);
static void v8_dead_zone_enable(void *device_data);
static void v8_run_mis_cal_read(void *device_data);
static void v8_get_mis_cal(void *device_data);
static void v8_force_recover(void *device_data);
static void v8_set_auto_recover(void *device_data);
static struct sec_cmd bt541_commands[] = {
	{SEC_CMD("fw_update", v8_fw_update),},
	{SEC_CMD("get_fw_ver_bin", v8_get_fw_ver_bin),},
	{SEC_CMD("get_fw_ver_ic", v8_get_fw_ver_ic),},
	{SEC_CMD("get_threshold", v8_get_threshold),},
	{SEC_CMD("module_off_master", v8_module_off_master),},
	{SEC_CMD("module_on_master", v8_module_on_master),},
	{SEC_CMD("module_off_slave", v8_module_off_slave),},
	{SEC_CMD("module_on_slave", v8_module_on_slave),},
	{SEC_CMD("get_chip_vendor", v8_get_chip_vendor),},
	{SEC_CMD("get_chip_name", v8_get_chip_name),},
	{SEC_CMD("get_x_num", v8_get_x_num),},
	{SEC_CMD("get_y_num", v8_get_y_num),},
	{SEC_CMD("not_support_cmd", v8_not_support_cmd),},

	/* vendor dependant command */
	{SEC_CMD("run_reference_read", v8_run_reference_read),},
	{SEC_CMD("get_dnd_all_data", v8_get_reference),},
	{SEC_CMD("run_delta_read", v8_run_delta_read),},
	{SEC_CMD("get_delta_all_data", v8_get_delta),},
	{SEC_CMD("run_dnd_read", v8_run_dnd_read),},
	{SEC_CMD("get_dnd", v8_get_dnd),},
	{SEC_CMD("run_dnd_v_gap_read", v8_run_dnd_v_gap_read),},
	{SEC_CMD("get_dnd_v_gap", v8_get_dnd_v_gap),},
	{SEC_CMD("run_dnd_h_gap_read", v8_run_dnd_h_gap_read),},
	{SEC_CMD("get_dnd_h_gap", v8_get_dnd_h_gap),},
	{SEC_CMD("run_hfdnd_read", v8_run_hfdnd_read),},
	{SEC_CMD("get_hfdnd", v8_get_hfdnd),},
	{SEC_CMD("run_hfdnd_v_gap_read", v8_run_hfdnd_v_gap_read),},
	{SEC_CMD("get_hfdnd_v_gap", v8_get_hfdnd_v_gap),},
	{SEC_CMD("run_hfdnd_h_gap_read", v8_run_hfdnd_h_gap_read),},
	{SEC_CMD("get_hfdnd_h_gap", v8_get_hfdnd_h_gap),},
	{SEC_CMD("run_gapjitter_read", v8_run_gapjitter_read),},
	{SEC_CMD("get_gapjitter", v8_get_gapjitter),},
	{SEC_CMD("hfdnd_spec_adjust", v8_hfdnd_spec_adjust),},
	{SEC_CMD("clear_reference_data", v8_clear_reference_data),},
	{SEC_CMD("run_force_calibration", v8_run_force_calibration),},
#ifdef PAT_CONTROL
	{SEC_CMD("get_pat_information", v8_get_pat_information),},
	{SEC_CMD("get_calibration_nv_data", v8_get_calibration_nv_data),},
	{SEC_CMD("get_tune_fix_ver_data", v8_get_tune_fix_ver_data),},
	{SEC_CMD("set_calibration_nv_data", v8_set_calibration_nv_data),},
	{SEC_CMD("set_tune_fix_ver_data", v8_set_tune_fix_ver_data),},
#endif
	{SEC_CMD("dead_zone_enable", v8_dead_zone_enable),},
	{SEC_CMD("run_mis_cal_read", v8_run_mis_cal_read),},
	{SEC_CMD("get_mis_cal", v8_get_mis_cal),},
	{SEC_CMD("force_recover", v8_force_recover),},
	{SEC_CMD("get_watchdog_status", get_watchdog_status),},
	{SEC_CMD("set_auto_recover", v8_set_auto_recover),},
	{SEC_CMD("get_touch_health", get_touch_health),},
};
#endif

#define TSP_NORMAL_EVENT_MSG			1
static int m_ts_debug_mode = ZINITIX_DEBUG;

#if ESD_TIMER_INTERVAL
static struct workqueue_struct *esd_tmr_workqueue;
#endif

struct coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8	minor_width;
	u8	angle;
#endif
};

struct point_info {
	u16	status;
#if (TOUCH_POINT_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
};
struct sec_point_info {
//	u8	finger_cnt;
	u8	finger_state;
	int	move_count;
};

#define TOUCH_V_FLIP				0x01
#define TOUCH_H_FLIP				0x02
#define TOUCH_XY_SWAP				0x04

/*Test Mode (Monitoring Raw Data) */
/*---------------------------------------------------------------------*/
/* GF1 */
/*---------------------------------------------------------------------*/
#define TSP_INIT_TEST_RATIO			100

#define	SEC_DND_N_COUNT				20
#define	SEC_DND_U_COUNT				10
#define	SEC_DND_FREQUENCY			239		/*100khz*/

#define	SEC_HFDND_N_COUNT			20
#define	SEC_HFDND_U_COUNT			10
#define	SEC_HFDND_FREQUENCY			47		/*500khz*/

#define SEC_ISRC_CTRL				0x0F00

/*---------------------------------------------------------------------*/
struct capa_info {
	u16	vendor_id;
	u16	ic_revision;
	u16	fw_version;
	u16	fw_minor_version;
	u16	reg_data_version;
	u16	threshold;
	u16	key_threshold;
	u16	dummy_threshold;
	u32	ic_fw_size;
	u32	MaxX;
	u32	MaxY;
	u32	MinX;
	u32	MinY;
	u8	gesture_support;
	u16	multi_fingers;
	u16	button_num;
	u16	ic_int_mask;
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
	u16	hw_id;
	u16	module_id;
	u16	afe_frequency;
	u16	shift_value;
	u16	N_cnt;
	u16	U_cnt;
	u16 isrc_ctrl;
	u16	i2s_checksum;
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	EALRY_SUSPEND,
	SUSPEND,
	RESUME,
	LATE_RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
	PROBE,
};

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct bt541_ts_info;

struct bt541_ts_platform_data {
	u32			gpio_int;
	u32			gpio_ldo_en;
	u32			x_resolution;
	u32			y_resolution;
	u32			page_size;
	u32			orientation;
	int			bringup;
	int			vdd_en;
	int			vdd_en_flag;
	int			mis_cal_check;
	const char		*fw_name;
	struct regulator	*vreg_vio;
	bool			support_lpm;
#ifdef PAT_CONTROL
	int pat_function;
	int afe_base;
#endif

};

/* V8 diagnostics: counters survive recovery; cached fields use this lock. */
struct bt541_touch_health {
	spinlock_t lock;
	atomic_t irq;
	atomic_t invalid_gpio;
	atomic_t lock_busy;
	atomic_t state_busy;
	atomic_t coord_recovery;
	atomic_t heartbeat;
	atomic_t nonzero_packet;
	atomic_t contacts;
	atomic_t sync;
	atomic_t invalid_coord;
	struct point_info packet;
	u16 packet_mode;
	u64 last_packet_jiffies;
	u64 last_contact_jiffies;
	bool packet_valid;
	bool contact_valid;
};

struct bt541_ts_info {
	struct i2c_client			*client;
	struct input_dev			*input_dev;
	struct bt541_ts_platform_data		*pdata;
	struct pinctrl				*pinctrl;
	char					phys[32];
	struct capa_info			cap_info;
	struct point_info			touch_info;
	struct point_info			reported_touch_info;
	struct sec_point_info			sec_point_info[MAX_SUPPORTED_FINGER_NUM];
	u16					icon_event_reg;
	u16					prev_icon_event;
	int					irq;
	u8					button[MAX_SUPPORTED_BUTTON_NUM];
	u8					work_state;
	struct semaphore			work_lock;
	struct bt541_touch_health		health;
#if ESD_TIMER_INTERVAL
	struct work_struct			tmr_work;
	struct timer_list			esd_timeout_tmr;
	struct timer_list			*p_esd_timeout_tmr;
	spinlock_t				lock;
	struct mutex v8_control_lock;
	struct delayed_work v8_work;
	bool v8_stopping;
	bool v8_ready;
	bool v8_busy;
	bool v8_auto_enabled;
	bool v8_cooldown_valid;
	atomic_t v8_screen_on;
	struct notifier_block v8_fb;
	bool v8_fb_registered;
	struct device *v8_factory_tk_dev;
	struct device *v8_sec_pretest_dev;
	bool v8_sec_initialized;
	bool v8_factory_tk_group_created;
	struct file *v8_raw_owner;
	unsigned long v8_last_point, v8_next_recovery;
	u32 v8_point_seq, v8_probe_seq;
	bool v8_suspect;
	u32 v8_checks, v8_recoveries, v8_attempts, v8_errors, v8_peak;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend			early_suspend;
#endif
	struct semaphore			raw_data_lock;
	u16					touch_mode;
	s16					cur_data[MAX_TRAW_DATA_SZ];
	u8					update;
	int					octa_id;
	bool					enabled;
#ifdef CONFIG_SEC_FACTORY_TEST
	struct sec_cmd_data			sec;
	struct tsp_raw_data			*raw_data;
	int					touch_count;
	s16 Gap_max_x;
	s16 Gap_max_y;
	s16 Gap_max_val;
	s16 Gap_min_x;
	s16 Gap_min_y;
	s16 Gap_min_val;
	s16 Gap_Gap_val;
	s16 Gap_node_num;
	u32 version;
	bool latest_flag;
	bool pat_flag;
#endif
#ifdef PAT_CONTROL
	int cal_count;
	int tune_fix_ver;
#endif
	bool ium_lock_enable;
};
/* Dummy touchkey code */
#define KEY_DUMMY_HOME1				249
#define KEY_DUMMY_HOME2				250
#define KEY_DUMMY_MENU				251
#define KEY_DUMMY_HOME				252
#define KEY_DUMMY_BACK				253
/*<= you must set key button mapping*/
#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_RECENT, KEY_BACK};
#else
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_DUMMY_MENU, KEY_RECENT,// KEY_DUMMY_HOME1,
	/*KEY_DUMMY_HOME2,*/ KEY_BACK, KEY_DUMMY_BACK};
#endif

static int bt541_ts_open(struct input_dev *dev);
static void bt541_ts_close(struct input_dev *dev);
int bt541_pinctrl_configure(struct bt541_ts_info *info, int active);



#if TPD_SUPPORT_I2C_DMA
static s32 i2c_dma_read_mtk(struct bt541_ts_info *info,char *write_buf, unsigned int wlen, u8 *buffer, s32 len);
static s32 i2c_dma_write_mtk(struct bt541_ts_info *info, u8 *buffer, s32 len);
static u8 *gpDMABuf_va;
static dma_addr_t gpDMABuf_pa;
struct mutex dma_mutex;
DEFINE_MUTEX(dma_mutex);

static s32 i2c_dma_write_mtk(struct bt541_ts_info *info, u8 *buffer, s32 len)
{
	s32 ret = 0;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = 0;
	int count = 0;
	struct i2c_msg msg = {
		.flags = !I2C_M_RD,
		.ext_flag = (info->client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		.addr = (info->client->addr & I2C_MASK_FLAG),
		.timing = I2C_MASTER_CLOCK,
		.buf = (u8 *)(uintptr_t)gpDMABuf_pa,
	};

	if (buffer != NULL)
		address = (buffer[0]<<8) | buffer[1];

	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > (IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH))
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE - GTP_ADDR_LENGTH;
		else
			transfer_length = len - pos;

		gpDMABuf_va[0] = (address >> 8) & 0xFF;
		gpDMABuf_va[1] = address & 0xFF;
		memcpy(&gpDMABuf_va[GTP_ADDR_LENGTH], &buffer[pos], transfer_length);

		msg.len = transfer_length + GTP_ADDR_LENGTH;
		//if (!info->is_lpm_suspend) {/*workround log too much*/
		/* LineageOS: require complete DMA write transfer. */
		count = 0;
retry:
		ret = i2c_transfer(info->client->adapter, &msg, 1);
		if (ret != 1) {
			if (++count < 3) {
				usleep_range(200, 200);
				goto retry;
			}
			input_err(true, &info->client->dev,
					"%s I2C DMA write failed (ret=%d)\n",
					__func__, ret);
			ret = ERROR_IIC;
			break;
		}
		ret = 0;
		pos += transfer_length;
		address += transfer_length;
	}
	mutex_unlock(&dma_mutex);
	return ret;
}

static s32 i2c_dma_read_mtk(struct bt541_ts_info *info ,char *write_buf, unsigned int wlen, u8 *buffer, s32 len)
{
	s32 ret = eRROR;
	s32 pos = 0;
	s32 transfer_length;
	u16 address = 0;
	int count = 0;
	//u8 addr_buf[GTP_ADDR_LENGTH] = { 0 };

	struct i2c_msg msgs[2] = {
		{
		 .flags = 0,	/*!I2C_M_RD,*/
		 .addr = ( info->client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .len = wlen,
		 .buf = write_buf,
		 },
		{
		 .flags = I2C_M_RD,
		 .ext_flag = (info->client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
		 .addr = (info->client->addr & I2C_MASK_FLAG),
		 .timing = I2C_MASTER_CLOCK,
		 .buf = (u8 *)(uintptr_t)gpDMABuf_pa,
		},
	};

	if (write_buf != NULL)
		address = (write_buf[0]<<8) | write_buf[1];

	mutex_lock(&dma_mutex);
	while (pos != len) {
		if (len - pos > IIC_DMA_MAX_TRANSFER_SIZE)
			transfer_length = IIC_DMA_MAX_TRANSFER_SIZE;
		else
			transfer_length = len - pos;

		msgs[0].buf[0] = (address >> 8) & 0xFF;
		msgs[0].buf[1] = address & 0xFF;
		msgs[1].len = transfer_length;
		/* LineageOS: retry complete DMA register read transaction. */
		count = 0;
retry:
		ret = i2c_transfer(info->client->adapter, &msgs[0], 1);
		if (ret != 1) {
			if (++count < 3) {
				usleep_range(200, 200);
				goto retry;
			}
			input_err(true, &info->client->dev,
					"%s I2C DMA address phase failed (ret=%d)\n",
					__func__, ret);
			ret = ERROR_IIC;
			goto out;
		}

		usleep_range(200, 200);
		ret = i2c_transfer(info->client->adapter, &msgs[1], 1);
		if (ret != 1) {
			if (++count < 3) {
				usleep_range(200, 200);
				goto retry;
			}
			input_err(true, &info->client->dev,
					"%s I2C DMA data phase failed (ret=%d)\n",
					__func__, ret);
			ret = ERROR_IIC;
			goto out;
		}

		ret = 0;
		memcpy(&buffer[pos], gpDMABuf_va, transfer_length);
		pos += transfer_length;
		address += transfer_length;
	};
out:
	mutex_unlock(&dma_mutex);
	return ret;
}

#endif


static int cal_mode;
static int get_boot_mode(char *str)
{
	get_option(&str, &cal_mode);
	printk("%s get_boot_mode, uart_mode : %d\n", SECLOG, cal_mode);
	return 1;
}
__setup("calmode=", get_boot_mode);
/* define i2c sub functions*/
static inline s32 read_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	int count = 0;

#if TPD_SUPPORT_I2C_DMA
	ret = i2c_dma_read_mtk(info, (u8 *)&reg, 2, values, length);
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return ret;
#else	
retry:
	/* select register*/
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);
		if (++count < 8)
			goto retry;

		return ret;
	}
	/* for setup tx transaction. */
	usleep_range(DELAY_FOR_TRANSCATION, DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
#endif
}

static inline s32 write_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	int count = 0;
	u8 pkt[10]; /* max packet */
	pkt[0] = (reg) & 0xff; /* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

#if TPD_SUPPORT_I2C_DMA
	ret = i2c_dma_write_mtk(info, pkt, length + 2);
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return ret;

#else
retry:
	ret = i2c_master_send(client , pkt , length + 2);
	if (ret < 0) {
		usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);

		if (++count < 8)
			goto retry;

		input_info(true, &info->client->dev, "%s, count = %d\n", __func__, count);

		return ret;
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;	
#endif
}

static inline s32 write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static inline s32 write_cmd(struct i2c_client *client, u16 reg)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	int count = 0;

#if TPD_SUPPORT_I2C_DMA
	ret = i2c_dma_write_mtk(info, (u8 *)&reg, 2);
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return ret;
	
#else
retry:
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);

		if (++count < 8)
			goto retry;

		return ret;
	}

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return I2C_SUCCESS;
#endif
}

static inline s32 read_raw_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	int count = 0;

#if TPD_SUPPORT_I2C_DMA
	ret = i2c_dma_read_mtk(info, (u8 *)&reg, 2, values, length);
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return ret;	
#else
retry:
	/* select register */
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);

		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	usleep_range(200, 200);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
#endif
}

static inline s32 read_firmware_data(struct i2c_client *client,
		u16 addr, u8 *values, u16 length)
{
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	int ret = 0;
	/* select register*/
#if TPD_SUPPORT_I2C_DMA
	ret = i2c_dma_read_mtk(info, (u8 *)&addr, 2, values, length);
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return ret;
#else
	ret = i2c_master_send(client , (u8 *)&addr , 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;
	usleep_range(DELAY_FOR_POST_TRANSCATION, DELAY_FOR_POST_TRANSCATION);
	return length;
#endif
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bt541_ts_early_suspend(struct early_suspend *h);
static void bt541_ts_late_resume(struct early_suspend *h);
#endif

static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl);
static int bt541_power(struct bt541_ts_info *info, int enable);

static bool init_touch(struct bt541_ts_info *info, int fw_oneshot_upgrade);
static bool mini_init_touch(struct bt541_ts_info *info);
static void clear_report_data(struct bt541_ts_info *info);
#if ESD_TIMER_INTERVAL
static void esd_timer_start(u16 sec, struct bt541_ts_info *info);
static void esd_timer_stop(struct bt541_ts_info *info);
static void esd_timer_init(struct bt541_ts_info *info);
static void esd_timeout_handler(unsigned long data);
#endif

static long ts_misc_fops_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.unlocked_ioctl = ts_misc_fops_ioctl,
	.compat_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};

#define TOUCH_IOCTL_BASE			0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION		_IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE		_IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA		_IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE		_IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE		_IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA		_IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION		_IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG			_IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG			_IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT		_IOW(TOUCH_IOCTL_BASE, 19, int)

struct bt541_ts_info *misc_info;
static DEFINE_MUTEX(bt541_device_lock);

static u16 m_optional_mode = 0;
static u16 m_prev_optional_mode = 0;
static void bt541_set_optional_mode(struct bt541_ts_info *info, bool force)
{
	if (m_prev_optional_mode == m_optional_mode && !force)
		return;

	if (write_reg(info->client, BT541_OPTIONAL_SETTING, m_optional_mode) == I2C_SUCCESS) {
		m_prev_optional_mode = m_optional_mode;
		input_info(true, &misc_info->client->dev, "TA setting changed to %d\n",
				m_optional_mode&0x1);
	}
}

#define I2C_BUFFER_SIZE 64
/*
 * Integration fragment for bt541_ts.c, base SHA-256:
 * f8ffdebedfe7d8f81b41fb1cb899c041aeccd8d34dd6322b48df82b6a2030dfd
 *
 * This is not a translation unit.  Insert the RAW helpers in place of the
 * original get_raw_data(), the MODE helpers in place of all three original
 * ts_set_touchmode*() functions, and replace get_raw_data_size() with the
 * final wrapper.  No watchdog policy is introduced here.
 */

/* BEGIN RAW HELPERS + get_raw_data REPLACEMENT */
#define BT541_RAW_FRAME_TIMEOUT_MS 1000
#define BT541_RAW_READY_TIMEOUT_MS 50
#define BT541_RAW_MAX_SKIP_FRAMES 10

static bool bt541_raw_dimensions_valid(struct bt541_ts_info *info)
{
	u32 x = info->cap_info.x_node_num;
	u32 y = info->cap_info.y_node_num;
	u32 nodes;

	if (!x || !y || x > MAX_RAW_DATA_SZ || y > MAX_RAW_DATA_SZ)
		return false;
	nodes = x * y;
	if (nodes > MAX_RAW_DATA_SZ || nodes != info->cap_info.total_node_num)
		return false;
#ifdef CONFIG_SEC_FACTORY_TEST
	/* Factory arrays and their specification tables have fixed dimensions. */
	if (x > TSP_CMD_X_NUM || y > TSP_CMD_Y_NUM ||
			nodes > TSP_CMD_NODE_NUM)
		return false;
#endif
	return true;
}

static bool bt541_wait_raw_ready(struct bt541_ts_info *info,
		unsigned int timeout_ms)
{
	unsigned int elapsed;

	for (elapsed = 0; elapsed < timeout_ms; elapsed++) {
		if (!gpio_get_value(info->pdata->gpio_int))
			return true;
		msleep(1);
	}
	return !gpio_get_value(info->pdata->gpio_int);
}

/*
 * requested_bytes == 0 selects one complete node matrix.  The buffer stays
 * unchanged on every error, including the final interrupt acknowledgement.
 * Callers still have to check the return and restore POINT mode separately.
 */
static bool bt541_get_raw_data_common(struct bt541_ts_info *info, u8 *buff,
		int skip_cnt, int requested_bytes, unsigned int capacity)
{
	u8 *staging;
	unsigned int bytes, offset, block;
	unsigned int chunk;
	s32 ret;
	int i;
	bool ok = false;

	if (!info || !buff || !capacity || requested_bytes < 0 ||
			skip_cnt < 0 || skip_cnt > BT541_RAW_MAX_SKIP_FRAMES)
		return false;
	if (requested_bytes && ((requested_bytes & 1) ||
			(unsigned int)requested_bytes > capacity))
		return false;

	staging = kmalloc(capacity, GFP_KERNEL);
	if (!staging)
		return false;

	/* Balanced with factory callers which have already disabled this IRQ. */
	disable_irq(info->irq);
	down(&info->work_lock);
	if (!info->enabled || info->work_state != NOTHING) {
		input_err(true, &info->client->dev,
				"raw read unavailable: enabled=%d state=%d\n",
				info->enabled, info->work_state);
		goto out_unlock;
	}
	if (!bt541_raw_dimensions_valid(info)) {
		input_err(true, &info->client->dev,
				"raw read rejected invalid dimensions %u x %u, total=%u\n",
				info->cap_info.x_node_num, info->cap_info.y_node_num,
				info->cap_info.total_node_num);
		goto out_unlock;
	}
	bytes = requested_bytes ? (unsigned int)requested_bytes :
			(unsigned int)info->cap_info.total_node_num * sizeof(s16);
	if (!bytes || bytes > capacity)
		goto out_unlock;

	info->work_state = RAW_DATA;
	for (i = 0; i < skip_cnt; i++) {
		if (!bt541_wait_raw_ready(info, BT541_RAW_FRAME_TIMEOUT_MS)) {
			input_err(true, &info->client->dev,
					"raw read timeout discarding frame %d\n", i);
			goto out_state;
		}
		if (write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD) !=
				I2C_SUCCESS) {
			input_err(true, &info->client->dev,
					"raw read failed to acknowledge skipped frame\n");
			goto out_state;
		}
		msleep(1);
	}

	if (!bt541_wait_raw_ready(info, BT541_RAW_READY_TIMEOUT_MS)) {
		input_err(true, &info->client->dev,
				"raw read timeout waiting for final frame\n");
		goto out_state;
	}

	for (offset = 0, block = 0; offset < bytes; block++) {
		chunk = min_t(unsigned int, I2C_BUFFER_SIZE, bytes - offset);
		ret = read_raw_data(info->client, BT541_RAWDATA_REG + block,
				staging + offset, chunk);
#if TPD_SUPPORT_I2C_DMA
		if (ret != I2C_SUCCESS) {
#else
		if (ret != chunk) {
#endif
			input_err(true, &info->client->dev,
					"raw read failed at block %u (ret=%d)\n", block, ret);
			goto out_state;
		}
		offset += chunk;
	}
	if (write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS) {
		input_err(true, &info->client->dev,
				"raw read failed to acknowledge final frame\n");
		goto out_state;
	}
	memcpy(buff, staging, bytes);
	ok = true;

out_state:
	info->work_state = NOTHING;
out_unlock:
	up(&info->work_lock);
	enable_irq(info->irq);
	kfree(staging);
	return ok;
}

static bool get_raw_data(struct bt541_ts_info *info, u8 *buff, int skip_cnt)
{
#ifdef CONFIG_SEC_FACTORY_TEST
	return bt541_get_raw_data_common(info, buff, skip_cnt, 0,
			TSP_CMD_NODE_NUM * sizeof(s16));
#else
	return bt541_get_raw_data_common(info, buff, skip_cnt, 0,
			MAX_RAW_DATA_SZ * sizeof(s16));
#endif
}
/* END RAW HELPERS + get_raw_data REPLACEMENT */

/* BEGIN MODE HELPERS + THREE ts_set_touchmode REPLACEMENTS */
enum bt541_mode_variant {
	BT541_MODE_STANDARD,
	BT541_MODE_HFDND,
	BT541_MODE_GAPJITTER,
};

static bool bt541_mode_write(struct bt541_ts_info *info, u16 reg, u16 value)
{
	if (write_reg(info->client, reg, value) == I2C_SUCCESS)
		return true;
	input_err(true, &info->client->dev,
			"mode change: register 0x%04x write failed\n", reg);
	return false;
}

static bool bt541_mode_clear_frames(struct bt541_ts_info *info)
{
	int i;

	for (i = 0; i < 10; i++) {
		/* Original nominal delay was 20 ms; do not use local HZ as a unit. */
		msleep(20);
		if (write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD) !=
				I2C_SUCCESS) {
			input_err(true, &info->client->dev,
					"mode change: interrupt clear failed at frame %d\n", i);
			return false;
		}
	}
	return true;
}

/*
 * Try every restoration write even after an earlier failure.  N_COUNT was
 * changed by DND/HFDND too, so restore its saved normal value as well.
 * The GAPJITTER variant keeps its original omission of SHIFT_VALUE writes.
 */
static bool bt541_mode_restore_normal(struct bt541_ts_info *info,
		enum bt541_mode_variant variant)
{
	bool ok = true;

	if (!bt541_mode_write(info, BT541_AFE_FREQUENCY,
			info->cap_info.afe_frequency))
		ok = false;
	if (!bt541_mode_write(info, BT541_DND_U_COUNT, info->cap_info.U_cnt))
		ok = false;
	if (variant != BT541_MODE_GAPJITTER &&
			!bt541_mode_write(info, BT541_SHIFT_VALUE,
			info->cap_info.shift_value))
		ok = false;
	if (!bt541_mode_write(info, BT541_ISRC_CTRL, info->cap_info.isrc_ctrl))
		ok = false;
	if (!bt541_mode_write(info, BT541_DND_N_COUNT, info->cap_info.N_cnt))
		ok = false;
	return ok;
}

/* IRQ is masked and work_lock is held by the caller. */
static bool bt541_set_touchmode_locked(struct bt541_ts_info *info, u16 value,
		enum bt541_mode_variant variant)
{
	u16 mode = value == TOUCH_SEC_MODE ? TOUCH_POINT_MODE : value;
	u16 old_mode = info->touch_mode;
	bool test_tuning;
	bool restored;
	u16 n_count, u_count, frequency;

	test_tuning = variant == BT541_MODE_GAPJITTER ?
			mode == TOUCH_H_GAP_JITTER_MODE : mode == TOUCH_DND_MODE;
	info->update = 0;

	if (test_tuning) {
		n_count = variant == BT541_MODE_STANDARD ?
				SEC_DND_N_COUNT : SEC_HFDND_N_COUNT;
		u_count = variant == BT541_MODE_STANDARD ?
				SEC_DND_U_COUNT : SEC_HFDND_U_COUNT;
		frequency = variant == BT541_MODE_STANDARD ?
				SEC_DND_FREQUENCY : SEC_HFDND_FREQUENCY;
		if (!bt541_mode_write(info, BT541_DND_N_COUNT, n_count) ||
				!bt541_mode_write(info, BT541_DND_U_COUNT, u_count) ||
				!bt541_mode_write(info, BT541_AFE_FREQUENCY, frequency) ||
				!bt541_mode_write(info, BT541_ISRC_CTRL, SEC_ISRC_CTRL))
			goto rollback;
	} else if (variant != BT541_MODE_STANDARD ||
			old_mode == TOUCH_DND_MODE ||
			old_mode == TOUCH_H_GAP_JITTER_MODE) {
		if (!bt541_mode_restore_normal(info, variant))
			goto rollback;
	}

	if (mode != TOUCH_POINT_MODE &&
			!bt541_mode_write(info, BT541_DELAY_RAW_FOR_HOST,
			RAWDATA_DELAY_FOR_HOST))
		goto rollback;
	if (!bt541_mode_write(info, BT541_TOUCH_MODE, mode))
		goto rollback;
	if (!bt541_mode_clear_frames(info))
		goto rollback;

	/* Publish only a completed hardware transition. */
	info->touch_mode = mode;
	info->update = 0;
	return true;

rollback:
	restored = bt541_mode_restore_normal(info, variant);
	if (!bt541_mode_write(info, BT541_TOUCH_MODE, TOUCH_POINT_MODE))
		restored = false;
	if (!bt541_mode_clear_frames(info))
		restored = false;
	if (restored) {
		info->touch_mode = TOUCH_POINT_MODE;
		info->update = 0;
	} else {
		/* Leave the last confirmed mode cached; it is not hardware proof. */
		input_err(true, &info->client->dev,
				"mode change: POINT rollback failed, hardware state unknown\n");
	}
	return false;
}

static bool bt541_set_touchmode(struct bt541_ts_info *info, u16 value,
		enum bt541_mode_variant variant)
{
	bool ok = false;

	if (!info)
		return false;
	/* Also protects TOUCH_IOCTL_SET_RAW_DATA_MODE, whose caller has no mask. */
	disable_irq(info->irq);
	down(&info->work_lock);
	if (!info->enabled || info->work_state != NOTHING) {
		input_err(true, &info->client->dev,
				"mode change unavailable: enabled=%d state=%d\n",
				info->enabled, info->work_state);
		goto out;
	}
	info->work_state = SET_MODE;
	ok = bt541_set_touchmode_locked(info, value, variant);
	info->work_state = NOTHING;
out:
	up(&info->work_lock);
	enable_irq(info->irq);
	return ok;
}

static bool ts_set_touchmode(u16 value)
{
	return bt541_set_touchmode(misc_info, value, BT541_MODE_STANDARD);
}

static bool ts_set_touchmode2(u16 value)
{
	return bt541_set_touchmode(misc_info, value, BT541_MODE_HFDND);
}

static bool ts_set_touchmode16(u16 value)
{
	return bt541_set_touchmode(misc_info, value, BT541_MODE_GAPJITTER);
}
/* END MODE HELPERS + THREE ts_set_touchmode REPLACEMENTS */

/* BEGIN get_raw_data_size REPLACEMENT (within CONFIG_SEC_FACTORY_TEST) */
#ifdef CONFIG_SEC_FACTORY_TEST
static bool get_raw_data_size(struct bt541_ts_info *info, u8 *buff,
		int skip_cnt, int sz)
{
	if (sz <= 0)
		return false;
	return bt541_get_raw_data_common(info, buff, skip_cnt, sz,
			sizeof(((struct tsp_raw_data *)0)->reference_data));
}
#endif
/* END get_raw_data_size REPLACEMENT */




static bool ts_get_raw_data(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	s32 sz;
	u32 temp_sz;
	int i;

	if (down_trylock(&info->raw_data_lock)) {
		input_err(true, &client->dev, "Failed to occupy sema\n");
		info->touch_info.status = 0;
		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);
	for (i = 0; sz > 0; i++) {
		temp_sz = I2C_BUFFER_SIZE;
		if (sz	< I2C_BUFFER_SIZE)
			temp_sz = sz;
		if (read_raw_data(info->client, BT541_RAWDATA_REG + i,
					(char *)((u8*)(info->cur_data) + (i * I2C_BUFFER_SIZE)), temp_sz) < 0) {
			input_err(true, &client->dev, "Failed to read raw data %d, %d \n", i, temp_sz);
			up(&info->raw_data_lock);
			return false;
		}
		sz -= I2C_BUFFER_SIZE;
	}

	info->update = 1;
	memcpy((u8 *)(&info->touch_info),
			(u8 *)&info->cur_data[total_node],
			sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

#if ZINITIX_I2C_CHECKSUM
#define ZINITIX_I2C_CHECKSUM_WCNT		0x016a
#define ZINITIX_I2C_CHECKSUM_RESULT		0x016c
static bool i2c_checksum(struct bt541_ts_info *info, s16 *pChecksum, u16 wlength)
{
	s16 checksum_result;
	s16 checksum_cur;
	int i;

	checksum_cur = 0;
	for (i = 0; i < wlength; i++) {
		checksum_cur += (s16)pChecksum[i];
	}
	if (read_data(info->client,
				ZINITIX_I2C_CHECKSUM_RESULT,
				(u8 *)(&checksum_result), 2) < 0) {
		input_err(true, &info->client->dev, "error read i2c checksum rsult.-\n");
		return false;
	}
	if (checksum_cur != checksum_result) {
		input_err(true, &info->client->dev, "checksum error : %d, %d\n", checksum_cur, checksum_result);
		return false;
	}
	return true;
}

#endif

static bool ts_read_coord(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
#if (TOUCH_POINT_MODE == 1)
	int i;
#endif

	/* for	Debugging Tool */

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (info->update == 0) {
			if (ts_get_raw_data(info) == false)
				return false;
		} else {
			info->touch_info.status = 0;
		}

		input_err(true, &client->dev, "status = 0x%04X\n", info->touch_info.status);

		goto out;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&info->touch_info,
			0x0, sizeof(struct point_info));

#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if ((write_reg(info->client, ZINITIX_I2C_CHECKSUM_WCNT, 2)) != I2C_SUCCESS)
			return false;

#endif
	if (read_data(info->client, BT541_POINT_STATUS_REG, (u8 *)(&info->touch_info), 4) < 0) {
		input_err(true, &client->dev, "%s: Failed to read point info\n", __func__);

		return false;
	}

#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (i2c_checksum(info, (s16 *)(&info->touch_info), 2) == false)
		return false;
#endif
	input_info(true, &client->dev, "status reg = 0x%x , event_flag = 0x%04x\n",
			info->touch_info.status, info->touch_info.event_flag);

	bt541_set_optional_mode(info, false);
	if (info->touch_info.event_flag == 0)
		goto out;

#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if ((write_reg(info->client, ZINITIX_I2C_CHECKSUM_WCNT, sizeof(struct point_info)/2)) != I2C_SUCCESS)
			return false;
#endif
	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			usleep_range(20, 20);

			if (read_data(info->client, BT541_POINT_STATUS_REG + 2 + ( i * 4),
						(u8 *)(&info->touch_info.coord[i]),
						sizeof(struct coord)) < 0) {
				input_err(true, &client->dev, "Failed to read point info\n");

				return false;
			}
#if ZINITIX_I2C_CHECKSUM
			if (info->cap_info.i2s_checksum)
				if (i2c_checksum(info, (s16 *)(&info->touch_info.coord[i]), sizeof(struct point_info)/2) == false)
					return false;
#endif
		}
	}

#else
#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (write_reg(info->client,
					ZINITIX_I2C_CHECKSUM_WCNT,
					(sizeof(struct point_info)/2)) != I2C_SUCCESS) {
			input_err(true, &client->dev, "error write checksum wcnt.-\n");
			return false;
		}
#endif
	if (read_data(info->client, BT541_POINT_STATUS_REG,
				(u8 *)(&info->touch_info), sizeof(struct point_info)) < 0) {
		input_err(true, &client->dev, "Failed to read point info\n");

		return false;
	}
#if ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (i2c_checksum(info, (s16 *)(&info->touch_info), sizeof(struct point_info)/2) == false)
			return false;
#endif

	bt541_set_optional_mode(info, false);

#endif

out:
	/* error */
	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO)) {
		input_err(true, &client->dev, "Invalid must zero bit(%04x)\n",
				info->touch_info.status);

		return false;
	}

	write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);

	return true;
}

#if ESD_TIMER_INTERVAL
static void esd_timeout_handler(unsigned long data)
{
	struct bt541_ts_info *info = (struct bt541_ts_info *)data;
	if (READ_ONCE(info->v8_ready) && READ_ONCE(info->enabled) && !READ_ONCE(info->v8_stopping) &&
			!READ_ONCE(info->v8_busy))
		queue_work(esd_tmr_workqueue, &info->tmr_work);
}

static void esd_timer_start(u16 sec, struct bt541_ts_info *info)
{
	if (READ_ONCE(info->v8_ready) && READ_ONCE(info->enabled) && !READ_ONCE(info->v8_stopping) &&
			!READ_ONCE(info->v8_busy))
		mod_timer(&info->esd_timeout_tmr,
			jiffies + msecs_to_jiffies((unsigned int)sec * 1000U));
}

static void esd_timer_stop(struct bt541_ts_info *info)
{
	del_timer_sync(&info->esd_timeout_tmr);
}

static void esd_timer_init(struct bt541_ts_info *info)
{
	setup_timer(&info->esd_timeout_tmr, esd_timeout_handler, (unsigned long)info);
}

static void ts_tmr_work(struct work_struct *work)
{
	struct bt541_ts_info *info =
		container_of(work, struct bt541_ts_info, tmr_work);
	struct i2c_client *client = info->client;

#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "tmr queue work ++\n");
#endif

	if (!READ_ONCE(info->enabled) || READ_ONCE(info->v8_stopping) ||
			READ_ONCE(info->v8_busy))
		return;

	if (down_trylock(&info->work_lock)) {
		input_err(true, &client->dev, "%s: Failed to occupy work lock\n", __func__);

		/* LineageOS: do not rearm ESD while suspend/remove owns work_lock. */
		if (info->enabled &&
				info->work_state != EALRY_SUSPEND &&
				info->work_state != SUSPEND &&
				info->work_state != REMOVE)
			esd_timer_start(CHECK_ESD_TIMER, info);

		return;
	}

	if (!READ_ONCE(info->enabled) || READ_ONCE(info->v8_stopping) ||
			READ_ONCE(info->v8_busy)) {
		up(&info->work_lock);
		return;
	}

	if (info->work_state != NOTHING) {
		bool rearm_esd;

		/* LineageOS: rearm ESD watchdog only while touch is operational. */
		rearm_esd = info->enabled &&
			info->work_state != EALRY_SUSPEND &&
			info->work_state != SUSPEND &&
			info->work_state != REMOVE;

		input_info(true, &client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);

		if (rearm_esd)
			esd_timer_start(CHECK_ESD_TIMER, info);

		return;
	}

	if(info->ium_lock_enable == true)
		input_info(true, &client->dev, "ium_lock\n", __func__);

	info->work_state = ESD_TIMER;
	info->touch_mode = TOUCH_POINT_MODE;
	info->update = 0;
	info->v8_point_seq++;
	info->v8_last_point = jiffies;

	disable_irq(info->irq);
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	clear_report_data(info);
	if (mini_init_touch(info) == false)
		goto fail_time_out_init;

	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "tmr queue work--\n");
#endif

	return;
fail_time_out_init:
	input_err(true, &client->dev, "%s: Failed to restart\n", __func__);
	/* LineageOS: do not resurrect ESD after suspend started. */
	if (info->enabled)
		esd_timer_start(CHECK_ESD_TIMER, info);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return;
}
#endif


/*
 * V6: recover a half-alive BT541 without unregistering the input/I2C
 * device. The reproduced failure still answers I2C/raw-data requests and
 * keeps the periodic ESD interrupt alive, while point events disappear.
 *
 * Do not automatically reset on status == 0: the BT541 uses zero-status
 * periodic interrupts during normal idle too.
 */
static bool bt541_v8_recover_locked(struct bt541_ts_info *info,
		const char *reason)
{
	bool off_ok, on_ok, ok = false;

	/* IRQ is disabled and work_lock is held by the caller. */
	info->v8_suspect = false;
	info->v8_point_seq++;
	info->v8_last_point = jiffies;
	clear_report_data(info);
	info->touch_mode = TOUCH_POINT_MODE;
	info->update = 0;
	input_info(true, &info->client->dev, "BT541 V8 recovery: %s\n", reason);
	off_ok = bt541_power_control(info, POWER_OFF);
	on_ok = bt541_power_control(info, POWER_ON_SEQUENCE);
	if (on_ok)
		ok = mini_init_touch(info);
	return off_ok && on_ok && ok;
}

static bool bt541_force_recovery(struct bt541_ts_info *info,
		const char *reason)
{
	bool ok = false;

	if (!info || !info->input_dev || !info->enabled || info->v8_stopping)
		return false;
	WRITE_ONCE(info->v8_busy, true);
	disable_irq(info->irq);
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
	if (down_trylock(&info->work_lock))
		goto out_irq;
	if (info->work_state == NOTHING) {
		info->work_state = ESD_TIMER;
		ok = bt541_v8_recover_locked(info, reason);
		info->work_state = NOTHING;
	}
	up(&info->work_lock);
out_irq:
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
	esd_timer_start(CHECK_ESD_TIMER, info);
	return ok;
}





/*
 * Learn only from consecutive idle heartbeat packets.
 *
 * Any real point packet resets the learning window.  This prevents normal
 * touch activity from falsely proving that time_stamp is an idle liveness
 * counter.
 */





static bool bt541_power_sequence(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;
	u16 chip_code;

retry_power_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "Failed to send power sequence(vendor cmd enable)\n");
		goto fail_power_sequence;
	}
	usleep_range(10, 10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		input_err(true, &client->dev, "Failed to read chip code\n");
		goto fail_power_sequence;
	}

	input_info(true, &client->dev, "%s: chip code = 0x%x\n", __func__, chip_code);
	usleep_range(10, 10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		input_err(true, &client->dev, "Failed to send power sequence(intn clear)\n");
		goto fail_power_sequence;
	}
	usleep_range(10, 10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "Failed to send power sequence(nvm init)\n");
		goto fail_power_sequence;
	}
	usleep_range(2*BT541_USEC_PER_MSEC, 2*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "Failed to send power sequence(program start)\n");
		goto fail_power_sequence;
	}
	usleep_range(FIRMWARE_ON_DELAY*BT541_USEC_PER_MSEC, FIRMWARE_ON_DELAY*BT541_USEC_PER_MSEC);	/* wait for checksum cal */

	if (write_reg(client, 0x002E, IUM_SET_TIMEOUT) != I2C_SUCCESS)
		input_err(true, &client->dev, "%s: failed to set ium timeout\n", __func__);

	return true;

fail_power_sequence:
	if (retry++ < 3) {
		usleep_range(CHIP_ON_DELAY*BT541_USEC_PER_MSEC, CHIP_ON_DELAY*BT541_USEC_PER_MSEC);
		input_info(true, &client->dev, "retry = %d\n", retry);
		goto retry_power_sequence;
	}

	input_err(true, &client->dev, "Failed to send power sequence\n");
	return false;
}

static int bt541_power(struct bt541_ts_info *info, int enable)
{
	struct i2c_client *client = info->client;
	int ret = 0;

	if (info->pdata->vdd_en_flag) {
		gpio_direction_output(info->pdata->gpio_ldo_en, enable);
		if(enable == 0)
			usleep_range(CHIP_OFF_DELAY*BT541_USEC_PER_MSEC, CHIP_OFF_DELAY*BT541_USEC_PER_MSEC);
		else
			usleep_range(CHIP_ON_DELAY*BT541_USEC_PER_MSEC, CHIP_ON_DELAY*BT541_USEC_PER_MSEC);
		input_info(true, &client->dev, "%s gpio_direction_ouput:%d\n", __func__, enable);
	}

	if (!IS_ERR_OR_NULL(info->pdata->vreg_vio)) {
		if (enable) {
			if (!regulator_is_enabled(info->pdata->vreg_vio)) {
				ret = regulator_enable(info->pdata->vreg_vio);
				if (ret) {
					input_err(true, &client->dev, "%s [ERROR] touch_regulator enable "
						"failed  (%d)\n", __func__, ret);
					return -EIO;
				}
				usleep_range(CHIP_ON_DELAY*BT541_USEC_PER_MSEC, CHIP_ON_DELAY*BT541_USEC_PER_MSEC);
				input_info(true, &client->dev, "%s power on\n", __func__);

			} else {
				input_info(true, &client->dev, "%s already power on\n", __func__);
			}
		} else {
			if (regulator_is_enabled(info->pdata->vreg_vio)) {
				ret = regulator_disable(info->pdata->vreg_vio);
				if(ret) {
					input_err(true, &client->dev, "%s [ERROR] touch_regulator disable "
						"failed (%d)\n", __func__, ret);
					return -EIO;
				}
				usleep_range(CHIP_ON_DELAY*BT541_USEC_PER_MSEC, CHIP_ON_DELAY*BT541_USEC_PER_MSEC);
				input_info(true, &client->dev, "%s power off\n", __func__);

			} else {
				input_info(true, &client->dev, "%s already power off\n", __func__);
			}
		}

	}

	return 0;
}

static bool bt541_power_control(struct bt541_ts_info *info, u8 ctl)
{
	int ret = 0;

	input_info(true, &info->client->dev, "%s, %d\n", __func__, ctl);

	ret = bt541_power(info, ctl);
	if (ret)
		return false;

	if (ctl == POWER_ON_SEQUENCE) {
		return bt541_power_sequence(info);
	}
	info->ium_lock_enable = false;

	return true;
}

#define DEF_IUM_ADDR	64*2
#define DEF_IUM_ADDR_OFFSET	0xF0A0
#define DEF_IUM_LOCK	0xF0F6
#define DEF_IUM_UNLOCK	0xF0FA

bool tsp_nvm_ium_lock(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	if(info->ium_lock_enable == false)
	{
		if(write_cmd(client, DEF_IUM_LOCK))
		{
			input_err(true, &client->dev, "failed ium lock\n", __func__);
			return false;
		}
		info->ium_lock_enable = true;
		msleep(40);
	}

	return true;
}
bool tsp_nvm_ium_unlock(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;

	if(write_cmd(client, DEF_IUM_UNLOCK))
	{
		input_err(true, &client->dev, "failed ium unlock\n", __func__);
		return false;
	}

	info->ium_lock_enable = false;
	return true;
}

#ifdef PAT_CONTROL
/* Use TSP NV area
 * buff[0] : cal_count data or tune_fix_verison
 * addr : 0x00 cal_count 0x02 and 0x04 tune_fix_version and tune_dummy_fix_version
 */
void set_tsp_nvm_data(struct bt541_ts_info *info, u8 addr, u8 data)
{
	struct i2c_client *client = info->client;
	char buff[2] = { 0 };

	input_info(true, &info->client->dev, "%s\n", __func__);

	buff[0] = data;

	if (write_data(client, addr + DEF_IUM_ADDR_OFFSET,
			(u8 *)buff, 2) < 0) {
		input_err(true, &client->dev, "%s error : write zinitix \n", __func__);
		goto fail_ium_random_write;
	}

	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to write nvm wp disable\n", __func__);
		goto fail_ium_random_write;
	}
	
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	if (write_cmd(client, 0xF0F8) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed save ium\n", __func__);
		goto fail_ium_random_write;
	}
	usleep_range(30*BT541_USEC_PER_MSEC, 30*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n", __func__);
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	return;

fail_ium_random_write:
	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	return;
	
}

int get_tsp_nvm_data(struct bt541_ts_info *info, u8 addr)
{
	struct i2c_client *client = info->client;
	char buff[2] = { 0 };

	input_info(true, &info->client->dev, "%s, addr:%u\n", __func__, addr);

	/* send NV data using command
	 * Use TSP NV area : in this model, use only one byte
	 * buff[0] : cal_count_data or tune_fix_data
	*/

	if (info->ium_lock_enable == false) {
		if (write_cmd(client, DEF_IUM_LOCK)) {
			input_err(true, &client->dev, "failed ium lock\n", __func__);
			goto fail_ium_random_read;
		}
		info->ium_lock_enable = true;
		msleep(40);
	}

	memset(&buff, 0x00, 2);
	if (read_raw_data(client, addr + DEF_IUM_ADDR_OFFSET, (u8 *)buff, 2) < 0) {
		input_err(true, &client->dev, "Failed to read raw data \n");
		goto fail_ium_random_read;
	}
	

	return buff[0];

fail_ium_random_read:

	info->ium_lock_enable = false;
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	return -1;
	
}
/* zinitix test source */
#if 0
static void ium_random_write(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int i;
	int page_sz = info->pdata->page_size;
	u16 temp_sz, size;
	u16 chip_code;

	u8 buff[64]; // custom data buffer
	u16 length, buff_start;

	//Enable IRQ
	disable_irq(info->irq);

////////////// input custom data
	for( i=0 ; i<64 ; i++)
		buff[i] = i;
///////////// input custom data end
	
//////////data write start
	buff_start = 4;	//custom setting address(0~62)
	length = 8;		// custom odd number setting(max 64)
	if(length > TC_SECTOR_SZ)
		length = TC_SECTOR_SZ;
	if (write_data(client, buff_start + DEF_IUM_ADDR_OFFSET,
			(u8 *)&buff[buff_start], length) < 0) {
		input_err(true, &client->dev, "error : write zinitix tc firmare\n");
		goto fail_ium_random_write;
	}


	buff_start += TC_SECTOR_SZ;	//custom setting address(0~62)
	length = 6;					// custom setting(max 64)
	if(length > TC_SECTOR_SZ)
		length = TC_SECTOR_SZ;

	if (write_data(client, buff_start + DEF_IUM_ADDR_OFFSET,
			(u8 *)&buff[buff_start], length) < 0) {
		input_err(true, &client->dev, "error : write zinitix tc firmare\n");
		goto fail_ium_random_write;
	}
//////////data write end

//////////for save rom start
	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to write nvm wp disable\n");
		goto fail_ium_random_write;
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	if (write_cmd(client, 0xF0F8) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed save ium\n");
		goto fail_ium_random_write;
	}
	usleep_range(30*BT541_USEC_PER_MSEC, 30*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
//////////for save rom end

	enable_irq(info->irq);
	return;

fail_ium_random_write:
	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	enable_irq(info->irq);
	return;
}
static void ium_random_read(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	int i;
	int page_sz = info->pdata->page_size;
	u16 temp_sz, size;
	u16 chip_code;

	u8 buff[64]; // custom data buffer
	u16 length, buff_start;

	//Enable IRQ
	disable_irq(info->irq);

	for(i=0 ; i<64 ; i++)
		buff[i] = i;

	buff_start = 8;	//custom setting address(0~62)
	length = 6;		// custom setting(max 64)
	if(length > TC_SECTOR_SZ)
		length = TC_SECTOR_SZ;

	if (read_raw_data(client, buff_start + DEF_IUM_ADDR_OFFSET, (u8 *)&buff[buff_start], length) < 0) {
		input_err(true, &client->dev, "Failed to read raw data %d\n", length);
		goto fail_ium_random_read;
	}
	

	enable_irq(info->irq);
	return;

fail_ium_random_read:

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	enable_irq(info->irq);
	return;
}
static void ium_write(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 flash_addr;
	int i;
	int page_sz = info->pdata->page_size;
	u32	size;
	u16 chip_code;

	u8 buff[64*3]; // custom data buffer(buff[128]~)

	////////////// input custom data
	///////////// input custom data end
	//Enable IRQ
	disable_irq(info->client->irq);
	clear_report_data(info);

	size = page_sz*3;		

	for( i=DEF_IUM_ADDR ; i<size ; i++)
			buff[i] = i;
	
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	musleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (vendor cmd enable)\n");
		goto fail_ium_write;
	}

	usleep_range(10, 10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (intn clear)\n");
		goto fail_ium_write;
	}

	usleep_range(10, 10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (nvm init)\n");
		goto fail_ium_write;
	}

	usleep_range(5*BT541_USEC_PER_MSEC, 5*BT541_USEC_PER_MSEC);

	input_info(true, &client->dev, "init flash\n");

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to write nvm vpp on\n");
		goto fail_ium_write;
	}

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
		goto fail_ium_write;
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
	
	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to init flash\n");
		goto fail_ium_write;
	}

	input_info(true, &client->dev, "writing ium data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		if(flash_addr == DEF_IUM_ADDR)
		{
			if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
				input_err(true, &client->dev, "failed to write nvm wp disable\n");
				goto fail_ium_write;
			}
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
		}
		for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
			/*zinitix_debug_msg("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);*/
			/*zinitix_printk(KERN_INFO "writing :addr = %04x, len=%d \n", flash_addr, TC_SECTOR_SZ);*/
			if (write_data(client, BT541_WRITE_FLASH,
					(u8 *)&buff[flash_addr], TC_SECTOR_SZ) < 0) {
				input_err(true, &client->dev, "error : write zinitix tc firmare\n");
				goto fail_ium_write;
			}
			flash_addr += TC_SECTOR_SZ;
			usleep_range(100, 100);

		}

		usleep_range(30*BT541_USEC_PER_MSEC, 30*BT541_USEC_PER_MSEC); /*for fuzing delay*/
	}

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm write vpp off\n");
		goto fail_ium_write;
	}

	input_info(true, &client->dev,"ium write done\n");

fail_ium_write:
	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
	}
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	enable_irq(info->client->irq);
	return;
}
static void ium_read(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 flash_addr;
	int i;
	int page_sz = info->pdata->page_size;
	u32 size;
	u16 chip_code;
	u8 buff[64*3]; // custom data buffer(buff[128]~)

	//Enable IRQ
	disable_irq(info->client->irq);
	clear_report_data(info);

	size = page_sz*3;		

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (vendor cmd enable)\n");
		goto fail_ium_read;
	}

	usleep_range(10, 10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (intn clear)\n");
		goto fail_ium_read;
	}

	usleep_range(10, 10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (nvm init)\n");
		goto fail_ium_read;
	}

	usleep_range(5*BT541_USEC_PER_MSEC, 5*BT541_USEC_PER_MSEC);

	input_info(true, &client->dev,"init flash\n");

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to init flash\n");
		goto fail_ium_read;
	}

	input_info(true, &client->dev, "read ium data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
			/*zinitix_debug_msg("read :addr=%04x, len=%d\n",flash_addr, TC_SECTOR_SZ);*/
			if (read_firmware_data(client,
						BT541_READ_FLASH,
						(u8 *)&buff[flash_addr], TC_SECTOR_SZ) < 0) {
				input_err(true, &client->dev, "Failed to read ium_data\n");

				goto fail_ium_read;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}
	for (i = 128; i < 191; i++) {
		input_info(true, &client->dev, "buf %d \n", buff[i]);
	}
	
	input_info(true, &client->dev,"ium read done\n");

fail_ium_read:

	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	enable_irq(info->client->irq);

	return;
}
#endif
#endif

#if TOUCH_ONESHOT_UPGRADE
static bool ts_check_need_upgrade(struct bt541_ts_info *info, const u8 *firmware_data,
		u16 cur_version, u16 cur_minor_version, u16 cur_reg_version, u16 cur_hw_id)
{
	u16 new_version;
	u16 new_minor_version;
	u16 new_reg_version;
	/*u16 new_chip_code;*/
#if CHECK_HWID
	u16 new_hw_id;
#endif
	new_version = (u16) (firmware_data[0x34] | (firmware_data[0x35]<<8));
	new_minor_version = (u16) (firmware_data[0x38] | (firmware_data[0x39]<<8));
	new_reg_version = (u16) (firmware_data[0x3C] | (firmware_data[0x3D]<<8));
	/*new_chip_code = (u16) (m_firmware_data[64] | (m_firmware_data[65]<<8));*/

#if CHECK_HWID
	new_hw_id =  (u16)(firmware_data[0x7528] | (firmware_data[0x7529]<<8));
	input_info(true, &info->client->dev, "cur HW_ID = 0x%x, new HW_ID = 0x%x\n",
			cur_hw_id, new_hw_id);
#endif

	input_info(true, &info->client->dev, "cur version = 0x%x, new version = 0x%x\n",
			cur_version, new_version);

	input_info(true, &info->client->dev, "cur minor version = 0x%x, new minor version = 0x%x\n",
			cur_minor_version, new_minor_version);
	input_info(true, &info->client->dev, "cur reg data version = 0x%x, new reg data version = 0x%x\n",
			cur_reg_version, new_reg_version);

	if (cal_mode) {
		input_info(true, &info->client->dev, "didn't update TSP F/W!! in CAL MODE\n");
		return false;
	}

	if (cur_version > 0xFF)
		return true;
	if (cur_version < new_version)
		return true;
	else if (cur_version > new_version)
		return false;
#if CHECK_HWID
	if (cur_hw_id != new_hw_id)
		return true;
#endif
	if (cur_minor_version < new_minor_version)
		return true;
	else if (cur_minor_version > new_minor_version)
		return false;
	if (cur_reg_version < new_reg_version)
		return true;

	return false;
}
#endif

static u8 ts_upgrade_firmware(struct bt541_ts_info *info,
		const u8 *firmware_data, u32 size, int fw_state)
{
	struct i2c_client *client = info->client;
	u32 flash_addr;
	u16 reg_val;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;
	int page_sz = info->pdata->page_size;
	u16 chip_code;

	info->latest_flag = false;
	verify_data = kzalloc(size, GFP_KERNEL);
	if (verify_data == NULL) {
		input_err(true, &client->dev, "cannot alloc verify buffer\n");
		return false;
	}

	if (fw_state == fw_false) {
		input_info(true, &client->dev, "%s don't fw update\n", __func__);
		kfree(verify_data);
		return true;
	} else if (fw_state == fw_force) {
		goto retry_upgrade;
	}
	
#ifdef PAT_CONTROL
/* PAT_CONTROL_FORCE_UPDATE : device restore */
	if (info->pdata->pat_function == PAT_CONTROL_FORCE_UPDATE) {
		input_info(true, &client->dev, "%s rune forced f/w update and excute autotune \n", __func__);
		goto retry_upgrade;
	}
#endif

	if (ts_check_need_upgrade(info, firmware_data, info->cap_info.fw_version,
		info->cap_info.fw_minor_version, info->cap_info.reg_data_version,
		info->cap_info.hw_id) == false) {
		info->latest_flag = true; // latest version flag
		input_info(true, &client->dev, "%s fw_version latest version\n", __func__);
		kfree(verify_data);
		return true;
	}


retry_upgrade:
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON);
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (vendor cmd enable)\n");
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (read_data(client, 0xcc00, (u8 *)&chip_code, 2) < 0) {
		input_err(true, &client->dev, "failed to read chip code\n");
		goto fail_upgrade;
	}

	input_dbg(true, &client->dev, "chip code = 0x%x\n", chip_code);
	usleep_range(10, 10);

	flash_addr = (firmware_data[0x61]<<16) | (firmware_data[0x62]<<8) | firmware_data[0x63];
	flash_addr += ((firmware_data[0x65]<<16) | (firmware_data[0x66]<<8) | firmware_data[0x67]);
	
	if(flash_addr != 0 && flash_addr <= 0x10000)
		size = flash_addr;

	if (write_reg(client, 0xc201, 0x00be) != I2C_SUCCESS) {
		dev_err(&client->dev, "power sequence error (set clk speed)\n");
		goto fail_upgrade;
	}

	input_info(true, &client->dev, "f/w size = 0x%x\n", size);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (intn clear)\n");
		goto fail_upgrade;
	}

	usleep_range(10, 10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "power sequence error (nvm init)\n");
		goto fail_upgrade;
	}

	usleep_range(5*BT541_USEC_PER_MSEC, 5*BT541_USEC_PER_MSEC);

	input_info(true, &client->dev, "init flash\n");

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to write nvm vpp on\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to write nvm wp disable\n");
		goto fail_upgrade;
	}

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to init flash\n");
		goto fail_upgrade;
	}

	input_info(true, &client->dev, "writing firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
#ifdef PAT_CONTROL
/* zinitix patch, devide firmware section and calibration section when fw update. */ 
		if(flash_addr == DEF_IUM_ADDR)
		{
			if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
				input_err(true, &client->dev, "nvm wp enable\n");
				goto fail_upgrade;
			}
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
			for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
				/*zinitix_debug_msg("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);*/
				/*zinitix_printk(KERN_INFO "writing :addr = %04x, len=%d \n", flash_addr, TC_SECTOR_SZ);*/
				if (write_data(client, BT541_WRITE_FLASH,
						(u8 *)&firmware_data[flash_addr], TC_SECTOR_SZ) < 0) {
					input_err(true, &client->dev, "error : write zinitix tc firmare\n");
					goto fail_upgrade;
				}
				flash_addr += TC_SECTOR_SZ;
				usleep_range(100, 100);
			
			}
			
			if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS) {
				input_err(true, &client->dev, "failed to write nvm wp disable\n");
				goto fail_upgrade;
			}
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
		}
		else
#endif
		{
			for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
				/*zinitix_debug_msg("write :addr=%04x, len=%d\n",	flash_addr, TC_SECTOR_SZ);*/
				/*zinitix_printk(KERN_INFO "writing :addr = %04x, len=%d \n", flash_addr, TC_SECTOR_SZ);*/
				if (write_data(client, BT541_WRITE_FLASH,
						(u8 *)&firmware_data[flash_addr], TC_SECTOR_SZ) < 0) {
					input_err(true, &client->dev, "error : write zinitix tc firmare\n");
					goto fail_upgrade;
				}
				flash_addr += TC_SECTOR_SZ;
				usleep_range(100, 100);

			}
		}
		msleep(30);

	}

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm write vpp off\n");
		goto fail_upgrade;
	}

	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS) {
		input_err(true, &client->dev, "nvm wp enable\n");
		goto fail_upgrade;
	}

	input_info(true, &client->dev,"init flash\n");

	if (write_cmd(client, BT541_INIT_FLASH) != I2C_SUCCESS) {
		input_err(true, &client->dev, "failed to init flash\n");
		goto fail_upgrade;
	}

	input_info(true, &client->dev, "read firmware data\n");

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < page_sz / TC_SECTOR_SZ; i++) {
			/*zinitix_debug_msg("read :addr=%04x, len=%d\n",flash_addr, TC_SECTOR_SZ);*/
			if (read_firmware_data(client,
						BT541_READ_FLASH,
						(u8 *)&verify_data[flash_addr], TC_SECTOR_SZ) < 0) {
				input_err(true, &client->dev, "Failed to read firmare\n");

				goto fail_upgrade;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}
	/* verify */
	input_info(true, &client->dev, "verify firmware data\n");
#ifdef PAT_CONTROL
	for (i = DEF_IUM_ADDR; i < DEF_IUM_ADDR + page_sz; i++)
		verify_data[i] = firmware_data[i];
#endif
	if (memcmp((u8 *)&firmware_data[0], (u8 *)&verify_data[0], size) == 0) {
		input_info(true, &client->dev, "upgrade finished\n");
		if (verify_data) {
			kfree(verify_data);
			verify_data = NULL;
		}
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		for (i = 0; i < 5; i++) {
			if (read_data(client, BT541_CHECKSUM_RESULT,
					(u8 *)&reg_val, 2) < 0) {
				usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
				continue;
			}
		}

		if (reg_val != 0x55aa) {
			input_err(true, &client->dev, "upgrade done, but firmware checksum fail. reg_val = %x\n", reg_val);
			goto fail_upgrade;
		}

		return true;
	}

fail_upgrade:
	bt541_power_control(info, POWER_OFF);

	if (retry_cnt++ < INIT_RETRY_CNT) {
		input_err(true, &client->dev, "upgrade failed : so retry... (%d)\n", retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data)
		kfree(verify_data);

	input_info(true, &client->dev, "Failed to upgrade\n");

	return false;
}

int bt541_fw_update_from_kernel(struct bt541_ts_info *info, int fw_state)
{
	const struct firmware *fw;
	int retires = 3;
	int ret;
	char fw_path[64];

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	disable_irq(info->irq);
	clear_report_data(info);

	snprintf(fw_path, 64, "%s", info->pdata->fw_name);

	request_firmware(&fw, fw_path, &info->client->dev);

	if (!fw) {
		input_err(true, &info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		enable_irq(info->irq);
		goto ERROR;
	}

	//Update fw
	do {
		ret = ts_upgrade_firmware(info, fw->data, fw->size, fw_state);
		if (ret == true) {
			break;
		}
	} while (--retires);

	if (!retires) {
		input_err(true, &info->client->dev, "%s [ERROR] bt541_flash_fw failed\n", __func__);
		ret = -1;
	}

	release_firmware(fw);

	//Enable IRQ
	enable_irq(info->irq);

	if (ret < 0) {
		goto ERROR;
	}

	input_err(true, &info->client->dev, "%s [DONE]\n", __func__);
	return true;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return false;
	
	
}


static bool ts_hw_calibration(struct bt541_ts_info *info)
{
	struct i2c_client *client = info->client;
	u16	chip_eeprom_info;
	int time_out = 0;
	bool lock_flag = false;

	if (info->ium_lock_enable == true) { 
		if (tsp_nvm_ium_unlock(info) == false) {
			input_err(true, &client->dev, "failed ium unlock\n", __func__);
			return false;
		}
		lock_flag = true;
	}
	
	if (write_reg(client, BT541_TOUCH_MODE, 0x07) != I2C_SUCCESS)
		return false;
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	msleep(50);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);;

	if (write_cmd(client, BT541_CALIBRATE_CMD) != I2C_SUCCESS)
		return false;

	if (write_cmd(client, BT541_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;

	usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

	/* wait for h/w calibration*/
	do {
		msleep(500);
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

		if (read_data(client, BT541_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0)
			return false;

		input_dbg(true, &client->dev, "touch eeprom info = 0x%04X\r\n",
				chip_eeprom_info);
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;

		if (time_out++ == 4) {
			write_cmd(client, BT541_CALIBRATE_CMD);
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			input_err(true, &client->dev, "h/w calibration retry timeout.\n");
		}

		if (time_out++ > 10) {
			input_err(true, &client->dev, "h/w calibration timeout.\n");
			break;
		}

	} while (1);

	if (write_reg(client,
				BT541_TOUCH_MODE, TOUCH_POINT_MODE) != I2C_SUCCESS)
		return false;

	if (info->cap_info.ic_int_mask != 0) {
		if (write_reg(client,
					BT541_INT_ENABLE_FLAG,
					info->cap_info.ic_int_mask)
				!= I2C_SUCCESS)
			return false;
	}

	write_reg(client, 0xc003, 0x0001);
	write_reg(client, 0xc104, 0x0001);
	usleep_range(100, 100);
	if (write_cmd(client, BT541_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
		return false;

	msleep(1000);
	write_reg(client, 0xc003, 0x0000);
	write_reg(client, 0xc104, 0x0000);

	if (lock_flag) {
		if (tsp_nvm_ium_lock(info) == false) {
			input_err(true, &client->dev, "failed ium lock\n", __func__);
		}
	}
	return true;
}

static bool init_touch(struct bt541_ts_info *info, int fw_oneshot_upgrade)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;
#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;
	bool magic_cal = false;

retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_EEPROM_INFO_REG,
					(u8 *)&chip_eeprom_info, 2) < 0) {
			input_err(true, &client->dev, "Failed to read eeprom info(%d)\n", i);
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	input_dbg(true, &client->dev, "%s: Check checksum\n", __func__);

	checksum_err = 0;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT541_CHECKSUM_RESULT,
					(u8 *)&chip_check_sum, 2) < 0) {
			usleep_range(10*BT541_USEC_PER_MSEC, 10*BT541_USEC_PER_MSEC);
			continue;
		}

#if defined(TSP_VERBOSE_DEBUG)
		input_info(true, &client->dev, "chip_check_sum 0x%04X\n", chip_check_sum);
#endif

		if (chip_check_sum == 0x55aa)
			break;
		else {
			checksum_err = 1;
			break;
		}
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		input_err(true, &client->dev, "Failed to check firmware data\n");
		if (checksum_err == 1 && retry_cnt < INIT_RETRY_CNT)
			retry_cnt = INIT_RETRY_CNT;

		goto fail_init;
	}
#endif

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		input_err(true, &client->dev, "Failed to write reset command\n");
		goto fail_init;
	}

	cap->button_num = SUPPORTED_BUTTON_NUM;

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
#if (TOUCH_POINT_MODE == 2)
	zinitix_bit_set(reg_val, BIT_PALM);
	zinitix_bit_set(reg_val, BIT_PALM_REJECT);
#endif

	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);

	cap->ic_int_mask = reg_val;

	if (write_reg(client, BT541_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

	input_dbg(true, &client->dev, "%s: Send reset command\n", __func__);
	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (read_data(client, BT541_VENDOR_ID,
				(u8 *)&cap->vendor_id, 2) < 0) {
		input_err(true, &client->dev, "failed to read chip revision\n");
		goto fail_init;
	}


	if (read_data(client, BT541_CHIP_REVISION,
				(u8 *)&cap->ic_revision, 2) < 0) {
		input_err(true, &client->dev, "failed to read chip revision\n");
		goto fail_init;
	}

	cap->ic_fw_size = 32*1024;

	if (read_data(client, BT541_HW_ID, (u8 *)&cap->hw_id, 2) < 0) {
		input_err(true, &client->dev, "Failed to read hw id\n");
		goto fail_init;
	}

	if (read_data(client, BT541_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_BUTTON_SENSITIVITY,
				(u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	/*if (read_data(client, BT541_DUMMY_BUTTON_SENSITIVITY,
				(u8 *)&cap->dummy_threshold, 2) < 0)
			goto fail_init;*/

	if (read_data(client, BT541_TOTAL_NUMBER_OF_X,
				(u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_TOTAL_NUMBER_OF_Y,
				(u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;


	if (read_data(client, BT541_SHIFT_VALUE,
				(u8 *)&cap->shift_value, 2) < 0)
		goto fail_init;
	input_dbg(true, &client->dev, "Shift value = %d\n", cap->shift_value);


	if (read_data(client, BT541_DND_N_COUNT,
				(u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;
	input_dbg(true, &client->dev, "N count = %d\n", cap->N_cnt);

	if (read_data(client, BT541_DND_U_COUNT,
				(u8 *)&cap->U_cnt, 2) < 0)
		goto fail_init;
	input_dbg(true, &client->dev, "u count = %d\n", cap->U_cnt);

	if (read_data(client, BT541_AFE_FREQUENCY,
				(u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;
	input_dbg(true, &client->dev, "afe frequency = %d\n", cap->afe_frequency);

	
	if (read_data(client, BT541_ISRC_CTRL,
				(u8 *)&cap->isrc_ctrl, 2) < 0)
		goto fail_init;
	input_dbg(true, &client->dev, "isrc_ctrl = %d\n", cap->isrc_ctrl);

	/* get chip firmware version */
	if (read_data(client, BT541_FIRMWARE_VERSION,
				(u8 *)&cap->fw_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_MINOR_FW_VERSION,
				(u8 *)&cap->fw_minor_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT541_DATA_VERSION_REG,
				(u8 *)&cap->reg_data_version, 2) < 0)
		goto fail_init;

	if (!info->pdata->bringup) {
#if TOUCH_ONESHOT_UPGRADE
/* fw_true : nomal, fw_false : Don't fw update, fw_force : Do force fw update */
		if (!bt541_fw_update_from_kernel(info, fw_oneshot_upgrade))
			goto fail_init;  
#ifdef PAT_CONTROL
		if (write_reg(client, 0x002E, IUM_SET_TIMEOUT) != I2C_SUCCESS) {
			input_err(true, &client->dev, "%s: failed to set ium timeout\n", __func__);
			//goto fail_init;
		}		
		if (tsp_nvm_ium_lock(info) == false) {
			input_err(true, &client->dev, "failed ium lock\n", __func__);
			goto fail_init;
		}
		if (info->pdata->pat_function == PAT_CONTROL_CLEAR_NV && info->latest_flag == false) { /* pat_function(1) */
			input_info(true, &client->dev, "%s ts_hw_calibration start \n", __func__);
			if (ts_hw_calibration(info) == false)
				goto fail_init;
			set_tsp_nvm_data(info, PAT_CAL_DATA, 0);
			if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0)
				goto fail_init; /* get fix_tune_version */
			if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0)
				goto fail_init;
			if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0)
				goto fail_init;
			set_tsp_nvm_data(info, PAT_DUMMY_VERSION, (info->cap_info.fw_version << 4) | info->cap_info.fw_minor_version);
			set_tsp_nvm_data(info, PAT_FIX_VERSION, info->cap_info.reg_data_version);
		} else if (info->pdata->pat_function == PAT_CONTROL_PAT_MAGIC ||
				(info->pdata->pat_function == PAT_CONTROL_FORCE_UPDATE)) { /* pat_function(2)) */
			info->cal_count = get_tsp_nvm_data(info, PAT_CAL_DATA);
			info->tune_fix_ver = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) |
				get_tsp_nvm_data(info, PAT_FIX_VERSION);
			if(info->cal_count == -1 || info->tune_fix_ver == -1)
				goto fail_init;        
			if (info->pdata->afe_base > info->tune_fix_ver)
				magic_cal = true;
			if ((info->cal_count == 0) || (magic_cal == true) || info->pat_flag == true ||
				(info->pdata->pat_function == PAT_CONTROL_FORCE_UPDATE)) { /* || pat_function(5)*/
				input_info(true, &client->dev, "%s ts_hw_calibration start \n", __func__);
				if (ts_hw_calibration(info) == false)
					goto fail_init;
				if(info->pat_flag == true) {
					info->cal_count++;
					set_tsp_nvm_data(info, PAT_CAL_DATA, info->cal_count);
					info->pat_flag = false;
				} else {
					set_tsp_nvm_data(info, PAT_CAL_DATA, PAT_MAGIC_NUMBER);
				}
				
				if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0)
					goto fail_init; /* get fix_tune_version */
				if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0)
					goto fail_init;
				if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0)
					goto fail_init;
				set_tsp_nvm_data(info, PAT_DUMMY_VERSION, (info->cap_info.fw_version << 4) | info->cap_info.fw_minor_version);
				set_tsp_nvm_data(info, PAT_FIX_VERSION, info->cap_info.reg_data_version);
			}
		} else if (info->pdata->pat_function == PAT_CONTROL_NONE) {
			info->cal_count = get_tsp_nvm_data(info, PAT_CAL_DATA);
			if (info->cal_count == 0) {
				input_info(true, &client->dev, "%s ts_hw_calibration start \n", __func__);
				if (ts_hw_calibration(info) == false)
					goto fail_init;
				if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2) < 0)
					goto fail_init; /* get fix_tune_version */
				if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2) < 0)
					goto fail_init;
				if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2) < 0)
					goto fail_init;
				set_tsp_nvm_data(info, PAT_CAL_DATA, 0);
				set_tsp_nvm_data(info, PAT_DUMMY_VERSION, (info->cap_info.fw_version << 4) | info->cap_info.fw_minor_version);
				set_tsp_nvm_data(info, PAT_FIX_VERSION, info->cap_info.reg_data_version);
			}
		}

		info->cal_count = get_tsp_nvm_data(info, PAT_CAL_DATA);
		info->tune_fix_ver = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) |
			get_tsp_nvm_data(info, PAT_FIX_VERSION);
		
		if (tsp_nvm_ium_unlock(info) == false) {
			input_err(true, &client->dev, "failed ium unlock\n", __func__);
			goto fail_init;
		}
		input_info(true, &client->dev, "%s info->cal_count:0x%02x"
			" info->tune_fix_ver:0x%04x\n", __func__, info->cal_count, info->tune_fix_ver);
#else
		if (ts_hw_calibration(info) == false)
			goto fail_init;
#endif
			/* disable chip interrupt */
		if (write_reg(client, BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;

			/* get chip firmware version */
		if (read_data(client, BT541_FIRMWARE_VERSION,
				(u8 *)&cap->fw_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_MINOR_FW_VERSION,
				(u8 *)&cap->fw_minor_version, 2) < 0)
			goto fail_init;

		if (read_data(client, BT541_DATA_VERSION_REG,
				(u8 *)&cap->reg_data_version, 2) < 0)
			goto fail_init;		

#endif
}

	if (read_data(client, BT541_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	if (zinitix_bit_test(chip_eeprom_info, 0)) { /* hw calibration bit*/
		if (ts_hw_calibration(info) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (write_reg(client, BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
				(u16)pdata->x_resolution) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_Y_RESOLUTION,
				(u16)pdata->y_resolution) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32)0;
	cap->MinY = (u32)0;
	cap->MaxX = (u32)pdata->x_resolution;
	cap->MaxY = (u32)pdata->y_resolution;

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
				(u16)cap->button_num) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
				(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;
	input_dbg(true, &client->dev, "max supported finger num = %d\n",
			cap->multi_fingers);

	cap->gesture_support = 0;
	input_dbg(true, &client->dev, "set other configuration\n");

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
				TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT541_TOUCH_MODE, info->touch_mode) != I2C_SUCCESS)
		goto fail_init;

#if ZINITIX_I2C_CHECKSUM
	if (read_data(client, ZINITIX_INTERNAL_FLAG_02,
				(u8 *)&reg_val, 2) < 0)
		goto fail_init;
	cap->i2s_checksum = !(!zinitix_bit_test(reg_val, 15));
	input_info(true, &client->dev, "use i2s checksum = %d\n",
			cap->i2s_checksum);
#endif

	bt541_set_optional_mode(info, true);
	/* soft calibration */
	/*	if (write_cmd(client, BT541_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_init;*/

	if (write_reg(client, 0x002E, IUM_SET_TIMEOUT) != I2C_SUCCESS) {
		input_err(true, &client->dev, "%s: failed to set ium timeout\n", __func__);
		//goto fail_init;
	}

	if (write_reg(client, BT541_INT_ENABLE_FLAG,
				cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
					RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			input_err(true, &client->dev, "%s: Failed to set DELAY_RAW_FOR_HOST\n",
					__func__);

			goto fail_init;
		}
	}
#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
				SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_init;

	read_data(client, BT541_PERIODICAL_INTERRUPT_INTERVAL, (u8 *)&reg_val, 2);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "Esd timer register = %d\n", reg_val);
#endif
#endif
	info->version = (u32)((u32)(cap->hw_id & 0xff) << 16) | ((cap->fw_version & 0xf) << 12)
		| ((cap->fw_minor_version & 0xf) << 8) | (cap->reg_data_version & 0xff);
	input_info(true, &client->dev, "successfully initialized\n");
	return true;

fail_init:
	if (cal_mode) {
		input_err(true, &client->dev,"didn't update TSP F/W!! in CAL MODE\n");
		return false;
	}
	if (++retry_cnt <= INIT_RETRY_CNT) {
		bt541_power_control(info, POWER_OFF);
		bt541_power_control(info, POWER_ON_SEQUENCE);

		input_dbg(true, &client->dev, "retry to initiallize(retry cnt = %d)\n",
				retry_cnt);
		goto retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT+1) {
		cap->ic_fw_size = 32*1024;

		input_dbg(true, &client->dev, "retry to initiallize(retry cnt = %d)\n", retry_cnt);

#if TOUCH_FORCE_UPGRADE

		if (!bt541_fw_update_from_kernel(info, true)) {
			input_info(true, &client->dev, "upgrade failed\n");
			return false;
		}

		msleep(100);

		/* hw calibration and make checksum */
		if (ts_hw_calibration(info) == false) {
			input_info(true, &client->dev, "failed to initiallize\n");
			return false;
		}
		goto retry_init;
#endif
	}

	input_err(true, &client->dev, "Failed to initiallize\n");

	return false;
}

static bool mini_init_touch(struct bt541_ts_info *info)
{
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
#if USE_CHECKSUM
	u16 chip_check_sum;

	/*dev_info(&client->dev, "check checksum\n");*/

	if (read_data(client, BT541_CHECKSUM_RESULT,
				(u8 *)&chip_check_sum, 2) < 0)
		goto fail_mini_init;

	if (chip_check_sum != 0x55aa) {
		input_err(true, &client->dev, "Failed to check firmware"
				" checksum(0x%04x)\n", chip_check_sum);

		goto fail_mini_init;
	}
#endif

	if (write_cmd(client, BT541_SWRESET_CMD) != I2C_SUCCESS) {
		input_info(true, &client->dev, "Failed to write reset command\n");

		goto fail_mini_init;
	}

	/* initialize */
	if (write_reg(client, BT541_X_RESOLUTION,
				(u16)(pdata->x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client,BT541_Y_RESOLUTION,
				(u16)(pdata->y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	/*dev_info(&client->dev, "touch max x = %d\r\n", pdata->x_resolution);
	  dev_info(&client->dev, "touch max y = %d\r\n", pdata->y_resolution);*/

	if (write_reg(client, BT541_BUTTON_SUPPORTED_NUM,
				(u16)info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_SUPPORTED_FINGER_NUM,
				(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INITIAL_TOUCH_MODE,
				TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_TOUCH_MODE,
				info->touch_mode) != I2C_SUCCESS)
		goto fail_mini_init;

	bt541_set_optional_mode(info, true);

	if (write_reg(client, 0x002E, IUM_SET_TIMEOUT) != I2C_SUCCESS) {
		input_err(true, &client->dev, "%s: failed to set ium timeout\n", __func__);
		//goto fail_mini_init;
	}

	/* soft calibration */
	if (write_cmd(client, BT541_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT541_INT_ENABLE_FLAG,
				info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);;
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(client, BT541_DELAY_RAW_FOR_HOST,
					RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			input_err(true, &client->dev, "Failed to set BT541_DELAY_RAW_FOR_HOST\n");

			goto fail_mini_init;
		}
	}

#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
				SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_mini_init;

	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "Started esd timer\n");
#endif
#endif

	input_info(true, &client->dev, "Successfully mini initialized\r\n");

	return true;

fail_mini_init:
	input_err(true, &client->dev, "Failed to initialize mini init\n");
#if 0 // if happen firmware crack, need re-define.
	bt541_power_control(info, POWER_OFF);
	bt541_power_control(info, POWER_ON_SEQUENCE);

	if (init_touch(info, fw_true) == false) {
		input_err(true, &client->dev, "Failed to initialize\n");

		return false;
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, info);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "Started esd timer\n");
#endif
#endif

	return true;
#else
	return false;
#endif
}

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
static char location_detect(struct bt541_ts_info *info, int coord, bool flag)
{
	/* flag ? coord = Y : coord = X */
	int x_devide = info->pdata->x_resolution / 3;
	int y_devide = info->pdata->y_resolution / 3;

	if (flag) {
		if (coord < y_devide)
			return 'H';
		else if (coord < y_devide * 2)
			return 'M';
		else
			return 'L';
	} else {
		if (coord < x_devide)
			return '0';
		else if (coord < x_devide * 2)
			return '1';
		else
			return '2';
	}

	return 'E';
}
#endif


static void clear_report_data(struct bt541_ts_info *info)
{
	int i;
	u8 reported = 0;
	u8 sub_status;

	input_dbg(true, &info->client->dev, "%s\n", __func__);
	for (i = 0; i < info->cap_info.button_num; i++) {
		if (info->button[i] == ICON_BUTTON_DOWN) {
			info->button[i] = ICON_BUTTON_UP;
			input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 0);
			reported = true;
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			input_info(true, &info->client->dev, "key %s\n", i ? "Back R" :"Menu R"  );
#else
			input_info(true, &info->client->dev, "key R");
#endif
		}
	}

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
			reported = true;
			if (!m_ts_debug_mode && TSP_NORMAL_EVENT_MSG) {
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
				input_info(true, &info->client->dev,
					"R[%d] loc:%c%c V[%06x] tc:%d mc:%d\n",
					i, location_detect(info, info->reported_touch_info.coord[i].x, 1),
					location_detect(info, info->reported_touch_info.coord[i].y, 0),
					info->version, info->touch_count, info->sec_point_info[i].move_count);
#else
				input_info(true, &info->client->dev,
					"R[%d] (%d, %d) V[%06x] tc:%d mc:%d\n",
					i, info->reported_touch_info.coord[i].x, info->reported_touch_info.coord[i].y,
					info->version, info->touch_count, info->sec_point_info[i].move_count);
#endif
			info->sec_point_info[i].move_count = 0;

			}
		}
		info->reported_touch_info.coord[i].sub_status = 0;
		info->sec_point_info[i].finger_state = 0;
		info->sec_point_info[i].move_count = 0;
		info->touch_count = 0;
	}

	if (reported)
		input_sync(info->input_dev);
}

/*
 * V8 experimental DELTA watchdog.
 * Recovery requires v8_control_lock + work_lock with the device IRQ disabled.
 *
 * v8_checks counts complete three-frame rounds with successful POINT restore.
 * v8_attempts counts every automatic reset attempt, including restore repair.
 * v8_recoveries counts successful automatic resets, including restore repair.
 * v8_errors counts failed rounds/restores and failed recovery attempts.
 * v8_peak is the peak of the most recent valid round; zero after a failed one.
 */
#define BT541_V8_INTERVAL_MS          30000U
#define BT541_V8_QUIET_MS             10000U
#define BT541_V8_CONFIRM_MS            2000U
#define BT541_V8_COOLDOWN_MS         120000U
#define BT541_V8_FRAME_TIMEOUT_MS       250U
#define BT541_V8_ROUND_TIMEOUT_MS      1500U
#define BT541_V8_DELTA_THRESHOLD        500

static bool bt541_v8_can_run(struct bt541_ts_info *info)
{
	return READ_ONCE(info->v8_ready) && READ_ONCE(info->enabled) &&
		!READ_ONCE(info->v8_stopping) &&
		READ_ONCE(info->v8_auto_enabled) &&
		atomic_read(&info->v8_screen_on);
}

static bool bt541_v8_contacts_active(struct bt541_ts_info *info)
{
	int i;

	for (i = 0; i < MAX_SUPPORTED_FINGER_NUM; i++) {
		if (zinitix_bit_test(READ_ONCE(
				info->reported_touch_info.coord[i].sub_status),
				SUB_BIT_EXIST))
			return true;
	}
	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++) {
		if (READ_ONCE(info->button[i]) == ICON_BUTTON_DOWN)
			return true;
	}
	return false;
}

/* Approximate before masking IRQ; authoritative when work_lock is held. */
static bool bt541_v8_idle(struct bt541_ts_info *info)
{
	if (!bt541_v8_can_run(info) ||
			READ_ONCE(info->touch_mode) != TOUCH_POINT_MODE ||
			READ_ONCE(info->work_state) != NOTHING ||
			bt541_v8_contacts_active(info))
		return false;
	if (time_before(jiffies, READ_ONCE(info->v8_last_point) +
			msecs_to_jiffies(BT541_V8_QUIET_MS)))
		return false;
	if (info->v8_suspect &&
			READ_ONCE(info->v8_point_seq) != info->v8_probe_seq)
		return false;
	return true;
}

static bool bt541_v8_in_cooldown(struct bt541_ts_info *info)
{
	return info->v8_cooldown_valid &&
		time_before(jiffies, info->v8_next_recovery);
}

static void bt541_v8_reschedule(struct bt541_ts_info *info,
		unsigned int delay_ms)
{
	if (bt541_v8_can_run(info))
		schedule_delayed_work(&info->v8_work,
				msecs_to_jiffies(delay_ms));
}

static int bt541_v8_sample_guard(struct bt541_ts_info *info,
		unsigned long deadline)
{
	if (!bt541_v8_can_run(info))
		return -ECANCELED;
	if (time_after_eq(jiffies, deadline))
		return -ETIMEDOUT;
	return 0;
}

static unsigned long bt541_v8_frame_deadline(unsigned long round_deadline)
{
	unsigned long deadline = jiffies +
		msecs_to_jiffies(BT541_V8_FRAME_TIMEOUT_MS);

	return time_before(deadline, round_deadline) ? deadline : round_deadline;
}

static int bt541_v8_wait_gpio(struct bt541_ts_info *info, int level,
		unsigned long deadline)
{
	int ret;

	do {
		ret = bt541_v8_sample_guard(info, deadline);
		if (ret)
			return ret;
		if (!!gpio_get_value(info->pdata->gpio_int) == level)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

/* An observed deassertion separates the next ready frame from this one. */
static int bt541_v8_ack_frame(struct bt541_ts_info *info,
		unsigned long deadline)
{
	int ret = bt541_v8_sample_guard(info, deadline);

	if (ret)
		return ret;
	if (write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return -EIO;
	return bt541_v8_wait_gpio(info, 1, deadline);
}

/* No nested locking, no legacy success-returning mode helper. */
static int bt541_v8_set_mode_locked(struct bt541_ts_info *info, u16 mode,
		unsigned long round_deadline)
{
	u16 actual = 0xffff;
	int ret;

	/* POINT cleanup must finish even if sampling timed out or screen went off. */
	if (mode == TOUCH_DELTA_MODE) {
		ret = bt541_v8_sample_guard(info, round_deadline);
		if (ret)
			return ret;
	}

	if (mode == TOUCH_DELTA_MODE &&
			write_reg(info->client, BT541_DELAY_RAW_FOR_HOST,
				RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
		return -EIO;
	if (mode == TOUCH_DELTA_MODE) {
		ret = bt541_v8_sample_guard(info, round_deadline);
		if (ret)
			return ret;
	}
	if (write_reg(info->client, BT541_TOUCH_MODE, mode) != I2C_SUCCESS)
		return -EIO;
	msleep(20);
	if (mode == TOUCH_DELTA_MODE) {
		ret = bt541_v8_sample_guard(info, round_deadline);
		if (ret)
			return ret;
	}
	if (read_data(info->client, BT541_TOUCH_MODE,
			(u8 *)&actual, sizeof(actual)) < 0 || actual != mode)
		return -EIO;
	info->touch_mode = mode;
	if (mode == TOUCH_POINT_MODE) {
		/* A new healthy point may reassert INT immediately: no high wait. */
		return write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD) ==
			I2C_SUCCESS ? 0 : -EIO;
	}
	return bt541_v8_ack_frame(info,
			bt541_v8_frame_deadline(round_deadline));
}

/* All three frames must succeed; each vote uses a complete fresh matrix. */
static int bt541_v8_sample_locked(struct bt541_ts_info *info,
		s16 *samples, unsigned int nodes, bool *strong, u32 *peak)
{
	unsigned int frame, offset, bytes, chunk, node, magnitude;
	unsigned int votes = 0;
	unsigned long deadline;
	unsigned long round_deadline = jiffies +
		msecs_to_jiffies(BT541_V8_ROUND_TIMEOUT_MS);
	u32 frame_peak;
	int ret;

	*strong = false;
	*peak = 0;
	ret = bt541_v8_set_mode_locked(info, TOUCH_DELTA_MODE, round_deadline);
	if (ret)
		return ret;

	/* Discard two newly asserted frames after entering DELTA. */
	for (frame = 0; frame < 2; frame++) {
		deadline = bt541_v8_frame_deadline(round_deadline);
		ret = bt541_v8_wait_gpio(info, 0, deadline);
		if (ret)
			return ret;
		ret = bt541_v8_ack_frame(info, deadline);
		if (ret)
			return ret;
	}

	bytes = nodes * sizeof(*samples);
	for (frame = 0; frame < 3; frame++) {
		deadline = bt541_v8_frame_deadline(round_deadline);
		ret = bt541_v8_wait_gpio(info, 0, deadline);
		if (ret)
			return ret;
		for (offset = 0; offset < bytes; offset += chunk) {
			ret = bt541_v8_sample_guard(info, deadline);
			if (ret)
				return ret;
			chunk = min_t(unsigned int, I2C_BUFFER_SIZE,
					bytes - offset);
			if (read_raw_data(info->client,
					BT541_RAWDATA_REG + offset / I2C_BUFFER_SIZE,
					(u8 *)samples + offset, chunk) < 0)
				return -EIO;
		}
		ret = bt541_v8_ack_frame(info, deadline);
		if (ret)
			return ret;

		frame_peak = 0;
		for (node = 0; node < nodes; node++) {
			magnitude = abs((int)samples[node]);
			if (magnitude > frame_peak)
				frame_peak = magnitude;
		}
		if (frame_peak > *peak)
			*peak = frame_peak;
		if (frame_peak >= BT541_V8_DELTA_THRESHOLD)
			votes++;
	}
	ret = bt541_v8_sample_guard(info, round_deadline);
	if (ret)
		return ret;
	*strong = votes >= 2;
	return 0;
}

static void bt541_v8_attempt_recovery_locked(struct bt541_ts_info *info,
		const char *reason)
{
	bool recovered;

	info->v8_suspect = false;
	info->v8_attempts++;
	info->work_state = ESD_TIMER;
	recovered = bt541_v8_recover_locked(info, reason);
	/* Preserve the cooldown even if mini_init resets other watchdog state. */
	info->v8_suspect = false;
	info->v8_cooldown_valid = true;
	info->v8_next_recovery = jiffies +
		msecs_to_jiffies(BT541_V8_COOLDOWN_MS);
	if (recovered)
		info->v8_recoveries++;
	else
		info->v8_errors++;
}

static void bt541_v8_work_fn(struct work_struct *work)
{
	struct bt541_ts_info *info = container_of(to_delayed_work(work),
			struct bt541_ts_info, v8_work);
	s16 *samples = NULL;
	unsigned int nodes, next_delay = BT541_V8_INTERVAL_MS;
	u32 peak = 0;
	bool strong = false, second_round;
	int ret, restore_ret;

	/* Stop/factory paths may hold this mutex while cancelling this work. */
	if (!mutex_trylock(&info->v8_control_lock)) {
		bt541_v8_reschedule(info, BT541_V8_INTERVAL_MS);
		return;
	}
	if (!bt541_v8_idle(info) || bt541_v8_in_cooldown(info)) {
		info->v8_suspect = false;
		goto out;
	}
	nodes = info->cap_info.total_node_num;
	if (!nodes || nodes > MAX_RAW_DATA_SZ ||
			nodes != (unsigned int)info->cap_info.x_node_num *
				info->cap_info.y_node_num) {
		info->v8_errors++;
		info->v8_suspect = false;
		goto out;
	}
	samples = kcalloc(nodes, sizeof(*samples), GFP_KERNEL);
	if (!samples) {
		info->v8_errors++;
		info->v8_suspect = false;
		goto out;
	}

	WRITE_ONCE(info->v8_busy, true);
	disable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
#endif
	if (down_trylock(&info->work_lock)) {
		info->v8_suspect = false;
		goto out_irq;
	}
	/* A point/transition may have completed before disable_irq returned. */
	if (!bt541_v8_idle(info) || bt541_v8_in_cooldown(info)) {
		info->v8_suspect = false;
		goto out_unlock;
	}
	second_round = info->v8_suspect;
	if (!second_round)
		info->v8_probe_seq = info->v8_point_seq;
	info->work_state = RAW_DATA;
	info->v8_peak = 0;

	ret = bt541_v8_sample_locked(info, samples, nodes, &strong, &peak);
	/* Always attempt POINT restoration, including failed DELTA entry. */
	restore_ret = bt541_v8_set_mode_locked(info, TOUCH_POINT_MODE, 0);
	if (ret || restore_ret) {
		info->v8_errors++;
		info->v8_suspect = false;
		/* One message per failed round; normal retry interval is 30 seconds. */
		input_err(true, &info->client->dev,
				"V8 DELTA check failed: sample=%d restore=%d errors=%u\n",
				ret, restore_ret, info->v8_errors);
		if (restore_ret && READ_ONCE(info->enabled) &&
				!READ_ONCE(info->v8_stopping))
			bt541_v8_attempt_recovery_locked(info,
					"V8 POINT restoration failed");
		goto out_finish;
	}
	info->v8_checks++;
	info->v8_peak = peak;
	if (!strong || !bt541_v8_can_run(info) ||
			info->v8_point_seq != info->v8_probe_seq ||
			bt541_v8_contacts_active(info)) {
		info->v8_suspect = false;
		goto out_finish;
	}
	if (second_round) {
		/* IRQ remains off and work_lock held: no check-to-reset race. */
		bt541_v8_attempt_recovery_locked(info,
				"V8 repeated DELTA without reported contacts");
	} else {
		info->v8_suspect = true;
		next_delay = BT541_V8_CONFIRM_MS;
	}

out_finish:
	info->work_state = NOTHING;
out_unlock:
	up(&info->work_lock);
out_irq:
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	/* Keep traditional ESD alive, including after a failed repair attempt. */
	if (READ_ONCE(info->enabled) && !READ_ONCE(info->v8_stopping))
		esd_timer_start(CHECK_ESD_TIMER, info);
#endif
out:
	kfree(samples);
	/* Reschedule under control_lock; stop must gate new work before cancel. */
	bt541_v8_reschedule(info, next_delay);
	mutex_unlock(&info->v8_control_lock);
}


#ifdef CONFIG_FB
static int bt541_v8_fb_event(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct bt541_ts_info *info = container_of(nb, struct bt541_ts_info, v8_fb);
	struct fb_event *ev = data;
	bool on;
	int blank;
	if (!ev || !ev->info || ev->info->node != 0 || !ev->data)
		return NOTIFY_DONE;
	if (event != FB_EARLY_EVENT_BLANK && event != FB_EVENT_BLANK &&
			event != FB_R_EARLY_EVENT_BLANK)
		return NOTIFY_DONE;
	blank = *(int *)ev->data;
	if (event == FB_EARLY_EVENT_BLANK && blank == FB_BLANK_UNBLANK)
		return NOTIFY_DONE;
	on = blank == FB_BLANK_UNBLANK;
	if (event == FB_R_EARLY_EVENT_BLANK)
		on = !on;
	atomic_set(&info->v8_screen_on, on);
	if (!on)
		cancel_delayed_work_sync(&info->v8_work);
	mutex_lock(&info->v8_control_lock);
	info->v8_suspect = false;
	if (on && info->v8_ready && !info->v8_stopping && info->enabled &&
			info->v8_auto_enabled)
		mod_delayed_work(system_wq, &info->v8_work,
			msecs_to_jiffies(BT541_V8_INTERVAL_MS));
	mutex_unlock(&info->v8_control_lock);
	return NOTIFY_OK;
}
#endif

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static irqreturn_t bt541_touch_work(int irq, void *data)
{
	struct bt541_ts_info* info = (struct bt541_ts_info*)data;
	struct bt541_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
	u8 sub_status;
	u8 prev_sub_status;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
	u8 palm = 0;
	u16 val = 0;
	unsigned long health_flags;
#ifdef CONFIG_SEC_FACTORY
	int ret = 0;
#endif

	atomic_inc(&info->health.irq);
	if (!READ_ONCE(info->v8_ready) || !READ_ONCE(info->enabled) || READ_ONCE(info->v8_stopping) ||
			READ_ONCE(info->v8_busy))
		return IRQ_HANDLED;
	if (gpio_get_value(info->pdata->gpio_int)) {
		atomic_inc(&info->health.invalid_gpio);
		input_err(true, &client->dev, "Invalid interrupt\n");

		return IRQ_HANDLED;
	}

	if (down_trylock(&info->work_lock)) {
		atomic_inc(&info->health.lock_busy);
		input_err(true, &client->dev, "%s: Failed to occupy work lock\n", __func__);
		return IRQ_HANDLED;
	}
#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif
	if (info->work_state != NOTHING) {
		atomic_inc(&info->health.state_busy);
		input_err(true, &client->dev, "%s: Other process occupied\n", __func__);
		usleep_range(DELAY_FOR_SIGNAL_DELAY, DELAY_FOR_SIGNAL_DELAY);

		if (!gpio_get_value(info->pdata->gpio_int)) {
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
			usleep_range(DELAY_FOR_SIGNAL_DELAY, DELAY_FOR_SIGNAL_DELAY);
		}

		goto out;
	}

	info->work_state = NORMAL;
#if ZINITIX_I2C_CHECKSUM
	i = 0;

	if (ts_read_coord(info) == false || info->touch_info.status == 0xffff
			|| info->touch_info.status == 0x1) {
		/* more retry*/
		for (i = 1; i < 50; i++) {	/* about 10ms*/
			if (!(ts_read_coord(info) == false || info->touch_info.status == 0xffff
						|| info->touch_info.status == 0x1))
				break;
		}

	}
	if (i == 50) {
		input_err(true, &client->dev, "Failed to read info coord\n");
		atomic_inc(&info->health.coord_recovery);
		/* LineageOS: verify coordinate-error hard recovery. */
		bt541_power_control(info, POWER_OFF);
		clear_report_data(info);

		if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
			input_err(true, &client->dev,
					"Coordinate recovery failed during power sequence\n");
			goto out;
		}

		if (mini_init_touch(info) == false) {
			input_err(true, &client->dev,
					"Coordinate recovery failed during mini init\n");
			goto out;
		}

		input_info(true, &client->dev,
				"Recovered touchscreen after coordinate read error\n");

		goto out;
	}
#else
	if (ts_read_coord(info) == false || info->touch_info.status == 0xffff
			|| info->touch_info.status == 0x1) { /* maybe desirable reset */
		input_err(true, &client->dev, "Failed to read info coord\n");
		atomic_inc(&info->health.coord_recovery);
		/* LineageOS: verify coordinate-error hard recovery. */
		bt541_power_control(info, POWER_OFF);
		clear_report_data(info);

		if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
			input_err(true, &client->dev,
					"Coordinate recovery failed during power sequence\n");
			goto out;
		}

		if (mini_init_touch(info) == false) {
			input_err(true, &client->dev,
					"Coordinate recovery failed during mini init\n");
			goto out;
		}

		input_info(true, &client->dev,
				"Recovered touchscreen after coordinate read error\n");

		goto out;
	}
#endif
	/* Preserve the accepted packet before slot transforms and release clears. */
	spin_lock_irqsave(&info->health.lock, health_flags);
	info->health.packet = info->touch_info;
	info->health.packet_mode = info->touch_mode;
	info->health.last_packet_jiffies = get_jiffies_64();
	info->health.packet_valid = true;
	spin_unlock_irqrestore(&info->health.lock, health_flags);
	if (info->touch_info.status == 0x0)
		atomic_inc(&info->health.heartbeat);
	else
		atomic_inc(&info->health.nonzero_packet);



	/* invalid : maybe periodical repeated int. */

	if (info->touch_info.status == 0x0)
		goto out;

	if (zinitix_bit_test(info->touch_info.status, BIT_ICON_EVENT)) {
		if (read_data(info->client, BT541_ICON_STATUS_REG,
					(u8 *)(&info->icon_event_reg), 2) < 0) {
			input_err(true, &client->dev, "Failed to read button info\n");
			write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);

			goto out;
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
						(BIT_O_ICON0_DOWN + i))) {
				info->button[i] = ICON_BUTTON_DOWN;
				input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 1);
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
				input_info(true, &client->dev, "Key P\n");
#else
				input_info(true, &client->dev, "Key %s\n", i ? "back P" : "menu P");
#endif
			}
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg,
						(BIT_O_ICON0_UP + i))) {
				info->button[i] = ICON_BUTTON_UP;
				input_report_key(info->input_dev, BUTTON_MAPPING_KEY[i], 0);
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
				input_info(true, &client->dev, "Key R\n");
#else
				input_info(true, &client->dev, "Key %s\n", i ? "back R" : "menu R");
#endif

			}
		}
	}


#ifdef SUPPORTED_PALM_TOUCH
	if (zinitix_bit_test(info->touch_info.status, BIT_PALM)) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		input_info(true, &client->dev, "Palm report\n");
#endif
		palm = 1;
	}

	if (zinitix_bit_test(info->touch_info.status, BIT_PALM_REJECT)) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		input_info(true, &client->dev, "Palm reject\n");
#endif
		palm = 2;
	}
#endif

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			/* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY
					+ info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX
					+ info->cap_info.MinX - x;

			maxX = info->cap_info.MaxX;
			maxY = info->cap_info.MaxY;

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			if (x > maxX || y > maxY) {
				atomic_inc(&info->health.invalid_coord);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
				input_err(true, &client->dev,
						"Invalid coord %d : x=%d, y=%d\n", i, x, y);
#endif
				continue;
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;

			if (w == 0)
				w = 1;

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0) {
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			} else if (palm == 1) {	/*palm report*/
				w = PALM_REPORT_WIDTH;
				/*				info->touch_info.coord[i].minor_width
				= PALM_REPORT_WIDTH;*/
			} else if (palm == 2) {	/* palm reject*/
				/*				x = y = 0;*/
				w = PALM_REJECT_WIDTH;
				/*				info->touch_info.coord[i].minor_width = PALM_REJECT_WIDTH;*/
			}
#endif
#ifdef CONFIG_SEC_FACTORY
			ret = read_data(client, BT541_REAL_WIDTH + i, (u8*)&val, 2);
			if (ret < 0)
					input_info(true, &client->dev, ": Failed to read %d's Real width %s\n", i, __func__);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, (u32)val);
#else
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, (u32)w);
#endif
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR,
					(u32)((palm == 1) ? w-40 : w));
#if (TOUCH_POINT_MODE == 2)
			input_report_abs(info->input_dev,
					ABS_MT_TOUCH_MINOR, (u32)info->touch_info.coord[i].minor_width);
			/*			input_report_abs(info->input_dev,
							ABS_MT_WIDTH_MINOR, (u32)info->touch_info.coord[i].minor_width);*/
#ifdef SUPPORTED_PALM_TOUCH
			/*input_report_abs(info->input_dev, ABS_MT_ANGLE,
			  (palm > 1)?70:info->touch_info.coord[i].angle - 90);*/
			/*dev_info(&client->dev, "finger [%02d] angle = %03d\n", i,
			  info->touch_info.coord[i].angle);*/
			input_report_abs(info->input_dev, ABS_MT_PALM, (palm > 0)?1:0);
#endif
			/*			input_report_abs(info->input_dev, ABS_MT_PALM, 1);*/
#endif

			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			atomic_inc(&info->health.contacts);
			spin_lock_irqsave(&info->health.lock, health_flags);
			info->health.last_contact_jiffies = get_jiffies_64();
			info->health.contact_valid = true;
			spin_unlock_irqrestore(&info->health.lock, health_flags);
			if (info->sec_point_info[i].finger_state > 0)
				info->sec_point_info[i].move_count++;

			if (info->sec_point_info[i].finger_state == 0) {
				info->sec_point_info[i].finger_state = 1;
				info->sec_point_info[i].move_count = 0;
				info->touch_count++;
			}

			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN))
			{

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
				input_info(true, &client->dev, "P[%d] loc(%c, %c) z:%d factoryZ:%d p:%d m:%d, %d tc:%d\n",
						i, location_detect(info, info->reported_touch_info.coord[i].x, 1),
						location_detect(info, info->reported_touch_info.coord[i].y, 0) ,
						w, val, (palm > 0)?1:0, w, ((palm == 1) ? w-40 : w), info->touch_count);
#else
				input_info(true, &client->dev, "P[%d] (%d, %d) z:%d factoryZ:%d p:%d m:%d, %d tc:%d\n",
						i, x, y, w, val, (palm > 0)?1:0, w, ((palm == 1) ? w-40 : w), info->touch_count);
#endif
			}
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP)||
				zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
			input_info(true, &client->dev,
				"R[%d] loc:%c%c V[%06x] tc:%d mc:%d\n",
				i, location_detect(info, info->reported_touch_info.coord[i].x, 1),
				location_detect(info, info->reported_touch_info.coord[i].y, 0),
				info->version, info->touch_count, info->sec_point_info[i].move_count);
#else

			input_info(true, &client->dev,
				"R[%d] (%d, %d) V[%06x] tc:%d mc:%d\n",
				i, info->reported_touch_info.coord[i].x, info->reported_touch_info.coord[i].y,
				info->version, info->touch_count, info->sec_point_info[i].move_count);
#endif

			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
			info->sec_point_info[i].move_count = 0;
			info->sec_point_info[i].finger_state = 0;
			info->touch_count--;
		} else {
			memset(&info->touch_info.coord[i], 0x0, sizeof(struct coord));
		}
	}
	memcpy((char *)&info->reported_touch_info, (char *)&info->touch_info,
			sizeof(struct point_info));

	input_sync(info->input_dev);
	atomic_inc(&info->health.sync);

out:
	if (info->work_state == NORMAL) {
#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);
#endif
		info->work_state = NOTHING;
	}

	up(&info->work_lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bt541_ts_late_resume(struct early_suspend *h)
{
	struct bt541_ts_info *info = misc_info;
	//info = container_of(h, struct bt541_ts_info, early_suspend);
	struct capa_info *cap = &(info->cap_info);

	if (info == NULL)
		return;
	input_info(true, &info->client->dev, "late resume++\r\n");

	down(&info->work_lock);
	if (info->work_state != RESUME
			&& info->work_state != EALRY_SUSPEND) {
		input_err(true, &info->client->dev, "invalid work proceedure (%d)\r\n",
				info->work_state);
		up(&info->work_lock);
		return;
	}
#if 0
	write_cmd(info->client, BT541_WAKEUP_CMD);
	usleep_range(BT541_USEC_PER_MSEC, BT541_USEC_PER_MSEC);
#else
	bt541_power_control(info, POWER_ON_SEQUENCE);
#endif
	info->work_state = RESUME;
	if (mini_init_touch(info) == false)
		goto fail_late_resume;

	if (read_data(info->client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2))
		;
	if (read_data(info->client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2))
		;
	if (read_data(info->client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2))
		;
	if (read_data(info->client, BT541_HW_ID, (u8 *)&cap->hw_id, 2))
		;
	if (read_data(info->client, BT541_VENDOR_ID, (u8 *)&cap->vendor_id, 2))
		;
	enable_irq(info->irq);
	info->work_state = NOTHING;
	up(&info->work_lock);
	input_err(true, &info->client->dev, "late resume--\n");
	return;
fail_late_resume:
	input_err(true, &info->client->dev, "failed to late resume\n");
	enable_irq(info->irq);
	info->work_state = NOTHING;
	up(&info->work_lock);
	return;
}

static void bt541_ts_early_suspend(struct early_suspend *h)
{
	struct bt541_ts_info *info = misc_info;
	/*info = container_of(h, struct bt541_ts_info, early_suspend);*/

	if (info == NULL)
		return;

	input_info(true, &info->client->dev, "early suspend++\n");

	disable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
#endif

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		input_err(true, &info->client->dev, "invalid work proceedure (%d)\r\n",
				info->work_state);
		up(&info->work_lock);
		enable_irq(info->irq);
		return;
	}
	info->work_state = EALRY_SUSPEND;

	input_err(true, &info->client->dev, "clear all reported points\r\n");
	clear_report_data(info);

#if ESD_TIMER_INTERVAL
	/*write_reg(info->client, BT541_PERIODICAL_INTERRUPT_INTERVAL, 0);*/
	esd_timer_stop(info);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &info->client->dev, "Stopped esd timer\n");
#endif
#endif

#if 0
	write_reg(info->client, BT541_INT_ENABLE_FLAG, 0x0);

	usleep_range(100, 100);
	if (write_cmd(info->client, BT541_SLEEP_CMD) != I2C_SUCCESS) {
		input_err(true, &misc_info->client->dev, "failed to enter into sleep mode\n");
		up(&info->work_lock);
		return;
	}
#else
	bt541_power_control(info, POWER_OFF);
#endif
	input_info(true, &info->client->dev, "early suspend--\n");
	up(&info->work_lock);
	return;
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int bt541_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	struct input_dev *input = info->input_dev;
	int ret = 0;

	/* Serialize with input enabled/open/close.  A system resume must not
	 * override a touchscreen disabled by the display/proximity policy.
	 */
	mutex_lock(&input->mutex);
	if (!input->disabled && input->users)
		ret = bt541_ts_open(input);
	mutex_unlock(&input->mutex);

	return ret;
}

static int bt541_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt541_ts_info *info = i2c_get_clientdata(client);
	struct input_dev *input = info->input_dev;

	mutex_lock(&input->mutex);
	bt541_ts_close(input);
	mutex_unlock(&input->mutex);

	return 0;
}
#endif

static int bt541_ts_open_locked(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);

	if (!READ_ONCE(info->v8_ready))
		return 0;
	if (info->v8_stopping)
		return -ENODEV;
	if(info->enabled == true) {
		input_err(true, &client->dev, "%s already open\n", __func__);
		return 0;
	}

#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "resume++\n");
#endif
	WRITE_ONCE(info->v8_busy, true);
	down(&info->work_lock);
	if (info->work_state != SUSPEND) {
		input_err(true, &client->dev, "%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);
		WRITE_ONCE(info->v8_busy, false);
		return -EBUSY;
	}
	bt541_power_control(info, POWER_ON_SEQUENCE);
	bt541_pinctrl_configure(info, 1);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	info->work_state = RESUME;
#else
	info->work_state = NOTHING;
	if (mini_init_touch(info) == false) {
		input_err(true, &client->dev, "Failed to resume, trying hard recovery\n");

		/* LineageOS: retry one full power-cycle before giving up on resume. */
		bt541_power_control(info, POWER_OFF);
		if (bt541_power_control(info, POWER_ON_SEQUENCE) == false ||
				mini_init_touch(info) == false) {
			input_err(true, &client->dev,
					"Failed to recover touchscreen after resume\n");
#if ESD_TIMER_INTERVAL
			/*
			 * mini_init_touch() only starts the one-shot ESD timer on
			 * success. Keep the recovery path alive after a failed resume.
			 */
			esd_timer_start(CHECK_ESD_TIMER, info);
#endif
		}
	}

	if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&cap->fw_version, 2))
		;
	if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&cap->fw_minor_version, 2))
		;
	if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&cap->reg_data_version, 2))
		;
	if (read_data(client, BT541_HW_ID, (u8 *)&cap->hw_id, 2))
		;
	if (read_data(client, BT541_VENDOR_ID, (u8 *)&cap->vendor_id, 2))
		;

	if (!gpio_get_value(info->pdata->gpio_int))
	{
		write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);
		usleep_range(50, 50);
		write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);
		usleep_range(50, 50);
		write_cmd(info->client, BT541_CLEAR_INT_STATUS_CMD);
	}
	info->work_state = NOTHING;
#endif

#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "resume--\n");
#endif
	info->v8_suspect = false;
	info->v8_last_point = jiffies;
	info->v8_point_seq++;
	info->enabled = true;
	up(&info->work_lock);
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
	esd_timer_start(CHECK_ESD_TIMER, info);
	if (info->v8_auto_enabled && atomic_read(&info->v8_screen_on))
		mod_delayed_work(system_wq, &info->v8_work,
			msecs_to_jiffies(BT541_V8_INTERVAL_MS));
	return 0;
}

static int bt541_ts_open(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	int ret;
	mutex_lock(&info->v8_control_lock);
	ret = bt541_ts_open_locked(dev);
	mutex_unlock(&info->v8_control_lock);
	return ret;
}


static void bt541_ts_close_locked(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;

	if (!READ_ONCE(info->v8_ready))
		return;
	if (info->v8_stopping)
		return;
	if(info->enabled == false) {
		input_err(true, &client->dev, "%s already suspended\n", __func__);
		return;
	}
	 
	input_info(true, &client->dev, "%s\n", __func__);
	WRITE_ONCE(info->enabled, false);
	WRITE_ONCE(info->v8_busy, true);
	info->v8_suspect = false;
	cancel_delayed_work_sync(&info->v8_work);


       

#ifndef CONFIG_HAS_EARLYSUSPEND
	disable_irq(info->irq);
#endif

	/*
	 * LineageOS: quiesce ESD before draining its workqueue.
	 * Mark inactive first, stop the one-shot timer, synchronously drain ESD
	 * work, then stop once more in case a running recovery rearmed it.
	 */
	info->enabled = false;
#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
#endif

	down(&info->work_lock);
	if (info->work_state != NOTHING
			&& info->work_state != SUSPEND) {
		input_err(true, &client->dev,"%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);

		/* LineageOS: restore ESD watchdog if suspend is aborted. */
		info->enabled = true;
		WRITE_ONCE(info->v8_busy, false);
#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);
#endif
		up(&info->work_lock);
#ifndef CONFIG_HAS_EARLYSUSPEND
		enable_irq(info->irq);
#endif
		return;
	}

#ifndef CONFIG_HAS_EARLYSUSPEND
	clear_report_data(info);

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "Stopped esd timer\n");
#endif
#endif
#endif
	write_cmd(info->client, BT541_SLEEP_CMD);
	bt541_power_control(info, POWER_OFF);
	bt541_pinctrl_configure(info, 0);
	info->work_state = SUSPEND;
	info->touch_mode = TOUCH_POINT_MODE;
	info->update = 0;
	info->v8_raw_owner = NULL;

#if defined(TSP_VERBOSE_DEBUG)
	input_err(true, &info->client->dev, "suspend--\n");
#endif
	info->enabled = false;
	up(&info->work_lock);
	WRITE_ONCE(info->v8_busy, false);

	return;
}

static void bt541_ts_close(struct input_dev *dev)
{
	struct bt541_ts_info *info = input_get_drvdata(dev);
	mutex_lock(&info->v8_control_lock);
	bt541_ts_close_locked(dev);
	mutex_unlock(&info->v8_control_lock);
}










static int ts_upgrade_sequence(const u8 *firmware_data, u32 firmware_size)
{
	disable_irq(misc_info->irq);
	down(&misc_info->work_lock);
	misc_info->work_state = UPGRADE;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	input_info(true, &misc_info->client->dev, "clear all reported points\r\n");
	clear_report_data(misc_info);

	input_info(true, &misc_info->client->dev, "start upgrade firmware\n");
	if (ts_upgrade_firmware(misc_info,
				firmware_data, firmware_size,
				fw_force) == false) {
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;
	}

	if (init_touch(misc_info, fw_false) == false) {
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &misc_info->client->dev, "Started esd timer\n");
#endif
#endif

	enable_irq(misc_info->irq);
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return 0;
}

#ifdef CONFIG_SEC_FACTORY_TEST

static void fw_update(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	int ret = 0;
	const u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	char fw_path[MAX_FW_PATH+1];
	char result[16] = {0};
	const struct firmware *fw;

	sec_cmd_set_default_result(sec);

	request_firmware(&fw, info->pdata->fw_name, &info->client->dev);
	if (!fw && (sec->cmd_param[0] == BUILT_IN)) {
		input_err(true, &info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		goto update_fail;
	}

	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
		goto update_fail;
	}
	info->cal_count = get_tsp_nvm_data(info, PAT_CAL_DATA);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
		goto update_fail;
	}
	input_info(true, &info->client->dev, "%s pat_flag cal_count:0x%02x\n", __func__, info->cal_count);
	if (info->cal_count >= PAT_MAGIC_NUMBER)
		info->pat_flag = true;

	switch (sec->cmd_param[0]) {
	case BUILT_IN:
		ret = ts_upgrade_sequence((u8 *)fw->data, fw->size);
		if (ret < 0) {
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			release_firmware(fw);
			return;
		}
		break;

	case UMS:
		old_fs = get_fs();
		set_fs(get_ds());

		snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", TSP_FW_FILENAME);
		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			input_err(true, &info->client->dev,
					"file %s open error:%lu\n", fw_path, (unsigned long)fp);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			input_err(true, &info->client->dev, "invalid fw size!!\n");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto err_open;
		}

		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (!buff) {
			input_err(true, &info->client->dev, "failed to alloc buffer for fw\n");
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto err_alloc;
		}

		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);
		input_info(true, &info->client->dev, "ums fw is loaded!!\n");

		ret = ts_upgrade_sequence((u8 *)buff, fsize);
		if (ret < 0) {
			kfree(buff);
			sec->cmd_state = SEC_CMD_STATUS_FAIL;
			goto update_fail;
		}
		break;

	default:
		input_err(true, &info->client->dev, "invalid fw file type!!\n");
		goto update_fail;
	}

	sec->cmd_state = 2;
	snprintf(result, sizeof(result) , "%s", "OK");
	sec_cmd_set_cmd_result(sec, result,
			strnlen(result, sizeof(result)));
	release_firmware(fw);
	kfree(buff);

	return;


	if (fp != NULL) {
err_fw_size:
		kfree(buff);
err_alloc:
		filp_close(fp, NULL);
err_open:
		set_fs(old_fs);
	}
update_fail:
	snprintf(result, sizeof(result) , "%s", "NG");
	sec_cmd_set_cmd_result(sec, result, strnlen(result, sizeof(result)));
	release_firmware(fw);
}

static void get_fw_ver_bin(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	const struct firmware *fw;
	char fw_path[64];
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;

	sec_cmd_set_default_result(sec);
	
	snprintf(fw_path, 64, "%s", info->pdata->fw_name);
	request_firmware(&fw, fw_path, &info->client->dev);

	if (!fw) {
		input_err(true, &info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		snprintf(buf, sizeof(buf), "%s", "NG");
		goto EXIT;
	} 
	/* modify m_firmware_data */
	fw_version = (u16)(fw->data[0x34] | (fw->data[0x35] << 8));
	fw_minor_version = (u16)(fw->data[0x38] | (fw->data[0x39] << 8));
	reg_version = (u16)(fw->data[0x3C] | (fw->data[0x3D] << 8));
	hw_id =  (u16)(fw->data[0x30] | (fw->data[0x31]<<8));
	/*vendor_id = ntohs(*(u16 *)&m_firmware_data[0x57e2]);*/
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf ) << 12)
		| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);

	/*length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length,
				"%06X", version);*/
	release_firmware(fw);

	snprintf(buf, sizeof(buf), "ZI%06X", version);

	sec->cmd_state = SEC_CMD_STATUS_OK;
EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	u16 fw_version, fw_minor_version, reg_version, hw_id;
	u32 version;

	sec_cmd_set_default_result(sec);

	fw_version = info->cap_info.fw_version;
	fw_minor_version = info->cap_info.fw_minor_version;
	reg_version = info->cap_info.reg_data_version;
	hw_id = info->cap_info.hw_id;
	/*vendor_id = ntohs(info->cap_info.vendor_id);*/
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
		| ((fw_minor_version & 0xf) << 8) | (reg_version & 0xff);

	/*length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length,
				"%06X", version);*/

	snprintf(buf, sizeof(buf), "ZI%06X", version);

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

static void get_threshold(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf),
			"%d", info->cap_info.threshold);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}



/*
 * Use the kernel conversion functions, not this driver's local BT541_USEC_PER_MSEC macro.
 * The 64-bit timestamp avoids the 32-bit jiffies wrap.  Ages saturate at
 * 2147483647 ms; -1 means no packet/contact has been observed since probe.
 */
static s32 bt541_health_age_ms(u64 now, u64 last, bool valid)
{
	u64 elapsed;

	if (!valid)
		return -1;
	elapsed = now - last;
	if (elapsed >= msecs_to_jiffies(2147483647U))
		return 2147483647;
	return jiffies_to_msecs((unsigned long)elapsed);
}

/*
 * Read-only factory pages: no I2C, IRQ masking, mode changes or recovery.
 * Page 0 consists of independent atomic counters (not one atomic snapshot).
 * Page 1 copies the last accepted packet before the IRQ modifies its slots.
 * A separate cache lock never contends with the IRQ's work_lock.  The live
 * enabled/state/mode/contact-count fields are approximate individual reads,
 * not one simultaneous snapshot with the cached packet.
 */
static void get_touch_health(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info =
		container_of(sec, struct bt541_ts_info, sec);
	struct point_info packet;
	char buff[384];
	u64 now, last_packet, last_contact;
	unsigned long flags;
	u16 touch_mode, packet_mode;
	u8 work_state;
	int touch_count;
	s32 packet_age, contact_age;
	bool enabled, packet_valid, contact_valid;
	int len;

	sec_cmd_set_default_result(sec);
	if (sec->cmd_param[0] == 0) {
		/* contacts counts accepted active-slot reports, including moves. */
		len = scnprintf(buff, sizeof(buff),
			"v:1 irq:%u gpio:%u lock:%u busy:%u err:%u "
			"hb:%u pkt:%u contacts:%u sync:%u badxy:%u",
			(unsigned int)atomic_read(&info->health.irq),
			(unsigned int)atomic_read(&info->health.invalid_gpio),
			(unsigned int)atomic_read(&info->health.lock_busy),
			(unsigned int)atomic_read(&info->health.state_busy),
			(unsigned int)atomic_read(&info->health.coord_recovery),
			(unsigned int)atomic_read(&info->health.heartbeat),
			(unsigned int)atomic_read(&info->health.nonzero_packet),
			(unsigned int)atomic_read(&info->health.contacts),
			(unsigned int)atomic_read(&info->health.sync),
			(unsigned int)atomic_read(&info->health.invalid_coord));
	} else if (sec->cmd_param[0] == 1) {
		spin_lock_irqsave(&info->health.lock, flags);
		packet = info->health.packet;
		packet_valid = info->health.packet_valid;
		packet_mode = info->health.packet_mode;
		last_packet = info->health.last_packet_jiffies;
		last_contact = info->health.last_contact_jiffies;
		contact_valid = info->health.contact_valid;
		spin_unlock_irqrestore(&info->health.lock, flags);

		enabled = ACCESS_ONCE(info->enabled);
		work_state = ACCESS_ONCE(info->work_state);
		touch_mode = ACCESS_ONCE(info->touch_mode);
		touch_count = ACCESS_ONCE(info->touch_count);
		now = get_jiffies_64();
		packet_age = bt541_health_age_ms(now, last_packet, packet_valid);
		contact_age = bt541_health_age_ms(now, last_contact, contact_valid);

		len = scnprintf(buff, sizeof(buff),
			"v:1 en:%u ws:%u mode:%u tc:%d pv:%u pm:%u st:%04x "
#if (TOUCH_POINT_MODE == 1)
			"fc:-1 ts:-1 ef:%04x "
#else
			"fc:%u ts:%u "
#endif
			"s0:%02x s1:%02x pa:%d ca:%d",
			enabled ? 1 : 0, work_state, touch_mode, touch_count,
			packet_valid ? 1 : 0, packet_mode, packet.status,
#if (TOUCH_POINT_MODE == 1)
			packet.event_flag,
#else
			packet.finger_cnt, packet.time_stamp,
#endif
			packet.coord[0].sub_status, packet.coord[1].sub_status,
			packet_age, contact_age);
	} else {
		len = scnprintf(buff, sizeof(buff), "NG_PARAM");
		goto fail;
	}
	sec_cmd_set_cmd_result(sec, buff, len);
	sec->cmd_state = SEC_CMD_STATUS_OK;
	return;

fail:
	sec_cmd_set_cmd_result(sec, buff, len);
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
}

static void get_watchdog_status(void *device_data)
{
	struct sec_cmd_data *sec = device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buff[256];

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff),
		"v8 en:%u screen:%d suspect:%u checks:%u err:%u peak:%u attempts:%u ok:%u",
		READ_ONCE(info->v8_auto_enabled), atomic_read(&info->v8_screen_on),
		READ_ONCE(info->v8_suspect), READ_ONCE(info->v8_checks),
		READ_ONCE(info->v8_errors), READ_ONCE(info->v8_peak),
		READ_ONCE(info->v8_attempts), READ_ONCE(info->v8_recoveries));
	sec_cmd_set_cmd_result(sec, buff, strlen(buff));
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

static void set_auto_recover(void *device_data)
{
	struct sec_cmd_data *sec = device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);

	sec_cmd_set_default_result(sec);
	if (sec->cmd_param[0] != 0 && sec->cmd_param[0] != 1) {
		sec_cmd_set_cmd_result(sec, "NG_PARAM", 8);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}
	WRITE_ONCE(info->v8_auto_enabled, !!sec->cmd_param[0]);
	info->v8_suspect = false;
	/* Worker uses trylock, so draining it under control_lock is safe. */
	cancel_delayed_work_sync(&info->v8_work);
	if (info->v8_auto_enabled && info->enabled &&
			atomic_read(&info->v8_screen_on) && !info->v8_stopping)
		mod_delayed_work(system_wq, &info->v8_work,
				msecs_to_jiffies(BT541_V8_INTERVAL_MS));
	sec_cmd_set_cmd_result(sec, "OK", 2);
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

static void force_recover(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info =
		container_of(sec, struct bt541_ts_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };
	bool ret;

	sec_cmd_set_default_result(sec);


	ret = bt541_force_recovery(info, "factory command");

	snprintf(buff, sizeof(buff), "%s", ret ? "OK" : "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = ret ? SEC_CMD_STATUS_OK : SEC_CMD_STATUS_FAIL;

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

static void module_off_master(void *device_data)
{
	return;
}

static void module_on_master(void *device_data)
{
	return;
}

static void module_off_slave(void *device_data)
{
	return;
}

static void module_on_slave(void *device_data)
{
	return;
}

#define BT541_VENDOR_NAME "ZINITIX"

static void get_chip_vendor(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%s", BT541_VENDOR_NAME);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

#define BT541_CHIP_NAME "BT541C"

static void get_chip_name(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%s", BT541_CHIP_NAME);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

static void get_x_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%u", info->cap_info.x_node_num);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

static void get_y_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "%u", info->cap_info.y_node_num);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

static void not_support_cmd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

	sprintf(buf, "%s", "NA");
	sec_cmd_set_cmd_result(sec, buf,
			strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;

	input_info(true, &info->client->dev, "%s: \"%s(%d)\"\n", __func__, buf,
			(int)strnlen(buf, sizeof(buf)));

	return;
}

/*
## Mis Cal result ##
FD : spec out
F3,F4 : i2c failed
F2 : power off state
F1 : not support mis cal concept
F0 : initial value in function
00 : pass
*/

static void run_tsp_rawdata_read(void *device_data, u16 rawdata_mode, s16* buff)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif

	ts_set_touchmode(rawdata_mode);
	get_raw_data(info, (u8 *)buff, 2);
	ts_set_touchmode(TOUCH_POINT_MODE);

	input_info(true, &info->client->dev, "touch rawdata %d start\n", rawdata_mode);

	for (i = 0; i < x_num; i++) {
		printk("[%02d] :", i);
		for (j = 0; j < y_num; j++) {
			printk("%d\t", buff[(i * y_num) + j]);
		}
		printk("\n");
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}

static void run_mis_cal_read(void * device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, x, y, node_num;
	char mis_cal_data = 0xF0;
	int ret = 0;
	s16 raw_data_buff[TSP_CMD_NODE_NUM];
	
#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);

	if (info->pdata->mis_cal_check == 0) {
		input_info(true, &info->client->dev, "%s: [ERROR] not support, %d\n", __func__);
		mis_cal_data = 0xF1;
		goto NG;
	}

	if (info->work_state == SUSPEND) {
		input_info(true, &info->client->dev, "%s: [ERROR] Touch is stopped\n",__func__);
		mis_cal_data = 0xF2;
		goto NG;
	}

	ts_set_touchmode(TOUCH_REF_ABNORMAL_TEST_MODE);
	ret = get_raw_data(info, (u8 *)raw_data->reference_data_abnormal, 2);
	if (!ret) {
		input_info(true, &info->client->dev, "%s:[ERROR] i2c fail!\n", __func__);
		mis_cal_data = 0xF3;
		goto NG;
	}
	ts_set_touchmode(TOUCH_POINT_MODE);

	input_info(true, &info->client->dev, "%s start\n", __func__);

	ret = 1;
	for (i = 0; i < x_num; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;
			printk("%d ", raw_data->reference_data_abnormal[offset]);

			if (ret && raw_data->reference_data_abnormal[offset] > reference_data_abnormal_max[i][j]) {
				val =  raw_data->reference_data_abnormal[offset];
				x = i;
				y = j;
				node_num = offset;
				mis_cal_data = 0xFD;
				ret = 0;
			}
		}
		printk("\n");
	}
	
	if(!ret)
		goto NG;

	mis_cal_data = 0x00;
	snprintf(buf, sizeof(buf), "%d,%d,%d,%d", mis_cal_data, raw_data->reference_data_abnormal[0],
			raw_data->reference_data_abnormal[1], raw_data->reference_data_abnormal[2]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &info->client->dev, "%s: %s\n", __func__, buf);
	enable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
NG:
	snprintf(buf, sizeof(buf), "%d,%d,%d,%d", mis_cal_data, 0, 0, 0);
	if (mis_cal_data == 0xFD) {
		run_tsp_rawdata_read(device_data, TOUCH_CNDDATA_MODE, raw_data_buff);
		run_tsp_rawdata_read(device_data, TOUCH_REFERENCE_MODE, raw_data_buff);
	}
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
 	input_info(true, &info->client->dev, "%s: %s\n", __func__, buf);
	enable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}

static void get_mis_cal(void * device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int offset, x_node, y_node;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);
	
	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(buf, sizeof(buf), "%s", "abnormal");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		enable_irq(info->irq);
		return;
	}
	
	offset = (x_node * y_num) + y_node;
	
	snprintf(buf, sizeof(buf), "%d", raw_data->reference_data_abnormal[offset]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &info->client->dev, "%s: %s\n", __func__, buf);
	enable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}

static void run_dnd_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val = 0, x = 0, y = 0, node_num = 0;
	bool result = true;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);

	ts_set_touchmode(TOUCH_DND_MODE);
	get_raw_data(info, (u8 *)raw_data->dnd_data, 2);
	ts_set_touchmode(TOUCH_POINT_MODE);

	input_info(true, &info->client->dev, "DND start\n");

	for (i = 0; i < x_num; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;
			printk("%d ", raw_data->dnd_data[offset]);

			if ((raw_data->dnd_data[offset] > dnd_max[i][j]) ||
			  (raw_data->dnd_data[offset] &&
			  (raw_data->dnd_data[offset] < dnd_min[i][j]))) {
				val =  raw_data->dnd_data[offset];
				x = i;
				y = j;
				node_num = offset;
				result = false;
			}
		}
		printk("\n");
	}

	if (result) {
		input_info(true, &info->client->dev, "DND Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "DND Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, dnd_min[x][y], dnd_max[x][y], val);
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	enable_irq(info->irq);

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}

static void run_dnd_v_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val, x = 0, y = 0, node_num = 0, fail_val = 0;
	bool result = true;

	sec_cmd_set_default_result(sec);

	memset(raw_data->vgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &info->client->dev, "DND V Gap start\n");

	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->dnd_data[offset];
			next_val = raw_data->dnd_data[offset + y_num];
			if (!next_val) {
				raw_data->vgap_data[offset] = next_val;
				continue;
	}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
	else
				val = 100 - ((next_val * 100) / cur_val);

			printk("%d ", val);
			cur_val = (s16)(dnd_v_gap[i][j]);

			if (val > cur_val) {
				fail_val = val;
				x = i;
				y = j;
				node_num = offset;
				result = false;
				}
			raw_data->vgap_data[offset] = val;
			}
		printk("\n");
			}

	if (result) {
		input_info(true, &info->client->dev, "DND V Gap Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "DND V Gap Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, 0, dnd_v_gap[x][y], fail_val);
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	return;
}

static void run_dnd_h_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val, x = 0, y = 0, node_num = 0, fail_val = 0;
	bool result = true;

	sec_cmd_set_default_result(sec);

	memset(raw_data->hgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &info->client->dev, "DND H Gap start\n");

	for (i = 0; i < x_num ; i++) {
		for (j = 0; j < y_num-1; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->dnd_data[offset];
			if (!cur_val) {
				raw_data->hgap_data[offset] = cur_val;
				continue;
			}

			next_val = raw_data->dnd_data[offset + 1];
			if (!next_val) {
				raw_data->hgap_data[offset] = next_val;
				for (++j; j < y_num - 1; j++) {
					offset = (i * y_num) + j;

					next_val = raw_data->dnd_data[offset];
					if (!next_val) {
						raw_data->hgap_data[offset]
							= next_val;
						continue;
			}

					break;
		}
	}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
	else
				val = 100 - ((next_val * 100) / cur_val);

			printk("%d ", val);
			cur_val = (s16)(dnd_h_gap[i][j]);

			if (val > cur_val) {
				fail_val = val;
				x = i;
				y = j;
				node_num = offset;
			result = false;
			}
			raw_data->hgap_data[offset] = val;
		}
		printk("\n");
		}
	
	if (result) {
		input_info(true, &info->client->dev, "DND H Gap Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "DND H Gap Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, 0, dnd_h_gap[x][y], fail_val);
	}

	sec_cmd_set_cmd_result(sec, buf,strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	return;
}

static void get_dnd_h_gap(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= x_num || y_node < 0 || y_node >= y_num - 1) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = (x_node * y_num) + y_node;

	sprintf(buf, "%d", raw_data->hgap_data[node_num]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_dnd_v_gap(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= x_num - 1 || y_node < 0 || y_node >= y_num) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = (x_node * y_num) + y_node;

	sprintf(buf, "%d", raw_data->vgap_data[node_num]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
}

static void run_hfdnd_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset;
	bool result = true;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);

	for (i = 0; i < TSP_CMD_NODE_NUM; i++)
		raw_data->hfdnd_data_sum[i] = 0;

	ts_set_touchmode2(TOUCH_DND_MODE);
	
	get_raw_data(info, (u8 *)raw_data->hfdnd_data, 2);
	write_cmd(misc_info->client, BT541_CLEAR_INT_STATUS_CMD);
	ts_set_touchmode2(TOUCH_POINT_MODE);
	
	input_info(true, &info->client->dev, "HF DND start\n");

	for (i = 0; i < x_num; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;
			printk("%d ", raw_data->hfdnd_data[offset]);
		}
		printk("\n");
	}

	if (result) {
		input_info(true, &info->client->dev, "HF DND Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	enable_irq(info->irq);

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}


static void run_hfdnd_v_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val, x = 0, y = 0, node_num = 0, fail_val = 0;
	bool result = true;

	sec_cmd_set_default_result(sec);

	memset(raw_data->vgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &info->client->dev, "HF DND V Gap start\n");

	for (i = 0; i < x_num - 1; i++) {
		for (j = 0; j < y_num; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->hfdnd_data[offset];
			next_val = raw_data->hfdnd_data[offset + y_num];
			if (!next_val) {
				raw_data->vgap_data[offset] = next_val;
				continue;
			}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
	else
				val = 100 - ((next_val * 100) / cur_val);

			printk("%d ", val);
			cur_val = (s16)(hfdnd_v_gap[i][j]);

			if (val > cur_val) {
				fail_val = val;
				x = i;
				y = j;
				node_num = offset;
				result = false;
				}
			raw_data->vgap_data[offset] = val;
			}
		printk("\n");
			}

	if (result) {
		input_info(true, &info->client->dev, "HF DND V Gap Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "HF DND V Gap Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, 0, hfdnd_v_gap[x][y], fail_val);
			}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	return;
}

static void run_hfdnd_h_gap_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, next_val, x = 0, y = 0, node_num = 0, fail_val = 0;
	bool result = true;

	sec_cmd_set_default_result(sec);

	memset(raw_data->hgap_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &info->client->dev, "HF DND H Gap start\n");

	for (i = 0; i < x_num ; i++) {
		for (j = 0; j < y_num-1; j++) {
			offset = (i * y_num) + j;

			cur_val = raw_data->hfdnd_data[offset];
			if (!cur_val) {
				raw_data->hgap_data[offset] = cur_val;
				continue;
			}

			next_val = raw_data->hfdnd_data[offset + 1];
			if (!next_val) {
				raw_data->hgap_data[offset] = next_val;
				for (++j; j < y_num - 1; j++) {
					offset = (i * y_num) + j;

					next_val = raw_data->hfdnd_data[offset];
					if (!next_val) {
						raw_data->hgap_data[offset]
							= next_val;
						continue;
			}

					break;
		}
	}

			if (next_val > cur_val)
				val = 100 - ((cur_val * 100) / next_val);
	   else
				val = 100 - ((next_val * 100) / cur_val);

			printk("%d ", val);
			cur_val = (s16)(hfdnd_h_gap[i][j]);

			if (val > cur_val) {
				fail_val = val;
				x = i;
				y = j;
				node_num = offset;
			result = false;
			}
			raw_data->hgap_data[offset] = val;
		}
		printk("\n");
		}
	
	if (result) {
		input_info(true, &info->client->dev, "HF DND H Gap Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "HF DND H Gap Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, 0, hfdnd_h_gap[x][y], fail_val);
	}

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	return;
}

static void get_hfdnd_h_gap(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= x_num || y_node < 0 || y_node >= y_num - 1) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = (x_node * y_num) + y_node;

	sprintf(buf, "%d", raw_data->hgap_data[node_num]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
}

static void get_hfdnd_v_gap(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_node, y_node;
	int node_num;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= x_num - 1 || y_node < 0 || y_node >= y_num) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = (x_node * y_num) + y_node;

	sprintf(buf, "%d", raw_data->vgap_data[node_num]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
}

static void run_gapjitter_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int i, j, offset, val, cur_val, x = 0, y = 0, node_num = 0, fail_val = 0;
	bool result = true;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);

	memset(raw_data->gapjitter_data, 0x00, TSP_CMD_NODE_NUM);

	input_info(true, &info->client->dev, "Gap Jitter start\n");

	ts_set_touchmode16(TOUCH_H_GAP_JITTER_MODE);
	get_raw_data(info, (u8 *)raw_data->gapjitter_data, 2);
	write_cmd(misc_info->client, BT541_CLEAR_INT_STATUS_CMD);
	ts_set_touchmode16(TOUCH_POINT_MODE);

	for (i = 0; i < x_num ; i++) {
		for (j = 0; j < y_num-1; j++) {
			offset = (i * y_num) + j;

			val = raw_data->gapjitter_data[offset];

			printk("%d ", val);
			cur_val = (s16)(hfdnd_h_jitter_gap[i][j]);

			if (val > cur_val) {
				fail_val = val;
				x = i;
				y = j;
				node_num = offset;
				result = false;
			}
		}
		printk("\n");
		}

	if (result) {
		input_info(true, &info->client->dev, "Gap Jitter Pass\n");
		snprintf(buf, sizeof(buf), "OK\n");
	} else {
		input_err(true, &info->client->dev, "Gap Jitter Fail\n");
		snprintf(buf, sizeof(buf), "Fail,%d,%d,%d,%d\n",
				node_num, 0, hfdnd_h_jitter_gap[x][y], fail_val);
	}

	sec_cmd_set_cmd_result(sec, buf,strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	enable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}

static void get_gapjitter(void * device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	int x_num = info->cap_info.x_node_num, y_num = info->cap_info.y_node_num;
	int offset, x_node, y_node;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	sec_cmd_set_default_result(sec);
	
	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(buf, sizeof(buf), "%s", "abnormal");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}
	
	offset = (x_node * y_num) + y_node;
	snprintf(buf, sizeof(buf), "%d", raw_data->gapjitter_data[offset]);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &info->client->dev, "%s: %s\n", __func__, buf);
#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}



static void run_reference_read(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	struct tsp_raw_data *raw_data = info->raw_data;
	int min = 0xFFFF, max = 0x0000;
	s32 i, j, touchkey_node = 2;
	int buffer_offset;
	char buf[SEC_CMD_STR_LEN] = { 0 };

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	disable_irq(info->irq);
	sec_cmd_set_default_result(sec);

	ts_set_touchmode(TOUCH_REFERENCE_MODE);
	get_raw_data_size(info, (u8 *)raw_data->reference_data, 2,
		2 * (info->cap_info.total_node_num * 2 + info->cap_info.y_node_num + info->cap_info.x_node_num));
	ts_set_touchmode(TOUCH_POINT_MODE);

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		printk("%s: ref_data[%2d] : ", __func__, i);
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			printk(" %5d", raw_data->reference_data[i * info->cap_info.y_node_num + j]);

			if (i == (info->cap_info.x_node_num - 1)) {
				if ((j == touchkey_node)||(j == (info->cap_info.y_node_num - 1) - touchkey_node)) {
					if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] < min &&
						raw_data->reference_data[(i * info->cap_info.y_node_num) + j] >= 0)
						min = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];

					if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] > max)
						max = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];
				}
			} else {
				if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] < min &&
					raw_data->reference_data[(i * info->cap_info.y_node_num) + j] >= 0)
					min = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];

				if (raw_data->reference_data[(i * info->cap_info.y_node_num) + j] > max)
					max = raw_data->reference_data[(i * info->cap_info.y_node_num) + j];
			}
		}
		printk("\n");
	}

	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec_cmd_set_cmd_result(sec, buf,strnlen(buf, sizeof(buf)));

	buffer_offset = info->cap_info.total_node_num;
	for (i = 0; i < info->cap_info.x_node_num; i++) {
		printk("%s: ref_data(mode1)[%2d] : ", __func__, i);
		for (j = 0; j < info->cap_info.y_node_num; j++) {
			printk(" %5d", raw_data->reference_data[buffer_offset + i * info->cap_info.y_node_num + j]);
		}
		printk("\n");
	}

	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
	enable_irq(info->irq);

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif
	return;
}


static void get_reference(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	//struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
/*	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;*/

	sec_cmd_set_default_result(sec);
/*
	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
		finfo->cmd_state = FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->ref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
	strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
			(int)strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
*/
	return;
}

static void get_dnd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(buf, sizeof(buf), "%s", "abnormal");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->dnd_data[node_num];
	snprintf(buf, sizeof(buf), "%u", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
		(int)strnlen(buf, sizeof(buf)));

	return;
}

static void get_hfdnd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(buf, sizeof(buf), "%s", "abnormal");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->hfdnd_data[node_num];
	snprintf(buf, sizeof(buf), "%u", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
		(int)strnlen(buf, sizeof(buf)));

	return;
}

static void run_delta_read(void *device_data)
{
	struct sec_cmd_data *sec = device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN];
	s16 *data = info->raw_data->delta_data;
	int i, min = 32767, max = -32768;
	bool acquired = false, restored, entered;

	sec_cmd_set_default_result(sec);
	WRITE_ONCE(info->v8_busy, true);
	disable_irq(info->irq);
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
	entered = ts_set_touchmode(TOUCH_DELTA_MODE);
	if (entered)
		acquired = get_raw_data(info, (u8 *)data, 10);
	restored = ts_set_touchmode(TOUCH_POINT_MODE);
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
	if (!restored)
		bt541_force_recovery(info, "factory DELTA restore failed");
	esd_timer_start(CHECK_ESD_TIMER, info);
	if (!entered || !acquired || !restored) {
		sec_cmd_set_cmd_result(sec, "NG_IO", 5);
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		return;
	}
	for (i = 0; i < info->cap_info.total_node_num; i++) {
		if (data[i] < min)
			min = data[i];
		if (data[i] > max)
			max = data[i];
	}
	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec_cmd_set_cmd_result(sec, buf, strlen(buf));
	sec->cmd_state = SEC_CMD_STATUS_OK;
}

static void get_delta(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	sec_cmd_set_default_result(sec);

	x_node = sec->cmd_param[0];
	y_node = sec->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(buf, sizeof(buf), "%s", "abnormal");
		sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(buf, sizeof(buf), "%u", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__, buf,
		(int)strnlen(buf, sizeof(buf)));

	return;
}

static void hfdnd_spec_adjust(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	int test;

	sec_cmd_set_default_result(sec);

	test = sec->cmd_param[0];

	if (test) {
		dnd_h_gap = assy_dnd_h_gap;
		dnd_v_gap = assy_dnd_v_gap;
		hfdnd_h_gap = assy_hfdnd_h_gap;
		hfdnd_v_gap = assy_hfdnd_v_gap;
		dnd_max = assy_dnd_max;
		dnd_min = assy_dnd_min;
		hfdnd_max = assy_hfdnd_max;
		hfdnd_min = assy_hfdnd_min;		
		hfdnd_h_jitter_gap = assy_hfdnd_h_jitter_gap;
	} else {
		dnd_h_gap = tsp_dnd_h_gap;
		dnd_v_gap = tsp_dnd_v_gap;
		hfdnd_h_gap = tsp_hfdnd_h_gap;
		hfdnd_v_gap = tsp_hfdnd_v_gap;
		dnd_max = tsp_dnd_max;
		dnd_min = tsp_dnd_min;
		hfdnd_max = tsp_hfdnd_max;
		hfdnd_min = tsp_hfdnd_min;		
		hfdnd_h_jitter_gap = tsp_hfdnd_h_jitter_gap;
	}

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
	return;
}

static void clear_reference_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	struct i2c_client *client = info->client;
	char buf[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
	write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL, 0);
#endif

	write_reg(client, BT541_EEPROM_INFO_REG, 0xffff);

	write_reg(client, 0xc003, 0x0001);
	write_reg(client, 0xc104, 0x0001);
	usleep_range(100, 100);
	if (write_cmd(client, BT541_SAVE_STATUS_CMD) != I2C_SUCCESS)
		return;

	msleep(500);
	write_reg(client, 0xc003, 0x0000);
	write_reg(client, 0xc104, 0x0000);
	usleep_range(100, 100);

#if ESD_TIMER_INTERVAL
	write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL);
	esd_timer_start(CHECK_ESD_TIMER, info);
#endif
	input_info(true, &info->client->dev, "%s: TSP clear calibration bit\n", __func__);

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
			buf, (int)strnlen(buf, sizeof(buf)));
	return;
}

static void run_force_calibration(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	char buff = 0;
	struct i2c_client *client = info->client;
	int i;
	bool ret;

	disable_irq(info->irq);
	
	sec_cmd_set_default_result(sec);

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
	write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL, 0);
#endif

	write_reg(client, 0x01CA, 1300);
	ret = ts_hw_calibration(info);
	if (ret == true) {
		input_info(true, &client->dev, "%s: TSP calibration Pass\n", __func__);
	} else {
		input_info(true, &client->dev, "%s: TSP calibration Fail\n", __func__);
		goto out;
	}

#ifdef PAT_CONTROL
/* adb and factory lcia */
	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &client->dev, "failed ium lock\n", __func__);
		goto out;
	}
	buff = get_tsp_nvm_data(info, PAT_CAL_DATA);
	if (buff >= PAT_MAGIC_NUMBER)
		buff = 1;
	else if (buff >= PAT_MAX_LCIA)
		buff = PAT_MAX_LCIA;
	else
		buff = buff + 1;
	
	info->cal_count = buff;

	if (read_data(client, BT541_FIRMWARE_VERSION, (u8 *)&info->cap_info.fw_version, 2) < 0)
		goto out;
	if (read_data(client, BT541_DATA_VERSION_REG, (u8 *)&info->cap_info.reg_data_version, 2) < 0)
		goto out;
	if (read_data(client, BT541_MINOR_FW_VERSION, (u8 *)&info->cap_info.fw_minor_version, 2) < 0)
		goto out;
	set_tsp_nvm_data(info, PAT_CAL_DATA, info->cal_count);
	set_tsp_nvm_data(info, PAT_DUMMY_VERSION, (info->cap_info.fw_version << 4) | info->cap_info.fw_minor_version);
	set_tsp_nvm_data(info, PAT_FIX_VERSION, info->cap_info.reg_data_version);
	info->tune_fix_ver = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) |
				get_tsp_nvm_data(info, PAT_FIX_VERSION);
	input_info(true, &client->dev, " %s info->cal_count:0x%02x info->tune_fix_ver: 0x%04x\n", __func__,
		info->cal_count, info->tune_fix_ver);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &client->dev, "failed ium unlock\n", __func__);
		goto out;
	}
#endif

out:
	write_reg(client, 0x01CA, 0);
	write_reg(client, 0x0149, 0);

	for (i = 0; i < 5; i++) {
		write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
		usleep_range(10, 10);
	}
#if ESD_TIMER_INTERVAL
	write_reg(client, BT541_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL);
	esd_timer_start(CHECK_ESD_TIMER, info);
#endif
	if (ret) {
		snprintf(buf, sizeof(buf), "%s", "OK");
		sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_OK;
	} else {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
	}
	input_info(true, &info->client->dev, "%s: %s(%d)\n", __func__,
				buf, (int)strnlen(buf, sizeof(buf)));

	enable_irq(info->irq);
	
	return;
}

#ifdef PAT_CONTROL
static void get_pat_information(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buff[SEC_CMD_STR_LEN] = { 0 };

	sec_cmd_set_default_result(sec);

    /* fixed tune version will be saved at excute autotune */
	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
		goto FAIL;
	}
	info->cal_count = get_tsp_nvm_data(info, PAT_CAL_DATA);
	info->tune_fix_ver = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) |
		get_tsp_nvm_data(info, PAT_FIX_VERSION);
	if (tsp_nvm_ium_unlock(info) == false){
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
	}
	input_info(true, &info->client->dev, "%s info->cal_count: 0x%02x"
		" info->tune_fix_ver:0x%04x\n", __func__, info->cal_count, info->tune_fix_ver);
	snprintf(buff, sizeof(buff), "P%02XT%04X",info->cal_count, info->tune_fix_ver);

	sec_cmd_set_cmd_result(sec, buff, (int)strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
FAIL:
	snprintf(buff, sizeof(buff), "NG");
	sec_cmd_set_cmd_result(sec, buff, (int)strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
	return;
}

static void get_calibration_nv_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	char buff = 0;

	sec_cmd_set_default_result(sec);
	
	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
	}
	buff = get_tsp_nvm_data(info, PAT_CAL_DATA);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
	}

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: fix_ver_data 0x%02x  %s(%d)\n", __func__,
			buff, buf, (int)strnlen(buf, sizeof(buf)));
	return;        
}

static void get_tune_fix_ver_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	char buff = 0;

	sec_cmd_set_default_result(sec);
	
	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
	}
	buff = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) + get_tsp_nvm_data(info, PAT_FIX_VERSION);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
	}

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: fix_ver_data 0x%04x  %s(%d)\n", __func__,
			buff, buf, (int)strnlen(buf, sizeof(buf)));
	return;        
}

static void set_calibration_nv_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	int val = sec->cmd_param[0];
	char buff = 0;

	sec_cmd_set_default_result(sec);

	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
	}
	set_tsp_nvm_data(info, PAT_CAL_DATA, val);
	buff = get_tsp_nvm_data(info, PAT_CAL_DATA);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
	}

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: nv_data 0x%02x %s(%d) \n", __func__,
			buff, buf, (int)strnlen(buf, sizeof(buf)));
	return;        
}

static void set_tune_fix_ver_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	int val = sec->cmd_param[0];
	char buff = 0;

	sec_cmd_set_default_result(sec);

	if (tsp_nvm_ium_lock(info) == false) {
		input_err(true, &info->client->dev, "failed ium lock\n", __func__);
	}
	//set_tsp_nvm_data(info, PAT_DUMMY_VERSION, 0x00);
	set_tsp_nvm_data(info, PAT_FIX_VERSION, val);
	buff = (get_tsp_nvm_data(info, PAT_DUMMY_VERSION) << 8) + get_tsp_nvm_data(info, PAT_FIX_VERSION);
	if (tsp_nvm_ium_unlock(info) == false) {
		input_err(true, &info->client->dev, "failed ium unlock\n", __func__);
	}
 
	snprintf(buf, sizeof(buf), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s: fix_ver_data 0x%04x %s(%d)\n", __func__,
			buff, buf, (int)strnlen(buf, sizeof(buf)));
	return;        
}
#endif

static void dead_zone_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec);
	char buf[SEC_CMD_STR_LEN] = { 0 };
	int val = sec->cmd_param[0];

	sec_cmd_set_default_result(sec);

	if(val) //disable
		zinitix_bit_clr(m_optional_mode, 3);
	else //enable
		zinitix_bit_set(m_optional_mode, 3);

	snprintf(buf, sizeof(buf), "dead_zone %s", val ? "disable" : "enable");
	sec_cmd_set_cmd_result(sec, buf, (int)strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s(), %s\n", __func__, buf);

	return;
}

static ssize_t show_enabled(struct device *dev, struct device_attribute
			*devattr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	int val = 0;

	if (info->work_state == EALRY_SUSPEND || info->work_state == SUSPEND)
		val = 0;
	else
		val = 1;
	input_info(true, &client->dev, " enabled is %d", info->work_state);

	return snprintf(buf, sizeof(val), "%d\n", val);
}
static DEVICE_ATTR(enabled, S_IRUGO, show_enabled, NULL);

static struct attribute *sec_touch_pretest_attributes[] = {
	&dev_attr_enabled.attr,
	NULL,
};

static struct attribute_group sec_touch_pretest_attr_group = {
	.attrs	= sec_touch_pretest_attributes,
};


#ifdef SUPPORTED_TOUCH_KEY
static ssize_t show_touchkey_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct capa_info *cap = &(info->cap_info);

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	input_info(true, &info->client->dev, "%s: key threshold = %d\n", __func__,
			cap->key_threshold);

	return snprintf(buf, 41, "%d", cap->key_threshold);
#else
	input_info(true, &info->client->dev, "%s: key threshold = %d %d %d %d\n", __func__,
			cap->dummy_threshold, cap->key_threshold, cap->key_threshold, cap->dummy_threshold);

	return snprintf(buf, 41, "%d %d %d %d", cap->dummy_threshold,
			cap->key_threshold,  cap->key_threshold,
			cap->dummy_threshold);
#endif
}

static ssize_t show_touchkey_sensitivity(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bt541_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	u16 val = 0;
	int ret = 0;
	int i;

#ifdef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 1;
	else {
		input_err(true, &info->client->dev, "%s: Invalid attribute\n", __func__);

		goto err_out;
	}

#else
	if (!strcmp(attr->attr.name, "touchkey_dummy_btn1"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_recent"))
		i = 1;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 2;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn4"))
		i = 3;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn5"))
		i = 4;
	else if (!strcmp(attr->attr.name, "touchkey_dummy_btn6"))
		i = 5;
	else {
		input_err(true, &info->client->dev, "%s: Invalid attribute\n", __func__);

		goto err_out;
	}
#endif
	down(&info->work_lock);
	ret = read_data(client, BT541_BTN_WIDTH + i, (u8 *)&val, 2);
	up(&info->work_lock);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: Failed to read %d's key sensitivity\n",
				__func__, i);

		goto err_out;
	}

	input_info(true, &info->client->dev, "%s: %d's key sensitivity = %d\n",
			__func__, i, val);

	return snprintf(buf, 6, "%d", val);

err_out:
	return sprintf(buf, "NG");
}

static ssize_t show_back_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t show_menu_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(touchkey_threshold, S_IRUGO, show_touchkey_threshold, NULL);
static DEVICE_ATTR(touchkey_recent, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, show_touchkey_sensitivity, NULL);
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
static DEVICE_ATTR(touchkey_dummy_btn1, S_IRUGO,
		show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn3, S_IRUGO,
		show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn4, S_IRUGO,
		show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_dummy_btn6, S_IRUGO,
		show_touchkey_sensitivity, NULL);
#endif
static DEVICE_ATTR(touchkey_raw_back, S_IRUGO, show_back_key_raw_data, NULL);
static DEVICE_ATTR(touchkey_raw_menu, S_IRUGO, show_menu_key_raw_data, NULL);

static struct attribute *touchkey_attributes[] = {
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_recent.attr,
	&dev_attr_touchkey_raw_menu.attr,
	&dev_attr_touchkey_raw_back.attr,
#ifndef NOT_SUPPORTED_TOUCH_DUMMY_KEY
	&dev_attr_touchkey_dummy_btn1.attr,
	&dev_attr_touchkey_dummy_btn3.attr,
	&dev_attr_touchkey_dummy_btn4.attr,
	&dev_attr_touchkey_dummy_btn6.attr,
#endif
	NULL,
};
static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};
#endif

/* Whole-command exclusion: factory, misc, PM and V8 share one mutex. */
#define BT541_V8_FACTORY_WRAPPER(fn) \
static void v8_##fn(void *device_data) \
{ \
	struct sec_cmd_data *sec = device_data; \
	struct bt541_ts_info *info = container_of(sec, struct bt541_ts_info, sec); \
	mutex_lock(&info->v8_control_lock); \
	info->v8_suspect = false; \
	if (!info->v8_ready || info->v8_stopping || (!info->enabled && fn != set_auto_recover)) { \
		sec_cmd_set_default_result(sec); \
		sec_cmd_set_cmd_result(sec, "NG_INACTIVE", 11); \
		sec->cmd_state = SEC_CMD_STATUS_FAIL; \
	} else if (fn == set_auto_recover) { \
		fn(device_data); \
	} else { \
		WRITE_ONCE(info->v8_busy, true); \
		disable_irq(info->irq); \
		esd_timer_stop(info); \
		cancel_work_sync(&info->tmr_work); \
		esd_timer_stop(info); \
		fn(device_data); \
		WRITE_ONCE(info->v8_busy, false); \
		enable_irq(info->irq); \
		esd_timer_start(CHECK_ESD_TIMER, info); \
	} \
	mutex_unlock(&info->v8_control_lock); \
}
BT541_V8_FACTORY_WRAPPER(fw_update)
BT541_V8_FACTORY_WRAPPER(get_fw_ver_bin)
BT541_V8_FACTORY_WRAPPER(get_fw_ver_ic)
BT541_V8_FACTORY_WRAPPER(get_threshold)
BT541_V8_FACTORY_WRAPPER(module_off_master)
BT541_V8_FACTORY_WRAPPER(module_on_master)
BT541_V8_FACTORY_WRAPPER(module_off_slave)
BT541_V8_FACTORY_WRAPPER(module_on_slave)
BT541_V8_FACTORY_WRAPPER(get_chip_vendor)
BT541_V8_FACTORY_WRAPPER(get_chip_name)
BT541_V8_FACTORY_WRAPPER(get_x_num)
BT541_V8_FACTORY_WRAPPER(get_y_num)
BT541_V8_FACTORY_WRAPPER(not_support_cmd)
BT541_V8_FACTORY_WRAPPER(run_reference_read)
BT541_V8_FACTORY_WRAPPER(get_reference)
BT541_V8_FACTORY_WRAPPER(run_delta_read)
BT541_V8_FACTORY_WRAPPER(get_delta)
BT541_V8_FACTORY_WRAPPER(run_dnd_read)
BT541_V8_FACTORY_WRAPPER(get_dnd)
BT541_V8_FACTORY_WRAPPER(run_dnd_v_gap_read)
BT541_V8_FACTORY_WRAPPER(get_dnd_v_gap)
BT541_V8_FACTORY_WRAPPER(run_dnd_h_gap_read)
BT541_V8_FACTORY_WRAPPER(get_dnd_h_gap)
BT541_V8_FACTORY_WRAPPER(run_hfdnd_read)
BT541_V8_FACTORY_WRAPPER(get_hfdnd)
BT541_V8_FACTORY_WRAPPER(run_hfdnd_v_gap_read)
BT541_V8_FACTORY_WRAPPER(get_hfdnd_v_gap)
BT541_V8_FACTORY_WRAPPER(run_hfdnd_h_gap_read)
BT541_V8_FACTORY_WRAPPER(get_hfdnd_h_gap)
BT541_V8_FACTORY_WRAPPER(run_gapjitter_read)
BT541_V8_FACTORY_WRAPPER(get_gapjitter)
BT541_V8_FACTORY_WRAPPER(hfdnd_spec_adjust)
BT541_V8_FACTORY_WRAPPER(clear_reference_data)
BT541_V8_FACTORY_WRAPPER(run_force_calibration)
BT541_V8_FACTORY_WRAPPER(get_pat_information)
BT541_V8_FACTORY_WRAPPER(get_calibration_nv_data)
BT541_V8_FACTORY_WRAPPER(get_tune_fix_ver_data)
BT541_V8_FACTORY_WRAPPER(set_calibration_nv_data)
BT541_V8_FACTORY_WRAPPER(set_tune_fix_ver_data)
BT541_V8_FACTORY_WRAPPER(dead_zone_enable)
BT541_V8_FACTORY_WRAPPER(run_mis_cal_read)
BT541_V8_FACTORY_WRAPPER(get_mis_cal)
BT541_V8_FACTORY_WRAPPER(force_recover)
BT541_V8_FACTORY_WRAPPER(set_auto_recover)
#undef BT541_V8_FACTORY_WRAPPER

static void bt541_v8_exit_factory(struct bt541_ts_info *info)
{
        struct device *dev;

        if (info->v8_sec_initialized) {
                dev = info->sec.fac_dev;
                /* sec_cmd_exit logs fac_dev after device_destroy while
                 * draining the command FIFO; keep that device alive.
                 */
                get_device(dev);
                sysfs_remove_link(&dev->kobj, "input");
                /* CONFIG_SEC_SYSFS allocates devt dynamically. The class
                 * selector SEC_CLASS_DEVT_TSP is not necessarily its devt.
                 */
                sec_cmd_exit(&info->sec, dev->devt);
                info->v8_sec_initialized = false;
                info->sec.fac_dev = NULL;
                put_device(dev);
        }

#ifdef SUPPORTED_TOUCH_KEY
        dev = info->v8_factory_tk_dev;
        if (dev) {
                if (info->v8_factory_tk_group_created) {
                        sysfs_remove_group(&dev->kobj, &touchkey_attr_group);
                        info->v8_factory_tk_group_created = false;
                }
                dev_set_drvdata(dev, NULL);
                sec_device_destroy(dev->devt);
                info->v8_factory_tk_dev = NULL;
        }
#endif

        /* The original driver creates this device but never installs
         * sec_touch_pretest_attr_group on it. There is no group to remove.
         */
        dev = info->v8_sec_pretest_dev;
        if (dev) {
                dev_set_drvdata(dev, NULL);
                sec_device_destroy(dev->devt);
                info->v8_sec_pretest_dev = NULL;
        }
}

static bool bt541_v8_begin_terminal_stop(struct bt541_ts_info *info)
{
        bool was_enabled;

        mutex_lock(&info->v8_control_lock);
        was_enabled = info->enabled;
        WRITE_ONCE(info->v8_stopping, true);
        WRITE_ONCE(info->enabled, false);
        WRITE_ONCE(info->v8_busy, true);
        mutex_unlock(&info->v8_control_lock);
        return was_enabled;
}

static void bt541_v8_unregister_notifications(struct bt541_ts_info *info)
{
#ifdef CONFIG_FB
        if (info->v8_fb_registered) {
                fb_unregister_client(&info->v8_fb);
                info->v8_fb_registered = false;
        }
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&info->early_suspend);
#endif
#if defined(CONFIG_PM_RUNTIME) && !defined(CONFIG_HAS_EARLYSUSPEND)
        pm_runtime_disable(&info->client->dev);
#endif
}

static void bt541_v8_quiesce_locked(struct bt541_ts_info *info,
                bool was_enabled)
{
        if (was_enabled)
                disable_irq(info->irq);
        else
                synchronize_irq(info->irq);

        cancel_delayed_work_sync(&info->v8_work);
#if ESD_TIMER_INTERVAL
        esd_timer_stop(info);
        cancel_work_sync(&info->tmr_work);
        /* A worker already running at the first stop may have rearmed it. */
        esd_timer_stop(info);
#endif

        down(&info->work_lock);
        info->work_state = REMOVE;
        clear_report_data(info);
#if ESD_TIMER_INTERVAL
        if (was_enabled)
                write_reg(info->client,
                                BT541_PERIODICAL_INTERRUPT_INTERVAL, 0);
#endif
        bt541_power_control(info, POWER_OFF);
        bt541_pinctrl_configure(info, 0);
        up(&info->work_lock);
}

static int init_sec_factory(struct bt541_ts_info *info)
{
        struct device *dev;
        int ret;

        /* Finish all callback-visible initialization before publishing cmd. */
        info->raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
        if (!info->raw_data)
                return -ENOMEM;

        dnd_h_gap = assy_dnd_h_gap;
        dnd_v_gap = assy_dnd_v_gap;
        hfdnd_h_gap = assy_hfdnd_h_gap;
        hfdnd_v_gap = assy_hfdnd_v_gap;
        hfdnd_h_jitter_gap = assy_hfdnd_h_jitter_gap;
        dnd_max = assy_dnd_max;
        dnd_min = assy_dnd_min;
        hfdnd_max = assy_hfdnd_max;
        hfdnd_min = assy_hfdnd_min;
        reference_data_abnormal_max = assy_reference_data_abnormal_max;

        ret = sec_cmd_init(&info->sec, bt541_commands,
                        ARRAY_SIZE(bt541_commands), SEC_CLASS_DEVT_TSP);
        if (ret)
                goto err_free_raw;
        info->v8_sec_initialized = true;

        ret = sysfs_create_link(&info->sec.fac_dev->kobj,
                        &info->input_dev->dev.kobj, "input");
        if (ret)
                goto err_interfaces;

#ifdef SUPPORTED_TOUCH_KEY
        dev = sec_device_create(info, "sec_touchkey");
        if (IS_ERR(dev)) {
                ret = PTR_ERR(dev);
                goto err_interfaces;
        }
        info->v8_factory_tk_dev = dev;
#endif

        dev = sec_device_create(info, "input");
        if (IS_ERR(dev)) {
                ret = PTR_ERR(dev);
                goto err_interfaces;
        }
        info->v8_sec_pretest_dev = dev;

#ifdef SUPPORTED_TOUCH_KEY
        ret = sysfs_create_group(&info->v8_factory_tk_dev->kobj,
                        &touchkey_attr_group);
        if (ret)
                goto err_interfaces;
        info->v8_factory_tk_group_created = true;
#endif
        return 0;

err_interfaces:
        bt541_v8_exit_factory(info);
err_free_raw:
        kfree(info->raw_data);
        info->raw_data = NULL;
        return ret;
}
#endif

static void bt541_v8_control_begin(struct bt541_ts_info *info)
{
	WRITE_ONCE(info->v8_busy, true);
	disable_irq(info->irq);
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
}

static void bt541_v8_control_end(struct bt541_ts_info *info)
{
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
	esd_timer_start(CHECK_ESD_TIMER, info);
}

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	struct bt541_ts_info *info;
	mutex_lock(&bt541_device_lock);
	info = misc_info;
	if (info) {
		mutex_lock(&info->v8_control_lock);
		if (info->v8_raw_owner == filp) {
			info->v8_raw_owner = NULL;
			info->v8_suspect = false;
			if (info->v8_ready && info->enabled && !info->v8_stopping) {
				bt541_v8_control_begin(info);
				if (!ts_set_touchmode(TOUCH_POINT_MODE))
					bt541_force_recovery(info, "RAW owner closed");
				bt541_v8_control_end(info);
			}
		}
		mutex_unlock(&info->v8_control_lock);
	}
	mutex_unlock(&bt541_device_lock);
	return 0;
}

static long ts_misc_fops_ioctl_locked(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct raw_ioctl raw_ioctl;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 version;
	u16 mode;
	const struct firmware *fw;
	struct reg_ioctl reg_ioctl;
	u16 val;
	int nval = 0;

	if (misc_info == NULL) {
		pr_err("%s misc device NULL?\n", SECLOG);
		return -1;
	}

	switch (cmd) {

	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}
		if (nval)
			input_err(true, &misc_info->client->dev, "[zinitix_touch] on debug mode (%d)\n",
					nval);
		else
			input_err(true, &misc_info->client->dev, "[zinitix_touch] off debug mode (%d)\n",
					nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		input_info(true, &misc_info->client->dev, "[zinitix_touch]: firmware size = %d, %d\r\n",
			(int)sz, misc_info->cap_info.ic_fw_size);
		if (misc_info->cap_info.ic_fw_size != sz) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: firmware size error\r\n");
			return -1;
		}
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		request_firmware(&fw, misc_info->pdata->fw_name, &misc_info->client->dev);

		if (!fw) {
			input_err(true, &misc_info->client->dev, "%s [ERROR] request_firmware\n", __func__);
			return -1;
		}
		
		if (copy_from_user(fw->data,
					argp, misc_info->cap_info.ic_fw_size)) {
			release_firmware(fw);
			return -1;
		}

		version = (u16) (fw->data[52] | (fw->data[53]<<8));

		input_err(true, &misc_info->client->dev, "[zinitix_touch]: firmware version = %x\r\n", version);

		if (copy_to_user(argp, &version, sizeof(version))) {
			release_firmware(fw);
			return -1;
		}
		
		release_firmware(fw);

		break;

	case TOUCH_IOCTL_START_UPGRADE:
		request_firmware(&fw, misc_info->pdata->fw_name, &misc_info->client->dev);
		if (!fw) {
			input_err(true, &misc_info->client->dev, "%s [ERROR] request_firmware\n", __func__);
			return -1;
		}
		ret = ts_upgrade_sequence((u8 *)fw->data, fw->size);

		release_firmware(fw);

		return ret;
		
	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		disable_irq(misc_info->irq);
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: other process occupied.. (%d)\r\n",
					misc_info->work_state);
			up(&misc_info->work_lock);
			enable_irq(misc_info->irq);
			return -1;
		}
		misc_info->work_state = HW_CALIBRAION;
		msleep(100);

		/* h/w calibration */
		if (ts_hw_calibration(misc_info) == true)
			ret = 0;

		mode = misc_info->touch_mode;
		if (write_reg(misc_info->client,
					BT541_TOUCH_MODE, mode) != I2C_SUCCESS) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: failed to set touch mode %d.\n",
					mode);
			goto fail_hw_cal;
		}

		if (write_cmd(misc_info->client,
					BT541_SWRESET_CMD) != I2C_SUCCESS)
			goto fail_hw_cal;

		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;
fail_hw_cal:
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (copy_from_user(&nval, argp, 4)) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user\r\n");
			misc_info->work_state = NOTHING;
			return -1;
		}
		return ts_set_touchmode((u16)nval) ? 0 : -EIO;

	case TOUCH_IOCTL_GET_REG:
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]:other process occupied.. (%d)\n",
					misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (read_data(misc_info->client,
					(u16)reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		nval = (int)val;

		if (copy_to_user((void *)(unsigned long)reg_ioctl.val, (u8 *)&nval, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_to_user\n");
			return -1;
		}

		input_info(true, &misc_info->client->dev, "read : reg addr = 0x%x, val = 0x%x\n",
				reg_ioctl.addr, nval);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: other process occupied.. (%d)\n",
					misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
					argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user(1)\n");
			return -1;
		}

		if (copy_from_user(&nval, (void *)(unsigned long)reg_ioctl.val, sizeof(nval))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user(2)\n");
			return -1;
		}

		if (write_reg(misc_info->client,
					(u16)reg_ioctl.addr, (u16)nval) != I2C_SUCCESS)
			ret = -1;

		input_info(true, &misc_info->client->dev, "write : reg addr = 0x%x, val = 0x%x\r\n",
				reg_ioctl.addr, val);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: other process occupied.. (%d)\r\n",
					misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (write_reg(misc_info->client,
					BT541_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			ret = -1;
		input_info(true, &misc_info->client->dev, "write : reg addr = 0x%x, val = 0x0\r\n",
				BT541_INT_ENABLE_FLAG);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			input_err(true, &misc_info->client->dev, "[zinitix_touch]: other process occupied.." \
					"(%d)\r\n", misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = SET_MODE;
		ret = 0;
		write_reg(misc_info->client, 0xc003, 0x0001);
		write_reg(misc_info->client, 0xc104, 0x0001);
		if (write_cmd(misc_info->client,
					BT541_SAVE_STATUS_CMD) != I2C_SUCCESS)
			ret =  -1;

		msleep(1000);	/* for fusing eeprom */
		write_reg(misc_info->client, 0xc003, 0x0000);
		write_reg(misc_info->client, 0xc104, 0x0000);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_info->touch_mode == TOUCH_POINT_MODE)
			return -1;

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
					argp, sizeof(struct raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			input_err(true, &misc_info->client->dev, "[zinitix_touch] error : copy_from_user\r\n");
			return -1;
		}

		misc_info->update = 0;

		u8Data = (u8 *)&misc_info->cur_data[0];
		if (raw_ioctl.sz > MAX_TRAW_DATA_SZ*2)
			raw_ioctl.sz = MAX_TRAW_DATA_SZ*2;
		if (copy_to_user((void *)(unsigned long)raw_ioctl.buf, (u8 *)u8Data,
					raw_ioctl.sz)) {
			up(&misc_info->raw_data_lock);
			return -1;
		}

		up(&misc_info->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct bt541_ts_info *info;
	long ret = -ENODEV;
	mutex_lock(&bt541_device_lock);
	info = misc_info;
	if (info) {
		mutex_lock(&info->v8_control_lock);
		if (info->v8_ready && !info->v8_stopping && info->enabled) {
			info->v8_suspect = false;
			bt541_v8_control_begin(info);
			ret = ts_misc_fops_ioctl_locked(filp, cmd, arg);
			if (!ret && cmd == TOUCH_IOCTL_SET_RAW_DATA_MODE)
				info->v8_raw_owner = info->touch_mode == TOUCH_POINT_MODE ? NULL : filp;
			bt541_v8_control_end(info);
		}
		mutex_unlock(&info->v8_control_lock);
	}
	mutex_unlock(&bt541_device_lock);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id tsp_dt_ids[] = {
	{ .compatible = "Zinitix,bt541_ts", },
	{},
};
MODULE_DEVICE_TABLE(of, tsp_dt_ids);

static int bt541_ts_probe_dt(struct device_node *np,
		struct device *dev,
		struct bt541_ts_platform_data *pdata)
{
	//struct bt541_ts_info *info = dev_get_drvdata(dev);
	int ret;

	input_dbg(true, dev, "%s [START]\n", __func__);

	if (!np)
		return -EINVAL;

	pdata->gpio_int = of_get_named_gpio(np,"bt541,irq-gpio", 0);
	if (pdata->gpio_int < 0) {
		input_err(true, dev, "%s: of_get_named_gpio failed: %d\n", __func__,
				pdata->gpio_int);
		pdata->gpio_int = 188;
	}

	pdata->vdd_en = of_get_named_gpio(np,"bt541,vdd_en", 0);
	if (pdata->vdd_en < 0) {
		pdata->vdd_en_flag = 0;
		input_err(true, dev, "%s: vdd_en_flag %d\n", __func__, pdata->vdd_en_flag);
	} else {
		pdata->vdd_en_flag = 1;
		input_err(true, dev, "%s: vdd_en_flag %d\n", __func__, pdata->vdd_en_flag);
	}

	ret = of_property_read_u32(np, "bt541,x_resolution", &pdata->x_resolution);
	if (ret) {
		pdata->x_resolution = 539;
		input_err(true, dev, "%s [ERROR] max_x and hard data insert %d\n", __func__, pdata->x_resolution);
	}

	ret = of_property_read_u32(np, "bt541,y_resolution", &pdata->y_resolution);
	if (ret) {
		pdata->y_resolution = 959;
		input_err(true, dev, "%s [ERROR] max_y and hard data insert %d\n", __func__, pdata->y_resolution);
	}

	ret = of_property_read_u32(np, "bt541,orientation", &pdata->orientation);
	if (ret) {
		pdata->orientation = 0;
		input_err(true, dev, "%s [ERROR] orientation and hard data insert %d\n", __func__, pdata->orientation);
	}

	ret = of_property_read_u32(np, "bt541,page_size", &pdata->page_size);
	if (ret) {
		pdata->page_size = 128;
		input_err(true, dev, "%s [ERROR] orientation and hard data insert %d\n", __func__, pdata->page_size);
	}
	ret = of_property_read_string(np, "bt541,fw_name", &pdata->fw_name);
	if (ret < 0)
		pdata->fw_name = "tsp_zinitix/bt541_GP.fw";

	ret = of_property_read_u32(np, "bt541,bringup", &pdata->bringup);
	if (ret < 0)
		pdata->bringup = 0;

	ret = of_property_read_u32(np, "bt541,mis_cal_check", &pdata->mis_cal_check);
	if (ret < 0)
		pdata->mis_cal_check = 0;

#ifdef	PAT_CONTROL
	ret = of_property_read_u32(np, "bt541,pat_function", &pdata->pat_function);
	if (ret < 0) {
		pdata->pat_function = 0;
		input_err(true, dev, "Failed to get pat_function property\n");
	}

	ret = of_property_read_u32(np, "bt541,afe_base", &pdata->afe_base);
	if (ret < 0) {
		pdata->afe_base = 0;
		input_err(true, dev, "Failed to get afe_Base property\n");		  
	}
#endif
	input_info(true, dev, "%s: get touch_regulator start\n");
	pdata->vreg_vio = regulator_get(dev, "vgp1");
	if (IS_ERR(pdata->vreg_vio)) {
		pdata->vreg_vio = NULL;
		input_info(true, dev, "%s: get touch_regulator error\n",
				__func__);
		if (pdata->vdd_en_flag) {
			input_info(true, dev, "%s: touch vdd control use gpio_en\n", __func__);
		} else {
			input_info(true, dev, "%s: touch vdd control do not use gpio_en"
				"and don't get regulator error\n", __func__);
			return -EIO;
		}

	}
	if (pdata->vreg_vio) {
		ret = regulator_set_voltage(pdata->vreg_vio, 3000000, 3000000);
		if (ret < 0)
			input_info(true, dev, "%s: set voltage error(%d)\n", __func__, ret);
	}
	input_info(true, dev, "%s max_x:%d, max_y:%d, irq:%d, orientation:%d, page_size:%d,"
		"bringup:%d vdd_en_flag:%d pat_function:%d afe_base:%d mis_cal_check:%d\n", __func__, pdata->x_resolution,
		pdata->y_resolution, pdata->gpio_int, pdata->orientation,
		pdata->page_size, pdata->bringup, pdata->vdd_en_flag, pdata->pat_function, pdata->afe_base, pdata->mis_cal_check);

	return 0;
}
#endif

int bt541_pinctrl_configure(struct bt541_ts_info *info, int active)
{
	struct pinctrl_state *set_state_i2c;
	int retval;

	input_dbg(true, &info->client->dev, "%s: %d\n", __func__, active);

	if (active) {
		set_state_i2c =	pinctrl_lookup_state(info->pinctrl, "tsp_int_active");
		if (IS_ERR(set_state_i2c)) {
			input_err(true, &info->client->dev,
				"%s: cannot get pinctrl(i2c) active state\n", __func__);
			return PTR_ERR(set_state_i2c);
		}
	} else {
		set_state_i2c =	pinctrl_lookup_state(info->pinctrl, "tsp_int_suspend");
		if (IS_ERR(set_state_i2c)) {
			input_err(true, &info->client->dev,
				"%s: cannot get pinctrl(i2c) suspend state\n", __func__);
			return PTR_ERR(set_state_i2c);
		}
	}
	retval = pinctrl_select_state(info->pinctrl, set_state_i2c);
	if (retval) {
		input_err(true, &info->client->dev,
			"%s: cannot set pinctrl(i2c) %d state\n", __func__, active);
		return retval;
	}

	return 0;
}


static int bt541_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bt541_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	int i;
	bool input_registered = false, irq_registered = false;
	bool misc_registered = false, gpio_requested = false;
	bool notifications_registered = false;

	struct device_node *np = client->dev.of_node;

	/*info->octa_id = get_lcd_attached("GET");
	if (!octa_id) {
		input_err(true, &client->dev, "%s: LCD is not attached\n", __func__);
		ret = -EIO;
		goto err_octa_id;
	}*/

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev, "Not compatible i2c function\n");
		ret = -EIO;
		goto err_i2c_check;
	}
	
	info = kzalloc(sizeof(struct bt541_ts_info), GFP_KERNEL);
	if (!info) {
		input_err(true, &client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	mutex_init(&info->v8_control_lock);
	INIT_DELAYED_WORK(&info->v8_work, bt541_v8_work_fn);
	info->v8_auto_enabled = BT541_V8_DEFAULT_AUTO;
	atomic_set(&info->v8_screen_on, 0); /* wait for confirmed FB unblank */
	info->v8_last_point = jiffies;
	sema_init(&info->work_lock, 1);
	sema_init(&info->raw_data_lock, 1);
	esd_timer_init(info);
	INIT_WORK(&info->tmr_work, ts_tmr_work);
	info->client = client;
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		info->pdata = devm_kzalloc(&client->dev,
				sizeof(struct bt541_ts_platform_data), GFP_KERNEL);
		if (!info->pdata) {
			input_err(true, &client->dev,
					"%s [ERROR] pdata devm_kzalloc\n", __func__);
			goto err_no_platform_data;
		}
		ret = bt541_ts_probe_dt(np, &client->dev, info->pdata);
		if (ret) {
			input_err(true, &client->dev, "%s [ERROR] tsp is not"
					" vdd_en or vdd_regulator\n", __func__);
			goto err_no_platform_data;
		}
	} else
#endif
	{
		info->pdata = client->dev.platform_data;
		if (info->pdata == NULL) {
			input_err(true, &client->dev, "%s [ERROR] pdata is null\n", __func__);
			ret = -EINVAL;
			goto err_no_platform_data;
		}
	}

	ret = gpio_request(info->pdata->gpio_int, "bt541_int");
	if (ret < 0) {
		input_err(true, &client->dev, "%s: Request GPIO failed, gpio %d (%d)\n", BT541_TS_DEVICE,
				info->pdata->gpio_int, ret);
		goto err_gpio_alloc;
	}
	gpio_requested = true;
	/* if pinctrl set up at dts file, gpio_direction_input don't use. */ 
	/*gpio_direction_input(info->pdata->gpio_int);*/
	i2c_set_clientdata(client, info);
	info->client = client;
	input_dev = input_allocate_device();
	if (!input_dev) {
		input_err(true, &client->dev, "Failed to allocate input device\n");
		ret = -EPERM;
		goto err_alloc;
	}

	info->input_dev = input_dev;
	info->work_state = PROBE;
	info->enabled = true;





	spin_lock_init(&info->health.lock);
	atomic_set(&info->health.irq, 0);
	atomic_set(&info->health.invalid_gpio, 0);
	atomic_set(&info->health.lock_busy, 0);
	atomic_set(&info->health.state_busy, 0);
	atomic_set(&info->health.coord_recovery, 0);
	atomic_set(&info->health.heartbeat, 0);
	atomic_set(&info->health.nonzero_packet, 0);
	atomic_set(&info->health.contacts, 0);
	atomic_set(&info->health.sync, 0);
	atomic_set(&info->health.invalid_coord, 0);
	memset(&info->health.packet, 0, sizeof(info->health.packet));
	info->health.packet_mode = TOUCH_POINT_MODE;
	info->health.last_packet_jiffies = 0;
	info->health.last_contact_jiffies = 0;
	info->health.packet_valid = false;
	info->health.contact_valid = false;

	/* because buffer size, MTK use TPD_SUPPORT_I2C_DMA */ 
#if TPD_SUPPORT_I2C_DMA
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	gpDMABuf_va = (u8 *) dma_alloc_coherent(&client->dev, IIC_DMA_MAX_TRANSFER_SIZE,
		&gpDMABuf_pa, GFP_KERNEL);
	if (!gpDMABuf_va) {
		input_err(true, &client->dev, "%s Allocate DMA I2C Buffer failed!\n", __func__);
		ret = -EPERM;
		goto err_dma;
	}
	memset(gpDMABuf_va, 0, IIC_DMA_MAX_TRANSFER_SIZE);
#endif

	/* MTK use DTS file pinctrl, If it don't use, pinctrl is default. */
	info->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(info->pinctrl)) {
		if (PTR_ERR(info->pinctrl) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto err_pinctrl;
		}

		input_err(true, &info->client->dev, "%s: Target does not use pinctrl\n", __func__);
		info->pinctrl = NULL;
	}

	if (info->pinctrl) {
		ret = bt541_pinctrl_configure(info, 1);
		if (ret)
			input_err(true, &info->client->dev, "%s: cannot set pinctrl state\n", __func__);
	}

	if (bt541_power_control(info, POWER_ON_SEQUENCE) == false) {
		ret = -EPERM;
		goto err_power_sequence;
	}
	
	/* FW version read from tsp */

	memset(&info->reported_touch_info,
			0x0, sizeof(struct point_info));

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;
	mutex_lock(&bt541_device_lock);
	misc_info = info;
	mutex_unlock(&bt541_device_lock);
	info->pat_flag = false;
	if (init_touch(info, fw_true) == false) {
		ret = -EPERM;
		goto err_init_touch;
	}

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		info->button[i] = ICON_BUTTON_UNCHANGE;

	snprintf(info->phys, sizeof(info->phys),
			"%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchscreen";
	input_dev->id.bustype = BUS_I2C;
	/*	input_dev->id.vendor = 0x0001; */
	input_dev->phys = info->phys;
	/*	input_dev->id.product = 0x0002; */
	/*	input_dev->id.version = 0x0100; */
	input_dev->dev.parent = &client->dev;
#if BT541_USE_INPUT_OPEN_CLOSE
	input_dev->open = bt541_ts_open;
	input_dev->close = bt541_ts_close;
#endif
	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(BUTTON_MAPPING_KEY[i], info->input_dev->keybit);

	if (info->pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
				info->cap_info.MinX,
				info->cap_info.MaxX + ABS_PT_OFFSET,
				0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
				info->cap_info.MinY,
				info->cap_info.MaxY + ABS_PT_OFFSET,
				0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
				info->cap_info.MinX,
				info->cap_info.MaxX + ABS_PT_OFFSET,
				0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
				info->cap_info.MinY,
				info->cap_info.MaxY + ABS_PT_OFFSET,
				0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
			0, 255, 0, 0);
#ifdef CONFIG_SEC_FACTORY
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
			0, 3000, 0, 0);
#endif
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR,
			0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR,
			0, 255, 0, 0);
	/*	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MINOR,
		0, 255, 0, 0); */
	/*	input_set_abs_params(info->input_dev, ABS_MT_ORIENTATION,
		-128, 127, 0, 0);  */
	/*	input_set_abs_params(info->input_dev, ABS_MT_ANGLE,
		-90, 90, 0, 0);*/
	input_set_abs_params(info->input_dev, ABS_MT_PALM,
			0, 1, 0, 0);
#endif

	set_bit(MT_TOOL_FINGER, info->input_dev->keybit);
	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers,0);

	input_info(true, &client->dev, "register %s input device\n",
			info->input_dev->name);
	input_set_drvdata(info->input_dev, info);
	ret = input_register_device(info->input_dev);
	if (ret) {
		input_err(true, &info->client->dev, "unable to register %s input device\r\n",
				info->input_dev->name);
		goto err_input_register_device;
	}

	input_registered = true;
	/* configure irq */
	info->irq = gpio_to_irq(info->pdata->gpio_int);
	if (info->irq < 0)
		input_info(true, &client->dev, "error. gpio_to_irq(..) function is not \
				supported? you should define GPIO_TOUCH_IRQ.\n");

	input_info(true, &client->dev, "request irq (irq = %d, pin = %d) \r\n",
			info->irq, info->pdata->gpio_int);

	info->work_state = NOTHING;



#if ESD_TIMER_INTERVAL
	spin_lock_init(&info->lock);
	INIT_WORK(&info->tmr_work, ts_tmr_work);
	esd_tmr_workqueue =
		create_singlethread_workqueue("esd_tmr_workqueue");

	if (!esd_tmr_workqueue) {
		input_err(true, &client->dev, "Failed to create esd tmr work queue\n");
		ret = -EPERM;

		goto err_esd_input_unregister_device;
	}



#if defined(TSP_VERBOSE_DEBUG)
	input_info(true, &client->dev, "Started esd timer\n");
#endif
#endif
	ret = request_threaded_irq(info->irq, NULL, bt541_touch_work,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT , BT541_TS_DEVICE, info);

	if (ret) {
		input_err(true, &client->dev, "unable to register irq.(%s)\n",
				info->input_dev->name);
		goto err_request_irq;
	}
	irq_registered = true;
	input_info(true, &client->dev, "zinitix touch probe.\r\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = bt541_ts_early_suspend;
	info->early_suspend.resume = bt541_ts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

#if defined(CONFIG_PM_RUNTIME) && !defined(CONFIG_HAS_EARLYSUSPEND)
	pm_runtime_enable(&client->dev);
#endif



	notifications_registered = true;
	ret = misc_register(&touch_misc_device);
	if (ret) {
		input_err(true, &client->dev, "Failed to register touch misc device\n");
		goto err_misc_register;
	}

	misc_registered = true;
#ifdef CONFIG_SEC_FACTORY_TEST
	ret = init_sec_factory(info);
	if (ret) {
		input_err(true, &client->dev, "Failed to init sec factory device\n");

		goto err_kthread_create_failed;
	}
#endif
#ifdef CONFIG_FB
	info->v8_fb.notifier_call = bt541_v8_fb_event;
	ret = fb_register_client(&info->v8_fb);
	if (!ret)
		info->v8_fb_registered = true;
	else
		input_err(true, &client->dev, "V8 FB registration failed: %d; auto paused\n", ret);
#endif
	mutex_lock(&info->v8_control_lock);
	WRITE_ONCE(info->v8_busy, true);
	disable_irq(info->irq);
	write_cmd(client, BT541_CLEAR_INT_STATUS_CMD);
	WRITE_ONCE(info->v8_ready, true);
	if (info->v8_auto_enabled && atomic_read(&info->v8_screen_on))
		mod_delayed_work(system_wq, &info->v8_work,
			msecs_to_jiffies(BT541_V8_INTERVAL_MS));
	WRITE_ONCE(info->v8_busy, false);
	enable_irq(info->irq);
	esd_timer_start(CHECK_ESD_TIMER, info);
	mutex_unlock(&info->v8_control_lock);
	input_info(true, &client->dev, "BT541 V8 loaded: auto=%u, threshold=%d\n",
		info->v8_auto_enabled, BT541_V8_DELTA_THRESHOLD);
	return 0;

#ifdef CONFIG_SEC_FACTORY_TEST
err_kthread_create_failed:
#endif
err_misc_register:
err_request_irq:
#if ESD_TIMER_INTERVAL
err_esd_input_unregister_device:
#endif
err_input_register_device:
err_init_touch:
err_power_sequence:
err_pinctrl:
#if TPD_SUPPORT_I2C_DMA
err_dma:
#endif
err_alloc:
err_gpio_alloc:
err_no_platform_data:
	mutex_lock(&info->v8_control_lock);
	WRITE_ONCE(info->v8_stopping, true);
	WRITE_ONCE(info->enabled, false);
	WRITE_ONCE(info->v8_busy, true);
	mutex_unlock(&info->v8_control_lock);
	if (notifications_registered)
		bt541_v8_unregister_notifications(info);
#ifdef CONFIG_SEC_FACTORY_TEST
	bt541_v8_exit_factory(info);
#endif
	if (misc_registered)
		misc_deregister(&touch_misc_device);
	mutex_lock(&bt541_device_lock);
	if (misc_info == info)
		misc_info = NULL;
	mutex_unlock(&bt541_device_lock);
	if (irq_registered)
		disable_irq(info->irq);
	cancel_delayed_work_sync(&info->v8_work);
	esd_timer_stop(info);
	cancel_work_sync(&info->tmr_work);
	esd_timer_stop(info);
	if (irq_registered)
		free_irq(info->irq, info);
	if (esd_tmr_workqueue) {
		destroy_workqueue(esd_tmr_workqueue);
		esd_tmr_workqueue = NULL;
	}
	if (input_registered)
		input_unregister_device(info->input_dev);
	else if (info->input_dev)
		input_free_device(info->input_dev);
#if TPD_SUPPORT_I2C_DMA
	if (gpDMABuf_va) {
		dma_free_coherent(&client->dev, IIC_DMA_MAX_TRANSFER_SIZE,
			gpDMABuf_va, gpDMABuf_pa);
		gpDMABuf_va = NULL;
		gpDMABuf_pa = 0;
	}
#endif
	if (info->pdata) {
		bt541_power_control(info, POWER_OFF);
#ifdef CONFIG_OF
		if (client->dev.of_node && !IS_ERR_OR_NULL(info->pdata->vreg_vio))
			regulator_put(info->pdata->vreg_vio);
#endif
	}
	if (gpio_requested)
		gpio_free(info->pdata->gpio_int);
#ifdef CONFIG_SEC_FACTORY_TEST
	kfree(info->raw_data);
#endif
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&info->v8_control_lock);
	kfree(info);
err_mem_alloc:
err_i2c_check:
	return ret ? ret : -ENOMEM;
}

static int bt541_ts_remove(struct i2c_client *client)
{
        struct bt541_ts_info *info = i2c_get_clientdata(client);
        struct bt541_ts_platform_data *pdata;
        bool was_enabled;

        if (!info)
                return 0;
        pdata = info->pdata;

        was_enabled = bt541_v8_begin_terminal_stop(info);
        bt541_v8_unregister_notifications(info);

#ifdef CONFIG_SEC_FACTORY_TEST
        bt541_v8_exit_factory(info);
#endif
        misc_deregister(&touch_misc_device);

        /* An already-open misc FD still exists after misc_deregister. Drain
         * its in-flight ioctl and make every later ioctl return -ENODEV.
         * Never take device_lock while holding control_lock (opposite order).
         */
        mutex_lock(&bt541_device_lock);
        if (misc_info == info)
                misc_info = NULL;
        mutex_unlock(&bt541_device_lock);

        mutex_lock(&info->v8_control_lock);
        bt541_v8_quiesce_locked(info, was_enabled);
        mutex_unlock(&info->v8_control_lock);

        free_irq(info->irq, info);
#if ESD_TIMER_INTERVAL
        if (esd_tmr_workqueue) {
                destroy_workqueue(esd_tmr_workqueue);
                esd_tmr_workqueue = NULL;
        }
#endif

        /* input_unregister invokes close. Do not hold either driver lock;
         * the close wrapper must see v8_stopping and leave the IRQ alone.
         */
        input_unregister_device(info->input_dev);
        info->input_dev = NULL;

#ifdef CONFIG_SEC_FACTORY_TEST
        kfree(info->raw_data);
        info->raw_data = NULL;
#endif
#if TPD_SUPPORT_I2C_DMA
        if (gpDMABuf_va) {
                dma_free_coherent(&client->dev, IIC_DMA_MAX_TRANSFER_SIZE,
                                gpDMABuf_va, gpDMABuf_pa);
                gpDMABuf_va = NULL;
                gpDMABuf_pa = 0;
        }
#endif
        if (gpio_is_valid(pdata->gpio_int))
                gpio_free(pdata->gpio_int);

        /* The DT path obtains its own regulator reference with regulator_get.
         * Platform-data ownership is external and is therefore left alone.
         */
#ifdef CONFIG_OF
        if (client->dev.of_node && !IS_ERR_OR_NULL(pdata->vreg_vio)) {
                regulator_put(pdata->vreg_vio);
                pdata->vreg_vio = NULL;
        }
#endif
        i2c_set_clientdata(client, NULL);
        mutex_destroy(&info->v8_control_lock);
        kfree(info);
        return 0;
}

void bt541_ts_shutdown(struct i2c_client *client)
{
        struct bt541_ts_info *info = i2c_get_clientdata(client);
        bool was_enabled;

        if (!info)
                return;

        was_enabled = bt541_v8_begin_terminal_stop(info);
        bt541_v8_unregister_notifications(info);

#ifdef CONFIG_SEC_FACTORY_TEST
        /* Touchkey attributes are independent of SEC_CMD wrappers. Drain
         * them as well, so no sensitivity read can start after power-off.
         */
        bt541_v8_exit_factory(info);
#endif
        mutex_lock(&info->v8_control_lock);
        bt541_v8_quiesce_locked(info, was_enabled);
        mutex_unlock(&info->v8_control_lock);
}


static struct i2c_device_id bt541_idtable[] = {
	{BT541_TS_DEVICE, 0},
	{ }
};

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/* LineageOS: register both system-sleep and runtime PM callbacks. */
static const struct dev_pm_ops bt541_ts_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bt541_ts_suspend, bt541_ts_resume)
#if defined(CONFIG_PM_RUNTIME)
	SET_RUNTIME_PM_OPS(bt541_ts_suspend, bt541_ts_resume, NULL)
#endif
};
#endif

static struct i2c_driver bt541_ts_driver = {
	.probe		= bt541_ts_probe,
	.remove		= bt541_ts_remove,
	.shutdown	= bt541_ts_shutdown,
	.id_table	= bt541_idtable,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= BT541_TS_DEVICE,
		.of_match_table	= tsp_dt_ids,
/* LineageOS: allow BT541 to actually suspend with the device. */
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm		= &bt541_ts_pm_ops,
#endif
	},
};

#ifdef CONFIG_BATTERY_SAMSUNG
extern unsigned int lpcharge;
#endif

static void __init bt541_ts_init_async(void *dummy, async_cookie_t cookie)
{
	i2c_add_driver(&bt541_ts_driver);
}

static int __init bt541_ts_init(void)
{
#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge) {
		pr_err("%s %s : LPM Charging mode!!\n", SECLOG, __func__);
		return 0;
	}
#endif
	pr_err("%s %s \n", SECLOG, __func__);

	return async_schedule(bt541_ts_init_async, NULL); 
}

static void __exit bt541_ts_exit(void)
{
	i2c_del_driver(&bt541_ts_driver);
}

module_init(bt541_ts_init);
module_exit(bt541_ts_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_LICENSE("GPL");

