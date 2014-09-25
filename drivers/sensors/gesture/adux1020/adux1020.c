/**
 *\mainpage
 * ADUX1020 driver
 \n
 * @copyright 2014 Analog Devices Inc.
 \n
 * Licensed under the GPL version 3 or later.
 * \date	June-2014
 * \version	Driver 2.1
 * \version	Linux 3.4.0
 * \version	Android 4.3
*/
/**
@file adux1020.c
@brief ADUX1020	- Low level driver  Source 'C' File
*/
#include <linux/types.h>
#include <linux/workqueue.h>	/* workqueue related header */
#include <linux/kernel.h>	/* kernel related header */
#include <linux/err.h>		/* error related macro */
#include <linux/slab.h>		/* memory allocation related header */
#include <linux/device.h>	/* device related header */
#include <linux/input.h>	/* input related header */
#include <linux/delay.h>	/* sleep realted header */
#include <linux/irq.h>		/*  irq related functionality */
#include <linux/interrupt.h>	/* interrupt related functionality  */
#include <linux/sysfs.h>	/* sysfs related dunctionality */
#include <linux/uaccess.h>	/* filp open usage */
#include <linux/stat.h>		/* user mode permission */
#include <linux/init.h>		/* module initialization api */
#include <linux/module.h>	/* module functionality */
#include <linux/i2c.h>		/* i2c related functionality */
#include <linux/jiffies.h>	/* jiffies related macro and api */
#include <linux/time.h>
#include <linux/pm.h>

#include <linux/sensors/adux1020.h>		/**< adux1020 chip related header*/
#include <linux/sensors/sensparams.h>
#include <linux/wakelock.h>


#define VERSION	"ADUX1020 Android Generic driver 2.0"
//#ifdef CONFIG_FB
//#include <linux/notifier.h>
//#include <linux/fb.h>
/*#elif defined(CONFIG_xxxxx)*/
//#endif

#define ADUX_STAT
#define PC_TOOL_SUPPORT
#define INT_EDGE

#ifdef ADUX_STAT
#include <linux/utsname.h>
#include <linux/average.h>
#else
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

/**
	@brief below value may depend upon the platform to platform due to glass
*/
#define R_GEST_DI_TH	30
/**
	@brief When value is set as high it detect slow gesture
*/
#define R_GEST_DIDT_M	0x7
/**
	@brief gesture points
*/
#define R_GEST_N_PTS	0x5
/**
	@brief Macro for wakeup key
*/
#define WAKEUP_KEY	KEY_POWER

/*sleep*/
#define SLEEP           3000

#define MIN_DISTANCE	3
#define MAX_DISTANCE	5

/************* FILP - CONFIG **************************************/
/**
	@brief Macro to fetch configuration value
*/
#define		CONFIG(VALUE)			CONFIG_##VALUE##_VALUE
/**
	@brief General configuration value
*/
#define		CONFIG_GENERAL_VALUE            1
/**
	@brief Low Power configuration value
*/
#define		CONFIG_LOW_POWER_VALUE		6
/**
	@brief configuration form filp
*/
#define		FRM_FILE			0
/**
	@brief configuration from array
*/
#define		FRM_ARR				1
/*	@brief directory based on android*/
/**
	@brief General configuration path
*/
#define		GENERAL_CONFIG		"/data/misc/StandardR1.dcfg"
/**
	@brief Low Power configuration path
*/
#define		LOW_POWER_CONFIG	"/data/misc/LowPowerR1.dcfg"

/******************************************************************/

/**************** ADUX1020_CHIP_REG_ADDR ****************************/
/**
	@brief CHIP_ID VERSION Address.
*/
#define		ADUX_CHIP_ID_VERSION_ADDR	0x8
/**
	@brief Oscillator Calibration output register
*/
#define		ADUX_OSC_CAL_OUT		0xA
/**
	@brief Software Reset setting register
*/
#define		ADUX_SW_RESET			0xF
/**
	@brief ADUX1020 Oscillator set register 1
*/
#define		ADUX_OSCS_1			0x18
/**
	@brief ADUX1020 Oscillator set register 3
*/
#define		ADUX_OSCS_3			0x1A
/**
	@brief I2C control and speed mode register
*/
#define		ADUX_I2C_1			0x1E
/**
	@brief ADUX1020 FIFO threshold register
*/
#define		ADUX_I2C_2			0x1F
/**
	@brief ADUX1020 R_GEST_DI_DT register
*/
#define         ADUX_GEST_ORIEN_NPTS            0x29
/**
	@brief ADUX1020 R_GEST_DI_DT register
*/
#define		ADUX_GEST_DI_TH			0x28
/**
	@brief ADUX1020 Proximity Type register
*/
#define		ADUX_PROX_TYPE			0x2F
/**
	@brief ADUX1020 calibration enable register
*/
#define		ADUX_TEST_MODES_1		0x30
/**
	@brief ADUX1020 power down control and Slave address
	configurable register
*/
#define		ADUX_TEST_MODES_3		0x32
/**
	@brief ADUX1020 force mode register
*/
#define		ADUX_FORCE_MODE_ADDR		0x33
/**
	@brief ADUX1020 frequency register
*/
#define		ADUX_FREQUENCY			0x40
/**
	@brief ADUX1020 Decimation register
*/
#define		ADUX_DECIMATION			0x46
/**
	@brief ADUX1020 operating mode register
*/
#define		ADUX_OP_MODE_ADDR		0x45
/**
	@brief ADUX1020 interrupt mask register
*/
#define		ADUX_INT_MASK_ADDR		0x48
/**
	@brief ADUX1020 interrupt status register
*/
#define		ADUX_INT_STATUS_ADDR		0x49
/**
	@brief ADUX1020 data buffer register
*/
#define		ADUX_DATA_BUFFER		0x60
/*******************************************************************/
/**************** ADUX1020 Driver **********************************/
/**
	@brief vendor name for class create
*/
#define		VENDOR				"ADUX1020"
/**
	@brief Gesture input device name
*/
#define		ADUX1020_INPUT1_NAME		"adux1020_gesture"
/**
	@brief Proximity input device name
*/
#define		ADUX1020_INPUT2_NAME		"adux1020_proximity"
/**
	@brief Define number of times the key event to send
*/
#define		GEST_KEY_REPEAT_RANGE		1
/**
	@brief ADUX1020 fifo size mask
*/
#define		FIFO_SIZE			0x7F00
/**
	@brief Self Resetting set enable
*/
#define		SELF_RESETTING			0x8000
/**
	@brief Maximum retry count for retrying a register
*/
#define		MAX_RETRY			5
/*******************************************************************/
/**************** ADUX1020 CHIP REG VALUE **************************/
/**
	@brief Force Clock On enable value
*/
#define		FORCE_CLOCK_ON			0xF4F
/**
	@brief Force Clock Reset value
*/
#define		FORCE_CLOCK_RESET		0x40
/*******************************************************************/
/**************** I2C - POWER UP ***********************************/
/**
	@brief chip ID mask value
*/
#define		CHIP_ID_MASK			0x0FFF
/**
	@brief macro to get version number
*/
#define		GET_VERSION(value)		((value & 0xF000) >> 12)
/**
	@brief i2c speed mask value
*/
#define		I2C_SPEED_MASK			0x1000
/**
	@brief ADUX1020 software reset set value
*/
#define		SW_RESET_SET			0x0001
/*******************************************************************/
/**************** I2C - READ, WRITE ********************************/
/**
	@brief condition failed
*/
#define		FAILED				0
/**
	@brief condition success
*/
#define		SUCCESS				1
/**
	@brief size of register in bytes
*/
#define		ADUX1020_REG_SIZE_BYTES		2
/**
	@brief size of register in byte
*/
#define		ADUX1020_REG_SIZE_1BYTE		1
/**
	@brief retry count for loop
*/
#define		RETRY_CNT			5
/**
	@brief maximum data count for i2c write
*/
#define		MAX_DATA_CNT			8
/**
	@brief ADUX1020 big endian format
*/
#define		ADUX1020_BIG_ENDIAN		1
/**
	@brief ADUX1020 little endian format
*/
#define		ADUX1020_LITTLE_ENDIAN		2
/**
	@brief ADUX1020 address range limit
*/
#define		ADUX_ADDRESS_RANGE		127
/**
	@brief data size in adux1020 mentioned in bytes
*/
#define		DATA_SIZE			2
/**
	@brief word size in adux1020 mentioned in word(16bits)
*/
#define		WORD_SIZE			1
/**
	@brief message count used in i2c transfer method
*/
#define		MSG_EXECUTED			2
/******************************************************************/
/***************** OPERATING MODES ********************************/
/**
	@brief true conditions
*/
#define		TRUE				1
/**
	@brief false conditions
*/
#define		FALSE				-1

/**
	@brief OFF state used in proximity
*/
#define		OFF				0x0
/**
	@brief ON state used in proximity
*/
#define		ON				0x1
/**
	@brief UNKNOWN state used in proximity
*/
#define		UNKNOWN				0x2
/**
	@brief ADUX1020 Proximity operating mode value from user space
*/
#define		PROXIMITY			0x1
/**
	@brief ADUX1020 Proximity I operating mode value from user space
*/
#define		PROXIMITY_I			0x2
/**
	@brief ADUX1020 Force operating mode value from driver
*/
#define		FORCE				0xE
/**
	@brief ADUX1020 Idle operating mode value from user space
*/
#define		IDLE				0xF

/**
	@brief ADUX1020 Gesture operating mode value from user space
*/
#define		GESTURE				0x10
/**
	@brief ADUX1020 Sample operating mode value from user space
*/
#define		SAMPLE				0x20
/**
	@brief ADUX1020 Sample RAW operating mode value from user space
*/
#define		SAMPLE_RAW			0x20
/**
	@brief ADUX1020 Fifo operating mode value from user space
*/
#define		FIFO				0x30
/**
	@brief ADUX1020 R_ORIENTATION mask value
*/
#define         R_ORIENTATION_MASK      (0xFFFF & (0x7))
/**
	@brief ADUX1020 Fifo prevent mask value
*/
#define		R_FIFO_PREVENT_MASK		(0xFFFF & ~(0x1 << 8))

/**
	@brief ADUX1020 Sample out mask value
*/
#define		R_SAMP_OUT_MASK			(0xFFFF & ~(0x1 << 12))

/**
	@brief ADUX1020 Fifo prevent enable macro
*/
#define		R_FIFO_PREVENT_EN(value)	(value << 8)

/**
	@brief ADUX1020 Sample out mode enable macro
*/
#define		R_SAMP_OUT_MODE(value)		(value << 12)

/**
	@brief ADUX1020 Read outdata flag enable macro
*/
#define		R_RDATA_FLAG_EN			(0x1 << 11)
/**
	@brief ADUX1020 Read back data out enable macro
*/
#define		R_RDOUT_MODE			(0x1 << 13)

/**
	@brief Off operating mode value to ADUX1020
*/
#define		OP_OFF_DATA_NONE		(0x0000)

/**
	@brief Idle operating mode value to ADUX1020
*/
#define		OP_IDLE_DATA_NONE		(0xF/*OP_MODE*/)

/**
	@brief Proximity none operating mode value to ADUX1020
*/
#define		OP_PROXIMITY_DATA_NONE		((0x1/*OP_MODE*/) |\
						(0x0/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Proximity I operating mode value to ADUX1020
*/
#define		OP_PROXIMITY_DATA_I		((0x1/*OP_MODE*/) |\
						(0x1/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)


/**
	@brief Gesture none operating mode value to ADUX1020
*/
#define		OP_GESTURE_DATA_NONE		((0x2/*OP_MODE*/) |\
						(0x2/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Sample Raw operating mode value to ADUX1020
*/
#define		OP_SAMPLE_DATA_RAW		((0x8/*OP_MODE*/) |\
						(0x4/*DATA_MODE*/ << 4) |\
						R_RDATA_FLAG_EN | R_RDOUT_MODE)

/**
	@brief Operating mode Interrupt mask value
*/
#define		OP_MODE_INT_MASK		0x00FF

/**
	@brief General mode Interrupt mask value
*/
#define		INTR_GENERAL			0x00FF
/**
	@brief Off mode Interrupt mask value
*/
#define		INTR_OFF			0x0000
/**
	@brief Idle mode Interrupt mask value
*/
#define		INTR_IDLE			0x0000
/**
	@brief Proximity Interrupt mask value
*/
#define		INTR_PROXIMITY			0x000F
/**
	@brief Gesture Interrupt mask value
*/
#define		INTR_GESTURE			0x0010
/**
	@brief Sample Interrutp mask value
*/
#define		INTR_SAMPLE			0x0020
/**
	@brief Watchdog Interrupt mask value
*/
#define		INTR_WATCHDOG			0x0040
/**
	@brief Fifo Interrupt mask value
*/
#define		INTR_FIFO			0x0080

/**
	@brief macro to fetch operating mode value from Operating
	mode from user space and corresponding data out mode.
*/
#define		SET_MODE_VALUE(OP_MODE, DATA_MODE)\
					OP_##OP_MODE##_DATA_##DATA_MODE

/**
	@brief macro to fetch Interrupt mask value from Operating
	mode from user space.
*/
#define		SET_INTR_MASK_VALUE(OP_MODE)	\
				((~(INTR_##OP_MODE)) & OP_MODE_INT_MASK)

/**
	@brief macro to fetch Interrupt status value from Operating
	mode from user space.
*/
#define		INTR_STATUS_MASK(OP_MODE)	(INTR_##OP_MODE)

/**
	@brief Proximity 1 mask value
*/
#define		PROXIMITY_1			0x3
/**
	@brief Proximity 2 mask value
*/
#define		PROXIMITY_2			0xC
/**
	@brief Proximity ON1 mask value
*/
#define		PROX_ON1			0x1
/**
	@brief Proximity OFF1 mask value
*/
#define		PROX_OFF1			0x2
/**
	@brief Proximity ON2 mask value
*/
#define		PROX_ON2			0x4
/**
	@brief Proximity OFF2 mask value
*/
#define		PROX_OFF2			0x8
/******************************************************************/
/**************** FILP ********************************************/
/**
	@brief Minimum loop to execute
*/
#define		MINI_SIZE			256
/******************************************************************/
/**************** EVENT/BUFFER SIZE *******************************/
/**
	@brief Proximity I buffer size
*/
#define		PROX_I_BUF_SIZE			1
/**
	@brief Gesture buffer size
*/
#define		GEST_BUF_SIZE			2
/**
	@brief Sample RAW buffer size
*/
#define		SAMPLE_RAW_BUF_SIZE		4
/**
	@brief Maximum buffer size
*/
#define		MAX_BUFFER			72

/******************************************************************/
/**************** DEBUG PRINT *************************************/
/**
  @def ADUX_DBG_PRINT
  @brief ADUX_DBG_PRINT is defined during build to enable dbg print
 */

#define ADUX_DBG_PRINT
#define ADUX_ERR_PRINT
#define ADUX_INFO_PRINT

#ifdef ADUX_DBG_PRINT
#define ADUX1020_dbg(format, arg...)    printk(KERN_ERR "ADUX1020 : "\
							format,	##arg)
#else
#define ADUX1020_dbg(format, arg...)    {if (0); }
#endif
/**
	@def ADUX_INFO_PRINT
	@brief ADUX_INFO_PRINT is defined during build to enable info print
	@note enabling this print may DEGRADE THE PERFORMANCE.
 */
#ifdef ADUX_INFO_PRINT
#define ADUX1020_info(format, arg...)   printk(KERN_ERR "ADUX1020 : "\
							format, ##arg)
#else
#define ADUX1020_info(format, arg...)	{if (0); }
#endif
/**
	@def ADUX_ERR_PRINT
	@brief ADUX_ERR_PRINT is defined during build to enable err print
 */
#ifdef ADUX_ERR_PRINT
#define ADUX1020_err(format, arg...)    printk(KERN_ERR "ADUX1020 Error : "\
							format, ##arg)
#else
#define ADUX1020_err(format, arg...)	{if (0); }
#endif
/******************************************************************/

/**
	@brief ADUX1020 Indexes for Gesture sign
*/
typedef enum {
	G_SIGN_INVALID = 0,
	G_SIGN_WE,
	G_SIGN_EW,
	G_SIGN_SN,
	G_SIGN_NS,
	G_SIGN_HOVER = 0x1A,
	G_SIGN_LOCK
} ENUM_GESTURE_INDEX;

/**
	@brief ADUX1020 Profiling Status
*/

struct wrk_q_timestamp {
	struct timeval interrupt_trigger;
};

#ifdef ADUX_STAT
struct adux_stats {
	atomic_t interrupts;
	atomic_t fifo_requires_sync;
	atomic_t fifo_bytes[3];
	atomic_t wq_pending;
	atomic_t wq_schedule_time_peak_usec;
	atomic_t wq_schedule_time_last_usec;
	struct ewma wq_schedule_time_avg_usec;
	atomic_t data_process_time_peak_usec;
	atomic_t data_process_time_last_usec;
	struct ewma data_process_time_avg_usec;
	struct wrk_q_timestamp stamp;
};
#else
#endif

/**
	@brief ADUX1020 data for gesture
*/
struct adux1020_gest_data {
	unsigned dirX:2;
	unsigned dirY:2;
	unsigned dirZ:1;
	unsigned dntcare:3;
	u8 gest_length;
};

/**
	@brief ADUX1020 Gesture Sequence
*/

//#if defined(CONFIG_FB)
struct adux1020_wakeup_seq {
	atomic_t gesture_count;
	atomic_t gesture_seq_val;
	unsigned long prev_entry;
};
//#endif

/**
	@brief ADUX1020 Bus Operations
*/
struct adux_bus_ops {
	s32 (*read)(struct i2c_client *, u16 reg,
		    u32 len, u16 *data, u32 endian);
	s32 (*write)(struct i2c_client *, u16 reg,
		     u16 data, u32 endian);
};

/**
	@brief adux1020 main chip
*/
struct adux1020 {
	struct i2c_client *client;

	struct device *adux_dev;
	struct class *adux_class;

	struct mutex mutex;	/*lock */

	struct adux_platform_data *pdata;
	struct adux_platform_data *runtime_pdata;

	struct input_dev *gest_input;
	struct input_dev *prox_input;

	struct work_struct work;
	struct adux1020_gest_data adux_gest_data;

//#ifdef CONFIG_FB
	struct adux1020_wakeup_seq adux_wq;
	//struct notifier_block adux1020_fb_notif;
	unsigned short wakeup_mode;
//#endif

#ifdef ADUX_STAT
	struct adux_stats stats;
#else
#endif
	unsigned long time;
	s32 irq;
	u32 mode;
	u32 prev_mode;
	u32 endian;

	u32 discard_time;
	u16 chip_id;
	u16 product;
	u16 version;
	u16 curr_intr;
	u16 fifosize;
	u16 fifosize_byte;
	u16 fifo_th;

	u16 r_fifo_prevent;
	u16 r_pack_start;

	u16 prox_on1_th;
	u16 prox_off1_th;
	u16 prox_on2_th;
	u16 prox_off2_th;

	u16 intensity;

	u16 prox_type;
	u16 clk_status;

	u16 *pu16_databuff;
	/*sysfs register read write */
	u16 sysfs_I2C_regaddr;
	u16 sysfs_I2C_regval;
	u16 sysfslastreadval;

	u16 sequence;

	u8 file_name[20];
	bool adux1020_suspend;
	bool adux1020_interrupt;
	bool adux1020_calibrate;
	struct wake_lock  cal_wake_lock;

	s32 (*read)(struct i2c_client *, u16 reg,
		    u32 len, u16 *data, u32 endian);
	s32 (*write)(struct i2c_client *, u16 reg,
		     u16 data, u32 endian);
};
/**
	@brief ADUX1020 data buffer
*/
static u16 databuffer[MAX_BUFFER];
/*************** PLAT DATA ****************************************/
/**
	@struct adux_platform_data
	@brief Platform data for initialization if not provided by the user
*/
static struct adux_platform_data adux1020_default_platform_data = {
	.general_reg_cnt = 54,
	/*Derived from StandardR1.dcfg */
	.general_regs = {
			 0x000c000f, 0x00101010, 0x0011004c, 0x00125f0c,
			 0x0013ada5, 0x00140080, 0x00150000, 0x00160600,
			 0x00170000, 0x00182693, 0x00190004, 0x001a4280,
			 0x001b0060, 0x001c2094, 0x001d0000, 0x001e0001,
			 0x001f0000, 0x00200320, 0x00210513, 0x00220320,
			 0x00230113, 0x00240000, 0x00252414, 0x00262414,
			 0x00270022, 0x00280020, 0x00290300, 0x002a0850,
			 0x002b0700, 0x002c4268, 0x002d2710, 0x002e0000,
			 0x002f0000, 0x00300000, 0x00310000, 0x00320040,
			 0x00330000, 0x0034E400, 0x00388080, 0x00398080,
			 0x003a1ee2, 0x003b1f00, 0x003c1efe, 0x003d1f2d,
			 0x003e0000, 0x0040706d, 0x00411f2f, 0x00424000,
			 0x00430000, 0x00440005, 0x00460030, 0x004800ef,
			 0x00490000, 0x0045000f,
			 },
	.low_power_reg_cnt = 54,
	/*Derived from LowPowerR1.dcfg */
	.low_power_regs = {
			   0x000c000f, 0x00101010, 0x0011004c, 0x00125f0c,
			   0x0013ada5, 0x00140080, 0x00150000, 0x00160600,
			   0x00170000, 0x00182693, 0x00190004, 0x001a4280,
			   0x001b0060, 0x001c2094, 0x001d0000, 0x001e0001,
			   0x001f0000, 0x00200220, 0x00210311, 0x00220220,
			   0x00230111, 0x00240000, 0x00251c15, 0x00261c15,
			   0x00270022, 0x00280020, 0x00290300, 0x002a0850,
			   0x002b0700, 0x002c4268, 0x002d2710, 0x002e0000,
			   0x002f0000, 0x00300000, 0x00310000, 0x00320040,
			   0x00330000, 0x0034E400, 0x00388080, 0x00398080,
			   0x003a1e34, 0x003b1e49, 0x003c1e48, 0x003d1e69,
			   0x003e0000, 0x0040706c, 0x00411f2f, 0x00424000,
			   0x00430000, 0x00440003, 0x00460030, 0x004800ef,
			   0x00490000, 0x0045000f,
			   },
};

/******************************************************************/
/*I2C_INLINE - START **********************************************/
/**
@brief This API is used as an wrapper to call the I2C read
*       @param pst_adux the ADUX device pointer
*	@param reg 8 bit address to be read
*       @return u16
*/
static inline u16
adux1020_read(struct adux1020 *pst_adux, u16 reg)
{
	u16 value;
	s32 ret;
	ret = pst_adux->read(pst_adux->client,
			     reg, WORD_SIZE, &value, pst_adux->endian);
	if (ret < FAILED)
		value = 0xFFFF;
	return value;
}

/**
@brief This API is used as an wrapper to call the multi I2C read
*       @param pst_adux the ADUX device pointer
*       @param first_reg 8 bit register address to be read
*	@param count Number of data to be read
*	@param buf Buffer to place the read data
*       @return int
*/
static inline int
adux1020_multi_read(struct adux1020 *pst_adux,
		    u16 first_reg, u16 count, u16 *buf)
{
	return pst_adux->read(pst_adux->client,
			      first_reg, count, buf, pst_adux->endian);
}

/**
@brief This API is used as an wrapper to call the I2C write
*	@param pst_adux the ADUX device pointer
*	@param reg 8 bit register address
*	@param val 16 bit value to be written
*       @return int
*/
static inline int
adux1020_write(struct adux1020 *pst_adux, u16 reg, u16 val)
{
	return pst_adux->write(pst_adux->client, reg, val, pst_adux->endian);
}

/*I2C_INLINE - END ************************************************/
/*FILP API - START ************************************************/
/**
@brief This API is used to store configuration from user
*       @param pst_adux the ADUX device pointer
*	@param config
*       @return void
*/
void
adux_reg_store(struct adux1020 *pst_adux, u16 config)
{
	u32 value = 0;
	u16 i = 0;
	u16 read_reg = 0;

	for (i = 0; i <= 0x49; i++) {
		if (!(i == 0x45 || i == 0x48 || i == 0x49)) {
			read_reg = adux1020_read(pst_adux, i);
			value = (i << 16) | (read_reg & 0xFFFF);
			switch (config) {
			case CONFIG(GENERAL):
				pst_adux->pdata->general_regs[i] = (u32) value;
				pst_adux->pdata->general_reg_cnt = i + 1;
				break;
			case CONFIG(LOW_POWER):
				pst_adux->pdata->low_power_regs[i] =
				    (u32) value;
				pst_adux->pdata->low_power_reg_cnt = i + 1;
				break;
			default:
				ADUX1020_dbg("INVALID INPUT\n");
				break;
			};
			value = 0;
		}
	}
}

/**
@brief This API is used to retrieve configuration from user
*       @param pst_adux the ADUX device pointer
*	@param config
*       @return void
*/
void
adux_reg_retrieve(struct adux1020 *pst_adux, u16 config)
{
	u32 *ptr_reg = NULL;
	u32 count = 0;

	u16 i = 0;
	u16 reg = 0;
	u16 reg_value = 0;
	u16 err = 0;

	switch (config) {
	case CONFIG(GENERAL):
		ptr_reg = pst_adux->pdata->general_regs;
		count = pst_adux->pdata->general_reg_cnt;
		ADUX1020_dbg("G - cnt %d\n", count);
		break;
	case CONFIG(LOW_POWER):
		ptr_reg = pst_adux->pdata->low_power_regs;
		count = pst_adux->pdata->low_power_reg_cnt;
		ADUX1020_dbg("LP - cnt %d\n", count);
		break;
	default:
		ADUX1020_dbg("INVALID INPUT\n");
		err = 1;
		break;
	};

	if (!err) {
		for (i = 0; i < count; i++) {
			reg = (ptr_reg[i] >> 16);
			reg_value = (ptr_reg[i] & 0xFFFF);
			if (!(reg == 0x45 || reg == 0x48 || reg == 0x49)) {
				ADUX1020_dbg("Write Reg [%x] = 0x%x\n", reg,
					     reg_value);
				adux1020_write(pst_adux, reg, reg_value);
				reg_value = 0;
			}
		}
	}

}

/**
This function is used to parse string
*	@param recv_buf pointer point a string
*	@return int
*/
static int
parse_data(char *recv_buf)
{
	char **bp = (char **)&recv_buf;
	char *token, parsing_cnt = 0;
	long val;
	int test_data;
	unsigned int data = 0;
	int ret = 0;
	char test[10] = {'\0'};

	while (parsing_cnt < 2) {
		token = strsep(bp, " \t");
		if (parsing_cnt == 1)
			token[4] = '\0';
		memset(test, '\0', 10);
		memcpy(test, token, strlen(token));
		memmove(test+2, test, strlen(test));
		test[0] = '0';
		test[1] = 'x';
		ret = kstrtol(test, 0, &val);
		if (ret) {
			if (ret  == -ERANGE) {
				ADUX1020_info("out of range\n");
				val = 0;
			}
			if (ret ==  -EINVAL) {
				ADUX1020_info("parsing error\n");
				sscanf(test, "%x", &test_data);
				val = test_data;
			}
		}
		if (parsing_cnt == 0) {
			data = (int) (0xFF & val);
			if (data == 0)
				return -1;
		}
		if (parsing_cnt == 1) {
			data = data << 16;
			data |= (0xFFFF & val);
		}
		parsing_cnt++;
		/*if (parsing_cnt >= 2)
			break;	*/
	}
	return data;
}

/**
This function is used for reading configuration file
*	@param filename hold the string pointer of the file location
*	@param pst_adux the ADUX1022 data structure
*	@param config configuration option.
*	@return int status of the configuration file.
*/
static int
adux_filp_start_calib(s8 *filename, struct adux1020 *pst_adux, u16 config)
{
	mm_segment_t old_fs;
	struct file *fpt_adux = NULL;
	struct adux_platform_data *config_reg = NULL;
	int start = 0;
	int cnt = 0;
	int ret = 0;
	int count = 0;
	char get_char;
	char *recieved_buf = NULL;
	loff_t pos = 0;
	config_reg = pst_adux->runtime_pdata;
	ADUX1020_dbg("Config from %s\n", filename);
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fpt_adux = filp_open(filename, O_RDONLY, 0666);
	if (IS_ERR(fpt_adux)) {
		ADUX1020_dbg("unable to find de file %ld\n", PTR_ERR(fpt_adux));
		set_fs(old_fs);
		ret = /*PTR_ERR(fpt_adux_filp); */ -1;
		goto err_filp_open;
	}

	recieved_buf = kzalloc(sizeof(char) * 15, GFP_KERNEL);
	memset(recieved_buf, '\0', sizeof(recieved_buf));

	while (vfs_read(fpt_adux, &get_char, sizeof(char), &pos)) {
		if (get_char == '\n') {
			if (cnt > 1) {
				ret = parse_data(recieved_buf);
				if (ret == -1) {
					ADUX1020_dbg("Error in reading dcfg\n");
					break;
				}
				/*ADUX1020_info */
				ADUX1020_info("0x%08x\n", ret);
				switch (config) {
				case CONFIG(GENERAL):
					{
						config_reg->general_regs[count]
							= (u32) ret;
						config_reg->general_reg_cnt
							= count + 1;
					}
					break;
				case CONFIG(LOW_POWER):
					{
						config_reg->
							low_power_regs[count]
								= (u32) ret;
						config_reg->low_power_reg_cnt
							= count + 1;
					}
					break;
				default:
					{
						ADUX1020_info(
							"INVALID INPUT\n");
					}
					break;
				}
				count++;
			}
			memset(recieved_buf, '\0', sizeof(recieved_buf));
			start = 0;
			cnt = 0;
			ret = 0;
		} else {
			if ((get_char != '#') & (get_char != '/')) {
				if (start != 0xF) {
					recieved_buf[cnt] = get_char;
					cnt++;
				}
			} else {
				start = 0xF;
			}
		}
	}

	filp_close(fpt_adux, NULL);

	set_fs(old_fs);
	kfree(recieved_buf);
	return 0;
err_filp_open:
	return -1;
}

/*FILP API - END **************************************************/
/*COMMON_API - START **********************************************/
/**	@fn cmd_parsing (const char *buf,
	unsigned short cnt, unsigned short *data)
*	@brief This API is used to convert the buf(string)
		into integer or hexadecimal value.
*	@param buf stores the pointer of the string
*	@param cnt store the number of value need to retrive
*	@param data store the pointer of the value derived from the string
*	@return int status of the derived count from the string
*/
static int
cmd_parsing(const char *buf, unsigned short cnt, unsigned short *data)
{

	char **bp = (char **)&buf;
	char *token, minus, parsing_cnt = 0;
	int val = 0;
	int pos = 0;
	int arg_cnt = 0;

	while ((token = strsep(bp, " "))) {
		pos = 0;
		minus = false;
		if ((char) token[pos] == '-') {
			minus = true;
			pos++;
		}
		if ((token[pos] == '0') &&
		    (token[pos + 1] == 'x' || token[pos + 1] == 'X')) {
			if (kstrtoul(&token[pos + 2],
				     16, (long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		} else {
			if (kstrtoul(&token[pos],
				     10, (long unsigned int *)&val))
				val = 0;
			if (minus)
				val *= (-1);
		}

		if (parsing_cnt < cnt) {
			*(data + parsing_cnt) = val;
			arg_cnt++;
		} else {
			break;
		}
		parsing_cnt++;
	}
	return arg_cnt;
}

/*COMMON_API - END ************************************************/
/*INTERRUPT MONITOR - START ***************************************/
/**
This function is used for disabling the irq of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structur
@return void
*/
static void
adux1020_disable_irq(struct adux1020 *pst_adux)
{
	/*if adux1020_interrupt is 0, then interrupt is been disable already
	   therefore it is unneccessary to disable again */
	if (pst_adux->adux1020_interrupt != 0) {
		pst_adux->adux1020_interrupt = 0;
		disable_irq(pst_adux->irq);
	}
}

/**
This function is used for enabling the irq of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structur
@return void
*/
static void
adux1020_enable_irq(struct adux1020 *pst_adux)
{
	if (pst_adux->adux1020_interrupt != 1) {
		pst_adux->adux1020_interrupt = 1;
		enable_irq(pst_adux->irq);
	}
}

/*INTERRUPT MONITOR - END *****************************************/

/*FIFO BUFFER READ - START ****************************************/
/**
This function is used for  setting the force calibration of  the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  s32 return value
*/
static s32
adux1020_set_force_clock(struct adux1020 *pst_adux)
{
	s32 ret;

	/*Andre:  FORCE_CLOCK_ON - 0xF4F */
	ret = adux1020_write(pst_adux, ADUX_TEST_MODES_3, FORCE_CLOCK_ON);
	return ret;
}

/**
This function is used to reset the force clock of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 reset force clock status of the ADUX1020 chip
*/
static s32
adux1020_reset_force_clock(struct adux1020 *pst_adux)
{
	s32 ret;

	ret = adux1020_write(pst_adux, ADUX_TEST_MODES_3, FORCE_CLOCK_RESET);
	return ret;
}

/**
This function is used to read the data of the fifo of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 fifo data of the ADUX1020 chip
*/
static s32
adux1020_fifo_data(struct adux1020 *pst_adux)
{
	s32 ret;
	/* Force Clock On */
	adux1020_set_force_clock(pst_adux);

	if (pst_adux->fifosize_byte) {
		ret = adux1020_multi_read(pst_adux,
					  ADUX_DATA_BUFFER,
					  (pst_adux->fifosize +
					   (1 & (pst_adux->fifosize_byte))),
					  pst_adux->pu16_databuff);
		if (ret < 0) {
			ADUX1020_err(" %s read error -%d\n", __func__, ret);
			return FAILED;
		}
	}

	/* Clock ctrl by Internal state machine */
	adux1020_reset_force_clock(pst_adux);

	return SUCCESS;
}

/*FIFO BUFFER READ - END ******************************************/
/*GESTURE ONCHIP ALGORITHM - START ********************************/
/**
This function is used for the Gesture Sign for the Gesture
utility with N-E-W-S data
@param pst_adux the ADUX device pointer.
@return Gesture index value
*/
static ENUM_GESTURE_INDEX
adux1020_get_gesture_sign(struct adux1020 *pst_adux)
{
	ENUM_GESTURE_INDEX gest_index;

	u8 DirX = pst_adux->adux_gest_data.dirX;
	u8 DirY = pst_adux->adux_gest_data.dirY;
	u8 DirZ = pst_adux->adux_gest_data.dirZ;

	/** need clarification */

	ADUX1020_dbg("DirX = %d , DirY = %d , DirZ = %d\n", DirX, DirY, DirZ);
	if ((DirX == 1) && (DirY == 0) && (DirZ == 0)) {
		ADUX1020_dbg("GESTURE DIRECTION - East to West\n");
		gest_index = G_SIGN_EW;
	} else if ((DirX == 2) && (DirY == 0) && (DirZ == 0)) {
		ADUX1020_dbg("GESTURE DIRECTION - West to East\n");
		gest_index = G_SIGN_WE;
	} else if ((DirX == 0) && (DirY == 1) && (DirZ == 0)) {
		ADUX1020_dbg("GESTURE DIRECTION - South to North\n");
		gest_index = G_SIGN_SN;
	} else if ((DirX == 0) && (DirY == 2) && (DirZ == 0)) {
		ADUX1020_dbg("GESTURE DIRECTION - North to South\n");
		gest_index = G_SIGN_NS;
	} else if ((DirX == 0) && (DirY == 0) && (DirZ == 1)) {
		ADUX1020_dbg("GESTURE DIRECTION - HOVER\n");
		gest_index = G_SIGN_HOVER;
	} else {
		ADUX1020_dbg("GESTURE DIRECTION INVALID!!\n");
		gest_index = G_SIGN_INVALID;
	}

	ADUX1020_dbg("gest_index = %d\n", gest_index);
	return gest_index;
}

/*GESTURE ONCHIP ALGORITHM - END **********************************/
/*INPUT_DEVICE - START ********************************************/
/**
This function is used to send Gesture proximity data throught event interface
@param prox_inputdev the ADUX input device pointer.
@return void
*/
static void
adux1020_proxi_evt(struct input_dev *prox_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(prox_inputdev);
	u16 cnt_i;
	u16 cnt_j;

	for (cnt_i = 0; cnt_i < pst_adux->fifosize; cnt_i += PROX_I_BUF_SIZE)
		for (cnt_j = 0; cnt_j < PROX_I_BUF_SIZE; cnt_j++) {
			input_event(prox_inputdev,
				    EV_MSC,
				    MSC_RAW,
				    (int) *(pst_adux->pu16_databuff +
					    cnt_j + cnt_i));
			/*Last stored Intensity value */
			pst_adux->intensity =
			    (int) *(pst_adux->pu16_databuff + cnt_j + cnt_i);
			input_sync(prox_inputdev);
		}
}


/**
This function is used to send Gesture proximity data throught event interface
@param gest_inputdev the ADUX input device pointer.
@return void
*/
static void
adux1020_gest_inputevent(struct input_dev *gest_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(gest_inputdev);
	u16 cnt_i;
	u16 cnt_j;

	for (cnt_i = 0; cnt_i < pst_adux->fifosize; cnt_i += GEST_BUF_SIZE) {
		for (cnt_j = 0; cnt_j < GEST_BUF_SIZE; cnt_j++) {
			ADUX1020_info("Gesture data 0x%x\t",
				      (int) *(pst_adux->pu16_databuff + cnt_j +
					      cnt_i));

			input_event(gest_inputdev,
				    EV_MSC,
				    MSC_GESTURE,
				    (int) *(pst_adux->pu16_databuff +
					    cnt_j + cnt_i));
		}
		input_sync(gest_inputdev);
	}

}

/**
This function is is used to find out gesture sequence in driver itself
@param pst_adux pointer to ADUX1020 chip structure
@param gest_index the Gesture index value
@return void
*/
static void
adux1020_gest_seq(struct adux1020 *pst_adux, ENUM_GESTURE_INDEX gest_index)
{
	ADUX1020_dbg(" %s\n", __func__);
	if (atomic_read(&pst_adux->adux_wq.gesture_count) == 1) {
		if (jiffies_to_msecs(jiffies -
				     pst_adux->adux_wq.prev_entry)
		>= SLEEP) {
			ADUX1020_info("********RESET\n");
			atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
			atomic_set(&pst_adux->adux_wq.gesture_count, 0);
			pst_adux->adux_wq.prev_entry = 0;
		}
	}
	if (gest_index == 1) {
		if (atomic_read(&pst_adux->adux_wq.gesture_count) == 0) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_seq_val,
				   gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_count, 1);
			pst_adux->adux_wq.prev_entry = jiffies;
		} else if (atomic_read(&pst_adux->adux_wq.gesture_seq_val)
				== 2) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			ADUX1020_dbg("gesture matched\n");
			atomic_set(&pst_adux->adux_wq.gesture_count, 0xF);
		} else {
			atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
			atomic_set(&pst_adux->adux_wq.gesture_count, 0);
		}
	} else if (gest_index == 2) {
		if (atomic_read(&pst_adux->adux_wq.gesture_count) == 0) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_seq_val,
				   gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_count, 1);
			pst_adux->adux_wq.prev_entry = jiffies;
		} else if (atomic_read(&pst_adux->adux_wq.gesture_seq_val)
				== 1) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			ADUX1020_dbg("gesture matched\n");
			atomic_set(&pst_adux->adux_wq.gesture_count, 0xF);
		} else {
			atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
			atomic_set(&pst_adux->adux_wq.gesture_count, 0);
		}
	} else if (gest_index == 3) {
		if (atomic_read(&pst_adux->adux_wq.gesture_count) == 0) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_seq_val,
				   gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_count, 1);
			pst_adux->adux_wq.prev_entry = jiffies;
		} else if (atomic_read(&pst_adux->adux_wq.gesture_seq_val)
				== 4) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			ADUX1020_dbg("gesture matched\n");
			atomic_set(&pst_adux->adux_wq.gesture_count, 0xF);
		} else {
			atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
			atomic_set(&pst_adux->adux_wq.gesture_count, 0);
		}
	} else if (gest_index == 4) {
		if (atomic_read(&pst_adux->adux_wq.gesture_count) == 0) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_seq_val,
				   gest_index);
			atomic_set(&pst_adux->adux_wq.gesture_count, 1);
			pst_adux->adux_wq.prev_entry = jiffies;
		} else if (atomic_read(&pst_adux->adux_wq.gesture_seq_val)
				== 3) {
			ADUX1020_dbg("Gesture index - %d\n", gest_index);
			ADUX1020_dbg("gesture matched\n");
			atomic_set(&pst_adux->adux_wq.gesture_count, 0xF);
		} else {
			atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
			atomic_set(&pst_adux->adux_wq.gesture_count, 0);
		}
	} else {

	}
	if (atomic_read(&pst_adux->adux_wq.gesture_count) == 0xF) {
		ADUX1020_dbg("Gesture Sequence matched Success\n");
		atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
		atomic_set(&pst_adux->adux_wq.gesture_count, 0);
		pst_adux->adux_wq.prev_entry = 0;
		input_event(pst_adux->gest_input, EV_KEY, WAKEUP_KEY, 1);
		input_sync(pst_adux->gest_input);

		input_event(pst_adux->gest_input, EV_KEY, WAKEUP_KEY, 0);
		input_sync(pst_adux->gest_input);

		adux1020_write(pst_adux, ADUX_INT_MASK_ADDR,
			       SET_INTR_MASK_VALUE(IDLE));
		adux1020_write(pst_adux, ADUX_OP_MODE_ADDR,
			       SET_MODE_VALUE(IDLE, NONE));
		 /*unknown */
		atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
	}
}

/**
This function is used to send the Gesture keyevent data throught event interface
@param pst_adux the ADUX device pointer.
@return void
*/
static void
adux1020_gest_evt(struct adux1020 *pst_adux)
{
	u32 gest_index = 0;

	gest_index = adux1020_get_gesture_sign(pst_adux);
	if (pst_adux->adux1020_suspend) {
		ADUX1020_dbg("Gesture Sequence is ON\n");
		adux1020_gest_seq(pst_adux, gest_index);
	} else {
		adux1020_gest_inputevent(pst_adux->gest_input);
	}

}

/**
This function is used to send the RAW data throught event interface
@param sample_inputdev ADUX input device pointer.
@return void
*/
static void
adux1020_sample_evt(struct input_dev *sample_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(sample_inputdev);
	u16 cnt_i;
	u16 cnt_j;
	s32 event_value1 = 0;
	s32 event_value2 = 0;

	switch (pst_adux->mode) {
	case SAMPLE_RAW:
		{
			for (cnt_i = 0;
			     cnt_i < pst_adux->fifosize;
			     cnt_i += SAMPLE_RAW_BUF_SIZE) {
				for (cnt_j = 0;
				     cnt_j < SAMPLE_RAW_BUF_SIZE; cnt_j++) {
					if (cnt_j < 2) {
						event_value1 =
						    event_value1 << 16;
						event_value1 = event_value1 |
						    ((int)
						     *(pst_adux->pu16_databuff +
						       cnt_i + cnt_j));

					} else {
						event_value2 =
						    event_value2 << 16;
						event_value2 = event_value2 |
						    ((int)
						     *(pst_adux->pu16_databuff +
						       cnt_i + cnt_j));
					}
				}
				if (pst_adux->discard_time == 0) {
					input_event(sample_inputdev,
						    EV_MSC, MSC_RAW,
						    event_value1);
					input_event(sample_inputdev, EV_MSC,
						    MSC_RAW, event_value2);
					input_sync(sample_inputdev);
					event_value1 = 0;
					event_value2 = 0;
				} else {
					ADUX1020_info("Mis Fired samples\n");
					pst_adux->discard_time =
					    pst_adux->discard_time - 1;
				}
			}
		}
		break;
	default:
		ADUX1020_dbg("Invalid Sample mode\n");
		break;
	}
}

/**
This function is used for the input allocation of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@param dev The device structure for the input device.
@param inputdev_name the name of the input device.
@return input_dev the input_dev device allocated
*/
static struct input_dev *
adux1020_inputdev_alloc(struct adux1020 *pst_adux,
			struct device *dev, s8 *inputdev_name)
{
	struct input_dev *pst_inputdev;
	s32 error;
	ADUX1020_info(" %s\n", __func__);
	/* Input device Registration for gesture gesture_raw and proximity */

	pst_inputdev = input_allocate_device();
	if (!pst_inputdev) {
		error = -ENOMEM;
		ADUX1020_err(" %s Error -%s\n", __func__, inputdev_name);
		return ERR_PTR(error);
	}

	pst_inputdev->id.bustype = BUS_I2C;
	pst_inputdev->id.product = pst_adux->product;
	pst_inputdev->id.version = pst_adux->version;
	pst_inputdev->name = inputdev_name;
	/*pst_inputdev->dev.parent = pst_adux->adux_dev; */
	input_set_drvdata(pst_inputdev, pst_adux);

	return pst_inputdev;
}

/**
This function is used for the registering of the input capabilities and
registration of the input device of the ADUX1020 chip for gesture mode
@param pst_inputdev the input device pointer to open.
@return s32
*/
static s32
adux1020_gesture_inputdev_reg(struct input_dev *pst_inputdev)
{
	s32 error;

	ADUX1020_info(" %s\n", __func__);

	 /**************** register input device for gesture *****************/
	__set_bit(EV_MSC, pst_inputdev->evbit);
	__set_bit(EV_KEY, pst_inputdev->evbit);

	__set_bit(MSC_GESTURE, pst_inputdev->mscbit);	/* set Raw bit */
	__set_bit(MSC_RAW, pst_inputdev->mscbit);	/* set Raw bit */
	__set_bit(WAKEUP_KEY, pst_inputdev->keybit);	/* set Wakeup key bit*/

	error = input_register_device(pst_inputdev);
	if (error < 0) {
		ADUX1020_dbg(" %s registered failed", pst_inputdev->name);
		return error;
	}
	return 0;
}

/**
This function is used to deregister the input device of the ADUX1020 chip
@param pst_inputdev the input device pointer to unregister
@return void
*/
static void
adux1020_gesture_inputdev_unreg(struct input_dev *pst_inputdev)
{
	ADUX1020_info(" %s\n", __func__);
	input_unregister_device(pst_inputdev);
}

/**
This function is used to Register the input device
to proximity mode of the ADUX1020 chip
@param pst_inputdev the input device pointer to close.
@return s32
*/
static s32
adux1020_proximity_inputdev_reg(struct input_dev *pst_inputdev)
{
	s32 error;

	ADUX1020_info(" %s\n", __func__);
	/*************** register input device for proximity ****************/
	__set_bit(EV_ABS, pst_inputdev->evbit);
	__set_bit(EV_MSC, pst_inputdev->evbit);

	__set_bit(MSC_RAW, pst_inputdev->mscbit);	/* set RAW bit */

	__set_bit(ABS_DISTANCE, pst_inputdev->absbit);

	input_set_abs_params(pst_inputdev, ABS_DISTANCE, MIN_DISTANCE,
			     MAX_DISTANCE, 0, 0);

	error = input_register_device(pst_inputdev);
	if (error < 0) {
		ADUX1020_dbg(" %s registered failed", pst_inputdev->name);
		return error;
	}
	input_event(pst_inputdev, EV_ABS, ABS_DISTANCE, UNKNOWN);
	input_sync(pst_inputdev);

	return 0;
}

/**
This function is used to unregister the input device
from proximity mode of the ADUX1020 chip
@param pst_inputdev the input device pointer to close.
@return void
*/
static void
adux1020_proximity_inputdev_unreg(struct input_dev *pst_inputdev)
{
	ADUX1020_info(" %s\n", __func__);
	input_unregister_device(pst_inputdev);
}

/*INPUT_DEVICE -END ***********************************************/

/*ADUX DRIVER - API **********************************************/
/**
This function is used to flush buffer the Adux1020 chip.
@param pst_adux ADUX device pointer.
@return void
*/
static void
adux1020_flush_buffer(struct adux1020 *pst_adux)
{
	ADUX1020_info(" %s\n", __func__);
	memset(pst_adux->pu16_databuff, 0, MAX_BUFFER);
}

/**
This function is used for  reset the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_reset_device(struct adux1020 *pst_adux)
{
	u16 resetdevice;
	u16 retrycnt = MAX_RETRY;

	ADUX1020_info(" %s\n", __func__);
	/*/set ADUX SW RESET bit [0] */
	adux1020_write(pst_adux, ADUX_SW_RESET, (u16) SW_RESET_SET);
	do {
		/*wait for transition to get completed */
		/*      udelay_range(10000, 10000);*/
		msleep(20);
		resetdevice = adux1020_read(pst_adux, ADUX_SW_RESET);
		retrycnt--;
	} while (retrycnt && (resetdevice != 0x0));

}

/**
This function is used to set the I2c speed of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_set_i2c_speed(struct adux1020 *pst_adux)
{
	u16 i2cspeed;
	u16 val;

	ADUX1020_info(" %s\n", __func__);
	val = adux1020_read(pst_adux, ADUX_I2C_1);

	i2cspeed = (val & I2C_SPEED_MASK) >> 12;

	ADUX1020_info("I2c Speed 0x%x\n", i2cspeed);
	i2cspeed = val | I2C_SPEED_MASK;
	ADUX1020_dbg("%s write value 0x%x\n", __func__, i2cspeed);
	adux1020_write(pst_adux, ADUX_I2C_1, i2cspeed);

	val = adux1020_read(pst_adux, ADUX_I2C_1);
	i2cspeed = (val & I2C_SPEED_MASK) >> 12;
	ADUX1020_dbg("I2c Speed 0x%x\n", i2cspeed);
}

/**
This function is used for  read the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return void
*/
static void
adux1020_read_chip(struct adux1020 *pst_adux)
{
	u16 checkdevice = 0;
	ADUX1020_info(" %s\n", __func__);
	checkdevice = adux1020_read(pst_adux, ADUX_CHIP_ID_VERSION_ADDR);
	pst_adux->chip_id = checkdevice & CHIP_ID_MASK;
	pst_adux->version = GET_VERSION(checkdevice);
}

/**
This function is used to update the threshold for fifo of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return u16 the updated threshold value
*/
static u16
adux1020_update_fifo_th(struct adux1020 *pst_adux)
{
	u16 fifo_th;

	fifo_th = adux1020_read(pst_adux, ADUX_I2C_2);
	fifo_th = (fifo_th & 0x0F00) >> 8;
	return fifo_th;
}

/**
This function is used for clearing the interrupt status of the ADUX1020 chip
@param pst_adux pointer to  Adux1020 chip structure
@return s32
*/
static s32
adux1020_clr_intr_status(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 clear;
	clear = pst_adux->curr_intr;
	ret = adux1020_write(pst_adux, ADUX_INT_STATUS_ADDR, clear);
	return ret;
}

/**
This function is used to read the interrupt status of the fifo
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 interrupt status of the ADUX1020 chip
*/
static s32
adux1020_rd_intr_fifo(struct adux1020 *pst_adux)
{
	u16 status;
#ifdef ADUX_STAT
	unsigned int mask = 0;
	unsigned int mod = 0;
#else
#endif
	status = adux1020_read(pst_adux, ADUX_INT_STATUS_ADDR);
	pst_adux->fifosize_byte = ((status & FIFO_SIZE) >> 8);
	pst_adux->fifosize =
	    (((status & FIFO_SIZE) >> 8) / ADUX1020_REG_SIZE_BYTES);
	switch (pst_adux->mode) {
	case PROXIMITY_I:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(PROXIMITY);
#ifdef ADUX_STAT
		atomic_set(&pst_adux->stats.fifo_requires_sync, 0);
		mod = 0;
#else
#endif
		break;
	case GESTURE:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(GESTURE);
#ifdef ADUX_STAT
		mask = 0x01;
		mod = pst_adux->fifosize & mask;
#else
#endif
		break;
	case SAMPLE_RAW:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(SAMPLE);
#ifdef ADUX_STAT
		mask = 0x03;
		mod = pst_adux->fifosize & mask;
#else
#endif
		break;
	case IDLE:
	case OFF:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(GENERAL);
		break;
	case FIFO:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(FIFO);
		break;
	default:
		pst_adux->curr_intr = status & INTR_STATUS_MASK(WATCHDOG);
#ifdef ADUX_STAT
		atomic_set(&pst_adux->stats.fifo_requires_sync, 0);
		mod = 0;
#else
#endif
		break;
	};

#ifdef ADUX_STAT
	if (pst_adux->fifosize <= 16)
		atomic_inc(&pst_adux->stats.fifo_bytes[0]);
	else if (pst_adux->fifosize <= 32)
		atomic_inc(&pst_adux->stats.fifo_bytes[1]);
	else
		atomic_inc(&pst_adux->stats.fifo_bytes[2]);
	if (mod) {
		ADUX1020_info("Keeping %d samples in FIFO from %d\n",
			      mod, pst_adux->fifosize);
		pst_adux->fifosize &= ~mask;
		atomic_inc(&pst_adux->stats.fifo_requires_sync);
	}

#else
#endif
	return SUCCESS;
}

/**
This function is used for clearing the interrupt fifo of the ADUX1020 chip
@param pst_adux pointer to  Adux1020 chip structure
@return s32
*/
static s32
adux1020_clr_intr_fifo(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 clear;
	u16 loopcnt;

	do {
		adux1020_rd_intr_fifo(pst_adux);

		clear = (pst_adux->curr_intr) | SELF_RESETTING;

		ADUX1020_dbg("FIFO size during SWITCH %x\n",
			     pst_adux->fifosize);
		ADUX1020_dbg("FS byte %x\n", pst_adux->fifosize_byte);
		if (pst_adux->fifosize_byte) {
			adux1020_write(pst_adux,
				       ADUX_TEST_MODES_3, FORCE_CLOCK_ON);
			adux1020_multi_read(pst_adux,
					    ADUX_DATA_BUFFER,
					    (pst_adux->fifosize +
					     (1 & (pst_adux->fifosize_byte))),
					    pst_adux->pu16_databuff);
			for (loopcnt = 0; loopcnt < pst_adux->fifosize;
			     loopcnt++)
				ADUX1020_info("0x%x\n",
					      pst_adux->pu16_databuff[loopcnt]);

			/*1p3 - clear status is done before force clock reset */
			ret = adux1020_write(pst_adux,
					     ADUX_INT_STATUS_ADDR, clear);
			adux1020_write(pst_adux,
				       ADUX_TEST_MODES_3, FORCE_CLOCK_RESET);
		}
	} while (pst_adux->fifosize != 0);
	adux1020_flush_buffer(pst_adux);
	return ret;
}

/**
This function is used to enter idle mode of the Adux1020
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_enter_idle(struct adux1020 *pst_adux)
{
	s32 ret;
	u16 mode_val = 0;
	u16 mask_val = 0;
	ADUX1020_info("%s\n", __func__);

	/*1p3 - Work around for switching to Idle_mode */
	/*1p3 - set R_FORCE_MODE[0x33] = 0x08 */
	adux1020_write(pst_adux, ADUX_FORCE_MODE_ADDR, 0x08);
	/*1p3 - set R_OP_MODE[0x45] = 0xE -- set operating mode as force mode */
	adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, 0x0E);
	/*1p3 - set R_OP_MODE[0x45] = 0x0 -- set operating mode as OFF mode */
	adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, 0x00);

	mode_val = SET_MODE_VALUE(IDLE, NONE);
	mask_val = SET_INTR_MASK_VALUE(IDLE);
	ret = adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, mode_val);

	if (ret < 0) {
		ADUX1020_err("%s write error -%d\n", __func__, ret);
		return FAILED;
	}

/*	pst_adux->mode = IDLE;*/

	ret = adux1020_write(pst_adux, ADUX_INT_MASK_ADDR, mask_val);

	if (ret < 0) {
		ADUX1020_err("%s write error -%d\n", __func__, ret);
		return FAILED;
	}

	/*  Clear IRQ Status */
	adux1020_clr_intr_fifo(pst_adux);

	return SUCCESS;
}

static int get_calibrate_value(struct adux1020 *pst_adux)
{
	struct SensorCalInfo cal_info[2];
	unsigned short cal_val[6];
	unsigned short reg_map[6] = {0x3A, 0x3B, 0x3c, 0x3D, 0x1A, 0x18};
	int i, k;

	wake_lock(&pst_adux->cal_wake_lock);
	//read from yulong param
	if ((sizeof(cal_info[0]) != sensparams_read_from_flash(SENSPARAMS_TYPE_PROX,	\
		(unsigned char *)&cal_info[0], sizeof(cal_info[0])))) {
		printk(KERN_ERR "ADUX1020:YLLOG:1:get prox calibrate value failed.\n");
		goto exit;
	}
	if ((sizeof(cal_info[1]) != sensparams_read_from_flash(SENSPARAMS_TYPE_RESV1,	\
		(unsigned char *)&cal_info[1], sizeof(cal_info[1])))) {
		printk(KERN_ERR "ADUX1020:YLLOG:2:get prox calibrate value failed.\n");
		goto exit;
	}

	if ( (!cal_info[0].SensorCalStatus) || (!cal_info[1].SensorCalStatus) ) {
		printk(KERN_ERR "ADUX1020:YLLOG:Invalid calibrate data.\n");
		goto exit;
	}

	//parse calibrate value & write to relative regs
	i = 0;
	//for (j = 0; j < 2; j++) {
		for (k = 0; k < 6; k += 2) {
			cal_val[i] =
				((unsigned short)cal_info[0].value[k+1] << 8) | \
				((unsigned short)cal_info[0].value[k]);
			i++;
		}
		for (k = 0; k < 6; k += 2) {
			cal_val[i] =
				((unsigned short)cal_info[1].value[k+1] << 8) | \
				((unsigned short)cal_info[1].value[k]);
			i++;
		}
	//}

	for (i = 0; i < 6; i++) {
		adux1020_write(pst_adux, reg_map[i], cal_val[i]);
		printk(KERN_ERR "ADUX1020:YLLOG===0x%x====\n", cal_val[i]);
	}

	pst_adux->adux1020_calibrate = 1;
	wake_unlock(&pst_adux->cal_wake_lock);

	return 0;
exit:
	wake_unlock(&pst_adux->cal_wake_lock);
	return -1;

}

/**
 *@brief This API is used to multi write
 *@param pst_adux the device structure
 *@param mode the operational mode.
 *@return u16 short int value of the status.
*/
static u16
adux1020_mode_switching(struct adux1020 *pst_adux, u16 mode)
{
	s32 ret = 1;
	u16 mode_val = 0;
	u16 intr_mask_val = 0;
	u16 read_reg = 0;
	u32 discard_time = 0;

	mutex_lock(&pst_adux->mutex);
	adux1020_disable_irq(pst_adux);
	/*enter to idle mode to avoid unnecessary intr */
	adux1020_enter_idle(pst_adux);
	pst_adux->discard_time = 0;
	switch (mode) {
	case PROXIMITY_I:
		read_reg = adux1020_read(pst_adux, ADUX_FREQUENCY);
		read_reg = (read_reg & 0xFFF) | 0x8000;
		adux1020_write(pst_adux, ADUX_FREQUENCY, read_reg);
		adux1020_write(pst_adux, ADUX_GEST_DI_TH, 0);
		mode_val = SET_MODE_VALUE(PROXIMITY, I);
		intr_mask_val = SET_INTR_MASK_VALUE(PROXIMITY);

		if (!pst_adux->adux1020_calibrate) {
			if (get_calibrate_value(pst_adux))
				printk(KERN_ERR "ADUX1020:YLLOG:calibrate failed in mode switch.\n");
		}
		adux1020_write(pst_adux, 0x2A, pst_adux->prox_on1_th);
		adux1020_write(pst_adux, 0x2B, pst_adux->prox_off1_th);
		adux1020_write(pst_adux, 0x2C, pst_adux->prox_on2_th);
		adux1020_write(pst_adux, 0x2D, pst_adux->prox_off2_th);
		break;
	case GESTURE:
		read_reg = adux1020_read(pst_adux, ADUX_FREQUENCY);
		read_reg = (read_reg & 0xFFF) | (R_GEST_DIDT_M << 12);
		adux1020_write(pst_adux, ADUX_FREQUENCY, read_reg);

		read_reg = adux1020_read(pst_adux, ADUX_GEST_ORIEN_NPTS);
		read_reg = (read_reg & 0x7) | (R_GEST_N_PTS << 8);
		adux1020_write(pst_adux, ADUX_GEST_ORIEN_NPTS, read_reg);

		adux1020_write(pst_adux, ADUX_GEST_DI_TH, R_GEST_DI_TH);
		mode_val = SET_MODE_VALUE(GESTURE, NONE);
		intr_mask_val = SET_INTR_MASK_VALUE(GESTURE);
		break;
	case SAMPLE_RAW:
		read_reg = adux1020_read(pst_adux, ADUX_FREQUENCY);
		read_reg = (read_reg & 0xFFF) + (8 << 12);
		adux1020_write(pst_adux, ADUX_FREQUENCY, read_reg);

		read_reg =
		    adux1020_read(pst_adux, ADUX_GEST_ORIEN_NPTS);
		read_reg = (read_reg & 0x7) + (3 << 8);
		adux1020_write(pst_adux, ADUX_GEST_ORIEN_NPTS,
			       read_reg);
		mode_val = SET_MODE_VALUE(SAMPLE, RAW);
		adux1020_write(pst_adux, ADUX_GEST_DI_TH, 0);
		intr_mask_val = SET_INTR_MASK_VALUE(SAMPLE);
		/*discard first two packet during start*/
		discard_time = 2;
		break;
	case IDLE:
		mode_val = SET_MODE_VALUE(IDLE, NONE);
		intr_mask_val = SET_INTR_MASK_VALUE(IDLE);
		pst_adux->r_fifo_prevent = 1;
		break;
	default:
		ADUX1020_dbg("Invalid mode - entered into idle\n");
		ret = 0;
		mode = IDLE;
		break;
	};
	ADUX1020_info("mode_val - 0x%04x\tIntr_mask_val - 0x%04x\n",
		      mode_val, intr_mask_val);
	if (pst_adux->r_fifo_prevent == 1)
		mode_val = mode_val | (1 << 8);

	ADUX1020_info("mode_val - 0x%04x\tIntr_mask_val - 0x%04x\n",
		      mode_val, intr_mask_val);
	if (ret) {
		if (mode == PROXIMITY_I) {
			adux1020_write(pst_adux, ADUX_OP_MODE_ADDR,
					SET_MODE_VALUE(GESTURE, NONE));
		}

		adux1020_write(pst_adux, ADUX_INT_MASK_ADDR, intr_mask_val);
		adux1020_write(pst_adux, ADUX_OP_MODE_ADDR, mode_val);
		if (mode == GESTURE) {
			/* the below procedure is used to remove the
			   initial false gesture */
			adux1020_rd_intr_fifo(pst_adux);
			adux1020_fifo_data(pst_adux);	/*read FIFO */
			adux1020_clr_intr_status(pst_adux);

			pst_adux->discard_time = discard_time;
		}
		pst_adux->discard_time = discard_time;
	}
	pst_adux->mode = mode;
	pst_adux->time = jiffies;
	adux1020_enable_irq(pst_adux);
	mutex_unlock(&pst_adux->mutex);
	return SUCCESS;
}

/**
@brief This API used to initialize the adux1020 structure
@param dev pointer to Device Structure of the adux1020 chip
@param devid pointer to i2c_device_id of the adux1020 chip
@param irq GPIO irq number
@param pst_adux pointer to Adux1020 Structure
@param pt_i2c_bus_ops pointer to adux_bus_ops Structure of the adux1020 chip
@return void
*/
static void
adux1020_struct_init(struct device *dev,
		     const struct i2c_device_id *devid,
		     unsigned irq,
		     struct adux1020 *pst_adux, const struct adux_bus_ops
		     *pt_i2c_bus_ops)
{
	ADUX1020_info(" %s\n", __func__);
	pst_adux->adux_dev = NULL;
	pst_adux->adux_class = NULL;

	pst_adux->product = 0x1;
	pst_adux->version = 0x00;
	pst_adux->irq = irq;
	pst_adux->read = pt_i2c_bus_ops->read;
	pst_adux->write = pt_i2c_bus_ops->write;
	/*  Assigning Default Values chk */
	pst_adux->mode = 0xF;
	pst_adux->prev_mode = 0;
	pst_adux->fifosize = 0;
	pst_adux->fifo_th = 0;
	pst_adux->pu16_databuff = databuffer;
	pst_adux->chip_id = CHIP_ID_MASK;
	/*dynamic memory allocation */
	pst_adux->runtime_pdata = kzalloc(sizeof(struct adux_platform_data),
					  GFP_KERNEL);

	pst_adux->pdata = &adux1020_default_platform_data;
	ADUX1020_dbg("Default Configuration\n");

/*
	if (pst_adux->pdata == NULL) {
		pst_adux->pdata = &adux1020_default_platform_data;
		ADUX1020_dbg("Default Configuration\n");
	} else {
		ADUX1020_dbg("Board File Configuration\n");
	}
*/

	pst_adux->endian = ADUX1020_BIG_ENDIAN;

	pst_adux->r_fifo_prevent = 1;

	pst_adux->prox_type = 0;

	pst_adux->wakeup_mode = 0;	/*onchip gesture disable default */
	pst_adux->sequence = 1;	/*onchip gesture sequenc is ON by default */
	pst_adux->adux1020_suspend = 0;
	pst_adux->adux1020_interrupt = 0;
	pst_adux->adux1020_calibrate = 0;//bool

	pst_adux->discard_time = 0;

	pst_adux->prox_on1_th = 0;
	pst_adux->prox_off1_th = 0;
	pst_adux->prox_on2_th = 0;
	pst_adux->prox_off2_th = 0;

	pst_adux->intensity = 0;
	atomic_set(&pst_adux->adux_wq.gesture_count, 0);
	atomic_set(&pst_adux->adux_wq.gesture_seq_val, 0);
	pst_adux->adux_wq.prev_entry = 0;
}

/**
This function is used for  getting the power on initialization
status of  ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@return  void
*/
static void
adux1020_power_init(struct adux1020 *pst_adux)
{
	ADUX1020_info(" %s\n", __func__);
	/* Read chip ID  and board revision details */
	adux1020_read_chip(pst_adux);
	ADUX1020_dbg(" CHIP ID 0x%x\tVERSION 0x%x\n", pst_adux->chip_id,
		     pst_adux->version);
	if (pst_adux->chip_id != CHIP_ID_MASK) {
		adux1020_reset_device(pst_adux);
		/*  Set I2C high speed mode (400K)  */
		adux1020_set_i2c_speed(pst_adux);
	}
}

#ifdef ADUX_STAT
/**
This function clears all the statistic counters.
@param pst_adux the ADUX1020 data structure
@return void
*/
static void
adux1020_stat_reset(struct adux1020 *pst_adux)
{
	atomic_set(&pst_adux->stats.interrupts, 0);
	atomic_set(&pst_adux->stats.wq_pending, 0);
	atomic_set(&pst_adux->stats.wq_schedule_time_peak_usec, 0);
	atomic_set(&pst_adux->stats.wq_schedule_time_last_usec, 0);
	atomic_set(&pst_adux->stats.data_process_time_peak_usec, 0);
	atomic_set(&pst_adux->stats.data_process_time_last_usec, 0);
	atomic_set(&pst_adux->stats.fifo_requires_sync, 0);
	atomic_set(&pst_adux->stats.fifo_bytes[0], 0);
	atomic_set(&pst_adux->stats.fifo_bytes[1], 0);
	atomic_set(&pst_adux->stats.fifo_bytes[2], 0);
	ewma_init(&pst_adux->stats.wq_schedule_time_avg_usec, 2048, 128);
	ewma_init(&pst_adux->stats.data_process_time_avg_usec, 2048, 128);
}
#else
#endif

/******************************************************************/
/*CONFIG RELATED - START ******************************************/
/**
This function is used for general  configuration of the ADUX1020 Chipset
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_general_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->general_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration for general registers */
	ADUX1020_info(" %s\n", __func__);
	for (i = 0; i < config_reg->general_reg_cnt; i++) {
		addr = (u16) ((config_reg->general_regs[i] & 0xffff0000) >> 16);
		value = (u16) (config_reg->general_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_dbg(" %s i2c wr err %d\n",
					     __func__, ret);
				return FAILED;
			}
		}

	}
	return SUCCESS;
}

/**
This function is used low power configuration initialzation of the ADUX1020 Chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32 SUCCESS /FAILED bassed on the outcome.
*/
static s32
adux1020_low_power_config_init(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	s32 i = 0;
	u16 addr;
	u16 value = 0;

	struct adux_platform_data *config_reg;
	if (pst_adux->runtime_pdata->low_power_reg_cnt != 0)
		config_reg = pst_adux->runtime_pdata;
	else
		config_reg = pst_adux->pdata;
	/** configuration forproximity  registers */
	ADUX1020_info(" %s\n", __func__);
	for (i = 0; i < config_reg->low_power_reg_cnt; i++) {
		addr = (u16) ((config_reg->low_power_regs[i] & 0xffff0000) >>
			      16);
		value = (u16) (config_reg->low_power_regs[i] & 0x0000ffff);
		if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
			ret = adux1020_write(pst_adux, addr, value);
			if (ret < 0) {
				ADUX1020_dbg(" %s i2c wr err %d\n",
					     __func__, ret);
				return FAILED;
			}
		}
	}
	return SUCCESS;
}

/**
This function is used to configure the fifo and
	also for the updation of the same
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_update_config_init(struct adux1020 *pst_adux)
{
	u16 read_reg;
	ADUX1020_info(" %s\n", __func__);
	pst_adux->fifo_th = adux1020_update_fifo_th(pst_adux);
	/*set value of proximity type during configuration */
	read_reg = adux1020_read(pst_adux, ADUX_PROX_TYPE);
	read_reg = (read_reg & 0x7FFF) | (pst_adux->prox_type << 15);
	adux1020_write(pst_adux, ADUX_PROX_TYPE, (u16) read_reg);

	ADUX1020_dbg("Proximity threshold updated\n");
	pst_adux->prox_on1_th = adux1020_read(pst_adux, 0x2A);
	pst_adux->prox_off1_th = adux1020_read(pst_adux, 0x2B);
	pst_adux->prox_on2_th = adux1020_read(pst_adux, 0x2C);
	pst_adux->prox_off2_th = adux1020_read(pst_adux, 0x2D);
	return SUCCESS;
}

/**
This function is used for configuration initilization of the ADUX1020 chip
@param pst_adux pointer to ADUX1020 chip structure
@param config store the config mode number
@return  u16 return value
*/
static s32
adux1020_config_init(struct adux1020 *pst_adux, u16 config)
{
	s32 ret = 0;

	switch (config) {
	case CONFIG(GENERAL):
		ret = adux1020_general_config_init(pst_adux);
		break;
	case CONFIG(LOW_POWER):
		ret = adux1020_low_power_config_init(pst_adux);
		break;
	default:
		ADUX1020_dbg("INVALID INPUT\n");
		break;
	}
	ret = adux1020_update_config_init(pst_adux);
	return ret;
}

/*CONFIG RELATED - END ********************************************/
/*ISR & WORKQUEUE - START *****************************************/
/**
This function is used to read the fifo data for
	proximity mode of the ADUX1020 chip
@param prox_inputdev  pointer to ADUX1020 chip structure
@return s32  number of data read from fifo of the ADUX1020 chip
*/
static s32
adux1020_proximity(struct input_dev *prox_inputdev)
{
	struct adux1020 *pst_adux =
	    (struct adux1020 *)input_get_drvdata(prox_inputdev);
	s32 ret = 0;

	ret = adux1020_fifo_data(pst_adux);
	adux1020_proxi_evt(prox_inputdev);
	return ret;
}

/**
This function is used to read the fifo data for
	gesture mode of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32
*/
static s32
adux1020_gesture(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	/*Read FIFO Data  */
	ret = adux1020_fifo_data(pst_adux);

	pst_adux->adux_gest_data.dirX = (u8) ((pst_adux->pu16_databuff[0] &
					       0x60) >> 5);
	pst_adux->adux_gest_data.dirY = (u8) ((pst_adux->pu16_databuff[0] &
					       0x18) >> 3);
	pst_adux->adux_gest_data.dirZ = (u8) ((pst_adux->pu16_databuff[0] &
					       0x80) >> 7);
	pst_adux->adux_gest_data.gest_length =
	    (u8) ((pst_adux->pu16_databuff[0] >> 8) & 0xFF);

	if (jiffies_to_msecs(jiffies - pst_adux->time) > 3000) {
		if (pst_adux->adux_gest_data.gest_length != 0xFF)
			adux1020_gest_evt(pst_adux);
		else
			ADUX1020_info("gest length - 0x%04x\n",
				      pst_adux->adux_gest_data.gest_length);
	} else {

	}
	return ret;
}

/**
This function is used to read the fifo data for sample mode of the ADUX1020 chip
@param pst_adux  pointer to ADUX1020 chip structure
@return s32  number of data read from fifo of the ADUX1020 chip
*/
static s32
adux1020_sample(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	/*Read FIFO Data  */
	ret = adux1020_fifo_data(pst_adux);
	adux1020_sample_evt(pst_adux->gest_input);
	return ret;
}

/**
This function is used to process the interrupts and
	is called from the Work queue
@param pst_adux the adux1020 chip structure.
@return void
*/
static void
adux1020_work(struct adux1020 *pst_adux)
{
	mutex_lock(&pst_adux->mutex);
#ifdef INT_LEVEL
	adux1020_disable_irq(pst_adux);
#else
#endif
	/*Read Intr status and update Fifo size to the adux structure */
	if (adux1020_rd_intr_fifo(pst_adux) != SUCCESS) {
		/*error */
		ADUX1020_err("error\n");
	}
#ifdef INT_EDGE
	adux1020_clr_intr_status(pst_adux);
#else
	adux1020_clr_intr_status(pst_adux);
#endif

	if (pst_adux->curr_intr != 0) {
		switch (pst_adux->mode) {
		case PROXIMITY_I:
			{
				adux1020_proximity(pst_adux->prox_input);
				input_event(pst_adux->prox_input,
					    EV_ABS, ABS_DISTANCE,
					    4);
				input_sync(pst_adux->prox_input);
				if (pst_adux->intensity >
				    pst_adux->prox_on1_th) {
					ADUX1020_dbg("Near to Sensor\n");
					input_event(pst_adux->prox_input,
						    EV_ABS, ABS_DISTANCE,
						    MIN_DISTANCE);
					input_sync(pst_adux->prox_input);
				} else if (pst_adux->intensity <
					   pst_adux->prox_off1_th) {
					ADUX1020_dbg("Aaway from sensor\n");
					input_event(pst_adux->prox_input,
						    EV_ABS, ABS_DISTANCE,
						    MAX_DISTANCE);
					input_sync(pst_adux->prox_input);
				} else {

				}
			}
			break;
		case GESTURE:
			{
				adux1020_gesture(pst_adux);
			}
			break;
		case SAMPLE_RAW:
			{
				adux1020_sample(pst_adux);
			}
			break;
		case FIFO:
			{
				ADUX1020_info(" INT_FIFO_FULL triggered\n");
			/*FIFO mainted in driver need to reset the buffer,
			   if it is in event driven
			   If FIFO interrupt raised during in
			   EVENT DRIVEN model,
			   TBD->the buffer allocated for the data buffer
			   need to cleared and reallocated */
			/* TBD clear the data buffer if allocated. */
				adux1020_flush_buffer(pst_adux);
				/* chk whther need to reset the device */
			}
			break;
		case IDLE:
			{
				adux1020_fifo_data(pst_adux);
				adux1020_flush_buffer(pst_adux);
			}
			break;
		default:
			{
				ADUX1020_err("mode = 0x%x\n", pst_adux->mode);
				ADUX1020_err("curr_intr = 0x%x\n",
					     (u32) pst_adux->curr_intr);
				ADUX1020_err("INVALID!! INT triggered\n");
			}
			break;
		};
	}
#ifdef INT_LEVEL
	adux1020_enable_irq(pst_adux);
#else
#endif
	mutex_unlock(&pst_adux->mutex);
}

/**
This work function that is executed from the IRQ
@param work the work assigned.
@return void
*/
static void
adux1020_work_queue(struct work_struct *work)
{
	struct adux1020 *pst_adux = container_of(work,
						 struct adux1020,
						 work);
#ifdef ADUX_STAT
	struct timeval wkq_start;
	struct timeval wkq_comp;
	int diff_usec = 0;
	if (!pst_adux->discard_time) {
		do_gettimeofday(&wkq_start);

		diff_usec = timeval_compare(&wkq_start,
					    &pst_adux->stats.stamp
						.interrupt_trigger);

		if (diff_usec > 1) {
			if (diff_usec >
				atomic_read(
					&pst_adux->stats
						.wq_schedule_time_peak_usec))
				atomic_set(
					&pst_adux->stats
						.wq_schedule_time_peak_usec,
					diff_usec);
			atomic_set(&pst_adux->stats.wq_schedule_time_last_usec,
				   diff_usec);
			ewma_add(&pst_adux->stats.wq_schedule_time_avg_usec,
				 diff_usec);
		}
	}
#else
#endif

	adux1020_work(pst_adux);

#ifdef ADUX_STAT
	if (!pst_adux->discard_time) {
		do_gettimeofday(&wkq_comp);

		diff_usec = timeval_compare(&wkq_comp, &wkq_start);

		if (diff_usec > 1) {
			if (diff_usec >
				atomic_read(
					&pst_adux->stats
						.data_process_time_peak_usec))
				atomic_set(
					&pst_adux->stats
						.data_process_time_peak_usec,
					diff_usec);
			atomic_set(&pst_adux->stats.data_process_time_last_usec,
				   diff_usec);
			ewma_add(&pst_adux->stats.data_process_time_avg_usec,
				 diff_usec);
		}
	}
#else
#endif

}

/**
This ISR function is the back end of the irs in the thread
	to handle the sleeping functions
@param irq the IRQ assigned.
@param dev_id the Device ID
@return irqreturn_t IRQ_HANDLED values
*/
static irqreturn_t
adux1020_isr(int irq, void *dev_id)
{
	struct adux1020 *pst_adux = dev_id;
#ifdef INT_EDGE
	unsigned long flags;
	local_irq_save(flags);
	local_irq_disable();
#else
#endif

#ifdef ADUX_STAT
	if (!pst_adux->discard_time)
		atomic_inc(&pst_adux->stats.interrupts);
	if (!work_pending(&pst_adux->work)) {
#ifdef INT_LEVEL
		disable_irq_nosync(pst_adux->irq);
		pst_adux->adux1020_interrupt = 0;
#else
#endif
		if (!pst_adux->discard_time)
			do_gettimeofday(&pst_adux->stats.stamp
							.interrupt_trigger);
		if (!schedule_work(&pst_adux->work))
			if (!pst_adux->discard_time)
				atomic_inc(&pst_adux->stats.wq_pending);
	} else {
		if (!pst_adux->discard_time) {
			atomic_inc(&pst_adux->stats.wq_pending);
			ADUX1020_info("work_pending !!\n");
		}
	}
#else


	if (!work_pending(&pst_adux->work)) {
#ifdef INT_LEVEL
		disable_irq_nosync(pst_adux->irq);
		pst_adux->adux1020_interrupt = 0;
#else
#endif
		schedule_work(&pst_adux->work);
	} else {
		ADUX1020_err("work pending!!\n");
	}
#endif

#ifdef INT_EDGE
	local_irq_restore(flags);
#else
#endif

	return IRQ_HANDLED;
}

/*ISR & WORKQUEUE - END *******************************************/
/*SYSFS - START ***************************************************/
/**
This function is used for I2C read from the sysfs
	file system of the ADUX1020 Chip
This is called from the Sysfs functions
@param pst_adux  pointer to ADUX1020 chip structure
@return s16 the bytes of read data.
*/
static s16
adux1020_sysfs_I2C_rd(struct adux1020 *pst_adux)
{
	u16 value = 0;
	ADUX1020_info(" %s\n", __func__);
	/*adux1020_read return length on success and negative on failure */
	value = adux1020_read(pst_adux, pst_adux->sysfs_I2C_regaddr);
	return value;
}

/**
This function is used for I2C write from the sysfs
	file system of the ADUX1020 Chip
This is called from the Sysfs functions
@param pst_adux  pointer to ADUX1020 chip structure
@return s16 the bytes of written data.
*/
static s16
adux1020_sysfs_I2C_wr(struct adux1020 *pst_adux)
{
	s32 ret = 0;
	u16 value = pst_adux->sysfs_I2C_regval;

	ADUX1020_info(" %s\n", __func__);
	/*pst_adux->wrtie return length on success and return -error on error */
	ret = adux1020_write(pst_adux, pst_adux->sysfs_I2C_regaddr, value);
	if (ret < 0) {
		ADUX1020_err(" %s i2c read error %d\n", __func__, ret);
		return -1;
	} else {
		return value;
	}
}

/**
This function is used to show the mode of operation of the driver
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr standard Linux Device attribute.
*	@param buf The buffer to store the read data.
*	@return ssize_t The count of the data sent to the user buffer.
*/

static ssize_t
adux1020_sysfs_mode_show(struct device *adux_dev,
			 struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("OP_MODE 0x%x\n\n", pst_adux->mode);
	return sprintf(buf, "0x%x", pst_adux->mode);
}

/**
This function is used to store the mode of operation of the driver
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The size of the Buffer.
*	@return ssize_t The Size of the User buffer.
*/
static ssize_t
adux1020_sysfs_mode_store(struct device *adux_dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	u16 mode = 0;
	u16 parse_data[2];

	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);

	mode = parse_data[0];

	if ((mode != OFF) || (mode != FORCE)) {
		ADUX1020_dbg("OP_MODE 0x%x\n", mode);
		adux1020_mode_switching(pst_adux, mode);
	} else {
		ADUX1020_info(" Invalid mode !!!\n");
		return -EINVAL;
	}
	return count;
}

/**
This function is used to display the last read register value
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data,USER space buffer.
*	@return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_reg_read_show(struct device *adux_dev,
			     struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Regval : 0x%4x\n", pst_adux->sysfslastreadval);
	return sprintf(buf, "0x%x", pst_adux->sysfslastreadval);
}

/**
This function is used to Read the register values of the ADUX1020 chip
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The count of the data in the buffer provided from user.
*	@return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_reg_read_store(struct device *adux_dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short addr;
	unsigned short parse_data[4];
	unsigned short ret;

	memset(parse_data, 0, sizeof(unsigned short) * 4);
	cmd_parsing(buf, 2, parse_data);
	addr = parse_data[0];

	mutex_lock(&pst_adux->mutex);

	pst_adux->sysfs_I2C_regaddr = addr;

	ret = adux1020_sysfs_I2C_rd(pst_adux);
	if (ret != (-1)) {
		pst_adux->sysfslastreadval = ret;
	} else {
		ADUX1020_info("%s Error\n", __func__);
		pst_adux->sysfslastreadval = (unsigned short) -1;
	}

	mutex_unlock(&pst_adux->mutex);
	return count;
}

/**
This function is used to write to a register in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the Read data
*	@param count The count of the data in the user space buffer.
*	@return ssize_t returns The count of the data received.
*/
static ssize_t
adux1020_sysfs_reg_write_store(struct device *adux_dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short addr = 0;
	unsigned short parse_data[3];
	unsigned short value[4];
	unsigned short ret = 0;
	int arg_cnt = 0;

	memset(parse_data, 0, sizeof(unsigned short) * 3);
	arg_cnt = cmd_parsing(buf, 3, parse_data);

	addr = parse_data[0];
	switch (arg_cnt) {
	case 2:
		value[0] = parse_data[1];
	break;
	case 3:
		value[0] = parse_data[2];
	break;
	default:
		ret = -EINVAL;
	break;
	};
	if (ret < 0) {
		ADUX1020_dbg("Too many arguments\n");
		return -EINVAL;
	}

	if (!(addr == 0x45 || addr == 0x48 || addr == 0x49)) {
		pst_adux->sysfs_I2C_regaddr = addr;
		pst_adux->sysfs_I2C_regval = value[0];

		mutex_lock(&pst_adux->mutex);

		ret = adux1020_sysfs_I2C_wr(pst_adux);

		if ((addr >= 0x2A) && (addr <= 0x2F))
			adux1020_update_config_init(pst_adux);
		mutex_unlock(&pst_adux->mutex);
	} else {

	}

	return count;
}

//yulong add
static ssize_t
adux1020_sysfs_calibrate_show(struct device *adux_dev,
			     struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);

	printk(KERN_ERR "ADUX1020:YLLOG:calibrate flag = %d.\n",
				pst_adux->adux1020_calibrate);
	return sprintf(buf, "%d", pst_adux->adux1020_calibrate);
}
static ssize_t
adux1020_sysfs_calibrate_store(struct device *adux_dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short cal_val[6];
	int arg_cnt = 0;
	struct SensorCalInfo cal_info[2];
	int i, j, k;

	wake_lock(&pst_adux->cal_wake_lock);
	memset(cal_val, 0, 6 * sizeof(unsigned short));
	arg_cnt = cmd_parsing(buf, 6, cal_val);
	memset(cal_info, 0, 2 * sizeof(struct SensorCalInfo));

	if (6 == arg_cnt) {
		for (i =0; i <6; i++)
			printk(KERN_ERR "ADUX1020:YLLOG===0x%x====\n", cal_val[i]);
		cal_info[0].SensorCalStatus = 1;
		cal_info[1].SensorCalStatus = 1;

		//init cal_info[0]
		k = 0;
		for (j = 0; j < 2; j++) {
			for (i = 0; i < 6; i += 2) {
				cal_info[j].value[i] = cal_val[k] & 0x00ff;
				cal_info[j].value[i+1] = (cal_val[k] & 0xff00) >> 8;
				printk(KERN_ERR "===0x%x%x===",
					cal_info[j].value[i+1],
					cal_info[j].value[i]);
				k++;
			}
		}
	} else {
		printk(KERN_ERR "ADUX1020:YLLOG:invalid calibrate value.\n");
		goto exit;
	}

	//write to flash
	if (sizeof(cal_info[0]) !=
			sensparams_write_to_flash(SENSPARAMS_TYPE_PROX,
						(unsigned char *)&cal_info[0],
						sizeof(cal_info[0]))) {
		printk(KERN_ERR "ADUX1020:YLLOG:1:write to flash failed !\n");
		goto exit;
	}
	if (sizeof(cal_info[1]) !=
			sensparams_write_to_flash(SENSPARAMS_TYPE_RESV1,
						(unsigned char *)&cal_info[1],
						sizeof(cal_info[1]))) {
		printk(KERN_ERR "ADUX1020:YLLOG:1:write to flash failed !\n");
		goto exit;
	}

	pst_adux->adux1020_calibrate = 0;
	wake_unlock(&pst_adux->cal_wake_lock);

	return count;
exit:
	wake_unlock(&pst_adux->cal_wake_lock);
	return -1;
}
//yulong add end

/**
@brief This API used to display the status of sysfs version
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the adux1020
@param buf The buffer to store the Read data
@return ssize_t The Size of the Read Data.
*/
static ssize_t
adux1020_sysfs_version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION);
}

/**
This function is used to display the register's cofiguration of the mode
	at runtime in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data
*	@return ssize_t The count of the data received.
*/
static ssize_t
adux1020_sysfs_config_show(struct device *adux_dev,
			   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "status of config thread\n");
}

/**
This function is used to configure the registers based on the modes at runtime in the ADUX 1020 / ADUX 1020i chip
*	@param adux_dev The Device ID and Information structure(Linux Standard argument)
*	@param attr Standard linux device attributes.
*	@param buf The buffer to store the read data
*	@param count The count of the data in the user space buffer.
*	@return ssize_t The count of the data received..
*	@note configuration Supported
	Config from File  - "0"
	Config from Array - "1"
*/
static ssize_t
adux1020_sysfs_config_store(struct device *adux_dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	/* struct test_config_register config_reg[50]; */
	unsigned short parse_data[1];
	memset(parse_data, 0, sizeof(unsigned short) * 1);
	cmd_parsing(buf, 1, parse_data);

	if (parse_data[0] == FRM_ARR) {
		ADUX1020_dbg("Config from Static Array\n");
		adux1020_config_init(pst_adux, CONFIG(GENERAL));
	} else if (parse_data[0] == FRM_FILE) {
		adux_filp_start_calib(GENERAL_CONFIG, pst_adux,
				      CONFIG(GENERAL));
		adux1020_config_init(pst_adux, CONFIG(GENERAL));
	} else {
		ADUX1020_dbg("set 1 to config\n");
		return -EINVAL;
	}
	return count;
}

//#ifdef CONFIG_FB
/**
This function is used to display the status of wakeup mode
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data,USER space buffer.
*       @return ssize_t The count of the data sent to the user buffer.
*/
static ssize_t
adux1020_sysfs_wakeup_mode_show(struct device *adux_dev,
				struct device_attribute *attr, char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	ADUX1020_dbg("Wakeup mode, 0 - Fullpower down, 1 - Normal wakeup\n");
	ADUX1020_dbg("Wakeup mode - 0x%x\n", pst_adux->wakeup_mode);
	return sprintf(buf, "0x%x", pst_adux->wakeup_mode);
}

/**
This function is used to wakeup mode full power down mode and onchip gesture mode.
*       @param adux_dev The device ID and Information structure(Linux Standard argument)
*       @param attr Standard linux device attributes.
*       @param buf The buffer to store the Read data
*       @param count The count of the data in the buffer provided from user.
*       @return ssize_t The count of the data received by default.
*/
static ssize_t
adux1020_sysfs_wakeup_mode_store(struct device *adux_dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct adux1020 *pst_adux = dev_get_drvdata(adux_dev);
	unsigned short parse_data[2];
	memset(parse_data, 0, sizeof(unsigned short) * 2);
	cmd_parsing(buf, 1, parse_data);
	if (pst_adux->adux1020_suspend != 1) {
		if (parse_data[0] == 0) {
			pst_adux->wakeup_mode = 0;
			ADUX1020_dbg("In suspend - switch to IDLE mode\n");
		} else if (parse_data[0] == 1) {
			pst_adux->wakeup_mode = 1;
			ADUX1020_dbg("In suspend - switch to Gesture mode\n");
		} else {

		}
	} else {
		ADUX1020_dbg("Err - can't change the mode in suspend\n");
	}
	return count;
}
//#endif

#ifdef ADUX_STAT
/**
This function is used for getting the status of adux1022 and driver
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@return ssize_t size of data presnt in the buffer
*/
static ssize_t
adux1020_sysfs_stat_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	unsigned int interrupts = atomic_read(&pst_adux->stats.interrupts);
	unsigned int wq_pending = atomic_read(&pst_adux->stats.wq_pending);
	unsigned int wq_schedule_time_peak_usec =
		atomic_read(&pst_adux->stats.wq_schedule_time_peak_usec);
	unsigned int wq_schedule_time_last_usec =
		atomic_read(&pst_adux->stats.wq_schedule_time_last_usec);
	unsigned int data_process_time_peak_usec =
		atomic_read(&pst_adux->stats.data_process_time_peak_usec);
	unsigned int data_process_time_last_usec =
		atomic_read(&pst_adux->stats.data_process_time_last_usec);

	return sprintf(buf, "\
		interrupts                  : %d\n\
		wq_pending                  : %d\n\
		wq_schedule_time_peak_usec  : %d\n\
		wq_schedule_time_avg_usec   : %d\n\
		wq_schedule_time_last_usec  : %d\n\
		data_process_time_peak_usec : %d\n\
		data_process_time_avg_usec  : %d\n\
		data_process_time_last_usec : %d\n\
		fifo_requires_sync          : %d\n\
		fifo bytes history          : [%d %d %d]\n\
		Build time                  : %s\n",
		interrupts, wq_pending,
		wq_schedule_time_peak_usec,
		(int)ewma_read(&pst_adux->stats.wq_schedule_time_avg_usec),
		wq_schedule_time_last_usec,
		data_process_time_peak_usec,
		(int)ewma_read(&pst_adux->stats.data_process_time_avg_usec),
		data_process_time_last_usec,
		atomic_read(&pst_adux->stats.fifo_requires_sync),
		atomic_read(&pst_adux->stats.fifo_bytes[0]),
		atomic_read(&pst_adux->stats.fifo_bytes[1]),
		atomic_read(&pst_adux->stats.fifo_bytes[2]),
		utsname()->version);
}

#define ADUX1020_STAT_RESET     1

/**
This function is used for wrting the adux stat value to zero
@param dev linux device structure
@param attr pointer point linux device_attribute structure
@param buf pointer point the buffer
@param size buffer size
@return ssize_t size of data written to the buffer
*/
static ssize_t
adux1020_sysfs_stat_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct adux1020 *pst_adux = dev_get_drvdata(dev);
	unsigned short parse_data[1];
	memset(parse_data, 0, sizeof(unsigned short) * 1);
	cmd_parsing(buf, 1, parse_data);

	if (parse_data[0] == ADUX1020_STAT_RESET) {
		ADUX1020_dbg("Resetting statistics\n");
		adux1020_stat_reset(pst_adux);
	}

	return size;
}
#else
#endif
#ifdef PC_TOOL_SUPPORT
/*PC TOOL TEST: this should be removed during production*/
static ssize_t
adux1020_sysfs_gestout_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "0\n");
}

/*PC TOOL TEST: this should be removed during production*/
static ssize_t
adux1020_sysfs_gestout_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	return size;
}
#else
#endif
/**
  Sysfs device attributes list
 */

static DEVICE_ATTR(adux1020_configuration,
		   0664,
		   adux1020_sysfs_config_show, adux1020_sysfs_config_store);

static DEVICE_ATTR(adux1020_mode,
		   0664,
		   adux1020_sysfs_mode_show, adux1020_sysfs_mode_store);

static DEVICE_ATTR(adux1020_reg_read,
		   0664,
		   adux1020_sysfs_reg_read_show, adux1020_sysfs_reg_read_store);

static DEVICE_ATTR(adux1020_reg_write,
		   0664, NULL, adux1020_sysfs_reg_write_store);

static DEVICE_ATTR(adux1020_calibrate,
			0664,
			adux1020_sysfs_calibrate_show, adux1020_sysfs_calibrate_store);

#ifdef PC_TOOL_SUPPORT
/*PC TOOL TEST: this should be removed during production*/
static DEVICE_ATTR(adux1020_gestout,
		   0664, adux1020_sysfs_gestout_show,
		   adux1020_sysfs_gestout_store);
#else
#endif
#ifdef ADUX_STAT
static DEVICE_ATTR(adux1020_stat,
		   0664,
		   adux1020_sysfs_stat_get,
		   adux1020_sysfs_stat_set);
#endif
//#ifdef CONFIG_FB
static DEVICE_ATTR(adux1020_wakeup_mode,
		   0664,
		   adux1020_sysfs_wakeup_mode_show,
		   adux1020_sysfs_wakeup_mode_store);
//#endif
static DEVICE_ATTR(version, 0664, adux1020_sysfs_version_show, NULL);

/**
array of attributes
*/
static struct device_attribute *adux_sensor_attrs[] = {
	&dev_attr_adux1020_configuration,
	&dev_attr_adux1020_reg_read,
	&dev_attr_adux1020_reg_write,
	&dev_attr_adux1020_mode,
	&dev_attr_adux1020_calibrate,
#ifdef PC_TOOL_SUPPORT
/*PC TOOL TEST: this should be removed during production*/
	&dev_attr_adux1020_gestout,
#else
#endif
//#ifdef CONFIG_FB
	&dev_attr_adux1020_wakeup_mode,
//#endif
#ifdef ADUX_STAT
	&dev_attr_adux1020_stat,
#endif
	&dev_attr_version,
	NULL
};

/**
This function is used to register sysfs attribute to the device
gesture_sensor in the class name adux_sensor.
@param pst_adux  pointer to ADUX1020 chip structure
@param attributes pointer holding array of device attributes.
@return int the status of the function.
*/
static int adux1020_sysfs_create(struct adux1020 *pst_adux,
				 struct device_attribute *attributes[])
{
	int i = 0;
	/*ADUX_SENSOR class is created*/
	if (!pst_adux->adux_class) {
		pst_adux->adux_class = class_create(THIS_MODULE, "adux_sensor");
		if (IS_ERR(pst_adux->adux_class)) {
			pst_adux->adux_class = NULL;
			/*return PTR_ERR(pst_adux->adux_class);*/
			return -1;
		}
	}

	/*Gesture sensor attribute is created*/
	if (!pst_adux->adux_dev) {
		pst_adux->adux_dev = device_create(pst_adux->adux_class,
						   NULL,
						   0,
						   pst_adux,
						   "%s",
						   "gesture_sensor");
	}

	if (IS_ERR(pst_adux->adux_dev)) {
		/*return PTR_ERR(pst_adux->adux_dev);*/
		pst_adux->adux_dev = NULL;
		return -1;
	}

	for (i = 0; attributes[i] != NULL; i++)
		if ((device_create_file(pst_adux->adux_dev, attributes[i])) < 0)
			goto err_sysfs_interface;

	dev_set_drvdata(pst_adux->adux_dev, pst_adux);

	return 0;
err_sysfs_interface:
	for (; i >= 0; i--)
		device_remove_file(pst_adux->adux_dev, attributes[i]);
	ADUX1020_dbg("Unable to create sysfs interface\n");
	return -1;
}

/**
This function is used to unregister sysfs attribute, device gesture_sensor,
class adux_sensor.
@param pst_adux  pointer to ADUX1020 chip structure
@param attributes pointer holding array of device attributes.
@return int the status of the function.
*/
static int adux1020_sysfs_remove(struct adux1020 *pst_adux,
				 struct device_attribute *attributes[])
{
	int i = 0;

	if (pst_adux->adux_dev != NULL) {
		dev_set_drvdata(pst_adux->adux_dev, NULL);
		for (i = 0; attributes[i] != NULL; i++)
			device_remove_file(pst_adux->adux_dev, attributes[i]);
		pst_adux->adux_dev = NULL;
	}


	if (pst_adux->adux_class != NULL) {
		device_destroy(pst_adux->adux_class, 0);
		class_destroy(pst_adux->adux_class);
		pst_adux->adux_class = NULL;
	}

	return 0;
}


#ifdef PC_TOOL_SUPPORT
/*PC TOOL TEST: this should be removed during production*/
/************************************************************************/
static struct class *adux_class_link;
static struct device *adux_dev_link;


void adux1020_sysfs_link_create(struct adux1020 *pst_adux)
{
	int ret = 0;
	adux_class_link = class_create(THIS_MODULE, "ADUX_SENSOR");
	adux_dev_link = device_create(adux_class_link, NULL, 0, NULL, "%s",
				      "ADUX1020");

	ret = sysfs_create_link(&adux_dev_link->kobj,
				&pst_adux->adux_dev->kobj,
				"attribute");
	if (ret)
		ADUX1020_dbg("Symbolic creation link failed\n");
	else
		ADUX1020_dbg("Symbolic link created\n");

	return;
}

void adux1020_sysfs_link_destroy(struct adux1020 *pst_adux)
{
	sysfs_remove_link(&adux_dev_link->kobj, "attribute");
	device_destroy(adux_class_link, 0);
	class_destroy(adux_class_link);
	return;
}
/************************************************************************/
#else
#endif
/*SYSFS - END *****************************************************/

#ifdef CONFIG_OF
static int adux1020_get_dt_regs(struct device *dev, char *name, u32 *array)
{
        struct property *prop;
        struct device_node *np = dev->of_node;
        int rc;

        prop = of_find_property(np, name, NULL);
        if (!prop)
                return -EINVAL;
        if (!prop->value)
                return -ENODATA;

        rc = of_property_read_u32_array(np, name, array, prop->length/sizeof(u32));
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "ADUX1020:%s: Unable to read %s\n", __func__, name);
                return rc;
        }

        ADUX1020_info("ADUX1020:%s size is %d\n", name, prop->length/sizeof(u32));
        return rc;
}

static int adux1020_parse_dt(struct device *dev, struct adux_platform_data *pdata)
{
        int ret;
        enum of_gpio_flags flags;
        struct device_node *np = dev->of_node;
        u32 value;
/* parse irq and request gpio */
        value = of_get_named_gpio_flags(np, "ad,irq-gpio", 0, &flags);
        ADUX1020_info("%s: irq gpio num is %d\n", __func__, value);
        if (value < 0) {
                return -EINVAL;
        }
        gpio_request(value, "adux-irq");
        gpio_direction_input(value);

/* general_reg */
        ret = of_property_read_u32(np, "ad,general_reg_cnt", &pdata->general_reg_cnt);
        if (ret) {
                dev_err(dev, "ADUX1020:Looking up %s property in node %s failed",
                        "ad,general_reg_cnt", np->full_name);
                return -ENODEV;
        }
        ret = adux1020_get_dt_regs(dev, "ad,general_regs", (u32 *)pdata->general_regs);
        if (ret) {
                dev_err(dev, "ADUX1020:Looking up %s property in node %s failed",
                        "ad,general_regs", np->full_name);
                return -ENODEV;
        }
        ADUX1020_info("general_reg_cnt %d\n", pdata->general_reg_cnt);

/* low_power_reg */
        ret = of_property_read_u32(np, "ad,low_power_reg_cnt", &pdata->low_power_reg_cnt);
        if (ret) {
                dev_err(dev, "ADUX1020:Looking up %s property in node %s failed",
                        "ad,gesture_reg_cnt", np->full_name);
                return -ENODEV;
        }
        ret = adux1020_get_dt_regs(dev, "ad,low_power_regs", (u32 *)pdata->low_power_regs);
        if (ret) {
                dev_err(dev, "ADUX1020:Looking up %s property in node %s failed",
                        "adux,low_power_regs", np->full_name);
                return -ENODEV;
        }
        ADUX1020_info("low_power_reg_cnt %d\n", pdata->low_power_reg_cnt);

        return 0;
}
#endif

/**
  This is a function used to prepare the ADUX device while suspend
  @param pst_adux the ADUX device pointer
*/
static void
adux1020_suspend_notifier(struct adux1020 *pst_adux)
{
#ifdef ADUX_LOW_POWER
	int ret = 0;
#else
#endif

	if (pst_adux->mode >= 3) {
		ADUX1020_dbg("%s - mode[0x%x] is not in proximity\n", __func__,
			     pst_adux->mode);
		pst_adux->prev_mode = pst_adux->mode;
		pst_adux->adux1020_suspend = 1;
		adux1020_mode_switching(pst_adux, IDLE);

#ifdef ADUX_LOW_POWER
		adux_reg_store(pst_adux, CONFIG(GENERAL));
		ret =
		    adux_filp_start_calib(LOW_POWER_CONFIG, pst_adux,
					  CONFIG(LOW_POWER));
		if (ret == -1) {
			pst_adux->runtime_pdata->low_power_reg_cnt = 0;
			ADUX1020_dbg("invalid F - %s\n", LOW_POWER_CONFIG);
		}
		adux1020_config_init(pst_adux, CONFIG(LOW_POWER));
#else
#endif

		if (pst_adux->wakeup_mode) {
			ADUX1020_dbg("Gesture Sequence ON\n");
			adux1020_mode_switching(pst_adux, GESTURE);
		} else {

			adux1020_mode_switching(pst_adux, IDLE);
		}
	} else {
		ADUX1020_dbg("%s - mode is in proximity\n", __func__);
	}

}

/**
  This is a function used to prepare the ADUX device while resume
  @param pst_adux the ADUX device pointer
*/
static void
adux1020_resume_notifier(struct adux1020 *pst_adux)
{
	if (pst_adux->mode >= 3 || pst_adux->mode == 0) {
		ADUX1020_dbg("%s - mode is not in proximity\n", __func__);

		pst_adux->adux1020_suspend = 0;
		adux1020_mode_switching(pst_adux, IDLE);
		if (pst_adux->wakeup_mode) {
			adux1020_mode_switching(pst_adux, pst_adux->prev_mode);
		} else {
			#ifdef ADUX_LOW_POWER
			pst_adux->runtime_pdata->general_reg_cnt = 0;
			adux1020_config_init(pst_adux, CONFIG(GENERAL));
			#else
			#endif
			adux1020_mode_switching(pst_adux, pst_adux->prev_mode);
		}
	} else {
		ADUX1020_dbg("%s - mode is in proximity\n", __func__);
	}
}

/**
  This function is callback function for recieving the notfication signal
  from frame buffer
  @param self notifier_block structure
  @param event event
  @param data pointer
  @return s32 status of the called function
*/
//static int fb_notifier_callback(struct notifier_block *self,
//				unsigned long event, void *data)
//{
//	struct fb_event *evdata = data;
//	int *blank;
//	struct adux1020 *pst_adux =
//				container_of(self,
//					     struct adux1020,
//					     adux1020_fb_notif);
//
//	//if (pst_adux->mode == PROXIMITY_I) {
//	//	ADUX1020_dbg("Device in proximity mode\n");
//	//	return 0;
//	//}
//	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
//		blank = evdata->data;
//		if (*blank == FB_BLANK_UNBLANK) {
//			ADUX1020_dbg("--------FB_BLANK_UNBLANK\n");
//			adux1020_resume_notifier(pst_adux);
//		} else if (*blank == FB_BLANK_POWERDOWN) {
//			ADUX1020_dbg("--------FB_BLANK_POWERDOWN\n");
//			adux1020_suspend_notifier(pst_adux);
//		}
//	}
//	return 0;
//}

/**
  This function is  used for Device probe. All initialization routines are handled here
  This is the callback function from the adux1020_i2c_probe
  @param dev This is the device structure of the i2c client
  @param devid ID to ADUX1020 chip
  @param irq GPIO irq number
  @param pt_i2c_bus_ops pointer to the adux_bus_ops structure
  @return ADUX1020 Device structure on Sucess.On failure -ENOMEM, -EINVAL ,etc.,
 */
static struct adux1020 *
adux1020_probe(struct device *dev,
	       const struct i2c_device_id *devid,
	       unsigned irq, const struct adux_bus_ops *pt_i2c_bus_ops)
{
	struct adux1020 *pst_adux;
	struct adux_platform_data *plat_data;
	s32 error;

	ADUX1020_info(" %s\n", __func__);
	if (!irq) {
		dev_err(dev, "no IRQ?\n");
		error = -EINVAL;
		goto err_out;
	}
#ifdef ENDIAN
	if (!endian) {
		dev_err(dev, "no endian?\n");
		error = -EINVAL;
		goto err_out;
	}
#endif

#ifdef CONFIG_OF
        plat_data = kzalloc(sizeof(*plat_data), GFP_KERNEL);
        if (!plat_data) {
                ADUX1020_dbg("faild to alloc pdata memory\n");
                error = -ENOMEM;        /* out of memory */
                goto err_out;
        }
        error = adux1020_parse_dt(dev, plat_data);
        if (error) {
                ADUX1020_dbg("faild to parse dt\n");
                goto err_free_pdata;
        }
#else
        plat_data = dev->platform_data;
#endif

	pst_adux = kzalloc(sizeof(struct adux1020), GFP_KERNEL);

	if (IS_ERR(pst_adux)) {
		error = -ENOMEM;	/* out of memory */
		goto err_free_mem;
	}

	pst_adux->client = to_i2c_client(dev);

	/* adux1020 structure initialization is done here */
	pst_adux->pdata = plat_data;
	adux1020_struct_init(dev, devid, irq, pst_adux, pt_i2c_bus_ops);

	/* Power on initilization */
	adux1020_power_init(pst_adux);
	if (pst_adux->chip_id == CHIP_ID_MASK) {
		ADUX1020_dbg("adux1020 i2c slave device not found!!!\n");
		error = -ENXIO;	/*No such device or address */
		goto err_free_mem;
	}
	dev_info(dev, " ADUX1020 Chip OK, ID is 0x%x\n", pst_adux->chip_id);

	/* General configuration */
	adux_filp_start_calib(GENERAL_CONFIG, pst_adux, CONFIG(GENERAL));
	adux1020_config_init(pst_adux, CONFIG(GENERAL));
	/*adux1020_clock_calibration(pst_adux); */

	adux1020_enter_idle(pst_adux);

	mutex_init(&pst_adux->mutex);


	wake_lock_init(&pst_adux->cal_wake_lock, WAKE_LOCK_SUSPEND, "prox_cal_wakelock");
	INIT_WORK(&pst_adux->work, adux1020_work_queue);

#ifdef INT_EDGE
	irq_set_irq_type(pst_adux->irq, IRQ_TYPE_EDGE_RISING);
	ADUX1020_dbg("Interrupt is configured for Edge triggering\n");

	/* Interrupt Handle Request */
	error = request_irq(pst_adux->irq,
			    adux1020_isr,
			    IRQF_TRIGGER_RISING, dev_name(dev), pst_adux);
#else

#endif
#ifdef INT_LEVEL
	irq_set_irq_type(pst_adux->irq, IRQ_TYPE_LEVEL_HIGH);
	ADUX1020_dbg("Interrupt is configured for Level triggering\n");
	error = request_irq /*threaded_irq */ (pst_adux->irq,	/* NULL, */
					       adux1020_isr,
					       IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					       dev_name(dev), pst_adux);
#else

#endif

	if (error) {
		ADUX1020_dbg(" irq %d busy?\n", pst_adux->irq);
		goto err_free_mem;
	}

	pst_adux->adux1020_interrupt = 0;
	disable_irq_nosync(pst_adux->irq);

	/* Allocation of input devices */
	pst_adux->gest_input = adux1020_inputdev_alloc(pst_adux,
						       dev,
						       ADUX1020_INPUT1_NAME);
	if (!pst_adux->gest_input) {
		error = -ENOMEM;
		goto err_free_irq;
	}
	pst_adux->prox_input = adux1020_inputdev_alloc(pst_adux,
							dev,
							ADUX1020_INPUT2_NAME);
	if (!pst_adux->prox_input) {
		error = -ENOMEM;
		goto err_free_gesture_dev;
	}

	/* Registering input device */
	error = adux1020_gesture_inputdev_reg(pst_adux->gest_input);
	if (error < 0)
		goto err_free_proximity_dev;
	error = adux1020_proximity_inputdev_reg(pst_adux->prox_input);
	if (error < 0)
		goto err_unregister_gesture_dev;

	/* Sysfs attribute is create under device name gesture_sensor
	under class name adux_sensor. The vendor/developer can choose their
	own method to declare the sysfs path according to their framework
	*/
	error = adux1020_sysfs_create(pst_adux, adux_sensor_attrs);
	if (error < 0) {
		adux1020_sysfs_remove(pst_adux, adux_sensor_attrs);
		goto err_unregister_proximity_dev;
	}

#ifdef PC_TOOL_SUPPORT
	/*PC TOOL TEST: this should be removed during production*/
	adux1020_sysfs_link_create(pst_adux);
#else
#endif
	//pst_adux->adux1020_fb_notif.notifier_call = fb_notifier_callback;
	//if (fb_register_client(&pst_adux->adux1020_fb_notif))
	//	ADUX1020_dbg("******* FB registration failed\n");

#ifdef ADUX_STAT
	memset(&pst_adux->stats, 0, sizeof(pst_adux->stats));
	adux1020_stat_reset(pst_adux);
#else
#endif
	adux1020_enable_irq(pst_adux);
	return pst_adux;

err_unregister_proximity_dev:
	adux1020_proximity_inputdev_unreg(pst_adux->prox_input);
err_unregister_gesture_dev:
	adux1020_gesture_inputdev_unreg(pst_adux->gest_input);
err_free_proximity_dev:
	input_free_device(pst_adux->prox_input);
err_free_gesture_dev:
	input_free_device(pst_adux->gest_input);
err_free_irq:
	free_irq(pst_adux->irq, pst_adux);
err_free_mem:
	kfree(pst_adux->runtime_pdata);
	kfree(pst_adux);
err_free_pdata:
#ifdef CONFIG_OF
        kfree(plat_data);
#endif
err_out:
	return ERR_PTR(error);

}

/**
  Used to remove device.
  This function is used to remove ADUX1020 from the system by unreg ADUX1020
  @param pst_adux The chip Structure
  @return void Nothing returned
 */
static void
adux1020_remove(struct adux1020 *pst_adux)
{
	/*adux1020 remove called */
	ADUX1020_info(" %s\n", __func__);
	free_irq(pst_adux->irq, pst_adux);
	adux1020_enter_idle(pst_adux);
	//fb_unregister_client(&pst_adux->adux1020_fb_notif);
	flush_work(&pst_adux->work);
#ifdef PC_TOOL_SUPPORT
	/*PC TOOL TEST: this should be removed during production*/
	adux1020_sysfs_link_destroy(pst_adux);
#else
#endif
	adux1020_sysfs_remove(pst_adux, adux_sensor_attrs);
	adux1020_proximity_inputdev_unreg(pst_adux->prox_input);
	adux1020_gesture_inputdev_unreg(pst_adux->gest_input);
	kfree(pst_adux->runtime_pdata);
	kfree(pst_adux);
}

/*#ifndef CONFIG_HAS_EARLYSUSPEND*/
/**	@see adux1020_i2c_suspend
 *	@brief This API is used to suspend the device
 *	@param  *dev the device structure
  *	@return static s32
 */
static s32
adux1020_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adux1020 *pst_adux = i2c_get_clientdata(client);

	ADUX1020_dbg(" %s\n", __func__);
	adux1020_suspend_notifier(pst_adux);

	disable_irq(client->irq);
	if ((device_may_wakeup(&client->dev)) &&	\
		(PROXIMITY_I == pst_adux->mode))
		enable_irq_wake(client->irq);
	return 0;
}

/**	static s32 adux1020_i2c_resume (struct i2c_client *client)
 *	@brief This API is used to resume the device from sleep mode
 *	@param  *dev the device structure
 *	@return static s32
 */
static s32
adux1020_i2c_resume(struct device *dev)
{
	/*adux1020_resume(dev);
	   return 0; */
	struct i2c_client *client = to_i2c_client(dev);
	struct adux1020 *pst_adux = i2c_get_clientdata(client);

	ADUX1020_dbg(" %s\n", __func__);
	if ((device_may_wakeup(&client->dev)) &&	\
		(PROXIMITY_I == pst_adux->mode))
		disable_irq_wake(client->irq);
	enable_irq(client->irq);

	adux1020_resume_notifier(pst_adux);

	return 0;
}
/*#else
#define adux1020_i2c_suspend NULL
#define adux1020_i2c_resume NULL
#endif*/

/**     @fn static s32 adux1020_i2c_read(struct i2c_client  *client,
  u16 reg,
  u32 len,
  u16 *data,
  u32 endian)
 *      @brief This API is used to multi read
 *      @param *client store the pointer of i2c client device
 *      @param reg store the register value
 *      @param len store the length of word to read
 *      @param *data store the data pointer
 *      @param endian store the type of endian
 *      @return static s32
 */
static s32
adux1020_i2c_read(struct i2c_client *client,
		  u16 reg, u32 len, u16 *data, u32 endian)
{
	struct i2c_msg msg[2];
	u8 block_data[2];
	s32 ret, icnt;
	u8 retrycnt = 0;
	u8 *_reg = (u8 *)&reg;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;

	if (_reg[1] < ADUX_ADDRESS_RANGE) {
		msg[0].len = ADUX1020_REG_SIZE_1BYTE;
		block_data[0] = *_reg;
	} else {
		msg[0].len = ADUX1020_REG_SIZE_BYTES;
		switch (endian) {
		default:
		case ADUX1020_BIG_ENDIAN:
			reg = cpu_to_be16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		case ADUX1020_LITTLE_ENDIAN:
			reg = cpu_to_le16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		}
	}

	msg[0].buf = (s8 *)&block_data;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = len * ADUX1020_REG_SIZE_BYTES;
	msg[1].buf = (s8 *)data;

	do {
		ret = i2c_transfer(client->adapter, msg, MSG_EXECUTED);
		if (ret != MSG_EXECUTED)
			retrycnt++;
	} while ((ret != MSG_EXECUTED) && (retrycnt <= RETRY_CNT));

	if (ret != MSG_EXECUTED) {
		dev_err(&client->dev, "I2C read err (%d) reg: 0x%X len: %d\n",
			ret, reg, len);
		return -EIO;
	}
	switch (endian) {
	default:
	case ADUX1020_BIG_ENDIAN:
		for (icnt = 0; icnt < len; icnt++)
			data[icnt] = be16_to_cpu(data[icnt]);
		break;
	case ADUX1020_LITTLE_ENDIAN:
		for (icnt = 0; icnt < len; icnt++)
			data[icnt] = le16_to_cpu(data[icnt]);
		break;
	}
	return len;
}

/**
This function is used for  i2c write of ADUX1020 chip
@param *client store the pointer of i2c client device
@param reg  register to read
@param data store the 16bits data to write
@param endian little or big endian
 *      @return static s32 len read length
*/
static s32
adux1020_i2c_write(struct i2c_client *client, u16 reg, u16 data, u32 endian)
{
	s32 ret;
	u8 block_data[MAX_DATA_CNT];
	u8 retrycnt = 0;
	u8 *_reg = (u8 *)&reg;
	u16 value = 0;
	u32 size = 0;

	if (_reg[1] < ADUX_ADDRESS_RANGE) {
		size = ADUX1020_REG_SIZE_1BYTE;
		block_data[0] = *_reg;

	} else {
		size = ADUX1020_REG_SIZE_BYTES;
		switch (endian) {
		default:
		case ADUX1020_BIG_ENDIAN:
			reg = cpu_to_be16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		case ADUX1020_LITTLE_ENDIAN:
			reg = cpu_to_le16(reg);
			memcpy(block_data, &reg, DATA_SIZE);
			break;
		}
	}

	switch (endian) {
	default:
	case ADUX1020_BIG_ENDIAN:
		value = cpu_to_be16(data);
		memcpy(&block_data[size], &value, DATA_SIZE);
		value = 0;
		break;
	case ADUX1020_LITTLE_ENDIAN:
		value = cpu_to_le16(data);
		memcpy(&block_data[size], &value, DATA_SIZE);
		value = 0;
		break;
	}

	do {
		ret = i2c_master_send(client, (s8 *)block_data,
				      (size /*address */  +
				       ADUX1020_REG_SIZE_BYTES));
		if (ret < 0)
			retrycnt++;
	} while ((ret < 0) && (retrycnt <= RETRY_CNT));

	if (ret < 0) {
		dev_err(&client->dev,
			"I2C write error reg: 0x%x len: ox%x\n",
			reg, WORD_SIZE);
		return ret;
	}
	return ADUX1020_REG_SIZE_BYTES;

}

/**     @see struct ad7166_bus_ops
 *      @note  Bus Operation Table
 */
static const struct adux_bus_ops adux1020_i2c_bus_ops = {
	.read = adux1020_i2c_read,
	.write = adux1020_i2c_write,
};

/**
Probes the Device based on ID using i2c.
@param client The Client to be probed
@param id the Client id
@return s32 0 on success ,PTR_ERR(chip) on error
@see adux1020_i2c_read
*/
static s32 __devinit
adux1020_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/*struct adux1020 *pst_adux = NULL; */
	void *pst_adux = NULL;
	s32 ret = 0;

	ADUX1020_info(" %s\n", __func__);

	/* check whether i2c support smbus protocol as we need to
	   set i2c bus mode during start up */
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "SMBUS Word Data not Supported\n");
		return -EIO;	/* I/O error */
	}

	pst_adux = (struct adux1020 *)adux1020_probe(&(client->dev),
						     id,
						     client->irq,
						     &adux1020_i2c_bus_ops);
	if (IS_ERR(pst_adux)) {
		ret = -ENOMEM;	/* out of memory */
		return ret;
	}

	device_init_wakeup(&client->dev, 1);
	i2c_set_clientdata(client, (struct adux1020 *)pst_adux);
	return 0;
}

/**
This function is used to remove the I2C client for its existance in the system
@param client The Client to be removed
@return static s32 0 on success ,PTR_ERR(chip) on error
*/
static s32 __devexit
adux1020_i2c_remove(struct i2c_client *client)
{
	struct adux1020 *pst_adux = i2c_get_clientdata(client);
	ADUX1020_info(" %s\n", __func__);
	device_init_wakeup(&client->dev, 0);
	adux1020_remove(pst_adux);
	i2c_set_clientdata(client, NULL);
	return 0;
}

/**
  This table tell which framework it supported
  @brief the name has to get matched to the board configuration file setup
 */
static struct i2c_device_id adux_id[] = {
	{"adux1020", 0},
	{"ADUX_SENSOR", 0},
	{}
};

static const struct dev_pm_ops adux_pm_ops = {
/*#ifndef CONFIG_HAS_EARLYSUSPEND*/
	.resume = adux1020_i2c_resume,
	.suspend = adux1020_i2c_suspend,
/*#endif*/
};

/**
  i2c operation structure
 */
struct i2c_driver adux1020_i2c_driver = {
	.driver = {
		   .name = "adux1020",
		   .owner = THIS_MODULE,
		   .pm = &adux_pm_ops,
		   },
	.probe = adux1020_i2c_probe,
	.remove = adux1020_i2c_remove,
	.id_table = adux_id,
};

/**
  The adux1020 module start here.
  s32 adux1020_start( void)
  @brief this is used to initiate I2C driver
@return s32
 */
static s32 __init
adux1020_start(void)
{
	s32 ret;
	ADUX1020_info(" %s\n", __func__);

	ret = i2c_add_driver(&adux1020_i2c_driver);
	if (!ret) {
		/*print i2c register successfully */
		ADUX1020_info(" i2c register successfully\n");
	} else {
		/*print i2c register failed */
		ADUX1020_dbg(" i2c register failed\n");
	}
	return ret;
}

/**
  The adux1020 module stop here.
  void adux1020_stop( void)
  @brief this is used to de register and exit I2C driver
  @return void
 */
static void __exit
adux1020_stop(void)
{
	ADUX1020_dbg(" %s\n", __func__);
	i2c_del_driver(&adux1020_i2c_driver);
}

module_init(adux1020_start);
module_exit(adux1020_stop);
MODULE_DESCRIPTION("ANALOG DEVICES - ADUX1020");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("");
