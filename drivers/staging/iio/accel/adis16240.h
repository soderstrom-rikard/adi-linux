#ifndef SPI_ADIS16240_H_
#define SPI_ADIS16240_H_

#define ADIS16240_STARTUP_DELAY	220 /* ms */

#define ADIS16240_READ_REG(a)    a
#define ADIS16240_WRITE_REG(a) ((a) | 0x80)

#define ADIS16240_FLASH_CNT      0x00 /* Flash memory write count */
#define ADIS16240_SUPPLY_OUT     0x02 /* Output, power supply */
#define ADIS16240_XACCL_OUT      0x04 /* Output, x-axis accelerometer */
#define ADIS16240_YACCL_OUT      0x06 /* Output, y-axis accelerometer */
#define ADIS16240_ZACCL_OUT      0x08 /* Output, z-axis accelerometer */
#define ADIS16240_AUX_ADC        0x0A /* Output, auxiliary ADC input */
#define ADIS16240_TEMP_OUT       0x0C /* Output, temperature */
#define ADIS16240_XPEAK_OUT      0x0E /* Output, x-axis acceleration peak */
#define ADIS16240_YPEAK_OUT      0x10 /* Output, y-axis acceleration peak */
#define ADIS16240_ZPEAK_OUT      0x12 /* Output, z-axis acceleration peak */
#define ADIS16240_XYZPEAK_OUT    0x14 /* Output, sum-of-squares acceleration peak */
#define ADIS16240_CAPT_BUF1      0x16 /* Output, Capture Buffer 1, X and Y acceleration */
#define ADIS16240_CAPT_BUF2      0x18 /* Output, Capture Buffer 2, Z acceleration */
#define ADIS16240_DIAG_STAT      0x1A /* Diagnostic, error flags */
#define ADIS16240_EVNT_CNTR      0x1C /* Diagnostic, event counter */
#define ADIS16240_CHK_SUM        0x1E /* Diagnostic, check sum value from firmware test */
#define ADIS16240_XACCL_OFF      0x20 /* Calibration, x-axis acceleration offset adjustment */
#define ADIS16240_YACCL_OFF      0x22 /* Calibration, y-axis acceleration offset adjustment */
#define ADIS16240_ZACCL_OFF      0x24 /* Calibration, z-axis acceleration offset adjustment */
#define ADIS16240_CLK_TIME       0x2E /* Clock, hour and minute */
#define ADIS16240_CLK_DATE       0x30 /* Clock, month and day */
#define ADIS16240_CLK_YEAR       0x32 /* Clock, year */
#define ADIS16240_WAKE_TIME      0x34 /* Wake-up setting, hour and minute */
#define ADIS16240_WAKE_DATE      0x36 /* Wake-up setting, month and day */
#define ADIS16240_ALM_MAG1       0x38 /* Alarm 1 amplitude threshold */
#define ADIS16240_ALM_MAG2       0x3A /* Alarm 2 amplitude threshold */
#define ADIS16240_ALM_CTRL       0x3C /* Alarm control */
#define ADIS16240_XTRIG_CTRL     0x3E /* Capture, external trigger control */
#define ADIS16240_CAPT_PNTR      0x40 /* Capture, address pointer */
#define ADIS16240_CAPT_CTRL      0x42 /* Capture, configuration and control */
#define ADIS16240_GPIO_CTRL      0x44 /* General-purpose digital input/output control */
#define ADIS16240_MSC_CTRL       0x46 /* Miscellaneous control */
#define ADIS16240_SMPL_PRD       0x48 /* Internal sample period (rate) control */
#define ADIS16240_GLOB_CMD       0x4A /* System command */

/* MSC_CTRL */
#define ADIS16240_MSC_CTRL_XYZPEAK_OUT_EN	(1 << 15) /* Enables sum-of-squares output (XYZPEAK_OUT) */
#define ADIS16240_MSC_CTRL_X_Y_ZPEAK_OUT_EN	(1 << 14) /* Enables peak tracking output
							  (XPEAK_OUT, YPEAK_OUT, and ZPEAK_OUT) */
#define ADIS16240_MSC_CTRL_SELF_TEST_EN	        (1 << 8)  /* Self-test enable: 1 = apply electrostatic force,
							  0 = disabled */
#define ADIS16240_MSC_CTRL_DATA_RDY_EN	        (1 << 2)  /* Data-ready enable: 1 = enabled, 0 = disabled */
#define ADIS16240_MSC_CTRL_ACTIVE_HIGH	        (1 << 1)  /* Data-ready polarity: 1 = active high, 0 = active low */
#define ADIS16240_MSC_CTRL_DATA_RDY_DIO2	(1 << 0)  /* Data-ready line selection: 1 = DIO2, 0 = DIO1 */

/* DIAG_STAT */
#define ADIS16240_DIAG_STAT_ALARM2      (1<<9) /* Alarm 2 status: 1 = alarm active, 0 = alarm inactive */
#define ADIS16240_DIAG_STAT_ALARM1      (1<<8) /* Alarm 1 status: 1 = alarm active, 0 = alarm inactive */
#define ADIS16240_DIAG_STAT_CPT_BUF_FUL (1<<7) /* Capture buffer full: 1 = capture buffer is full */
#define ADIS16240_DIAG_STAT_CHKSUM      (1<<6) /* Flash test, checksum flag: 1 = mismatch, 0 = match */
#define ADIS16240_DIAG_STAT_PWRON_FAIL  (1<<5) /* Power-on, self-test flag: 1 = failure, 0 = pass */
#define ADIS16240_DIAG_STAT_PWRON_BUSY  (1<<4) /* Power-on self-test: 1 = in-progress, 0 = complete */
#define ADIS16240_DIAG_STAT_SPI_FAIL	(1<<3) /* SPI communications failure */
#define ADIS16240_DIAG_STAT_FLASH_UPT	(1<<2) /* Flash update failure */
#define ADIS16240_DIAG_STAT_POWER_HIGH	(1<<1) /* Power supply above 3.625 V */
#define ADIS16240_DIAG_STAT_POWER_LOW	(1<<0) /* Power supply below 3.15 V */

/* GLOB_CMD */
#define ADIS16240_GLOB_CMD_RESUME	(1<<8)
#define ADIS16240_GLOB_CMD_SW_RESET	(1<<7)
#define ADIS16240_GLOB_CMD_STANDBY	(1<<2)

#define ADIS16240_MAX_TX 24
#define ADIS16240_MAX_RX 24

#define ADIS16240_SPI_SLOW	(u32)(300 * 1000)
#define ADIS16240_SPI_BURST	(u32)(1000 * 1000)
#define ADIS16240_SPI_FAST	(u32)(2000 * 1000)

/**
 * struct adis16240_state - device instance specific data
 * @us:			actual spi_device
 * @work_trigger_to_ring: bh for triggered event handling
 * @work_cont_thresh: CLEAN
 * @inter:		used to check if new interrupt has been triggered
 * @last_timestamp:	passing timestamp from th to bh of interrupt handler
 * @indio_dev:		industrial I/O device structure
 * @trig:		data ready trigger registered with iio
 * @tx:			transmit buffer
 * @rx:			recieve buffer
 * @buf_lock:		mutex to protect tx and rx
 **/
struct adis16240_state {
	struct spi_device		*us;
	struct work_struct		work_trigger_to_ring;
	struct iio_work_cont		work_cont_thresh;
	s64				last_timestamp;
	struct iio_dev			*indio_dev;
	struct iio_trigger		*trig;
	u8				*tx;
	u8				*rx;
	struct mutex			buf_lock;
};

int adis16240_spi_write_reg_8(struct device *dev,
			      u8 reg_address,
			      u8 val);

int adis16240_spi_read_burst(struct device *dev, u8 *rx);

int adis16240_spi_read_sequence(struct device *dev,
				      u8 *tx, u8 *rx, int num);

int adis16240_set_irq(struct device *dev, bool enable);

int adis16240_reset(struct device *dev);

int adis16240_stop_device(struct device *dev);

int adis16240_check_status(struct device *dev);

#ifdef CONFIG_IIO_RING_BUFFER
/* At the moment triggers are only used for ring buffer
 * filling. This may change!
 */

enum adis16240_scan {
	ADIS16240_SCAN_XY,
	ADIS16240_SCAN_Z,
	ADIS16240_SCAN_XYZ,
};

void adis16240_remove_trigger(struct iio_dev *indio_dev);
int adis16240_probe_trigger(struct iio_dev *indio_dev);

ssize_t adis16240_read_data_from_ring(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);


int adis16240_configure_ring(struct iio_dev *indio_dev);
void adis16240_unconfigure_ring(struct iio_dev *indio_dev);

int adis16240_initialize_ring(struct iio_ring_buffer *ring);
void adis16240_uninitialize_ring(struct iio_ring_buffer *ring);
#else /* CONFIG_IIO_RING_BUFFER */

static inline void adis16240_remove_trigger(struct iio_dev *indio_dev)
{
}

static inline int adis16240_probe_trigger(struct iio_dev *indio_dev)
{
	return 0;
}

static inline ssize_t
adis16240_read_data_from_ring(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return 0;
}

static int adis16240_configure_ring(struct iio_dev *indio_dev)
{
	return 0;
}

static inline void adis16240_unconfigure_ring(struct iio_dev *indio_dev)
{
}

static inline int adis16240_initialize_ring(struct iio_ring_buffer *ring)
{
	return 0;
}

static inline void adis16240_uninitialize_ring(struct iio_ring_buffer *ring)
{
}

#endif /* CONFIG_IIO_RING_BUFFER */
#endif /* SPI_ADIS16240_H_ */
