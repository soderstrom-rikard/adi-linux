#ifndef SPI_ADIS16209_H_
#define SPI_ADIS16209_H_

#define ADIS16209_STARTUP_DELAY	220 /* ms */

#define ADIS16209_READ_REG(a)    a
#define ADIS16209_WRITE_REG(a) ((a) | 0x80)

#define ADIS16209_FLASH_CNT      0x00 /* Flash memory write count */
#define ADIS16209_SUPPLY_OUT     0x02 /* Output, power supply */
#define ADIS16209_XACCL_OUT      0x04 /* Output, x-axis accelerometer */
#define ADIS16209_YACCL_OUT      0x06 /* Output, y-axis accelerometer */
#define ADIS16209_AUX_ADC        0x08 /* Output, auxiliary ADC input */
#define ADIS16209_TEMP_OUT       0x0A /* Output, temperature */
#define ADIS16209_XINCL_OUT      0x0C /* Output, x-axis inclination */
#define ADIS16209_YINCL_OUT      0x0E /* Output, y-axis inclination */
#define ADIS16209_ROT_OUT        0x10 /* Output, +/-180 vertical rotational position */
#define ADIS16209_XACCL_NULL     0x12 /* Calibration, x-axis acceleration offset null */
#define ADIS16209_YACCL_NULL     0x14 /* Calibration, y-axis acceleration offset null */
#define ADIS16209_XINCL_NULL     0x16 /* Calibration, x-axis inclination offset null */
#define ADIS16209_YINCL_NULL     0x18 /* Calibration, y-axis inclination offset null */
#define ADIS16209_ROT_NULL       0x1A /* Calibration, vertical rotation offset null */
#define ADIS16209_ALM_MAG1       0x20 /* Alarm 1 amplitude threshold */
#define ADIS16209_ALM_MAG2       0x22 /* Alarm 2 amplitude threshold */
#define ADIS16209_ALM_SMPL1      0x24 /* Alarm 1, sample period */
#define ADIS16209_ALM_SMPL2      0x26 /* Alarm 2, sample period */
#define ADIS16209_ALM_CTRL       0x28 /* Alarm control */
#define ADIS16209_AUX_DAC        0x30 /* Auxiliary DAC data */
#define ADIS16209_GPIO_CTRL      0x32 /* General-purpose digital input/output control */
#define ADIS16209_MSC_CTRL       0x34 /* Miscellaneous control */
#define ADIS16209_SMPL_PRD       0x36 /* Internal sample period (rate) control */
#define ADIS16209_AVG_CNT        0x38 /* Operation, filter configuration */
#define ADIS16209_SLP_CNT        0x3A /* Operation, sleep mode control */
#define ADIS16209_DIAG_STAT      0x3C /* Diagnostics, system status register */
#define ADIS16209_GLOB_CMD       0x3E /* Operation, system command register */

#define ADIS16209_OUTPUTS        8

/* MSC_CTRL */
#define ADIS16209_MSC_CTRL_PWRUP_SELF_TEST	(1 << 10) /* Self-test at power-on: 1 = disabled, 0 = enabled */
#define ADIS16209_MSC_CTRL_SELF_TEST_EN	        (1 << 8)  /* Self-test enable */
#define ADIS16209_MSC_CTRL_DATA_RDY_EN	        (1 << 2)  /* Data-ready enable: 1 = enabled, 0 = disabled */
#define ADIS16209_MSC_CTRL_ACTIVE_HIGH	        (1 << 1)  /* Data-ready polarity: 1 = active high, 0 = active low */
#define ADIS16209_MSC_CTRL_DATA_RDY_DIO2	(1 << 0)  /* Data-ready line selection: 1 = DIO2, 0 = DIO1 */

/* DIAG_STAT */
#define ADIS16209_DIAG_STAT_ALARM2        (1<<9) /* Alarm 2 status: 1 = alarm active, 0 = alarm inactive */
#define ADIS16209_DIAG_STAT_ALARM1        (1<<8) /* Alarm 1 status: 1 = alarm active, 0 = alarm inactive */
#define ADIS16209_DIAG_STAT_SELFTEST_FAIL (1<<5) /* Self-test diagnostic error flag: 1 = error condition,
						0 = normal operation */
#define ADIS16209_DIAG_STAT_SPI_FAIL	  (1<<3) /* SPI communications failure */
#define ADIS16209_DIAG_STAT_FLASH_UPT	  (1<<2) /* Flash update failure */
#define ADIS16209_DIAG_STAT_POWER_HIGH	  (1<<1) /* Power supply above 3.625 V */
#define ADIS16209_DIAG_STAT_POWER_LOW	  (1<<0) /* Power supply below 3.15 V */

/* GLOB_CMD */
#define ADIS16209_GLOB_CMD_SW_RESET	(1<<7)
#define ADIS16209_GLOB_CMD_CLEAR_STAT	(1<<4)
#define ADIS16209_GLOB_CMD_FACTORY_CAL	(1<<1)

#define ADIS16209_MAX_TX 24
#define ADIS16209_MAX_RX 24

#define ADIS16209_SPI_BURST	(u32)(1000 * 1000)
#define ADIS16209_SPI_FAST	(u32)(2000 * 1000)

/**
 * struct adis16209_state - device instance specific data
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
struct adis16209_state {
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

int adis16209_spi_write_reg_8(struct device *dev,
			      u8 reg_address,
			      u8 val);

int adis16209_spi_read_burst(struct device *dev, u8 *rx);

int adis16209_spi_read_sequence(struct device *dev,
				      u8 *tx, u8 *rx, int num);

int adis16209_set_irq(struct device *dev, bool enable);

int adis16209_reset(struct device *dev);

int adis16209_stop_device(struct device *dev);

int adis16209_check_status(struct device *dev);

#ifdef CONFIG_IIO_RING_BUFFER
enum adis16209_scan {
	ADIS16209_SCAN_SUPPLY,
	ADIS16209_SCAN_ACC_X,
	ADIS16209_SCAN_ACC_Y,
	ADIS16209_SCAN_AUX_ADC,
	ADIS16209_SCAN_TEMP,
	ADIS16209_SCAN_INCLI_X,
	ADIS16209_SCAN_INCLI_Y,
	ADIS16209_SCAN_ROT,
};

void adis16209_remove_trigger(struct iio_dev *indio_dev);
int adis16209_probe_trigger(struct iio_dev *indio_dev);

ssize_t adis16209_read_data_from_ring(struct device *dev,
				      struct device_attribute *attr,
				      char *buf);

int adis16209_configure_ring(struct iio_dev *indio_dev);
void adis16209_unconfigure_ring(struct iio_dev *indio_dev);

int adis16209_initialize_ring(struct iio_ring_buffer *ring);
void adis16209_uninitialize_ring(struct iio_ring_buffer *ring);
#else /* CONFIG_IIO_RING_BUFFER */

static inline void adis16209_remove_trigger(struct iio_dev *indio_dev)
{
}

static inline int adis16209_probe_trigger(struct iio_dev *indio_dev)
{
	return 0;
}

static inline ssize_t
adis16209_read_data_from_ring(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return 0;
}

static int adis16209_configure_ring(struct iio_dev *indio_dev)
{
	return 0;
}

static inline void adis16209_unconfigure_ring(struct iio_dev *indio_dev)
{
}

static inline int adis16209_initialize_ring(struct iio_ring_buffer *ring)
{
	return 0;
}

static inline void adis16209_uninitialize_ring(struct iio_ring_buffer *ring)
{
}

#endif /* CONFIG_IIO_RING_BUFFER */
#endif /* SPI_ADIS16209_H_ */
