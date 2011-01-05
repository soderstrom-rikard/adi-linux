#ifndef SPI_ADXRS450_H_
#define SPI_ADXRS450_H_

#define ADXRS450_STARTUP_DELAY	50 /* ms */

/* The MSB for the spi commands */
#define ADXRS450_SENSOR_DATA    (0x2 << 27 | 0x1)
#define ADXRS450_READ_REG(a)    ((0x20 << 26) | (a << 17) | 0x1)
#define ADXRS450_WRITE_REG(a, value) \
		((0x10 << 26) | (a << 17) | (value << 1) | 0x1)

#define ADXRS450_RATE1	0x00	/* Rate Registers */
#define ADXRS450_TEMP1	0x02	/* Temperature Registers */
#define ADXRS450_LOCST1	0x04	/* Low CST Memory Registers */
#define ADXRS450_HICST1	0x06	/* High CST Memory Registers */
#define ADXRS450_QUAD1	0x08	/* Quad Memory Registers */
#define ADXRS450_FAULT1	0x0A	/* Fault Registers */
#define ADXRS450_PID1	0x0C	/* Part ID Registers */
#define ADXRS450_SNH	0x0E	/* Serial Number Registers, 4 bytes */
#define ADXRS450_SNL	0x10
#define ADXRS450_DNC1	0x12	/* Dynamic Null Correction Registers */

#define ADXRS450_WRERR_MASK	(0x7 << 29)

#define ADXRS450_MAX_RX 1
#define ADXRS450_MAX_TX 1

#define ADXRS450_SPI_SLOW	(u32)(300 * 1000)
#define ADXRS450_SPI_FAST	(u32)(2000 * 1000)
#define ADXRS450_GET_ST(a)	((a >> 26) & 0x3)

/**
 * struct adxrs450_state - device instance specific data
 * @us:			actual spi_device
 * @indio_dev:		industrial I/O device structure
 * @tx:			transmit buffer
 * @rx:			recieve buffer
 * @buf_lock:		mutex to protect tx and rx
 **/
struct adxrs450_state {
	struct spi_device		*us;
	struct iio_dev			*indio_dev;
	u32				*tx;
	u32				*rx;
	struct mutex			buf_lock;
};

#endif /* SPI_ADXRS450_H_ */
