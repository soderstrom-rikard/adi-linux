#ifndef _AD7476_H_
#define  _AD7476_H_

struct ad7476_mode {
	const char	*name;
	int		numvals;
};

struct ad7476_state {
	struct iio_dev			*indio_dev;
	struct spi_device 		*spi;
	char				setupbyte;
	char				configbyte;
	const struct ad7476_chip_info	*chip_info;
	const struct ad7476_mode	*current_mode;
	struct work_struct		poll_work;
	atomic_t			protect_ring;
	struct iio_trigger		*trig;
	struct spi_transfer		xfer;
	struct spi_message		msg;
	unsigned char 			data[2];
};

#define CHIP_NAME "AD7876/7/8"

#ifdef CONFIG_IIO_RING_BUFFER

ssize_t ad7476_scan_from_ring(struct device *dev,
			       struct device_attribute *attr,
			       char *buf);
int ad7476_register_ring_funcs_and_init(struct iio_dev *indio_dev);
void ad7476_ring_cleanup(struct iio_dev *indio_dev);

int ad7476_initialize_ring(struct iio_ring_buffer *ring);
void ad7476_uninitialize_ring(struct iio_ring_buffer *ring);

#else /* CONFIG_IIO_RING_BUFFER */

static inline void ad7476_uninitialize_ring(struct iio_ring_buffer *ring)
{
};

static inline int ad7476_initialize_ring(struct iio_ring_buffer *ring)
{
	return 0;
};


static inline ssize_t ad7476_scan_from_ring(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	return 0;
};

static inline int
ad7476_register_ring_funcs_and_init(struct iio_dev *indio_dev)
{
	return 0;
};

static inline void ad7476_ring_cleanup(struct iio_dev *indio_dev) {};
#endif /* CONFIG_IIO_RING_BUFFER */
#endif /* _AD7476_H_ */
