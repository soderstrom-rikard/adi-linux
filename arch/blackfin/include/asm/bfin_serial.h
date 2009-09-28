/*
 * bfin_serial.h - Blackfin UART/Serial definitions
 *
 * Copyright 2006-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __BFIN_ASM_SERIAL_H__
#define __BFIN_ASM_SERIAL_H__

#include <linux/serial_core.h>
#include <mach/anomaly.h>
#include <mach/bfin_serial.h>

#if defined(CONFIG_BFIN_UART0_CTSRTS) || \
    defined(CONFIG_BFIN_UART1_CTSRTS) || \
    defined(CONFIG_BFIN_UART2_CTSRTS) || \
    defined(CONFIG_BFIN_UART3_CTSRTS)
# define CONFIG_BFIN_UART_CTSRTS
#endif

struct circ_buf;
struct timer_list;
struct work_struct;

struct bfin_serial_port {
	struct uart_port port;
	unsigned int old_status;
	int status_irq;
#ifndef BFIN_UART_BF54X_STYLE
	unsigned int lsr;
#endif
#ifdef CONFIG_SERIAL_BFIN_DMA
	int tx_done;
	int tx_count;
	struct circ_buf rx_dma_buf;
	struct timer_list rx_dma_timer;
	int rx_dma_nrows;
	unsigned int tx_dma_channel;
	unsigned int rx_dma_channel;
	struct work_struct tx_dma_workqueue;
#elif ANOMALY_05000363
	unsigned int anomaly_threshold;
#endif
#ifdef CONFIG_SERIAL_BFIN_CTSRTS
# ifdef BFIN_UART_HARD_CTSRTS
	int scts;
# endif
	int cts_pin;
	int rts_pin;
#endif
};

#ifdef BFIN_UART_BF54X_STYLE
# define OFFSET_DLL              0x00  /* Divisor Latch (Low-Byte)        */
# define OFFSET_DLH              0x04  /* Divisor Latch (High-Byte)       */
# define OFFSET_GCTL             0x08  /* Global Control Register         */
# define OFFSET_LCR              0x0C  /* Line Control Register           */
# define OFFSET_MCR              0x10  /* Modem Control Register          */
# define OFFSET_LSR              0x14  /* Line Status Register            */
# define OFFSET_MSR              0x18  /* Modem Status Register           */
# define OFFSET_SCR              0x1C  /* SCR Scratch Register            */
# define OFFSET_IER_SET          0x20  /* Set Interrupt Enable Register   */
# define OFFSET_IER_CLEAR        0x24  /* Clear Interrupt Enable Register */
# define OFFSET_THR              0x28  /* Transmit Holding register       */
# define OFFSET_RBR              0x2C  /* Receive Buffer register         */
#else /* BF533 style */
# define OFFSET_THR              0x00  /* Transmit Holding register         */
# define OFFSET_RBR              0x00  /* Receive Buffer register           */
# define OFFSET_DLL              0x00  /* Divisor Latch (Low-Byte)          */
# define OFFSET_DLH              0x04  /* Divisor Latch (High-Byte)         */
# define OFFSET_IER              0x04  /* Interrupt Enable Register         */
# define OFFSET_IIR              0x08  /* Interrupt Identification Register */
# define OFFSET_LCR              0x0C  /* Line Control Register             */
# define OFFSET_MCR              0x10  /* Modem Control Register            */
# define OFFSET_LSR              0x14  /* Line Status Register              */
# define OFFSET_MSR              0x18  /* Modem Status Register             */
# define OFFSET_SCR              0x1C  /* SCR Scratch Register              */
# define OFFSET_GCTL             0x24  /* Global Control Register           */
/* code should not need IIR, so force build error if they use it */
# undef OFFSET_IIR
#endif

#define UART_GET_CHAR(uart)      bfin_read16((uart)->port.membase + OFFSET_RBR)
#define UART_GET_DLL(uart)       bfin_read16((uart)->port.membase + OFFSET_DLL)
#define UART_GET_DLH(uart)       bfin_read16((uart)->port.membase + OFFSET_DLH)
#define UART_GET_GCTL(uart)      bfin_read16((uart)->port.membase + OFFSET_GCTL)
#define UART_GET_LCR(uart)       bfin_read16((uart)->port.membase + OFFSET_LCR)
#define UART_GET_MCR(uart)       bfin_read16((uart)->port.membase + OFFSET_MCR)
#define UART_GET_MSR(uart)       bfin_read16((uart)->port.membase + OFFSET_MSR)

#define UART_PUT_CHAR(uart, v)   bfin_write16((uart)->port.membase + OFFSET_THR, v)
#define UART_PUT_DLL(uart, v)    bfin_write16((uart)->port.membase + OFFSET_DLL, v)
#define UART_PUT_DLH(uart, v)    bfin_write16((uart)->port.membase + OFFSET_DLH, v)
#define UART_PUT_GCTL(uart, v)   bfin_write16((uart)->port.membase + OFFSET_GCTL, v)
#define UART_PUT_LCR(uart, v)    bfin_write16((uart)->port.membase + OFFSET_LCR, v)
#define UART_PUT_MCR(uart, v)    bfin_write16((uart)->port.membase + OFFSET_MCR, v)

#ifdef BFIN_UART_BF54X_STYLE

#define UART_CLEAR_IER(uart, v)  bfin_write16((uart)->port.membase + OFFSET_IER_CLEAR, v)
#define UART_GET_IER(uart)       bfin_read16((uart)->port.membase + OFFSET_IER_SET)
#define UART_SET_IER(uart, v)    bfin_write16((uart)->port.membase + OFFSET_IER_SET, v)

#define UART_CLEAR_DLAB(uart)    /* MMRs not muxed on BF54x */
#define UART_SET_DLAB(uart)      /* MMRs not muxed on BF54x */

#define UART_CLEAR_LSR(uart)     bfin_write16((uart)->port.membase + OFFSET_LSR, -1)
#define UART_GET_LSR(uart)       bfin_read16((uart)->port.membase + OFFSET_LSR)
#define UART_PUT_LSR(uart, v)    bfin_write16((uart)->port.membase + OFFSET_LSR, v)

/* This handles hard CTS/RTS */
#define BFIN_UART_CTSRTS_HARD
#define UART_CLEAR_SCTS(uart)   bfin_write16(((uart)->port.membase + OFFSET_MSR), SCTS)
#define UART_GET_CTS(x)         (UART_GET_MSR(x) & CTS)
#define UART_DISABLE_RTS(x)     UART_PUT_MCR(x, UART_GET_MCR(x) & ~(ARTS | MRTS))
#define UART_ENABLE_RTS(x)      UART_PUT_MCR(x, UART_GET_MCR(x) | MRTS | ARTS)
#define UART_ENABLE_INTS(x, v)  UART_SET_IER(x, v)
#define UART_DISABLE_INTS(x)    UART_CLEAR_IER(x, 0xF)

#else /* BF533 style */

#define UART_CLEAR_IER(uart, v)  UART_PUT_IER(uart, UART_GET_IER(uart) & ~(v))
#define UART_GET_IER(uart)       bfin_read16((uart)->port.membase + OFFSET_IER)
#define UART_PUT_IER(uart, v)    bfin_write16((uart)->port.membase + OFFSET_IER, v)
#define UART_SET_IER(uart, v)    UART_PUT_IER(uart, UART_GET_IER(uart) | (v))

#define UART_CLEAR_DLAB(uart)   do { UART_PUT_LCR(uart, UART_GET_LCR(uart) & ~DLAB); SSYNC(); } while (0)
#define UART_SET_DLAB(uart)     do { UART_PUT_LCR(uart, UART_GET_LCR(uart) | DLAB); SSYNC(); } while (0)

/* The hardware clears the LSR bits upon read, so we need to cache
 * some of the more fun bits in software so they don't get lost
 * when checking the LSR in other code paths (TX).
 */
static inline void UART_CLEAR_LSR(struct bfin_serial_port *uart)
{
	uart->lsr = 0;
	bfin_write16(uart->port.membase + OFFSET_LSR, -1);
}
static inline unsigned int UART_GET_LSR(struct bfin_serial_port *uart)
{
	unsigned int lsr = bfin_read16(uart->port.membase + OFFSET_LSR);
	uart->lsr |= (lsr & (BI|FE|PE|OE));
	return lsr | uart->lsr;
}
static inline void UART_PUT_LSR(struct bfin_serial_port *uart, uint16_t val)
{
	uart->lsr &= ~val;
}

/* This handles soft CTS/RTS */
#define UART_GET_CTS(x)        gpio_get_value(x->cts_pin)
#define UART_DISABLE_RTS(x)    gpio_set_value(x->rts_pin, 1)
#define UART_ENABLE_RTS(x)     gpio_set_value(x->rts_pin, 0)
#define UART_ENABLE_INTS(x, v) UART_PUT_IER(x, v)
#define UART_DISABLE_INTS(x)   UART_PUT_IER(x, 0)

#endif

#ifndef BFIN_UART_TX_FIFO_SIZE
# define BFIN_UART_TX_FIFO_SIZE 2
#endif

#endif /* __BFIN_ASM_SERIAL_H__ */
