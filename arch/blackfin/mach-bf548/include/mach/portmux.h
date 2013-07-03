/*
 * Copyright 2007-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _MACH_PORTMUX_H_
#define _MACH_PORTMUX_H_

#define P_SPORT2_TFS	(P_DEFINED | P_IDENT(GPIO_PA0) | P_FUNCT(0))
#define P_SPORT2_DTSEC	(P_DEFINED | P_IDENT(GPIO_PA1) | P_FUNCT(0))
#define P_SPORT2_DTPRI	(P_DEFINED | P_IDENT(GPIO_PA2) | P_FUNCT(0))
#define P_SPORT2_TSCLK	(P_DEFINED | P_IDENT(GPIO_PA3) | P_FUNCT(0))
#define P_SPORT2_RFS	(P_DEFINED | P_IDENT(GPIO_PA4) | P_FUNCT(0))
#define P_SPORT2_DRSEC	(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(0))
#define P_SPORT2_DRPRI	(P_DEFINED | P_IDENT(GPIO_PA6) | P_FUNCT(0))
#define P_SPORT2_RSCLK	(P_DEFINED | P_IDENT(GPIO_PA7) | P_FUNCT(0))
#define P_SPORT3_TFS	(P_DEFINED | P_IDENT(GPIO_PA8) | P_FUNCT(0))
#define P_SPORT3_DTSEC	(P_DEFINED | P_IDENT(GPIO_PA9) | P_FUNCT(0))
#define P_SPORT3_DTPRI	(P_DEFINED | P_IDENT(GPIO_PA10) | P_FUNCT(0))
#define P_SPORT3_TSCLK	(P_DEFINED | P_IDENT(GPIO_PA11) | P_FUNCT(0))
#define P_SPORT3_RFS	(P_DEFINED | P_IDENT(GPIO_PA12) | P_FUNCT(0))
#define P_SPORT3_DRSEC	(P_DEFINED | P_IDENT(GPIO_PA13) | P_FUNCT(0))
#define P_SPORT3_DRPRI	(P_DEFINED | P_IDENT(GPIO_PA14) | P_FUNCT(0))
#define P_SPORT3_RSCLK	(P_DEFINED | P_IDENT(GPIO_PA15) | P_FUNCT(0))
#define P_TMR4	(P_DEFINED | P_IDENT(GPIO_PA1) | P_FUNCT(1))
#define P_TMR5	(P_DEFINED | P_IDENT(GPIO_PA5) | P_FUNCT(1))
#define P_TMR6	(P_DEFINED | P_IDENT(GPIO_PA9) | P_FUNCT(1))
#define P_TMR7	(P_DEFINED | P_IDENT(GPIO_PA13) | P_FUNCT(1))

#define P_TWI1_SCL	(P_DEFINED | P_IDENT(GPIO_PB0) | P_FUNCT(0))
#define P_TWI1_SDA	(P_DEFINED | P_IDENT(GPIO_PB1) | P_FUNCT(0))
#define P_UART3_RTS	(P_DEFINED | P_IDENT(GPIO_PB2) | P_FUNCT(0))
#define P_UART3_CTS	(P_DEFINED | P_IDENT(GPIO_PB3) | P_FUNCT(0))
#define P_UART2_TX	(P_DEFINED | P_IDENT(GPIO_PB4) | P_FUNCT(0))
#define P_UART2_RX	(P_DEFINED | P_IDENT(GPIO_PB5) | P_FUNCT(0))
#define P_UART3_TX	(P_DEFINED | P_IDENT(GPIO_PB6) | P_FUNCT(0))
#define P_UART3_RX	(P_DEFINED | P_IDENT(GPIO_PB7) | P_FUNCT(0))
#define P_SPI2_SS	(P_DEFINED | P_IDENT(GPIO_PB8) | P_FUNCT(0))
#define P_SPI2_SSEL1	(P_DEFINED | P_IDENT(GPIO_PB9) | P_FUNCT(0))
#define P_SPI2_SSEL2	(P_DEFINED | P_IDENT(GPIO_PB10) | P_FUNCT(0))
#define P_SPI2_SSEL3	(P_DEFINED | P_IDENT(GPIO_PB11) | P_FUNCT(0))
#define P_SPI2_SCK	(P_DEFINED | P_IDENT(GPIO_PB12) | P_FUNCT(0))
#define P_SPI2_MOSI	(P_DEFINED | P_IDENT(GPIO_PB13) | P_FUNCT(0))
#define P_SPI2_MISO	(P_DEFINED | P_IDENT(GPIO_PB14) | P_FUNCT(0))
#define P_TMR0	(P_DEFINED | P_IDENT(GPIO_PB8) | P_FUNCT(1))
#define P_TMR1	(P_DEFINED | P_IDENT(GPIO_PB9) | P_FUNCT(1))
#define P_TMR2	(P_DEFINED | P_IDENT(GPIO_PB10) | P_FUNCT(1))
#define P_TMR3	(P_DEFINED | P_IDENT(GPIO_PB11) | P_FUNCT(1))

#define P_SPORT0_TFS	(P_DEFINED | P_IDENT(GPIO_PC0) | P_FUNCT(0))
#define P_SPORT0_DTSEC	(P_DEFINED | P_IDENT(GPIO_PC1) | P_FUNCT(0))
#define P_SPORT0_DTPRI	(P_DEFINED | P_IDENT(GPIO_PC2) | P_FUNCT(0))
#define P_SPORT0_TSCLK	(P_DEFINED | P_IDENT(GPIO_PC3) | P_FUNCT(0))
#define P_SPORT0_RFS	(P_DEFINED | P_IDENT(GPIO_PC4) | P_FUNCT(0))
#define P_SPORT0_DRSEC	(P_DEFINED | P_IDENT(GPIO_PC5) | P_FUNCT(0))
#define P_SPORT0_DRPRI	(P_DEFINED | P_IDENT(GPIO_PC6) | P_FUNCT(0))
#define P_SPORT0_RSCLK	(P_DEFINED | P_IDENT(GPIO_PC7) | P_FUNCT(0))
#define P_SD_D0	(P_DEFINED | P_IDENT(GPIO_PC8) | P_FUNCT(0))
#define P_SD_D1	(P_DEFINED | P_IDENT(GPIO_PC9) | P_FUNCT(0))
#define P_SD_D2	(P_DEFINED | P_IDENT(GPIO_PC10) | P_FUNCT(0))
#define P_SD_D3	(P_DEFINED | P_IDENT(GPIO_PC11) | P_FUNCT(0))
#define P_SD_CLK	(P_DEFINED | P_IDENT(GPIO_PC12) | P_FUNCT(0))
#define P_SD_CMD	(P_DEFINED | P_IDENT(GPIO_PC13) | P_FUNCT(0))
#define P_MMCLK	(P_DEFINED | P_IDENT(GPIO_PC1) | P_FUNCT(1))
#define P_MBCLK	(P_DEFINED | P_IDENT(GPIO_PC5) | P_FUNCT(1))

#define P_PPI1_D0	(P_DEFINED | P_IDENT(GPIO_PD0) | P_FUNCT(0))
#define P_PPI1_D1	(P_DEFINED | P_IDENT(GPIO_PD1) | P_FUNCT(0))
#define P_PPI1_D2	(P_DEFINED | P_IDENT(GPIO_PD2) | P_FUNCT(0))
#define P_PPI1_D3	(P_DEFINED | P_IDENT(GPIO_PD3) | P_FUNCT(0))
#define P_PPI1_D4	(P_DEFINED | P_IDENT(GPIO_PD4) | P_FUNCT(0))
#define P_PPI1_D5	(P_DEFINED | P_IDENT(GPIO_PD5) | P_FUNCT(0))
#define P_PPI1_D6	(P_DEFINED | P_IDENT(GPIO_PD6) | P_FUNCT(0))
#define P_PPI1_D7	(P_DEFINED | P_IDENT(GPIO_PD7) | P_FUNCT(0))
#define P_PPI1_D8	(P_DEFINED | P_IDENT(GPIO_PD8) | P_FUNCT(0))
#define P_PPI1_D9	(P_DEFINED | P_IDENT(GPIO_PD9) | P_FUNCT(0))
#define P_PPI1_D10	(P_DEFINED | P_IDENT(GPIO_PD10) | P_FUNCT(0))
#define P_PPI1_D11	(P_DEFINED | P_IDENT(GPIO_PD11) | P_FUNCT(0))
#define P_PPI1_D12	(P_DEFINED | P_IDENT(GPIO_PD12) | P_FUNCT(0))
#define P_PPI1_D13	(P_DEFINED | P_IDENT(GPIO_PD13) | P_FUNCT(0))
#define P_PPI1_D14	(P_DEFINED | P_IDENT(GPIO_PD14) | P_FUNCT(0))
#define P_PPI1_D15	(P_DEFINED | P_IDENT(GPIO_PD15) | P_FUNCT(0))

#define P_HOST_D8	(P_DEFINED | P_IDENT(GPIO_PD0) | P_FUNCT(1))
#define P_HOST_D9	(P_DEFINED | P_IDENT(GPIO_PD1) | P_FUNCT(1))
#define P_HOST_D10	(P_DEFINED | P_IDENT(GPIO_PD2) | P_FUNCT(1))
#define P_HOST_D11	(P_DEFINED | P_IDENT(GPIO_PD3) | P_FUNCT(1))
#define P_HOST_D12	(P_DEFINED | P_IDENT(GPIO_PD4) | P_FUNCT(1))
#define P_HOST_D13	(P_DEFINED | P_IDENT(GPIO_PD5) | P_FUNCT(1))
#define P_HOST_D14	(P_DEFINED | P_IDENT(GPIO_PD6) | P_FUNCT(1))
#define P_HOST_D15	(P_DEFINED | P_IDENT(GPIO_PD7) | P_FUNCT(1))
#define P_HOST_D0	(P_DEFINED | P_IDENT(GPIO_PD8) | P_FUNCT(1))
#define P_HOST_D1	(P_DEFINED | P_IDENT(GPIO_PD9) | P_FUNCT(1))
#define P_HOST_D2	(P_DEFINED | P_IDENT(GPIO_PD10) | P_FUNCT(1))
#define P_HOST_D3	(P_DEFINED | P_IDENT(GPIO_PD11) | P_FUNCT(1))
#define P_HOST_D4	(P_DEFINED | P_IDENT(GPIO_PD12) | P_FUNCT(1))
#define P_HOST_D5	(P_DEFINED | P_IDENT(GPIO_PD13) | P_FUNCT(1))
#define P_HOST_D6	(P_DEFINED | P_IDENT(GPIO_PD14) | P_FUNCT(1))
#define P_HOST_D7	(P_DEFINED | P_IDENT(GPIO_PD15) | P_FUNCT(1))
#define P_SPORT1_TFS	(P_DEFINED | P_IDENT(GPIO_PD0) | P_FUNCT(2))
#define P_SPORT1_DTSEC	(P_DEFINED | P_IDENT(GPIO_PD1) | P_FUNCT(2))
#define P_SPORT1_DTPRI	(P_DEFINED | P_IDENT(GPIO_PD2) | P_FUNCT(2))
#define P_SPORT1_TSCLK	(P_DEFINED | P_IDENT(GPIO_PD3) | P_FUNCT(2))
#define P_SPORT1_RFS	(P_DEFINED | P_IDENT(GPIO_PD4) | P_FUNCT(2))
#define P_SPORT1_DRSEC	(P_DEFINED | P_IDENT(GPIO_PD5) | P_FUNCT(2))
#define P_SPORT1_DRPRI	(P_DEFINED | P_IDENT(GPIO_PD6) | P_FUNCT(2))
#define P_SPORT1_RSCLK	(P_DEFINED | P_IDENT(GPIO_PD7) | P_FUNCT(2))
#define P_PPI2_D0	(P_DEFINED | P_IDENT(GPIO_PD8) | P_FUNCT(2))
#define P_PPI2_D1	(P_DEFINED | P_IDENT(GPIO_PD9) | P_FUNCT(2))
#define P_PPI2_D2	(P_DEFINED | P_IDENT(GPIO_PD10) | P_FUNCT(2))
#define P_PPI2_D3	(P_DEFINED | P_IDENT(GPIO_PD11) | P_FUNCT(2))
#define P_PPI2_D4	(P_DEFINED | P_IDENT(GPIO_PD12) | P_FUNCT(2))
#define P_PPI2_D5	(P_DEFINED | P_IDENT(GPIO_PD13) | P_FUNCT(2))
#define P_PPI2_D6	(P_DEFINED | P_IDENT(GPIO_PD14) | P_FUNCT(2))
#define P_PPI2_D7	(P_DEFINED | P_IDENT(GPIO_PD15) | P_FUNCT(2))
#define P_PPI0_D18	(P_DEFINED | P_IDENT(GPIO_PD0) | P_FUNCT(3))
#define P_PPI0_D19	(P_DEFINED | P_IDENT(GPIO_PD1) | P_FUNCT(3))
#define P_PPI0_D20	(P_DEFINED | P_IDENT(GPIO_PD2) | P_FUNCT(3))
#define P_PPI0_D21	(P_DEFINED | P_IDENT(GPIO_PD3) | P_FUNCT(3))
#define P_PPI0_D22	(P_DEFINED | P_IDENT(GPIO_PD4) | P_FUNCT(3))
#define P_PPI0_D23	(P_DEFINED | P_IDENT(GPIO_PD5) | P_FUNCT(3))
#define P_KEY_ROW0	(P_DEFINED | P_IDENT(GPIO_PD8) | P_FUNCT(3))
#define P_KEY_ROW1	(P_DEFINED | P_IDENT(GPIO_PD9) | P_FUNCT(3))
#define P_KEY_ROW2	(P_DEFINED | P_IDENT(GPIO_PD10) | P_FUNCT(3))
#define P_KEY_ROW3	(P_DEFINED | P_IDENT(GPIO_PD11) | P_FUNCT(3))
#define P_KEY_COL0	(P_DEFINED | P_IDENT(GPIO_PD12) | P_FUNCT(3))
#define P_KEY_COL1	(P_DEFINED | P_IDENT(GPIO_PD13) | P_FUNCT(3))
#define P_KEY_COL2	(P_DEFINED | P_IDENT(GPIO_PD14) | P_FUNCT(3))
#define P_KEY_COL3	(P_DEFINED | P_IDENT(GPIO_PD15) | P_FUNCT(3))

#define GPIO_DEFAULT_BOOT_SPI_CS GPIO_PE4
#define P_DEFAULT_BOOT_SPI_CS P_SPI0_SSEL1
#define P_SPI0_SCK	(P_DEFINED | P_IDENT(GPIO_PE0) | P_FUNCT(0))
#define P_SPI0_MISO	(P_DEFINED | P_IDENT(GPIO_PE1) | P_FUNCT(0))
#define P_SPI0_MOSI	(P_DEFINED | P_IDENT(GPIO_PE2) | P_FUNCT(0))
#define P_SPI0_SS	(P_DEFINED | P_IDENT(GPIO_PE3) | P_FUNCT(0))
#define P_SPI0_SSEL1	(P_DEFINED | P_IDENT(GPIO_PE4) | P_FUNCT(0))
#define P_SPI0_SSEL2	(P_DEFINED | P_IDENT(GPIO_PE5) | P_FUNCT(0))
#define P_SPI0_SSEL3	(P_DEFINED | P_IDENT(GPIO_PE6) | P_FUNCT(0))
#define P_UART0_TX	(P_DEFINED | P_IDENT(GPIO_PE7) | P_FUNCT(0))
#define P_UART0_RX	(P_DEFINED | P_IDENT(GPIO_PE8) | P_FUNCT(0))
#define P_UART1_RTS	(P_DEFINED | P_IDENT(GPIO_PE9) | P_FUNCT(0))
#define P_UART1_CTS	(P_DEFINED | P_IDENT(GPIO_PE10) | P_FUNCT(0))
#define P_PPI1_CLK	(P_DEFINED | P_IDENT(GPIO_PE11) | P_FUNCT(0))
#define P_PPI1_FS1	(P_DEFINED | P_IDENT(GPIO_PE12) | P_FUNCT(0))
#define P_PPI1_FS2	(P_DEFINED | P_IDENT(GPIO_PE13) | P_FUNCT(0))
#define P_TWI0_SCL	(P_DEFINED | P_IDENT(GPIO_PE14) | P_FUNCT(0))
#define P_TWI0_SDA	(P_DEFINED | P_IDENT(GPIO_PE15) | P_FUNCT(0))
#define P_KEY_COL7	(P_DEFINED | P_IDENT(GPIO_PE0) | P_FUNCT(1))
#define P_KEY_ROW6	(P_DEFINED | P_IDENT(GPIO_PE1) | P_FUNCT(1))
#define P_KEY_COL6	(P_DEFINED | P_IDENT(GPIO_PE2) | P_FUNCT(1))
#define P_KEY_ROW5	(P_DEFINED | P_IDENT(GPIO_PE3) | P_FUNCT(1))
#define P_KEY_COL5	(P_DEFINED | P_IDENT(GPIO_PE4) | P_FUNCT(1))
#define P_KEY_ROW4	(P_DEFINED | P_IDENT(GPIO_PE5) | P_FUNCT(1))
#define P_KEY_COL4	(P_DEFINED | P_IDENT(GPIO_PE6) | P_FUNCT(1))
#define P_KEY_ROW7	(P_DEFINED | P_IDENT(GPIO_PE7) | P_FUNCT(1))

#define P_PPI0_D0	(P_DEFINED | P_IDENT(GPIO_PF0) | P_FUNCT(0))
#define P_PPI0_D1	(P_DEFINED | P_IDENT(GPIO_PF1) | P_FUNCT(0))
#define P_PPI0_D2	(P_DEFINED | P_IDENT(GPIO_PF2) | P_FUNCT(0))
#define P_PPI0_D3	(P_DEFINED | P_IDENT(GPIO_PF3) | P_FUNCT(0))
#define P_PPI0_D4	(P_DEFINED | P_IDENT(GPIO_PF4) | P_FUNCT(0))
#define P_PPI0_D5	(P_DEFINED | P_IDENT(GPIO_PF5) | P_FUNCT(0))
#define P_PPI0_D6	(P_DEFINED | P_IDENT(GPIO_PF6) | P_FUNCT(0))
#define P_PPI0_D7	(P_DEFINED | P_IDENT(GPIO_PF7) | P_FUNCT(0))
#define P_PPI0_D8	(P_DEFINED | P_IDENT(GPIO_PF8) | P_FUNCT(0))
#define P_PPI0_D9	(P_DEFINED | P_IDENT(GPIO_PF9) | P_FUNCT(0))
#define P_PPI0_D10	(P_DEFINED | P_IDENT(GPIO_PF10) | P_FUNCT(0))
#define P_PPI0_D11	(P_DEFINED | P_IDENT(GPIO_PF11) | P_FUNCT(0))
#define P_PPI0_D12	(P_DEFINED | P_IDENT(GPIO_PF12) | P_FUNCT(0))
#define P_PPI0_D13	(P_DEFINED | P_IDENT(GPIO_PF13) | P_FUNCT(0))
#define P_PPI0_D14	(P_DEFINED | P_IDENT(GPIO_PF14) | P_FUNCT(0))
#define P_PPI0_D15	(P_DEFINED | P_IDENT(GPIO_PF15) | P_FUNCT(0))

#ifdef CONFIG_BF548_ATAPI_ALTERNATIVE_PORT
# define P_ATAPI_D0A	(P_DEFINED | P_IDENT(GPIO_PF0) | P_FUNCT(1))
# define P_ATAPI_D1A	(P_DEFINED | P_IDENT(GPIO_PF1) | P_FUNCT(1))
# define P_ATAPI_D2A	(P_DEFINED | P_IDENT(GPIO_PF2) | P_FUNCT(1))
# define P_ATAPI_D3A	(P_DEFINED | P_IDENT(GPIO_PF3) | P_FUNCT(1))
# define P_ATAPI_D4A	(P_DEFINED | P_IDENT(GPIO_PF4) | P_FUNCT(1))
# define P_ATAPI_D5A	(P_DEFINED | P_IDENT(GPIO_PF5) | P_FUNCT(1))
# define P_ATAPI_D6A	(P_DEFINED | P_IDENT(GPIO_PF6) | P_FUNCT(1))
# define P_ATAPI_D7A	(P_DEFINED | P_IDENT(GPIO_PF7) | P_FUNCT(1))
# define P_ATAPI_D8A	(P_DEFINED | P_IDENT(GPIO_PF8) | P_FUNCT(1))
# define P_ATAPI_D9A	(P_DEFINED | P_IDENT(GPIO_PF9) | P_FUNCT(1))
# define P_ATAPI_D10A	(P_DEFINED | P_IDENT(GPIO_PF10) | P_FUNCT(1))
# define P_ATAPI_D11A	(P_DEFINED | P_IDENT(GPIO_PF11) | P_FUNCT(1))
# define P_ATAPI_D12A	(P_DEFINED | P_IDENT(GPIO_PF12) | P_FUNCT(1))
# define P_ATAPI_D13A	(P_DEFINED | P_IDENT(GPIO_PF13) | P_FUNCT(1))
# define P_ATAPI_D14A	(P_DEFINED | P_IDENT(GPIO_PF14) | P_FUNCT(1))
# define P_ATAPI_D15A	(P_DEFINED | P_IDENT(GPIO_PF15) | P_FUNCT(1))
#else
# define P_ATAPI_D0A	(P_DONTCARE)
# define P_ATAPI_D1A	(P_DONTCARE)
# define P_ATAPI_D2A	(P_DONTCARE)
# define P_ATAPI_D3A	(P_DONTCARE)
# define P_ATAPI_D4A	(P_DONTCARE)
# define P_ATAPI_D5A	(P_DONTCARE)
# define P_ATAPI_D6A	(P_DONTCARE)
# define P_ATAPI_D7A	(P_DONTCARE)
# define P_ATAPI_D8A	(P_DONTCARE)
# define P_ATAPI_D9A	(P_DONTCARE)
# define P_ATAPI_D10A	(P_DONTCARE)
# define P_ATAPI_D11A	(P_DONTCARE)
# define P_ATAPI_D12A	(P_DONTCARE)
# define P_ATAPI_D13A	(P_DONTCARE)
# define P_ATAPI_D14A	(P_DONTCARE)
# define P_ATAPI_D15A	(P_DONTCARE)
#endif

#define P_PPI0_CLK	(P_DEFINED | P_IDENT(GPIO_PG0) | P_FUNCT(0))
#define P_PPI0_FS1	(P_DEFINED | P_IDENT(GPIO_PG1) | P_FUNCT(0))
#define P_PPI0_FS2	(P_DEFINED | P_IDENT(GPIO_PG2) | P_FUNCT(0))
#define P_PPI0_D16	(P_DEFINED | P_IDENT(GPIO_PG3) | P_FUNCT(0))
#define P_PPI0_D17	(P_DEFINED | P_IDENT(GPIO_PG4) | P_FUNCT(0))
#define P_SPI1_SSEL1	(P_DEFINED | P_IDENT(GPIO_PG5) | P_FUNCT(0))
#define P_SPI1_SSEL2	(P_DEFINED | P_IDENT(GPIO_PG6) | P_FUNCT(0))
#define P_SPI1_SSEL3	(P_DEFINED | P_IDENT(GPIO_PG7) | P_FUNCT(0))
#define P_SPI1_SCK	(P_DEFINED | P_IDENT(GPIO_PG8) | P_FUNCT(0))
#define P_SPI1_MISO	(P_DEFINED | P_IDENT(GPIO_PG9) | P_FUNCT(0))
#define P_SPI1_MOSI	(P_DEFINED | P_IDENT(GPIO_PG10) | P_FUNCT(0))
#define P_SPI1_SS	(P_DEFINED | P_IDENT(GPIO_PG11) | P_FUNCT(0))
#define P_CAN0_TX	(P_DEFINED | P_IDENT(GPIO_PG12) | P_FUNCT(0))
#define P_CAN0_RX	(P_DEFINED | P_IDENT(GPIO_PG13) | P_FUNCT(0))
#define P_CAN1_TX	(P_DEFINED | P_IDENT(GPIO_PG14) | P_FUNCT(0))
#define P_CAN1_RX	(P_DEFINED | P_IDENT(GPIO_PG15) | P_FUNCT(0))
#ifdef CONFIG_BF548_ATAPI_ALTERNATIVE_PORT
# define P_ATAPI_A0A	(P_DEFINED | P_IDENT(GPIO_PG2) | P_FUNCT(1))
# define P_ATAPI_A1A	(P_DEFINED | P_IDENT(GPIO_PG3) | P_FUNCT(1))
# define P_ATAPI_A2A	(P_DEFINED | P_IDENT(GPIO_PG4) | P_FUNCT(1))
#else
# define P_ATAPI_A0A	(P_DONTCARE)
# define P_ATAPI_A1A	(P_DONTCARE)
# define P_ATAPI_A2A	(P_DONTCARE)
#endif
#define P_HOST_CE	(P_DEFINED | P_IDENT(GPIO_PG5) | P_FUNCT(1))
#define P_HOST_RD	(P_DEFINED | P_IDENT(GPIO_PG6) | P_FUNCT(1))
#define P_HOST_WR	(P_DEFINED | P_IDENT(GPIO_PG7) | P_FUNCT(1))
#define P_MTXONB	(P_DEFINED | P_IDENT(GPIO_PG11) | P_FUNCT(1))
#define P_PPI2_FS2	(P_DEFINED | P_IDENT(GPIO_PG5) | P_FUNCT(2))
#define P_PPI2_FS1	(P_DEFINED | P_IDENT(GPIO_PG6) | P_FUNCT(2))
#define P_PPI2_CLK	(P_DEFINED | P_IDENT(GPIO_PG7) | P_FUNCT(2))
#define P_CNT_CZM	(P_DEFINED | P_IDENT(GPIO_PG5) | P_FUNCT(3))

#define P_UART1_TX	(P_DEFINED | P_IDENT(GPIO_PH0) | P_FUNCT(0))
#define P_UART1_RX	(P_DEFINED | P_IDENT(GPIO_PH1) | P_FUNCT(0))
#define P_ATAPI_RESET	(P_DEFINED | P_IDENT(GPIO_PH2) | P_FUNCT(0))
#define P_HOST_ADDR	(P_DEFINED | P_IDENT(GPIO_PH3) | P_FUNCT(0))
#define P_HOST_ACK	(P_DEFINED | P_IDENT(GPIO_PH4) | P_FUNCT(0))
#define P_MTX	(P_DEFINED | P_IDENT(GPIO_PH5) | P_FUNCT(0))
#define P_MRX	(P_DEFINED | P_IDENT(GPIO_PH6) | P_FUNCT(0))
#define P_MRXONB	(P_DEFINED | P_IDENT(GPIO_PH7) | P_FUNCT(0))
#define P_A4	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH8) | P_FUNCT(0))
#define P_A5	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH9) | P_FUNCT(0))
#define P_A6	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH10) | P_FUNCT(0))
#define P_A7	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH11) | P_FUNCT(0))
#define P_A8	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH12) | P_FUNCT(0))
#define P_A9	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PH13) | P_FUNCT(0))
#define P_PPI1_FS3	(P_DEFINED | P_IDENT(GPIO_PH0) | P_FUNCT(1))
#define P_PPI2_FS3	(P_DEFINED | P_IDENT(GPIO_PH1) | P_FUNCT(1))
#define P_TMR8	(P_DEFINED | P_IDENT(GPIO_PH2) | P_FUNCT(1))
#define P_TMR9	(P_DEFINED | P_IDENT(GPIO_PH3) | P_FUNCT(1))
#define P_TMR10	(P_DEFINED | P_IDENT(GPIO_PH4) | P_FUNCT(1))
#define P_DMAR0	(P_DEFINED | P_IDENT(GPIO_PH5) | P_FUNCT(1))
#define P_DMAR1	(P_DEFINED | P_IDENT(GPIO_PH6) | P_FUNCT(1))
#define P_PPI0_FS3	(P_DEFINED | P_IDENT(GPIO_PH2) | P_FUNCT(2))
#define P_CNT_CDG	(P_DEFINED | P_IDENT(GPIO_PH3) | P_FUNCT(2))
#define P_CNT_CUD	(P_DEFINED | P_IDENT(GPIO_PH4) | P_FUNCT(2))

#define P_A10	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI0) | P_FUNCT(0))
#define P_A11	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI1) | P_FUNCT(0))
#define P_A12	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI2) | P_FUNCT(0))
#define P_A13	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI3) | P_FUNCT(0))
#define P_A14	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI4) | P_FUNCT(0))
#define P_A15	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI5) | P_FUNCT(0))
#define P_A16	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI6) | P_FUNCT(0))
#define P_A17	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI7) | P_FUNCT(0))
#define P_A18	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI8) | P_FUNCT(0))
#define P_A19	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI9) | P_FUNCT(0))
#define P_A20	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI10) | P_FUNCT(0))
#define P_A21	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI11) | P_FUNCT(0))
#define P_A22	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI12) | P_FUNCT(0))
#define P_A23	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI13) | P_FUNCT(0))
#define P_A24	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI14) | P_FUNCT(0))
#define P_A25	(P_MAYSHARE | P_DEFINED | P_IDENT(GPIO_PI15) | P_FUNCT(0))
#define P_NOR_CLK	(P_DEFINED | P_IDENT(GPIO_PI15) | P_FUNCT(1))

#define P_AMC_ARDY_NOR_WAIT	(P_DEFINED | P_IDENT(GPIO_PJ0) | P_FUNCT(0))
#define P_NAND_CE	(P_DEFINED | P_IDENT(GPIO_PJ1) | P_FUNCT(0))
#define P_NAND_RB	(P_DEFINED | P_IDENT(GPIO_PJ2) | P_FUNCT(0))
#define P_ATAPI_DIOR	(P_DEFINED | P_IDENT(GPIO_PJ3) | P_FUNCT(0))
#define P_ATAPI_DIOW	(P_DEFINED | P_IDENT(GPIO_PJ4) | P_FUNCT(0))
#define P_ATAPI_CS0	(P_DEFINED | P_IDENT(GPIO_PJ5) | P_FUNCT(0))
#define P_ATAPI_CS1	(P_DEFINED | P_IDENT(GPIO_PJ6) | P_FUNCT(0))
#define P_ATAPI_DMACK	(P_DEFINED | P_IDENT(GPIO_PJ7) | P_FUNCT(0))
#define P_ATAPI_DMARQ	(P_DEFINED | P_IDENT(GPIO_PJ8) | P_FUNCT(0))
#define P_ATAPI_INTRQ	(P_DEFINED | P_IDENT(GPIO_PJ9) | P_FUNCT(0))
#define P_ATAPI_IORDY	(P_DEFINED | P_IDENT(GPIO_PJ10) | P_FUNCT(0))
#define P_AMC_BR	(P_DEFINED | P_IDENT(GPIO_PJ11) | P_FUNCT(0))
#define P_AMC_BG	(P_DEFINED | P_IDENT(GPIO_PJ12) | P_FUNCT(0))
#define P_AMC_BGH	(P_DEFINED | P_IDENT(GPIO_PJ13) | P_FUNCT(0))


#define P_NAND_D0	(P_DONTCARE)
#define P_NAND_D1	(P_DONTCARE)
#define P_NAND_D2	(P_DONTCARE)
#define P_NAND_D3	(P_DONTCARE)
#define P_NAND_D4	(P_DONTCARE)
#define P_NAND_D5	(P_DONTCARE)
#define P_NAND_D6	(P_DONTCARE)
#define P_NAND_D7	(P_DONTCARE)
#define P_NAND_WE	(P_DONTCARE)
#define P_NAND_RE	(P_DONTCARE)
#define P_NAND_CLE	(P_DONTCARE)
#define P_NAND_ALE	(P_DONTCARE)

#ifdef CONFIG_PINCTRL

#include <mach/gpio.h>
#include <asm/blackfin.h>
#include <asm/irq_handler.h>

#define gpio_pint_regs bfin_pint_regs
#define adi_internal_set_wake bfin_internal_set_wake

static const struct pinctrl_pin_desc adi_pads[] = {
	PINCTRL_PIN(0, "PA0"),
	PINCTRL_PIN(1, "PA1"),
	PINCTRL_PIN(2, "PA2"),
	PINCTRL_PIN(3, "PG3"),
	PINCTRL_PIN(4, "PA4"),
	PINCTRL_PIN(5, "PA5"),
	PINCTRL_PIN(6, "PA6"),
	PINCTRL_PIN(7, "PA7"),
	PINCTRL_PIN(8, "PA8"),
	PINCTRL_PIN(9, "PA9"),
	PINCTRL_PIN(10, "PA10"),
	PINCTRL_PIN(11, "PA11"),
	PINCTRL_PIN(12, "PA12"),
	PINCTRL_PIN(13, "PA13"),
	PINCTRL_PIN(14, "PA14"),
	PINCTRL_PIN(15, "PA15"),
	PINCTRL_PIN(16, "PB0"),
	PINCTRL_PIN(17, "PB1"),
	PINCTRL_PIN(18, "PB2"),
	PINCTRL_PIN(19, "PB3"),
	PINCTRL_PIN(20, "PB4"),
	PINCTRL_PIN(21, "PB5"),
	PINCTRL_PIN(22, "PB6"),
	PINCTRL_PIN(23, "PB7"),
	PINCTRL_PIN(24, "PB8"),
	PINCTRL_PIN(25, "PB9"),
	PINCTRL_PIN(26, "PB10"),
	PINCTRL_PIN(27, "PB11"),
	PINCTRL_PIN(28, "PB12"),
	PINCTRL_PIN(29, "PB13"),
	PINCTRL_PIN(30, "PB14"),
	PINCTRL_PIN(32, "PC0"),
	PINCTRL_PIN(33, "PC1"),
	PINCTRL_PIN(34, "PC2"),
	PINCTRL_PIN(35, "PC3"),
	PINCTRL_PIN(36, "PC4"),
	PINCTRL_PIN(37, "PC5"),
	PINCTRL_PIN(38, "PC6"),
	PINCTRL_PIN(39, "PC7"),
	PINCTRL_PIN(40, "PC8"),
	PINCTRL_PIN(41, "PC9"),
	PINCTRL_PIN(42, "PC10"),
	PINCTRL_PIN(43, "PC11"),
	PINCTRL_PIN(44, "PC12"),
	PINCTRL_PIN(45, "PC13"),
	PINCTRL_PIN(48, "PD0"),
	PINCTRL_PIN(49, "PD1"),
	PINCTRL_PIN(50, "PD2"),
	PINCTRL_PIN(51, "PD3"),
	PINCTRL_PIN(52, "PD4"),
	PINCTRL_PIN(53, "PD5"),
	PINCTRL_PIN(54, "PD6"),
	PINCTRL_PIN(55, "PD7"),
	PINCTRL_PIN(56, "PD8"),
	PINCTRL_PIN(57, "PD9"),
	PINCTRL_PIN(58, "PD10"),
	PINCTRL_PIN(59, "PD11"),
	PINCTRL_PIN(60, "PD12"),
	PINCTRL_PIN(61, "PD13"),
	PINCTRL_PIN(62, "PD14"),
	PINCTRL_PIN(63, "PD15"),
	PINCTRL_PIN(64, "PE0"),
	PINCTRL_PIN(65, "PE1"),
	PINCTRL_PIN(66, "PE2"),
	PINCTRL_PIN(67, "PE3"),
	PINCTRL_PIN(68, "PE4"),
	PINCTRL_PIN(69, "PE5"),
	PINCTRL_PIN(70, "PE6"),
	PINCTRL_PIN(71, "PE7"),
	PINCTRL_PIN(72, "PE8"),
	PINCTRL_PIN(73, "PE9"),
	PINCTRL_PIN(74, "PE10"),
	PINCTRL_PIN(75, "PE11"),
	PINCTRL_PIN(76, "PE12"),
	PINCTRL_PIN(77, "PE13"),
	PINCTRL_PIN(78, "PE14"),
	PINCTRL_PIN(79, "PE15"),
	PINCTRL_PIN(80, "PF0"),
	PINCTRL_PIN(81, "PF1"),
	PINCTRL_PIN(82, "PF2"),
	PINCTRL_PIN(83, "PF3"),
	PINCTRL_PIN(84, "PF4"),
	PINCTRL_PIN(85, "PF5"),
	PINCTRL_PIN(86, "PF6"),
	PINCTRL_PIN(87, "PF7"),
	PINCTRL_PIN(88, "PF8"),
	PINCTRL_PIN(89, "PF9"),
	PINCTRL_PIN(90, "PF10"),
	PINCTRL_PIN(91, "PF11"),
	PINCTRL_PIN(92, "PF12"),
	PINCTRL_PIN(93, "PF13"),
	PINCTRL_PIN(94, "PF14"),
	PINCTRL_PIN(95, "PF15"),
	PINCTRL_PIN(96, "PG0"),
	PINCTRL_PIN(97, "PG1"),
	PINCTRL_PIN(98, "PG2"),
	PINCTRL_PIN(99, "PG3"),
	PINCTRL_PIN(100, "PG4"),
	PINCTRL_PIN(101, "PG5"),
	PINCTRL_PIN(102, "PG6"),
	PINCTRL_PIN(103, "PG7"),
	PINCTRL_PIN(104, "PG8"),
	PINCTRL_PIN(105, "PG9"),
	PINCTRL_PIN(106, "PG10"),
	PINCTRL_PIN(107, "PG11"),
	PINCTRL_PIN(108, "PG12"),
	PINCTRL_PIN(109, "PG13"),
	PINCTRL_PIN(110, "PG14"),
	PINCTRL_PIN(111, "PG15"),
	PINCTRL_PIN(112, "PH0"),
	PINCTRL_PIN(113, "PH1"),
	PINCTRL_PIN(114, "PH2"),
	PINCTRL_PIN(115, "PH3"),
	PINCTRL_PIN(116, "PH4"),
	PINCTRL_PIN(117, "PH5"),
	PINCTRL_PIN(118, "PH6"),
	PINCTRL_PIN(119, "PH7"),
	PINCTRL_PIN(120, "PH8"),
	PINCTRL_PIN(121, "PH9"),
	PINCTRL_PIN(122, "PH10"),
	PINCTRL_PIN(123, "PH11"),
	PINCTRL_PIN(124, "PH12"),
	PINCTRL_PIN(125, "PH13"),
	PINCTRL_PIN(128, "PI0"),
	PINCTRL_PIN(129, "PI1"),
	PINCTRL_PIN(130, "PI2"),
	PINCTRL_PIN(131, "PI3"),
	PINCTRL_PIN(132, "PI4"),
	PINCTRL_PIN(133, "PI5"),
	PINCTRL_PIN(134, "PI6"),
	PINCTRL_PIN(135, "PI7"),
	PINCTRL_PIN(136, "PI8"),
	PINCTRL_PIN(137, "PI9"),
	PINCTRL_PIN(138, "PI10"),
	PINCTRL_PIN(139, "PI11"),
	PINCTRL_PIN(140, "PI12"),
	PINCTRL_PIN(141, "PI13"),
	PINCTRL_PIN(142, "PI14"),
	PINCTRL_PIN(143, "PI15"),
	PINCTRL_PIN(144, "PJ0"),
	PINCTRL_PIN(145, "PJ1"),
	PINCTRL_PIN(146, "PJ2"),
	PINCTRL_PIN(147, "PJ3"),
	PINCTRL_PIN(148, "PJ4"),
	PINCTRL_PIN(149, "PJ5"),
	PINCTRL_PIN(150, "PJ6"),
	PINCTRL_PIN(151, "PJ7"),
	PINCTRL_PIN(152, "PJ8"),
	PINCTRL_PIN(153, "PJ9"),
	PINCTRL_PIN(154, "PJ10"),
	PINCTRL_PIN(155, "PJ11"),
	PINCTRL_PIN(156, "PJ12"),
	PINCTRL_PIN(157, "PJ13"),
};

static const unsigned uart0_pins[] = {
	GPIO_PE7, GPIO_PE8,
};

static const unsigned uart1_pins[] = {
	GPIO_PH0, GPIO_PH1,
#ifdef CONFIG_BFIN_UART1_CTSRTS
	GPIO_PE9, GPIO_PE10,
#endif
};

static const unsigned uart2_pins[] = {
	GPIO_PB4, GPIO_PB5,
};

static const unsigned uart3_pins[] = {
	GPIO_PB6, GPIO_PB7,
#ifdef CONFIG_BFIN_UART3_CTSRTS
	GPIO_PB2, GPIO_PB3,
#endif
};

static const unsigned rsi0_pins[] = {
	GPIO_PC8, GPIO_PC9, GPIO_PC10, GPIO_PC11, GPIO_PC12, GPIO_PC13,
};

static const unsigned spi0_pins[] = {
	GPIO_PE0, GPIO_PE1, GPIO_PE2,
};

static const unsigned spi1_pins[] = {
	GPIO_PG8, GPIO_PG9, GPIO_PG10,
};

static const unsigned twi0_pins[] = {
	GPIO_PE14, GPIO_PE15,
};

static const unsigned twi1_pins[] = {
	GPIO_PB0, GPIO_PB1,
};

static const unsigned rotary_pins[] = {
	GPIO_PH4, GPIO_PH3, GPIO_PH5,
};

static const unsigned can0_pins[] = {
	GPIO_PG13, GPIO_PG12,
};

static const unsigned can1_pins[] = {
	GPIO_PG14, GPIO_PG15,
};

static const unsigned smc0_pins[] = {
	GPIO_PH8, GPIO_PH9, GPIO_PH10, GPIO_PH11, GPIO_PH12, GPIO_PH13,
	GPIO_PI0, GPIO_PI1, GPIO_PI2, GPIO_PI3, GPIO_PI4, GPIO_PI5, GPIO_PI6,
	GPIO_PI7, GPIO_PI8, GPIO_PI9, GPIO_PI10, GPIO_PI11,
	GPIO_PI12, GPIO_PI13, GPIO_PI14, GPIO_PI15,
};

static const unsigned sport0_pins[] = {
	GPIO_PC0, GPIO_PC2, GPIO_PC3, GPIO_PC4, GPIO_PC6, GPIO_PC7,
};

static const unsigned sport1_pins[] = {
	GPIO_PD0, GPIO_PD2, GPIO_PD3, GPIO_PD4, GPIO_PD6, GPIO_PD7,
};

static const unsigned sport2_pins[] = {
	GPIO_PA0, GPIO_PA2, GPIO_PA3, GPIO_PA4, GPIO_PA6, GPIO_PA7,
};

static const unsigned sport3_pins[] = {
	GPIO_PA8, GPIO_PA10, GPIO_PA11, GPIO_PA12, GPIO_PA14, GPIO_PA15,
};

static const unsigned ppi0_8b_pins[] = {
	GPIO_PF0, GPIO_PF1, GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5, GPIO_PF6,
	GPIO_PF7, GPIO_PF13, GPIO_PG0, GPIO_PG1, GPIO_PG2,
};

static const unsigned ppi0_16b_pins[] = {
	GPIO_PF0, GPIO_PF1, GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5, GPIO_PF6,
	GPIO_PF7, GPIO_PF9, GPIO_PF10, GPIO_PF11, GPIO_PF12,
	GPIO_PF13, GPIO_PF14, GPIO_PF15,
	GPIO_PG0, GPIO_PG1, GPIO_PG2,
};

static const unsigned ppi0_24b_pins[] = {
	GPIO_PF0, GPIO_PF1, GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5, GPIO_PF6,
	GPIO_PF7, GPIO_PF8, GPIO_PF9, GPIO_PF10, GPIO_PF11, GPIO_PF12,
	GPIO_PF13, GPIO_PF14, GPIO_PF15, GPIO_PD0, GPIO_PD1, GPIO_PD2,
	GPIO_PD3, GPIO_PD4, GPIO_PD5, GPIO_PG3, GPIO_PG4,
	GPIO_PG0, GPIO_PG1, GPIO_PG2,
};

static const unsigned ppi1_8b_pins[] = {
	GPIO_PD0, GPIO_PD1, GPIO_PD2, GPIO_PD3, GPIO_PD4, GPIO_PD5, GPIO_PD6,
	GPIO_PD7, GPIO_PE11, GPIO_PE12, GPIO_PE13,
};

static const unsigned ppi1_16b_pins[] = {
	GPIO_PD0, GPIO_PD1, GPIO_PD2, GPIO_PD3, GPIO_PD4, GPIO_PD5, GPIO_PD6,
	GPIO_PD7, GPIO_PD8, GPIO_PD9, GPIO_PD10, GPIO_PD11, GPIO_PD12,
	GPIO_PD13, GPIO_PD14, GPIO_PD15,
	GPIO_PE11, GPIO_PE12, GPIO_PE13,
};

static const unsigned ppi2_8b_pins[] = {
	GPIO_PD8, GPIO_PD9, GPIO_PD10, GPIO_PD11, GPIO_PD12,
	GPIO_PD13, GPIO_PD14, GPIO_PD15,
	GPIO_PA7, GPIO_PB0, GPIO_PB1, GPIO_PB2, GPIO_PB3,
};

static const unsigned atapi_pins[] = {
	GPIO_PH2, GPIO_PJ3, GPIO_PJ4, GPIO_PJ5, GPIO_PJ6,
	GPIO_PJ7, GPIO_PJ8, GPIO_PJ9, GPIO_PJ10,
	GPIO_PG5, GPIO_PG6, GPIO_PG7,
#ifdef CONFIG_BF548_ATAPI_ALTERNATIVE_PORT
	GPIO_PF0, GPIO_PF1, GPIO_PF2, GPIO_PF3, GPIO_PF4, GPIO_PF5, GPIO_PF6,
	GPIO_PF7, GPIO_PF8, GPIO_PF9, GPIO_PF10, GPIO_PF11, GPIO_PF12,
	GPIO_PF13, GPIO_PF14, GPIO_PF15, GPIO_PG2, GPIO_PG3, GPIO_PG4,
#endif
};

static const unsigned nfc0_pins[] = {
	GPIO_PJ1, GPIO_PJ2,
};

static const unsigned keys_4x4_pins[] = {
	GPIO_PD8, GPIO_PD9, GPIO_PD10, GPIO_PD11,
	GPIO_PD12, GPIO_PD13, GPIO_PD14, GPIO_PD15,
};

static const unsigned keys_8x8_pins[] = {
	GPIO_PD8, GPIO_PD9, GPIO_PD10, GPIO_PD11,
	GPIO_PD12, GPIO_PD13, GPIO_PD14, GPIO_PD15,
	GPIO_PE0, GPIO_PE1, GPIO_PE2, GPIO_PE3,
	GPIO_PE4, GPIO_PE5, GPIO_PE6, GPIO_PE7,
};

 /**
 * struct adi_pin_group - describes a pin group
 * @name: the name of this pin group
 * @pins: an array of pins
 * @num: the number of pins in this array
 */
struct adi_pin_group {
	const char *name;
	const unsigned *pins;
	const unsigned num;
};

#define ADI_PIN_GROUP(n, p)  \
	{			\
		.name = n,	\
		.pins = p,	\
		.num = ARRAY_SIZE(p),	\
	}

static const struct adi_pin_group adi_pin_groups[] = {
	ADI_PIN_GROUP("uart0grp", uart0_pins),
	ADI_PIN_GROUP("uart1grp", uart1_pins),
	ADI_PIN_GROUP("rsi0grp", rsi0_pins),
	ADI_PIN_GROUP("spi0grp", spi0_pins),
	ADI_PIN_GROUP("spi1grp", spi1_pins),
	ADI_PIN_GROUP("twi0grp", twi0_pins),
	ADI_PIN_GROUP("twi1grp", twi1_pins),
	ADI_PIN_GROUP("rotarygrp", rotary_pins),
	ADI_PIN_GROUP("can0grp", can0_pins),
	ADI_PIN_GROUP("can1grp", can1_pins),
	ADI_PIN_GROUP("smc0grp", smc0_pins),
	ADI_PIN_GROUP("sport0grp", sport0_pins),
	ADI_PIN_GROUP("sport1grp", sport1_pins),
	ADI_PIN_GROUP("sport2grp", sport2_pins),
	ADI_PIN_GROUP("sport3grp", sport3_pins),
	ADI_PIN_GROUP("ppi0_8bgrp", ppi0_8b_pins),
	ADI_PIN_GROUP("ppi0_16bgrp", ppi0_16b_pins),
	ADI_PIN_GROUP("ppi0_24bgrp", ppi0_24b_pins),
	ADI_PIN_GROUP("ppi1_8bgrp", ppi1_8b_pins),
	ADI_PIN_GROUP("ppi1_16bgrp", ppi1_16b_pins),
	ADI_PIN_GROUP("ppi2_8bgrp", ppi2_8b_pins),
	ADI_PIN_GROUP("atapigrp", atapi_pins),
	ADI_PIN_GROUP("nfc0grp", nfc0_pins),
	ADI_PIN_GROUP("keys_4x4grp", keys_4x4_pins),
	ADI_PIN_GROUP("keys_8x8grp", keys_8x8_pins),
};

static const unsigned short uart0_mux[] = {
	P_UART0_TX, P_UART0_RX,
	0
};

static const unsigned short uart1_mux[] = {
	P_UART1_TX, P_UART1_RX,
#ifdef CONFIG_BFIN_UART1_CTSRTS
	P_UART1_RTS, P_UART1_CTS,
#endif
	0
};

static const unsigned short uart2_mux[] = {
	P_UART2_TX, P_UART2_RX,
	0
};

static const unsigned short uart3_mux[] = {
	P_UART3_TX, P_UART3_RX,
#ifdef CONFIG_BFIN_UART3_CTSRTS
	P_UART3_RTS, P_UART3_CTS,
#endif
	0
};

static const unsigned short rsi0_mux[] = {
	P_SD_D0, P_SD_D1, P_SD_D2, P_SD_D3, P_SD_CLK, P_SD_CMD,
	0
};

static const unsigned short spi0_mux[] = {
	P_SPI0_SCK, P_SPI0_MISO, P_SPI0_MOSI, 0
};

static const unsigned short spi1_mux[] = {
	P_SPI1_SCK, P_SPI1_MISO, P_SPI1_MOSI, 0
};

static const unsigned short twi0_mux[] = {
	P_TWI0_SCL, P_TWI0_SDA, 0
};

static const unsigned short twi1_mux[] = {
	P_TWI1_SCL, P_TWI1_SDA, 0
};

static const unsigned short rotary_mux[] = {
	P_CNT_CUD, P_CNT_CDG, P_CNT_CZM, 0
};

static const unsigned short sport0_mux[] = {
	P_SPORT0_TFS, P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
	P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0
};

static const unsigned short sport1_mux[] = {
	P_SPORT1_TFS, P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS,
	P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0
};

static const unsigned short sport2_mux[] = {
	P_SPORT2_TFS, P_SPORT2_DTPRI, P_SPORT2_TSCLK, P_SPORT2_RFS,
	P_SPORT2_DRPRI, P_SPORT2_RSCLK, 0
};

static const unsigned short sport3_mux[] = {
	P_SPORT3_TFS, P_SPORT3_DTPRI, P_SPORT3_TSCLK, P_SPORT3_RFS,
	P_SPORT3_DRPRI, P_SPORT3_RSCLK, 0
};

static const unsigned short can0_mux[] = {
	P_CAN0_RX, P_CAN0_TX, 0
};

static const unsigned short can1_mux[] = {
	P_CAN1_RX, P_CAN1_TX, 0
};

static const unsigned short smc0_mux[] = {
	P_A4, P_A5, P_A6, P_A7, P_A8, P_A9, P_A10, P_A11, P_A12,
	P_A13, P_A14, P_A15, P_A16, P_A17, P_A18, P_A19, P_A20, P_A21,
	P_A22, P_A23, P_A24, P_A25, P_NOR_CLK, 0,
};

static const unsigned short ppi0_8b_mux[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};

static const unsigned short ppi0_16b_mux[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};

static const unsigned short ppi0_24b_mux[] = {
	P_PPI0_D0, P_PPI0_D1, P_PPI0_D2, P_PPI0_D3,
	P_PPI0_D4, P_PPI0_D5, P_PPI0_D6, P_PPI0_D7,
	P_PPI0_D8, P_PPI0_D9, P_PPI0_D10, P_PPI0_D11,
	P_PPI0_D12, P_PPI0_D13, P_PPI0_D14, P_PPI0_D15,
	P_PPI0_D16, P_PPI0_D17, P_PPI0_D18, P_PPI0_D19,
	P_PPI0_D20, P_PPI0_D21, P_PPI0_D22, P_PPI0_D23,
	P_PPI0_CLK, P_PPI0_FS1, P_PPI0_FS2,
	0,
};

static const unsigned short ppi1_8b_mux[] = {
	P_PPI1_D0, P_PPI1_D1, P_PPI1_D2, P_PPI1_D3,
	P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7,
	P_PPI1_CLK, P_PPI1_FS1, P_PPI1_FS2,
	0,
};

static const unsigned short ppi1_16b_mux[] = {
	P_PPI1_D0, P_PPI1_D1, P_PPI1_D2, P_PPI1_D3,
	P_PPI1_D4, P_PPI1_D5, P_PPI1_D6, P_PPI1_D7,
	P_PPI1_D8, P_PPI1_D9, P_PPI1_D10, P_PPI1_D11,
	P_PPI1_D12, P_PPI1_D13, P_PPI1_D14, P_PPI1_D15,
	P_PPI1_CLK, P_PPI1_FS1, P_PPI1_FS2,
	0,
};

static const unsigned short ppi2_8b_mux[] = {
	P_PPI2_D0, P_PPI2_D1, P_PPI2_D2, P_PPI2_D3,
	P_PPI2_D4, P_PPI2_D5, P_PPI2_D6, P_PPI2_D7,
	P_PPI2_CLK, P_PPI2_FS1, P_PPI2_FS2,
	0,
};

static const unsigned short atapi_mux[] = {
	P_ATAPI_RESET, P_ATAPI_DIOR, P_ATAPI_DIOW, P_ATAPI_CS0, P_ATAPI_CS1,
	P_ATAPI_DMACK, P_ATAPI_DMARQ, P_ATAPI_INTRQ, P_ATAPI_IORDY,
	P_ATAPI_D0A, P_ATAPI_D1A, P_ATAPI_D2A, P_ATAPI_D3A, P_ATAPI_D4A,
	P_ATAPI_D5A, P_ATAPI_D6A, P_ATAPI_D7A, P_ATAPI_D8A, P_ATAPI_D9A,
	P_ATAPI_D10A, P_ATAPI_D11A, P_ATAPI_D12A, P_ATAPI_D13A, P_ATAPI_D14A,
	P_ATAPI_D15A, P_ATAPI_A0A, P_ATAPI_A1A, P_ATAPI_A2A,
	0
};

static const unsigned short nfc0_mux[] = {
	P_NAND_CE, P_NAND_RB,
	0
};

static const unsigned short keys_4x4_mux[] = {
	P_KEY_ROW3, P_KEY_ROW2, P_KEY_ROW1, P_KEY_ROW0,
	P_KEY_COL3, P_KEY_COL2, P_KEY_COL1, P_KEY_COL0,
	0
};

static const unsigned short keys_8x8_mux[] = {
	P_KEY_ROW7, P_KEY_ROW6, P_KEY_ROW5, P_KEY_ROW4,
	P_KEY_ROW3, P_KEY_ROW2, P_KEY_ROW1, P_KEY_ROW0,
	P_KEY_COL7, P_KEY_COL6, P_KEY_COL5, P_KEY_COL4,
	P_KEY_COL3, P_KEY_COL2, P_KEY_COL1, P_KEY_COL0,
	0
};

struct adi_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
	const unsigned short *mux;
};

static const char * const uart0grp[] = { "uart0grp" };
static const char * const uart1grp[] = { "uart1grp" };
static const char * const uart2grp[] = { "uart2grp" };
static const char * const uart3grp[] = { "uart3grp" };
static const char * const rsi0grp[] = { "rsi0grp" };
static const char * const spi0grp[] = { "spi0grp" };
static const char * const spi1grp[] = { "spi1grp" };
static const char * const twi0grp[] = { "twi0grp" };
static const char * const twi1grp[] = { "twi1grp" };
static const char * const rotarygrp[] = { "rotarygrp" };
static const char * const can0grp[] = { "can0grp" };
static const char * const can1grp[] = { "can1grp" };
static const char * const smc0grp[] = { "smc0grp" };
static const char * const sport0grp[] = { "sport0grp" };
static const char * const sport1grp[] = { "sport1grp" };
static const char * const sport2grp[] = { "sport2grp" };
static const char * const sport3grp[] = { "sport3grp" };
static const char * const ppi0_8bgrp[] = { "ppi0_8bgrp" };
static const char * const ppi0_16bgrp[] = { "ppi0_16bgrp" };
static const char * const ppi0_24bgrp[] = { "ppi0_24bgrp" };
static const char * const ppi1_8bgrp[] = { "ppi1_8bgrp" };
static const char * const ppi1_16bgrp[] = { "ppi1_16bgrp" };
static const char * const ppi2_8bgrp[] = { "ppi2_8bgrp" };
static const char * const atapigrp[] = { "atapigrp" };
static const char * const nfc0grp[] = { "nfc0grp" };
static const char * const keys_4x4grp[] = { "keys_4x4grp" };
static const char * const keys_8x8grp[] = { "keys_8x8grp" };

#define ADI_PMX_FUNCTION(n, g, m)		\
	{					\
		.name = n,			\
		.groups = g,			\
		.num_groups = ARRAY_SIZE(g),	\
		.mux = m,			\
	}

static const struct adi_pmx_func adi_pmx_functions[] = {
	ADI_PMX_FUNCTION("uart0", uart0grp, uart0_mux),
	ADI_PMX_FUNCTION("uart1", uart1grp, uart1_mux),
	ADI_PMX_FUNCTION("uart2", uart0grp, uart2_mux),
	ADI_PMX_FUNCTION("uart3", uart1grp, uart3_mux),
	ADI_PMX_FUNCTION("rsi0", rsi0grp, rsi0_mux),
	ADI_PMX_FUNCTION("spi0", spi0grp, spi0_mux),
	ADI_PMX_FUNCTION("spi1", spi1grp, spi1_mux),
	ADI_PMX_FUNCTION("twi0", twi0grp, twi0_mux),
	ADI_PMX_FUNCTION("twi1", twi1grp, twi1_mux),
	ADI_PMX_FUNCTION("rotary", rotarygrp, rotary_mux),
	ADI_PMX_FUNCTION("can0", can0grp, can0_mux),
	ADI_PMX_FUNCTION("can1", can1grp, can1_mux),
	ADI_PMX_FUNCTION("smc0", smc0grp, smc0_mux),
	ADI_PMX_FUNCTION("sport0", sport0grp, sport0_mux),
	ADI_PMX_FUNCTION("sport1", sport1grp, sport1_mux),
	ADI_PMX_FUNCTION("sport2", sport2grp, sport2_mux),
	ADI_PMX_FUNCTION("sport3", sport3grp, sport3_mux),
	ADI_PMX_FUNCTION("ppi0_8b", ppi0_8bgrp, ppi0_8b_mux),
	ADI_PMX_FUNCTION("ppi0_16b", ppi0_16bgrp, ppi0_16b_mux),
	ADI_PMX_FUNCTION("ppi0_24b", ppi0_24bgrp, ppi0_24b_mux),
	ADI_PMX_FUNCTION("ppi1_8b", ppi1_8bgrp, ppi1_8b_mux),
	ADI_PMX_FUNCTION("ppi1_16b", ppi1_16bgrp, ppi1_16b_mux),
	ADI_PMX_FUNCTION("ppi2_8b", ppi2_8bgrp, ppi2_8b_mux),
	ADI_PMX_FUNCTION("atapi", atapigrp, atapi_mux),
	ADI_PMX_FUNCTION("nfc0", nfc0grp, nfc0_mux),
	ADI_PMX_FUNCTION("keys_4x4", keys_4x4grp, keys_4x4_mux),
	ADI_PMX_FUNCTION("keys_8x8", keys_8x8grp, keys_8x8_mux),
};

#endif	/* CONFIG_PINCTRL */

#endif /* _MACH_PORTMUX_H_ */
