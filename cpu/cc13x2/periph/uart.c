/*
 * Copyright (C) 2016 Leon George
 * Copyright (C) 2018 Anton Gerasimov
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cc13x2
 * @ingroup     drivers_periph_uart
 * @{
 *
 * @file
 * @brief       Low-level UART driver implementation
 *
 * @author      Leon M. George <leon@georgemail.eu>
 * @author      Anton Gerasimov <tossel@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "periph/uart.h"

/**
 * @brief   Bit mask for the fractional part of the baudrate
 */
#define FRAC_BITS           (6U)
#define FRAC_MASK           (0x3f)

/**
 * @brief   Get the enable mask depending on enabled HW flow control
 */
#if UART_HW_FLOW_CONTROL
#define ENABLE_MASK         (UART_CTSEN | UART_CTL_RTSEN | \
                             UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN)
#else
#define ENABLE_MASK         (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN)
#endif

/**
 * @brief allocate memory to store callback functions
 */
static uart_isr_ctx_t ctx[UART_NUMOF];

static void delay_nop(uint32_t ms) {
    volatile uint32_t counter = 0;

    for(;ms!=0; ms--) {
        for(counter = 4800; counter !=0; counter--);
    }
}

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    /* make sure the uart device is valid */
    if (uart > 1) {
        return UART_NODEV;
    }

    uart_regs_t* uart_reg = (uart) ? UART1 : UART0;
    int tx_pin = (uart) ? UART1_TX_PIN : UART0_TX_PIN;
    int rx_pin = (uart) ? UART1_RX_PIN : UART0_RX_PIN;
    int irqn = (uart) ? UART1_IRQN : UART0_IRQN;
#if UART_HW_FLOW_CONTROL
    int rts_pin = (uart) ? UART1_RTS_PIN : UART0_RTS_PIN;
    int cts_pin = (uart) ? UART1_CTS_PIN : UART0_CTS_PIN;
#endif

    /* enable clocks: serial power domain and UART */
    PRCM->PDCTL0SERIAL = 1;
    //while (!(PRCM->PDSTAT0 & PDSTAT0_SERIAL_ON)) ;
    uart_poweron(uart);

    /* disable and reset the UART */
    uart_reg->CTL = 0;

    /* save context */
    ctx[uart].rx_cb = rx_cb;
    ctx[uart].arg = arg;

    /* configure pins */
    IOC->CFG[tx_pin] =  IOCFG_PORTID_UART0_TX;
    IOC->CFG[rx_pin] = (IOCFG_PORTID_UART0_RX | IOCFG_INPUT_ENABLE);
#if UART_HW_FLOW_CONTROL
    IOC->CFG[rts_pin] =  IOCFG_PORTID_UART0_RTS;
    IOC->CFG[cts_pin] = (IOCFG_PORTID_UART0_CTS | IOCFG_INPUT_ENABLE);
#endif

    /* calculate baud-rate */
    uint32_t tmp = (CLOCK_CORECLOCK * 4);
    tmp += (baudrate / 2);
    tmp /= baudrate;
    uart_reg->IBRD = (tmp >> FRAC_BITS);
    uart_reg->FBRD = (tmp & FRAC_MASK);

    /* configure line to 8N1 mode, LRCH must be written after IBRD and FBRD! */
    uart_reg->LCRH = UART_LCRH_WLEN_8;

    /* enable the RX interrupt */
    uart_reg->IMSC = UART_IMSC_RXIM;
    NVIC_EnableIRQ(irqn);

    /* start the UART */
    uart_reg->CTL = ENABLE_MASK;

    return UART_OK;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    uart_regs_t* uart_reg = (uart) ? UART1 : UART0;

    for (size_t i = 0; i < len; i++) {
        while (uart_reg->FR & UART_FR_TXFF) {}
        uart_reg->DR = data[i];
    }
}

void uart_poweron(uart_t uart)
{
    uart_regs_t* uart_reg = (uart) ? UART1 : UART0;

    PRCM->UARTCLKGR |= 0x11;
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    delay_nop(1);
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}
    while (!(PRCM->PDSTAT0 & PDSTAT0_SERIAL_ON)) ;

    uart_reg->CTL = ENABLE_MASK;
}

void uart_poweroff(uart_t uart)
{
    uart_regs_t* uart_reg = (uart) ? UART1 : UART0;

    uart_reg->CTL = 0;

    PRCM->UARTCLKGR = 0;
    PRCM->CLKLOADCTL = CLKLOADCTL_LOAD;
    while (!(PRCM->CLKLOADCTL & CLKLOADCTL_LOADDONE)) {}

}

static void isr_uart(uart_t uart)
{
    uart_regs_t* uart_reg = (uart) ? UART1 : UART0;
    /* remember pending interrupts */
    uint32_t mis = uart_reg->MIS;
    /* clear them */
    uart_reg->ICR = mis;

    /* read received byte and pass it to the RX callback */
    if (mis & UART_MIS_RXMIS) {
        ctx[uart].rx_cb(ctx[uart].arg, (uint8_t)uart_reg->DR);
    }

    cortexm_isr_end();
}

void isr_uart0(void) {
    isr_uart(0);
}

void isr_uart1(void) {
    isr_uart(1);
}

