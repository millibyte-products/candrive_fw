/* USART3 polled debug @ 115200 8N1, pins PB10 (TX) / PB11 (RX). */

#include "usart_dbg.h"
#include "clocks.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx.h"

int usart_dbg_init(uint32_t baud)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB | LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    /* PB10: AF push-pull 50 MHz */
    LL_GPIO_InitTypeDef io = {0};
    io.Pin        = LL_GPIO_PIN_10;
    io.Mode       = LL_GPIO_MODE_ALTERNATE;
    io.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &io);

    /* PB11: floating input */
    io.Pin   = LL_GPIO_PIN_11;
    io.Mode  = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOB, &io);

    LL_USART_InitTypeDef u = {0};
    u.BaudRate            = baud;
    u.DataWidth           = LL_USART_DATAWIDTH_8B;
    u.StopBits            = LL_USART_STOPBITS_1;
    u.Parity              = LL_USART_PARITY_NONE;
    u.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    u.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    if (LL_USART_Init(USART3, &u) != SUCCESS) return -1;
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_Enable(USART3);
    return 0;
}

void usart_dbg_putc(char c)
{
    while (!LL_USART_IsActiveFlag_TXE(USART3)) { /* wait for TDR empty */ }
    LL_USART_TransmitData8(USART3, (uint8_t)c);
}

void usart_dbg_write(const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len--) usart_dbg_putc((char)*p++);
}

void usart_dbg_puts(const char *s)
{
    while (*s) usart_dbg_putc(*s++);
}
