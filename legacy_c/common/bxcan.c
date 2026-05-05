/* bxCAN1 — bare-LL polled driver.
 *
 * Pin map (CAN_REMAP=0): CAN1_RX = PA11, CAN1_TX = PA12.
 * Bit timing (phase 1): 1 Mbit/s @ PCLK1 = 32 MHz with
 *   BRP = 2, BS1 = 13, BS2 = 2, SJW = 1
 *   total quanta = 1+13+2 = 16; tq = 1/(32MHz/2) = 62.5 ns; bit = 1.0 us.
 *   sample point = (1+13)/16 = 87.5 %.
 *
 * No software state — bxCAN's TX mailboxes and RX FIFOs are the state.
 * That keeps this file linkable into common.elf (no .data, no .bss).
 */

#include "bxcan.h"
#include "clocks.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx.h"
#include <string.h>

/* Spin-wait helper with a generous deadline; bxCAN should leave
 * INAK/SLAK within a few hundred bit times. */
static int wait_bit_set(volatile uint32_t *reg, uint32_t mask, uint32_t spins)
{
    while (spins--) if (*reg & mask) return 0;
    return -1;
}
static int wait_bit_clear(volatile uint32_t *reg, uint32_t mask, uint32_t spins)
{
    while (spins--) if (!(*reg & mask)) return 0;
    return -1;
}

int bxcan_init(uint32_t bitrate)
{
    if (bitrate != 1000000U) return -1;   /* phase 1: only 1 Mbit/s */

    /* Clocks: GPIOA + AFIO + CAN1 */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA |
                             LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN1);

    /* Pins (default mapping, no remap):
     *   PA11 — input floating (CAN1_RX)
     *   PA12 — alt-function push-pull 50 MHz (CAN1_TX) */
    LL_GPIO_InitTypeDef io = {0};
    io.Pin   = LL_GPIO_PIN_11;
    io.Mode  = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &io);

    io.Pin        = LL_GPIO_PIN_12;
    io.Mode       = LL_GPIO_MODE_ALTERNATE;
    io.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    io.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &io);

    /* Reset CAN to a known state */
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_CAN1);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_CAN1);

    /* Enter init mode */
    CAN1->MCR |= CAN_MCR_INRQ;
    if (wait_bit_set(&CAN1->MSR, CAN_MSR_INAK, 0x10000)) return -2;

    /* MCR options:
     *   - exit sleep
     *   - no automatic bus-off recovery
     *   - no auto-retransmit (useful for real-time telemetry; for fw
     *     update we'll re-enable later)
     *   - TX FIFO priority by request order
     *   - no time-triggered comm */
    CAN1->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_TTCM | CAN_MCR_ABOM |
                   CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP);

    /* BTR: BRP-1 in [9:0], BS1-1 in [19:16], BS2-1 in [22:20], SJW-1 in [25:24].
     * BRP=2, BS1=13, BS2=2, SJW=1  ->  raw 1, 12, 1, 0. */
    const uint32_t btr =
        (1U <<  0) |   /* BRP - 1 */
        (12U << 16) |  /* BS1 - 1 */
        (1U << 20) |   /* BS2 - 1 */
        (0U << 24);    /* SJW - 1 */
    CAN1->BTR = btr;

    /* Leave init mode */
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (wait_bit_clear(&CAN1->MSR, CAN_MSR_INAK, 0x10000)) return -3;

    /* Default filter: bank 0, 32-bit mask mode, accept-all to FIFO0. */
    CAN1->FMR  |=  CAN_FMR_FINIT;
    CAN1->FA1R &= ~(1U << 0);          /* deactivate bank 0 */
    CAN1->FS1R |=  (1U << 0);          /* 32-bit scale */
    CAN1->FM1R &= ~(1U << 0);          /* mask mode */
    CAN1->FFA1R &= ~(1U << 0);         /* assign to FIFO0 */
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;  /* mask=0 -> match all */
    CAN1->FA1R |=  (1U << 0);          /* activate */
    CAN1->FMR  &= ~CAN_FMR_FINIT;

    return 0;
}

int bxcan_set_filter(const can_filter_t *spec)
{
    if (!spec || spec->bank > 13) return -1;
    const uint32_t mask = 1U << spec->bank;

    /* 16-bit list mode: each filter register holds two STDID-shifted IDs.
     * Filter scale = 16-bit, filter mode = list, FIFO0. */
    CAN1->FMR  |=  CAN_FMR_FINIT;
    CAN1->FA1R &= ~mask;
    CAN1->FS1R &= ~mask;       /* 16-bit scale */
    CAN1->FM1R |=  mask;       /* list mode */
    CAN1->FFA1R &= ~mask;      /* assign to FIFO0 */

    const uint32_t a = ((uint32_t)(spec->id1 & 0x7FFU)) << 5;
    const uint32_t b = ((uint32_t)(spec->id2 & 0x7FFU)) << 5;
    CAN1->sFilterRegister[spec->bank].FR1 = (b << 16) | a;
    CAN1->sFilterRegister[spec->bank].FR2 = (b << 16) | a;

    CAN1->FA1R |=  mask;       /* activate */
    CAN1->FMR  &= ~CAN_FMR_FINIT;
    return 0;
}

int bxcan_send(const can_frame_t *f)
{
    if (!f || f->len > 8) return -1;

    int mb = -1;
    if      (CAN1->TSR & CAN_TSR_TME0) mb = 0;
    else if (CAN1->TSR & CAN_TSR_TME1) mb = 1;
    else if (CAN1->TSR & CAN_TSR_TME2) mb = 2;
    else return 0;             /* all mailboxes busy */

    CAN_TxMailBox_TypeDef *m = &CAN1->sTxMailBox[mb];
    m->TIR = ((f->id & 0x7FFU) << 21) | (f->rtr ? CAN_TI0R_RTR : 0);
    m->TDTR = f->len;

    uint32_t low = 0, high = 0;
    for (uint8_t i = 0; i < f->len && i < 4; i++) low  |= ((uint32_t)f->data[i]) << (8U * i);
    for (uint8_t i = 4; i < f->len;            i++) high |= ((uint32_t)f->data[i]) << (8U * (i - 4));
    m->TDLR = low;
    m->TDHR = high;
    m->TIR |= CAN_TI0R_TXRQ;
    return 1;
}

int bxcan_recv(can_frame_t *out)
{
    if (!out) return -1;
    if (!(CAN1->RF0R & CAN_RF0R_FMP0)) return 0;   /* FIFO0 empty */

    CAN_FIFOMailBox_TypeDef *m = &CAN1->sFIFOMailBox[0];
    out->id  = (m->RIR >> 21) & 0x7FFU;
    out->rtr = (m->RIR & CAN_RI0R_RTR) ? 1U : 0U;
    out->len = (uint8_t)(m->RDTR & 0xF);
    if (out->len > 8) out->len = 8;

    const uint32_t low  = m->RDLR;
    const uint32_t high = m->RDHR;
    for (uint8_t i = 0; i < out->len && i < 4; i++) out->data[i] = (uint8_t)(low  >> (8U * i));
    for (uint8_t i = 4; i < out->len;            i++) out->data[i] = (uint8_t)(high >> (8U * (i - 4)));

    /* Release the FIFO entry. */
    CAN1->RF0R |= CAN_RF0R_RFOM0;
    return 1;
}
