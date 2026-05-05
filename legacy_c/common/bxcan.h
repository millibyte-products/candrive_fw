#ifndef CANDRIVE_BXCAN_H
#define CANDRIVE_BXCAN_H

#include <stdint.h>
#include "common_api.h"  /* can_frame_t, can_filter_t */

/* bxCAN1 LL driver — PA11 (CAN1_RX) / PA12 (CAN1_TX), polled.
 * No interrupts in phase 1 — both bootloader and app pump RX from
 * their main loop. */

 /* bitrate: only 1_000_000 supported in phase 1 (assert). Returns 0 ok. */
int bxcan_init(uint32_t bitrate);

/* Configure one 16-bit list-mode filter bank to accept up to two IDs. */
int bxcan_set_filter(const can_filter_t* spec);

/* Send a frame. Returns 1 if accepted into a TX mailbox, 0 if all full. */
int bxcan_send(const can_frame_t* frame);

/* Try to receive from FIFO0. Returns 1 and fills *out, or 0 if empty. */
int bxcan_recv(can_frame_t* out);

#endif /* CANDRIVE_BXCAN_H */
