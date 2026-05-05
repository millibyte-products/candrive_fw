#include "stream_handler.h"
#include "user_store.h"

typedef enum
{
    IDLE,
    READ,
    WRITE,
    COMPLETE,
} stream_state_t;

static stream_state_t _stream_state = IDLE;
static uint32_t op_length = 0;
static uint32_t remaining_length = 0;
static uintptr_t stream_ptr = 0;
static uintptr_t expected_crc = 0;
static uint8_t page_buffer[PAGE_SIZE];

void configure_stream(uint8_t stream_target, uint8_t flags)
{
    if (_stream_state != IDLE)
    {
        return;
    }
    switch (stream_target)
    {
    case STREAM_TARGET_USER_STORE:
    {
        // Start a write operation to user store
        _stream_state = WRITE;
        op_length = 0;
        remaining_length = 0;
        // stream_ptr = (uintptr_t)user_store_get_write_ptr();
    }
    break;
    case STREAM_TARGET_FIRMWARE:
    {
    }
    break;

    default:
        break;
    }
}
// Start a read operation to firmware

void handle_stream_start(uint16_t stream_length, uint32_t checksum)
{
    if (_stream_state != IDLE)
    {
        return;
    }
}

void handle_stream_data(uint8_t sequence_id, uint8_t *buffer, size_t length)
{
}