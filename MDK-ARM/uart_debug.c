#include "uart_debug.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static UART_HandleTypeDef *debug_huart;
static char ring_buf[DEBUG_BUF_SIZE];
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;
static volatile bool tx_busy = false;

static char tx_chunk[256];

void Debug_Init(UART_HandleTypeDef *huart)
{
    debug_huart = huart;
    head = tail = 0;
    tx_busy = false;
}

static uint16_t ring_free(void)
{
    if (head >= tail) {
        return DEBUG_BUF_SIZE - 1 - (head - tail);
    }
    return tail - head - 1;
}

static uint16_t ring_used(void)
{
    if (head >= tail) {
        return head - tail;
    }
    return DEBUG_BUF_SIZE - tail + head;
}

void Debug_Print(const char *str)
{
    uint16_t len = strlen(str);
    if (len > ring_free()) {
        return;  
    }
    
    for (uint16_t i = 0; i < len; i++) {
        ring_buf[head] = str[i];
        head = (head + 1) % DEBUG_BUF_SIZE;
    }
}

void Debug_Printf(const char *fmt, ...)
{
    static char temp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);
    Debug_Print(temp);
}

void Debug_Process(void)
{
    if (tx_busy || ring_used() == 0) {
        return;
    }
    
    uint16_t chunk_len = 0;
    uint16_t avail = ring_used();
    uint16_t max_len = sizeof(tx_chunk);
    
    while (chunk_len < max_len && chunk_len < avail) {
        tx_chunk[chunk_len] = ring_buf[tail];
        tail = (tail + 1) % DEBUG_BUF_SIZE;
        chunk_len++;
    }
    
    if (chunk_len > 0) {
        tx_busy = true;
        HAL_UART_Transmit_DMA(debug_huart, (uint8_t*)tx_chunk, chunk_len);
    }
}

void Debug_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == debug_huart) {
        tx_busy = false;
    }
}