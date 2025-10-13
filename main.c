/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "RP2040.h"
#include "pico/time.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/structs/dma.h"
#include "hardware/structs/watchdog.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"

#ifdef DEBUG
#include <stdio.h>
#define DBG_PRINTF_INIT() { }
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF_INIT() { }
#define DBG_PRINTF(...) { }
#endif

// The bootloader can be entered in three ways:
//  - BOOTLOADER_ENTRY_PIN is low
//  - Watchdog scratch[5] == BOOTLOADER_ENTRY_MAGIC && scratch[6] == ~BOOTLOADER_ENTRY_MAGIC
//  - No valid image header
#define BOOTLOADER_ENTRY_PIN 22
#define BOOTLOADER_ENTRY_MAGIC 0xb105f00d

#define UART_TX_PIN 8 // UART1  to avoid conflicts with the USB serial
#define UART_RX_PIN 9 // UART1  to avoid conflicts with the USB serial
#define UART_BAUD   921600

#define CMD_SYNC   (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define CMD_READ   (('R' << 0) | ('E' << 8) | ('A' << 16) | ('D' << 24))
#define CMD_CSUM   (('C' << 0) | ('S' << 8) | ('U' << 16) | ('M' << 24))
#define CMD_CRC    (('C' << 0) | ('R' << 8) | ('C' << 16) | ('C' << 24))
#define CMD_ERASE  (('E' << 0) | ('R' << 8) | ('A' << 16) | ('S' << 24))
#define CMD_WRITE  (('W' << 0) | ('R' << 8) | ('I' << 16) | ('T' << 24))
#define CMD_SEAL   (('S' << 0) | ('E' << 8) | ('A' << 16) | ('L' << 24))
#define CMD_GO     (('G' << 0) | ('O' << 8) | ('G' << 16) | ('O' << 24))
#define CMD_INFO   (('I' << 0) | ('N' << 8) | ('F' << 16) | ('O' << 24))
#define CMD_REBOOT (('B' << 0) | ('O' << 8) | ('O' << 16) | ('T' << 24))

#define RSP_SYNC (('P' << 0) | ('I' << 8) | ('C' << 16) | ('O' << 24))
#define RSP_OK   (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define RSP_ERR  (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

#define IMAGE_HEADER_OFFSET (12 * 1024)

#define WRITE_ADDR_MIN (XIP_BASE + IMAGE_HEADER_OFFSET + FLASH_SECTOR_SIZE)
#define ERASE_ADDR_MIN (XIP_BASE + IMAGE_HEADER_OFFSET)
#define FLASH_ADDR_MAX (XIP_BASE + PICO_FLASH_SIZE_BYTES)

// ------------------------------------------------------------
// Raisons d'entrée + clignotement LED
// ------------------------------------------------------------
enum boot_reason {
    BOOT_REASON_NONE  = 0,
    BOOT_REASON_PIN   = 2, // 2 bips
    BOOT_REASON_NOIMG = 3, // 3 bips
    BOOT_REASON_WD    = 4, // 4 bips
};

static enum boot_reason g_boot_reason = BOOT_REASON_NONE;

// timings en ms
#define BLINK_ON_MS     120
#define BLINK_OFF_MS    120
#define BLINK_PAUSE_MS  600

static inline bool wd_magic(void) {
    return (watchdog_hw->scratch[5] == BOOTLOADER_ENTRY_MAGIC) &&
           (watchdog_hw->scratch[6] == ~BOOTLOADER_ENTRY_MAGIC);
}

struct image_header {
    uint32_t vtor;
    uint32_t size;
    uint32_t crc;
    uint8_t pad[FLASH_PAGE_SIZE - (3 * 4)];
};
static_assert(sizeof(struct image_header) == FLASH_PAGE_SIZE, "image_header must be FLASH_PAGE_SIZE bytes");

static bool image_header_ok(struct image_header *hdr);

// Détermine la raison d'entrée dans le bootloader
static enum boot_reason compute_boot_reason(struct image_header *hdr) {
    bool pin_low = !gpio_get(BOOTLOADER_ENTRY_PIN);
    bool img_ok  = image_header_ok(hdr);
    bool wd      = wd_magic();

    // Priorité: pas d'image valide > pin bas > watchdog
    if (!img_ok)        return BOOT_REASON_NOIMG;
    if (pin_low)        return BOOT_REASON_PIN;
    if (wd)             return BOOT_REASON_WD;
    return BOOT_REASON_NONE;
}

// Service de clignotement non bloquant à appeler régulièrement
static void blink_service(void) {
    static uint32_t last_ms = 0;
    static uint32_t phase_ms = 0;
    static uint32_t blinks_done = 0;
    static bool led_on = false;

    if (g_boot_reason == BOOT_REASON_NONE) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now == last_ms) return;
    uint32_t dt = now - last_ms;
    last_ms = now;

    phase_ms += dt;

    if (led_on) {
        if (phase_ms >= BLINK_ON_MS) {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            led_on = false;
            phase_ms = 0;
        }
    } else {
        if (blinks_done < (uint32_t)g_boot_reason) {
            if (phase_ms >= BLINK_OFF_MS) {
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
                led_on = true;
                phase_ms = 0;
                blinks_done++;
            }
        } else {
            if (phase_ms >= BLINK_PAUSE_MS) {
                blinks_done = 0;
                phase_ms = 0;
            }
        }
    }
}

// ------------------------------------------------------------

static void disable_interrupts(void)
{
    SysTick->CTRL &= ~1;
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;
}

static void reset_peripherals(void)
{
    reset_block(~(
            RESETS_RESET_IO_QSPI_BITS |
            RESETS_RESET_PADS_QSPI_BITS |
            RESETS_RESET_SYSCFG_BITS |
            RESETS_RESET_PLL_SYS_BITS
    ));
}

static void jump_to_vtor(uint32_t vtor)
{
    uint32_t reset_vector = *(volatile uint32_t *)(vtor + 0x04);

    SCB->VTOR = (volatile uint32_t)(vtor);

    asm volatile("msr msp, %0"::"g"
            (*(volatile uint32_t *)vtor));
    asm volatile("bx %0"::"r" (reset_vector));
}

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_csum(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_csum(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_crc(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_crc(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_erase(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_write(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_write(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_seal(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_go(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t handle_info(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
static uint32_t size_reboot(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_reboot(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

struct command_desc {
    uint32_t opcode;
    uint32_t nargs;
    uint32_t resp_nargs;
    uint32_t (*size)(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
    uint32_t (*handle)(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
};

const struct command_desc cmds[] = {
    { .opcode = CMD_SYNC,  .nargs = 0, .resp_nargs = 0, .size = NULL,       .handle = &handle_sync },
    { .opcode = CMD_READ,  .nargs = 2, .resp_nargs = 0, .size = &size_read, .handle = &handle_read },
    { .opcode = CMD_CSUM,  .nargs = 2, .resp_nargs = 1, .size = &size_csum, .handle = &handle_csum },
    { .opcode = CMD_CRC,   .nargs = 2, .resp_nargs = 1, .size = &size_crc,  .handle = &handle_crc },
    { .opcode = CMD_ERASE, .nargs = 2, .resp_nargs = 0, .size = NULL,       .handle = &handle_erase },
    { .opcode = CMD_WRITE, .nargs = 2, .resp_nargs = 1, .size = &size_write,.handle = &handle_write },
    { .opcode = CMD_SEAL,  .nargs = 3, .resp_nargs = 0, .size = NULL,       .handle = &handle_seal },
    { .opcode = CMD_GO,    .nargs = 1, .resp_nargs = 0, .size = NULL,       .handle = &handle_go },
    { .opcode = CMD_INFO,  .nargs = 0, .resp_nargs = 5, .size = NULL,       .handle = &handle_info },
    { .opcode = CMD_REBOOT,.nargs = 1, .resp_nargs = 0, .size = &size_reboot,.handle = &handle_reboot },
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));
const uint32_t MAX_NARG = 5;
const uint32_t MAX_DATA_LEN = 1024; //FLASH_SECTOR_SIZE;

// ---------- util UART non bloquant qui préserve le clignotement ----------
static void uart_read_exact(uint8_t *buf, size_t len) {
    size_t i = 0;
    while (i < len) {
        while (!uart_is_readable(uart1)) {
            blink_service();        // on continue de clignoter
            tight_loop_contents();
        }
        uart_read_blocking(uart1, &buf[i], 1);
        i++;
    }
}
// ------------------------------------------------------------------------

static void print_int(uint32_t value)
{
    char hex_chars[] = "0123456789ABCDEF";
    uint8_t *val = (uint8_t *)&value;

    for (int i = 0; i < 4; i++)
    {
        char high = hex_chars[(val[i] >> 4) & 0x0F];
        char low  = hex_chars[val[i] & 0x0F];
        uart_write_blocking(uart1, (uint8_t *)&high, 1);
        uart_write_blocking(uart1, (uint8_t *)&low, 1);
        uart_write_blocking(uart1, (uint8_t *)" ", 1);
    }
    uart_write_blocking(uart1, (uint8_t *)"\r\n", 2);
}

static bool is_error(uint32_t status)
{
    return status == RSP_ERR;
}

static uint32_t handle_sync(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    return RSP_SYNC;
}

static uint32_t size_read(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
    uint32_t size = args_in[1];
    if (size > MAX_DATA_LEN) {
        return RSP_ERR;
    }
    *data_len_out = 0;
    *resp_data_len_out = size;
    return RSP_OK;
}

static uint32_t handle_read(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];
    memcpy(resp_data_out, (void *)addr, size);
    return RSP_OK;
}

static uint32_t size_csum(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    if ((addr & 0x3) || (size & 0x3)) return RSP_ERR;

    *data_len_out = 0;
    *resp_data_len_out = 0;
    return RSP_OK;
}

static uint32_t handle_csum(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    uint32_t dummy_dest;
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    int channel = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);

    dma_hw->sniff_data = 0;
    dma_sniffer_enable(channel, 0xf, true);

    dma_channel_configure(channel, &c, &dummy_dest, (void *)addr, size / 4, true);
    dma_channel_wait_for_finish_blocking(channel);

    dma_sniffer_disable();
    dma_channel_unclaim(channel);

    *resp_args_out = dma_hw->sniff_data;
    return RSP_OK;
}

static uint32_t size_crc(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    if ((addr & 0x3) || (size & 0x3)) return RSP_ERR;

    *data_len_out = 0;
    *resp_data_len_out = 0;
    return RSP_OK;
}

// ptr must be 4-byte aligned and len must be a multiple of 4
static uint32_t calc_crc32(void *ptr, uint32_t len)
{
    uint32_t dummy_dest, crc;

    int channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);

    // Seed the CRC calculation
    dma_hw->sniff_data = 0xffffffff;

    // Mode 1 + reverse bits -> CRC IEEE802.3
    dma_sniffer_enable(channel, 0x1, true);
    dma_hw->sniff_ctrl |= DMA_SNIFF_CTRL_OUT_REV_BITS;

    dma_channel_configure(channel, &c, &dummy_dest, ptr, len / 4, true);
    dma_channel_wait_for_finish_blocking(channel);

    crc = dma_hw->sniff_data ^ 0xffffffff;

    dma_sniffer_disable();
    dma_channel_unclaim(channel);

    return crc;
}

static uint32_t handle_crc(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];
    resp_args_out[0] = calc_crc32((void *)addr, size);
    return RSP_OK;
}

static uint32_t handle_erase(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    if ((addr < ERASE_ADDR_MIN) || (addr + size >= FLASH_ADDR_MAX)) return RSP_ERR;
    if ((addr & (FLASH_SECTOR_SIZE - 1)) || (size & (FLASH_SECTOR_SIZE - 1))) return RSP_ERR;

    flash_range_erase(addr - XIP_BASE, size);
    return RSP_OK;
}

static uint32_t size_write(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    if ((addr < WRITE_ADDR_MIN) || (addr + size >= FLASH_ADDR_MAX)) return RSP_ERR;
    if ((addr & (FLASH_PAGE_SIZE - 1)) || (size & (FLASH_PAGE_SIZE -1))) return RSP_ERR;
    if (size > MAX_DATA_LEN) return RSP_ERR;

    *data_len_out = size;
    *resp_data_len_out = 0;
    return RSP_OK;
}

static uint32_t handle_write(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    uint32_t addr = args_in[0];
    uint32_t size = args_in[1];

    flash_range_program(addr - XIP_BASE, data_in, size);
    resp_args_out[0] = calc_crc32((void *)addr, size);
    return RSP_OK;
}

static bool image_header_ok(struct image_header *hdr)
{
    uint32_t *vtor = (uint32_t *)hdr->vtor;

    uint32_t calc = calc_crc32((void *)hdr->vtor, hdr->size);

    if (calc != hdr->crc) return false;
    if (vtor[0] < SRAM_BASE) return false;
    if (vtor[1] > hdr->vtor + hdr->size) return false;
    if (!(vtor[1] & 1)) return false;

    return true;
}

static uint32_t handle_seal(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    struct image_header hdr = {
        .vtor = args_in[0],
        .size = args_in[1],
        .crc  = args_in[2],
    };

    if ((hdr.vtor & 0xff) || (hdr.size & 0x3)) return RSP_ERR;
    if (!image_header_ok(&hdr)) return RSP_ERR;

    flash_range_erase(IMAGE_HEADER_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(IMAGE_HEADER_OFFSET, (const uint8_t *)&hdr, sizeof(hdr));

    struct image_header *check = (struct image_header *)(XIP_BASE + IMAGE_HEADER_OFFSET);
    if (memcmp(&hdr, check, sizeof(hdr))) return RSP_ERR;

    return RSP_OK;
}

static uint32_t handle_go(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    disable_interrupts();
    reset_peripherals();
    jump_to_vtor(args_in[0]);
    while(1);
    return RSP_ERR;
}

static uint32_t handle_info(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    resp_args_out[0] = WRITE_ADDR_MIN;
    resp_args_out[1] = (XIP_BASE + PICO_FLASH_SIZE_BYTES) - WRITE_ADDR_MIN;
    resp_args_out[2] = FLASH_SECTOR_SIZE;
    resp_args_out[3] = FLASH_PAGE_SIZE;
    resp_args_out[4] = MAX_DATA_LEN;
    return RSP_OK;
}

static void do_reboot(bool to_bootloader)
{
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    if (to_bootloader) {
        watchdog_hw->scratch[5] = BOOTLOADER_ENTRY_MAGIC;
        watchdog_hw->scratch[6] = ~BOOTLOADER_ENTRY_MAGIC;
    } else {
        watchdog_hw->scratch[5] = 0;
        watchdog_hw->scratch[6] = 0;
    }
    watchdog_reboot(0, 0, 0);
    while (1) {
        tight_loop_contents();
        asm("");
    }
}

static uint32_t size_reboot(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
    *data_len_out = 0;
    *resp_data_len_out = 0;
    return RSP_OK;
}

static uint32_t handle_reboot(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
    do_reboot(args_in[0]);
    return RSP_ERR;
}

static const struct command_desc *find_command_desc(uint32_t opcode)
{
    for (unsigned int i = 0; i < N_CMDS; i++) {
        if (cmds[i].opcode == opcode) return &cmds[i];
    }
    return NULL;
}

struct cmd_context {
    uint8_t *uart_buf;
    const struct command_desc *desc;
    uint32_t opcode;
    uint32_t status;
    uint32_t *args;
    uint8_t *data;
    uint32_t *resp_args;
    uint8_t *resp_data;
    uint32_t data_len;
    uint32_t resp_data_len;
};

enum state {
    STATE_WAIT_FOR_SYNC,
    STATE_READ_OPCODE,
    STATE_READ_ARGS,
    STATE_READ_DATA,
    STATE_HANDLE_DATA,
    STATE_ERROR,
};

// Attente de sync non bloquante avec clignotement continu
static enum state state_wait_for_sync(struct cmd_context *ctx)
{
    int idx = 0;
    uint8_t *recv = (uint8_t *)&ctx->opcode;
    uint8_t *match = (uint8_t *)&ctx->status;

    ctx->status = CMD_SYNC;

    while (idx < (int)sizeof(ctx->opcode)) {
        while (!uart_is_readable(uart1)) {
            blink_service();
            tight_loop_contents();
        }
        uart_read_blocking(uart1, &recv[idx], 1);

        if (recv[idx] != match[idx]) {
            idx = 0;
        } else {
            idx++;
        }
    }

    assert(ctx->opcode == CMD_SYNC);

    // On ne coupe plus la LED ici: clignotement maintenu tant qu'on reste en bootloader
    return STATE_READ_ARGS;
}

static enum state state_read_opcode(struct cmd_context *ctx)
{
    uart_read_exact((uint8_t *)&ctx->opcode, sizeof(ctx->opcode));
    return STATE_READ_ARGS;
}

static enum state state_read_args(struct cmd_context *ctx)
{
    const struct command_desc *desc = find_command_desc(ctx->opcode);
    if (!desc) {
        ctx->status = RSP_ERR;
        return STATE_ERROR;
    }

    ctx->desc = desc;
    ctx->args = (uint32_t *)(ctx->uart_buf + sizeof(ctx->opcode));
    ctx->data = (uint8_t *)(ctx->args + desc->nargs);
    ctx->resp_args = ctx->args;
    ctx->resp_data = (uint8_t *)(ctx->resp_args + desc->resp_nargs);

    uart_read_exact((uint8_t *)ctx->args, sizeof(*ctx->args) * desc->nargs);
    return STATE_READ_DATA;
}

static enum state state_read_data(struct cmd_context *ctx)
{
    const struct command_desc *desc = ctx->desc;

    if (desc->size) {
        ctx->status = desc->size(ctx->args, &ctx->data_len, &ctx->resp_data_len);
        if (is_error(ctx->status)) return STATE_ERROR;
    } else {
        ctx->data_len = 0;
        ctx->resp_data_len = 0;
    }

    uart_read_exact((uint8_t *)ctx->data, ctx->data_len);
    return STATE_HANDLE_DATA;
}

static enum state state_handle_data(struct cmd_context *ctx)
{
    const struct command_desc *desc = ctx->desc;

    if (desc->handle) {
        ctx->status = desc->handle(ctx->args, ctx->data, ctx->resp_args, ctx->resp_data);
        if (is_error(ctx->status)) return STATE_ERROR;
    } else {
        ctx->status = RSP_OK;
    }

    size_t resp_len = sizeof(ctx->status) + (sizeof(*ctx->resp_args) * desc->resp_nargs) + ctx->resp_data_len;
    memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
    uart_write_blocking(uart1, ctx->uart_buf, resp_len);

    return STATE_READ_OPCODE;
}

static enum state state_error(struct cmd_context *ctx)
{
    size_t resp_len = sizeof(ctx->status);
    memcpy(ctx->uart_buf, &ctx->status, sizeof(ctx->status));
    uart_write_blocking(uart1, ctx->uart_buf, resp_len);
    return STATE_WAIT_FOR_SYNC;
}

static void print_uint32_decimal(uint32_t value)
{
    char buf[10];
    int i = 0;

    if (value == 0) {
        char c = '0';
        uart_write_blocking(uart1, (uint8_t *)&c, 1);
        uart_write_blocking(uart1, (uint8_t *)"\r\n", 2);
        return;
    }

    while (value > 0 && i < (int)sizeof(buf)) {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }

    while (i--) {
        uart_write_blocking(uart1, (uint8_t *)&buf[i], 1);
    }
    uart_write_blocking(uart1, (uint8_t *)"\r\n", 2);
}

static void uart_write_str(const char *str) {
    while (*str) {
        uart_write_blocking(uart1, (const uint8_t *)str, 1);
        str++;
    }
}

int main(void)
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    gpio_init(BOOTLOADER_ENTRY_PIN);
    gpio_pull_up(BOOTLOADER_ENTRY_PIN);
    gpio_set_dir(BOOTLOADER_ENTRY_PIN, 0);

    sleep_ms(10);

    struct image_header *hdr = (struct image_header *)(XIP_BASE + IMAGE_HEADER_OFFSET);

    // Détermine la raison d'entrée
    g_boot_reason = compute_boot_reason(hdr);

    if (g_boot_reason == BOOT_REASON_NONE) {
        // Image valide et aucune demande d'entrée -> jump
        uint32_t vtor = *((uint32_t *)(XIP_BASE + IMAGE_HEADER_OFFSET));
        disable_interrupts();
        reset_peripherals();
        jump_to_vtor(vtor);
    }

    DBG_PRINTF_INIT();

    uart_init(uart1, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);

    struct cmd_context ctx;
    uint8_t uart_buf[(sizeof(uint32_t) * (1 + MAX_NARG)) + MAX_DATA_LEN];
    ctx.uart_buf = uart_buf;
    enum state state = STATE_WAIT_FOR_SYNC;

    while (1) {
        // clignotement continu tant qu'on est en bootloader
        blink_service();

        switch (state) {
        case STATE_WAIT_FOR_SYNC:
            DBG_PRINTF("wait_for_sync\n");
            state = state_wait_for_sync(&ctx);
            DBG_PRINTF("wait_for_sync done\n");
            break;
        case STATE_READ_OPCODE:
            DBG_PRINTF("read_opcode\n");
            state = state_read_opcode(&ctx);
            DBG_PRINTF("read_opcode done\n");
            break;
        case STATE_READ_ARGS:
            DBG_PRINTF("read_args\n");
            state = state_read_args(&ctx);
            DBG_PRINTF("read_args done\n");
            break;
        case STATE_READ_DATA:
            DBG_PRINTF("read_data\n");
            state = state_read_data(&ctx);
            DBG_PRINTF("read_data done\n");
            break;
        case STATE_HANDLE_DATA:
            DBG_PRINTF("handle_data\n");
            state = state_handle_data(&ctx);
            DBG_PRINTF("handle_data done\n");
            break;
        case STATE_ERROR:
            DBG_PRINTF("error\n");
            state = state_error(&ctx);
            DBG_PRINTF("error done\n");
            break;
        }
    }

    return 0;
}
