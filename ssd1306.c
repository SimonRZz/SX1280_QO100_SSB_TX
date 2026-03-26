// ssd1306.c - SSD1306 OLED driver with DMA for Raspberry Pi Pico
// 128x64 monochrome, zero-CPU display transfer via DMA → I2C TX FIFO

#include "ssd1306.h"
#include <string.h>
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/regs/dreq.h"

// ============================================================
// Framebuffer
// ============================================================
static uint8_t framebuf[SSD1306_WIDTH * SSD1306_PAGES];

// ============================================================
// DMA state
// ============================================================
#define OLED_DMA_TOTAL (1 + SSD1306_WIDTH * SSD1306_PAGES)  // 0x40 prefix + 1024 data
static uint16_t dma_buf[OLED_DMA_TOTAL];  // each entry → I2C data_cmd word
static int dma_chan = -1;

// ============================================================
// Minimal 5x7 ASCII font (characters 32-126)
// Each char is 5 bytes wide, each byte is a column with LSB at top
// ============================================================
static const uint8_t font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, // Space
    0x00,0x00,0x5F,0x00,0x00, // !
    0x00,0x07,0x00,0x07,0x00, // "
    0x14,0x7F,0x14,0x7F,0x14, // #
    0x24,0x2A,0x7F,0x2A,0x12, // $
    0x23,0x13,0x08,0x64,0x62, // %
    0x36,0x49,0x55,0x22,0x50, // &
    0x00,0x05,0x03,0x00,0x00, // '
    0x00,0x1C,0x22,0x41,0x00, // (
    0x00,0x41,0x22,0x1C,0x00, // )
    0x08,0x2A,0x1C,0x2A,0x08, // *
    0x08,0x08,0x3E,0x08,0x08, // +
    0x00,0x50,0x30,0x00,0x00, // ,
    0x08,0x08,0x08,0x08,0x08, // -
    0x00,0x60,0x60,0x00,0x00, // .
    0x20,0x10,0x08,0x04,0x02, // /
    0x3E,0x51,0x49,0x45,0x3E, // 0
    0x00,0x42,0x7F,0x40,0x00, // 1
    0x42,0x61,0x51,0x49,0x46, // 2
    0x21,0x41,0x45,0x4B,0x31, // 3
    0x18,0x14,0x12,0x7F,0x10, // 4
    0x27,0x45,0x45,0x45,0x39, // 5
    0x3C,0x4A,0x49,0x49,0x30, // 6
    0x01,0x71,0x09,0x05,0x03, // 7
    0x36,0x49,0x49,0x49,0x36, // 8
    0x06,0x49,0x49,0x29,0x1E, // 9
    0x00,0x36,0x36,0x00,0x00, // :
    0x00,0x56,0x36,0x00,0x00, // ;
    0x00,0x08,0x14,0x22,0x41, // <
    0x14,0x14,0x14,0x14,0x14, // =
    0x41,0x22,0x14,0x08,0x00, // >
    0x02,0x01,0x51,0x09,0x06, // ?
    0x32,0x49,0x79,0x41,0x3E, // @
    0x7E,0x11,0x11,0x11,0x7E, // A
    0x7F,0x49,0x49,0x49,0x36, // B
    0x3E,0x41,0x41,0x41,0x22, // C
    0x7F,0x41,0x41,0x22,0x1C, // D
    0x7F,0x49,0x49,0x49,0x41, // E
    0x7F,0x09,0x09,0x01,0x01, // F
    0x3E,0x41,0x41,0x51,0x32, // G
    0x7F,0x08,0x08,0x08,0x7F, // H
    0x00,0x41,0x7F,0x41,0x00, // I
    0x20,0x40,0x41,0x3F,0x01, // J
    0x7F,0x08,0x14,0x22,0x41, // K
    0x7F,0x40,0x40,0x40,0x40, // L
    0x7F,0x02,0x04,0x02,0x7F, // M
    0x7F,0x04,0x08,0x10,0x7F, // N
    0x3E,0x41,0x41,0x41,0x3E, // O
    0x7F,0x09,0x09,0x09,0x06, // P
    0x3E,0x41,0x51,0x21,0x5E, // Q
    0x7F,0x09,0x19,0x29,0x46, // R
    0x46,0x49,0x49,0x49,0x31, // S
    0x01,0x01,0x7F,0x01,0x01, // T
    0x3F,0x40,0x40,0x40,0x3F, // U
    0x1F,0x20,0x40,0x20,0x1F, // V
    0x7F,0x20,0x18,0x20,0x7F, // W
    0x63,0x14,0x08,0x14,0x63, // X
    0x03,0x04,0x78,0x04,0x03, // Y
    0x61,0x51,0x49,0x45,0x43, // Z
    0x00,0x00,0x7F,0x41,0x41, // [
    0x02,0x04,0x08,0x10,0x20, // backslash
    0x41,0x41,0x7F,0x00,0x00, // ]
    0x04,0x02,0x01,0x02,0x04, // ^
    0x40,0x40,0x40,0x40,0x40, // _
    0x00,0x01,0x02,0x04,0x00, // `
    0x20,0x54,0x54,0x54,0x78, // a
    0x7F,0x48,0x44,0x44,0x38, // b
    0x38,0x44,0x44,0x44,0x20, // c
    0x38,0x44,0x44,0x48,0x7F, // d
    0x38,0x54,0x54,0x54,0x18, // e
    0x08,0x7E,0x09,0x01,0x02, // f
    0x08,0x14,0x54,0x54,0x3C, // g
    0x7F,0x08,0x04,0x04,0x78, // h
    0x00,0x44,0x7D,0x40,0x00, // i
    0x20,0x40,0x44,0x3D,0x00, // j
    0x00,0x7F,0x10,0x28,0x44, // k
    0x00,0x41,0x7F,0x40,0x00, // l
    0x7C,0x04,0x18,0x04,0x78, // m
    0x7C,0x08,0x04,0x04,0x78, // n
    0x38,0x44,0x44,0x44,0x38, // o
    0x7C,0x14,0x14,0x14,0x08, // p
    0x08,0x14,0x14,0x18,0x7C, // q
    0x7C,0x08,0x04,0x04,0x08, // r
    0x48,0x54,0x54,0x54,0x20, // s
    0x04,0x3F,0x44,0x40,0x20, // t
    0x3C,0x40,0x40,0x20,0x7C, // u
    0x1C,0x20,0x40,0x20,0x1C, // v
    0x3C,0x40,0x30,0x40,0x3C, // w
    0x44,0x28,0x10,0x28,0x44, // x
    0x0C,0x50,0x50,0x50,0x3C, // y
    0x44,0x64,0x54,0x4C,0x44, // z
    0x00,0x08,0x36,0x41,0x00, // {
    0x00,0x00,0x7F,0x00,0x00, // |
    0x00,0x41,0x36,0x08,0x00, // }
    0x08,0x04,0x08,0x10,0x08, // ~
};

// ============================================================
// SSD1306 init sequence
// ============================================================
static const uint8_t ssd1306_init_cmds[] = {
    0xAE,       // Display OFF
    0xD5, 0x80, // Set display clock div
    0xA8, 0x3F, // Set multiplex ratio (64-1)
    0xD3, 0x00, // Set display offset = 0
    0x40,       // Set start line = 0
    0x8D, 0x14, // Charge pump ON
    0x20, 0x00, // Horizontal addressing mode
    0xA1,       // Segment remap
    0xC8,       // COM scan direction remapped
    0xDA, 0x12, // COM pins config for 128x64
    0x81, 0xCF, // Set contrast
    0xD9, 0xF1, // Set precharge period
    0xDB, 0x40, // Set VCOMH deselect level
    0xA4,       // Entire display ON (follow RAM)
    0xA6,       // Normal display
    0xAF,       // Display ON
};

// ============================================================
// Low-level I2C command (blocking, used for init + addressing)
// ============================================================
static void ssd1306_write_cmd(i2c_inst_t *i2c, uint8_t cmd) {
    uint8_t buf[2] = { 0x00, cmd };
    i2c_write_blocking(i2c, SSD1306_ADDR, buf, 2, false);
}

// ============================================================
// Init
// ============================================================
void ssd1306_init(i2c_inst_t *i2c) {
    for (size_t i = 0; i < sizeof(ssd1306_init_cmds); i++) {
        ssd1306_write_cmd(i2c, ssd1306_init_cmds[i]);
    }

    // Claim a DMA channel
    dma_chan = dma_claim_unused_channel(true);

    ssd1306_clear();
    ssd1306_display(i2c);
}

// ============================================================
// Clear
// ============================================================
void ssd1306_clear(void) {
    memset(framebuf, 0, sizeof(framebuf));
}

void ssd1306_clear_pages(int page_start, int page_end) {
    if (page_start < 0) page_start = 0;
    if (page_end >= SSD1306_PAGES) page_end = SSD1306_PAGES - 1;
    for (int p = page_start; p <= page_end; p++) {
        memset(&framebuf[p * SSD1306_WIDTH], 0, SSD1306_WIDTH);
    }
}

// ============================================================
// Blocking display (init only)
// ============================================================
void ssd1306_display(i2c_inst_t *i2c) {
    ssd1306_write_cmd(i2c, 0x21); ssd1306_write_cmd(i2c, 0); ssd1306_write_cmd(i2c, SSD1306_WIDTH - 1);
    ssd1306_write_cmd(i2c, 0x22); ssd1306_write_cmd(i2c, 0); ssd1306_write_cmd(i2c, SSD1306_PAGES - 1);

    const int chunk = 128;
    int total = SSD1306_WIDTH * SSD1306_PAGES;
    uint8_t txbuf[1 + 128];
    for (int offset = 0; offset < total; offset += chunk) {
        int len = total - offset;
        if (len > chunk) len = chunk;
        txbuf[0] = 0x40;
        memcpy(&txbuf[1], &framebuf[offset], len);
        i2c_write_blocking(i2c, SSD1306_ADDR, txbuf, 1 + len, false);
    }
}

// ============================================================
// DMA-based non-blocking display
// ============================================================
//
// How it works:
//   The RP2350 I2C controller has a TX FIFO fed via the IC_DATA_CMD register.
//   Each 32-bit write to data_cmd contains: data[7:0], CMD[8], STOP[9], RESTART[10].
//   The controller auto-sends START + slave address when the first data word is written.
//
//   We build a uint16_t buffer:
//     [0]   = 0x40   (SSD1306 data mode prefix, no STOP)
//     [1..1023] = framebuf bytes (no STOP)
//     [1024] = last framebuf byte | 0x200 (STOP bit)
//
//   DMA reads this buffer and writes each entry to &i2c_hw->data_cmd (as 16-bit).
//   DREQ paces the DMA so it only writes when the TX FIFO has room.
//   The CPU is 100% free during the ~25ms transfer.

bool ssd1306_dma_busy(void) {
    if (dma_chan < 0) return false;
    return dma_channel_is_busy(dma_chan);
}

bool ssd1306_display_dma(i2c_inst_t *i2c) {
    if (dma_chan < 0) return false;

    // Don't start if previous transfer still running
    if (dma_channel_is_busy(dma_chan)) return false;

    // --- Send addressing commands (blocking, very fast ~60us) ---
    ssd1306_write_cmd(i2c, 0x21); ssd1306_write_cmd(i2c, 0); ssd1306_write_cmd(i2c, SSD1306_WIDTH - 1);
    ssd1306_write_cmd(i2c, 0x22); ssd1306_write_cmd(i2c, 0); ssd1306_write_cmd(i2c, SSD1306_PAGES - 1);

    // --- Build DMA buffer ---
    const int fb_size = SSD1306_WIDTH * SSD1306_PAGES;  // 1024

    // First entry: 0x40 = I2C data mode control byte
    dma_buf[0] = 0x40;

    // Copy framebuffer bytes, each as a uint16_t word
    for (int i = 0; i < fb_size; i++) {
        dma_buf[1 + i] = (uint16_t)framebuf[i];
    }
    // Set STOP bit on the very last word
    dma_buf[OLED_DMA_TOTAL - 1] |= (1u << 9);

    // --- Get I2C hardware pointers ---
    i2c_hw_t *hw = i2c_get_hw(i2c);

    // --- Configure DMA ---
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);     // 16-bit writes
    channel_config_set_read_increment(&c, true);                // read from buffer
    channel_config_set_write_increment(&c, false);              // write to same register
    channel_config_set_dreq(&c, I2C_DREQ_NUM(i2c, true));      // pace by I2C TX FIFO

    dma_channel_configure(
        dma_chan,
        &c,
        &hw->data_cmd,     // write address: I2C data command register
        dma_buf,            // read address: our prepared buffer
        OLED_DMA_TOTAL,     // number of 16-bit transfers
        true                // start immediately
    );

    return true;
}

// ============================================================
// Drawing primitives (same as before - all operate on framebuf)
// ============================================================

void ssd1306_set_pixel(int x, int y, bool on) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    int page = y / 8;
    int bit = y % 8;
    if (on)
        framebuf[page * SSD1306_WIDTH + x] |= (1 << bit);
    else
        framebuf[page * SSD1306_WIDTH + x] &= ~(1 << bit);
}

void ssd1306_hline(int x0, int x1, int y) {
    for (int x = x0; x <= x1; x++) {
        ssd1306_set_pixel(x, y, true);
    }
}

void ssd1306_draw_char(int x, int page, char c) {
    if (c < 32 || c > 126) c = '?';
    int idx = (c - 32) * 5;
    if (page < 0 || page >= SSD1306_PAGES) return;
    for (int col = 0; col < 5; col++) {
        int px = x + col;
        if (px >= 0 && px < SSD1306_WIDTH)
            framebuf[page * SSD1306_WIDTH + px] = font5x7[idx + col];
    }
    int px = x + 5;
    if (px >= 0 && px < SSD1306_WIDTH)
        framebuf[page * SSD1306_WIDTH + px] = 0x00;
}

void ssd1306_draw_string(int x, int page, const char *str) {
    while (*str) {
        ssd1306_draw_char(x, page, *str);
        x += 6;
        str++;
    }
}

void ssd1306_draw_string_bold(int x, int page, const char *str) {
    if (page < 0 || page >= SSD1306_PAGES) return;
    while (*str) {
        char c = *str;
        if (c < 32 || c > 126) c = '?';
        int idx = (c - 32) * 5;
        for (int col = 0; col < 5; col++) {
            uint8_t d = font5x7[idx + col];
            int px = x + col;
            if (px >= 0 && px < SSD1306_WIDTH)
                framebuf[page * SSD1306_WIDTH + px] |= d;
            px = x + col + 1;
            if (px >= 0 && px < SSD1306_WIDTH)
                framebuf[page * SSD1306_WIDTH + px] |= d;
        }
        // gap
        int px = x + 6;
        if (px >= 0 && px < SSD1306_WIDTH)
            framebuf[page * SSD1306_WIDTH + px] = 0x00;
        x += 7;
        str++;
    }
}

void ssd1306_draw_string_bold_y(int x, int y, const char *str) {
    while (*str) {
        char c = *str;
        if (c < 32 || c > 126) c = '?';
        int idx = (c - 32) * 5;
        for (int col = 0; col < 5; col++) {
            uint8_t d = font5x7[idx + col];
            for (int bit = 0; bit < 7; bit++) {
                if (d & (1 << bit)) {
                    int py = y + bit;
                    int px = x + col;
                    if (px >= 0 && px < SSD1306_WIDTH && py >= 0 && py < SSD1306_HEIGHT) {
                        framebuf[(py / 8) * SSD1306_WIDTH + px] |= (1 << (py % 8));
                    }
                    px = x + col + 1;  // bold: +1px right
                    if (px >= 0 && px < SSD1306_WIDTH && py >= 0 && py < SSD1306_HEIGHT) {
                        framebuf[(py / 8) * SSD1306_WIDTH + px] |= (1 << (py % 8));
                    }
                }
            }
        }
        x += 7;
        str++;
    }
}

void ssd1306_draw_string_bold_y_inv(int x, int y, const char *str, int field_w) {
    // Inverted bold: white background, black text
    // Fill background rectangle first
    for (int fy = y - 1; fy <= y + 7; fy++) {
        for (int fx = x - 1; fx < x + field_w; fx++) {
            if (fx >= 0 && fx < SSD1306_WIDTH && fy >= 0 && fy < SSD1306_HEIGHT)
                framebuf[(fy / 8) * SSD1306_WIDTH + fx] |= (1 << (fy % 8));
        }
    }
    // Draw text by CLEARING pixels (inverted)
    while (*str) {
        char c = *str;
        if (c < 32 || c > 126) c = '?';
        int idx = (c - 32) * 5;
        for (int col = 0; col < 5; col++) {
            uint8_t d = font5x7[idx + col];
            for (int bit = 0; bit < 7; bit++) {
                if (d & (1 << bit)) {
                    int py = y + bit;
                    int px = x + col;
                    if (px >= 0 && px < SSD1306_WIDTH && py >= 0 && py < SSD1306_HEIGHT)
                        framebuf[(py / 8) * SSD1306_WIDTH + px] &= ~(1 << (py % 8));
                    px = x + col + 1;  // bold: +1px right
                    if (px >= 0 && px < SSD1306_WIDTH && py >= 0 && py < SSD1306_HEIGHT)
                        framebuf[(py / 8) * SSD1306_WIDTH + px] &= ~(1 << (py % 8));
                }
            }
        }
        x += 7;
        str++;
    }
}

void ssd1306_draw_string_2x(int x, int page, const char *str) {
    if (page < 0 || page + 1 >= SSD1306_PAGES) return;
    while (*str) {
        char c = *str;
        if (c < 32 || c > 126) c = '?';
        int idx = (c - 32) * 5;
        for (int col = 0; col < 5; col++) {
            uint8_t src = font5x7[idx + col];
            uint16_t expanded = 0;
            for (int bit = 0; bit < 7; bit++) {
                if (src & (1 << bit))
                    expanded |= (3 << (bit * 2));
            }
            uint8_t lo = (uint8_t)(expanded & 0xFF);
            uint8_t hi = (uint8_t)((expanded >> 8) & 0xFF);
            for (int dx = 0; dx < 2; dx++) {
                int px = x + col * 2 + dx;
                if (px >= 0 && px < SSD1306_WIDTH) {
                    framebuf[page * SSD1306_WIDTH + px] = lo;
                    framebuf[(page + 1) * SSD1306_WIDTH + px] = hi;
                }
            }
        }
        for (int dx = 0; dx < 2; dx++) {
            int px = x + 10 + dx;
            if (px >= 0 && px < SSD1306_WIDTH) {
                framebuf[page * SSD1306_WIDTH + px] = 0x00;
                framebuf[(page + 1) * SSD1306_WIDTH + px] = 0x00;
            }
        }
        x += 12;
        str++;
    }
}

void ssd1306_vline(int x, int y0, int y1) {
    if (x < 0 || x >= SSD1306_WIDTH) return;
    for (int y = y0; y <= y1; y++)
        ssd1306_set_pixel(x, y, true);
}

void ssd1306_fill_rect(int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            ssd1306_set_pixel(x, y, true);
}

void ssd1306_scroll_region_left(int x0, int y0, int x1, int y1) {
    int page_start = y0 / 8;
    int page_end = y1 / 8;
    for (int page = page_start; page <= page_end; page++) {
        int py0 = page * 8;
        int py1 = py0 + 7;
        int bit_lo = (y0 > py0) ? (y0 - py0) : 0;
        int bit_hi = (y1 < py1) ? (y1 - py0) : 7;
        if (bit_lo == 0 && bit_hi == 7) {
            uint8_t *row = &framebuf[page * SSD1306_WIDTH];
            memmove(&row[x0], &row[x0 + 1], x1 - x0);
            row[x1] = 0x00;
        } else {
            uint8_t mask = 0;
            for (int b = bit_lo; b <= bit_hi; b++) mask |= (1 << b);
            uint8_t *row = &framebuf[page * SSD1306_WIDTH];
            for (int x = x0; x < x1; x++)
                row[x] = (row[x] & ~mask) | (row[x + 1] & mask);
            row[x1] &= ~mask;
        }
    }
}

void ssd1306_draw_wf_column(int x, int y_top, int y_bot, uint8_t level_0_8) {
    if (x < 0 || x >= SSD1306_WIDTH) return;
    int height = y_bot - y_top + 1;
    int bar = (int)level_0_8;
    if (bar > height) bar = height;
    for (int y = y_top; y <= y_bot; y++)
        ssd1306_set_pixel(x, y, false);
    for (int i = 0; i < bar; i++)
        ssd1306_set_pixel(x, y_bot - i, true);
}

uint8_t *ssd1306_get_framebuf(void) {
    return framebuf;
}
