#include "lcd.h"
#include "command.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_netif.h"

static const char *TAG = "oled";

/* ── I2C handles ────────────────────────────────────── */
static i2c_master_bus_handle_t  s_bus;
static i2c_master_dev_handle_t  s_dev;

/* ── Framebuffer (128×64 = 1024 bytes) ──────────────── */
static uint8_t s_fb[SSD1306_WIDTH * SSD1306_PAGES];

/* ═══════════════════════════════════════════════════════
   5×7 pixel font (ASCII 32–126)
   Each character is 5 bytes wide, 1 pixel gap between chars.
   ═══════════════════════════════════════════════════════ */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 32   */
    {0x00,0x00,0x5F,0x00,0x00}, /* 33 ! */
    {0x00,0x07,0x00,0x07,0x00}, /* 34 " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 35 # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 36 $ */
    {0x23,0x13,0x08,0x64,0x62}, /* 37 % */
    {0x36,0x49,0x55,0x22,0x50}, /* 38 & */
    {0x00,0x05,0x03,0x00,0x00}, /* 39 ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* 40 ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* 41 ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* 42 * */
    {0x08,0x08,0x3E,0x08,0x08}, /* 43 + */
    {0x00,0x50,0x30,0x00,0x00}, /* 44 , */
    {0x08,0x08,0x08,0x08,0x08}, /* 45 - */
    {0x00,0x60,0x60,0x00,0x00}, /* 46 . */
    {0x20,0x10,0x08,0x04,0x02}, /* 47 / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 48 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 49 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 50 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 51 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 52 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 53 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 54 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 55 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 56 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 57 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* 58 : */
    {0x00,0x56,0x36,0x00,0x00}, /* 59 ; */
    {0x00,0x08,0x14,0x22,0x41}, /* 60 < */
    {0x14,0x14,0x14,0x14,0x14}, /* 61 = */
    {0x41,0x22,0x14,0x08,0x00}, /* 62 > */
    {0x02,0x01,0x51,0x09,0x06}, /* 63 ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* 64 @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 65 A */
    {0x7F,0x49,0x49,0x49,0x36}, /* 66 B */
    {0x3E,0x41,0x41,0x41,0x22}, /* 67 C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 68 D */
    {0x7F,0x49,0x49,0x49,0x41}, /* 69 E */
    {0x7F,0x09,0x09,0x01,0x01}, /* 70 F */
    {0x3E,0x41,0x41,0x51,0x32}, /* 71 G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 72 H */
    {0x00,0x41,0x7F,0x41,0x00}, /* 73 I */
    {0x20,0x40,0x41,0x3F,0x01}, /* 74 J */
    {0x7F,0x08,0x14,0x22,0x41}, /* 75 K */
    {0x7F,0x40,0x40,0x40,0x40}, /* 76 L */
    {0x7F,0x02,0x04,0x02,0x7F}, /* 77 M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 78 N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 79 O */
    {0x7F,0x09,0x09,0x09,0x06}, /* 80 P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 81 Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* 82 R */
    {0x46,0x49,0x49,0x49,0x31}, /* 83 S */
    {0x01,0x01,0x7F,0x01,0x01}, /* 84 T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 85 U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 86 V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* 87 W */
    {0x63,0x14,0x08,0x14,0x63}, /* 88 X */
    {0x03,0x04,0x78,0x04,0x03}, /* 89 Y */
    {0x61,0x51,0x49,0x45,0x43}, /* 90 Z */
    {0x00,0x00,0x7F,0x41,0x41}, /* 91 [ */
    {0x02,0x04,0x08,0x10,0x20}, /* 92 \ */
    {0x41,0x41,0x7F,0x00,0x00}, /* 93 ] */
    {0x04,0x02,0x01,0x02,0x04}, /* 94 ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* 95 _ */
    {0x00,0x01,0x02,0x04,0x00}, /* 96 ` */
    {0x20,0x54,0x54,0x54,0x78}, /* 97 a */
    {0x7F,0x48,0x44,0x44,0x38}, /* 98 b */
    {0x38,0x44,0x44,0x44,0x20}, /* 99 c */
    {0x38,0x44,0x44,0x48,0x7F}, /*100 d */
    {0x38,0x54,0x54,0x54,0x18}, /*101 e */
    {0x08,0x7E,0x09,0x01,0x02}, /*102 f */
    {0x08,0x14,0x54,0x54,0x3C}, /*103 g */
    {0x7F,0x08,0x04,0x04,0x78}, /*104 h */
    {0x00,0x44,0x7D,0x40,0x00}, /*105 i */
    {0x20,0x40,0x44,0x3D,0x00}, /*106 j */
    {0x00,0x7F,0x10,0x28,0x44}, /*107 k */
    {0x00,0x41,0x7F,0x40,0x00}, /*108 l */
    {0x7C,0x04,0x18,0x04,0x78}, /*109 m */
    {0x7C,0x08,0x04,0x04,0x78}, /*110 n */
    {0x38,0x44,0x44,0x44,0x38}, /*111 o */
    {0x7C,0x14,0x14,0x14,0x08}, /*112 p */
    {0x08,0x14,0x14,0x18,0x7C}, /*113 q */
    {0x7C,0x08,0x04,0x04,0x08}, /*114 r */
    {0x48,0x54,0x54,0x54,0x20}, /*115 s */
    {0x04,0x3F,0x44,0x40,0x20}, /*116 t */
    {0x3C,0x40,0x40,0x20,0x7C}, /*117 u */
    {0x1C,0x20,0x40,0x20,0x1C}, /*118 v */
    {0x3C,0x40,0x30,0x40,0x3C}, /*119 w */
    {0x44,0x28,0x10,0x28,0x44}, /*120 x */
    {0x0C,0x50,0x50,0x50,0x3C}, /*121 y */
    {0x44,0x64,0x54,0x4C,0x44}, /*122 z */
    {0x00,0x08,0x36,0x41,0x00}, /*123 { */
    {0x00,0x00,0x7F,0x00,0x00}, /*124 | */
    {0x00,0x41,0x36,0x08,0x00}, /*125 } */
    {0x08,0x08,0x2A,0x1C,0x08}, /*126 ~ */
};

/* ═══════════════════════════════════════════════════════
   SSD1306 low-level I2C helpers
   ═══════════════════════════════════════════════════════ */

static esp_err_t ssd1306_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd };   /* Co=0, D/C#=0 → command */
    return i2c_master_transmit(s_dev, buf, 2, -1);
}

static esp_err_t ssd1306_data(const uint8_t *data, size_t len)
{
    /* Allocate buffer: 1 control byte + pixel data */
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x40;                    /* Co=0, D/C#=1 → data */
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_transmit(s_dev, buf, len + 1, -1);
    free(buf);
    return ret;
}

/* ═══════════════════════════════════════════════════════
   SSD1306 init sequence (128×64)
   ═══════════════════════════════════════════════════════ */
static void ssd1306_init_panel(void)
{
    ssd1306_cmd(0xAE);         /* display off */
    ssd1306_cmd(0xD5); ssd1306_cmd(0x80);  /* clock divide */
    ssd1306_cmd(0xA8); ssd1306_cmd(0x3F);  /* multiplex 64 */
    ssd1306_cmd(0xD3); ssd1306_cmd(0x00);  /* display offset 0 */
    ssd1306_cmd(0x40);         /* start line 0 */
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14);  /* charge pump ON */
    ssd1306_cmd(0x20); ssd1306_cmd(0x00);  /* horizontal addressing */
    ssd1306_cmd(0xA1);         /* segment remap (flip H) */
    ssd1306_cmd(0xC8);         /* COM scan descending (flip V) */
    ssd1306_cmd(0xDA); ssd1306_cmd(0x12);  /* COM pins config */
    ssd1306_cmd(0x81); ssd1306_cmd(0xCF);  /* contrast */
    ssd1306_cmd(0xD9); ssd1306_cmd(0xF1);  /* pre-charge period */
    ssd1306_cmd(0xDB); ssd1306_cmd(0x40);  /* VCOMH deselect */
    ssd1306_cmd(0xA4);         /* display from RAM */
    ssd1306_cmd(0xA6);         /* normal (not inverted) */
    ssd1306_cmd(0xAF);         /* display on */

    ESP_LOGI(TAG, "SSD1306 initialised (128x64)");
}

/* ═══════════════════════════════════════════════════════
   Framebuffer operations
   ═══════════════════════════════════════════════════════ */

static void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void fb_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    uint16_t idx = x + (y / 8) * SSD1306_WIDTH;
    if (on)
        s_fb[idx] |=  (1 << (y & 7));
    else
        s_fb[idx] &= ~(1 << (y & 7));
}

/* Draw a character at pixel position (x,y) — top-left corner.
   Returns width consumed (6 = 5px char + 1px gap). */
static int fb_char(int x, int y, char c)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            fb_pixel(x + col, y + row, (line >> row) & 1);
        }
    }
    /* 1 pixel gap */
    for (int row = 0; row < 7; row++)
        fb_pixel(x + 5, y + row, false);
    return 6;
}

/* Draw a string. Returns final x position. */
static int fb_string(int x, int y, const char *str)
{
    while (*str) {
        x += fb_char(x, y, *str++);
    }
    return x;
}

/* Draw a horizontal line */
static void fb_hline(int x0, int x1, int y)
{
    for (int x = x0; x <= x1; x++) fb_pixel(x, y, true);
}

/* Flush framebuffer to SSD1306 */
static void fb_flush(void)
{
    /* Set column and page address to cover full screen */
    ssd1306_cmd(0x21); ssd1306_cmd(0); ssd1306_cmd(127); /* col 0–127 */
    ssd1306_cmd(0x22); ssd1306_cmd(0); ssd1306_cmd(7);   /* page 0–7  */
    ssd1306_data(s_fb, sizeof(s_fb));
}

/* ═══════════════════════════════════════════════════════
   Get current WiFi IP as a string
   ═══════════════════════════════════════════════════════ */
static bool get_ip_string(char *buf, size_t len)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) { snprintf(buf, len, "No WiFi"); return false; }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK ||
        ip_info.ip.addr == 0) {
        snprintf(buf, len, "Connecting...");
        return false;
    }

    snprintf(buf, len, IPSTR, IP2STR(&ip_info.ip));
    return true;
}

/* ═══════════════════════════════════════════════════════
   Display update task
   ═══════════════════════════════════════════════════════

   Layout (128×64):
   ┌──────────────────────────┐
   │  ARGUS                   │  row 0  (title)
   │──────────────────────────│  row 10 (divider)
   │  IP: 10.144.113.73       │  row 14
   │                          │
   │  Motor L: +0.65          │  row 28
   │  Motor R: +0.65          │  row 38
   │                          │
   │  IMU  ax:+0.01  gx:-0.2  │  row 52
   └──────────────────────────┘
   ═══════════════════════════════════════════════════════ */
static void lcd_task(void *pv)
{
    char buf[32];
    motor_cmd_t cmd;

    for (;;) {
        fb_clear();

        /* ── Title bar ── */
        fb_string(0, 0, "ARGUS");
        fb_hline(0, 127, 10);

        /* ── IP address ── */
        char ip[20];
        bool connected = get_ip_string(ip, sizeof(ip));
        fb_string(0, 14, "IP:");
        fb_string(20, 14, ip);

        /* Connection dot */
        if (connected) {
            /* small filled circle at right side */
            fb_pixel(122, 14, true); fb_pixel(123, 14, true);
            fb_pixel(121, 15, true); fb_pixel(122, 15, true);
            fb_pixel(123, 15, true); fb_pixel(124, 15, true);
            fb_pixel(122, 16, true); fb_pixel(123, 16, true);
        }

        /* ── Motor values ── */
        command_get(&cmd);
        snprintf(buf, sizeof(buf), "Motor L: %+.2f", cmd.left);
        fb_string(0, 28, buf);
        snprintf(buf, sizeof(buf), "Motor R: %+.2f", cmd.right);
        fb_string(0, 38, buf);

        /* ── Divider ── */
        fb_hline(0, 127, 49);

        /* ── IMU summary (compact) ── */
        fb_string(0, 52, "IMU");

        /* Flush to display */
        fb_flush();

        vTaskDelay(pdMS_TO_TICKS(LCD_UPDATE_MS));
    }
}

/* ═══════════════════════════════════════════════════════
   Public API
   ═══════════════════════════════════════════════════════ */
esp_err_t lcd_init(void)
{
    /* ── Create I2C bus on I2C_NUM_1 ── */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = LCD_I2C_PORT,
        .sda_io_num = LCD_SDA_GPIO,
        .scl_io_num = LCD_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── Add SSD1306 device ── */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SSD1306_ADDR,
        .scl_speed_hz    = LCD_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── Init display ── */
    ssd1306_init_panel();
    fb_clear();
    fb_string(20, 28, "ARGUS  boot...");
    fb_flush();

    /* ── Start display task ── */
    xTaskCreate(lcd_task, "lcd_task", LCD_TASK_STACK, NULL,
                LCD_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "OLED task started (update every %d ms)", LCD_UPDATE_MS);
    return ESP_OK;
}
