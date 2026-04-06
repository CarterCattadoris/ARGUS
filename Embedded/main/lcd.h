#ifndef LCD_H
#define LCD_H

#include "esp_err.h"

/* ── I2C configuration (separate bus from IMU) ──────── */
#define LCD_SDA_GPIO        40
#define LCD_SCL_GPIO        41
#define LCD_I2C_PORT        I2C_NUM_1
#define LCD_I2C_FREQ_HZ     400000      /* 400 kHz fast-mode */

/* ── SSD1306 I2C address ───────────────────────────── */
#define SSD1306_ADDR        0x3C

/* ── Display dimensions ────────────────────────────── */
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_PAGES       (SSD1306_HEIGHT / 8)

/* ── Task parameters ───────────────────────────────── */
#define LCD_TASK_STACK      4096
#define LCD_TASK_PRIORITY   3
#define LCD_UPDATE_MS       250         /* refresh every 250 ms */

/**
 * Initialise the I2C bus and SSD1306 OLED, then start the
 * display update task.  Call once from app_main() after
 * wifi_init_sta() so the IP is available.
 */
esp_err_t lcd_init(void);

#endif /* LCD_H */
