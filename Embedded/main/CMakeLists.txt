idf_component_register(SRCS "main.c" "wifi.c" "udp.c" "command.c" "motor.c" "imu.c"
PRIV_REQUIRES esp_wifi nvs_flash esp_timer  esp_driver_ledc esp_driver_gpio esp_driver_i2c
INCLUDE_DIRS ".")