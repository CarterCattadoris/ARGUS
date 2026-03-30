idf_component_register(SRCS "main.c" "wifi.c" "udp.c" "command.c" "motor.c"
PRIV_REQUIRES esp_wifi nvs_flash esp_timer  esp_driver_ledc esp_driver_gpio
INCLUDE_DIRS ".")