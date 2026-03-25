idf_component_register(SRCS "main.c" "wifi.c" "udp.c" "command.c" 
PRIV_REQUIRES esp_wifi nvs_flash esp_timer 
INCLUDE_DIRS ".")