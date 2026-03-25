idf_component_register(SRCS "main.c" "wifi.c" "udp.c" 
PRIV_REQUIRES esp_wifi nvs_flash 
INCLUDE_DIRS ".")