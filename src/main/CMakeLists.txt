idf_component_register(SRCS "ethernet_example_main.c"
                       PRIV_REQUIRES esp_netif esp_eth ethernet_init esp_driver_pcnt esp_driver_ledc esp_timer esp_driver_gptimer
                       INCLUDE_DIRS ".")