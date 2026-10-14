idf_component_register(
    SRCS 
    "wifi_ap_manager.c"

    INCLUDE_DIRS 
    "include"
    
    REQUIRES 
    esp_event 
    esp_netif 
    esp_wifi 
    esp_timer
)
