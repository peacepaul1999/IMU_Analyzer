idf_component_register(
    SRCS 
    "MPU9250.c"

    INCLUDE_DIRS 
    "include"
    
    REQUIRES 
    MY_I2C 
    driver 
    nvs_flash 
    esp_timer
)
