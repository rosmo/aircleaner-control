idf_component_register(
    SRCS
    "main.c" "rotary_encoder.c"
    INCLUDE_DIRS "."
    REQUIRES ieee802154 openthread nvs_flash 
)
