idf_component_register(
    SRCS "slcan_bridge.c" "slcan.c"
    INCLUDE_DIRS "."
    REQUIRES driver tinyusb
)

