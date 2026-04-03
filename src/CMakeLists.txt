include(FetchContent)
FetchContent_Declare(
    etl
    GIT_REPOSITORY https://github.com/ETLCPP/etl
    GIT_TAG        20.46.2
)
FetchContent_MakeAvailable(etl)

add_executable(rpauto
    main.cpp
    usb_descriptors.cpp
    usb_gs_can.cpp
    mcp2518fd.cpp
)

# Link required libraries
target_link_libraries(rpauto 
    pico_stdlib 
    hardware_adc 
    hardware_i2c 
    hardware_pwm 
    hardware_spi 
    hardware_uart
    hardware_gpio
    tinyusb_device 
    tinyusb_board
    etl
)

# Include directory for our headers (tusb_config.h etc.)
target_include_directories(rpauto PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Disable stdio over USB since we're using USB for gs_usb vendor class
pico_enable_stdio_usb(rpauto 0)
pico_enable_stdio_uart(rpauto 1)

# Create UF2 file for flashing
pico_add_extra_outputs(rpauto)
