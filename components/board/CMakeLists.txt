# The board the emulator runs on (Kconfig.projbuild, TIE_BOARD): its display, audio output, SD
# card and input. input.c is common to the boards; each board has a directory of its own.
if(CONFIG_TIE_BOARD_WAVESHARE_ESP32P4_DEVKIT)
    set(board_dir waveshare_esp32p4_devkit)
    set(board_srcs board.c card.c usb_hid.c)
    set(board_requires fatfs sdmmc esp_driver_sdmmc esp_driver_gpio esp_driver_i2c esp_driver_i2s
                       esp_driver_ppa esp_driver_dma esp_lcd esp_hw_support esp_mm
                       esp_codec_dev esp_lcd_ili9881c cherryusb)
endif()
list(TRANSFORM board_srcs PREPEND ${board_dir}/)

idf_component_register(SRCS input.c ${board_srcs}
                       INCLUDE_DIRS include
                       PRIV_INCLUDE_DIRS . ${board_dir}
                       REQUIRES machine
                       PRIV_REQUIRES ${board_requires})
