/* The board's microSD card, mounted at /sdcard */
#pragma once

#include "esp_err.h"

esp_err_t card_mount(void);
