#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp32/ulp.h"

#include "soc/soc.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#define ULP_RESERVE_MEM CONFIG_ULP_COPROC_RESERVE_MEM

#define ULP_BINARY_MAGIC_ESP32 (0x00706c75)

static const char* TAG = "ulp";

typedef struct {
    uint32_t magic;
    uint16_t text_offset;
    uint16_t text_size;
    uint16_t data_size;
    uint16_t bss_size;
} ulp_binary_header_t;

esp_err_t ulp_load_binary_ex(uint32_t load_addr, const uint8_t* program_binary, size_t program_size)
{
    size_t program_size_bytes = program_size * sizeof(uint32_t);
    size_t load_addr_bytes = load_addr * sizeof(uint32_t);

    if (program_size_bytes < sizeof(ulp_binary_header_t)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (load_addr_bytes > ULP_RESERVE_MEM) {
        return ESP_ERR_INVALID_ARG;
    }
    if (load_addr_bytes + program_size_bytes > ULP_RESERVE_MEM) {
        printf("addr bytes %d program bytes %d ULP RESERVE %d\n",load_addr_bytes,program_size_bytes,ULP_RESERVE_MEM);
        return ESP_ERR_INVALID_SIZE;
    }

    // Make a copy of a header in case program_binary isn't aligned
    ulp_binary_header_t header;
    memcpy(&header, program_binary, sizeof(header));

    if (header.magic != ULP_BINARY_MAGIC_ESP32) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    size_t total_size = (size_t) header.text_offset + (size_t) header.text_size +
            (size_t) header.data_size;

    ESP_LOGD(TAG, "program_size_bytes: %d total_size: %d offset: %d .text: %d, .data: %d, .bss: %d",
            program_size_bytes, total_size, header.text_offset,
            header.text_size, header.data_size, header.bss_size);

    if (total_size != program_size_bytes) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t text_data_size = header.text_size + header.data_size;
    uint8_t* base = (uint8_t*) RTC_SLOW_MEM;

    memcpy(base + load_addr_bytes, program_binary + header.text_offset, text_data_size);
    memset(base + load_addr_bytes + text_data_size, 0, header.bss_size);

    return ESP_OK;
}
