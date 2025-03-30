#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nvs_storage.h"

#define NVS_NAMESPACE "storage"  // Пространство имен NVS
#define NVS_KEY "bool_value"     // Ключ для хранения значения

static const char *TAG = "NVS_STORAGE";

// Функция инициализации NVS
void nvs_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS инициализирован");
}

// Функция сохранения bool значения в NVS
void save_bool_to_nvs(bool value) {
    nvs_handle_t handle;
    esp_err_t ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(ret));
        return;
    }

    uint8_t val = (uint8_t)value;
    ret = nvs_set_u8(handle, NVS_KEY, val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка записи значения: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Сохранено значение: %d", val);
    }

    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка подтверждения записи: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
}

// Функция чтения bool значения из NVS
bool read_bool_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t ret;
    uint8_t value = 0;  // По умолчанию false

    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS для чтения: %s", esp_err_to_name(ret));
        return false;
    }

    ret = nvs_get_u8(handle, NVS_KEY, &value);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Прочитано значение: %d", value);
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Значение не найдено, возвращаем false");
        value = 0;
    } else {
        ESP_LOGE(TAG, "Ошибка чтения: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
    return (bool)value;
}