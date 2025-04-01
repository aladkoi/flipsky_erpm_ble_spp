#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nvs_storage.h"
#include "controller.h"

#define NVS_NAMESPACE "storage"  // Пространство имен NVS
#define NVS_KEY "limit_value"     // Ключ для хранения значения
#define AUTO_SPEED "autospeed_value"     // Ключ для хранения значения
#define START_SPEED "start_speed"
#define AUTO_START "auto_start"
#define SMART_BREAK "smart_break"  /// умный тормоз

static const char *TAG = "NVS_STORAGE";

// Функция инициализации NVS
void nvs_init(void) {
    nvs_handle_t handle;
    printf("nvs_init\n");
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_OK) {
        // Пространство имен существует, проверяем наличие ключа
        uint8_t test_value;
        ret = nvs_get_u8(handle, NVS_KEY, &test_value);
        if (ret == ESP_OK || ret == ESP_ERR_NVS_NOT_FOUND) {
            // Пространство имен уже инициализировано, ключ либо есть, либо его нет — всё в порядке
            printf("NVS уже инициализирован\n");
            printf("state.limit_speed=%d\n",test_value);
            nvs_close(handle);
            return;
        }
        nvs_close(handle);
    } else if (ret == ESP_ERR_NVS_NOT_INITIALIZED || ret == ESP_ERR_NVS_PART_NOT_FOUND) {
        // NVS не инициализирован, выполняем полную инициализацию
        printf("NVS не инициализирован, выполняем инициализацию\n");
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            printf("error: требуется очистка NVS\n");
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
    } 
}

// Функция сохранения bool значения в NVS
void save_to_nvs(uint8_t data) {
    nvs_handle_t handle;
    esp_err_t ret;
    //uint8_t value = 0; 
    uint8_t val;
    bool bit0;
    bool bit1;
    bool bit2;
    //bool bit3;    
    printf("save_to_nvs=%d\n",data);

    bit0 = (data & (1 << 0)) != 0;  // 0-й бит (младший)
    bit1 = (data & (1 << 1)) != 0;  // 1-й бит
    bit2 = (data & (1 << 2)) != 0;  // 2-й бит
    uint8_t start_sp = (data >> 4) & 0x0F; 
    printf("start_sp=%d\n",start_sp);
    //bit3 = (data & (1 << 2)) != 0;  // 3-й бит

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS0: %s", esp_err_to_name(ret));
        return;
    }


    if (bit1==1){
        if (bit0==1 && bit1==1){   //= 3
            state.limit_speed=true;
            len_crouise=3;                        
          }
        else{ // =2
            state.limit_speed=false;
            len_crouise=5;
        }  
        printf("len_crouise=%d\n", len_crouise);        
        val=(uint8_t)state.limit_speed;
        nvs_set_u8(handle, NVS_KEY, val);
    }

    if (bit2==1){
        state.auto_speed=true;
     }
        else{
            state.auto_speed=false;
        }
        val=(uint8_t)state.auto_speed;
        nvs_set_u8(handle, AUTO_SPEED, val);
    
    nvs_set_u8(handle, START_SPEED, start_sp); 
    state.start_level=start_sp;
        
    ret = nvs_commit(handle);
    nvs_close(handle);
    printf("state.limit_speed=%s\n", state.limit_speed ? "true" : "false");
    printf("state.auto_speed=%s\n", state.auto_speed ? "true" : "false");
}

// Функция сохранения bool значения в NVS
void save_to_nvs1(uint8_t data) {
    nvs_handle_t handle;
    esp_err_t ret;
    bool bit3;
    //uint8_t value = 0; 
    printf("save_to_nvs1=%d\n",data);

    bit3 = (data & (1 << 3)) != 0;  // 3-й бит
    uint8_t start_sp = (data >> 4) & 0x0F; 
    
    printf("aut_start_sp=%d\n",start_sp);
    printf("smart_break=%d\n",bit3);

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS0: %s", esp_err_to_name(ret));
        return;
    }

    nvs_set_u8(handle, AUTO_START, start_sp); 
    state.auto_start_level=start_sp;        
    
    if (bit3==1)state.smart_brake=true;
    else state.smart_brake=false; 
    uint8_t val=(uint8_t)state.smart_brake;
    nvs_set_u8(handle, SMART_BREAK, val);
    ret = nvs_commit(handle);
    nvs_close(handle);
}



// Функция чтения bool значения из NVS
void read_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t ret;
    uint8_t value = 0;  // По умолчанию false
    printf("read_from_nvs\n");
    ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS для чтения: %s", esp_err_to_name(ret));
        return;
    }
    nvs_get_u8(handle, NVS_KEY, &value);
    state.limit_speed=(value != 0);
    nvs_get_u8(handle, AUTO_SPEED, &value);
    state.auto_speed=(value != 0);
    nvs_get_u8(handle, START_SPEED, &value);
    state.start_level=value;
    nvs_get_u8(handle,  AUTO_START, &value);
    state.auto_start_level=value;
    nvs_get_u8(handle, SMART_BREAK, &value);
    state.smart_brake=(value != 0);

    printf("state.limit_speed=%s\n", state.limit_speed ? "true" : "false");
    printf("state.auto_speed=%s\n", state.auto_speed ? "true" : "false");
    printf("state.smart_brake=%s\n", state.smart_brake ? "true" : "false");
    printf("state.start_level=%d\n", state.start_level);
    printf("state.auto_start_level=%d\n", state.auto_start_level);
    nvs_close(handle);

}