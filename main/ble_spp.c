#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "nvs_flash.h"
#include <string.h>
#include <controller.h>




#define SPP_SERVER_NAME "BLE_ESP32_FLIPSKY85"
//#define SPP_SERVER_NAME "FTESC BLE"
#define TAG_BL "SPP"
TaskHandle_t send_task_handle = NULL; 

static uint32_t spp_handle = 0; // Дескриптор подключения SPP
// Мьютекс для синхронизации доступа к состоянию


//#define SEND_BUFFER_SIZE (sizeof(send_buffer) / sizeof(send_buffer[0])) 

// Функция для извлечения битов 4-7 из байта
static inline uint8_t get_bits_4_to_7(uint8_t data) {
    return (data >> 4) & 0x0F;  // Сдвиг вправо на 4 и маска 00001111
}


// Функция вычисления Adler-16 с учетом только битов 4-7
uint8_t* calculate_adler16(uint8_t* data, size_t length) {
    uint16_t sum1 = 0x1A; // Начальное значение для sum1
    uint16_t sum2 = 0x2B; // Начальное значение для sum2
    const uint16_t MOD_ADLER = 251; // Простое число для модуля

    // Проходим по массиву данных
    for (size_t i = 0; i < length; i++) {
        uint8_t bits_4_7 = get_bits_4_to_7(data[i]);  // Извлекаем биты 4-7
        sum1 = (sum1 + bits_4_7) % MOD_ADLER;         // Сумма битов 4-7
        sum2 = (sum2 + (sum1 ^ (bits_4_7 << 1))) % MOD_ADLER; // Усложнённая сумма
    }

    // Комбинируем результаты с побитовыми операциями
    uint16_t checksum16 = (sum2 << 8) | sum1;
    checksum16 = checksum16 ^ 0x55AA; // XOR с константой
    checksum16 = checksum16 & 0xFFFF; // Ограничиваем до 16 бит

    // Извлекаем младший и старший байты
    uint8_t low_byte = checksum16 & 0xFF;
    uint8_t high_byte = (checksum16 >> 8) & 0xFF;

    // Выделяем память для результата (2 байта)
    uint8_t* checksum = (uint8_t*)malloc(2 * sizeof(uint8_t));
    if (checksum == NULL) {
        ESP_LOGE(TAG_BL, "Ошибка выделения памяти для checksum");
        return NULL;
    }
    checksum[0] = low_byte;  // Младший байт
    checksum[1] = high_byte; // Старший байт

    return checksum;
}

    // uint16_t crc16arc_bit(uint8_t *data, size_t len) {
    //     uint16_t crc = 0;
    //     for (size_t i = 0; i < len; i++) {
    //         crc ^= data[i];
    //         for (unsigned k = 0; k < 8; k++) {
    //             crc = crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1;
    //         }
    //     }

    //     // Обработка второго байта
    //     uint8_t *bytePtr = (uint8_t *)&crc;
    //     uint8_t lastByte = bytePtr[0];
    //     int result = lastByte + 96;
    //     if (result > 127) {
    //         if (result & (1 << 7) && result & (1 << 5)) {
    //             result &= ~(1 << 7); // Если установлены 7 и 5 биты, сбрасываем 7
    //         } else if (result & (1 << 7) && result & (1 << 6)) {
    //             result &= ~(1 << 7); // Если установлены 7 и 6 биты, сбрасываем оба
    //             result &= ~(1 << 6);
    //         } else {
    //             result &= ~(1 << 7);
    //             result |= (1 << 6);
    //         }
    //     }
    //     bytePtr[0] = (uint8_t)result;
    //     return crc;
    // }



// Задача для отправки данных
static void send_task(void *pvParameters) {
    static uint8_t send_buffer[9] = {0xAA,0x06,0x00,0x00, 0x00, 0x00, 0x00,0x00,0xDD};   
    while (1) {
        if (spp_handle != 0) { // Проверяем, подключён ли клиент
            if (strcmp(state.name_cont, "FT85BS") != 0){
                send_buffer[2]=0; /// контроллер не подключен
            }    
            else {
                send_buffer[2]=1; 
                send_buffer[3]= (uint8_t)(state.Vbatt >> 8); //бат
                send_buffer[4]=(uint8_t)(state.Vbatt  & 0xFF); //бат
                send_buffer[5]=(uint8_t)(state.rpm_controller >> 8); //erpm
                send_buffer[6]=(uint8_t)(state.rpm_controller & 0xFF); //erpm
                //if (state.controllerBrake)send_buffer[11]=1;  
                //else send_buffer[11]=0;
                //send_buffer[11]=(uint8_t)(state.rpm_controller & 0xFF); //erpm
                send_buffer[7]=state.change_event;
            }
            //uint8_t data_length = send_buffer[5];
            //uint8_t payload[data_length];
            //memcpy(payload, &send_buffer[6], data_length);
            //ESP_LOGI(TAG_BL,"payload=====");
            //ESP_LOG_BUFFER_HEX(TAG_BL, payload, data_length);
            //uint8_t* crc2 =calculate_adler16(payload,data_length);
            //send_buffer[11]=crc2[0] << 8;
            //send_buffer[12]=crc2[1];
            //static uint8_t send_buffer[13] = {0x08,0x00, 0x00,0x00, 0xAA,0x05,0x00,0x00, 0x00, 0x00, 0x00, 0xDD};
            //ESP_LOG_BUFFER_HEX(TAG_BL, (uint8_t *)send_buffer, 13);

            esp_err_t ret = esp_spp_write(spp_handle, 9, (uint8_t *)send_buffer);
            if (state.change_event>0)state.change_event=0;
            //esp_err_t ret = esp_spp_write(spp_handle, sizeof(msg) - 1, (uint8_t *)msg);
            if (ret == ESP_OK) {
                //ESP_LOGI(TAG_BL, "Sent");
            } else {
                //ESP_LOGE(TAG_BL, "Failed to send data");
            }
        } else {
            //ESP_LOGI(TAG_BL, "No client connected, skipping send");
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Отправка каждые 2 секунды
    }
}

// // Обработчик событий SPP
// static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
//     static uint8_t buffer[512];
//     switch (event) {
//         case ESP_SPP_INIT_EVT:
//             //ESP_LOGI(TAG_BL, "SPP initialized");
//             esp_bt_gap_set_device_name(SPP_SERVER_NAME);
//             esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
//             esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
//             break;

//         case ESP_SPP_SRV_OPEN_EVT:
//             //ESP_LOGI(TAG_BL, "Client connected, handle: %lu", param->srv_open.handle);
//             break;

//             case ESP_SPP_DATA_IND_EVT:
//             //ESP_LOGI(TAG_BL, "Received data, length: %d", param->data_ind.len);
//             //ESP_LOG_BUFFER_HEX(TAG_BL, param->data_ind.data, param->data_ind.len);
        
//             // Проверяем, что длина пакета >= 4 байта (размер заголовка)
//             if (param->data_ind.len >= 4) {
//                 // Читаем первые 4 байта как длину (little-endian)
//                 uint32_t data_length = (param->data_ind.data[3] << 24) |
//                                        (param->data_ind.data[2] << 16) |
//                                        (param->data_ind.data[1] << 8)  |
//                                        param->data_ind.data[0];
//                 //ESP_LOGI(TAG_BL, "Parsed data length: %lu", data_length);
        
//                 // Проверяем, что данные после заголовка есть
//                 if (param->data_ind.len > 4) {
//                     uint32_t payload_length = param->data_ind.len - 4; // Длина полезных данных

//                     if (payload_length >= data_length) {
//                         payload_length = data_length; // Ограничиваем длиной из заголовка
//                     }
//                     if (payload_length <= sizeof(buffer) - 4) { // Проверяем, помещается ли в buffer
//                         memcpy(buffer, param->data_ind.data + 4, payload_length); 
//                         if (param->data_ind.len >= 5) {
//                             //ESP_LOGI(TAG_BL, "payload_length: %lu", payload_length);
//                             if (buffer[0] == 0xAA && buffer[payload_length - 1] == 0xDD) {
//                                 uint8_t data_length = buffer[1];
//                                     uint8_t payload[data_length];
//                                     memcpy(payload, &buffer[2], data_length);
//                                     //ESP_LOG_BUFFER_HEX(TAG_BL, payload, data_length);
//                                     uint8_t crc_high = buffer[data_length + 2]; // Старший байт CRC
//                                     uint8_t crc_low = buffer[data_length + 3];  // Младший байт CRC
//                                     uint16_t crc1 = (crc_high << 8) | crc_low;   // Собираем CRC
//                                     //ESP_LOGI(TAG_BL, "Extracted CRC1: 0x%04x", crc1); 
//                                     uint8_t* crc2 =calculate_adler16(payload, data_length);
//                                     uint16_t crc3 = (crc2[0] << 8) | crc2[1];
//                                     //ESP_LOGI(TAG_BL, "Extracted CRC2: 0x%04x", crc3); 
//                                     if (crc1==crc3){  /// данные корректны и можно обрабатывать
//                                         //ESP_LOGI(TAG_BL, "success"); 
//                                         if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
//                                         // 0 speed, 1 - номер круиза+1, 2 - тормоз, 3 (1=плюс скорость,2=плюс круиз),4 (1=минус скорость,2=минус круиз)  
//                                         if (payload[0]==10) { // запрос на подключение к контроллеру
//                                             //ESP_LOGI(TAG_BL, "Get task created");            
//                                         spp_handle = param->srv_open.handle;
//                                         if (send_task_handle == NULL) {
//                                             //ESP_LOGI(TAG_BL, "Send task created");
//                                             xTaskCreate(send_task, "send_task", 2048, NULL, 5, &send_task_handle);
                                            
//                                         }  


//                                              }
//                                         else if (payload[2]==1) { /// break
//                                             //ESP_LOGI(TAG_BL, "BREAK"); 
//                                             state.break_level = 0;
//                                             state.cr=-1;
//                                             state.volt_bl = 0;
//                                             stop_Speed(false);
//                                             state.break_long = true;
//                                             state.current_amper = 3;
//                                             state.current_level=0;
//                                             DoLight(128);
//                                             vTaskDelay(pdMS_TO_TICKS(1000));
//                                             state.break_level = 1;
//                                             DoLight(0);    
//                                             }        
//                                         else if (payload[1]>0) {
//                                             state.crouise_on = true;
//                                             state.speed_up = true;
//                                             state.croiuse_level=payload[1]-1;
//                                             //ESP_LOGI(TAG_BL, "CROUISE=%d",state.croiuse_level); 
//                                             setCrouise(state.croiuse_level);
//                                         }   
//                                         else if (payload[3]>0) {
//                                             //ESP_LOGI(TAG_BL, "add_but=%d",payload[3]); 
//                                             if (payload[3]==1)BUTTON_SINGLE_CLICK_ADD();
//                                             else if (payload[3]==2)BUTTON_PRESS_REPEAT_ADD();                                            
//                                         }   
//                                         else if (payload[4]>0) {
//                                             //ESP_LOGI(TAG_BL, "dec_but=%d",payload[4]); 
//                                             if (payload[4]==1)BUTTON_SINGLE_CLICK_DEC();
//                                             else if (payload[4]==2)BUTTON_PRESS_REPEAT_DEC();
//                                         }
          
//                                            xSemaphoreGive(state_mutex);
//                                         }
                                        

//                                     }    
//                                     //ESP_LOGI(TAG, "Checksum: %02x %02x", crc2[0], crc2[1]);
//                                     //ESP_LOGI(TAG, "Checksum (16-bit): %04x", (crc2[1] << 8) | crc2[0]);


//                             }    

//                         }


//                         //ESP_LOG_BUFFER_HEX(TAG_BL, buffer, payload_length);
//                     } 


//                     //ESP_LOGI(TAG, "Payload length: %lu", payload_length);
//                     // Приводим payload_length к int для использования с %.*s
//                     //ESP_LOGI(TAG, "Payload as string: %.*s", (int)payload_length, param->data_ind.data + 4);                    
//                     //if (param->data_ind.data + 4)=="0xaa")
//                     //ESP_LOG_BUFFER_HEX(TAG_BL, param->data_ind.data + 4, payload_length);
//                     //printf("==========================================\n");
//                 } else {
//                     //ESP_LOGW(TAG_BL, "No payload data (only header)");
//                 }
//             } else {
//                 //ESP_LOGE(TAG_BL, "Packet too short, expected at least 4 bytes");
//             }
//             break;

//         case ESP_SPP_CLOSE_EVT:
//             //ESP_LOGI(TAG_BL, "Client disconnected");
//             spp_handle = 0;
//             if (send_task_handle != NULL) {
//                 vTaskDelete(send_task_handle);
//                 send_task_handle = NULL;
//                 //ESP_LOGI(TAG_BL, "Send task deleted");
//             }
//             break;

//         default:
//             break;
//     }
// }

// Обработчик событий SPP
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    static uint8_t buffer[512];
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_bt_gap_set_device_name(SPP_SERVER_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            break;

        case ESP_SPP_DATA_IND_EVT:
            ESP_LOG_BUFFER_HEX(TAG_BL, param->data_ind.data, param->data_ind.len);

            // Копируем весь пакет в буфер (без смещения на 4 байта)
            uint32_t payload_length = param->data_ind.len; // Весь пакет — полезная нагрузка
            if (payload_length <= sizeof(buffer)) { // Проверяем, помещается ли в buffer
                memcpy(buffer, param->data_ind.data, payload_length);

                // Проверяем наличие маркеров 0xAA и 0xDD
                if (payload_length >= 5 && buffer[0] == 0xAA && buffer[payload_length - 1] == 0xDD) {
                    uint8_t data_length = buffer[1]; // Длина полезных данных из второго байта
                    if (payload_length >= data_length + 4) { // Проверяем, достаточно ли данных
                        uint8_t payload[data_length];
                        memcpy(payload, &buffer[2], data_length);

                        // Извлекаем CRC
                        uint8_t crc_high = buffer[data_length + 2]; // Старший байт CRC
                        uint8_t crc_low = buffer[data_length + 3];  // Младший байт CRC
                        uint16_t crc1 = (crc_high << 8) | crc_low;  // Собираем CRC

                        // Вычисляем CRC
                        uint8_t* crc2 = calculate_adler16(payload, data_length);
                        uint16_t crc3 = (crc2[0] << 8) | crc2[1];

                        if (crc1 == crc3) { // Данные корректны

                            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                                // Обработка команд
                                if (payload[0] == 255) { // Запрос на подключение
                                    spp_handle = param->srv_open.handle;
                                    if (send_task_handle == NULL) {
                                        xTaskCreate(send_task, "send_task", 2048, NULL, 5, &send_task_handle);
                                    }
                                }
                                else if (payload[0] == 128){   /// сохранение текущей скорости для круиза
                                    //state.diagnostic=!state.diagnostic; //переключить поток на диагностику
                                    savespeed_to_nvs(payload[1]);
                                }    
                                else if (payload[2]>1) { /// 2 - ограничение off 3- ограничение on
                                    save_to_nvs((uint8_t)payload[2]);
                                    save_to_nvs1((uint8_t)payload[1]);   
                                    // if (payload[2]==2){
                                    //     save_to_nvs(payload[2]);
                                    //     state.limit_speed=false;
                                    //     len_crouise=5;
                                    // }
                                    // else {
                                    //     save_bool_to_nvs(true); 
                                    //     state.limit_speed=true;                
                                    //     len_crouise=3;
                                    //      }
                                }
                                else if (payload[2] == 1) { // Тормоз
                                    state.break_level = 0;
                                    state.cr = -1;
                                    state.volt_bl = 0;
                                    stop_Speed(false);
                                    state.break_long = true;
                                    state.current_amper = 3;
                                    state.current_level = 0;
                                    DoLight(128);
                                    vTaskDelay(pdMS_TO_TICKS(1000));
                                    state.break_level = 1;
                                    DoLight(0);
                                }
                                else if (payload[1] > 0 && payload[1] < 10) { // Круиз
                                    state.crouise_on = true;
                                    state.speed_up = true;
                                    state.croiuse_level = payload[1] - 1;
                                    setCrouise(state.croiuse_level);
                                }
                                else if (payload[3] > 0) { // Плюс скорость/круиз
                                    if (payload[3] == 1) BUTTON_SINGLE_CLICK_ADD();
                                    else if (payload[3] == 2) BUTTON_PRESS_REPEAT_ADD();
                                    else if (payload[3] == 3) set_relay_state();
                                }
                                else if (payload[4] > 0) { // Минус скорость/круиз
                                    if (payload[4] == 1) BUTTON_SINGLE_CLICK_DEC();
                                    else if (payload[4] == 2) BUTTON_PRESS_REPEAT_DEC();
                                    else if (payload[4] == 3) pulse_relay();
                                }
                                xSemaphoreGive(state_mutex);
                            }
                            
                                   if (crc2 != NULL) {
                                          free(crc2);
                                          crc2 = NULL; // Обнуляем указатель после освобождения (хорошая практика)
                                           }
                        } else {
                            ESP_LOGE(TAG_BL, "CRC mismatch: received 0x%04x, calculated 0x%04x", crc1, crc3);
                        }
                    } else {
                        ESP_LOGE(TAG_BL, "Payload too short for data_length: %d, received: %lu", data_length, payload_length);
                    }
                } else {
                    ESP_LOGW(TAG_BL, "Invalid packet format (missing 0xAA or 0xDD)");
                }
            } else {
                ESP_LOGE(TAG_BL, "Packet too large for buffer: %lu bytes", payload_length);
            }
            break;

        case ESP_SPP_CLOSE_EVT:
            spp_handle = 0;
            if (send_task_handle != NULL) {
                vTaskDelete(send_task_handle);
                send_task_handle = NULL;
            }
            break;

        default:
            break;
    }
}


void start_ble() {
    // Инициализация NVS
    esp_log_level_set("*", ESP_LOG_WARN); // Только предупреждения и ошибки
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Инициализация Bluetooth-контроллера
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    // Инициализация Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Конфигурация SPP
    esp_spp_cfg_t spp_config = {
        .mode = ESP_SPP_MODE_CB,    // Callback-режим
        .enable_l2cap_ertm = false, // Не используем ERTM
        .tx_buffer_size = 2048,
    };

    // Регистрация коллбэка и запуск SPP
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_cb));
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_config));
    nvs_init();
    read_from_nvs();
    if (state.limit_speed)len_crouise=3;
    else len_crouise=5;
    rpm_crouise[0]=erpm_step[state.start_level];
    // Создание задачи для отправки данных
    

    //ESP_LOGI(TAG_BL, "Bluetooth SPP server started");
}