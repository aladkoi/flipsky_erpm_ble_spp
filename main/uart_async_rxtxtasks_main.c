/* UART asynchronous example with state structure
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <math.h>
//#include "esp_intr_alloc.h"
//#include "esp_pm.h"
#include "iot_button.h"
#include "esp_sleep.h"
#include <stdio.h>
#include <inttypes.h>
#include "button_gpio.h"
//#include "crc.h"  // Подсчет по duty
//#include "led_break.h"
#include "ble_spp.c"
//#include "esp_vfs_dev.h"
#include "controller.h"
#define TAG "BUTTON"
#define BUTTON_NUM1 23 // Левая кнопка
#define BUTTON_NUM2 22 // Правая кнопка
#define BUTTON_ACTIVE_LEVEL 0
#define BLUE_LED_PIN GPIO_NUM_2
#define POWER_PIN 4
#define BREAK_PIN 19 // Вход стоп-сигнала левый
#define BREAK_RIGTH_PIN 36 /// Вход стоп-сигнала правый
#define UART_NUM UART_NUM_1
#define BUF_SIZE 2048
#define RX_TIMEOUT (1000 / portTICK_PERIOD_MS)

// #define UART0_PORT_NUM     UART_NUM_0    // UART0 (обычно для USB-to-Serial)
// #define UART0_TXD_PIN      GPIO_NUM_1    // TXD UART0
// #define UART0_RXD_PIN      GPIO_NUM_3    // RXD UART0
// #define UART_QUEUE_SIZE    20            // 

// #define UART_BAUD_RATE     115200        // Скорость 115200 бод


// #define TXD_PIN (GPIO_NUM_1)      // TXD0
// #define RXD_PIN (GPIO_NUM_3)      // RXD0
// #define UART_PORT UART_NUM_0   

// Очередь UART
static QueueHandle_t uart_queue;
//static QueueHandle_t uart0_queue;
//static const char *TAG_USB = "UART_RETRANSLATOR";
static bool logging=false;

static TaskHandle_t loop_controller_handle = NULL;
// static TaskHandle_t uart0_controller_handle = NULL;
static TaskHandle_t addspeed_controller_handle = NULL;
static TaskHandle_t uart_task_handle = NULL;
// static TaskHandle_t uart0_to_uart2_task_handle = NULL;

// Названия событий кнопок
static const char *button_event_names[] = {
    [BUTTON_PRESS_DOWN] = "BUTTON_PRESS_DOWN",
    [BUTTON_PRESS_UP] = "BUTTON_PRESS_UP",
    [BUTTON_PRESS_REPEAT] = "BUTTON_PRESS_REPEAT",
    [BUTTON_SINGLE_CLICK] = "BUTTON_SINGLE_CLICK",
    [BUTTON_DOUBLE_CLICK] = "BUTTON_DOUBLE_CLICK",
    [BUTTON_LONG_PRESS_START] = "BUTTON_LONG_PRESS_START",
    [BUTTON_LONG_PRESS_HOLD] = "BUTTON_LONG_PRESS_HOLD",
    [BUTTON_LONG_PRESS_UP] = "BUTTON_LONG_PRESS_UP",
};



// void uart_init(void) {

//     uart_config_t uart0_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .rx_flow_ctrl_thresh = 122,
//         .source_clk = UART_SCLK_DEFAULT, 
//     };

//     uart_param_config(UART0_PORT_NUM, &uart0_config);
//     uart_set_pin(UART0_PORT_NUM, UART0_TXD_PIN, UART0_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     uart_driver_install(UART0_PORT_NUM, BUF_SIZE, BUF_SIZE, UART_QUEUE_SIZE, &uart0_queue, 0);
//     ESP_ERROR_CHECK(uart_enable_rx_intr(UART0_PORT_NUM));
// }    


static void do_add_speed(void* parameter) {
    const uint32_t DELAY_MS = 450;
    //const uint32_t CHECK_INTERVAL_MS = 450;
    //TickType_t last_check_time;
    int last_index = 0;
    printf("start do_add_speed");
    while (1) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            if (state.rpm_controller > 0) {
                if (state.rpm_controller > state.volt_bl)
                    state.current_amper = getCurrentDecIndexSpeed(state.volt_bl);
                else
                    state.current_amper = getCurrentIndexSpeed(state.rpm_controller);
            } else {
                state.current_amper = state.auto_start_level;
            }

            bool local_addspeed = state.addspeed;
            if (local_addspeed){
                state.break_level=1;
                if (state.current_amper<state.auto_start_level)state.current_amper=state.auto_start_level;
                //printf("state.break_level333=%d",state.break_level=1); 
            }
            xSemaphoreGive(state_mutex);
            
            if (local_addspeed) {
                //last_check_time = xTaskGetTickCount();
                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    last_index = find_index_linear(state.volt_bl);
                    if (last_index < 0) last_index = 0;                    
                    state.target_erpm = erpm_step[state.current_amper];
                    printf("========state.target_erp1=%d\n",state.target_erpm);
                    xSemaphoreGive(state_mutex);
                }

                while (state.break_level == 1 && local_addspeed && state.current_amper <= last_index) {
                    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                        state.target_erpm = erpm_step[state.current_amper];
                        printf("++++++++state.target_erpm2=%d\n",state.target_erpm);
                        state.current_amper++;
                        local_addspeed = state.addspeed;
                        xSemaphoreGive(state_mutex);
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    state.addspeed = false;
                    local_addspeed = false;
                    xSemaphoreGive(state_mutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    vTaskDelete(NULL);
}



/// это передний тормоз
static void button_event_rigth_break(void *arg, void *data) {   
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        printf("button_event_rigth_break\n");
        state.break_level = 0;
        state.break_long = true;
        gpio_set_level(BLUE_LED_PIN, 1);
        DoLight(128);
        state.controllerBrake = false;
        state.addspeed = false;
        stop_Speed(true);
        setCurrentLevel();
        state.change_event=20;
        xSemaphoreGive(state_mutex);
    }
}



// Обработчики событий кнопок
/// это задний тормоз
static void button_event_break(void *arg, void *data) {
    printf("button_event_break\n");
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        //printf("do button_event_break\n");
        printf("state.volt_bl_break_11=%d\n",state.volt_bl);
        if (state.start_break_event){  /// защита от двойного срабатывания
            xSemaphoreGive(state_mutex);
            return;
        }
        state.break_level = 0;
        gpio_set_level(BLUE_LED_PIN, 1);
        DoLight(128);
        state.controllerBrake = false;
        //state.break_volt_bl = state.volt_bl;
        //printf("state.break_volt_bl1=%d\n",state.break_volt_bl);       
        //state.break_croiuse_level = state.croiuse_level;
        state.addspeed = false;
        //stop_Speed(true);
        //setCurrentLevel();        
        printf("state.smart_brake=%s\n", state.smart_brake ? "true" : "false");
        if (!state.smart_brake){
            state.break_long=true;
            stop_Speed(true);
            setCurrentLevel();
        }
        state.change_event=20;
        printf("state.break_long=%s\n", state.break_long ? "true" : "false");
        printf("state.volt_bl_break_12=%d\n",state.volt_bl);
        printf("state.croiuse_level_12=%d\n",state.croiuse_level);        
        xSemaphoreGive(state_mutex);
    }
}



// static void button_event_break(void *arg, void *data) {
//     //printf("button_event_break\n");
//     if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
//         //printf("do button_event_break\n");
//         state.break_long = false;
//         state.break_level = 0;
//         gpio_set_level(BLUE_LED_PIN, 1);
//         DoLight(128);
//         state.controllerBrake = false;
//         state.break_volt_bl = state.volt_bl;
//         state.break_croiuse_level = state.croiuse_level;
//         state.addspeed = false;
//         stop_Speed(true);
//         setCurrentLevel();
//         state.change_event=20;
//         xSemaphoreGive(state_mutex);
//     }
//     //printf("button_event_break_end--------\n");
// }

// static void button_event_break_long(void *arg, void *data) { /// сработал долгий тормоз
//     //printf("button_event_break_long\n");
//     if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
//         state.break_long = true;
//         state.addspeed = false;
//         state.current_amper = 3; // Сбрасываем в исходное состояние
//         xSemaphoreGive(state_mutex);
//     }
//     //printf("button_event_break_long_end--------\n");
// }


static void button_event_break_end(void *arg, void *data) {
    printf("button_event_break_end\n");
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        gpio_set_level(BLUE_LED_PIN, 0);


            // Остановка задачи подсчёта


        if (!state.break_long) { /// не была нажата правая
            printf("state.volt_bl_left_break_end=%d\n",state.volt_bl);            
            //state.volt_bl = state.break_volt_bl;
            if (state.volt_bl > 0) { 
                //state.croiuse_level = get_level(true) - 1;
                printf("state.croiuse_level=%d\n",state.croiuse_level);        
                //if (state.croiuse_level < 1) state.croiuse_level = 0;
                if (!state.auto_speed){ /// если была начальная скорость или отключен плавный старт
                    printf("state.croiuse_level_not_=%d\n",state.croiuse_level);
                    state.change_event=state.croiuse_level+1;
                    state.break_level = 1;
                }
                else {
                //state.cr = -1;
                //state.crouise_on = true;
                //state.speed_up = true;
                /// для данного контроллера без использования add_speed  state.break_level = 1;
                //state.break_level = 1;
                //state.volt_bl = 0;
                //setCrouise(state.croiuse_level);
                state.change_event=state.croiuse_level+1;
                state.addspeed=true;      /// плавное увеличение скорости           
                     }
            }
        }
        else {
            state.break_level = 1;
            state.volt_bl = 0;
        }
        //state.break_volt_bl = 0;        
        state.break_long = false;
        DoLight(0);
        state.start_break_event=false;
        xSemaphoreGive(state_mutex);
    }
     //printf("button_event_break_end_end--------\n");
     //printf("state.break_level=%d\n",state.break_level);
}



// static void button_event_break_end(void *arg, void *data) {
//     //printf("button_event_break_end\n");
//     if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
//         gpio_set_level(BLUE_LED_PIN, 0);
//         state.rpm_controller = 0;
//         if (!state.break_long) {
//             //printf("state.break_volt_bl=%d",state.break_volt_bl);            
//             //printf("rpm_controller=%d",state.rpm_controller);
//             //printf("rpm_controller=%d\n",state.rpm_controller);
//             state.volt_bl = state.break_volt_bl;
//             if (state.volt_bl > 0) {
//                 state.croiuse_level = get_level(true) - 1;
//                 if (state.croiuse_level < 1) state.croiuse_level = 0;
//                 state.cr = -1;
//                 state.crouise_on = true;
//                 state.speed_up = true;
//                 //state.break_level = 1;
//                 state.volt_bl = 0;
//                 setCrouise(state.croiuse_level);
//                 state.change_event=state.croiuse_level+1;
//             }
//         }
//         else state.break_level = 1;
//         state.break_volt_bl = 0;
//         state.break_long = false;
//         DoLight(0);
//         xSemaphoreGive(state_mutex);
//     }
//      //printf("button_event_break_end_end--------\n");
//      //printf("state.break_level=%d\n",state.break_level);
// }

static void button_event_cb1(void *arg, void *data) {
    printf("button_event_cb1\n");
    button_handle_t btn_handle = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(btn_handle);
    if (event >= sizeof(button_event_names) / sizeof(button_event_names[0]) || !button_event_names[event]) return;

    const char *event_name = button_event_names[event];
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        if (strcmp(state.name_cont, "FT85BS") != 0) {
            xSemaphoreGive(state_mutex);
            return;
        }
          
        if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0) {
            //printf("button_event_single\n");
            //printf("state.break_level=%d\n",state.break_level);
            //printf("state.croiuse_level=%d\n",state.croiuse_level);
            //printf("state.rpm_controller%d\n",state.rpm_controller);
            
            BUTTON_SINGLE_CLICK_ADD();
        } else if (strcmp(event_name, "BUTTON_PRESS_REPEAT") == 0) {
            BUTTON_PRESS_REPEAT_ADD();          
        } else if (strcmp(event_name, "BUTTON_LONG_PRESS_START") == 0) {
            //AddSpeed();
            state.change_event=21;
            set_relay_state();
        }
        xSemaphoreGive(state_mutex);
    }
    printf("button_event_cb1_end\n");
}

static void button_event_cb2(void *arg, void *data) {
    //printf("button_event_cb2\n");
    button_handle_t btn_handle = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(btn_handle);
    if (event >= sizeof(button_event_names) / sizeof(button_event_names[0]) || !button_event_names[event]) return;

    const char *event_name = button_event_names[event];
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state.addspeed = false;
        if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0) {
            BUTTON_SINGLE_CLICK_DEC();
          
        } else if (strcmp(event_name, "BUTTON_PRESS_REPEAT") == 0) {
            BUTTON_PRESS_REPEAT_DEC();
        } else if (strcmp(event_name, "BUTTON_LONG_PRESS_START") == 0) {
            // state.crouise_on = false;
            // if (state.break_level == 1 && state.volt_bl >= state.start_level)
            //     state.volt_bl = getStep(false, state.volt_bl);
            // if (state.volt_bl < 0) state.volt_bl = 0;
            // setCurrentLevel();
            pulse_relay();
        }
        xSemaphoreGive(state_mutex);
    }
    //printf("button_event_cb2_end\n");
}

// Инициализация кнопок
static void button_init_rigth_break(uint32_t button_num) {
    // button_config_t btn_cfg = {
    //     .long_press_time = 3000,
    //     .short_press_time = 50,
    // };
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_DOWN, NULL, button_event_rigth_break, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_END, NULL, button_event_break_end, NULL);
    ESP_ERROR_CHECK(ret);
}


// Инициализация кнопок
static void button_init_break(uint32_t button_num) {
    // button_config_t btn_cfg = {
    //     .long_press_time = 3000,
    //     .short_press_time = 50,
    // };
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_DOWN, NULL, button_event_break, NULL);
    // ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_break_long, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_END, NULL, button_event_break_end, NULL);
    ESP_ERROR_CHECK(ret);
}

static void button_init1(uint32_t button_num) {
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, NULL, button_event_cb1, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb1, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_cb1, NULL);
    ESP_ERROR_CHECK(ret);
}

static void button_init2(uint32_t button_num) {
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, NULL, button_event_cb2, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb2, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_cb2, NULL);
    ESP_ERROR_CHECK(ret);
}

static void init_button(void) {
    button_init1(BUTTON_NUM1);
    button_init2(BUTTON_NUM2);
    button_init_break(BREAK_PIN);
    button_init_rigth_break(BREAK_RIGTH_PIN);
}

// Обработка данных UART
static uint8_t* processData(uint8_t* target_bytes, uint8_t* read_buff, int buff_size, int offset) {
    int index = -1;
    for (int i = 0; i < buff_size - 1; i++) {
        if (read_buff[i] == target_bytes[0] && read_buff[i + 1] == target_bytes[1]) {
            index = i;
            break;
        }
    }
    if (index != -1) {
        uint8_t* new_buff = (uint8_t*) malloc(buff_size - offset);
        if (new_buff) {
            memcpy(new_buff, read_buff, index);
            memcpy(new_buff + index, read_buff + index + offset, buff_size - index - offset);
            memcpy(read_buff, new_buff, buff_size - offset);
            free(new_buff);
        }
    }
    return read_buff;
}

static void sendBufferToController(uint8_t *buffer, uint8_t bufferSize) {
    uart_write_bytes(UART_NUM, (const char*)buffer, bufferSize);
    uart_wait_tx_done(UART_NUM, 2000 / portTICK_PERIOD_MS);
}


// void uart0_to_uart2_task(void *pvParameters) {
//     uart_event_t event;
//     uint8_t buffer[BUF_SIZE];
//     printf("start uart0_to_uart2_task");
//     while (1) {
//         if (xQueueReceive(uart0_queue, &event, portMAX_DELAY)) {
//             switch (event.type) {
//                 case UART_DATA:
//                     int len = uart_read_bytes(UART0_PORT_NUM, buffer, event.size, 0);
//                     //ESP_LOGI(TAG, "UART0 -> UART2: %d байт", len);
//                     if (len > 0) {
//                         ESP_LOGI(TAG, "UART0 -> UART2: %d байт", len);
//                         ESP_LOG_BUFFER_HEX(TAG, buffer, len);
//                         uart_write_bytes(UART_NUM, (const char *)buffer, len);
//                         ESP_LOGI(TAG, "UART0 -> UART2: %d байт", len);
//                     }
//                     break;
//                 case UART_FIFO_OVF:
//                     ESP_LOGE(TAG, "UART0: Переполнение FIFO");
//                     uart_flush_input(UART0_PORT_NUM);
//                     break;
//                 case UART_BUFFER_FULL:
//                     ESP_LOGE(TAG, "UART0: Буфер полный");
//                     uart_flush_input(UART0_PORT_NUM);
//                     break;
//                 default:
//                     ESP_LOGI(TAG, "UART0: Неизвестное событие: %d", event.type);
//                     break;
//             }
//         }
//     }
//     //free(rx_data);
//     vTaskDelete(NULL);
// }

// void uart2_to_uart0_task(void *pvParameters) {
//     uint8_t buffer[BUF_SIZE];
//     uart_event_t event;
//     uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE);
//     printf("uart2_to_uart0_task");
//     //if  (uart0_to_uart2_task_handle==NULL)xTaskCreatePinnedToCore(uart0_to_uart2_task, "uart0_to_uart2",4096, NULL, 5, &uart0_to_uart2_task_handle, 0);  
//     while (1) {
//         if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
//         switch (event.type) {
//                case UART_DATA:             
//         int len = uart_read_bytes(UART_NUM, rx_data, event.size, RX_TIMEOUT);
//         if (len > 0) {
//             ESP_LOGI(TAG, "UART2 -> UART0: %d байт", len);
//             ESP_LOG_BUFFER_HEX(TAG, buffer, len);
//             uart_write_bytes(UART0_PORT_NUM, (const char *)buffer, len);
//             ESP_LOGI(TAG_USB, "UART2 -> UART0: %d байт", len);
//             break;
//             case UART_FIFO_OVF:
//             ESP_LOGE("UART", "FIFO Overflow");
//             uart_flush_input(UART_NUM);
//             xQueueReset(uart_queue);
//             break;
//         case UART_BUFFER_FULL:
//             ESP_LOGE("UART", "Buffer Full");
//             uart_flush_input(UART_NUM);
//             xQueueReset(uart_queue);
//             break;
//         default:
//             break;     
//         } 
//         }
//       }
//     }
//     free(rx_data);
//     vTaskDelete(NULL);
// }


static void uart_task(void *arg) {
    uart_event_t event;
    uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE);
    size_t rx_length = 0;
    uint8_t start_bytes[] = {170, 12};

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    rx_length = uart_read_bytes(UART_NUM, rx_data, event.size, RX_TIMEOUT);
                    if (rx_length > 0) {
                        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                            if (state.operation == 0) {
                                processData(start_bytes, rx_data, rx_length, 6);
                                strncpy(state.name_cont, (char*)rx_data, 6);
                                state.name_cont[6] = '\0';
                                if (strcmp(state.name_cont, "FT85BS") == 0) {
                                    ESP_LOGI("UART", "Connected to controller");
                                    gpio_set_level(POWER_PIN, 1);
                                    state.operation = 1;
                                    state.count_telemtr = 0;                                    
                                }
                            } else if (state.operation == 1) {
                                if (rx_length == 34 && rx_data[0] == 170 && rx_data[1] == 29 && rx_data[33] == 221) {                                    
                                    state.Vbatt = (rx_data[5] << 8) | rx_data[6];
                                    uint16_t rpm = (rx_data[17] << 8) | rx_data[18];
                                    if (rpm < 30000) state.rpm_controller = rpm;
                                    //printf("state.rpm_controller =%d\n",state.rpm_controller);
                                    if (state.Vbatt < 3000) {
                                        memset(state.name_cont, 0, sizeof(state.name_cont));
                                        gpio_set_level(POWER_PIN, 0);                                        
                                    }
                                }
                            }
                            xSemaphoreGive(state_mutex);
                        }
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE("UART", "FIFO Overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGE("UART", "Buffer Full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                default:
                    break;
            }
        }
    }
    free(rx_data);
    vTaskDelete(NULL);
}

static void loop_controller(void* parameter) {
    printf("Start loop_controller\n");
    int my_voltbl = 0;
    uint8_t start_buffer[] = {170, 1, 17, 76, 127, 221};
    uint8_t telemetr_data[] = {0xaa, 0x04, 0x1a, 0x01, 0x00, 0x64, 0x17, 0x57, 0xdd};
    uint8_t stop_data[] = {170, 5, 4, 0, 0, 0, 0, 192, 213, 221};
    TickType_t wait_time = 2000 / portTICK_PERIOD_MS;
    
    if (uart_task_handle==NULL)xTaskCreatePinnedToCore(uart_task, "uart_task",4096, NULL, 10, &uart_task_handle, 0);  
    while (1) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            if (strcmp(state.name_cont, "FT85BS") != 0) {
                wait_time = 2000 / portTICK_PERIOD_MS;
                state.operation = 0;
                sendBufferToController(start_buffer, sizeof(start_buffer));
            } else {
                wait_time = 250 / portTICK_PERIOD_MS;
                if (state.operation == 1) {
                    if (state.count_telemtr == 0 || state.count_telemtr == 4 || state.count_telemtr == 8 ||
                        state.count_telemtr == 12 || state.count_telemtr == 16) {   
                        sendBufferToController(telemetr_data, sizeof(telemetr_data));
                    } else {
                        if (state.break_level == 0) {
                            printf("state.break_level == 0\n");
                            my_voltbl = 0;
                            sendBufferToController(stop_data, sizeof(stop_data));
                        } else {
                            if (state.volt_bl == 0 && state.current_level == 0) getDuty(0.0);
                            else if (state.addspeed) {
                                printf("==state.addspeed\n");
                                getSpeed(state.target_erpm);
                                sendBufferToController(speed_data, sizeof(speed_data));
                                state.croiuse_level = get_level(true) - 1;
                                if (state.croiuse_level < 1) state.croiuse_level = 0;
                            } else {
                                if (state.volt_bl == 0 && state.current_level > 0) state.volt_bl = state.current_level;
                                if (state.volt_bl > 20800) state.volt_bl = 20800;
                                //printf("setCurrentLevel() from loop_controller\n");                       
                                if (my_voltbl != state.volt_bl) {
                                    my_voltbl = state.volt_bl;
                                    getSpeed(state.volt_bl);
                                    setCurrentLevel();
                                    //sendBufferToController(speed_data, sizeof(speed_data));
                                }
                                    sendBufferToController(speed_data, sizeof(speed_data));
                            }
                        }
                    }
                    state.count_telemtr++;
                    if (state.count_telemtr > 19) state.count_telemtr = 0;
                }
            }
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(wait_time);
    }
}

static void configure_led(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLUE_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Функция-заглушка, которая игнорирует вывод
static int log_dummy(const char *fmt, va_list args) {
    return 0; // Просто возвращаем 0, ничего не выводим
}
static int null_output(struct _reent *r, void *fd, const char *ptr, int len) {
    (void)r;   // Подавляем предупреждение о неиспользуемой переменной
    (void)fd;  // То же самое для fd
    return len; // Возвращаем длину, как будто вывод успешен
}
void app_main(void) {
    if (!logging){
    esp_log_set_vprintf(log_dummy); // Устанавливаем заглушку на логи esp

    setvbuf(stdout, NULL, _IONBF, 0); // Отключаем буферизацию  
    stdout->_write = null_output;     // отключает вывод логов по printf
    }
    // uart_init();
    controller_init();
    init_button();
    configure_led();
    getDuty(0.0);
    initBreakLight();
    configure_gpio();
    configure_pulse_gpio();
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_PIN, 0);

    // esp_vfs_dev_uart_use_nonblocking(UART_PORT);
    // uart_driver_delete(UART_PORT); // Удаляем драйвер консоли

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT, 
    };

    // uart_config_t uart_config = {
    //     .baud_rate = 115200,              // Стандартная скорость для прошивки
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .rx_flow_ctrl_thresh = 122,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };

    // // Установка UART с буфером
    // ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 20, &uart_queue, 0));
    // ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_enable_rx_intr(UART_PORT));
    // // Минимизируем логи
    // esp_log_level_set(TAG, ESP_LOG_WARN);
    // ESP_LOGI(TAG, "UART0 initialized for Type-C");


    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    //ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM));

    printf("Start prog\n");


    //xTaskCreate(uart0_to_uart2_task, "uart0_to_uart2_task", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(loop_controller, "Loop_controller", 4096, NULL, 10, &loop_controller_handle, 1);
    //xTaskCreatePinnedToCore(loop_controller, "Loop_controller", 4096, NULL, 10, NULL, 1);
    start_ble();   
    //state.diagnostic=true;
    while (1) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {  
        //printf("state.diagnostic=%s\n", state.diagnostic ? "true" : "false");
        // if (state.diagnostic) {
        //      if (loop_controller_handle != NULL) {
        //         if (uart_task_handle!=NULL){
        //             vTaskDelete(uart_task_handle);
        //             uart_task_handle=NULL;
        //                                    }     
        //         vTaskDelete(loop_controller_handle);
        //         loop_controller_handle = NULL; //                 
        //                                          } 
        //          if (uart0_controller_handle==NULL)xTaskCreate(uart2_to_uart0_task, "uart2_to_uart0", 4096, NULL, 5,&uart0_controller_handle);
        //                      }
        //  else {
        //        if (uart0_controller_handle !=NULL){
        //         if (uart0_to_uart2_task_handle!=NULL){
        //             vTaskDelete(uart0_to_uart2_task_handle);    
        //             uart0_to_uart2_task_handle=NULL;
        //           }
        //           vTaskDelete(uart0_controller_handle);     
        //           uart0_controller_handle=NULL;                                    
        //                                         }
        //        if (loop_controller_handle==NULL)xTaskCreatePinnedToCore(loop_controller, "Loop_controller", 4096, NULL, 10, &loop_controller_handle, 1);
        //        }                                  

           
                                                
           if (!state.auto_speed && addspeed_controller_handle!=NULL){
            vTaskDelete(addspeed_controller_handle);
            addspeed_controller_handle=NULL;
           }                                
           if (state.auto_speed && addspeed_controller_handle==NULL)xTaskCreatePinnedToCore(do_add_speed, "Do add speed", 10000, NULL, 10, &addspeed_controller_handle, 0);  
           if (!state.auto_speed && addspeed_controller_handle!=NULL){
                  vTaskDelete(addspeed_controller_handle);     
                  addspeed_controller_handle=NULL;     
           }
           xSemaphoreGive(state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
             }
}