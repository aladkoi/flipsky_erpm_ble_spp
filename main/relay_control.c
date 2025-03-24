#include "relay_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Определение глобальной переменной
volatile bool relay_state = false; // false - выключено, true - включено

// Функция для настройки GPIO с подтяжкой вверх
void configure_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),    // Маска для GPIO 32
        .mode = GPIO_MODE_OUTPUT,               // Режим вывода
        .pull_up_en = GPIO_PULLUP_ENABLE,       // Включаем подтяжку вверх
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Отключаем подтяжку вниз
        .intr_type = GPIO_INTR_DISABLE          // Отключаем прерывания
    };
    gpio_config(&io_conf);

    // Устанавливаем начальное состояние (HIGH благодаря pull-up, реле выключено)
    gpio_set_level(RELAY_PIN, 1);
    relay_state = false; // Состояние "выключено", так как HIGH выключает реле
}

// Функция для переключения состояния реле
void set_relay_state(void) {
    if (relay_state == false)printf("relay_state=false\n");
    else printf("relay_state=true\n");
    if (relay_state == false) {
        gpio_set_level(RELAY_PIN, 0); // LOW - включаем реле (активный LOW)
        relay_state = true;
    } else {
        gpio_set_level(RELAY_PIN, 1); // HIGH - выключаем реле
        relay_state = false;
    }
}