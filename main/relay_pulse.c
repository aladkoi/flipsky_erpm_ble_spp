#include "relay_pulse.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Функция для настройки GPIO с подтяжкой вверх
void configure_pulse_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PULSE_PIN),    // Маска для GPIO 33
        .mode = GPIO_MODE_OUTPUT,                     // Режим вывода
        .pull_up_en = GPIO_PULLUP_ENABLE,             // Включаем подтяжку вверх
        .pull_down_en = GPIO_PULLDOWN_DISABLE,        // Отключаем подтяжку вниз
        .intr_type = GPIO_INTR_DISABLE                // Отключаем прерывания
    };
    gpio_config(&io_conf);

    // Устанавливаем начальное состояние (HIGH благодаря pull-up, реле выключено)
    gpio_set_level(RELAY_PULSE_PIN, 1);
}

// Функция для срабатывания реле на 1 секунду
void pulse_relay(void) {
    gpio_set_level(RELAY_PULSE_PIN, 0); // LOW - включаем реле (активный LOW)
    vTaskDelay(300 / portTICK_PERIOD_MS); // Ждём 1 секунду
    gpio_set_level(RELAY_PULSE_PIN, 1); // HIGH - выключаем реле
}