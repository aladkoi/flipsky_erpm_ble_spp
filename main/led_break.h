#ifndef LED_BREAK_H
#define LED_BREAK_H

#include <driver/gpio.h>
#include <driver/ledc.h>

// Определение пина для стоп-сигнала
#define BREAK_LIGHT_PIN GPIO_NUM_5 // Можно заменить на ваш пин

// Объявления функций
void initBreakLight(void);
void DoLight(int data);

#endif // LED_BREAK_H