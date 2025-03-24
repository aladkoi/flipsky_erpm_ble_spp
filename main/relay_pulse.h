#ifndef RELAY_PULSE_H
#define RELAY_PULSE_H

#include "driver/gpio.h"

// Определяем контакт для реле с импульсным управлением
#define RELAY_PULSE_PIN GPIO_NUM_33

// Функции для управления реле
void configure_pulse_gpio(void);
void pulse_relay(void); // Функция для срабатывания реле на 1 секунду

#endif // RELAY_PULSE_H