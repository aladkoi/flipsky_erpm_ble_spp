#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h" // Для SemaphoreHandle_t
#include "freertos/semphr.h"   // Для мьютекса
#include "crc.h"
#include "nvs_storage.h"
#include "led_break.h"
#include "relay_control.h"
#include "relay_pulse.h"







// Массивы круиза
// Структура состояния системы
typedef struct {
    char name_cont[8];         // Имя контроллера (с завершающим нулем)
    uint16_t Vbatt;               // Напряжение батареи
    uint16_t rpm_controller;        // Текущие обороты
    int volt_bl;               // Уровень для передачи в контроллер
    int break_volt_bl;         // Уровень сигнала от мобильного при торможении
    int start_level;           // Уровень начального старта
    int current_rpm;           // Текущий уровень RPM для сохранения
    int current_level;         // Текущий уровень газа
    int current_level_speed;   // Уровень газа для возобновления скорости
    int current_level_rpm;     // Уровень RPM для возобновления скорости
    int croiuse_level;         // Текущий уровень круиза
    int break_croiuse_level;   // Уровень круиза при торможении
    bool controllerBrake;      // Нажата ручка тормоза
    int currentSpeed;          // Скорость для круиза
    int cr;                    // Текущий индекс круиза
    int current_amper;         // Текущий индекс шага скорости
    int target_erpm;           // Целевая скорость (ERPM)
    bool crouise_on;           // Включен круиз
    bool isn;                  // Умный тормоз
    int count_telemtr;         // Количество телеметрии
    int numberCrouise;         // Номер круиза
    bool stop_from_mobile;     // Стоп с мобильного
    bool break_long;           // Долгое нажатие тормоза
    bool speed_up;             // Скорость нарастает
    bool addspeed;             // Увеличение скорости с 0
    int break_level;           // Уровень тормоза (1 - не нажат, 0 - нажат)
    int operation;             // Номер операции (0 - соединение, 1 - телеметрия)
    int change_event;             // событие для мобильного приложения
    int volt_add_speed;
    bool start_break_event;
    bool limit_speed;
} ControllerState_t;

// Глобальные переменные
extern ControllerState_t state;
extern SemaphoreHandle_t state_mutex;
extern volatile float level_crouise[6];
extern volatile int rpm_crouise[6];
//extern uint8_t speed_data[];
extern volatile int len_crouise;

// Функции управления состоянием
void controller_init(void);         // Добавлена инициализация
void stop_Speed(bool status);
void setCurrentLevel(void);
int get_level(bool forward);
void select_level(int crouise);
float start_crouise(void);
void AddSpeed(void);
void setCrouise(int crouise);
void BUTTON_PRESS_REPEAT_ADD(void);
void BUTTON_SINGLE_CLICK_ADD(void);
void BUTTON_SINGLE_CLICK_DEC(void);
void BUTTON_PRESS_REPEAT_DEC(void);
// void getDuty(float duty);
// int getStep(bool forward, int current);
// int getCurrentDecIndexSpeed(int value);
// int find_index_linear(int target);
// void getSpeed(int number);
// int getCurrentIndexSpeed(int value);

#endif // CONTROLLER_H