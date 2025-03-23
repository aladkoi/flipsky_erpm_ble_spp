#ifndef CRC_H
#define CRC_H

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>  // Добавляем для size_t

// Объявления глобальных массивов (без инициализации)
extern uint8_t duty_data[10];
extern uint8_t amper_data[10];
extern uint8_t speed_data[10];
extern uint16_t amper_crc[];
extern volatile int erpm_step[];
extern uint16_t erpm_crc[];

// Объявления функций
uint16_t crc16arc_bit(uint8_t *data, size_t len);
void convertToBytes(uint32_t number, uint8_t *bytes);
void getAmper(int number);
int find_index_linear(int target);
int getStep(bool forward, int value);
int getCurrentIndexSpeed(int value);
int getCurrentDecIndexSpeed(int value);
int getCurrentSpeed(int value);
void getSpeed(int number);
void getDuty(float number);

#endif // CRC_H