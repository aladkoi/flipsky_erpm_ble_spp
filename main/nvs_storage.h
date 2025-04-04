#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdbool.h>

// Инициализация NVS
void nvs_init(void);

// Сохранение bool значения в NVS
void save_to_nvs(uint8_t data);
void save_to_nvs1(uint8_t data);
void savespeed_to_nvs(int data);
// Чтение bool значения из NVS
void read_from_nvs(void);

#endif // NVS_STORAGE_H