#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <stdbool.h>

// Инициализация NVS
void nvs_init(void);

// Сохранение bool значения в NVS
void save_bool_to_nvs(bool value);

// Чтение bool значения из NVS
bool read_bool_from_nvs(void);

#endif // NVS_STORAGE_H