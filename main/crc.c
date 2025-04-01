#include "crc.h"

// Определения глобальных массивов
uint8_t duty_data[10] = {170, 5, 3, 0, 0, 0, 0, 0, 0, 221};  // =0
uint8_t amper_data[10] = {170, 5, 4, 0, 0, 0, 0, 0, 0, 221};  // =0
uint8_t speed_data[10] = {0xAA, 0x05, 0x27, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0xDD};

uint16_t amper_crc[] = {
    0xC0D5, 0x7ED5, 0x6CD6, 0x82d2, 0x48d0, 0x96d8, 0xd4db, 0xcade,
    0x00dc, 0xeecc, 0xfccf, 0x22cb, 0xe8c9, 0x36c1, 0xd4c3, 0xcac6,
    0x00c4, 0x8ee4, 0x9ce7, 0x72e3, 0xb8e1
};

volatile int erpm_step[55] = {
    0, 373, 746, 1119, 1492, 1865, 2238, 2611, 2984, 3358, 3731, 4104, 4477, 4700,
    4850, 5223, 5596, 5969, 6342, 6715, 7088, 7330, 7461, 7834, 8207, 8580, 8953,
    9326, 9699, 10073, 10115, 10446, 10819, 11192, 11565, 11938, 12311, 12684,
    12870, 13057, 13430, 13803, 14176, 14549, 14922, 15295, 15668, 16041, 16415,
    16788, 17161, 17534, 17907, 18280, 20800
};

uint16_t erpm_crc[] = {
    0x0710, 0x70d0, 0xe890, 0xff52, 0x0813, 0xc1d3, 0xb797, 0xb256, 0x8916, 0x9f94,
    0x0a54, 0x011c, 0x76dc, 0x9e1c, 0x229d, 0xed5e, 0x0e1f, 0x0bde, 0x559a, 0xb45a,
    0x431b, 0x7e99, 0x8cd8, 0xcc99, 0xc349, 0x3408, 0xe5c8, 0xeb8a, 0x1e4b, 0xcdcb,
    0x564a, 0x938f, 0x964e, 0x850e, 0x4acd, 0x1e8c, 0x0944, 0x3204, 0x9584, 0x37c5,
    0x2187, 0x1847, 0x07ae, 0x58c2, 0x5083, 0x8742, 0x4001, 0x19c0, 0x0f60, 0xf821,
    0x31e1, 0x27a3, 0xd262, 0x1922, 0x672c
};

// Реализация функций
uint16_t crc16arc_bit(uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned k = 0; k < 8; k++) {
            crc = crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }

    uint8_t *bytePtr = (uint8_t *)&crc;
    uint8_t lastByte = bytePtr[0];
    int result = lastByte + 96;
    if (result > 127) {
        if (result & (1 << 7) && result & (1 << 5)) {
            result &= ~(1 << 7);
        } else if (result & (1 << 7) && result & (1 << 6)) {
            result &= ~(1 << 7);
            result &= ~(1 << 6);
        } else {
            result &= ~(1 << 7);
            result |= (1 << 6);
        }
    }
    bytePtr[0] = (uint8_t)result;
    return crc;
}

void convertToBytes(uint32_t number, uint8_t *bytes) {
    bytes[0] = (uint8_t)(number >> 24);
    bytes[1] = (uint8_t)(number >> 16);
    bytes[2] = (uint8_t)(number >> 8);
    bytes[3] = (uint8_t)number;
}

void getAmper(int number) {
    uint8_t bytes[4];
    uint16_t crc;
    convertToBytes(number * 1000, bytes);
    for (int k = 0; k < sizeof(bytes); k++) {
        amper_data[k + 3] = bytes[k];
    }
    crc = amper_crc[number];
    amper_data[7] = (uint8_t)(crc >> 8);
    amper_data[8] = (uint8_t)crc;
}

int find_index_linear(int target) {
    for (int i = 0; i < sizeof(erpm_step) / sizeof(erpm_step[0]); i++) {
        if (erpm_step[i] == target) {
            return i;
        }
    }
    return -1;
}

int getStep(bool forward, int value) {
    int index = find_index_linear(value);
    if (index < 0) return 0;
    if (forward) {
        index++;
        if (index >= sizeof(erpm_step) / sizeof(erpm_step[0])) index--;
    } else {
        index--;
        if (index < 0) index = 0;
    }
    return erpm_step[index];
}

int getCurrentIndexSpeed(int value) {
    for (int i = 0; i < sizeof(erpm_step) / sizeof(erpm_step[0]); i++) {
        if (erpm_step[i] >= value) {
            i++;
            if (i >= sizeof(erpm_step) / sizeof(erpm_step[0])) i--;
            return i;
        }
    }
    return -1;
}

int getCurrentDecIndexSpeed(int value) {
    for (int i = 0; i < sizeof(erpm_step) / sizeof(erpm_step[0]); i++) {
        if (erpm_step[i] <= value) {
            return i;
        }
    }
    return -1;
}

int getCurrentSpeed(int value) {
    for (int i = 0; i < sizeof(erpm_step) / sizeof(erpm_step[0]); i++) {
        if (erpm_step[i] >= value) {
            if (i >= sizeof(erpm_step) / sizeof(erpm_step[0])) i--;
            return erpm_step[i];
        }
    }
    return -1;
}

void getSpeed(int number) {
    uint16_t crc;
    int index = find_index_linear(number);
    if (index != -1) {
        speed_data[5] = (uint8_t)(number >> 8) & 0xFF;
        speed_data[6] = (uint8_t)number & 0xFF;
        crc = erpm_crc[index];
        speed_data[7] = (uint8_t)(crc >> 8);
        speed_data[8] = (uint8_t)crc;
    } else {
        speed_data[5] = 0;
        speed_data[6] = 0;
        speed_data[7] = 0x07;
        speed_data[8] = 0x10;
    }
}

void getDuty(float number) {
    uint8_t bytes[4];
    uint16_t crc;
    convertToBytes(number * 1000, bytes);
    for (int k = 0; k < sizeof(bytes); k++) {
        duty_data[k + 3] = bytes[k];
    }
    crc = crc16arc_bit(bytes, sizeof(bytes) / sizeof(bytes[0]));
    duty_data[7] = (uint8_t)(crc >> 8);
    duty_data[8] = (uint8_t)crc;
}