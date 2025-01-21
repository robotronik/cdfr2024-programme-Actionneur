#pragma once
#include <stdint.h>

int8_t ReadInt8(uint8_t *buffer[]);
int16_t ReadInt16(uint8_t *buffer[]);
int32_t ReadInt32(uint8_t *buffer[]);
int64_t ReadInt64(uint8_t *buffer[]);

void WriteInt8(uint8_t *buffer[], int8_t val);
void WriteInt16(uint8_t *buffer[], int16_t val);
void WriteInt32(uint8_t *buffer[], int32_t val);
void WriteInt64(uint8_t *buffer[], int64_t val);