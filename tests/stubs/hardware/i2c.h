#ifndef TEST_I2C_H
#define TEST_I2C_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { int unused; } i2c_inst_t;
extern i2c_inst_t test_i2c;
#define i2c0 (&test_i2c)
int i2c_write_blocking(i2c_inst_t *, uint8_t, const uint8_t *, size_t, bool);
int i2c_read_blocking(i2c_inst_t *, uint8_t, uint8_t *, size_t, bool);
#endif
