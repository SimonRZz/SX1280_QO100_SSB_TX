#ifndef TEST_STDLIB_H
#define TEST_STDLIB_H
#include <stdint.h>
typedef uint64_t absolute_time_t;
extern uint64_t test_time_us;
static inline absolute_time_t get_absolute_time(void) { return test_time_us++; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return test_time_us + (uint64_t)ms * 1000; }
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) { return (int64_t)(to - from); }
static inline void sleep_us(uint64_t us) { test_time_us += us; }
static inline void sleep_ms(uint32_t ms) { test_time_us += (uint64_t)ms * 1000; }
#endif
