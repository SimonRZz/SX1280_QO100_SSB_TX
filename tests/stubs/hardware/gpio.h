#ifndef TEST_GPIO_H
#define TEST_GPIO_H
#include <stdbool.h>
#define GPIO_FUNC_SIO 0
#define GPIO_FUNC_I2C 3
#define GPIO_OUT true
static inline void gpio_set_function(unsigned pin, int fn) { (void)pin; (void)fn; }
static inline void gpio_set_dir(unsigned pin, bool out) { (void)pin; (void)out; }
static inline void gpio_put(unsigned pin, bool value) { (void)pin; (void)value; }
static inline void gpio_init(unsigned pin) { (void)pin; }
#endif
