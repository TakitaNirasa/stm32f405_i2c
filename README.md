# stm32f405_i2c
I2C library for stm32f4xx

How to use:
```c
#include "stm32f4xx_i2c.h"

void next_step(void *struct1)
{...}

int main(void)
{
    i2c_t  i2c_struct;
    i2c_init(&i2c_struct, 400000, i1pb9pb8);

    uint8_t a[5] = {0,1,2,3,4};

    i2c_write(&i2c_struct, 0b10100100, a, 5, 0, 0, &struct1, next_step);
    while(1)
    {
       i2c_main_cycle(&i2c_struct);
    }
}
```
