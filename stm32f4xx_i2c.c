#include "stm32f4xx_i2c.h"

/// Указатели на структуры, которые будут использоваться в прерываниях
i2c_t *i2c1_struct;
i2c_t *i2c2_struct;
i2c_t *i2c3_struct;

/**
 * @brief Инициализация пинов SDA и SCL
 * @param [in] SDA_port      Порт SDA
 * @param [in] SDA_pin       Пин SDA
 * @param [in] SCL_port      Порт SCL
 * @param [in] SCL_pin       Пин SCL
 */
static void i2c_gpio_init(GPIO_TypeDef *sda_port, uint8_t sda_pin,
                          GPIO_TypeDef *scl_port, uint8_t scl_pin)
{
    // Пины на выход(альтернативная функция)
    gpio_pin_set_mode(scl_port, scl_pin, 0x02);                  //Set GPIO mode AF
    gpio_pin_set_mode(sda_port, sda_pin, 0x02);                  

    gpio_pin_set_af(scl_port, scl_pin, 0b0100);                 // AF is I2C
    gpio_pin_set_af(sda_port, sda_pin, 0b0100);

    gpio_pin_set_speed (scl_port, scl_pin, 0);                  // Set to low freq
    gpio_pin_set_speed (sda_port, sda_pin, 0);
    // Open-drain выход
    gpio_pin_set_out_type(sda_port, sda_pin, 1);                // Open drain output
    gpio_pin_set_out_type(scl_port, scl_pin, 1);
}

i2c_prog_error_t i2c_init (i2c_t *i2c_st, const uint32_t freq, const i2c_init_hw_vars_t i2c_variant)
{
    // Для пустоко указателя просто возвращаем false
    if (!i2c_st)
    {
        return prog_i2c_empty_struct;
    }
    // Проверка правильности заданной частоты
    const uint32_t ccr_clk = APB1CLOCK / (2 * freq);
    if (ccr_clk > 0xFFF)                             // Если данные выходят за пределы регистра
    {
        return prog_i2c_err_init;
    }

    I2C_TypeDef *i2c_instance;

    if (i2c_variant == i1pb7pb6 || i2c_variant == i1pb7pb8 ||
            i2c_variant == i1pb9pb6 || i2c_variant == i1pb9pb8)
    {
        if (i2c1_struct != NULL)
            return prog_i2c_already_init;
        // Тактирование порта B
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        // Ресет I2C1
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
         // включаем тактирование I2C1
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        switch (i2c_variant)
        {
        case i1pb7pb6:
            i2c_gpio_init (GPIOB, 7, GPIOB, 6);
            break;
        case i1pb7pb8:
            i2c_gpio_init (GPIOB, 7, GPIOB, 8);
            break;
        case i1pb9pb6:
            i2c_gpio_init (GPIOB, 9, GPIOB, 6);
            break;
        case i1pb9pb8:
            i2c_gpio_init (GPIOB, 9, GPIOB, 8);
            break;
        default: break;
        }
        // Назначаем глобальному указателю адрес структуы(для прерываний)
        i2c1_struct = i2c_st;
        NVIC_EnableIRQ (I2C1_EV_IRQn);               // Разрешаем прерывания
        //NVIC_SetPriority (I2C1_EV_IRQn, 2);
        NVIC_EnableIRQ (I2C1_ER_IRQn);
        //NVIC_SetPriority (I2C1_ER_IRQn, 2);
        i2c_instance = I2C1;
    }
    else if (i2c_variant == i2pb11pb10)
    {
        if (i2c2_struct != NULL)
            return prog_i2c_already_init;
        // Тактирование порта B
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        // Ресет I2C2
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
         // включаем тактирование I2C2
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        i2c_gpio_init (GPIOB, 11, GPIOB, 10);
        // Присваиваем указателю значение
        i2c2_struct = i2c_st;
        NVIC_EnableIRQ (I2C2_EV_IRQn);               // Разрешаем прерывания
        NVIC_EnableIRQ (I2C2_ER_IRQn);
        i2c_instance = I2C2;
    }
    else if (i2c_variant == i3pc9pa8)
    {
        if (i2c3_struct != NULL)
            return prog_i2c_already_init;
        // Тактирование порта C
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        // Тактирование порта A
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        // Ресет I2C3
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
         // включаем тактирование I2C3
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        i2c_gpio_init (GPIOC, 9, GPIOA, 8);
        // Присваиваем указателю значение
        i2c3_struct = i2c_st;
        NVIC_EnableIRQ (I2C3_EV_IRQn);               // Разрешаем прерывания
        NVIC_EnableIRQ (I2C3_ER_IRQn);
        i2c_instance = I2C3;
    }
    else
    {
        return prog_i2c_err_init;
    }

    i2c_st->state = i2c_state_idle;
    i2c_st->error_i2c = i2c_err_no;

    i2c_instance->CR2 |= I2C_CR2_ITEVTEN;               // Прерывание по системному событию
    i2c_instance->CR2 |= I2C_CR2_ITERREN;               // Прерывание по ошибке
    i2c_instance->CR1 &= ~I2C_CR1_SMBUS;                // настраиваем модуль в режим I2C
    i2c_instance->CR2 &= ~I2C_CR2_FREQ;                 // указываем частоту тактирования модуля
    i2c_instance->CR2 |= APB1CLOCK / 1000000;           // задаём частоту генератора
    // конфигурируем I2C, standart mode, 100 KHz duty cycle 1/2
    i2c_instance->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY);
    i2c_instance->CCR |= ccr_clk;                       // задаем частоту работы модуля SCL
    i2c_instance->TRISE = 5;                            // rise time

    i2c_st->instance = i2c_instance;
    return prog_i2c_err_no;
}

i2c_prog_error_t i2c_deinit (i2c_t *i2c_st)
{
    // Проверка на существования аргумента
    if (i2c_st == NULL)
        return prog_i2c_empty_struct;
    // Если состояние не
    if(i2c_st->state != i2c_state_idle)
        return prog_i2c_err_i2c_busy;
    // Отключение прерываний
    i2c_st->instance->CR2 &= ~I2C_CR2_ITEVTEN;
    i2c_st->instance->CR2 &= ~I2C_CR2_ITERREN;
    i2c_st->instance->CR2 &= ~I2C_CR2_ITBUFEN;
    // Посылаем стоп, чтоб точно отпустить линию SDA
    i2c_st->instance->CR1 |= I2C_CR1_STOP;
    i2c_st->instance->CR1 &= ~I2C_CR1_PE;           // Отключение I2C
    // Находим используемый экземпляр I2C
    if (i2c_st->instance == I2C1)
    {
        i2c1_struct = NULL;                         // Чистим глобальную структуру
        NVIC_DisableIRQ (I2C1_EV_IRQn);             // Запрещаем прерывания
        NVIC_DisableIRQ (I2C1_ER_IRQn);
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;        // Отключаем тактирование интерфейса
    }
    else if (i2c_st->instance == I2C2)
    {
        i2c2_struct = NULL;
        NVIC_DisableIRQ (I2C2_EV_IRQn);
        NVIC_DisableIRQ (I2C2_ER_IRQn);
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
    }
    else if (i2c_st->instance == I2C3)
    {
        i2c3_struct = NULL;
        NVIC_DisableIRQ (I2C3_EV_IRQn);
        NVIC_DisableIRQ (I2C3_ER_IRQn);
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
    }
    // Обнуляем структуру
    i2c_st = NULL;
    return prog_i2c_err_no;
}

i2c_error_t i2c_errors_get (i2c_t *i2c_st)
{
    return i2c_st->error_i2c;
}
/**
 * @brief Обработчик прерывания ошибки железа
 * @param [in, out] i2c_st     Структура I2C
 */
static void i2c_error_handler (i2c_t *i2c_st)
{
    if (i2c_st == NULL)
        return;
    // Считываем SR1 сразу в переменную(SR2 считаем, когда понадобится сбросить флаг ADDR)
    const uint16_t reg_sr1 = i2c_st->instance->SR1;
    // Acknowledge failure
    if (reg_sr1 & I2C_SR1_AF)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_AF;       // Сбрасываем флаг
        i2c_st->error_i2c = i2c_err_af;             // Выставляем ошибку в структуру
    }
    // Bus error
    else if (reg_sr1 & I2C_SR1_BERR)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_BERR;
        i2c_st->error_i2c = i2c_err_berr;
    }
    // Arbitration lost
    else if (reg_sr1 & I2C_SR1_ARLO)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_ARLO;
        i2c_st->error_i2c = i2c_err_arlo;
    }
    // Overrun/Underrun
    else if (reg_sr1 & I2C_SR1_OVR)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_OVR;
        i2c_st->error_i2c = i2c_err_ovr;
    }
    // PEC Error in reception
    else if (reg_sr1 & I2C_SR1_PECERR)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_PECERR;
        i2c_st->error_i2c = i2c_err_pecerr;
    }
    // Timeout or Tlow error
    else if (reg_sr1 & I2C_SR1_TIMEOUT)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_TIMEOUT;
        i2c_st->error_i2c = i2c_err_timeout;
    }
    // SMBus alert
    else if (reg_sr1 & I2C_SR1_SMBALERT)
    {
        i2c_st->instance->SR1 &= ~I2C_SR1_SMBALERT;
        i2c_st->error_i2c = i2c_err_smbalert;
    }
    // Переходим к вызову колбэка

    i2c_st->instance->CR1 |= I2C_CR1_STOP;      // Скидываем стоп
    i2c_st->instance->CR1 &= ~I2C_CR2_ITBUFEN;  // Отключаем прерывания по приёму/передаче
    i2c_st->instance->CR1 &= ~I2C_CR1_PE;       // Отключение I2C

    i2c_st->state = i2c_state_callback;
}

/**
 * @brief Обработчик конечного автомата чтения из прерывания
 * @note  Стоит обратить внимание на описание приёма данных по I2C в даташите!!!
 * @param [in, out] i2c_st  Структура I2C
 * @param [in] reg_sr1      Значение регистра SR1
 */
static void i2c_read_handler (i2c_t *i2c_st, const uint16_t reg_sr1)
{
    I2C_TypeDef *i2cn = i2c_st->instance;       // Для удобства объявлям переменную I2Cn
    // Отправлен старт бит
    if (reg_sr1 & I2C_SR1_SB)
    {
        i2cn->DR = i2c_st->dev_addr | 0x01;     // Чтение
        return;
    }
    // Отправлен адрес, считываем байт
    if (reg_sr1 & I2C_SR1_ADDR)
    {
        if (i2c_st->len0 == 1)                  // Когда нам нужно один байт принять
            i2cn->CR1 &= ~I2C_CR1_ACK;          // Готовимся отправлять NACK
        else if (i2c_st->len0 == 2)             // Для двух байт процедура немного отличается
        {
            i2cn->CR1 &= ~I2C_CR1_ACK;          // Set ACK low
            i2cn->CR1 |= I2C_CR1_POS;           // Set POS high
        }
        // Для 3х и более байт мы только выставляем ACK
        else
            i2cn->CR1 |= I2C_CR1_ACK;           // Set ACK high
        i2c_st->byte_counter = 0;               // Обнуляем счётчик байт
        i2cn->CR2 |= I2C_CR2_ITBUFEN;           // Разрешаем прерывания по приёму
        (void)i2cn->SR2;                        // Сброс бита адреса
        return;
    }
    // Получили байт
    if (reg_sr1 & I2C_SR1_RXNE)
    {
        // Если принимаем только один байт
        if (i2c_st->len0 == 1)
        {
            i2c_st->data0[i2c_st->byte_counter] = i2cn->DR;
            i2cn->CR1 |= I2C_CR1_STOP;           // Скидываем стоп
            i2cn->CR1 &= ~I2C_CR2_ITBUFEN;       // Отключаем прерывания по приёму/передаче
            i2c_st->state = i2c_state_callback;  // Переходим в состояние конца передачи
        }
        // Если принимаем два байта
        else if (i2c_st->len0 == 2)
        {
            i2c_st->data0[i2c_st->byte_counter] = i2cn->DR;

            if (i2c_st->byte_counter == 0)       //Если это первое прерывание
            {
                i2cn->CR1 |= I2C_CR1_STOP;       // Отсылаем стоп
                // Увеличиваем счётчик, так как будет ещё одно прерывание
                i2c_st->byte_counter++;
            }
            else                                 // Для второго прерывания
            {
                i2cn->CR1 &= ~I2C_CR2_ITBUFEN;   // Отключаем прерывания по tx/rx
                // Переходим в состояние конца передачи
                i2c_st->state = i2c_state_callback;
            }
        }
        // Для данных размером трёх и более байт
        else if (i2c_st->len0 >= 3)
        {
            i2c_st->data0[i2c_st->byte_counter] = i2cn->DR;

            // Принят предпоследний байт(мы считаем количество от 1, поэтому -2)
            if (i2c_st->byte_counter == i2c_st->len0 - 2)
            {
                i2cn->CR1 &= ~I2C_CR1_ACK;       // Ставим NACK
            }
            // Принимаем последний байт
            else if (i2c_st->byte_counter == i2c_st->len0 - 1)
            {
                i2cn->CR1 |= I2C_CR1_STOP;       // Сразу скидываем стоп
                // Отключаем прерывания по приёму/передаче
                i2cn->CR1 &= ~I2C_CR2_ITBUFEN;   // Отключаем прерывания rx/tx
                // Переходим в состояние конца передачи
                i2c_st->state = i2c_state_callback;
            }
            // Добавляем 1 к счётчику
            i2c_st->byte_counter++;
        }
    }
}

/**
 * @brief Обработчик конечного автомата записи из прерывания
 * @param [in, out] i2c_st  Структура I2C
 * @param [in] reg_sr1      Значение регистра SR1
 */
static void i2c_write_handler (i2c_t *i2c_st, const uint16_t reg_sr1)
{
    I2C_TypeDef *i2cn = i2c_st->instance;               // Для удобства объявлям переменную I2Cn
    // Старт бит подан
    if (reg_sr1 & I2C_SR1_SB)
    {
       i2cn->DR = i2c_st->dev_addr & ~0x01;             // Передаём адрес с флагом записи
       return;
    }
    // Адрес передан
    if (reg_sr1 & I2C_SR1_ADDR)
    {
        (void)i2cn->SR2;                                // Сброс бита адреса
        i2c_st->byte_counter = 0;                       // Обнуляем счётчик байт
        i2cn->DR = i2c_st->data0[i2c_st->byte_counter]; // Записываем адрес в памяти
        if (i2c_st->len0 <= 1 && i2c_st->len1 == 0)
        {
            i2cn->CR1 |= I2C_CR1_STOP;                  // Посылаем стоп бит
            i2c_st->state = i2c_state_callback;         // Заканчиваем передачу
        }
        else
        {
            i2cn->CR1 |= I2C_CR2_ITBUFEN;               // Разрешаем прерывания по передаче
            i2c_st->byte_counter++;
        }
        return;
    }
    // Буфер передачи пуст, можно записать следующий байт
    if (reg_sr1 & I2C_SR1_TXE)
    {
        // Отправляем содержимое базового массива
        if (i2c_st->byte_counter < i2c_st->len0)
            i2cn->DR = i2c_st->data0[i2c_st->byte_counter];  // Записываем байт в регистр
        // Отправляем содержимое дополнительного массива данных
        else if (i2c_st->byte_counter >= i2c_st->len0 &&
                i2c_st->byte_counter < (i2c_st->len0 + i2c_st->len1))
        {
            i2cn->DR = i2c_st->data1[i2c_st->byte_counter - i2c_st->len0];
        }
        // Содержимое массивов закончилось
        else
        {
            i2cn->CR1 |= I2C_CR1_STOP;                  // Посылаем стоп бит
            i2cn->CR1 &= ~I2C_CR2_ITBUFEN;              // Выключаем прерывание по пустому массиву
            i2c_st->state = i2c_state_callback;         // Заканчиваем передачу
            return;
        }
        i2c_st->byte_counter++;                         // Увеличиваем счётчик
    }
}

/**
 * @brief Обработчик прерывания по событию I2C (для EV)
 * @param [in, out] i2c_st       Структура I2C
 */
static void i2c_event_handler (i2c_t *i2c_st)
{
    if (i2c_st == NULL)
        return;
    /// Считываем SR1 сразу в переменную(SR2 считаем, когда понадобится сбросить флаг ADDR)
    const uint16_t reg_sr1 = i2c_st->instance->SR1;

    // Если у нас ошибка взывалась в момент запуска новой передачи
    if (i2c_st->error_i2c != i2c_err_no)
    {
        // Останавливаем передачу
        i2c_st->instance->CR1 |= I2C_CR1_STOP;
        i2c_st->instance->CR1 &= ~I2C_CR1_PE;           // Отключение I2C
        i2c_st->state = i2c_state_callback;             // Заканчиваем передачу
        return;
    }
    // Если происходит чтение
    if (i2c_st->state == i2c_state_reading)
    {
        i2c_read_handler (i2c_st, reg_sr1);
    }
    // Если происходит запись
    else if (i2c_st->state == i2c_state_writing)
    {
        i2c_write_handler (i2c_st, reg_sr1);
    }
}

// Функции прерываний вызываются через макросс
#define I2C_IRQHandler(IRQ_INST, FUNC_INST, N) void I2C##N##_##IRQ_INST##_IRQHandler() \
{ \
    i2c_##FUNC_INST##_handler (i2c##N##_struct); \
}\

I2C_IRQHandler(EV, event, 1)
I2C_IRQHandler(ER, error, 1)
I2C_IRQHandler(EV, event, 2)
I2C_IRQHandler(ER, error, 2)
I2C_IRQHandler(EV, event, 3)
I2C_IRQHandler(ER, error, 3)

i2c_prog_error_t i2c_read (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data,
              const uint16_t len, void *object, void (*callback_func) (void *))
{
    // Пустой указатель
    if (i2c_st == NULL)
        return prog_i2c_empty_struct;
    // Проверка на отсутствие пустого глобального указателя на структуру(если был deinit)
    if ((i2c_st->instance == I2C1 && i2c1_struct == NULL) ||
            (i2c_st->instance == I2C2 && i2c2_struct == NULL) ||
            (i2c_st->instance == I2C3 && i2c3_struct == NULL))
        return prog_i2c_empty_struct;
    // В случае, если i2c занят
    if (i2c_st->state != i2c_state_idle)
        return prog_i2c_err_i2c_busy;
    // Если нужно считать 0 байт - ошибка
    if (len == 0 || data == NULL)
        return prog_i2c_err_wrong_data;

    i2c_st->error_i2c = i2c_err_no;         // Сбрасываем ошибки
    i2c_st->state = i2c_state_reading;      // Состояние чтения

    i2c_st->dev_addr = device_addr;         // Адрес устройства
    i2c_st->data0 = data;                   // Назначение указателя и размера
    i2c_st->len0 = len;

    i2c_st->ptr_obj = object;
    i2c_st->callback_func = callback_func;
    //Запуск периферии
    i2c_st->instance->CR1 |= I2C_CR1_PE;    // Включаем I2C
    i2c_st->instance->CR1 |= I2C_CR1_START; // Передаём старт
    return prog_i2c_err_no;
}

i2c_prog_error_t i2c_write (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data0,
               const uint16_t len0, uint8_t *data1, const uint16_t len1, void *object,
               void (*callback_func) (void *))
{
    // Пустой указатель
    if (i2c_st == NULL)
        return prog_i2c_empty_struct;
    // Проверка на отсутствие пустого глобального указателя на структуру(если был deinit)
    if ((i2c_st->instance == I2C1 && i2c1_struct == NULL) ||
            (i2c_st->instance == I2C2 && i2c2_struct == NULL) ||
            (i2c_st->instance == I2C3 && i2c3_struct == NULL))
        return prog_i2c_empty_struct;
    // Проверка на отсутствие процессов на I2C
    if (i2c_st->state != i2c_state_idle)
        return prog_i2c_err_i2c_busy;
    // Проверка на наличие правильных данных
    if (len0 == 0 || data0 == NULL || (len1 != 0 && data1 == NULL))
        return prog_i2c_err_wrong_data;
    // Меняем состояние структуры

    i2c_st->error_i2c = i2c_err_no;         // Сбрасываем ошибки
    i2c_st->state = i2c_state_writing;

    i2c_st->dev_addr = device_addr;
    i2c_st->data0 = data0;
    i2c_st->len0 = len0;
    i2c_st->data1 = data1;
    i2c_st->len1 = len1;

    i2c_st->ptr_obj = object;
    i2c_st->callback_func = callback_func;
    //Запуск передачи
    i2c_st->instance->CR1 |= I2C_CR1_PE;    // Включаем I2C
    i2c_st->instance->CR1 |= I2C_CR1_START; // Передаём старт
    return prog_i2c_err_no;
}

i2c_prog_error_t i2c_main_cycle (i2c_t *i2c_st)
{
    if (i2c_st == NULL) // Значения выйдут за пределы памяти
        return prog_i2c_empty_struct;
    // Окончание чтения, запуск колбэка
    if (i2c_st->state == i2c_state_callback)
    {
        i2c_st->state = i2c_state_idle;
        if(i2c_st->callback_func != NULL)
            i2c_st->callback_func (i2c_st->ptr_obj);    // Вызов функции
    }
    return prog_i2c_err_no;
}
