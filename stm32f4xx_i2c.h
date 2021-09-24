/** @file
 *  @brief Библиотека управления периферией i2c
 *
 *  Пример использования библиотеки:
 *
 *  #include "stm32f4xx_i2c.h"
 *
 *  void next_step(void *struct1)
 *  {...}
 *
 *  int main(void)
 *  {
 *      i2c_t  i2c_struct;
 *      i2c_init(&i2c_struct, 400000, i1pb9pb8);
 *
 *      uint8_t a[5] = {0,1,2,3,4};
 *
 *      i2c_write(&i2c_struct, 0b10100100, a, 5, 0, 0, &struct1, next_step);
 *      while(1)
 *      {
 *         i2c_main_cycle(&i2c_struct);
 *      }
 *  }
 *
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "stm32f405xx.h"
#include "sysclock.h"

/**
 * @brief Варианты распиновки I2C
 */
typedef enum i2c_init_hw_vars_e
{
 //I2C | SDA | SCL
    i1pb7pb6,
    i1pb7pb8,
    i1pb9pb6,
    i1pb9pb8,
    i2pb11pb10,
    i3pc9pa8
} i2c_init_hw_vars_t;

/**
 * @brief Конечный автомат состояний
 */
typedef enum i2c_st_machine_e
{
    i2c_state_idle,         /*!< Состояние ожидания команды                                     */
    i2c_state_reading,      /*!< Состояние чтения данных                                        */
    i2c_state_writing,      /*!< Состояние записи данных                                        */
    i2c_state_callback      /*!< Состояние ожидания вызова колбэка                              */
} i2c_st_machine_t;

/**
 * @brief Хардварные ошибки I2C
 */
typedef enum i2c_error_e
{
    i2c_err_no,             /*!< Нет ошибок                                                     */
    i2c_err_smbalert,       /*!< Ошибка SMB шины                                                */
    i2c_err_timeout,        /*!< SCL на земле более 25мс                                        */
    i2c_err_pecerr,         /*!< NACK в состоянии ресивера                                      */
    i2c_err_ovr,            /*!<                                                                */
    i2c_err_af,             /*!< Пришёл NACK                                                    */
    i2c_err_arlo,           /*!<                                                                */
    i2c_err_berr            /*!< Ошибка шины                                                    */
} i2c_error_t;

/**
 * @brief Программные ошибки
 */
typedef enum i2c_prog_error_e
{
    prog_i2c_err_no,                /*!< Нет ошибок                                             */
    prog_i2c_empty_struct,          /*!< Пустая структура                                       */
    prog_i2c_already_init,          /*!< Структура уже проинициализирована                      */
    prog_i2c_err_init,              /*!< Ошибка инициализации                                   */
    prog_i2c_err_i2c_busy,          /*!< Железо занято                                          */
    prog_i2c_err_wrong_data         /*!< Неправильные денные(указатели, размеры)                */
} i2c_prog_error_t;

/**
 * @brief Структура данных и состояния I2C
 */
typedef struct i2c_struct_e
{
   I2C_TypeDef *instance;           /*!< Экземпляр I2C                                          */

   i2c_st_machine_t state;          /*!< Указатель на структуру более высокого уровня           */
   i2c_error_t error_i2c;           /*!< Состояние ошибок железа                                */
   uint16_t byte_counter;           /*!< Счётчик информации                                     */

   uint8_t dev_addr;                /*!< Адрес устройства                                       */
   uint8_t *data0;                  /*!< Указатель на нулевой массив для записи/чтения          */
   uint16_t len0;                   /*!< Длинна нулевого массива                                */
   uint8_t *data1;                  /*!< Указатель на передаваемую информацию(второй блок)      */
   uint16_t len1;                   /*!< Размер второго массива информации                      */

   void *ptr_obj;                   /*!< Указатель на структуру более высокого уровня           */
   void (*callback_func) (void *);  /*!< Указатель на фунцию, вызываемую по окончанию передачи  */
} i2c_t;


/**
 * @brief Инициализация железа
 * @param [in,out] i2c_st       Структура I2C
 * @param [in] freq             Частота I2C в Гц
 * @param [in] i2c_variant      Вариант распиновки I2C
 * @return Возвращает элемент enum в соответсвии с тем, какая ошибка произошла или её не было
 */
i2c_prog_error_t i2c_init (i2c_t *i2c_st, const uint32_t freq, const i2c_init_hw_vars_t i2c_variant);

/**
 * @brief Деинициализация железа
 * @param [in,out] i2c_st       Структура I2C
 */
i2c_prog_error_t i2c_deinit (i2c_t *i2c_st);

/**
 * @brief Функция получения состояния линии
 * @param [in,out] i2c_st       Структура I2C
 * @return Возвращает состояние переменной ошибок I2C
 */
i2c_error_t i2c_errors_get (i2c_t *i2c_st);

/**
 * @brief Запуск чтения данных от устройства на шине I2C
 * @param [in,out] i2c_st       Структура I2C
 * @param [in] device_addr      Адрес устройства на линии(без бита W/R
 * @param [in] data             Указатель на массив данных(куда записываем)
 * @param [in] len              Длина массива данных
 * @param [in] object           Указатель на структуру уровня выше
 * @param [in] callback_func    Функция, вызываемая по заверщении передачи
 * @return Возвращает элемент enum в соответсвии с тем, какая ошибка произошла или её не было
 */
i2c_prog_error_t i2c_read (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data,
              const uint16_t len, void *object, void (*callback_func) (void *));

/**
 * @brief Запуск передачи данных в устройство на шине I2C
 * @param [in,out] i2c_st       Структура I2C
 * @param [in] device_addr      Адрес устройства на линии(без бита W/R
 * @param [in] data0            Указатель на данные(что передаём)
 * @param [in] len0             Длина массива данных
 * @param [in] data1            Указательна дополнительного массива данных
 * @param [in] len1             Длина дополнительного массива
 * @param [in] object           Указатель на структуру уровня выше
 * @param [in] callback_func    Функция, вызываемая по заверщении передачи
 * @return Возвращает элемент enum в соответсвии с тем, какая ошибка произошла или её не было
 */
i2c_prog_error_t i2c_write (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data0,
               const uint16_t len0, uint8_t *data1, const uint16_t len1, void *object,
               void (*callback_func) (void *));

/**
 * @brief Проверка состояния конечного автомата и запуск колбэка
 * @param [in, out] i2c_st      Структура I2C
 * @return Возвращает элемент enum в соответсвии с тем, какая ошибка произошла или её не было
 */
i2c_prog_error_t i2c_main_cycle (i2c_t *i2c_st);

