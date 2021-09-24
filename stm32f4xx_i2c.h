#pragma once


#include "stm32f405xx.h"
#include "sysclock.h"
#include <stdbool.h>
#include <stddef.h>

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
    i2c_idle,               /// Состояние ожидания команды
    i2c_reading,            /// Состояние чтания данных
    i2c_writing,            /// Состояние записи данных
    i2c_callback            /// Состояние ожидания вызова колбэка
} i2c_st_machine_t;

/**
 * @brief Хардварные ошибки I2C
 */
typedef enum i2c_error_e
{
    err_no,                 /// Нет ошибок
    err_smbalert,           /// Ошибка SMB шины
    err_timeout,            /// SCL на земле более 25мс
    err_pecerr,             /// NACK в состоянии ресивера
    err_ovr,                ///
    err_af,                 /// Пришёл NACK
    err_arlo,               ///
    err_berr                /// Ошибка шины
} i2c_error_t;

/**
 * @brief Программные ошибки
 */
typedef enum i2c_prog_error_e
{
    prog_i2c_err_no,                /// Нет ошибок
    prog_i2c_empty_struct,          /// Пустая структура
    prog_i2c_already_init,          /// Структура уже проинициализироана
    prog_i2c_err_init,              /// Ошибка инициализации
    prog_i2c_err_i2c_busy,          /// Железо занято
    prog_i2c_err_wrong_data         /// Неправильные денные(указатели, размеры)
} i2c_prog_error_t;

/**
 * @brief Структура данных и состояния I2C
 */
typedef struct i2c_struct_e
{
   I2C_TypeDef *instance;           /// Экземпляр I2C

   i2c_st_machine_t state;
   i2c_error_t error_i2c;           /// Состояние ошибок железа
   i2c_prog_error_t prog_error_i2c; /// Состояние программных ошибок
   uint8_t byte_counter;            /// Счётчик информации

   uint8_t dev_addr;                /// Адрес устройства
   uint8_t *data0;                  /// Указатель на нулевой массив для записи/чтения
   uint16_t len0;                   /// Длинна нулевого массива
   uint8_t *data1;                  /// Указатель на передаваемую информацию(второй блок)
   uint16_t len1;                   /// Размер второго массива информации

   void (*callback_func) (void *);   /// Указатель на фунцию, вызываемую по окончанию передачи

} i2c_t;


/**
 * @brief Инициализация железа
 * @param [in,out] i2c_st   Структура управления I2C
 * @param [in] freq              Частота I2C в Гц
 * @param [in] i2c_variant       Вариант распиновки I2C
 * @return false если что-то пошло не так и true если всё ок
 */
i2c_prog_error_t i2c_init (i2c_t *i2c_st, uint32_t freq, i2c_init_hw_vars_t i2c_variant);

/**
 * @brief Запуск чтения из I2C
 * @param [in,out] i2c_st        Экземпляр I2C
 * @param [in] device_addr       Адрес устройства на линии(без бита W/R
 * @param [in] data0             Указатель на массив данных(куда записываем)
 * @param [in] len0              Длина массива данных
 * @param [in] callback_func     Функция вызываемая по заверщению передачи
 * @return \true если чтение запущено, false при ошибке
 */
i2c_prog_error_t i2c_read (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data,
              const uint16_t len, void (*callback_func)(void *));
/**
 * @brief Запуск записи из I2C
 * @param [in,out] i2c_st       Экземпляр I2C
 * @param [in] device_addr      Адрес устройства на линии(без бита W/R
 * @param [in] data0            Указатель на данные(что передаём)
 * @param [in] len0             Длина массива данных
 * @param [in] data1            Указательна дополнительного массива данных
 * @param [in] len1             Длина дополнительного массива
 * @param [in] callback_funk    Функция вызываемая по заверщению передачи
 * @return true если чтение запущено, false при ошибке
 */
i2c_prog_error_t i2c_write (i2c_t *i2c_st, const uint8_t device_addr, uint8_t *data0,
               const uint16_t len0, uint8_t *data1, const uint16_t len1,
               void (*callback_func)(void *));

/**
 * @brief Проверка состояния конечного автомата и запуск колбэка
 * @param [in,out] i2c_st       Структура I2C
 */
i2c_prog_error_t i2c_main_cycle(i2c_t *i2c_st, void *some_data);

