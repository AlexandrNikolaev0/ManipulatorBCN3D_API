#pragma once
#ifndef TITRATOR

#include <Arduino.h>
#include <AccelStepper.h>

#define CALIBRATE_PIN       19///< Пин определения нулевого положения дозатора
#define ENABLE_PIN          24///< Пин включения драйвера мотора дозатора
#define D                   19///< Диаметр поршня (мм)
#define MOTOR_TYPE           1///< Тип управления мотором дозатора для AccelStepper
#define MOTOR_STEP_PIN      26///< Пин step драйвера мотора дозатора
#define MOTOR_DIR_PIN       28///< Пин dir драйвера мотора дозатора

#define MAX_SPEED        5000///< Максимальная скорость движения мотора дозатора
#define ACCEL               5///< Максимальное ускорение движения мотора дозатора
#define BAUD_RATE         9600///< Скорость передачи данных в Последовательный порт

class Titrator{

  public:
  Titrator();
  ~Titrator();

  AccelStepper *stepper = new AccelStepper(MOTOR_TYPE, MOTOR_STEP_PIN, MOTOR_DIR_PIN);//Объявляем мотор

  void calibrater();
  void dropLiquid(int volume);
  void pickLiquid(int volume);
  void pickAir(int volume);
  void dropTube();

};

#endif /* end of include guard:  */
