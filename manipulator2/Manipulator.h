#include <AccelStepper.h>
#include <MultiStepper.h>
//#include <exception>
#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <Arduino.h>
#include <GyverStepper.h>

#define STEPPER1_STEP 46 //драйвер z
#define STEPPER1_DIR 48
#define STEPPER1_SPLIT 32.0
#define STEPPER1_MAX_SPEED 470.0*STEPPER1_SPLIT
#define STEPPER1_ACCEL 300.0*STEPPER1_SPLIT
#define STEPS1 200.0*STEPPER1_SPLIT * 10.0
GStepper<STEPPER2WIRE> stepper1(STEPS1, STEPPER1_STEP, STEPPER1_DIR);

#define STEPPER2_STEP 60 //драйвер y
#define STEPPER2_DIR 61
#define STEPPER2_SPLIT 32.0
#define STEPPER2_MAX_SPEED 3500.0 * STEPPER2_SPLIT
#define STEPPER2_ACCEL 100.0 * STEPPER2_SPLIT
#define STEPS2 200.0 * STEPPER2_SPLIT * 5.5
GStepper<STEPPER2WIRE> stepper2(STEPS2, STEPPER2_STEP, STEPPER2_DIR);



#define STEPPER3_STEP 54 //драйвер x
#define STEPPER3_DIR 55
#define STEPPER3_SPLIT 8.0
#define STEPPER3_MAX_SPEED 20000.0 * STEPPER3_SPLIT
#define STEPPER3_ACCEL 750.0 * STEPPER3_SPLIT
#define STEPS3 200.0 * STEPPER3_SPLIT * 5.0 * 61/14
GStepper<STEPPER2WIRE> stepper3(STEPS3, STEPPER3_STEP, STEPPER3_DIR);


#define STEPPER4_STEP 36 //драйвер e1
#define STEPPER4_DIR 34
#define STEPPER4_SPLIT 32.0
#define STEPPER4_MAX_SPEED 1000.0*STEPPER4_SPLIT
#define STEPPER4_ACCEL 250.0*STEPPER4_SPLIT
#define STEPS4 200.0*STEPPER4_SPLIT * 4.5
GStepper<STEPPER2WIRE> stepper4(STEPS4, STEPPER4_STEP, STEPPER4_DIR);





#define E1_STEP_PIN           100///< Пин step драйвера E1
#define E1_DIR_PIN            100///< Пин dir драйвера E1

#define Z_STEP_PIN            100///< Пин step драйвера Z
#define Z_DIR_PIN             100///< Пин dir драйвера Z

#define Y_STEP_PIN            100///< Пин step драйвера Y
#define Y_DIR_PIN             100///< Пин dir драйвера Y

#define X_STEP_PIN            100///< Пин step драйвера X
#define X_DIR_PIN             100///< Пин dir драйвера X

#define MOTOR_TYPE             1///< Тип двигаля

#define JOINT_1_ENDSTOP        3///< Концевой выключатель первого двигателя
#define JOINT_2_ENDSTOP        2///< Концевой выключатель второго двигателя
#define JOINT_3_ENDSTOP       14///< Концевой выключатель третьего двигателя
#define JOINT_4_ENDSTOP       15///< Концевой выключатель четвёртого двигателя

#define JOINT_1_AVERAGE_SPEED  2000/2///< Скорость первого двигателя
#define JOINT_2_AVERAGE_SPEED    1700///< Скорость второго двигателя
#define JOINT_3_AVERAGE_SPEED   12000/2///< Скорость третьего двигателя
#define JOINT_4_AVERAGE_SPEED   1000/2///< Скорость четвёртого двигателя

#define JOINT_1_AVERAGE_ACCEL   10000/2//1000///< Ускорение первого двигателя
#define JOINT_2_AVERAGE_ACCEL   11000//1100///< Ускорение второго двигателя
#define JOINT_3_AVERAGE_ACCEL   40000/2//6000///< Ускорение третьего двигателя
#define JOINT_4_AVERAGE_ACCEL   6000/2//1300///< Ускорение четвёртого двигателя

#define JOINT_1_CALIB_VAL     -STEPS1*0.093  ///< Количесво шагов первого двигателя до нулевого положения от концевого выключателя
#define JOINT_2_CALIB_VAL     -STEPS2*0.29///< Количесво шагов второго двигателя до нулевого положения от концевого выключателя
#define JOINT_3_CALIB_VAL     STEPS3*0.312///< Количесво шагов третьего двигателя до нулевого положения от концевого выключателя
#define JOINT_4_CALIB_VAL     -STEPS4*0.27///< Количесво шагов четвёртого двигателя до нулевого положения от концевого выключателя

#define STEP_1                32400/2///< Количесво шагов первого двигателя на один оборот
#define STEP_2                16800///< Количесво шагов второго двигателя на один оборот
#define STEP_3                70400/2///< Количесво шагов третьего двигателя на один оборот
#define STEP_4                14400/2///< Количесво шагов четвёртого двигателя на один оборот


AccelStepper joint1(MOTOR_TYPE, Z_STEP_PIN,  Z_DIR_PIN);///< Двигатель первого звена
AccelStepper joint2(MOTOR_TYPE, Y_STEP_PIN,  Y_DIR_PIN);///< Двигатель второго звена
AccelStepper joint3(MOTOR_TYPE, X_STEP_PIN, X_DIR_PIN);///< Двигатель третьеого звена
AccelStepper joint4(MOTOR_TYPE, E1_STEP_PIN, E1_DIR_PIN);///< Двигатель четвёртого звена

MultiStepper steppers;///< Двигатели звеньев
AccelStepper steppersMulti[4];







long req_step[4];///< Желаемые углы поворота звеньев в шагах двигателей
long joints_control[4];

float desired_angle[4] = {0.0, 0.0, 0.0, 0.0}; ///< Желаемые углы поворота звеньев в радианах

///< параметры кинематики (длины звеньев)
const float l_12 = 0.2325;
const float l_3 = 0.227;
const float l_4 = 0.1645;
const float l_5r = 0.0985;
const float l_5h = 0.180;
//const float l_5rc = 0.126;//0.0985;
//const float l_5hc = 0.092;//0.180;

double x_cur = 0;
double y_cur = 0;
double z_cur = 0;

float xy_transf[2] = {0, 0};

const float limits[4][2] {
  { -50 * M_PI / 180 , M_PI / 2 }, { -100 * M_PI / 180, 100 * M_PI / 180},
  { -105 * M_PI / 180, 105 * M_PI / 180}, { -M_PI / 2, M_PI / 2}
};///< Пределы углов звеньев в радианах

int dir1 = -JOINT_1_CALIB_VAL > 0 ? 1 : -1;
int dir2 = -JOINT_2_CALIB_VAL > 0 ? 1 : -1;
int dir3 = JOINT_3_CALIB_VAL > 0 ? 1 : -1;
int dir4 = -JOINT_4_CALIB_VAL > 0 ? 1 : -1;


///Расчёт шагов звеньев
/*!
   Переводит углы в радианах в углы в шагах двигателей.
*/
void calc_req_step() {
  long stepsPerRevolution[4] = {(long)STEPS1, (long)STEPS2, (long)STEPS3, (long)STEPS4};
  req_step[0] = -int(desired_angle[0] * stepsPerRevolution[0] / (2 * M_PI)) * (JOINT_1_CALIB_VAL > 0 ? 1 : -1);
  req_step[1] = int(desired_angle[1] * stepsPerRevolution[1] / (2 * M_PI)) * (JOINT_2_CALIB_VAL > 0 ? 1 : -1);
  req_step[2] = -int(desired_angle[2] * stepsPerRevolution[2] / (2 * M_PI)) * (JOINT_3_CALIB_VAL > 0 ? 1 : -1);
  req_step[3] = int((desired_angle[3]) * stepsPerRevolution[3] / (2 * M_PI)) * (JOINT_4_CALIB_VAL > 0 ? 1 : -1);
}

///Функция калибровки звена
/*!
   Калибрует одно звено на заданный угол от концевого выключателя.
   \param joint Двигатель звена.
   \param steps Количество шагов от концевого выключателя до нулевого положения.
   \param endstop Концевой выключатель звена.
   \param av_speed Скорость двигателя.
   \param av_acc Ускорение двигателя.
*/
void calibrateJoint(AccelStepper joint, int steps, int endstop, int av_speed, int av_acc) {
  joint.setMaxSpeed(av_speed);
  joint.setAcceleration(av_acc);
  int dir = steps > 0 ? 1 : -1;
  joint.setSpeed(-dir * av_speed / 3);
  while (digitalRead(endstop)) joint.runSpeed();
  joint.stop();
  joint.setCurrentPosition(0);
  joint.setSpeed(-dir * av_speed);
  joint.moveTo(int(steps * 1.05));
  joint.runToPosition();
  joint.moveTo(steps);
  joint.runToPosition();
  joint.setCurrentPosition(0);
}

///Функция полной калибровки манипулятора (все звенья)
void fullCalibrateJoints() {

  //Для движения сочленений к концевику
  // установка макс. скорости в шагах/сек
  stepper1.setMaxSpeed(STEPPER1_MAX_SPEED);
  stepper2.setMaxSpeed(STEPPER2_MAX_SPEED);
  stepper3.setMaxSpeed(STEPPER3_MAX_SPEED);
  stepper4.setMaxSpeed(STEPPER4_MAX_SPEED);

  // установка ускорения в шагах/сек/сек
  stepper1.setAcceleration(STEPPER1_ACCEL);
  stepper2.setAcceleration(STEPPER2_ACCEL);
  stepper3.setAcceleration(STEPPER3_ACCEL);
  stepper4.setAcceleration(STEPPER4_ACCEL);


  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setSpeed(-dir1 * STEPPER1_MAX_SPEED / 1.5);

  while (digitalRead(JOINT_1_ENDSTOP))
  {
    stepper1.tick();
  }

  stepper1.brake();

  stepper2.setRunMode(KEEP_SPEED);

  stepper2.setSpeed(-dir2 * STEPPER2_MAX_SPEED / 80);
  stepper3.setRunMode(KEEP_SPEED);
  stepper3.setSpeed(dir3 * STEPPER3_MAX_SPEED / 40);
  stepper4.setRunMode(KEEP_SPEED);
  stepper4.setSpeed(-dir4 * STEPPER4_MAX_SPEED / 40);

  while (digitalRead(JOINT_2_ENDSTOP) || digitalRead(JOINT_3_ENDSTOP) || digitalRead(JOINT_4_ENDSTOP))
  {
    if (digitalRead(JOINT_2_ENDSTOP))
    {

      stepper2.tick();
    }
    else
    {
      stepper2.brake();
    }

    if (digitalRead(JOINT_3_ENDSTOP))
    {
      stepper3.tick();
    }
    else
    {
      stepper3.brake();
    }

    if (digitalRead(JOINT_4_ENDSTOP))
    {
      stepper4.tick();
    }
    else
    {
      stepper4.brake();
    }
  }


  stepper2.brake();
  stepper3.brake();
  stepper4.brake();

  stepper1.setCurrent(int(JOINT_1_CALIB_VAL));
  stepper2.setCurrent(int(JOINT_2_CALIB_VAL));
  stepper3.setCurrent(int(JOINT_3_CALIB_VAL));
  stepper4.setCurrent(int(JOINT_4_CALIB_VAL));


  long positions[4];

  positions[0] = int(JOINT_1_CALIB_VAL * 0.95);
  positions[1] = int(JOINT_2_CALIB_VAL * 0.95);
  positions[2] = int(JOINT_3_CALIB_VAL * 0.95);
  positions[3] = int(JOINT_4_CALIB_VAL * 0.95);

  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);
  stepper3.setRunMode(FOLLOW_POS);
  stepper4.setRunMode(FOLLOW_POS);

  stepper1.setTarget(positions[0]);
  stepper2.setTarget(positions[1]);
  stepper3.setTarget(positions[2]);
  stepper4.setTarget(positions[3]);

  while (stepper1.tick() | stepper2.tick() | stepper3.tick() | stepper4.tick())
  {
    delayMicroseconds(1);
  }



  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setSpeed(-dir1 * STEPPER1_MAX_SPEED * 0.2);

  while (digitalRead(JOINT_1_ENDSTOP))
  {
    stepper1.tick();
  }

  stepper1.brake();

  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setSpeed(-dir2 * STEPPER2_MAX_SPEED * 0.2);
  stepper3.setRunMode(KEEP_SPEED);
  stepper3.setSpeed(dir3 * STEPPER3_MAX_SPEED * 0.2);
  stepper4.setRunMode(KEEP_SPEED);
  stepper4.setSpeed(-dir4 * STEPPER4_MAX_SPEED * 0.2);


  while (digitalRead(JOINT_2_ENDSTOP) || digitalRead(JOINT_3_ENDSTOP) || digitalRead(JOINT_4_ENDSTOP))
  {
    if (digitalRead(JOINT_2_ENDSTOP))
    {

      stepper2.tick();
    }
    else
    {
      stepper2.brake();
    }

    if (digitalRead(JOINT_3_ENDSTOP))
    {
      stepper3.tick();
    }
    else
    {
      stepper3.brake();
    }

    if (digitalRead(JOINT_4_ENDSTOP))
    {
      stepper4.tick();
    }
    else
    {
      stepper4.brake();
    }
  }


  stepper2.brake();
  stepper3.brake();
  stepper4.brake();


  stepper2.setCurrent(int(JOINT_2_CALIB_VAL));
  stepper3.setCurrent(int(JOINT_3_CALIB_VAL));
  stepper4.setCurrent(int(JOINT_4_CALIB_VAL));


  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;


  stepper2.setRunMode(FOLLOW_POS);
  stepper3.setRunMode(FOLLOW_POS);
  stepper4.setRunMode(FOLLOW_POS);


  stepper2.setTarget(positions[1]);
  stepper3.setTarget(positions[2]);
  stepper4.setTarget(positions[3]);

  while (stepper2.tick() | stepper3.tick() | stepper4.tick())
  {
    delayMicroseconds(1);
  }


  stepper1.setCurrent(int(JOINT_1_CALIB_VAL));
  positions[0] = 0;
  stepper1.setRunMode(FOLLOW_POS);
  stepper1.setTarget(positions[0]);
  while (stepper1.tick()) {
    delayMicroseconds(1);
  }
  x_cur = 0;
  y_cur = 0;
  z_cur = 0;
}



///Функция полной калибровки манипулятора без возвращения в стартовое положение
void finishCalibrateJoints() {
  //Для движения сочленений к концевику
  // установка макс. скорости в шагах/сек
  stepper1.setMaxSpeed(STEPPER1_MAX_SPEED);
  stepper2.setMaxSpeed(STEPPER2_MAX_SPEED);
  stepper3.setMaxSpeed(STEPPER3_MAX_SPEED);
  stepper4.setMaxSpeed(STEPPER4_MAX_SPEED);

  // установка ускорения в шагах/сек/сек
  stepper1.setAcceleration(STEPPER1_ACCEL);
  stepper2.setAcceleration(STEPPER2_ACCEL);
  stepper3.setAcceleration(STEPPER3_ACCEL);
  stepper4.setAcceleration(STEPPER4_ACCEL);


  stepper1.setRunMode(KEEP_SPEED);
  stepper1.setSpeed(-dir1 * STEPPER1_MAX_SPEED / 1.5);

  while (digitalRead(JOINT_1_ENDSTOP))
  {
    stepper1.tick();
  }

  stepper1.brake();

  stepper2.setRunMode(KEEP_SPEED);
  stepper2.setSpeed(-dir2 * STEPPER2_MAX_SPEED / 80);
  stepper3.setRunMode(KEEP_SPEED);
  stepper3.setSpeed(dir3 * STEPPER3_MAX_SPEED / 40);
  stepper4.setRunMode(KEEP_SPEED);
  stepper4.setSpeed(-dir4 * STEPPER4_MAX_SPEED / 40);


  while (digitalRead(JOINT_2_ENDSTOP) || digitalRead(JOINT_3_ENDSTOP) || digitalRead(JOINT_4_ENDSTOP))
  {
    if (digitalRead(JOINT_2_ENDSTOP))
    {

      stepper2.tick();
    }
    else
    {
      stepper2.brake();
    }

    if (digitalRead(JOINT_3_ENDSTOP))
    {
      stepper3.tick();
    }
    else
    {
      stepper3.brake();
    }

    if (digitalRead(JOINT_4_ENDSTOP))
    {
      stepper4.tick();
    }
    else
    {
      stepper4.brake();
    }
  }


  stepper2.brake();
  stepper3.brake();
  stepper4.brake();

  stepper1.setCurrent(int(JOINT_1_CALIB_VAL));
  stepper2.setCurrent(int(JOINT_2_CALIB_VAL));
  stepper3.setCurrent(int(JOINT_3_CALIB_VAL));
  stepper4.setCurrent(int(JOINT_4_CALIB_VAL));


  long positions[4];

  positions[3] = int(-0.25 * STEPS4);

  stepper4.setRunMode(FOLLOW_POS);

  stepper4.setTarget(positions[3]);

  while (stepper4.tick())
  {
    delayMicroseconds(1);
  }
}





//Решение прямой задачи кинематики (получение координат инструмента по углам сочленений)
bool forward_kinematics()
{
  x_cur = ( l_3 * sin(desired_angle[1]) + l_4 * sin(desired_angle[1] + desired_angle[2]) + l_5r * sin(desired_angle[1] + desired_angle[2] + desired_angle[3]) + l_5h * sin(desired_angle[1] + desired_angle[2] + desired_angle[3] + M_PI / 2)) * cos(desired_angle[0]);
  y_cur = ( l_3 * sin(desired_angle[1]) + l_4 * sin(desired_angle[1] + desired_angle[2]) + l_5r * sin(desired_angle[1] + desired_angle[2] + desired_angle[3]) + l_5h * sin(desired_angle[1] + desired_angle[2] + desired_angle[3] + M_PI / 2)) * sin(desired_angle[0]);
  z_cur = l_12 + l_3 * cos(desired_angle[1]) + l_4 * cos(desired_angle[1] + desired_angle[2]) + l_5r * cos(desired_angle[1] + desired_angle[2] + desired_angle[3]) + l_5h * cos(desired_angle[1] + desired_angle[2] + desired_angle[3] + M_PI / 2);

  /*
    Serial.print("d1: ");
    Serial.print(desired_angle[0]);
    Serial.print(" d2: ");
    Serial.print(desired_angle[1]);
    Serial.print(" d3: ");
    Serial.println(desired_angle[2]);
    Serial.print(" d4: ");
    Serial.println(desired_angle[3]);

    Serial.print("c_x: ");
    Serial.print(x_cur);
    Serial.print(" c_y: ");
    Serial.print(y_cur);
    Serial.print(" c_z: ");
    Serial.println(z_cur);
  */
  return 1;
}

///Обратная задача кинематики
/*!
   Рассчитывает углы поворота каждого звена для достижения точки в Декартовой системе координат.
   \param x,y,z Коодинаты точки в Декартовой системе координат.
   \return True - точка достежима, False - нет.
*/

bool inverse_kinematics(float x, float y, float z) {

  float cos1 = (x / (sqrt(pow(x, 2) + pow(y, 2))));
  /*Serial.print("cos1:");
    Serial.print(cos1);
    Serial.println(";   ");*/
  float cos2 = (pow(l_3, 2) + pow(sqrt(pow(x, 2) + pow(y, 2)) - l_5r, 2) + pow(z - l_12 + l_5h, 2) - pow(l_4, 2)) / (2 * l_3 * sqrt(pow(sqrt( pow(x, 2) + pow(y, 2)) - l_5r, 2) + pow(z - l_12 + l_5h, 2)));
  /*Serial.print("cos2:");
    Serial.print(cos2);
    Serial.println(";   ");*/
  float cos3 = (pow(l_3, 2) + pow(l_4, 2) - pow(sqrt(pow(x, 2) + pow(y, 2)) - l_5r, 2) - pow(z - l_12 + l_5h, 2)) / (2 * l_3 * l_4);
  /*Serial.print("cos3:");
    Serial.print(cos3);
    Serial.println(";   ");*/
  if (abs(cos1) <= 1 && abs(cos2) <= 1 && abs(cos3) <= 1) {
    desired_angle[0] = acos(cos1) * abs(y) / (y + 0.0000001);
    /*Serial.print("a0:");
      Serial.print(desired_angle[0] * 180 / M_PI);
      Serial.println(";   ");*/
    desired_angle[1] = atan((sqrt(pow(x, 2) + pow(y, 2)) - l_5r) / (z - l_12 + l_5h)) - acos(cos2);
    /*Serial.print("a1:");
      Serial.print(desired_angle[1] * 180 / M_PI);
      Serial.println(";   ");*/
    desired_angle[2] = (M_PI - acos(cos3));
    /*Serial.print("a2:");
      Serial.print(desired_angle[2] * 180 / M_PI);
      Serial.println(";   ");*/
    desired_angle[3] = acos(cos2) - atan((sqrt(pow(x, 2) + pow(y, 2)) - l_5r) / (z - l_12 + l_5h)) - M_PI / 2 + acos(cos3);
    /*Serial.print("a3:");
      Serial.print(desired_angle[3] * 180 / M_PI);
      Serial.println(";   ");*/
    for (int i = 0; i < 4; i++) {
      if ((desired_angle[i] < limits[i][0]) || (desired_angle[i] > limits[i][1])) {
        return 0;
      }
    }

    return 1;
  }

  return 0;
}


//Функция перехода из координат камеры в глобальные координаты манипулятора
void cam_to_global_transform(float coord[6], float x_2s, float y_2s)
{
  forward_kinematics();

  float delta_alpha = coord[5] - desired_angle[0] + M_PI / 2 - 0.01; //
  float x_marker = x_2s;
  float y_marker = y_2s;

  float x_s = - x_marker * cos(coord[5]) + y_marker * sin(coord[5]);
  float y0_s = - x_marker * sin(coord[5]) - y_marker * cos(coord[5]);

  float deltaMarker_x = x_s * sin(desired_angle[0]) + y0_s * cos(desired_angle[0]);
  float deltaMarker_y = y0_s * sin(desired_angle[0]) - x_s * cos(desired_angle[0]);

  float delta_x = x_cur + deltaMarker_x; //+ (-l_5rc + l_5r)*cos(desired_angle[0]);
  float delta_y = y_cur + deltaMarker_y; //+ (-l_5rc + l_5r)*sin(desired_angle[0]);

  float x_glob = delta_x + coord[1] * sin(delta_alpha) + coord[0] * cos(delta_alpha);
  float y_glob = delta_y + coord[1] * cos(delta_alpha) - coord[0] * sin(delta_alpha);
  xy_transf[0] = x_glob;
  xy_transf[1] = y_glob;
}

//Функция перехода из координат камеры в глобальные координаты манипулятора
void cam_to_global_transform2(float coord[6])
{
  forward_kinematics();
  float delta_x = 0.42;
  float delta_y = 0.085;
  float delta_alpha = M_PI / 2;
  float x_marker = coord[0];
  float y_marker = coord[1];
  float x_glob = delta_x + y_marker * sin(delta_alpha) + x_marker * cos(delta_alpha);
  float y_glob = delta_y + y_marker * cos(delta_alpha) - x_marker * sin(delta_alpha);
  xy_transf[0] = x_glob;
  xy_transf[1] = y_glob;
}

///Обратная задача кинематики
/*!
   Рассчитывает углы поворота каждого звена для достижения точки в Декартовой системе координат.
   \param x,y,z Коодинаты точки в Декартовой системе координат.
   \return True - точка достежима, False - нет.
*/
/*
  bool inverse_kinematics(float x, float y, float z) {

  float alpha = M_PI / 2 * 0.95;//*0.954
  float r1 = sqrt(x * x + y * y);
  float r3 = sqrt(x * x + y * y + (z - d1) * (z - d1));

  if (r3 > a2 + a3)
    return 0;

  if (x == 0) {
    if (y == 0) {
      desired_angle[0] = 0;
    } else if (y > 0) {
      desired_angle[0] = M_PI / 2;
    } else {
      desired_angle[0] = -M_PI / 2;
    }
  } else if (y == 0) {
    if (x > 0) {
      desired_angle[0] = 0;
    } else {
      desired_angle[0] = M_PI;
    }
  } else {
    desired_angle[0] = atan2(y, x);
  }

  if (z - d1 == 0) {
    desired_angle[1] = M_PI / 2 - acos((a2 * a2 + r3 * r3 - a3 * a3) / (2 * a2 * r3));
  } else if (z - d1 > 0) {
    desired_angle[1] = atan(r1 / (z - d1)) - acos((a2 * a2 + r3 * r3 - a3 * a3) / (2 * a2 * r3));
  } else {
    desired_angle[1] = M_PI + atan(r1 / (z - d1)) - acos((a2 * a2 + r3 * r3 - a3 * a3) / (2 * a2 * r3));
  }

  desired_angle[2] = M_PI - acos((a2 * a2 + a3 * a3 - r3 * r3) / (2 * a2 * a3));
  desired_angle[3] = alpha - desired_angle[1] - desired_angle[2];

  for (int i = 0; i < 4; i++) {
    if ((desired_angle[i] < limits[i][0]) || (desired_angle[i] > limits[i][1])) {
      return 0;
    }
    return 1;
  }
  }*/


///Обратная задача кинематики с корректировкой последней оси
/*!
   Рассчитывает углы поворота каждого звена для достижения точки в Декартовой системе координат.
   \param x,y,z Коодинаты точки в Декартовой системе координат.
   \param kAngle Угол поворота последней оси в радианах.
   \return True - точка достежима, False - нет.
*/
bool inverse_kinematicsSpec(float x, float y, float z, float kAngle) {


  return 1;

}

///Функция перемещения в точку
/*!
   Перемещает манипулятор в точку в Декартовой системе координат, связанной с его основанием
   \param x,y,z Коодинаты точки в Декартовой системе координат.
   \return True - точка достежима, False - нет.
*/
bool moveToXYZ(float x, float y, float z) {
  long positions[4];
  if (inverse_kinematics(x, y, z)) {
    calc_req_step();

    positions[0] = req_step[0];
    positions[1] = req_step[1];
    positions[2] = req_step[2];
    positions[3] = req_step[3];
    float speeds[4] = {(float)STEPPER1_MAX_SPEED, (float)STEPPER2_MAX_SPEED, (float)STEPPER3_MAX_SPEED, (float)STEPPER4_MAX_SPEED};
    if (abs(positions[0]) < (float)STEPS1 / 16)
    {
      speeds[0] = (int)(((float)STEPPER1_MAX_SPEED) / 2);
    }

    if (abs(positions[1]) < (float)STEPS2 / 4)
    {
      speeds[1] = (int)(((float)STEPPER2_MAX_SPEED) / 4);
    }

    if (abs(positions[2]) < (float)STEPS3 / 4)
    {
      speeds[2] = (int)(((float)STEPPER3_MAX_SPEED) / 4);
    }

    if (abs(positions[3]) < (float)STEPS4 / 8)
    {
      speeds[3] = (int)(((float)STEPPER4_MAX_SPEED) / 4);
    }


    stepper1.setMaxSpeed(speeds[0]);
    stepper2.setMaxSpeed(speeds[1]);
    stepper3.setMaxSpeed(speeds[2]);
    stepper4.setMaxSpeed(speeds[3]);


    stepper1.setRunMode(FOLLOW_POS);
    stepper2.setRunMode(FOLLOW_POS);
    stepper3.setRunMode(FOLLOW_POS);
    stepper4.setRunMode(FOLLOW_POS);

    stepper1.setTarget(positions[0]);
    stepper2.setTarget(positions[1]);
    stepper3.setTarget(positions[2]);
    stepper4.setTarget(positions[3]);

    while (stepper1.tick() | stepper2.tick() | stepper3.tick() | stepper4.tick())
    {
      delayMicroseconds(1);
    }
    forward_kinematics();

    return 1;

  }
  return 0;
}

///Функция перемещения в точку
/*!
   Перемещает манипулятор в точку в Декартовой системе координат вертикально
   \param z Коодината Z точки в Декартовой системе координат.
   \return True - точка достежима, False - нет.
*/
bool moveZ(float delta_Z) {
  stepper1.setAcceleration(STEPPER1_ACCEL * 1.5);
  stepper2.setAcceleration(STEPPER2_ACCEL * 1.5);
  stepper3.setAcceleration(STEPPER3_ACCEL * 1.5);
  stepper4.setAcceleration(STEPPER4_ACCEL * 1.5);
  forward_kinematics();
  float start_z = z_cur;
  float finish_z = z_cur + delta_Z;
  float finish_x = x_cur;
  float finish_y = y_cur;
  float moveSpeed = 0.03;
  int num = 20;
  float zPoint[num + 1];
  float aPoints[num + 1][4];
  float aspeedPoints[num][4];
  float planTime = abs(delta_Z) / moveSpeed; //сек
  float dT = planTime / (float)num;
  int success = 1;
  float kp1 = 1;
  float kp2 = 1;
  float kp3 = 1;
  float kp4 = 1;

  for (int i = 0; i <= num; i++)
  {

    if (inverse_kinematics(finish_x, finish_y, start_z + (i)*delta_Z / (float)num)) {
      calc_req_step();

      aPoints[i][0] = req_step[0];
      aPoints[i][1] = req_step[1];
      aPoints[i][2] = req_step[2];
      aPoints[i][3] = req_step[3];
      /*
        Serial.print("a1:");
        Serial.print(aPoints[i][0]);
        Serial.print(", a2:");
        Serial.print(aPoints[i][1]);
        Serial.print(", a3:");
        Serial.print(aPoints[i][2]);
        Serial.print(", a4:");
        Serial.println(aPoints[i][3]);
      */
      if (i > 0)
      {
        for (int j = 0; j < 4; j++)
        {
          aspeedPoints[i - 1][j] = (aPoints[i][j]  - aPoints[i - 1][j]) / dT;
        }

        /*Serial.print("s1:");
          Serial.print(aspeedPoints[i - 1][0]);
          Serial.print(", s2:");
          Serial.print(aspeedPoints[i - 1][1]);
          Serial.print(", s3:");
          Serial.print(aspeedPoints[i - 1][2]);
          Serial.print(", s4:");
          Serial.println(aspeedPoints[i - 1][3]);*/
      }
    }
    else
    {
      success *= 0;
    }

  }

  float startTime = millis();
  float curTime = millis();
  float deltaTime =  curTime - startTime;
  int i = 0;
  int iPrev = 0;
  if (success == 1)
  {

    while (deltaTime < planTime * 1000)
    {
      curTime = millis();
      deltaTime = curTime - startTime;
      i = (int)(deltaTime / (dT * 1000));
      if (i != iPrev) {
        stepper1.setRunMode(KEEP_SPEED);
        stepper1.setSpeed(aspeedPoints[i][0] + ((aPoints[i][0] - stepper1.getCurrent())*kp1) );
        stepper2.setRunMode(KEEP_SPEED);
        stepper2.setSpeed(aspeedPoints[i][1] + ((aPoints[i][1] - stepper2.getCurrent())*kp2) );
        stepper3.setRunMode(KEEP_SPEED);
        stepper3.setSpeed(aspeedPoints[i][2] + ((aPoints[i][2] - stepper3.getCurrent())*kp3) );
        stepper4.setRunMode(KEEP_SPEED);
        stepper4.setSpeed(aspeedPoints[i][3] + ((aPoints[i][3] - stepper4.getCurrent())*kp4) );
      }

      stepper1.tick();
      stepper2.tick();
      stepper3.tick();
      stepper4.tick();

      iPrev = i;



    }
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    stepper4.stop();
  }
  /*
    for (int i = 1; i <= num; i++)
    {
    success *= (int)moveToXYZ(x_cur, y_cur, start_z + quant * (float)i);
    }
    success *= (int)moveToXYZ(x_cur, y_cur, start_z + quant * num);
    stepper1.setAcceleration(STEPPER1_ACCEL);
    stepper2.setAcceleration(STEPPER2_ACCEL);
    stepper3.setAcceleration(STEPPER3_ACCEL);
    stepper4.setAcceleration(STEPPER4_ACCEL);
  */
  success *= (int)moveToXYZ(finish_x, finish_y, finish_z);
  return success;

}


///Функция перемещения в точку
/*!
   П-Регулятор перемещения за маркером
*/
bool marker_tracking(float *camCoord, float *camDesiredCoord)
{
  cam_to_global_transform(camCoord, camCoord[0], camCoord[1]);
  float xy_glob[2];
  xy_glob[0] = xy_transf[0];
  xy_glob[1] = xy_transf[1];
  camDesiredCoord[5] = camCoord[5];
  cam_to_global_transform(camDesiredCoord, camCoord[0], camCoord[1]);
  float xy_desired[2];
  xy_desired[0] = xy_transf[0];
  xy_desired[1] = xy_transf[1];
  float err_x = xy_desired[0] - xy_glob[0];
  float err_y = xy_desired[1] - xy_glob[1];
  float err_z = camDesiredCoord[2] - camCoord[2] - 0.106;
  float err_xy = sqrt(err_x * err_x + err_y * err_y);
  float kx = 0.45;
  float ky = 0.45;
  float kz = 0.35;
  if (err_xy > 0.05)
  {
    kx = 0.85;
    ky = 0.85;
    kz = 0.6;
  }
  else
  {
    if (err_z > 0.01)
    {
      kz = 0.95;
    }
  }

  float u_x = kx * err_x;
  float u_y = ky * err_y;
  float u_z = kz * err_z;
  float u = sqrt(u_x * u_x + u_y * u_y + u_z * u_z);
  if (u < 0.0001)
  {
    return 1;
  }
  /*
    Serial.print("u_x: ");
    Serial.print(u_x);
    Serial.print(" u_y: ");
    Serial.print(u_y);
    Serial.print(" u_z: ");
    Serial.println(u_z);
  */
  bool statusMove = moveToXYZ(x_cur + u_x, y_cur + u_y, z_cur + u_z);
  return statusMove;
}



///Функция перемещения в точку с корректировкой последней оси
/*!
   Перемещает манипулятор в точку в Декартовой системе координат, связанной с его основанием
   \param x,y,z Коодинаты точки в Декартовой системе координат.
   \param kAngle Угол поворота последней оси в радианах.
   \return True - точка достежима, False - нет.
*/
bool moveToXYZSpec(float x, float y, float z) { //, float kAngle) {
  long positions[4];
  joint1.setMaxSpeed(JOINT_1_AVERAGE_SPEED / 3);
  joint2.setMaxSpeed(JOINT_2_AVERAGE_SPEED / 3);
  joint3.setMaxSpeed(JOINT_3_AVERAGE_SPEED / 3);
  joint4.setMaxSpeed(JOINT_4_AVERAGE_SPEED / 3);
  joint1.setSpeed(-dir1 * JOINT_1_AVERAGE_SPEED / 3);
  joint2.setSpeed(-dir2 * JOINT_2_AVERAGE_SPEED / 3);
  joint3.setSpeed(-dir3 * JOINT_3_AVERAGE_SPEED / 3);
  joint4.setSpeed(-dir4 * JOINT_4_AVERAGE_SPEED / 3);

  if (inverse_kinematics(x, y, z)) { //(inverse_kinematicsSpec(x, y, z, kAngle)) {
    calc_req_step();

    positions[0] = req_step[0];
    positions[1] = req_step[1];
    positions[2] = req_step[2];
    positions[3] = req_step[3] - int(1 / z * 6);

    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    return 1;
  }
  return 0;
}
///Функция перемещения вдоль вертикального отрезка сверху вниз
/*!
   Позиционирует манипулятор в точке Декартовой системы координат, связанной с его основанием и перемещает вдоль вертикали сверху вниз
   \param x,y Коодинаты вертикального отрезка в Декартовой системе координат (м).
   \param startZ Начальная величина координаты z (см).
   \param finishZ Конечная величина координаты z (см).
*/
void goInTube(float x, float y, float startZ, float finishZ)
{

  float z = startZ;
  while (z >= finishZ)
  {
    moveToXYZSpec(x, y, z / 100);
    z -= 0.25;
  }
  moveToXYZSpec(x, y, finishZ / 100);
  delay(100);
}

///Функция перемещения вдоль вертикального отрезка снизу вверх
/*!
   Позиционирует манипулятор в точке Декартовой системы координат, связанной с его основанием и перемещает вдоль вертикали снизу вверх
   \param x,y Коодинаты вертикального отрезка в Декартовой системе координат (м).
   \param startZ Начальная величина координаты z (см).
   \param finishZ Конечная величина координаты z (см).
*/
void goOutTube(float x, float y, float startZ, float finishZ)
{
  float z = startZ;
  while (z <= finishZ)
  {
    moveToXYZSpec(x, y, z / 100);
    z += 0.25;
  }
  moveToXYZSpec(x, y, finishZ / 100);
  delay(100);
}


#endif
