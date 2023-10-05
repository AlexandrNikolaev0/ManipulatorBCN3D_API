# ManipulatorAPI
Python interface for BCN3D Moveo manipulator controlling via Arduino Mega

## Arduino Firmwire 
Firstly, to controll the robot with python API, we have to upload our firmwire to Arduino Mega.

### <a name="48"></a> Работа с Arduino IDE и установка прошивки
1. [Скачайте](https://www.arduino.cc/en/software) установщик среды разработки Arduino IDE и установите.
2. Подключите контроллер Arduino Mega к компьютеру с запущенной средой Arduino IDE и включите его питание.
3. [Скачайте](https://www.arduino.cc/en/software) и установите драйвер CH340 (чтобы учесть все версии Arduino Mega).
4. Откройте диспетчер устройств на компьютере и убедитесь, что контроллер распознаётся компьтером (как показано на одном из примеров): <div align="center"><img width="200" src="imgProgramming/ArduinoMega.png"><img width="200" src="imgProgramming/CH340.png"></div>
5. Откройте Arduino IDE на компьютере и выберете порт устройства (как показано на примере):<div align="center"><img width="500" src="imgProgramming/COM4A.png"></div>
6. Установим библиотеки `AccelStepper` и `MultiStepper`: <div align="center"><img width="500" src="imgProgramming/lib.png"></div> <div align="center"><img width="500" src="imgProgramming/accel.png"></div>
7. Загрузим прошивку `manipulator2.ino` из папки `manipulator2` в контроллер: <div align="center"><img width="500" src="imgProgramming/download.png"></div>

### <a name="49"></a> Пример реализации собственных алгоритмов

Для того, чтобы работать с манипулятором, необходимо его откалибровать, а именно задать все параметры в файле `Manipulator.h`, в соответствие с комментариями. В файла `Titrator.cpp` и `Titrator.h` находится всё, что необходимо для работы с дозатором. Их следует рассмотреть и настроить некоторые параметры в соответствие с изложенными там комментариями. Файл `manipulator2.ino` определяет общий алгоритм движения манипулятора и является примером написанного алгоритма. По комментариям, находящимся в файле, возможно настроить свой алгоритм действий манипулятора, а так же настроить конфигурацию самого манипулятора. Программа может требовать доработки в зависимости от конкретной задачи. По сути, в алгоритме используются функции, которые определены в файле `Manipulator.h`

## Downloading Python API
The next step is to download [Python](https://www.python.org/downloads/) API:
```
git clone https://github.com/AlexandrNikolaev0/ManipulatorBCN3D_API.git
```
Making a virtual enviroment:
```
cd ManipulatorBCN3D_API
python -m venv manAPI_env
cd manAPI_env/Scripts
activate.bat
cd ../..
pip install -r requirements.txt
```

