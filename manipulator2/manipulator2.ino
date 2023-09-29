  
#include "Manipulator.h"
#include "Titrator.h"
#include <ArduinoJson.h>

Titrator *_titrator = NULL;
long int positions[4];
bool onlineStatus = false;
bool busyStatus = false;

double stend1PositionsX[6];    ///< Массив координат x для отверстий сетки, в которую манипулятор переносит вещества.
double stend1PositionsY[6];    ///< Массив координат y для отверстий сетки, в которую манипулятор переносит вещества.

double stend2PositionsX[6];
///< Массив координат x для отверстий сетки, из которой манипулятор переносит вещества.
double stend2PositionsY[6];       ///< Массив 6координат y для отверстий сетки, из которой манипулятор переносит вещества.

double pipetkiNXY[6][2];          ///< Массив координат x,y пипеток.
double volume[2][4];              ///< Массив объёмов веществ в мкл (не более 3000).
int commandsNumber = -1;
double camera_X;
double camera_Y;
double camera_Z;
double target_X;
double target_Y;
double target_Z;
int marker_id;
double camera_aX;
double camera_aY;
double camera_aZ;

String str = "";
String subStr = "";
char ch = 0;
int len = 0;
int skobkaCounter = 0;
bool skobkaFlag = true;

bool camListenerMode = false;

DynamicJsonDocument outputInfo(2048);
DynamicJsonDocument inputMsg(2048);
void setup() {
  Serial.begin(115200);
  delay(1000);

  _titrator = new Titrator();

  pinMode(JOINT_1_ENDSTOP, INPUT_PULLUP);
  pinMode(JOINT_2_ENDSTOP, INPUT_PULLUP);
  pinMode(JOINT_3_ENDSTOP, INPUT_PULLUP);
  pinMode(JOINT_4_ENDSTOP, INPUT_PULLUP);



}


//Выполение команды с input и output id пробирок и величиной преносимой жидкости volume в мкл.
void startCommand(int input, int output, int volume) {
  _titrator->calibrater();
  _titrator->dropLiquid(volume + 500);
  moveToXYZ(0.25, 0, 0.35);
  moveToXYZ(stend1PositionsX[input], stend1PositionsY[input], 0.30);
  goInTube(stend1PositionsX[input], stend1PositionsY[input], 30, 17);
  _titrator->pickLiquid(volume);
  delay(100);
  goOutTube(stend1PositionsX[input], stend1PositionsY[input], 17, 27);

  _titrator->pickAir(400);
  goOutTube(stend1PositionsX[input], stend1PositionsY[input], 27, 30);

  delay(100);
  moveToXYZ(stend1PositionsX[output], stend1PositionsY[output], 0.32);
  goInTube(stend1PositionsX[output], stend1PositionsY[output], 30, 25);
  ///Сброс вещества (значенее, немного превышающее объём вещества в пипетке, для улучшения качества сброса)
  _titrator->dropLiquid(volume + 500);
  delay(100);
  goOutTube(stend1PositionsX[output], stend1PositionsY[output], 25, 30);
}

//Сброс пипетки
void dropPip(int id)
{
  ///Сброс пипетки

  moveToXYZ(pipetkiNXY[id][0] * 1.0, pipetkiNXY[id][1] * 1.0, 0.34);
  goInTube(pipetkiNXY[id][0] * 1.0, pipetkiNXY[id][1] * 1.0, 34, 21);
  _titrator->calibrater();
  _titrator->dropTube();
  moveToXYZ(0.0, 0, 0.4);
}

//Взятие пипетки по id
void takePip(int id)
{
  fullCalibrateJoints();
  delay(50);
  _titrator->calibrater();
  delay(50);
  ///Взятие випетки
  moveToXYZ(pipetkiNXY[id][0], pipetkiNXY[id][1], 0.20);
  delay(50);
  goInTube(pipetkiNXY[id][0], pipetkiNXY[id][1], 20, 14.4);
  joint4.setSpeed(dir4 * JOINT_4_AVERAGE_SPEED / 8);
  float timeStart = millis();
  float timeCurrent = millis();
  while (timeCurrent - timeStart < 1000) {
    joint4.runSpeed();
    timeCurrent = millis();
  }
  joint4.stop();
  joint4.setSpeed(-dir4 * JOINT_4_AVERAGE_SPEED / 16);
  timeStart = millis();
  timeCurrent = millis();
  while (timeCurrent - timeStart < 100) {
    joint4.runSpeed();
    timeCurrent = millis();
  }
  joint4.stop();
  joint4.setSpeed(-dir4 * JOINT_4_AVERAGE_SPEED);
  goOutTube(pipetkiNXY[id][0], pipetkiNXY[id][1], 14.4, 35);
  fullCalibrateJoints();

}

void loop() {
  //Commands recieving
  while ((skobkaFlag || skobkaCounter > 0) || Serial.available()) {
    if (Serial.available()) {
      ch = Serial.read();
      if (ch == '{')
      {
        skobkaCounter += 1;
        skobkaFlag = false;
      }
      if (ch == '}')
      {
        skobkaCounter -= 1;
      }
      if (!skobkaFlag)
      {
        str += (char)ch;
        len++;
      }
      else
      {
        break;
      }
    }
    delay(1);

  }


  if (len > 0 && !skobkaFlag && skobkaCounter == 0)
  {
    //JsonCommands
    if (str.substring(0, 1) == "{")
    {
      String result = jsonParseAndProcess(str);
    }
    str = "";
    subStr = "";
    ch = 0;
    len = 0;
    skobkaCounter = 0;
    skobkaFlag = true;
  }
  if (len > 100)
  {
    str = "";
    subStr = "";
    ch = 0;
    len = 0;
    skobkaCounter = 0;
    skobkaFlag = true;
  }
  delay(1);
}
//Sends message object via Serial
void sendSerialMessage(String message) {
  Serial.print(message);
  Serial.print("\n");
}

//Parses incoming Json string and returns a status of parsing.
//The parameter "str" is a string with json message to parse.
//returns "Success", if there is no errors and every key is allowable.
String jsonParseAndProcess(String str)
{
  //Serial.println(str);
  inputMsg.clear();
  deserializeJson(inputMsg, str);

  if (camListenerMode)
  {
    if (inputMsg.containsKey("xt"))
    {
      outputInfo.clear();
      target_X = inputMsg["xt"].as<double>();
      target_Y = inputMsg["yt"].as<double>();
      target_Z = inputMsg["zt"].as<double>();

      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (inputMsg.containsKey("marker_id"))
    {
      outputInfo.clear();
      marker_id = inputMsg["marker_id"].as<int>();

      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (inputMsg.containsKey("x"))
    {
      outputInfo.clear();
      camera_X = inputMsg["x"].as<double>();
      camera_Y = inputMsg["y"].as<double>();
      camera_Z = inputMsg["z"].as<double>();

      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (inputMsg.containsKey("ax"))
    {
      outputInfo.clear();
      camera_aX = inputMsg["ax"].as<double>();
      camera_aY = inputMsg["ay"].as<double>();
      camera_aZ = inputMsg["az"].as<double>();
      camListenerMode = false;
      float camCoord[6] = {camera_X, camera_Y, camera_Z, camera_aX, camera_aY, camera_aZ};

      if (abs(camera_X) > 0.0001 & abs(camera_Y) > 0.0001 & abs(camera_Z) > 0.0001) {
        float camCoordDesired[6] = { target_X, target_Y, target_Z, 0.0000, 0.0000, 0.0000}; //{-0.02000, -0.02400, 0.25400, 0.0000, 0.0000, 0.0000};
        outputInfo["status"] = (int)marker_tracking(camCoord, camCoordDesired);
        String outputString = "";
        serializeJson(outputInfo, outputString);
        sendSerialMessage(outputString);
        return "Success";
      }
      else
      {
        outputInfo["status"] = 0;
        String outputString = "";
        serializeJson(outputInfo, outputString);
        sendSerialMessage(outputString);
        return "No marker";
      }


    }

  }

  if (inputMsg.containsKey("command"))
  {
    String command = inputMsg["command"].as<String>();
    if (command == "connect") {
      outputInfo.clear();
      outputInfo["status"] = (int)busyStatus;
      outputInfo["online"] = (int)onlineStatus;
      String outputString = "";
      serializeJson(outputInfo, outputString);

      sendSerialMessage(outputString);
      return "Success";
    }
    if (command == "grid") {
      outputInfo.clear();
      int cellsNumber = 12;
      String cellsTypes[cellsNumber] = {"both", "source", "none",
                                        "both", "source", "none",
                                        "both", "source", "none",
                                        "both", "source", "none"
                                       };
      for (int i = 0; i < cellsNumber; i++) {
        outputInfo["cells"][i] = cellsTypes[i];
      }
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "run")
    {
      outputInfo.clear();
      DynamicJsonDocument commandsInfo(4096);
      commandsInfo["commands"] = inputMsg["data"]["commands"];

      commandsNumber = inputMsg["listSize"].as<int>();
      //Serial.println(str);
      /*for (int i = 0; i < commandsNumber; i++)
        {
        int input = (int)commandsInfo["commands"][i]["input"];
        int output = (int)commandsInfo["commands"][i]["output"];
        int volume = (int)commandsInfo["commands"][i]["volume"];
        startCommand(input, output, volume);
        }*/
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "camUpd")
    {
      outputInfo.clear();
      camListenerMode = true;
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "moveToXYZ")
    {
      outputInfo.clear();
      float target_X = inputMsg["x"].as<float>();
      float target_Y = inputMsg["y"].as<float>();
      float target_Z = inputMsg["z"].as<float>();
      outputInfo["status"] = (int)moveToXYZ(target_X, target_Y, target_Z);
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }
    if (command == "moveZ")
    {
      outputInfo.clear();
      float delta_Z = inputMsg["dz"].as<float>();
      outputInfo["status"] = (int)moveZ(delta_Z);
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "titrCalibrate")
    {
      outputInfo.clear();
      _titrator->calibrater();
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "titrDropLiq")
    {
      outputInfo.clear();
      float volume = inputMsg["v"].as<float>();
      _titrator->dropLiquid(volume);
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "titrPickLiq")
    {
      outputInfo.clear();
      float volume = inputMsg["v"].as<float>();
      _titrator->pickLiquid(volume);
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "titrPickAir")
    {
      outputInfo.clear();
      float volume = inputMsg["v"].as<float>();
      _titrator->pickAir(volume);
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "titrDropPip")
    {
      outputInfo.clear();
      _titrator->calibrater();
      _titrator->dropTube();
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }

    if (command == "finalPipMove")
    {
      outputInfo.clear();

      stepper4.setRunMode(KEEP_SPEED);
      stepper4.setSpeed(-dir4 * STEPPER4_MAX_SPEED / 20);
      float timeStart = millis();
      float timeCurrent = millis();
      long currentStep4Pos = stepper4.getCurrent();
      while (timeCurrent - timeStart < 1000) {
        stepper4.tick();
        timeCurrent = millis();
      }
      stepper4.brake();
      stepper4.setSpeed(dir4 * STEPPER4_MAX_SPEED / 20);
      timeStart = millis();
      timeCurrent = millis();
      while (timeCurrent - timeStart < 380) {
        stepper4.tick();
        timeCurrent = millis();
      }
      stepper4.brake();
      stepper4.setSpeed(-dir4 * STEPPER4_MAX_SPEED);
      stepper4.setCurrent(currentStep4Pos);
    
      outputInfo["status"] = 1;
      String outputString = "";
      serializeJson(outputInfo, outputString);
      sendSerialMessage(outputString);
      return "Success";
    }


  }
  if (inputMsg.containsKey("task"))
  {
    //Serial.println(str);
    outputInfo.clear();
    int input = (int)inputMsg["task"]["input"];
    int output = (int)inputMsg["task"]["output"];
    int volume = (int)inputMsg["task"]["volume"];
    startCommand(input, output, volume);
    outputInfo["status"] = 1;
    String outputString = "";
    serializeJson(outputInfo, outputString);
    sendSerialMessage(outputString);
    return "Success";
  }
  if (inputMsg.containsKey("takePip"))
  {
    outputInfo.clear();
    int sourceId = (int)inputMsg["takePip"];
    takePip(sourceId);
    outputInfo["status"] = 1;
    String outputString = "";
    serializeJson(outputInfo, outputString);
    sendSerialMessage(outputString);
    return "Success";
  }
  if (inputMsg.containsKey("dropPip"))
  {
    outputInfo.clear();
    int sourceId = (int)inputMsg["dropPip"];
    dropPip(sourceId);
    outputInfo["status"] = 1;
    String outputString = "";
    serializeJson(outputInfo, outputString);
    sendSerialMessage(outputString);
    return "Success";
  }

  if (inputMsg.containsKey("calibrate"))
  {
    outputInfo.clear();
    fullCalibrateJoints();
    //moveToXYZ(0.25, 0.05, 0.35);
    outputInfo["status"] = 1;
    String outputString = "";
    serializeJson(outputInfo, outputString);
    sendSerialMessage(outputString);
    return "Success";
  }

  if (inputMsg.containsKey("finish")) {
    outputInfo.clear();
    finishCalibrateJoints();
    outputInfo["status"] = 1;
    String outputString = "";
    serializeJson(outputInfo, outputString);
    sendSerialMessage(outputString);
    return "Success";
  }

  return "Json is incorrect";
}
