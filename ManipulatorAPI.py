import json
import serial
import time
import serial.tools.list_ports
import sys
import glob
import matplotlib.pyplot as plt
from drawnow import drawnow
import csv
import math


import inputs
import cv2
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
from math import sin,cos,radians
from numpy import dot
import arucoOdometry
import matplotlib.ticker as ticker
import pandas as pd
from scipy.signal import savgol_filter



class ManipulatorAPI:
  tubesIds=[]
  status=False
  online=False
  serialArduino = serial.Serial()
  serialArduino.baudrate=115200
  serialArduino.timeout=15
  commandsList=[]
  taskSafety=False
  maxVolume=40000
  minVolume=500


  aruco_marker_side_length = 0.0344
  aruco_dictionary_name = "DICT_4X4_50"
  # Calibration parameters y-aml file
  camera_calibration_parameters_filename = 'calibration_chessboardDEXP720.yaml'

  odom = arucoOdometry.arucoOdometry()
  odom.setCameraParams(camera_calibration_parameters_filename)
  odom.setArucoLength(aruco_marker_side_length)
  odom.setArucoDict(aruco_dictionary_name)
  markers=[{"id":0,"size":aruco_marker_side_length},
  {"id":8,"size":aruco_marker_side_length},
  {"id":10,"size":0.035},
  {"id":14,"size":0.034},
  {"id":16,"size":0.034},
  {"id":18,"size":0.034},
  {"id":24,"size":0.034}]
  odom.setMarkers(markers)
  
  
  pipList=[{"id":0,"id_marker":16,"x":-0.040,"y":-0.056,"z":0.1600},
    {"id":1,"id_marker":16,"x":0.0085,"y":-0.053,"z":0.1600}]
  pipListDrop=[{"id":0,"id_marker":16,"x":-0.040,"y":-0.056,"z":0.1600},
    {"id":1,"id_marker":16,"x":0.0085,"y":-0.053,"z":0.1600}]

  tubesList=[
    {"id": 0,"id_marker":24,"x":-0.150,"y":-0.030,"topZ":0.280,"botZ":0.17},
    {"id": 1,"id_marker":24,"x":-0.108,"y":-0.015,"topZ":0.280,"botZ":0.17},
    {"id": 2,"id_marker":14,"x":0.0600,"y":-0.0550,"topZ":0.295,"botZ":0.27},
    {"id": 3,"id_marker":18,"x":0.053,"y":-0.058,"topZ":0.275,"botZ":0.245},
    {"id": 4,"id_marker":18,"x":0.050,"y":-0.140,"topZ":0.275,"botZ":0.245},
    ]

  #инициализация манипулятора
  def __init__(self,port,cap):
    self.cap=cap
    self.connect(port)

  #поиск COM-портов
  def portSearch(self):
        """Searches availiable for connection ports."""
        """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

  #Вывод COM-портов
  def portSearchShow():
    for i in serial.tools.list_ports.comports():
      print(i)

  def makeJSONInit(self):
        """Generates initital Json message for checking connetcion with a manipulator."""
        message = {
        "command":"connect",
        "data":""
        }
        return json.dumps(message)
  def makeJSONGrid(self):
        """Generates Json message for getting info from a manipulator."""
        message = {
        "command":"grid",
        "data":""
        }
        return json.dumps(message)
  def makeJSONTaskInit(self,input_reagents,mixing_rule):
        """Generates initital Json message for checking connetcion with a manipulator."""
        self.makeCommands(input_reagents,mixing_rule)
        taskCheckStatus=self.taskCheck(input_reagents)
        task={""}
        message = {
        "command":"run",
        "listSize":len(self.commandsList)
        }
        print(len(self.commandsList))
        return json.dumps(message)

  def makeJSONTask(self,task):
        """Generates task Json message for the manipulator."""
        message = {
        "task":task
        }
        print(len(self.commandsList))
        return json.dumps(message)

  #Генерирует команды алгоритма на основе задачи и исходных данных стола
  def makeCommands(self,input_reagents,mixing_rule):
    self.commandsList=[]
    for reagent in input_reagents["reagents"]:
      for solution in mixing_rule:
        for component in solution["reagents"]:
          if(reagent["name"]==component["name"]):
            command = {"input":reagent["id"],"name":component["name"],"volume":component["volume"],"output":solution["id"]}
            self.commandsList.append(command)
    print(self.commandsList)
  def sendJSON(self,stringJson):
        """Sends stringJson to controller"""
        self.serialArduino.write(bytes(stringJson, 'utf-8'))
        print(str(bytes(stringJson, 'utf-8')))
        answerMessage=self.serialArduino.readline()
        return answerMessage
  def parseJSONAns(self, stringJson):
        """Parses stringJson, that is an answer"""
        
        if(len(stringJson)>3):
          print("answer:")
          message = json.loads(stringJson)
          print(message)
          if(message.get("status")==1):
            self.status=True
          elif(message.get("status")==0):
            self.status=False
          self.online=True

          if(message.get("cells")!=None):
            for cell in message["cells"]:
              self.tubesIds.append(cell);
          if(message.get("numberOfMovements")!=None and message.get("result")!=None):
            print(str(message["numberOfMovements"]) + " movements "+str(message["result"]))
        else:
          self.status=False
          self.online=False

  #Подключение к контроллеру
  def connect(self,comPort):
    self.online=False
    if(not self.serialArduino.port==comPort):
        self.serialArduino.port=comPort
    if(not self.serialArduino.is_open):
        self.serialArduino.open()
    time.sleep(1)
    self.parseJSONAns(self.sendJSON(self.makeJSONInit()))
  def update_arduinoInfo(self):
    pass

  #Получить координаты сетки рабочего стола от контроллера
  def get_grid(self):
    self.parseJSONAns(self.sendJSON(self.makeJSONGrid()))
    return self.tubesIds
  def get_status(self):
    return self.status

  #Валидация алгоритма
  def taskCheck(self,input_reagents):
    tubesVolumeList={"id":[],"volume":[]}
    checkList=[]
    for reagent in input_reagents["reagents"]:
      if(reagent["id"] in tubesVolumeList["id"]):

        tubesVolumeList["volume"][reagent["id"]]+=reagent["volume"]
      else:
        tubesVolumeList["volume"].append(reagent["volume"])
        tubesVolumeList["id"].append(reagent["id"])
    for command in self.commandsList:
      if(not command["output"] in tubesVolumeList["id"]):
        tubesVolumeList["id"].append(command["output"])
        tubesVolumeList["volume"].append(0)
    tubesVolumeListInit=tubesVolumeList.copy()

    for command in self.commandsList:
      tubesVolumeList["volume"][tubesVolumeList["id"].index(command["input"])]-=command["volume"]
      tubesVolumeList["volume"][tubesVolumeList["id"].index(command["output"])]+=command["volume"]
      inputVol=tubesVolumeList["volume"][tubesVolumeList["id"].index(command["input"])]
      outputVol=tubesVolumeList["volume"][tubesVolumeList["id"].index(command["output"])]
      if(inputVol>=self.minVolume and inputVol<=self.maxVolume and outputVol<=self.maxVolume):
        checkList.append(1)
      else:
        checkList.append(0)
    print(tubesVolumeList)
    print(checkList)
    
    if(0 in checkList):
      if(checkList[0]==0):
        self.commandsList=[]
        print(self.commandsList)
        return False
      self.commandsList=self.commandsList[0:checkList.index(0)]
      print(self.commandsList)
      return False
    
    print(self.commandsList)
    return True

  #Финальная калибровка манипулятора до сложенного положения
  def finish(self):
    if(self.status and self.online or 1):
      shortTask={"finish":""}
      self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
      self.serialArduino.timeout=1
      self.status=False

  #Запуск теста(задачи)
  def make_test(self,input_reagents, mixing_rule):
    if(self.status and self.online or 1):
      self.parseJSONAns(self.sendJSON(self.makeJSONTaskInit(input_reagents,mixing_rule)))
      #self.calibrate()
      #self.goStart()
      sourceId = -1
      sourcesList=[]
      for index,task in enumerate(self.commandsList):
        self.serialArduino.timeout=1000
        if(not task["name"] in sourcesList):
          sourcesList.append(task["name"])
          sourceId+=1
          if(sourceId>0):
            self.dropPip(sourceId-1)
          self.takePip(sourceId)
        shortTask=task.copy()
        shortTask.pop("name")
        self.makeTask(shortTask)
      self.dropPip(sourceId)
      self.calibrate()
      self.serialArduino.timeout=1
      self.status=False
      #return answer
    #return status is busy
  
  #Взятие пипетки sourceId
  def takePip(self, sourceId):
    pipList=self.pipList
    targetPip=None
    for index, pip in enumerate(pipList):
      if(pip["id"]==sourceId):
        targetPip=pip
    self.calibrate()
    self.titrCalibrate()
    self.moveToXYZcam(targetPip["x"],targetPip["y"],targetPip["z"],targetPip["id_marker"])
    self.moveZ(-0.012)
    self.finalPipMove()
    self.moveZ(0.165)
    self.calibrate()

  #Сброс пипетки в ячейку sourceId
  def dropPip(self,sourceId):
    pipList=self.pipListDrop
    targetPip=None
    for index, pip in enumerate(pipList):
      if(pip["id"]==sourceId):
        targetPip=pip
    self.goStart()
    self.calibrate()
    self.moveToXYZcam(targetPip["x"],targetPip["y"]-0.003,targetPip["z"]+0.11,targetPip["id_marker"])
    self.moveToXYZcam(targetPip["x"],targetPip["y"]-0.003,targetPip["z"]+0.065,targetPip["id_marker"])
    self.titrDropPip()
    self.titrCalibrate()
    self.goStart()

  #Калибровка манипулятора
  def calibrate(self):
    shortTask={"calibrate":""}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
    self.goStart()

  #Калибровка дозатора
  def titrCalibrate(self):
    shortTask={"command":"titrCalibrate"}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Сброс volume мкл жидкости дозатором
  def titrDropLiq(self,volume):
    shortTask={"command":"titrDropLiq","v":volume}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Набор volume мкл жидкости дозатором
  def titrPickLiq(self, volume):
    shortTask={"command":"titrPickLiq","v":volume}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Взятие воздуха дозатором
  def titrPickAir(self, volume):
    shortTask={"command":"titrPickAir","v":volume}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Сброс пипетки(наконечника) дозатора
  def titrDropPip(self):
    shortTask={"command":"titrDropPip"}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Вертикальное перемещение инструмента
  def moveZ(self,deltaZ):
    shortTask={"command":"moveZ","dz":deltaZ}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
  
  #Финальное перемещение при взятии пипетки(наконечника) для плотной посадки
  def finalPipMove(self):
    shortTask={"command":"finalPipMove"}
    self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))

  #Создание задачи JSON
  def makeTask(self, shortTask):
    tubesList=self.tubesList
    inpTube=None
    outpTube=None
    for index, tube in enumerate(tubesList):
      if(tube["id"]==shortTask["input"]):
        inpTube=tube
      if(tube["id"]==shortTask["output"]):
        outpTube=tube

    self.titrCalibrate()
    self.titrDropLiq(shortTask["volume"]+500)
    self.goStart()
    self.moveToXYZcam(inpTube["x"],inpTube["y"],inpTube["topZ"],inpTube["id_marker"])
    self.moveZ(inpTube["botZ"]-inpTube["topZ"])
    self.titrPickLiq(shortTask["volume"])
    self.moveZ(inpTube["topZ"]-inpTube["botZ"])
    self.titrPickAir(400)

    self.moveToXYZcam(outpTube["x"],outpTube["y"],outpTube["topZ"],outpTube["id_marker"])
    self.moveZ(outpTube["botZ"]-outpTube["topZ"])
    self.titrDropLiq(shortTask["volume"]+500)
    self.moveZ(outpTube["topZ"]-outpTube["botZ"])
    self.goStart()

  #Создание json с данными о координатах инструмента в разных системах координат, в том числе СК маркера marker_id
  def makeCameraJson(self, x,y,z,a_x,a_y,a_z,x_t,y_t,z_t,marker_id):
    """Generates initital Json message for checking connetcion with a manipulator."""
    message1 = {"command":"camUpd"}
    message2 = {"x":x, "y":y, "z":z}
    message3 = {"ax":a_x, "ay":a_y, "az":a_z}
    message4 = {"xt":x_t,"yt":y_t,"zt":z_t}
    message5 = {"marker_id":marker_id}
    return json.dumps(message1),json.dumps(message2),json.dumps(message3),json.dumps(message4),json.dumps(message5)

  #Отправка json с данными с камеры на контроллер
  def sendCameraJson(self,x,y,z,a_x,a_y,a_z,x_t,y_t,z_t,marker_id):
    mess1, mess2, mess3, mess4, mess5 = self.makeCameraJson(
      float(format(x,".3g")),
      float(format(y,".3g")),
      float(format(z,".3g")),
      float(format(a_x,".3g")),
      float(format(a_y,".3g")),
      float(format(a_z,".3g")),
      float(format(x_t,".3g")),
      float(format(y_t,".3g")),
      float(format(z_t,".3g")),
      int(marker_id)
      )
    self.parseJSONAns(self.sendJSON(mess1))
    time.sleep(0.005)
    self.parseJSONAns(self.sendJSON(mess2))
    time.sleep(0.005)
    self.parseJSONAns(self.sendJSON(mess4))
    time.sleep(0.005)
    self.parseJSONAns(self.sendJSON(mess5))
    time.sleep(0.005)
    self.parseJSONAns(self.sendJSON(mess3))
    time.sleep(0.005)

  #Создание json для перемещения инструмента в новые x,y,z
  def makeJsonMoveToXYZ(self, x, y, z):
    """Generates initital Json message for checking connetcion with a manipulator."""
    message = {
    "command":"moveToXYZ",
    "x":x, "y":y, "z":z
    }
    return json.dumps(message)

  #Перемещение инструмента в новые x,y,z
  def moveToXYZ(self, x, y, z):
    self.parseJSONAns(self.sendJSON(self.makeJsonMoveToXYZ(x,y,z)))

  #Перемещение инструмента в новые x_t,y_t,z_t в системе координат маркера marker_id
  def moveToXYZcam(self,x_t,y_t,z_t,marker_id):
    startTime=time.time() * 1000
    while(True):
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      #ret, frame = self.cap.read()
      #time.sleep(0.005)
      ret, frame = self.cap.read()
      ret, frame = self.cap.read()
      frame,x,y,z,a_x,a_y,a_z = self.odom.updateCameraPoses(frame,time.time()*1000-startTime,marker_id, x_t,y_t,z_t)
      #cv2.imshow("im",frame)
      #cv2.waitKey(1)
      if(x==y==z==0):
        self.goStart()
        self.calibrate()
      else:
        diap=0.0025
        if(x>x_t-diap and x<x_t+diap and y>y_t-diap and y<y_t+diap and z>z_t-0.107-diap*2 and z<z_t-0.107+diap*2):
          break
        else:
          self.sendCameraJson(x,y,z,a_x,a_y,a_z,x_t,y_t,z_t,marker_id)
  #Перемещение инструмента в стартовое положение
  def goStart(self):
    self.moveToXYZ(0.29,0.1,0.37)
