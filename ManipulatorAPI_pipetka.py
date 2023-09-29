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
  


  def __init__(self,port):
    self.connect(port)

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
        """Generates initital Json message for checking connetcion with a manipulator."""
        message = {
        "task":task
        }
        print(len(self.commandsList))
        return json.dumps(message)

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
        """Sends stringJson to potentiostat. TODO."""
        self.serialArduino.write(bytes(stringJson, 'utf-8'))
        print(str(bytes(stringJson, 'utf-8')))
        answerMessage=self.serialArduino.readline()
        return answerMessage
  def parseJSONAns(self, stringJson):
        """Parses stringJson, that is an answer from potentiostat with it's system info."""
        
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

  def connect(self,comPort):
    self.online=False
    """Сonnects user interface to a potentiostat. TODO"""
    if(not self.serialArduino.port==comPort):
        self.serialArduino.port=comPort
    if(not self.serialArduino.is_open):
        self.serialArduino.open()
    time.sleep(1)
    self.parseJSONAns(self.sendJSON(self.makeJSONInit()))
  def update_arduinoInfo(self):
    pass
  def get_grid(self):
    self.parseJSONAns(self.sendJSON(self.makeJSONGrid()))
    return self.tubesIds
  def get_status(self):
    return self.status
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

  def finish(self):
    if(self.status and self.online or 1):
      shortTask={"finish":""}
      self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
      self.serialArduino.timeout=1
      self.status=False

  def make_test(self,input_reagents, mixing_rule):
    if(self.status and self.online or 1):
      self.parseJSONAns(self.sendJSON(self.makeJSONTaskInit(input_reagents,mixing_rule)))
      sourceId = -1
      sourcesList=[]
      for index,task in enumerate(self.commandsList):
        self.serialArduino.timeout=1000
        if(not task["name"] in sourcesList):
          sourcesList.append(task["name"])
          sourceId+=1
          if(sourceId>0):
            shortTask={"dropPip":sourceId-1}
            self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
          shortTask={"takePip":sourceId}
          self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
        shortTask=task.copy()
        shortTask.pop("name")
        self.parseJSONAns(self.sendJSON(self.makeJSONTask(shortTask)))
      shortTask={"dropPip":sourceId}
      self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
      shortTask={"calibrate":""}
      self.parseJSONAns(self.sendJSON(json.dumps(shortTask)))
      self.serialArduino.timeout=1
      self.status=False
      #return answer
    #return status is busy
  
  def makeCameraJson(self, x,y,z,a_x,a_y,a_z):
    """Generates initital Json message for checking connetcion with a manipulator."""
    message1 = {"command":"camUpd"}
    message2 = {"x":x, "y":y, "z":z}
    message3 = {"ax":a_x, "ay":a_y, "az":a_z}
    return json.dumps(message1),json.dumps(message2),json.dumps(message3)

  def sendCameraJson(self,x,y,z,a_x,a_y,a_z):
    mess1, mess2, mess3= self.makeCameraJson(float(format(x,".3g")),float(format(y,".3g")),float(format(z,".3g")),float(format(a_x,".3g")),float(format(a_y,".3g")),float(format(a_z,".3g")))
    self.parseJSONAns(self.sendJSON(mess1))
    time.sleep(0.01)
    self.parseJSONAns(self.sendJSON(mess2))
    time.sleep(0.01)
    self.parseJSONAns(self.sendJSON(mess3))
    time.sleep(0.05)


  def makeJsonMoveToXYZ(self, x, y, z):
    """Generates initital Json message for checking connetcion with a manipulator."""
    message = {
    "command":"moveToXYZ",
    "x":x, "y":y, "z":z
    }
    return json.dumps(message)

  def moveToXYZ(self, x, y, z):
    self.parseJSONAns(self.sendJSON(self.makeJsonMoveToXYZ(x,y,z)))





aruco_marker_side_length = 0.0344
aruco_dictionary_name = "DICT_4X4_50"
# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboardDEXP1080.yaml'


# Start the video stream
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

odom = arucoOdometry.arucoOdometry()
odom.setCameraParams(camera_calibration_parameters_filename)
odom.setArucoLength(aruco_marker_side_length)
odom.setArucoDict(aruco_dictionary_name)
markers=[{"id":0,"size":aruco_marker_side_length},{"id":8,"size":aruco_marker_side_length}]
odom.setMarkers(markers)

ManipulatorAPI.portSearchShow()
port = input()
manipulator = ManipulatorAPI(port)
grid = manipulator.get_grid()
print(grid)
input_reagents={"reagents": [
{"id": 0, "name": "voda", "volume": 40000},
{"id": 1, "name": "pivo", "volume": 40000}#,
#{"id": 2, "name": "sok", "volume": 40000}
]}

mixing_rule = [
  {
    "id": 3,
    "reagents": [
      {"name": "voda", "volume": 2000},
      {"name": "pivo", "volume": 2000}#,
      #{"name": "", "volume": 2000}
    ]
  },
  {
    "id": 4,
    "reagents": [
      {"name": "voda", "volume": 2000},
      {"name": "pivo", "volume": 2000}
    ]
  }
]
shortTask={"calibrate":""}
manipulator.parseJSONAns(manipulator.sendJSON(json.dumps(shortTask)))
manipulator.moveToXYZ(0.25,0.05,0.35)
time.sleep(1)
startTime=time.time() * 1000
while(True):
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = cap.read()

    frame,x,y,z,a_x,a_y,a_z = odom.updateCameraPoses(frame,time.time()*1000-startTime,8)
    cv2.imshow("im",frame)
    cv2.waitKey(1)
    if(x==y==z==0):
      manipulator.moveToXYZ(0.25,0.05,0.35)
      shortTask={"calibrate":""}
      manipulator.parseJSONAns(manipulator.sendJSON(json.dumps(shortTask)))
      manipulator.moveToXYZ(0.25,0.05,0.35)
    else:
      manipulator.sendCameraJson(x,y,z,a_x,a_y,a_z)
    
manipulator.finish()
