import time
from manipulator import Manipulator as ManipulatorMetalic
from ManipulatorAPI import ManipulatorAPI as ManipulatorPlastic
import cv2
import threading

metalicFlag = False

def plastic_thread():
	global metalicFlag
	print('START plastic thread')
	try:
		# Start the video stream
		cap = cv2.VideoCapture(1)
		cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) #параметры камеры
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		cap.set(cv2.CAP_PROP_FRAME_COUNT,1)
		cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
		cap.set(cv2.CAP_PROP_POS_FRAMES,0)
		port = "COM7"# порт ARDUINO пластикового робота
		plasticManipulator = ManipulatorPlastic(port,cap)
		grid = plasticManipulator.get_grid() 
		print(grid)

		#задаем исходные данные стола
		input_reagents={"reagents": [
		{"id": 0, "name": "liq1", "volume": 40000},
		{"id": 1, "name": "liq2", "volume": 40000}#,
		#{"id": 2, "name": "sok", "volume": 40000}
		]}

		#задаём задачу на смешивание
		mixing_rule = [
		  #{
		  #  "id": 3,
		  #  "reagents": [
		  #    {"name": "liq1", "volume": 2500},
		  #    {"name": "liq2", "volume": 2500}#,
		  #    #{"name": "", "volume": 2000}
		  #  ]
		  #},
		  {
		    "id": 4,
		    "reagents": [
		      #{"name": "liq1", "volume": 3500},
		      {"name": "liq2", "volume": 3000}
		    ]
		  }
		]


		time.sleep(1)

		#Инициализация работы с контроллером, исходных данных о столе и данных о задаче
		plasticManipulator.parseJSONAns(plasticManipulator.sendJSON(plasticManipulator.makeJSONTaskInit(input_reagents,mixing_rule)))
		sourceId = -1
		sourcesList=[]
		
		#Выполнение команд по исходным данным
		for index,task in enumerate(plasticManipulator.commandsList):
			plasticManipulator.serialArduino.timeout=1000
			#Если новый реагент - надеваем новую пипетку(наконечник)
			if(not task["name"] in sourcesList):
				sourcesList.append(task["name"])
				sourceId+=1
				if(sourceId>0):
					#Если пипетка уже была надета, сначала её сбросим
					plasticManipulator.dropPip(sourceId-1)
				#Наденем новую пипетку
				plasticManipulator.takePip(sourceId)
			#Запустить подзадачу из алгоритма общего
			shortTask=task.copy()
			shortTask.pop("name")
			plasticManipulator.makeTask(shortTask)
		metalicFlag=True
		while metalicFlag:
			time.sleep(2)
			print("plastic waits")
		print("plastic continues")
		#сброс пипетки
		plasticManipulator.dropPip(sourceId)
		#Калибровка манипулятора
		plasticManipulator.calibrate()
		plasticManipulator.serialArduino.timeout=1

	except KeyboardInterrupt as e:
	    print('KEYBOARD INTERRUPT Plastic thread ' + str(e))
	except Exception as e:
	    exc_type, exc_value, exc_traceback = sys.exc_info()
	    print('EXCEPTION Plastic thread ' + str(e))
try:
	threading.Thread(target=plastic_thread).start()
except e:
	print(str(e))
finally:
	print('Shutting down connection to plastic robot...')        


def metalic_thread():
	global metalicFlag
	print('START metalic thread')
	try:
		metalicRobot = ManipulatorMetalic(test_mode=False)
		metalicRobot.auto_calibrate()
		metalicRobot.null_position()
		time.sleep(5)
		metalicRobot.read_points('./Scripts_UV/opening_cap.txt')
		metalicRobot.read_points('./Scripts_UV/calibration.txt')
		while metalicFlag==False:
			time.sleep(1)
			print("metalic waits")
		print("metalic continues")
		metalicRobot.read_points('./Scripts_UV/take_cuvette_1.txt')
		metalicRobot.read_points('./Scripts_UV/move_to_UV.txt')
		metalicFlag = False
		metalicRobot.read_points('./Scripts_UV/take_cuvette_2.txt')
		metalicRobot.read_points('./Scripts_UV/closing_cap.txt')
		metalicRobot.read_points('./Scripts_UV/calibration.txt')
		metalicRobot.read_points('./Scripts_UV/opening_cap.txt')
		metalicRobot.read_points('./Scripts_UV/calibration.txt')
		metalicRobot.read_points('./Scripts_UV/moving_to_base.txt')
		metalicRobot.read_points('./Scripts_UV/calibration.txt')
		metalicRobot.read_points('./Scripts_UV/closing_cap.txt')
	except KeyboardInterrupt as e:
	    print('KEYBOARD INTERRUPT Metalic thread ' + str(e))
	except Exception as e:
	    exc_type, exc_value, exc_traceback = sys.exc_info()
	    print('EXCEPTION Metalic thread ' + str(e))


try:
    threading.Thread(target=metalic_thread).start()
except e:
    print(str(e))
finally:
    print('Shutting down connection to metalic robot...') 