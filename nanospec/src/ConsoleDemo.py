#
# Copyright (C) 2019 nanoLambda, Inc.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import sys
import traceback
import threading
import serial
import serial.tools.list_ports
from NanoLambdaNSP32 import *

"""A console program to demonstrate full functionalities of NSP32."""

ser = None
nsp32 = None
lockReturn = threading.Lock()

def main():
	global nsp32
	global lockReturn

	# open serial port
	while not OpenSerialPort() :
		pass

	# process console commands		
	while True :
		# wait for previous command return
		lockReturn.acquire()

		# show available console commands and get user input
		cmd = ShowConsoleCommands()
		
		if cmd == 'hello' :
			nsp32.Hello(0)
		elif cmd == 'sensorid' :
			nsp32.GetSensorId(0)
		elif cmd == 'wavelength' :
			nsp32.GetWavelength(0)
		elif cmd == 'spectrum' :
			nsp32.AcqSpectrum(0, 32, 3, False)	# integration time = 32; frame avg num = 3; disable AE
		elif cmd == 'xyz' :
			nsp32.AcqXYZ(0, 32, 3, False)		# integration time = 32; frame avg num = 3; disable AE
		elif cmd == 'exit' :
			break
		else:
			lockReturn.release()
			print('invalid command!')

def OpenSerialPort():
	"""open serial port
	
	Returns:
		bool: True for success; false for error
		
	"""

	global ser
	global nsp32

	# list all available serial ports
	print('')
	print('***********************')
	print('detected serial ports are:')

	ports = serial.tools.list_ports.comports()
	
	for idx, info in enumerate(ports) :
		print('%d) %s' % (idx + 1, info.device))

	print('')
	print("[NOTE] If the target port doesn't show up, re-plug the cable and press ENTER to try again.\n")
	
	# get user selected index number	
	index = None
	
	try :
		index = int(input('type the index number to connect to NSP32 (0 to exit): '))
	except :
		print('invalid index!')
		return False
	
	if index < 0 or index > len(ports) :
		print('invalid index!')
		return False

	# index 0 to exit program
	if index == 0 :
		sys.exit()

	try :
		# open user selected serial port
		ser = serial.Serial(ports[index - 1].device, baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)

		# create a new NSP32 instance
		nsp32 = NSP32(DataChannelSendData, OnReturnPacketReceived)

		# start the serial port receiving thread
		thread = threading.Thread(target = SerialPortReceive, args = (ser, nsp32))
		thread.daemon = True
		thread.start()
		
		return True
	except Exception as e :
		print('connect error: ' + str(e))

def DataChannelSendData(data):
	"""'send data' delegate: to send data (commands) to NSP32 through serial port
	
	Args:
		data(bytearray): data (commands) to send
		
	"""
	
	global ser
	
	try :
		if ser.isOpen():
			ser.write(data)
	except Exception as e :
		print('send error: ' + str(e))

def SerialPortReceive(ser, nsp32):
	"""serial port receiving thread
	
	Args:
		ser(serial.Serial): serial port instance
		nsp32(NSP32): NSP32 instance
		
	"""
	
	try :
		while ser.isOpen():
			if(ser.in_waiting > 0) :
				# feed received bytes to API, so that API can parse the return packet
				nsp32.OnReturnBytesReceived(ser.read(ser.in_waiting))
	except Exception as e :
		print('receive error: ' + str(e))

def OnReturnPacketReceived(pkt):
	"""'return packet received' delegate: to notify that a packet is received
	
	Args:
		pkt(ReturnPacket): the packet received
		
	"""
	
	global lockReturn
	
	# if invalid packet is received, show error message
	if not pkt.IsPacketValid:
		print('invalid packet received')
		lockReturn.release()
		return

	# process the return packet
	if pkt.CmdCode == CmdCodeEnum.Hello :			# Hello
		print('-------------')
		print('hello ok')
		lockReturn.release()
	elif pkt.CmdCode == CmdCodeEnum.GetSensorId :	# GetSensorId
		print('-------------')
		print('sensor id = ' + pkt.ExtractSensorIdStr())
		lockReturn.release()
	elif pkt.CmdCode == CmdCodeEnum.GetWavelength :	# GetWavelength
		infoW = pkt.ExtractWavelengthInfo()
	
		print('-------------')
		print('num of points = ' + str(infoW.NumOfPoints))
		print('wavelength = ')
		print(infoW.Wavelength)
		lockReturn.release()
	elif pkt.CmdCode == CmdCodeEnum.GetSpectrum :	# GetSpectrum
		infoS = pkt.ExtractSpectrumInfo()
	
		print('-------------')
		print('integration time = ' + str(infoS.IntegrationTime))
		print('saturation flag = ' + str(infoS.IsSaturated))
		print('num of points = ' + str(infoS.NumOfPoints))
		print('spectrum = ')
		print([round(x, 6) for x in infoS.Spectrum])
		print('X, Y, Z = ', [round(x, 2) for x in [infoS.X, infoS.Y, infoS.Z]])
		lockReturn.release()
	elif pkt.CmdCode == CmdCodeEnum.GetXYZ :		# GetXYZ
		infoXYZ = pkt.ExtractXYZInfo()
	
		print('-------------')
		print('integration time = ' + str(infoXYZ.IntegrationTime))
		print('saturation flag = ' + str(infoXYZ.IsSaturated))
		print('X, Y, Z = ', [round(x, 2) for x in [infoXYZ.X, infoXYZ.Y, infoXYZ.Z]])
		lockReturn.release()

def ShowConsoleCommands():
	"""show available console commands
	
	Returns:
		str: user input console command
	
	"""
	
	print('')
	print('***********************')
	print('1) hello - say hello to NSP32')
	print('2) sensorid - get sensor id string')
	print('3) wavelength - get wavelength')
	print('4) spectrum - start spectrum acquisition and get the result data')
	print('5) xyz - start XYZ acquisition and get the result data')
	print('6) exit - exit program')
	print('')
	return input('type an available command (case sensitive): ')


# start the program
try :
	main()
except Exception as e :
	print('error occurred: ' + str(e))
	print(traceback.format_exc())
