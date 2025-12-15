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

import threading
import serial
from NanoLambdaNSP32 import *

"""A clean and simple example for beginners to start with NSP32."""

ser = None
nsp32 = None

def main():
	#***********************************************************************************
	# modify this section to fit your need                                             *
	#***********************************************************************************
	
	portName = '/dev/nanospec'	# modify the port name your NSP32 is connected to (e.g. 'COM2' on Windows; '/dev/ttyS0' on Linux)
	
	#***********************************************************************************

	global ser	
	global nsp32

	# inform user to press ENTER to exit the program
	print("\n*** press ENTER to exit program ***\n")

	# open serial port
	ser = serial.Serial(portName, baudrate = 115200, bytesize = serial.EIGHTBITS, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE)
	
	# create a NSP32 instance
	nsp32 = NSP32(DataChannelSendData, OnReturnPacketReceived)
	
	# start the serial port receiving thread
	thread = threading.Thread(target = SerialPortReceive, args = (ser, nsp32))
	thread.daemon = True
	thread.start()
	
	# send commands
	nsp32.GetSensorId(0)
	nsp32.GetWavelength(0)
	nsp32.AcqSpectrum(0, 32, 3, False)	# integration time = 32; frame avg num = 3; disable AE
	
	# press ENTER to exit the program
	input()

def DataChannelSendData(data):
	"""'send data' delegate: to send data (commands) to NSP32 through serial port
	
	Args:
		data(bytearray): data (commands) to send
		
	"""
	
	global ser
	
	if ser.isOpen():
		ser.write(data)

def SerialPortReceive(ser, nsp32):
	"""serial port receiving thread
	
	Args:
		ser(serial.Serial): serial port instance
		nsp32(NSP32): NSP32 instance
		
	"""
	
	while ser.isOpen():
		if(ser.in_waiting > 0) :
			# feed received bytes to API, so that API can parse the return packet
			nsp32.OnReturnBytesReceived(ser.read(ser.in_waiting))

def OnReturnPacketReceived(pkt):
	"""'return packet received' delegate: to notify that a packet is received
	
	Args:
		pkt(ReturnPacket): the packet received
		
	"""
	
	if pkt.CmdCode == CmdCodeEnum.GetSensorId :		# GetSensorId
		print('sensor id = ' + pkt.ExtractSensorIdStr())
	elif pkt.CmdCode == CmdCodeEnum.GetWavelength :	# GetWavelength
		infoW = pkt.ExtractWavelengthInfo()
		print('first element of wavelength =', list(infoW.Wavelength))
		# TODO: get more information you need from infoW
	elif pkt.CmdCode == CmdCodeEnum.GetSpectrum :	# GetSpectrum
		infoS = pkt.ExtractSpectrumInfo()
		print('first element of spectrum =', infoS.Spectrum)
		# TODO: get more information you need from infoS


# start the program
main()
