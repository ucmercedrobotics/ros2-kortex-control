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

import enum
import struct
import queue

"""
.. module:: NanoLambdaNSP32
   :synopsis: NSP32 Python API for desktop
.. moduleauthor:: nanoLambda, Inc.
"""

class CmdCodeEnum(enum.IntEnum):
	"""command code enumeration"""

	Unknown			= 0x00		#: unknown
	Prefix0			= 0x03		#: prefix 0
	Prefix1			= 0xBB		#: prefix 1

	Hello			= 0x01		#: hello
	Standby			= 0x04		#: standby
	GetSensorId		= 0x06		#: get sensor id
	GetWavelength	= 0x24		#: get wavelength
	AcqSpectrum		= 0x26		#: spectrum acquisition
	GetSpectrum		= 0x28		#: get spectrum data
	AcqXYZ			= 0x2A		#: XYZ acquisition
	GetXYZ			= 0x2C		#: get XYZ data


class WavelengthInfo:
	"""wavelength info"""

	def __init__(self, packetBytes):
		"""__init__ method
		
		Args:
			packetBytes(bytearray): packet data bytes

		"""

		self._packetBytes = packetBytes
		self._numOfPoints = struct.unpack('<I', self._packetBytes[4:8])[0]		# num of points

	@property
	def NumOfPoints(self):
		"""int: num of points"""
		
		return self._numOfPoints

	@property
	def Wavelength(self):
		"""tuple: wavelength data"""
		
		# convert received bytes to short data
		return struct.unpack('<' + 'H' * self._numOfPoints, self._packetBytes[8 : 8 + self._numOfPoints * 2])


class SpectrumInfo:
	"""spectrum info"""

	def __init__(self, packetBytes):
		"""__init__ method
		
		Args:
			packetBytes(bytearray): packet data bytes

		"""

		self._packetBytes = packetBytes
		self._numOfPoints = struct.unpack('<I', self._packetBytes[8:12])[0]	# num of points

	@property
	def NumOfPoints(self):
		"""int: num of points"""

		return self._numOfPoints

	@property
	def IntegrationTime(self):
		"""int: integration time"""
		
		return struct.unpack('<H', self._packetBytes[4:6])[0]

	@property
	def IsSaturated(self):
		"""bool: saturation flag (True for saturated; False for not saturated)"""
		
		return self._packetBytes[6] == 1

	@property
	def Spectrum(self):
		"""tuple: spectrum data"""
		
		# convert received bytes to float data
		return struct.unpack('<' + 'f' * self._numOfPoints, self._packetBytes[12 : 12 + self._numOfPoints * 4])

	@property
	def X(self):
		"""float: X"""
		
		return struct.unpack('<f', self._packetBytes[12 + 135 * 4 : 12 + 135 * 4 + 4])[0]

	@property
	def Y(self):
		"""float: Y"""
		
		return struct.unpack('<f', self._packetBytes[12 + 135 * 4 + 4 : 12 + 135 * 4 + 8])[0]

	@property
	def Z(self):
		"""float: Z"""
		
		return struct.unpack('<f', self._packetBytes[12 + 135 * 4 + 8 : 12 + 135 * 4 + 12])[0]


class XYZInfo:
	"""XYZ info"""

	def __init__(self, packetBytes):
		"""__init__ method
		
		Args:
			packetBytes(bytearray): packet data bytes

		"""

		self._packetBytes = packetBytes

	@property
	def IntegrationTime(self):
		"""int: integration time"""
		
		return struct.unpack('<H', self._packetBytes[4:6])[0]

	@property
	def IsSaturated(self):
		"""bool: saturation flag (True for saturated; False for not saturated)"""
		
		return self._packetBytes[6] == 1

	@property
	def X(self):
		"""float: X"""

		return struct.unpack('<f', self._packetBytes[8:12])[0]

	@property
	def Y(self):
		"""float: Y"""

		return struct.unpack('<f', self._packetBytes[12:16])[0]

	@property
	def Z(self):
		"""float: Z"""

		return struct.unpack('<f', self._packetBytes[16:20])[0]


class ReturnPacket:
	"""return packet"""

	def __init__(self, cmdCode, userCode, isPacketValid, packetBytes):
		"""__init__ method
		
		Args:
			cmdCode(CmdCodeEnum): command function code

			userCode(int): command user code

			isPacketValid(bool): True for valid packet; False for invalid packet

			packetBytes(bytearray): packet data bytes
			
		"""

		self._cmdCode = cmdCode
		self._userCode = userCode
		self._isPacketValid = isPacketValid
		self._packetBytes = packetBytes

	@property
	def CmdCode(self):
		"""CmdCodeEnum: command function code"""
		
		return self._cmdCode

	@property
	def UserCode(self):
		"""int: command user code"""

		return self._userCode

	@property
	def IsPacketValid(self):
		"""bool: check if the packet is valid (True for valid; False for invalid)"""

		return self._isPacketValid

	@property
	def PacketBytes(self):
		"""bytearray: packet data bytes"""
	
		return self._packetBytes

	def ExtractSensorIdStr(self):
		"""extract sensor id string from the return packet
		
		Returns:
			str: sensor id string (return None if the packet type mismatches)

		"""
		
		return '-'.join( [ "%02X" % x for x in self._packetBytes[4:9] ] ) if self._cmdCode == CmdCodeEnum.GetSensorId else None

	def ExtractWavelengthInfo(self):
		"""extract wavelength info from the return packet
		
		Returns:
			WavelengthInfo: wavelength info (return None if the packet type mismatches)

		"""
		
		return WavelengthInfo(self._packetBytes) if self._cmdCode == CmdCodeEnum.GetWavelength else None

	def ExtractSpectrumInfo(self):
		"""extract spectrum info from the return packet
		
		Returns:
			SpectrumInfo: spectrum info (return None if the packet type mismatches)

		"""
		
		return SpectrumInfo(self._packetBytes) if self._cmdCode == CmdCodeEnum.GetSpectrum else None

	def ExtractXYZInfo(self):
		"""extract XYZ info from the return packet
		
		Returns:
			XYZInfo: XYZ info (return None if the packet type mismatches)

		"""
		
		return XYZInfo(self._packetBytes) if self._cmdCode == CmdCodeEnum.GetXYZ else None


class NSP32:
	"""NSP32 main class"""

	# command length table
	__CmdLen = 																			\
	[																					\
		   0,  5,  0,  0,  5,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x00~0x0F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x10~0x1F	\
		   0,  0,  0,  0,  5,  0, 10,  0,  5,  0, 10,  0,  5,  0,  0,  0 ,  # 0x20~0x2F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x30~0x3F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x40~0x4F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0x50~0x5F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x60~0x6F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x70~0x7F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0x80~0x8F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x90~0x9F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xA0~0xAF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0xB0~0xBF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xC0~0xCF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0xD0~0xDF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0xE0~0xEF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0	# 0xF0~0xFF	\
	]

	# return packet length table
	__RetPacketLen =																	\
	[																					\
		   0,  5,  0,  0,  5,  0, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x00~0x0F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x10~0x1F	\
		   0,  0,  0,  0,279,  0,  5,  0,565,  0,  5,  0, 21,  0,  0,  0 ,	# 0x20~0x2F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x30~0x3F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x40~0x4F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x50~0x5F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x60~0x6F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x70~0x7F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x80~0x8F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0x90~0x9F	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xA0~0xAF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xB0~0xBF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xC0~0xCF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,  # 0xD0~0xDF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 ,	# 0xE0~0xEF	\
		   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 	# 0xF0~0xFF	\
	]

	__CmdBufSize = max(__CmdLen)			# command buffer size
	__RetBufSize = max(__RetPacketLen)		# return packet buffer size

	def __init__(self, sendDataDelegate, returnPacketReceivedDelegate):
		"""__init__ method

		Args:
			sendDataDelegate(function): "send data" delegate

			returnPacketReceivedDelegate(function): "return packet received" delegate

		"""

		self._cmdBuf = bytearray(NSP32.__CmdBufSize)	# command buffer
		self._retBuf = bytearray(NSP32.__RetBufSize)	# return packet buffer
		self._retBufIdx = 0								# return packet buffer write index
		self._invalidPacketReceived = False				# "invalid packet received" flag

		self._cmdQueue = queue.Queue()					# command queue
		self._isWaitingCmdReturn = False				# "is waiting command return" flag

		self._dataChannelSendDataDelegate = sendDataDelegate					# "send data" delegate: to send data (commands) to NSP32 through data channel (e.g. UART, BLE, WIFI, ...)
		self._onReturnPacketReceivedDelegate = returnPacketReceivedDelegate		# "return packet received" delegate: to notify the listener that a packet is received
		
	def Hello(self, userCode):
		"""say hello to NSP32
		
		Args:
			userCode(int): user code
	
		"""

		self._QueueCmd(CmdCodeEnum.Hello, userCode)

	def Standby(self, userCode):
		"""standby NSP32
		
		Args:
			userCode(int): user code
	
		"""

		self._QueueCmd(CmdCodeEnum.Standby, userCode)

	def GetSensorId(self, userCode):
		"""get sensor id
		
		Args:
			userCode(int): user code
	
		"""

		self._QueueCmd(CmdCodeEnum.GetSensorId, userCode)

	def GetWavelength(self, userCode):
		"""get wavelength
		
		Args:
			userCode(int): user code
	
		"""

		self._QueueCmd(CmdCodeEnum.GetWavelength, userCode)

	def AcqSpectrum(self, userCode, integrationTime, frameAvgNum, enableAE):
		"""start spectrum acquisition
		
		We will let NSP32 actively send out the "GetSpectrum" return packet once the acquisition is done, so there is no GetSpectrum() function in this API.
		
		Args:
			userCode(int): user code

			integrationTime(int): integration time

			frameAvgNum(int): frame average num

			enableAE(bool): True to enable AE; False to disable AE

		"""

		self._cmdBuf[4] = integrationTime & 0xFF
		self._cmdBuf[5] = integrationTime >> 8
		self._cmdBuf[6] = frameAvgNum
		self._cmdBuf[7] = 1 if enableAE else 0
		self._cmdBuf[8] = 1		# active return

		self._QueueCmd(CmdCodeEnum.AcqSpectrum, userCode)

	def AcqXYZ(self, userCode, integrationTime, frameAvgNum, enableAE):
		"""start XYZ acquisition
		
		We will let NSP32 actively send out the "GetXYZ" return packet once the acquisition is done, so there is no GetXYZ() function in this API.
		
		Args:
			userCode(int): user code

			integrationTime(int): integration time

			frameAvgNum(int): frame average num

			enableAE(bool): True to enable AE; False to disable AE

		"""

		self._cmdBuf[4] = integrationTime & 0xFF
		self._cmdBuf[5] = integrationTime >> 8
		self._cmdBuf[6] = frameAvgNum
		self._cmdBuf[7] = 1 if enableAE else 0
		self._cmdBuf[8] = 1		# active return
		
		self._QueueCmd(CmdCodeEnum.AcqXYZ, userCode)

	def OnReturnByteReceived(self, rcv):
		"""byte received handler (call this function when receiving a single byte from data channel)
		
		Args:
			rcv(int): single byte received

		"""

		# copy received byte into return packet buffer
		if self._retBufIdx < NSP32.__RetBufSize :
			self._retBuf[self._retBufIdx] = rcv
			self._retBufIdx += 1

			# parse the return packet buffer (to see if a valid return packet is received)
			self._ParseRetBuf()

	def OnReturnBytesReceived(self, data):
		"""bytes received handler (call this function when receiving multiple bytes from data channel)
		
		Args:
			data(list): bytes received

		"""

		# copy received bytes into return packet buffer
		end = self._retBufIdx + len(data)
		
		if end <= NSP32.__RetBufSize :
			self._retBuf[self._retBufIdx : end] = data
			self._retBufIdx = end	# update return packet buffer write index

			# parse the return packet buffer (to see if a valid return packet is received)
			self._ParseRetBuf()

	def _ParseRetBuf(self):
		"""parse the return packet buffer (to see if a valid return packet is received)"""

		pkt = None

		# if prefix codes are not aligned, notify the listener an invalid packet is received
		if (self._retBufIdx == 1 and self._retBuf[0] != CmdCodeEnum.Prefix0) or (self._retBufIdx == 2 and self._retBuf[1] != CmdCodeEnum.Prefix1) :
			if not self._invalidPacketReceived :
				self._invalidPacketReceived = True
				pkt = ReturnPacket(CmdCodeEnum.Unknown, 0, False, self._ExtractPacketBytes())

			self._retBufIdx = 0		# reset return packet buffer write index
	
		# determine the expected packet length based on command function code
		returnLen = 0

		if self._retBufIdx >= 3 :
			returnLen = NSP32.__RetPacketLen[self._retBuf[2]]

			# if we get an unrecognized command function code, notify the listener an invalid packet is received				
			if returnLen == 0 :
				if not self._invalidPacketReceived :
					self._invalidPacketReceived = True
					pkt = ReturnPacket(CmdCodeEnum.Unknown, 0, False, self._ExtractPacketBytes())

				self._retBufIdx = 0    # reset return packet buffer write index
		
		# if a whole packet is received, process it
		if returnLen > 0 and self._retBufIdx >= returnLen :
			self._invalidPacketReceived = False
			isChecksumValid = self._IsChecksumValid(self._retBuf, returnLen)

			# prepare return packet object
			pkt = ReturnPacket(CmdCodeEnum(self._retBuf[2]), self._retBuf[3], isChecksumValid, self._ExtractPacketBytes())

			# reset return packet buffer for next packet
			self._retBufIdx = 0

		if pkt is not None :
			# notify the listener a return packet is received
			if self._onReturnPacketReceivedDelegate is not None :
				self._onReturnPacketReceivedDelegate(pkt)

			# send queued commands
			self._isWaitingCmdReturn = False
			self._SendQueuedCmd()

	def _SendQueuedCmd(self):
		"""send out queued command (all commands from main application will be queued, and then be sent to NSP32 after the previous one is returned)"""

		# if the previous command is not yet returned, do nothing, otherwise send the next command to NSP32 through data channel
		if (not self._isWaitingCmdReturn) and (not self._cmdQueue.empty()) :
			self._isWaitingCmdReturn = True
			self._dataChannelSendDataDelegate(self._cmdQueue.get())

	def _QueueCmd(self, cmdCode, userCode):
		"""queue command
		
		Args:
			cmdCode(CmdCodeEnum): command function code

			userCode(int): command user code
	
		"""

		cmdLen = NSP32.__CmdLen[cmdCode]
		
		self._cmdBuf[0] = CmdCodeEnum.Prefix0
		self._cmdBuf[1] = CmdCodeEnum.Prefix1
		self._cmdBuf[2] = cmdCode
		self._cmdBuf[3] = userCode
		self._PlaceChecksum(self._cmdBuf, cmdLen - 1)		# add checksum

		# queue the command (then the queued commands will be sent one by one)
		self._cmdQueue.put(self._cmdBuf[0 : cmdLen])
		
		# check if we can send this command immediately
		self._SendQueuedCmd()

	def _ExtractPacketBytes(self):
		"""extract packet bytes from return packet buffer
		
		Returns:
			bytearray: packet bytes

		"""

		return None if self._retBufIdx <= 0 else self._retBuf[0 : self._retBufIdx]

	def _PlaceChecksum(self, buf, len):
		"""calculate checksum and append it to the end of the buffer (use "modular sum" method)
		
		Args:
			buf(bytearray): buffer

			len(int): data length (excluding the checksum)

		"""

		# sum all bytes
		s = sum(buf[0 : len])
		
		# take two's complement, and append the checksum to the end
		buf[len] = ((~s) + 1) & 0xFF

	def _IsChecksumValid(self, buf, len):
		"""check if the checksum is valid (use "modular sum" method)
		
		Args:
			buf(bytearray): buffer

			len(int): data length (including the checksum)
			
		Returns:
			bool: True for valid; False for invalid

		"""

		# sum all bytes (including the checksum byte)
		# if the summation equals 0, the checksum is valid
		return (sum(buf[0 : len]) & 0xFF) == 0
