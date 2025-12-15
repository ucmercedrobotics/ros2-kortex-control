- Introduction
	There are two examples under this folder.
		1. Beginner
			A clean and simple example for beginners to start with NSP32. We'll demonstrate the basic usage of our API.
		2. ConsoleDemo
			A console program to demonstrate full functionalities of NSP32. Users can operate NSP32 by interactive console commands.
		3. SpectrumMeter
			A GUI program to visualize the spectrum measured by NSP32.

- *** hardware photos, screen shots ***

- Tested on
	Windows 7
	Windows 10
	macOS Mojave 10.14
	Ubuntu 16.04
	Ubuntu 18.04
	*** Windows 8 ***

- Hardware Setup
	If you are going to connect NSP32 through USB port on your desktop, you might need a "USB to TTL Serial Adapter that supports 3.3V" in advance.
	The following table shows the pin connections between NSP32 and the adapter.

	-------------------------
	  NSP32         Adapter  
	  Pin           Pin      
	-------------------------
	  VDD3V3        3V3      
	  GND           GND      
	  UART TX       UART RX
	  UART RX       UART TX

	If you are using NSP32 DBK board, make sure jumper J3 is ON (short).

- API Module Location
	Our API module file is put at [/examples/NanoLambdaNSP32.py].

- Software Setup
	1. Install Python 3.5 or above (Python 2 doesn't work).	
	2. All examples utilize the "pySerial" module, please visit [https://pythonhosted.org/pyserial/pyserial.html] for installation guidance.
	3. The "SpectrumMeter" example utilizes additional packages for GUI and plotting, please make sure the followings are installed under your environment.
		(1) tkinter
		(2) matplotlib

- Customization
	With the "Beginner" example, you need to modify the source code according to the serial port name your NSP32 is connected to.
	For your convenience, we have marked that code section with the title "modify this section to fit your need".

- Run the Example
	To run the examples, use Python commands:
		$ python Beginner.py
		$ python ConsoleDemo.py
		$ python SpectrumMeter.py
	
	Make sure to run the examples with Python 3. So under some environment, you might need to use:
		$ python3 Beginner.py
		$ python3 ConsoleDemo.py
		$ python3 SpectrumMeter.py

	Note:
		If you run into the "permission denied for opening serial port" problem on Ubuntu, try to run above commands as root.
		
	*** operations & behaviors (including the LED status, etc...) ***
