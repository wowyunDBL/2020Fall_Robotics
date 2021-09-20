import serial
ser = serial.Serial('/dev/ttyUSB1',57600,timeout = 0.1)
while 1:
	a=raw_input("a-1: ")
	ser.write((a+",000,000").encode())
