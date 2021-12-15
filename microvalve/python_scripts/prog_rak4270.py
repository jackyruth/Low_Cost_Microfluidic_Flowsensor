import subprocess
import serial
filename = "RAK4270.bin"
ser = serial.Serial("/dev/ttyUSB0",115200,timeout=1)
print(ser.name)


ser.write(b'at+version\r\n')
print(ser.readline())
ser.write(b'at+boot\r\n')
print(ser.readline())
ser.write(b'at+update\r\n')
print(ser.readline())
ser.close()


subprocess.call(["sudo","bash","ysend", filename])
