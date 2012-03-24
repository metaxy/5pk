import bpy
import time
import sys
sys.path.append("C:\Python32\Lib\site-packages")
import serial
import inspect
import numpy as np
from math import *


class IMU:
	ser = serial.Serial(2, 115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1, timeout=1, xonxoff=True)
	def start(this):
		this.ser.flush()
		this.ser.write('+'.encode('latin1'))
	def parse(this,a):
		return float(a.replace('+',''))
		
	def read(this):
		this.ser.flush()
		list = str(this.ser.readline(), encoding='utf8' ).split(' ')
		time = this.parse(list[0])
		v = [this.parse(list[3]),this.parse(list[4]),this.parse(list[5])]
		theta = [this.parse(list[8]),this.parse(list[9]),this.parse(list[10])]
		acc=[this.parse(list[13]),this.parse(list[14]),this.parse(list[15])]
		
		return [time,v,theta,acc]
	
	def update(this):
		this.read()
		this.read()
		for i in range(1000):
			time,v,theta,acc = this.read()
			if(i % 50 == 0):
				bpy.data.objects["Cone"].position_euler.x = theta[0]
				bpy.data.objects["Cone"].position_euler.y = theta[1]
				bpy.data.objects["Cone"].positionn_euler.z = theta[2]
				bpy.ops.render.render()
				bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
	def close(this):
		this.ser.close()
try:
	imu = IMU()
	imu.start()
	imu.update()
	imu.close();
except:
    imu.close()
    raise  #re-raise the exact same exception that was thrown

