#!/usr/bin/env python

import serial
import sys
import io

ser=serial.Serial(port='/dev/ttyACM0',baudrate=9600,timeout=0)

def servo_run(x,y):
	target = 132
	channel_x = 0
	channel_y = 1

	#tiltcom = x*4 & 0x7f
	#tiltcom2 = (x*4 >> 7) & 0x7f

	tilt_x = bytearray([target, channel_x, ((x*4) & 0x7f), (((x*4) >> 7) & 0x7f)])
	tilt_y = bytearray([target, channel_y, ((x*4) & 0x7f), (((x*4) >> 7) & 0x7f)])

	ser.write(tilt_y)
	# ser.write(serial_y)

	ser.close()

def main(args):
	servo_run(1000,1500)

if __name__ == '__main__':
    main(sys.argv)