import serial
import sys

def servo_run(x,y):
	target = 132
	channel_x = 0
	channel_y = 1

	ser = serial.Serial('/dev/ttyS1', 9600, timeout=0)

	serial_x = bytearray([target, channel_x, ((x*4) & 0x7F), (((x*4) >> 7) & 0x7F)])
	serial_y = bytearray([target, channel_y, ((x*4) & 0x7F), (((x*4) >> 7) & 0x7F)])

	ser.write(serial_x)
	ser.write(serial_y)

def main(args):
	servo_run(1500,1500)

if __name__ == '__main__':
    main(sys.argv)