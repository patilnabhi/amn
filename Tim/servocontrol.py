
import serial

def control():

    ser = serial.Serial('/dev/ttyACM0')
    

    paninput = int(raw_input("enter pan value> "))
    tiltinput = int(raw_input("enter tilt value> "))

    pancom = paninput*4 & 0x7f
    pancom2 = (paninput*4 >> 7) & 0x7f

    tiltcom = tiltinput*4 & 0x7f
    tiltcom2 = (tiltinput*4 >>7) & 0x7f

    #print pancom, pancom2
    #print tiltcom, tiltcom2

    panpos = bytearray([132,1,pancom,pancom2])
    tiltpos = bytearray([132,0,tiltcom,tiltcom2])

    ser.write(panpos)
    ser.write(tiltpos)

control()
