from datetime import datetime
import serial

ser = serial.Serial('COM5', 9600, timeout=1)
while True:
    line = ser.readline()
    line = str(line)
    if line != "b''":
        line = line.replace("b'", "")
        line = line.replace("'","")
        now = datetime.now()
        line = line.replace("#t#", now.strftime("%d/%m/%Y %H:%M:%S"))
        line.splitlines(True)
        line = line.replace("\\r\\n","")
        print (line)
        with open('pico.log', 'a+') as f:
            f.write(line+'\n')
