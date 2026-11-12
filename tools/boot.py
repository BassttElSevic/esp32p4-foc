import time, sys, serial, re
s = serial.Serial()
s.port = '/dev/ttyACM0'
s.baudrate = 115200
s.timeout = 0.3
s.open()
s.dtr = False
s.rts = False
time.sleep(0.3)
s.rts = True
time.sleep(0.15)
s.rts = False
buf = b''
end = time.time() + 10
while time.time() < end:
    buf += s.read(2048)
s.close()
txt = re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', buf.decode('utf-8', 'replace'))
sys.stdout.write(txt)
