import sys, time, serial, re
f    = sys.argv[1] if len(sys.argv)>1 else "5"
imax = sys.argv[2] if len(sys.argv)>2 else "800"
secs = float(sys.argv[3]) if len(sys.argv)>3 else 12.0
s = serial.Serial()
s.port='/dev/ttyACM0'; s.baudrate=115200; s.timeout=0.2
s.open(); s.dtr=False; s.rts=False
buf=b''
def pump(d):
    global buf
    t0=time.time()
    while time.time()-t0 < d: buf += s.read(4096)
def send(c,d=0.4):
    s.write(c.encode()+b"\r\n"); pump(d)
pump(1.0); send("stop",1.0); send("f "+f); send("i "+imax); send("run",0.4)
pump(secs); send("st",0.5); send("stop",1.0)
s.close()
sys.stdout.write(re.sub(r'\x1b\[[0-9;]*[a-zA-Z]','',buf.decode('utf-8','replace')))
