import sys, time, serial, re
freqs = [float(x) for x in sys.argv[1].split(',')]
imax  = sys.argv[2] if len(sys.argv) > 2 else "800"
settle = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
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
pump(1.0); send("stop",1.0); send("i "+imax)
send("f %g" % freqs[0]); send("run",0.4)
rows=[]
for f in freqs:
    send("f %g" % f, 0.4)
    buf=b''
    pump(settle)
    txt = re.sub(r'\x1b\[[0-9;]*[a-zA-Z]','',buf.decode('utf-8','replace'))
    lines=[l for l in txt.splitlines() if 'rpm=' in l]
    if not lines:
        rows.append((f,None)); continue
    L=lines[-1]
    def g(p):
        m=re.search(p,L); return m.group(1) if m else '0'
    rows.append((f, dict(rpm=g(r'rpm=\s*([+-]?[\d.]+)'),
                         vq =g(r'Vq=([+-]?[\d.]+)'),
                         id =g(r'Id=\s*([+-]?[\d.]+)'),
                         iq =g(r'Iq=\s*([+-]?[\d.]+)'),
                         ref=g(r'\(ref\s*([+-]?[\d.]+)\)'),
                         encf=g(r'enc \d+/(\d+)'))))
send("stop",1.0); s.close()
print("\n  f(Hz)  rpm    理论   Vq      Id      Iq     ref    enc_fail")
for f,r in rows:
    if r is None:
        print("%6.1f    --  %6.1f" % (f, f/7*60)); continue
    print("%6.1f %6.1f %6.1f  %+.3f %7.1f %7.1f %6.0f   %s"
          % (f, float(r['rpm']), f/7*60, float(r['vq']),
             float(r['id']), float(r['iq']), float(r['ref']), r['encf']))
