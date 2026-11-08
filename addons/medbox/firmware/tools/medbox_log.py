import serial, sys, time
s = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=0.2)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
t0 = time.time(); end = t0 + float(sys.argv[1]); buf = b""
cmd = next((a for a in sys.argv[2:] if not a.startswith("--")), None)
auto_reset = "--auto-reset" in sys.argv
reset_at = None
while time.time() < end:
    buf += s.read(4096)
    if reset_at and time.time() >= reset_at:
        s.write(b"RESET\n"); print(f"{time.time()-t0:7.1f}s  >>> auto RESET", flush=True); reset_at = None
    if cmd and time.time() - t0 > 3:
        s.write((cmd + "\n").encode()); print(f"{time.time()-t0:7.1f}s  >>> sent {cmd}", flush=True); cmd = None
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        txt = line.decode(errors="replace").rstrip()
        if txt.startswith(("[", "=====")):
            print(f"{time.time()-t0:7.1f}s  {txt}", flush=True)
            if auto_reset and "BOX_COMPROMISED" in txt:
                reset_at = time.time() + 10
