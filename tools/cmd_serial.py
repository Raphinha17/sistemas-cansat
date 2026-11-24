import serial, time, os, sys
puerto = os.environ.get("TX", "/dev/ttyUSB1")
p = serial.Serial(puerto, 115200, timeout=1)
time.sleep(2)
p.reset_input_buffer()
for c in sys.argv[1:]:
    p.write(c.encode() + b"\n")
    time.sleep(1.2)
    print(f">>> {c}")
    print(p.read(4000).decode(errors="replace"))
