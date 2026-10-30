import serial, time
PORT = "/dev/serial0"
ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(0.5)
ser.write(b'{"cmd":"STOP"}\n')
time.sleep(0.3)
deadline = time.time() + 6
count = 0
while time.time() < deadline:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        count += 1
        print(line)
print(f"\n--- {count} lines in 6 s ---")
ser.close()
