import serial
import time
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime

# ------------------------ CONFIG ------------------------
SERIAL_PORT = 'COM3'        # Replace with your ESP32 port
BAUD_RATE = 115200
LOG_INTERVAL = 1            # Log once every second
MAX_POINTS = 100            # Number of points to display in plots

# ------------------------ INIT --------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

# Data containers for plotting
timestamps = deque(maxlen=MAX_POINTS)
voltages1 = deque(maxlen=MAX_POINTS)
voltages2 = deque(maxlen=MAX_POINTS)
currents1 = deque(maxlen=MAX_POINTS)

# Setup Matplotlib
plt.ion()
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
fig.tight_layout(pad=4.0)

# Voltage1 vs Time
line_v1, = ax1.plot([], [], label='Voltage1 (V)')
ax1.set_title("Voltage1 vs Time")
ax1.set_xlabel("Time")
ax1.set_ylabel("Voltage1 (V)")
ax1.legend()
ax1.grid(True)

# Voltage2 vs Time
line_v2, = ax2.plot([], [], label='Voltage2 (V)')
ax2.set_title("Voltage2 vs Time")
ax2.set_xlabel("Time")
ax2.set_ylabel("Voltage2 (V)")
ax2.legend()
ax2.grid(True)

# Current1 vs Time
line_c1, = ax3.plot([], [], label='Current1 (A)')
ax3.set_title("Current1 vs Time")
ax3.set_xlabel("Time")
ax3.set_ylabel("Current1 (A)")
ax3.legend()
ax3.grid(True)

# Voltage1 vs Current1
line_v1c1, = ax4.plot([], [], label='Voltage1 vs Current1')
ax4.set_title("Voltage1 vs Current1")
ax4.set_xlabel("Voltage1 (V)")
ax4.set_ylabel("Current1 (A)")
ax4.legend()
ax4.grid(True)

# ------------------------ LOGGING ----------------------
filename = f'esp32_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
log_file = open(filename, 'w')
log_file.write('Timestamp,Voltage1,Voltage2,Current1,Power(W),Capacity(Ah)\n')
print(f"Logging to {filename}")

last_save_time = time.time()
last_capacity_time = time.time()
capacity = 0.0  # Ampere-hours accumulator

# ------------------------ MAIN LOOP ----------------------
try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        try:
            values = [float(v) for v in line.replace(',', ' ').split()]
            if len(values) != 3:
                continue
            voltage1, voltage2, current1 = values
        except:
            continue

        timestamp = datetime.now()

        # Store data for plotting
        timestamps.append(timestamp)
        voltages1.append(voltage1)
        voltages2.append(voltage2)
        currents1.append(current1)

        # Update all plots
        line_v1.set_data(timestamps, voltages1)
        ax1.relim()
        ax1.autoscale_view()

        line_v2.set_data(timestamps, voltages2)
        ax2.relim()
        ax2.autoscale_view()

        line_c1.set_data(timestamps, currents1)
        ax3.relim()
        ax3.autoscale_view()

        line_v1c1.set_data(voltages1, currents1)
        ax4.relim()
        ax4.autoscale_view()

        plt.pause(0.01)

        # Update capacity using time difference
        current_time = time.time()
        elapsed_time_hours = (current_time - last_capacity_time) / 3600.0
        capacity += current1 * elapsed_time_hours
        last_capacity_time = current_time

        # Log once every second
        if current_time - last_save_time >= LOG_INTERVAL:
            log_timestamp = timestamp.strftime('%Y-%m-%d %H:%M:%S')
            power = voltage1 * current1  # Power in Watts
            log_file.write(f"{log_timestamp},{voltage1:.2f},{voltage2:.2f},{current1:.3f},{power:.3f},{capacity:.6f}\n")
            log_file.flush()
            print(f"Logged: {log_timestamp}, V1={voltage1:.2f}V, V2={voltage2:.2f}V, I1={current1:.3f}A, P={power:.3f}W, Q={capacity:.6f}Ah")
            last_save_time = current_time

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected. Closing log file.")
    log_file.close()
    ser.close()
    print("Program terminated.")
