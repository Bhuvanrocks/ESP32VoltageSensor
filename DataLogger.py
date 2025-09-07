import serial
import time
import matplotlib.pyplot as plt
import pandas as pd
from collections import deque
from datetime import datetime

# ------------------------ CONFIG ------------------------
SERIAL_PORT = 'COM3'       # Replace with your ESP32 port
BAUD_RATE = 115200
LOG_INTERVAL = 60        # Save CSV every 3600 seconds (1 hour)
MAX_POINTS = 100            # Points to show in the plot window

# ------------------------ INIT --------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow ESP32 to reset

data1 = deque(maxlen=MAX_POINTS)
data2 = deque(maxlen=MAX_POINTS)
data3 = deque(maxlen=MAX_POINTS)
time_axis = deque(maxlen=MAX_POINTS)

# Create initial DataFrame
df = pd.DataFrame(columns=['Timestamp', 'Voltage1', 'Voltage2', 'Current'])

# Setup Matplotlib for interactive plotting
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Voltage1 (V)')
line2, = ax.plot([], [], label='Voltage2 (V)')
line3, = ax.plot([], [], label='Current (A)')
ax.set_xlabel('Samples')
ax.set_ylabel('Values')
ax.legend()
ax.grid(True)

last_save_time = time.time()
sample_count = 0

# ------------------------ MAIN LOOP ----------------------
try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        # Expecting 3 numeric values per line
        try:
            values = [float(v) for v in line.replace(',', ' ').split()]
            if len(values) != 3:
                continue
            v1, v2, c = values
        except:
            continue

        # Append to plotting data
        data1.append(v1)
        data2.append(v2)
        data3.append(c)
        time_axis.append(sample_count)
        sample_count += 1

        # Append to DataFrame with timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        df = pd.concat([df, pd.DataFrame([[timestamp, v1, v2, c]], columns=df.columns)], ignore_index=True)

        # Update plot
        line1.set_data(time_axis, data1)
        line2.set_data(time_axis, data2)
        line3.set_data(time_axis, data3)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.01)

        # Save CSV every LOG_INTERVAL seconds
        current_time = time.time()
        if current_time - last_save_time >= LOG_INTERVAL:
            filename = f'esp32_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
            df.to_csv(filename, index=False)
            print(f"Data saved to {filename}")
            last_save_time = current_time

except KeyboardInterrupt:
    # Save remaining data before exiting
    filename = f'esp32_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
    df.to_csv(filename, index=False)
    print(f"\nKeyboardInterrupt: Data saved to {filename}")
    ser.close()
