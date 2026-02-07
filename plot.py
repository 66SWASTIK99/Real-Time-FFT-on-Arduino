import serial
import struct
import matplotlib.pyplot as plt
import time
import numpy as np
from scipy.interpolate import make_interp_spline

# Configuration
PORT = 'COM3'       # Change to your Arduino port
BAUD = 230400
N = 64              # Number of floats per batch
START_BYTE = 0xAA

# Double buffer
buffer0 = [0.0] * N
buffer1 = [0.0] * N
readBuf = 0
writeBuf = 1

# Open serial port
ser = serial.Serial(PORT, BAUD, timeout=1)

# Function to read a full batch of floats
def read_batch():
    """
    Waits for a start byte and reads N floats (full batch).
    Returns True only if a full batch was successfully read.
    """
    global readBuf, writeBuf, buffer0, buffer1
    
    # wait for start byte
    while True:
        b = ser.read(1)
        if not b:
            return False  # no data, return to loop
        if b[0] == START_BYTE:
            break

    # read N floats (4 bytes each)
    data = ser.read(4 * N)
    if len(data) != 4 * N:
        print("Incomplete batch, ignoring...")
        return False

    floats = struct.unpack('<' + 'f'*N, data)  # little endian

    # write to write buffer
    if writeBuf == 0:
        buffer0[:] = floats
    else:
        buffer1[:] = floats

    # Swap buffers
    readBuf, writeBuf = writeBuf, readBuf
    return True


# setup Matplotlib for fixed-window plot
plt.ion()
fig, ax = plt.subplots()
x = [i * 4 for i in range(N)]
line, = ax.plot(x, [0]*N)  # initial empty line
ax.set_xticks(x[::5])

ax.set_ylim(0, 200)  # adjust according to your expected data range
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude")
ax.set_title("FFT Plot")

# main loop
try:
    while True:
        # only update the plot if a full batch is received
        if read_batch():
            current_data = buffer0 if readBuf == 0 else buffer1
            x_array = np.array(x)
            y_array = np.array(current_data)

            # create spline function
            spl = make_interp_spline(x_array, y_array, k=3)  
            x_smooth = np.linspace(min(x), max(x), 4*N)  
            y_smooth = spl(x_smooth)

            line.set_xdata(x_smooth)
            line.set_ydata(y_smooth)

            fig.canvas.draw()
            fig.canvas.flush_events()
        else:
            # no complete batch, keep previous plot
            time.sleep(0.01)

except KeyboardInterrupt:
    ser.close()