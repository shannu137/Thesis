import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the Excel file
excel_path = "data_Jun_2025/Acq_IEEE_SPACE/2025-07-03_18-21-06_0deg/sensor_data_filter.csv"
df = pd.read_csv(excel_path)

# Extract time and velocity signal
time = df['time'].to_numpy()            # assume in milliseconds
velocity = df['velocity'].to_numpy()

# Convert time to seconds and estimate sampling frequency
time_s = time * 1e-3                    # convert ms to seconds
dt = np.diff(time_s)
fs = 1 / np.mean(dt)                   # sampling frequency in Hz
print(f"Estimated Sampling Rate: {fs:.2f} Hz")

# Compute FFT
n = len(velocity)
fft_vals = np.fft.fft(velocity)
fft_freqs = np.fft.fftfreq(n, 1/fs)

# Keep only positive frequencies
pos_mask = fft_freqs >= 0
freqs = fft_freqs[pos_mask]
magnitude = np.abs(fft_vals[pos_mask]) / n

# Plot frequency content
plt.figure(figsize=(10, 5))
plt.plot(freqs, magnitude)
plt.title("Frequency Content of velocity_serial")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.tight_layout()
plt.show()
