import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_dual_fft(csv_path, original_col='velocity', filtered_col=None, time_column='time', time_unit='ms'):
    # Load the data
    df = pd.read_csv(csv_path)
    original = df[original_col].to_numpy()
    time = df[time_column].to_numpy()

    # Convert time to seconds if needed
    if time_unit == 'ms':
        time = time * 1e-3
    dt = np.mean(np.diff(time))  # Time step
    fs = 1.0 / dt  # Sampling frequency

    # Perform FFT
    n = len(original)
    fft_freqs = np.fft.fftfreq(n, d=dt)

    # Only take positive frequencies
    idx = fft_freqs >= 0
    freqs = fft_freqs[idx]

    # Compute FFT for original signal
    orig_fft = np.abs(np.fft.fft(original))[idx] / n
    orig_db = 20 * np.log10(orig_fft + 1e-12)  # add epsilon to avoid log(0)

    # Plot the original signal FFT
    plt.figure(figsize=(10, 5))
    plt.plot(freqs, orig_db, label='Original Velocity', color='blue')

    # If filtered column is provided, plot its FFT as well
    if filtered_col is not None:
        filtered = df[filtered_col].to_numpy()
        filt_fft = np.abs(np.fft.fft(filtered))[idx] / n
        filt_db = 20 * np.log10(filt_fft + 1e-12)
        plt.plot(freqs, filt_db, label='Filtered Velocity', linestyle='--', color='orange')

    # Customize plot
    plt.title("Frequency Spectrum of Velocity (dB)")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return freqs, orig_db

csv_path = "data_Jun_2025/Acq_IEEE_SPACE/2025-07-03_21-26-19_10deg/sensor_data_filtered.csv"
plot_dual_fft(csv_path, original_col='velocity', filtered_col= "Filtered Velocity", time_column='time', time_unit='ms')
