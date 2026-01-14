from scipy.signal import butter

# Given
fs = 94.74      # Hz
fc = 10         # Hz
order = 2       # 2nd order

# Design Butterworth filter
b, a = butter(N=order, Wn=fc, fs=fs, btype='low', analog=False)

print("Numerator (b):", b)
print("Denominator (a):", a)
