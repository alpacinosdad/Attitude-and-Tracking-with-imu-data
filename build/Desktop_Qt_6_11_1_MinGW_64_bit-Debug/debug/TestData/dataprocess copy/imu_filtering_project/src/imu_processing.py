import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def read_csv_file(file_path):
    """Read CSV file and return a DataFrame."""
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()  # Clean column names
    df = df.replace(r'^\s*$', np.nan, regex=True)  # Replace blanks with NaN
    return df

def filter_data_low_pass(data, cutoff_frequency, sampling_rate, order=4):
    """Apply a low-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_frequency / nyquist
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
    return signal.filtfilt(b, a, data)

def filter_data_high_pass(data, cutoff_frequency, sampling_rate, order=4):
    """Apply a high-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_frequency / nyquist
    b, a = signal.butter(order, normal_cutoff, btype='high', analog=False)
    return signal.filtfilt(b, a, data)

def filter_data_band_pass(data, low_cutoff, high_cutoff, sampling_rate, order=4):
    """Apply a band-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    low = low_cutoff / nyquist
    high = high_cutoff / nyquist
    b, a = signal.butter(order, [low, high], btype='band', analog=False)
    return signal.filtfilt(b, a, data)

def plot_data(time, data, title, xlabel='Time (s)', ylabel='Value', ylim=None):
    """Plot the data."""
    plt.figure(figsize=(14, 7))
    plt.plot(time, data)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if ylim:
        plt.ylim(ylim)
    plt.grid(True)
    plt.show()

# Example usage
if __name__ == "__main__":
    file_path = "imu_processed.csv"  # Update with your file path
    df = read_csv_file(file_path)

    # Assuming the DataFrame has columns 'time_sec', 'Accel_x', 'Accel_y', 'Accel_z'
    sampling_rate = 50  # Hz
    cutoff_frequency = 5  # Hz for low-pass
    order = 4

    # Apply filters
    df['Accel_x_low_pass'] = filter_data_low_pass(df['Accel_x'], cutoff_frequency, sampling_rate, order)
    df['Accel_x_high_pass'] = filter_data_high_pass(df['Accel_x'], cutoff_frequency, sampling_rate, order)

    # Plot results
    plot_data(df['time_sec'], df['Accel_x'], 'Raw Acceleration X')
    plot_data(df['time_sec'], df['Accel_x_low_pass'], 'Low-Pass Filtered Acceleration X')
    plot_data(df['time_sec'], df['Accel_x_high_pass'], 'High-Pass Filtered Acceleration X')