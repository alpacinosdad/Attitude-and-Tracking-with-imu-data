### 1. Python Script: `data_processing.py`

This script will contain functions for reading data, filtering, and plotting.

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def read_csv_file(file_path):
    """Read a CSV file and return a DataFrame."""
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()  # Clean column names
    df = df.replace(r'^\s*$', np.nan, regex=True)  # Replace blanks with NaN
    return df

def apply_low_pass_filter(data, cutoff_frequency, sampling_rate, order=4):
    """Apply a low-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_frequency / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

def apply_high_pass_filter(data, cutoff_frequency, sampling_rate, order=4):
    """Apply a high-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_frequency / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

def apply_band_pass_filter(data, lowcut, highcut, sampling_rate, order=4):
    """Apply a band-pass filter to the data."""
    nyquist = 0.5 * sampling_rate
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

def plot_accelerometer_data(time, accel_x, accel_y, accel_z, title="Accelerometer Data"):
    """Plot accelerometer data."""
    plt.figure(figsize=(14, 7))
    plt.plot(time, accel_x, label='Accel_x', color='blue', linewidth=0.8)
    plt.plot(time, accel_y, label='Accel_y', color='green', linewidth=0.8)
    plt.plot(time, accel_z, label='Accel_z', color='red', linewidth=0.8)
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()
```

### 2. Jupyter Notebook: `data_processing_notebook.ipynb`

This notebook will contain separate code blocks for different functionalities, including import statements, parameter configuration, filtering function calls, and plotting function calls.

```json
{
 "cells": [
  {
   "cell_type": "code",
   "metadata": {},
   "source": [
    "import pandas as pd\n",
    "import numpy as np\n",
    "import matplotlib.pyplot as plt\n",
    "from scipy.signal import butter, filtfilt\n",
    "\n",
    "def read_csv_file(file_path):\n",
    "    df = pd.read_csv(file_path)\n",
    "    df.columns = df.columns.str.strip()\n",
    "    df = df.replace(r'^\s*$', np.nan, regex=True)\n",
    "    return df\n",
    "\n",
    "def apply_low_pass_filter(data, cutoff_frequency, sampling_rate, order=4):\n",
    "    nyquist = 0.5 * sampling_rate\n",
    "    normal_cutoff = cutoff_frequency / nyquist\n",
    "    b, a = butter(order, normal_cutoff, btype='low', analog=False)\n",
    "    return filtfilt(b, a, data)\n",
    "\n",
    "def apply_high_pass_filter(data, cutoff_frequency, sampling_rate, order=4):\n",
    "    nyquist = 0.5 * sampling_rate\n",
    "    normal_cutoff = cutoff_frequency / nyquist\n",
    "    b, a = butter(order, normal_cutoff, btype='high', analog=False)\n",
    "    return filtfilt(b, a, data)\n",
    "\n",
    "def apply_band_pass_filter(data, lowcut, highcut, sampling_rate, order=4):\n",
    "    nyquist = 0.5 * sampling_rate\n",
    "    low = lowcut / nyquist\n",
    "    high = highcut / nyquist\n",
    "    b, a = butter(order, [low, high], btype='band')\n",
    "    return filtfilt(b, a, data)\n"
   ]
  },
  {
   "cell_type": "code",
   "metadata": {},
   "source": [
    "# Parameter Configuration\n",
    "file_path = 'imu_processed_index_time.csv'\n",
    "sampling_rate = 50  # Hz\n",
    "cutoff_frequency_low = 5  # Hz\n",
    "cutoff_frequency_high = 1  # Hz\n",
    "\n",
    "# Read Data\n",
    "df = read_csv_file(file_path)\n",
    "df['UTC_Time'] = pd.to_datetime(df['UTC_Time'])\n",
    "df = df.sort_values('UTC_Time')\n",
    "time = df['time_sec_index']\n",
    "accel_x_raw = df['Accel_x_ms2']\n",
    "accel_y_raw = df['Accel_y_ms2']\n",
    "accel_z_raw = df['Accel_z_ms2']\n"
   ]
  },
  {
   "cell_type": "code",
   "metadata": {},
   "source": [
    "# Gravity Removal using Low-Pass Filter\n",
    "accel_x_low_pass = apply_low_pass_filter(accel_x_raw, cutoff_frequency_low, sampling_rate)\n",
    "accel_y_low_pass = apply_low_pass_filter(accel_y_raw, cutoff_frequency_low, sampling_rate)\n",
    "accel_z_low_pass = apply_low_pass_filter(accel_z_raw, cutoff_frequency_low, sampling_rate)\n",
    "\n",
    "# Gravity Removal using High-Pass Filter\n",
    "accel_x_high_pass = apply_high_pass_filter(accel_x_raw, cutoff_frequency_high, sampling_rate)\n",
    "accel_y_high_pass = apply_high_pass_filter(accel_y_raw, cutoff_frequency_high, sampling_rate)\n",
    "accel_z_high_pass = apply_high_pass_filter(accel_z_raw, cutoff_frequency_high, sampling_rate)\n",
    "\n",
    "# Gravity Removal using IIR Method\n",
    "order = 4\n",
    "b, a = butter(order, cutoff_frequency_low / (0.5 * sampling_rate), btype='low')\n",
    "accel_x_iir = filtfilt(b, a, accel_x_raw)\n",
    "accel_y_iir = filtfilt(b, a, accel_y_raw)\n",
    "accel_z_iir = filtfilt(b, a, accel_z_raw)\n"
   ]
  },
  {
   "cell_type": "code",
   "metadata": {},
   "source": [
    "# Plotting the Results\n",
    "plt.figure(figsize=(14, 7))\n",
    "plt.plot(time, accel_x_raw, label='Raw Accel_x', color='gray', alpha=0.5)\n",
    "plt.plot(time, accel_x_low_pass, label='Low-Pass Accel_x', color='blue')\n",
    "plt.plot(time, accel_x_high_pass, label='High-Pass Accel_x', color='red')\n",
    "plt.plot(time, accel_x_iir, label='IIR Accel_x', color='green')\n",
    "plt.xlabel('Time (s)')\n",
    "plt.ylabel('Acceleration (m/s²)')\n",
    "plt.title('Gravity Removal Methods for Accel_x')\n",
    "plt.legend()\n",
    "plt.grid(True)\n",
    "plt.show()\n"
   ]
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 3",
   "language": "python",
   "name": "python3"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.8.5"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 4
}
```

### Summary of Changes
- **Python Script (`data_processing.py`)**: Contains functions for reading CSV files, applying low-pass, high-pass, and band-pass filters, and plotting accelerometer data.
- **Jupyter Notebook (`data_processing_notebook.ipynb`)**: Organized into separate code blocks for imports, parameter configuration, gravity removal using different methods, and plotting results. The gravity removal operations are included directly in the notebook without encapsulation in functions.

This structure ensures clarity, readability, and modularity while adhering to your requirements.