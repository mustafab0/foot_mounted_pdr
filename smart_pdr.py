from math import sqrt, atan2, pi, cos, sin
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


f_name = 'imu_data_20230907_161844.json'
f_name_ftm = 'ftm_5.txt'

#Variables
GYRO_THRESHOLDX = 125 #walk
GYRO_THRESHOLDY = 140 #walk
GYRO_THRESHOLDZ = 125 #walk

HGRAV_THRESHOLDY= 0.65*9.8 #walk
HGRAV_THRESHOLDX= 0.65*9.8 #walk
HGRAV_THRESHOLDZ= 0.65*9.8 #walk

SGRAV_THRESHOLDX= 0.2*9.8 #walk
SGRAV_THRESHOLDY= 0.2*9.8 #walk
SGRAV_THRESHOLDZ= 0.2*9.8 #walk

dt = 0.0125
acc_noise = 0.1
gyr_noise = 0.01
g = np.array([[0],[0],[9.8]])
r_noise = 0.001
fsize = (9,6)
m_noise = 0.03
b_noise = 20
ekf = 1 #ekf 1: HDR correction; ekf 0: mag and gyro

data = pd.read_json(f'C:/Users/workshop/Documents/Masters-NYU/foot_mounted_pdr/test_data/new_pdr/{f_name}')
data.head()
m, n = np.shape(data)
print(m,n)

times = data["timestamp"].to_numpy()
laccx = data["linearAccelerationX"].to_numpy()
laccy = data["linearAccelerationY"].to_numpy()
laccz = data["linearAccelerationZ"].to_numpy()
gravx = -data["gravityX"].to_numpy()
gravy = -data["gravityY"].to_numpy()
gravz = data["gravityZ"].to_numpy()
gyrox = data["gyroscopeX"].to_numpy()
gyrox = np.rad2deg(gyrox)
gyroy = data["gyroscopeY"].to_numpy()
gyroy = np.rad2deg(gyroy)
gyroz = data["gyroscopeZ"].to_numpy()
gyroz = np.rad2deg(gyroz)
# magx = data["magx"].to_numpy()
# magy = data["magy"].to_numpy()
# magz = data["magz"].to_numpy()
roll = data["roll"].to_numpy()
pitch = data["pitch"].to_numpy()
yaw = data["yaw"].to_numpy()
times = np.diff(times)
t = times.astype(float)*1e-9
# print(t)
t= np.insert(t,0,0)
t = np.cumsum(t)

#Determing the current mode Holding=1, Swinging=2, Pocket=3, transition =0
window = int(2/dt)-1
current_mode = 1
mode = np.zeros(m)

i=0
while(i < m-1):
    if current_mode != 0:
        # print("checking for transition",t[i])
        if current_mode == 1:
            if abs(gyroy[i]) > GYRO_THRESHOLDY or abs(gyroz[i]) > GYRO_THRESHOLDZ:
                prev_mode = current_mode
                current_mode = 0
        elif current_mode == 2:
            if abs(gyroy[i]) > GYRO_THRESHOLDY:
                prev_mode = current_mode
                current_mode = 0
        elif current_mode == 3:
            if abs(gyroz[i]) > GYRO_THRESHOLDZ:
                prev_mode = current_mode
                current_mode = 0
        else:
            current_mode = current_mode
    else:   # Monitor the gravity vector and identify the mode
        print("Transitioning",t[i])
        i = i+50
        swing_1 = 0
        swing_2 = 0
        chk = 0
        #check if at the end of array and terminate
        if i+window > m-1:
            break
        for j in range(i,i+window):
            if prev_mode != 1:
                # print(t[j],j,i + window,"gravz",gravz[j],'<',-HGRAV_THRESHOLDZ,"gravy",gravy[j],'<',-HGRAV_THRESHOLDY,"gravx",gravx[j],'<',HGRAV_THRESHOLDX)
                if (gravy[j] < -HGRAV_THRESHOLDY and gravz[j]<0) or (gravz[j] < -HGRAV_THRESHOLDZ and gravy[j]<0):
                    print(j,f"transitioning from {prev_mode} to 1")
                    current_mode = 1
                    break
                else:
                    if prev_mode == 2:
                        print(j,f"transitioning from {prev_mode} to 3")
                        current_mode = 3
                    elif prev_mode == 3:
                        print(j,f"transitioning from {prev_mode} to 2")
                        current_mode = 2
            else:
                # check if any sample during the window meets abs(accx)>SGRAV_THRESHOLDX and all samples of accz < SGRAV_THRESHOLDZ
                if abs(gravx[j]) > SGRAV_THRESHOLDX:
                    swing_1 = 1
                if gravz[j] < SGRAV_THRESHOLDZ:
                    swing_2 = 1
                else:
                    # chk += 1
                    swing_2 = 0
                # print(t[j],j,i + window,"gravz",gravz[j],'<',SGRAV_THRESHOLDZ,"gravx",abs(gravx[j]),'>',SGRAV_THRESHOLDX,"swing_1",swing_1,"swing_2",swing_2)
                if (j+1 == (i + window)):
                    print("trans_end",swing_1,swing_2)
                    if (swing_1 == 1 and swing_2 == 1):
                        current_mode = 2
                    else:
                        # print(t[j],f"transitioning from {prev_mode} to 3")
                        current_mode = 3
        i = i + window
    mode[i] = current_mode
    i+=1


# Step detection and step length estimation
auv = np.zeros(m)
#Assuming that the user acceleration is the linear acceleration laccx, laccy, laccz

# Define cut-off frequency and time step (sampling rate)
cut_off_freq = 5  # 5Hz
sampling_rate = 85  # Assuming data is sampled at 50Hz

# Calculate the time constant (tau) of the filter
time_constant = 1 / (2 * pi * cut_off_freq)

# Initialize filtered accelerometer data
filtered_laccx = laccx.copy()
filtered_laccy = laccy.copy()
filtered_laccz = laccz.copy()
# Initialize filtered gravity data
filtered_gravx = gravx.copy()
filtered_gravy = gravy.copy()
filtered_gravz = gravz.copy()

# Apply low-pass filter to each axis of the accelerometer data
for k in range(1, len(laccx)):
    dt = 1 / sampling_rate
    alpha = dt / (time_constant + dt)
    filtered_laccx[k] = alpha * laccx[k] + (1 - alpha) * filtered_laccx[k - 1]
    filtered_laccy[k] = alpha * laccy[k] + (1 - alpha) * filtered_laccy[k - 1]
    filtered_laccz[k] = alpha * laccz[k] + (1 - alpha) * filtered_laccz[k - 1]
    filtered_gravx[k] = alpha * gravx[k] + (1 - alpha) * filtered_gravx[k - 1]
    filtered_gravy[k] = alpha * gravy[k] + (1 - alpha) * filtered_gravy[k - 1]
    filtered_gravz[k] = alpha * gravz[k] + (1 - alpha) * filtered_gravz[k - 1]

psi = pi/2 - np.arctan2(abs(filtered_gravz),abs(filtered_gravy))

# #plot psi
# plt.figure(figsize=(5,5))
# plt.plot(np.rad2deg(psi))
# plt.title('psi')
# plt.show()

for i in range(m):        
    current_mode =1
    if current_mode == 0:
        #go to next iteration
        continue
    if current_mode == 1:
        # psi = atan2(filtered_gravz[i],filtered_gravy[i])
        # print(psi)
        a_uv = filtered_laccz[i]*sin(psi[i]) + filtered_laccy[i]*cos(psi[i])
        auv[i] = a_uv
        # print(a_uv)

# Parameters
minimum_interval = 0.2  # Minimum time interval between steps (in seconds)
nfpr = 5  # Size of the False Peak Rejection (FPR) window (number of samples)
threshold = 0.15*9.81  # Step threshold (in m/s^2)
step_frequency = 0  # Variable to store the step frequency
motion_state = np.zeros(len(auv))  # List to store the motion state (0 for standing still, 1 for walking)

# Variables
prev_step_time = None
peak_times = []
peak_values = []
step_lengths = []

# Step length model parameters
k_male = 0.3139
k_female = 0.2975

# Constants for step length calculation
height = 1.70  # Height of the subject (in meters)

# Main Loop for Peak Detection
for i in range(len(auv)):
    peak = auv[i]
    if i!=0:
        if motion_state[i-1]==1:
            motion_state[i]=1
        else:
            motion_state[i]=0

    # Check if peak magnitude is greater than the threshold
    if (peak) > threshold:
        # Criterion 1 - Minimum Time Interval between Steps
        if prev_step_time is not None:
            time_interval = t[i] - prev_step_time
            if time_interval < minimum_interval:
                # Discard peak as it does not meet the minimum time interval condition
                continue

        # Criterion 2 - False Peak Rejection (FPR) Mechanism
        opposite_polarity = -1 if peak > 0 else 1
        for j in range(1, nfpr + 1):
            if i + j >= len(auv):
                break
            next_peak = auv[i + j]
            if next_peak * opposite_polarity > threshold:
                # Discard current peak as it is likely a false peak
                break
        else:
            # No opposite polarity peak found within FPR window, consider the current peak as a valid step
            prev_step_time = t[i]
            peak_times.append(t[i])
            peak_values.append(peak)
            # Process the valid step here (e.g., update step count, calculate step frequency, etc.)
            step_frequency = 1 / (t[i] - peak_times[-2]) if len(peak_times) >= 2 else 0
            if step_frequency == 0:
                # Step interpreted as the initial step from a motionless state to walking mode
                motion_state[i] = 1
                # Set initial constant step length (you can use any appropriate value)
                step_lengths.append((i,0.7))
            else:
                motion_state[i] = (1 if step_frequency >= 0.5 else 0)
                # Calculate step length using the proposed step length model (equation (9))
                if step_frequency >= 0.5:
                    # Use k_male for males and k_female for females (modify as needed)
                    step_length = k_male * height * np.sqrt(step_frequency)
                else:
                    step_length = k_female * height * np.sqrt(step_frequency)
                step_lengths.append((i,step_length))


# Print step lengths and motion states
for i in range(len(step_lengths)):
    print("Step Length (m) at time %.2f: %.2f" % (t[step_lengths[i][0]], step_lengths[i][1]))
    # print("Motion State at time %.2f: %d" % (t[step_lengths[i][0]], motion_state[step_lengths[i][0]]))
    print("Cumulative step length (m) at time %.2f: %.2f" % (t[step_lengths[i][0]], sum([x[1] for x in step_lengths[:i+1]])))
    print("")
    
print("Steps: ",len(step_lengths))

# Print step frequency and motion state
print("Step Frequency (steps per second):", step_frequency)
# print("Motion State (0: Stationary, 1: Moving):", motion_state)
    
x = [0]
y = [0]
print(yaw[0])
for i in range(len(step_lengths)):
    # print("yaw",yaw[i])
    # print("t",step_lengths[i][0], yaw[step_lengths[i][0]])
    x.append(x[i] + step_lengths[i][1]*sin(np.deg2rad(yaw[step_lengths[i][0]]-yaw[0])))
    y.append(y[i] + step_lengths[i][1]*cos(np.deg2rad(yaw[step_lengths[i][0]]-yaw[0])))

x = np.array(x)
y = np.array(y)

# for i in range(len(x)):
#     print(x[i], y[i])

#plot x and y

plt.figure(figsize=(10,5))
plt.plot(x,y, label='xy')
plt.legend()
plt.axis('equal')
plt.title('XY tracking')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.grid()

plt.show()