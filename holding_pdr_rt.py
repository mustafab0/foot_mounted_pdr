from math import sqrt, atan2, pi, cos, sin
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import datetime

class Acceleration:
    def __init__(self, x, y, z):
        self.ax = x
        self.ay = y
        self.az = z

f_name = 'imu_data_20230907_161844.json'

dt = 0.0125


data = pd.read_json(f'C:/Users/workshop/Documents/Masters-NYU/foot_mounted_pdr/test_data/new_pdr/{f_name}')
data.head()
m, n = np.shape(data)
print(m,n)

# Initialize necessary variables and constants

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

times = data["timestamp"].to_numpy()
laccx, laccy, laccz = data[["linearAccelerationX", "linearAccelerationY", "linearAccelerationZ"]].to_numpy().T
gravx, gravy, gravz = -data[["gravityX", "gravityY", "gravityZ"]].to_numpy().T
roll, pitch, yaw = data[["roll", "pitch", "yaw"]].to_numpy().T
gyrox = np.rad2deg(data["gyroscopeX"].to_numpy())
gyroy = np.rad2deg(data["gyroscopeY"].to_numpy())
gyroz = np.rad2deg(data["gyroscopeZ"].to_numpy())
# magx = data["magx"].to_numpy()
# magy = data["magy"].to_numpy()
# magz = data["magz"].to_numpy()

times = np.diff(times)
t = times.astype(float)*1e-9
# print(t)
t= np.insert(t,0,0)
t = np.cumsum(t)


# Set up your accelerometer sensor and sampling rate
sampling_rate = 85; # Hz

def apply_low_pass_filter(new_acceleration, previous_filtered_acceleration, sample_rate):
    cutoff_frequency = 5 # Hz
    dt = 1 / sample_rate
    tau = 1 / (2 * np.pi * cutoff_frequency)
    
    filtered_acc_x = (1 - np.exp(-dt / tau)) * new_acceleration.ax + np.exp(-dt / tau) * previous_filtered_acceleration.ax
    filtered_acc_y = (1 - np.exp(-dt / tau)) * new_acceleration.ay + np.exp(-dt / tau) * previous_filtered_acceleration.ay
    filtered_acc_z = (1 - np.exp(-dt / tau)) * new_acceleration.az + np.exp(-dt / tau) * previous_filtered_acceleration.az

    filtered_acceleration = Acceleration(filtered_acc_x, filtered_acc_y, filtered_acc_z)
    
    return filtered_acceleration

def getVerticalAcc(time_stamp, prev_acceleration, prev_gravity):
    current_acceleration = Acceleration(laccx[time_stamp], laccy[time_stamp], laccz[time_stamp])
    current_gravity = Acceleration(gravx[time_stamp], gravy[time_stamp], gravz[time_stamp])
    # Apply the low-pass filter to the current_acceleration
    filtered_acceleration = apply_low_pass_filter(current_acceleration, prev_acceleration, sampling_rate)
    filtered_gravity = apply_low_pass_filter(current_gravity, prev_gravity, sampling_rate)
    prev_acceleration = filtered_acceleration
    prev_gravity = filtered_gravity

    # Calculate psi and a_uv based on filtered accelerometer data and other variables
    psi = pi/2 - atan2(abs(filtered_gravity.az),abs(filtered_gravity.ay))
    
    #compute the vertical acceleration
    v_acc = filtered_acceleration.az*sin(psi) + filtered_acceleration.ay*cos(psi)

    return v_acc, prev_acceleration, prev_gravity

def update_plot(x_position, y_position):
    # plt.clf()  # Clear the previous plot
    plt.plot(x_position, y_position, 'bo-')  # Plot x and y positions
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Real-time Position')
    plt.grid(True)
    plt.axis('equal')
    plt.draw()
    plt.pause(1)  # Pause to allow the plot to update
    # plt.close()

previous_acceleration = Acceleration(0, 0, 0)
previous_gravity = Acceleration(0, 0, -9.81)
#create numpy arrays for filtered data
auv = np.zeros(m)

# Parameters
minimum_interval = 0.2  # Minimum time interval between steps (in seconds)
nfpr = 5  # Size of the False Peak Rejection (FPR) window (number of samples)
threshold = 0.08*9.81  # Step threshold (in m/s^2)
step_frequency = 0  # Variable to store the step frequency
motion_state = np.zeros(len(auv))  # List to store the motion state (0 for standing still, 1 for walking)

# Variables
prev_step_time = 0
peak_times = []
peak_values = []
step_lengths = []
step_length = 0
x = [0]
y = [0]
tstamp = datetime.datetime.strptime(str(data["timestamp"][0]), '%Y-%m-%d %H:%M:%S.%f').timestamp()
x_pos = [(tstamp, x[0])]
y_pos = [(tstamp, y[0])]

print(x_pos,y_pos)

# Step length model parameters
k_male = 0.3139
k_female = 0.2975

# Constants for step length calculation
height = 1.70  # Height of the subject (in meters)

i = 0
# Main loop for real-time processing
while True:
    # Read accelerometer data and timestamp
    # print("i",i,m)
    time = t[i]
    
    a_uv, previous_acceleration, previous_gravity = getVerticalAcc(i, previous_acceleration, previous_gravity)

    auv[i] = a_uv
    if i!=0:
        if motion_state[i-1]==1:
            motion_state[i]=1
        else:
            motion_state[i]=0
    
    # Check if peak magnitude is greater than the threshold
    if (a_uv > threshold):
        # Criterion 1 - Minimum Time Interval between Steps
        print("i before valid peak",i)
        print(time)
        peak_is_valid = False
        time_interval = time - prev_step_time
        peak = a_uv
        peak_time = time
        if time_interval < minimum_interval:
            # Discard peak as it does not meet the minimum time interval condition
            print("criteria 1 not met")
            i += 1
            continue
        
        # Criterion 2 - False Peak Rejection (FPR) Mechanism
        opposite_polarity = -1 if a_uv > 0 else 1
        cnt = 0
        for frame in range(1,nfpr+1):
            if i+frame >= m:
                break
            # print(a_uv)
            a_uv, previous_acceleration, previous_gravity =  (i+frame, previous_acceleration, previous_gravity)
            auv[i+frame] = a_uv
            cnt = frame
            # print("cnt",cnt)
            if (opposite_polarity * a_uv > threshold):
                # Discard peak as it does not meet the FPR condition
                print("criteria 2 not met")
                peak_is_valid = False
                break
            # i += 1   
            peak_is_valid = True      
        if (peak_is_valid):
            # If the peak passes both criteria, it is a valid peak
            peak_is_valid = True
            prev_step_time = peak_time
            peak_times.append(peak_time)
            peak_values.append(peak)
            # Process the valid step here (e.g., update step count, calculate step frequency, etc.)
            step_frequency = 1 / (time - peak_times[-2]) if len(peak_times) >= 2 else 0
            if step_frequency == 0:
                # Step interpreted as the initial step from a motionless state to walking mode
                motion_state[i] = 1
                # Set initial constant step length (you can use any appropriate value)
                step_length = 0.7
                tstamp = datetime.datetime.strptime(str(data["timestamp"][i]), '%Y-%m-%d %H:%M:%S.%f').timestamp()
                step_lengths.append((i,tstamp,step_length))
            else:
                motion_state[i] = 1 if step_frequency >= 0.5 else 0
                # Calculate step length using the proposed step length model (equation (9))
                if step_frequency >= 0.5:
                    # Use k_male for males and k_female for females (modify as needed)
                    step_length = k_male * height * sqrt(step_frequency)
                else:
                    step_frequency = 0
                tstamp = datetime.datetime.strptime(str(data["timestamp"][i]), '%Y-%m-%d %H:%M:%S.%f').timestamp()
                step_lengths.append((i,tstamp,step_length))
            # print("step length",step_length, "yaw",yaw[i])
            x.append(x[-1] + step_length*sin(np.deg2rad(yaw[i]-yaw[0])))
            y.append(y[-1] + step_length*cos(np.deg2rad(yaw[i]-yaw[0])))
            x_pos.append((tstamp,x[-1]))
            y_pos.append((tstamp,y[-1]))
            i = i+cnt 
            # update_plot(x[-1], y[-1])
            print("i after valid peak",i)
    
    # print("In the loop")
    i += 1
    
    
    
    if i == m:
        break

    # Other real-time processing or display updates
    # ...

# print(step_lengths)
# print(x_pos,y_pos)
x_pos = np.asarray(x_pos)   
y_pos = np.asarray(y_pos)
# print(x_pos.shape,y_pos.shape)
pos = np.zeros((len(x_pos),3))
pos[:,0] = x_pos[:,0]
pos[:,1] = x_pos[:,1]
pos[:,2] = y_pos[:,1]
# pos = np.hstack((x_pos,y_pos[:,1]))  
print(pos)
np.savetxt("xy_pos.txt",pos)
# np.savetxt("y_pos.txt",y_pos)

#Plot the filtered accelerometer data
plt.figure(figsize=(10,5))
plt.scatter(peak_times, peak_values, color='red', label="Detected Peaks")
# plt.plot(t,motion_state, label="Motion State (0 for standing still, 1 for walking)", color='green', alpha=0.8)
plt.xticks(ticks=np.arange(0,t[-1],1))
plt.plot(t,auv)
plt.title('Verical acceleration')
plt.legend(['vertical accel'])
plt.grid()
plt.show()

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

plt.figure(figsize=(10,5))
plt.plot(t, yaw, label='xy')
plt.legend()
# plt.axis('equal')
plt.title('XY tracking')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.grid()

plt.show()

