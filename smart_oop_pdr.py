import pandas as pd
import matplotlib.pyplot as plt
import os
from pdr_classes import Position, IMU_Packet, AccelerationData, PositionTracker


f_name = 'Ani-Lab-Front-2.json'
script_directory = os.path.dirname(os.path.abspath(__file__))
# file_path = os.path.dirname(script_directory)
file_path = os.path.join(script_directory, "test_data", "new_pdr", f_name)
print("file path: ",file_path)
dt = 0.0125

data = pd.read_json(file_path)
data.head()
size = data.shape
print(size)

prev_data_packet = AccelerationData(0, None, None,None)
tracker = PositionTracker()
vertical_acceleration = []
pos = Position(0,0,0,0)
positions = []


criteria_1 = False
criteria_2 = False

for packet in data.iterrows():      #for each row in the dataframe

    linear_acceleration = IMU_Packet(packet[1]['linearAccelerationX'],packet[1]['linearAccelerationY'],packet[1]['linearAccelerationZ'])
    gravity             = IMU_Packet(packet[1]['gravityX'],packet[1]['gravityY'],packet[1]['gravityZ'])
    gyroscope           = IMU_Packet(packet[1]['gyroscopeX'],packet[1]['gyroscopeY'],packet[1]['gyroscopeZ'])
    rpy                 = IMU_Packet(packet[1]['roll'],packet[1]['pitch'],packet[1]['yaw'])
    timestamp           = packet[1]['timestamp'].timestamp()    #convert datetime object to float timestamp   

    curr_data_packet = AccelerationData(timestamp,linear_acceleration, gravity, gyroscope) 
    v_acc = curr_data_packet.compute_vertical_acceleration(prev_data_packet)
    vertical_acceleration.append(v_acc)

    if tracker.window != 0:
        criteria_2 = tracker.criterion2(v_acc)
        prev_data_packet = curr_data_packet
        if tracker.window == 0 and criteria_2 == False:    #if window is 0 and criteria_2 is false, then reset criteria_1
            criteria_1 = False
        continue
    

    if (criteria_1 and criteria_2 ):
        pos = tracker.update_position(pos)
        positions.append(pos)
        criteria_1 = False
        criteria_2 = False
        tracker.prev_peak_time = tracker.timestamp
        tracker.prev_step_time = tracker.timestamp
        prev_data_packet = curr_data_packet
        continue

    if tracker.detect_peak(curr_data_packet.timestamp, v_acc, rpy):
        criteria_1 = tracker.criterion1()
        prev_data_packet = curr_data_packet
        continue

    
    prev_data_packet = curr_data_packet


#extract complete column from position object
x = [pos.x for pos in positions]
y = [pos.y for pos in positions]

peak_times = [pos.t for pos in positions]
peak_values = [pos.peaks for pos in positions]

t = data['timestamp']
#convert datetime object to float timestamp
t = [timestamp.timestamp() for timestamp in t]
# t = [timestamp.timestamp()-t[0].timestamp() for timestamp in t]
# Plotting the data and highlighting the detected peaks and motion state
plt.figure(figsize=(10, 5))
plt.plot(t,vertical_acceleration, label="Accelerometer Data")
plt.plot(t, [tracker.threshold]*len(t), color='black', label="Threshold") 
plt.scatter(peak_times, peak_values, color='red', label="Detected Peaks")
plt.xlabel("Time (seconds)")
plt.ylabel("Accelerometer Value / Motion State")
plt.title("Detected Peaks and Motion State in Accelerometer Data")
plt.legend()
# plt.xticks(ticks=np.arange(0,t[-1],1))
plt.grid(True)
plt.show()

plt.figure(figsize=(10,5))
plt.plot(x,y, label='xy')
plt.legend()
plt.axis('equal')
plt.title('Position Traking using Smartphone IMU')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid()

plt.show()