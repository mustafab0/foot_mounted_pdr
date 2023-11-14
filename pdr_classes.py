from math import sqrt, atan2, pi, cos, sin

DEG2RAD = pi / 180

class Position:
    def __init__(self, x, y, timestamp, vertical_acceleration):
        self.x = x
        self.y = y
        self.t = timestamp
        self.peaks = vertical_acceleration

class IMU_Packet:
    def __init__(self, x, y, z):
        self.x = x  #In case of Euler angles this is roll  
        self.y = y  #In case of Euler angles this is pitch
        self.z = z  #In case of Euler angles this is yaw

class AccelerationData:
    def __init__(self, timestamp,linear_acceleration, gravity, gyroscope):
        self.timestamp = timestamp
        self.linear_acceleration = linear_acceleration
        self.gravity = gravity
        self.gyroscope = gyroscope
        self.sample_rate = 85 #Hz
        self.filtered_linear_acceleration = None
        self.filtered_gravity = None

    def apply_low_pass_filter(self, prev_filtered_acceleration, data_type="linear_acceleration"):

        if data_type == "linear_acceleration":
            data = self.linear_acceleration
        elif data_type == "gravity":
            data = self.gravity
        elif data_type == "gyroscope":
            data = self.gyroscope
        else:
            raise ValueError("Invalid data_type. Use 'linear_acceleration', 'gravity', or 'gyroscope'.")
        #implement a low pass filter that takes in previous filtered acceleration and new acceleration
        #and returns the new filtered acceleration

        if prev_filtered_acceleration is None:
            return data
        
        # Set the cutoff frequency
        cutoff_frequency = 5 # Hz

        # Calculate the time interval between samples
        dt = 1 / self.sample_rate

        # Calculate the smoothing parameter alpha
        alpha = dt / (1/(2*pi*cutoff_frequency) + dt)

        # Apply the low pass filter equation
        filtered_acc_x = (1 - alpha) * data.x + alpha * prev_filtered_acceleration.x
        filtered_acc_y = (1 - alpha) * data.y + alpha * prev_filtered_acceleration.y
        filtered_acc_z = (1 - alpha) * data.z + alpha * prev_filtered_acceleration.z

        # Combine the filtered x, y, and z acceleration values into a single Acceleration object
        filtered_acceleration = IMU_Packet(filtered_acc_x, filtered_acc_y, filtered_acc_z)

        return filtered_acceleration

    def compute_vertical_acceleration(self, prev_data):
        
        # Apply the low-pass filter
        filtered_acceleration = self.apply_low_pass_filter(prev_data.filtered_linear_acceleration)
        filtered_gravity = self.apply_low_pass_filter(prev_data.filtered_gravity, "gravity")

        # Calculate psi and a_uv based on filtered accelerometer data and other variables
        psi = pi / 2 - atan2(abs(filtered_gravity.z), abs(filtered_gravity.y))

        # Compute the vertical acceleration
        v_acc = filtered_acceleration.z * sin(psi) + filtered_acceleration.y * cos(psi)

        # Update filtered acceleration
        self.filtered_gravity = filtered_gravity
        self.filtered_linear_acceleration = filtered_acceleration

        return v_acc
    
class PositionTracker:
    def __init__(self):
        self.timestamp = None
        self.vertical_acceleration = None
        self.window = 0 #samples
        self.threshold = 0.1*9.81  #m/s^2
        self.sample_rate = 85 #Hz
        self.prev_step_time = 0
        self.step_length = 0.
        self.rpy = None
        self.peak_acceleration = None
        self.opposite_polarity = None
        self.prev_peak_time = None
        self.step_length_model_params = {
            'k_male': 0.3139,
            'k_female': 0.2975,
            'height': 1.70  # Height of the subject (in meters)
        }

    def detect_peak(self, timestamp, vertical_acceleration, rpy):
        
        self.timestamp = timestamp
        self.vertical_acceleration = vertical_acceleration
        self.rpy = rpy

        if self.vertical_acceleration > self.threshold:
            self.peak_acceleration = self.vertical_acceleration
            return True

    def criterion1(self):
        
        minimum_interval = 0.5 #seconds

        time_interval = self.timestamp - self.prev_step_time
        if time_interval < minimum_interval:
            # Discard peak as it does not meet the minimum time interval condition
            return False
        
        self.opposite_polarity = -1 if self.vertical_acceleration > 0 else 1
        self.window = 5 #samples
        
        return True
        
    def criterion2(self, vertical_acceleration):
        # Criterion 2 - False Peak Rejection (FPR) Mechanism
        # Check if the peak is a false peak by comparing it with the following datasets of given window
        # If the peak is a false peak, then the vertical acceleration of the peak will be less than the vertical acceleration of the following datasets
        self.vertical_acceleration = vertical_acceleration

        if self.opposite_polarity * vertical_acceleration > self.threshold:

            self.window = 0
            return False

        self.window -= 1

        if self.window == 0:
            self.opposite_polarity = None
            return True
        
        return False
    
    def calculate_step_frequency(self):
        if self.prev_peak_time is not None:
            time_difference = self.timestamp - self.prev_peak_time
            step_frequency = 1 / time_difference
            return step_frequency
        else:
            return 0.0  # Return 0 if there are not enough peak times for calculation

    def step_length_estimation(self):
        # Estimate the step 
        step_frequency = self.calculate_step_frequency()
        if step_frequency == 0.0:
            # Step interpreted as the initial step from a motionless state to walking mode
            # Set initial constant step length (you can use any appropriate value)
            step_length = 0.7
            tstamp = self.timestamp
        else:
            if step_frequency >= 0.5:
                # Use k_male for males and k_female for females (modify as needed)
                step_length = self.step_length_model_params['k_male'] * self.step_length_model_params['height'] * sqrt(step_frequency)
            else:
                step_frequency = 0
                step_length = 0
    
        return step_length

    def update_position(self,prev_pos): 
        
        step_length = self.step_length_estimation()

        x = prev_pos.x + step_length * sin(DEG2RAD * self.rpy.z)
        y = prev_pos.y + step_length * cos(DEG2RAD * self.rpy.z)

        return Position(x, y, self.timestamp, self.peak_acceleration)
