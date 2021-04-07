import serial
import time
import numpy as np
import matplotlib.pyplot as plt

from kalman import KalmanFilter

arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

print("Connected to: " + arduino.portstr)

# Variance in sensors
distance_var = 0.0032539285926189683
accelX_var = 0.0002538491460462614
delta_t = 0 # Time between each measurement

# Initial state
x = np.array([[20],
              [0]])

### Kalman filter for distance sensor ###
A = np.array([[0, 0],
              [0, 0]], dtype='float')
B = np.array([[0],
              [0]], dtype='float')
Q = 0
H = np.array([[1, 0]], dtype='float')
H_a = np.array([[0, 1]], dtype='float')
R = distance_var
R_a = accelX_var
P = np.array([[0, 0],
              [0, 0]], dtype='float')

s2_pos = 0.1 ** 2

est_state = np.zeros(2)
#est_cov = np.zeros((len(accelX), 2, 2))
#velocities = np.zeros(len(accelX))

kalman_filter_distance = KalmanFilter(A, B, H, Q, R, x, P, 0)
kalman_filter_accel = KalmanFilter(A, B, H, Q, R_a, x, P, 0)

velocity = 0
last_velocity = 0
last_time_stamp = 0
time_stamp = 0

i = 0
while True:
    while i < 5:
        msg = arduino.readline() #Read everything in the input buffer
        i += 1

    msg = arduino.readline() #Read everything in the input buffer
    msgVec = msg.decode('utf-8').split(',')
    if len(msgVec) == 5 and msgVec[0] and msgVec[1] and msgVec[2] and msgVec[3] and msgVec[4]:
        time_stamp = int(msgVec[0])
        print('Time Stamp: ', time_stamp)
        accelX = float(msgVec[1])
        distance_meas = float(msgVec[4])
        print('Acceleration X:')
        print(accelX)
        print('Distance X:')
        print(distance_meas)
        delta_t = (time_stamp - last_time_stamp) / 1000
        print('Delta T:')
        print(delta_t)

        velocity = float(accelX) * float(delta_t) + float(last_velocity)
        kalman_filter_accel.predict()
        kalman_filter_accel.update(velocity)
        (x, P) = kalman_filter_accel.get_state()

        if float(distance_meas) > 0.1:
            y = float(distance_meas)
            kalman_filter_distance.predict()
            kalman_filter_distance.update(y)
            (x, P) = kalman_filter_distance.get_state()
            velocity = x[1]
        est_state = x.transpose()
        print('Estimated state: ', est_state)
        #est_cov[i, ...] = P
        last_velocity = velocity

    A = np.array([[1, delta_t],
                  [0, 1]], dtype='float')

    B_a = np.array([[0.5*(delta_t**2)],
                    [delta_t]])

    base_sigma = np.array([[delta_t ** 3 / 3, delta_t ** 2 / 2],
                           [delta_t ** 2 / 2, delta_t]])
    Q = s2_pos * base_sigma

    kalman_filter_distance.updateParameters(A=A, B=B, H=H, Q=Q, R=R, u=0)
    kalman_filter_accel.updateParameters(A=A, B=B_a, H=H_a, Q=Q, R=R_a, u=accelX)

    estimated_distance = est_state.transpose()[0]
    print("Estimated distance: ", estimated_distance)

    last_time_stamp = time_stamp
    arduino.reset_input_buffer()
    time.sleep(0.1)
