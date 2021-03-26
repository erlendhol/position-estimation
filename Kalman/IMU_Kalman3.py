import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from kalman import KalmanFilter

imu_meas = pd.read_csv('maling7.csv')
accelX = imu_meas.iloc[5:, 1].values # Measured acceleration in m/s^2
accelY = imu_meas.iloc[5:, 2].values
accelZ = imu_meas.iloc[5:, 3].values

distance_meas = imu_meas.iloc[5:, 4].values / 100 # Measured distance in meters

time_stamps = imu_meas.iloc[5:, 0].values

# Variance in sensors
distance_var = 0.0032539285926189683
accelX_var = 0.0002538491460462614
delta_t = 0 # Time between each measurement

# Initial state
x = np.array([[distance_meas[0]],
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

est_state = np.zeros((len(accelX), 2))
est_cov = np.zeros((len(accelX), 2, 2))
velocities = np.zeros(len(accelX))

kalman_filter_distance = KalmanFilter(A, B, H, Q, R, x, P, 0)
kalman_filter_accel = KalmanFilter(A, B, H, Q, R_a, x, P, accelX[0])

velocity = 0
last_velocity = 0

for i in range(len(distance_meas)):
    if i > 0:
        delta_t = (time_stamps[i] - time_stamps[i-1]) / 1000 # Seconds
        velocity = accelX[i] * delta_t + last_velocity
        kalman_filter_accel.predict()
        kalman_filter_accel.update(velocity)
        (x, P) = kalman_filter_accel.get_state()
        #est_state[i, :] = x.transpose()
        #est_cov[i, ...] = P
        if distance_meas[i] > 0.1:
            y = distance_meas[i]
            kalman_filter_distance.predict()
            kalman_filter_distance.update(y)
            (x, P) = kalman_filter_distance.get_state()
            velocity = x[1]
        est_state[i, :] = x.transpose()
        est_cov[i, ...] = P
        velocities[i] = velocity
        last_velocity = velocity

    A = np.array([[1, delta_t],
                  [0, 1]], dtype='float')

    B_a = np.array([[0.5*(delta_t**2)],
                    [delta_t]])

    base_sigma = np.array([[delta_t ** 3 / 3, delta_t ** 2 / 2],
                           [delta_t ** 2 / 2, delta_t]])
    Q = s2_pos * base_sigma

    kalman_filter_distance.updateParameters(A=A, B=B, H=H, Q=Q, R=R, u=0)
    kalman_filter_accel.updateParameters(A=A, B=B_a, H=H_a, Q=Q, R=R_a, u=accelX[i])


plt.figure()
plt.ylim(0, 2)
plt.plot(distance_meas, '-bo')
plt.plot(est_state[:, 0], '-ko')
plt.plot(velocities, ':rx')
plt.xlabel('time')
plt.ylabel('distance')
plt.legend(['measured distance', 'estimated distance', 'estimated velocity'])
plt.tight_layout(pad=0)
plt.show()
