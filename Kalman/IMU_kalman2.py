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
accelX_var = 0.00006291755
delta_t = 0 # Time between each measurement

A = 0
B = 0
Q = 0
H = np.array([[1, 0],
              [0, 1]], dtype='float')
R = np.array([[distance_var, 0],
              [0, accelX_var]])
P = np.array([[0, 0],
              [0, 0]], dtype='float')

est_state = np.zeros((len(accelX), 2))
est_cov = np.zeros((len(accelX), 2, 2))
calculated_velocity = 0
last_velocity = 0
# Initial state
x = np.array([[distance_meas[0]],
              [0]])
last_y = np.array([[0],
                   [0]])
kalman_filter = KalmanFilter(A, B, H, Q, R, x, P, accelX[0])

s2_pos = 1.9 ** 2
s2_vel = 0.1 ** 2
num_of_lost_measurements = 0

for i in range(len(accelX)):
    last_velocity = calculated_velocity
    y = np.array([[distance_meas[i]],
                 [calculated_velocity]])
    if i > 0:
        delta_t = (time_stamps[i] - time_stamps[i-1]) / 1000 # Seconds
        calculated_velocity = (distance_meas[i] - distance_meas[i-1])/delta_t
        kalman_filter.predict()
        if distance_meas[i] > 0.1:
            kalman_filter.update(y)
            last_y = y
            #num_of_lost_measurements = 0
        #elif distance_meas[i] < 0.1 and num_of_lost_measurements < 3:
            #num_of_lost_measurements = num_of_lost_measurements + 1
            #kalman_filter.update(last_y)
        (x, P) = kalman_filter.get_state()
        est_state[i, :] = x.transpose()
        est_cov[i, ...] = P

    # Motion model parameters
    A = np.array([[1, delta_t],
                  [0, 1]], dtype='float')
    B = np.block([[0.5*(delta_t**2)],
                  [delta_t]])
    base_sigma = np.array([[delta_t ** 3 / 3, delta_t ** 2 / 2],
                           [delta_t ** 2 / 2, delta_t]])

    sigma_pos = s2_pos * base_sigma
    sigma_vel = s2_vel * base_sigma


    Q = sigma_pos
    #Q = np.array([[sigma_pos, 0],
                  #[0, sigma_vel]])

    kalman_filter.updateParameters(A=A, B=B, Q=Q, u=accelX[i])

plt.figure()
plt.ylim(0, 2)
plt.plot(distance_meas, '-bo')
plt.plot(est_state[:, 0], '-ko')
#plt.plot(accelX, ':rx')
plt.xlabel('time')
plt.ylabel('distance')
plt.legend(['measured distance', 'estimated distance'])
#plt.axis('square')
plt.tight_layout(pad=0)
plt.show()
