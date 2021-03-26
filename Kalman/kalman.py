import numpy as np
import matplotlib.pyplot as plt

class KalmanFilter():
    def __init__(self, A, B, H, Q, R, x_0, P_0, u_0):
        # Model parameters
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R

        # Initial state
        self._x = x_0
        self._P = P_0
        self._u = u_0

    def predict(self):
        self._x = self.A @ self._x + self.B * self._u
        self._P = self.A @ self._P @ self.A.transpose() + self.Q
        #print('Predicted state:', self._x)

    def update(self, z):
        self.S = self.H @ self._P @ self.H.transpose() + self.R
        self.V = z - self.H @ self._x
        self.K = self._P @ self.H.transpose() @ np.linalg.inv(self.S)
        print(self.K)
        self._x = self._x + self.K @ self.V
        self._P = self._P - self.K @ self.S @ self.K.transpose()

    def get_state(self):
        return self._x, self._P

    def updateParameters(self, A, B, Q, u):
        self.A = A
        self.B = B
        self.Q = Q
        self._u = u

class MotionModel():
    def __init__(self, A, Q):
        self.A = A
        self.Q = Q

        (m, _) = Q.shape
        self.zero_mean = np.zeros(m)

    def __call__(self, x):
        new_state = self.A @ x + np.random.multivariate_normal(self.zero_mean, self.Q)
        return new_state

class MeasurementModel():
    def __init__(self, H, R):
        self.H = H
        self.R = R

        (n, _) = R.shape
        self.zero_mean = np.zeros(n)

    def __call__(self, x):
        measurement = self.H @ x + np.random.multivariate_normal(self.zero_mean, self.R)
        return measurement

def create_model_parameters(T=1, s2_x=0.1 ** 2, s2_y=0.1 ** 2, lambda2=0.3 ** 2):
    # Motion model parameters
    F = np.array([[1, T],
                  [0, 1]])
    base_sigma = np.array([[T ** 3 / 3, T ** 2 / 2],
                           [T ** 2 / 2, T]])

    sigma_x = s2_x * base_sigma
    sigma_y = s2_y * base_sigma

    zeros_2 = np.zeros((2, 2))
    A = np.block([[F, zeros_2],
                  [zeros_2, F]])
    Q = np.block([[sigma_x, zeros_2],
                  [zeros_2, sigma_y]])

    # Measurement model parameters
    H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
    R = lambda2 * np.eye(2)

    return A, H, Q, R

def simulate_system(K, x0):
    (A, H, Q, R) = create_model_parameters()

    # Create models
    motion_model = MotionModel(A, Q)
    meas_model = MeasurementModel(H, R)

    (m, _) = Q.shape
    (n, _) = R.shape

    state = np.zeros((K, m))
    meas = np.zeros((K, n))

    # Initial state
    x = x0
    for k in range(K):
        x = motion_model(x)
        z = meas_model(x)

        state[k, :] = x
        meas[k, :] = z

    return state, meas
