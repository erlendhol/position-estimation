import numpy as np
import math

# Self-made libraries
import quaternion

class MadgwickFilter():
    """
    A class representing a Madgwick filter.
    :param beta: The filter gain representing all mean zero gyroscope errors,
    expressed as the magnitude of a quaternion derivative. It is defined using the
    angular velocity: beta = sqrt(3/4) * omegab, where omegab is the estimated
    mean zero gyroscope measurement error of each axis.
    """
    def __init__(self, beta):
        self.beta = beta
        self.q = np.array([1, 0, 0, 0]).T
        self.update_term = 0
        self.acc_normalized = np.array([0, 0, 0]).T
        self.mag_normalized = np.array([0, 0, 0, 0]).T
        pass


    def get_estimated_orientation(self, delta_t, gyro, acc, mag=None):
        """
        Calculates the estimated quaternion representation of the orientation.
        :param delta_t: The time between each measurement
        :param gyro: Array containing the gyroscope measurement
        :param acc: Array containing the accelerometer measurement
        :param mag: Array containing the magnetometer measurement. This should
        be a 4x1 array where the first element is 0, and the rest is the
        magnetometer measurement.
        """
        qw = self.q[0]
        qx = self.q[1]
        qy = self.q[2]
        qz = self.q[3]

        acc_norm = np.linalg.norm(acc)
        if acc_norm > 0:
            self.acc_normalized = np.divide(acc, acc_norm)

        ##ORIENTATION INCREMENT FROM ACC ##
        f_g = np.array([(2*(qx*qz - qw*qy) - self.acc_normalized[0]),
                        (2*(qw*qx - qy*qz) - self.acc_normalized[1]),
                        (2*(0.5 - qx**2 - qy**2) - self.acc_normalized[2])])

        j_g = np.array([[-2*qy, 2*qz, -2*qw, 2*qx],
                        [2*qx, 2*qw, 2*qz, 2*qy],
                        [0, -4*qx, -4*qy, 0]])

        ## ORIENTATION INCREMENT FROM GYRO ##
        q_w_dot = quaternion.multiply((0.5*self.q), np.array([0, gyro[0], gyro[1], gyro[2]]).transpose())

        if mag is None:
            grad_step = j_g.transpose() @ f_g
            grad_norm = np.linalg.norm(grad_step)

            if grad_norm != 0:
                self.update_term = -self.beta * np.divide(grad_step, grad_norm)
        else:
        ## CORRECT USING MAGNETOMETER ##
            mag_norm = np.linalg.norm(mag)
            if mag_norm > 0:
                self.mag_normalized = np.divide(mag, mag_norm)

            h = self.q * self.mag_normalized * quaternion.conjugate(self.q)
            bx = math.sqrt(h[1]**2 + h[2]**2)
            bz = h[3]

            f_b = np.array([(2*bx*(0.5-qy**2-qz**2) + 2*bz*(qx*qz-qw*qy) - self.mag_normalized[1]),
                           (2*bx*(qx*qy-qw*qz) + 2*bz*(qw*qx+qy*qz) - self.mag_normalized[2]),
                           (2*bx*(qw*qy+qx*qz) + 2*bz*(0.5-qx**2-qy**2) - self.mag_normalized[3])])

            j_b = np.array([(-2*bz*qy, 2*bz*qz, -4*bx*qy-2*bz*qw, -4*bx*qz+2*bz*qx),
                           (-2*bx*qz+2*bz*qx, 2*bx*qy+2*bz*qw, 2*bx*qx+2*bz*qz, -2*bx*qw+2*bz*qy),
                           (2*bx*qy, 2*bx*qz-4*bz*qx, 2*bx*qw-4*bz*qy, 2*bx*qx)])

            f_gb = np.block([f_g,
                             f_b])

            j_gb = np.block([[j_g],
                             [j_b]])

            grad_step = j_gb.transpose() @ f_gb
            grad_norm = np.linalg.norm(grad_step)

            if grad_norm != 0:
                self.update_term = -self.beta * np.divide(grad_step, grad_norm)

        q_est_dot = q_w_dot + self.update_term
        q_est = self.q + (q_est_dot * delta_t)
        self.q = quaternion.normalize(q_est)
        return self.q
