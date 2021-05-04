## Written by Erlend Holseker and Arvin Khodabandeh

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
        self.beta = float(beta)
        self.q = np.array([0.7071, 0, 0.7071, 0]).T
        self.update_term = 0
        self.acc_normalized = np.array([0, 0, 0]).T
        self.mag_normalized = np.array([0, 0, 0, 0]).T


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

        j_g = np.array([(-2*qy, 2*qz, -2*qw, 2*qx),
                        (2*qx, 2*qw, 2*qz, 2*qy),
                        (0, -4*qx, -4*qy, 0)])

        ## ORIENTATION INCREMENT FROM GYRO ##
        q_w_dot = 0.5*quaternion.multiply(self.q, np.array([0, gyro[0], gyro[1], gyro[2]]).transpose())

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

            h = quaternion.multiply(quaternion.multiply(self.q, self.mag_normalized), quaternion.conjugate(self.q))

            # mw, mx, my, mz = self.mag_normalized[0], self.mag_normalized[1], self.mag_normalized[2], self.mag_normalized[3]
            #
            # tw = qw*mw - qx*mx - qy*my - qz*mz
            # tx = qw*mx + qx*mw + qy*mz - qz*my
            # ty = qw*my + qy*mw + qz*mx - qx*mz
            # tz = qw*mz + qz*mw + qx*my - qy*mx
            #
            # cw, cx, cy, cz = qw, -qx, -qy, -qz
            #
            # hw = tw*cw - tx*cx - ty*cy - tz*cz
            # hx = tw*cx + tx*cw + ty*cz - tz*cy
            # hy = tw*cy + ty*cw + tz*cx - tx*cz
            # hz = tw*cz + tz*cw + tx*cy - ty*cx
            #
            # bx = math.sqrt(hx**2 + hy**2)
            # bz = hz

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

            #print('J_g: ', j_g)
            #print('J_b: ', j_b)

            #print('J_g.T: ', j_g.T)
            #print('J_b.T: ', j_b.T)

            j_gb = np.block([[j_g],
                             [j_b]])

            #print('J_gb: ', j_gb)
            #print('J_gb.T: ', j_gb.T)
            #print('F_gb: ', f_gb)

            grad_step = j_gb.T @ f_gb
            grad_norm = np.linalg.norm(grad_step)

            if grad_norm != 0:
                self.update_term = -self.beta * np.divide(grad_step, grad_norm)

        q_est_dot = q_w_dot + self.update_term
        q_est = self.q + (q_est_dot * delta_t)
        self.q = quaternion.normalize(q_est)
        return self.q
