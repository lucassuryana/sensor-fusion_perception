# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params


class Filter:
    '''Kalman filter class'''

    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        dt = params.dt
        F = np.eye(params.dim_state)  # Identity matrix for state transition
        F[0, 3] = dt  # Update position x with velocity x
        F[1, 4] = dt  # Update position y with velocity y
        F[2, 5] = dt  # Update position z with velocity z
        return F

        ############
        # END student code
        ############

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        dt = params.dt
        q = params.q
        Q = np.zeros((params.dim_state, params.dim_state))
        Q[0, 0] = (dt ** 3) / 3 * q  # Variance for position x
        Q[1, 1] = (dt ** 3) / 3 * q  # Variance for position y
        Q[2, 2] = (dt ** 3) / 3 * q  # Variance for position z
        Q[0, 3] = (dt ** 2) / 2 * q  # Covariance between position x and velocity x
        Q[1, 4] = (dt ** 2) / 2 * q  # Covariance between position y and velocity y
        Q[2, 5] = (dt ** 2) / 2 * q  # Covariance between position z and velocity z
        Q[3, 0] = (dt ** 2) / 2 * q
        Q[4, 1] = (dt ** 2) / 2 * q
        Q[5, 2] = (dt ** 2) / 2 * q
        Q[3, 3] = dt * q  # Variance for velocity x
        Q[4, 4] = dt * q  # Variance for velocity y
        Q[5, 5] = dt * q  # Variance for velocity z
        return Q

        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        Q = self.Q()
        x_pred = F @ track.x  # Predict state
        P_pred = F @ track.P @ F.T + Q  # Predict covariance
        track.set_x(x_pred)
        track.set_P(P_pred)

        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)  # Measurement matrix
        gamma = self.gamma(track, meas)  # Residual
        S = self.S(track, meas, H)  # Covariance of residual
        K = track.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        x_updated = track.x + K @ gamma  # Update state
        P_updated = (np.eye(params.dim_state) - K @ H) @ track.P  # Update covariance
        track.set_x(x_updated)
        track.set_P(P_updated)

        ############
        # END student code
        ############
        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # Step 1: calculate and return residual gamma
        ############
        H = meas.sensor.get_H(track.x)  # Measurement matrix
        gamma = meas.z - H @ track.x  # Residual
        return gamma

        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # Step 1: calculate and return covariance of residual S
        ############
        S = H @ track.P @ H.T + meas.R  # Covariance of residual
        return S

        ############
        # END student code
        ############ 