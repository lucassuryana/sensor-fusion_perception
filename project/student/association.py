# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params


class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''

    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self, track_list, meas_list, KF):
        ############
        # Step 3: Association
        ############
        N = len(track_list)  # number of tracks
        M = len(meas_list)  # number of measurements
        self.association_matrix = np.inf * np.ones((N, M))  # initialize with infinity
        self.unassigned_tracks = list(range(N))
        self.unassigned_meas = list(range(M))

        # Calculate Mahalanobis distance for all track-measurement pairs
        for i, track in enumerate(track_list):
            for j, meas in enumerate(meas_list):
                MHD = self.MHD(track, meas, KF)  # calculate Mahalanobis distance
                if self.gating(MHD, meas.sensor):  # apply gating
                    self.association_matrix[i, j] = MHD
        ############

    def get_closest_track_and_meas(self):
        ############
        # Step 3: Find closest track and measurement
        ############
        if self.association_matrix.size == 0:  # no associations available
            return np.nan, np.nan

        # Find minimum entry in the association matrix
        min_index = np.unravel_index(np.argmin(self.association_matrix, axis=None), self.association_matrix.shape)
        track_idx, meas_idx = min_index
        min_value = self.association_matrix[track_idx, meas_idx]

        # Remove the selected row and column
        self.association_matrix = np.delete(self.association_matrix, track_idx, axis=0)
        self.association_matrix = np.delete(self.association_matrix, meas_idx, axis=1)

        # Update unassigned lists
        track = self.unassigned_tracks.pop(track_idx)
        meas = self.unassigned_meas.pop(meas_idx)

        return track, meas if min_value < np.inf else (np.nan, np.nan)
        ############

    def gating(self, MHD, sensor):
        ############
        # Step 3: Gating logic
        ############
        limit = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)  # chi-squared threshold
        return MHD < limit
        ############

    def MHD(self, track, meas, KF):
        ############
        # Step 3: Mahalanobis distance
        ############
        H = meas.sensor.get_H(track.x)  # measurement matrix
        gamma = KF.gamma(track, meas)  # residual
        S = KF.S(track, meas, H)  # covariance of residual
        MHD = float(gamma.T @ np.linalg.inv(S) @ gamma)  # Mahalanobis distance
        return MHD
        ############

    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)

        # update associated tracks with measurements
        while self.association_matrix.shape[0] > 0 and self.association_matrix.shape[1] > 0:

            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]

            # check visibility, only update tracks in FOV
            if not meas_list[ind_meas].sensor.in_fov(track.x):
                continue

            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])

            # update score and track state
            manager.handle_updated_track(track)

            # save updated track
            manager.track_list[ind_track] = track

        # run track management
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)

        for track in manager.track_list:
            print('track', track.id, 'score =', track.score)