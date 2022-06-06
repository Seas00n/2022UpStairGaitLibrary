import numpy as np
import matplotlib.pyplot as plt
import scipy as sp


def init_phase_filter():
    dt = 1
    R = np.array([[0.5, 0], [0, 5]])
    P = 0.1 * R
    Q = np.array([[0, 0], [0, 1e-6]])
    F = np.array([[1, dt], [0, 1]])
    H = np.array([[1, 0], [0, 1]])
    kf = KalmanFilter(P=P, Q=Q, R=R, F=F, H=H, x_0=np.array([0, 1]))
    return kf


class OnlinePhasePredictor(object):
    def __init__(self, mode='level_ground'):
        self.gait_paras_dict = np.load('data/gait_paras_dict_{}.npy'.format(mode), allow_pickle=True).item()
        self.gait_start = False
        self.state = 2
        self.acc_foot_threshold = 15
        self.stance_end_threshold = self.gait_paras_dict['stance_end_threshold']
        self.swing_end_threshold = self.gait_paras_dict['swing_end_threshold']
        self.kf = init_phase_filter()
        self.thigh_angle_all = np.zeros(0)
        self.phase_vec = np.zeros(2)
        self.phase_all = np.zeros(1)
        self.thresholds = np.array([self.stance_end_threshold, self.swing_end_threshold])

    def start_init(self):
        self.kf = init_phase_filter()
        self.thigh_angle_all = np.zeros(0)
        self.phase_vec = np.zeros(2)
        self.phase_all = np.zeros(1)

    def phase_predict(self, thigh_angle, acc_z, dt=1):
        if self.state == 2 and (acc_z > self.acc_foot_threshold or acc_z < -self.acc_foot_threshold):
            self.gait_start = True
            self.state = 0
            self.start_init()
        if not self.gait_start:
            return None
        else:
            self.thigh_angle_all = np.append(self.thigh_angle_all, thigh_angle)
            if len(self.thigh_angle_all) > 1:
                self.gait_state_estimation()
                phase = self.thigh_angle_to_phase()
                self.phase_vec = update_vec(self.phase_vec, phase)
                F = np.array([[1, dt], [0, 1]])
                phase = self.kf.forward(np.array([phase, (self.phase_vec[1] - self.phase_vec[0]) / dt]), F)[0]
                phase = np.clip(phase, a_min=np.max(self.phase_all), a_max=100)
                self.phase_all = np.append(self.phase_all, phase)
            else:
                phase = 0
        return phase

    def get_State(self):
        return self.state

    def get_Threshold(self):
        return self.thresholds

    def gait_state_estimation(self):
        if self.state == 0:
            min_thigh_angle = np.min(self.thigh_angle_all)
            if (self.thigh_angle_all[-1] < self.stance_end_threshold or
                    (self.thigh_angle_all[
                         -1] > min_thigh_angle + 2 and min_thigh_angle < self.stance_end_threshold + 5)):
                self.state = 1  # stance to swing
                self.gait_paras_dict['stance_end_idx'] = len(self.thigh_angle_all)
        elif self.state == 1:
            max_thigh_angle = np.max(self.thigh_angle_all[self.gait_paras_dict['stance_end_idx'] - 1:-1])
            if (self.thigh_angle_all[-1] > self.swing_end_threshold or
                    (self.thigh_angle_all[
                         -1] < max_thigh_angle - 5 and max_thigh_angle > self.swing_end_threshold - 15)):
                self.state = 2  # swing to stance
                self.gait_paras_dict['swing_end_idx'] = len(self.thigh_angle_all)

    def thigh_angle_to_phase(self):
        proportional_list = self.gait_paras_dict['popt_list'][self.state]
        x_low = self.gait_paras_dict['idx_list'][self.state]
        x_high = self.gait_paras_dict['idx_list'][self.state + 1]
        theta_low = fun_x_to_y(x_low, *proportional_list)
        theta_high = fun_x_to_y(x_high, *proportional_list)
        thigh_angle = np.clip(self.thigh_angle_all[-1], min(theta_low, theta_high), max(theta_low, theta_high))
        phase = fun_y_to_x(thigh_angle, *proportional_list)
        phase = np.clip(phase, x_low, x_high)
        return phase


def fun_x_to_y(x, h, x0, k, b):  # 参数选择：h,x0,k,b
    y = h / (1 + np.exp(-k * (x - x0))) + b
    return y


def fun_y_to_x(y, h, x0, k, b):
    x = x0 - np.log(h / (y - b) - 1) / k
    return x


def update_vec(data_mat, data_vec):
    data_mat[:-1] = data_mat[1:]
    data_mat[-1] = data_vec
    return data_mat


class KalmanFilter(object):
    """
    Simplified Kalman Filter only deals with the following dynamic model:
    x_k = F_k-1 x_k-1 + w_k
    y_k = H_k x_k + v_k
    """

    def __init__(self, P, Q, R, F, H, x_0):
        '''
        P: covariance matrix of the state, which indicates the uncertainty of the current estimation
        Q: covariance matrix of the process noises w
        R: covariance matrix of the measures noises v
        F: state transition matrix
        H: observation matrix
        '''
        self.P = P
        self.Q = Q
        self.R = R
        self.F = F
        self.H = H
        self.x = x_0

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q  # @ indicates matrix multiplication

    def update(self, y):
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + K @ (y - self.H @ self.x)
        I = np.eye(len(self.x))
        self.P = (I - K @ self.H) @ self.P

    def forward(self, y, F):
        self.F = F
        self.predict()
        self.update(y)
        return self.x
