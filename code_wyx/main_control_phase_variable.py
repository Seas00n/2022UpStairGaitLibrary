import numpy as np
import time
import serial
import matplotlib.pyplot as plt
import Phase_Algo
from Dyna_Algo import *
from commucation import *
from PID_ import *


def main():
    control_motor = False
    # 加载数据库
    gait_paras_dict = np.load('data/gait_paras_dict_{}.npy'.format('level_ground'), allow_pickle=True).item()
    # 加载虚拟约束
    f_joint_angle = gait_paras_dict['f_joint_angle']
    print('-------ALL DATA HAVE BEEN LOADED IN------')
    imu_data_buf = np.memmap('Log/imu_data.npy', dtype='float32', mode='r', shape=(6,))
    q_thigh_init = initialize_q_thigh(imu_data_buf)
    phase_predictor = Phase_Algo.OnlinePhasePredictor(mode='level_ground_cxx')
    time_step_num = 5000
    q_thigh_vec = np.zeros((time_step_num,))
    qd_thigh_vec = np.zeros((time_step_num,))
    time_vec = np.array([-0.02, -0.02])
    phase_vec = np.zeros((time_step_num,))
    q_actual_vec, qd_actual_vec, q_desired_vec, qd_desired_vec, q_int_vec = pid_buffer_block_init(time_step_num, 2)
    torque_given = np.zeros((time_step_num, 2))
    Kp = 1.5
    Ki = 0.01
    Kd = 0.03

    plot_buf = np.memmap('Log/plot_buf.npy', dtype='float32', mode='r+', shape=(10,))
    if control_motor:
        ser = init_serial('COM27', 115200)
    start_time = time.time()
    time.sleep(1)
    if control_motor:
        send_signal_to_motor(ser, np.array([0, 0]))
    for i in range(time_step_num):
        t = time.time() - start_time
        q_thigh, acc_z = read_angle_and_acc(imu_data_buf, q_thigh_init)
        time_vec = fifo_mat(time_vec, t)
        dt = time_vec[-1] - time_vec[-2]
        phase = phase_predictor.phase_predict(q_thigh, acc_z, 100 * (time_vec[-1] - time_vec[-2]))
        if phase is not None:
            print('\rPhase={}'.format(phase), end='')
            phase_vec = fifo_mat(phase_vec, phase)
            q_thigh_vec, qd_thigh_vec = basic_ode(q_thigh, q_thigh_vec, qd_thigh_vec, dt)
            q_desired = predict_qd(phase, f_joint_angle)
            q_desired_vec, qd_desired_vec = basic_ode(q_desired, q_desired_vec, qd_desired_vec, dt)
            if control_motor:
                q_actual, q_actual_d = read_signal_from_motor(ser)
            else:
                q_actual = q_desired_vec[-1]
            q_actual_vec, qd_actual_vec = basic_ode(q_actual, q_actual_vec, qd_actual_vec, dt)
            q_int_vec = fifo_mat(q_int_vec, q_int_vec[-1] + q_desired_vec[-1] - q_actual_vec[-1])
            q_knee = q_actual_vec[-1, 0]
            q_ankle = q_actual_vec[-1, 1]
            qd_knee = qd_actual_vec[-1, 0]
            qd_ankle = qd_actual_vec[-1, 1]
            q_knee_desired = q_desired_vec[-1, 0]
            q_ankle_desired = q_desired_vec[-1, 1]
            qd_knee_desired = qd_desired_vec[-1, 0]
            qd_ankle_desired = qd_desired_vec[-1, 1]
            tau_knee = Position_control(q_knee, q_knee_desired, qd_knee, qd_knee_desired, q_int_vec[-1, 0], Kp, Ki, Kd)
            tau_ankle = Position_control(q_ankle, q_ankle_desired, qd_ankle, qd_ankle_desired, q_int_vec[-1, 1], Kp, Ki,
                                         Kd)
            torque_given = fifo_mat(torque_given, np.array([tau_knee, tau_ankle]))
            if control_motor:
                send_signal_to_motor(ser, torque_given[-1])
            plot_buf[5] = 0
            plot_buf[6] = q_ankle_desired
            plot_buf[7] = 0
            plot_buf[8] = 0
        else:
            print('\rNone', end='')
            phase = 0
            phase_vec = fifo_mat(phase_vec, phase)
            q_thigh_vec, qd_thigh_vec = basic_ode(q_thigh, q_thigh_vec, qd_thigh_vec, dt)
            q_desired = np.array([0, 0])
            q_desired_vec, qd_desired_vec = basic_ode(q_desired, q_desired_vec, qd_desired_vec, dt)
            # 读取关节角度，并存取
            if control_motor:
                q_actual, q_actual_d = read_signal_from_motor(ser)
            else:
                q_actual = np.array([0, 0])
            q_actual_vec, qd_actual_vec = basic_ode(q_actual, q_actual_vec, qd_actual_vec, dt)
            q_int_vec = fifo_mat(q_int_vec, q_int_vec[-1] + q_desired_vec[-1] - q_actual_vec[-1])
            # 抽取状态变量
            q_knee = q_actual_vec[-1, 0]
            q_ankle = q_actual_vec[-1, 1]
            q_knee_desired = q_desired_vec[-1, 0]
            q_ankle_desired = q_desired_vec[-1, 1]
            qd_knee = qd_actual_vec[-1, 0]
            qd_ankle = qd_actual_vec[-1, 1]
            qd_knee_desired = qd_desired_vec[-1, 0]
            qd_ankle_desired = qd_desired_vec[-1, 1]
            tau_knee = Position_control(q_knee, q_knee_desired, qd_knee, qd_knee_desired, q_int_vec[-1, 0], Kp,
                                        Ki, Kd)
            tau_ankle = Position_control(q_ankle, q_ankle_desired, qd_ankle, qd_ankle_desired, q_int_vec[-1, 1],
                                         Kp, Ki, Kd)
            torque_given = fifo_mat(torque_given, np.array([tau_knee, tau_ankle]))
            if control_motor:
                send_signal_to_motor(ser, torque_given[-1])
                plot_buf[5] = 0
                plot_buf[6] = q_ankle_desired
                plot_buf[7] = 0
                plot_buf[8] = 0
        plot_buf[0] = q_thigh
        plot_buf[1] = acc_z
        plot_buf[2] = phase
        plot_buf[3] = phase_predictor.get_State()
        plot_buf[4] = phase_predictor.get_Threshold()[0]
        print(phase_predictor.get_Threshold())
        plot_buf[9] = phase_predictor.get_Threshold()[1]
        plot_buf.flush()
        time.sleep(8e-3)
    np.save('results/q_desired.npy', q_desired_vec)
    np.save('results/q_actual.npy', q_actual_vec)
    np.save('results/q_thigh.npy', q_thigh_vec)
    np.save('results/phase.npy', phase_vec)
    view_trajectory(time_vec, q_thigh_vec, q_desired_vec, q_actual_vec, phase_vec)

    if control_motor:
        q_final = q_desired_vec[-1]
        qd_final = qd_desired_vec[-1]
        time_vec = np.array([-0.02, -0.02])
        q_actual_vec, qd_actual_vec, q_desired_vec, qd_desired_vec, q_int_vec = pid_buffer_block_init(time_step_num, 2)
        for i in range(time_step_num):
            q_actual, q_actual_d = read_signal_from_motor(ser)
            q_actual_vec, qd_actual_vec = basic_ode(q_actual, q_actual_vec, qd_actual_vec, dt)
            q_desired_vec, qd_desired_vec = basic_ode(q_final, q_desired_vec, qd_desired_vec, dt)
            q_int_vec = fifo_mat(q_int_vec, q_int_vec[-1] + q_desired_vec[-1] - q_actual_vec[-1])
            # 抽取状态变量
            q_knee = q_actual_vec[-1, 0]
            q_ankle = q_actual_vec[-1, 1]
            q_knee_desired = q_desired_vec[-1, 0]
            q_ankle_desired = q_desired_vec[-1, 1]
            qd_knee = qd_actual_vec[-1, 0]
            qd_ankle = qd_actual_vec[-1, 1]
            qd_knee_desired = qd_desired_vec[-1, 0]
            qd_ankle_desired = qd_desired_vec[-1, 1]
            tau_knee = Position_control(q_knee, q_knee_desired, qd_knee, qd_knee_desired, q_int_vec[-1, 0], Kp,
                                        Ki, Kd)
            tau_ankle = Position_control(q_ankle, q_ankle_desired, qd_ankle, qd_ankle_desired, q_int_vec[-1, 1],
                                         Kp, Ki, Kd)
            torque_given = fifo_mat(torque_given, np.array([tau_knee, tau_ankle]))
            send_signal_to_motor(ser, torque_given[-1])

def view_trajectory(time, q_thigh, q_desired, q_actual, phase):
    fig = plt.figure()
    plt.subplot(311)
    plt.plot(time, q_thigh)
    plt.xlabel('Time')
    plt.ylabel('Thigh_Angle')
    plt.subplot(312)
    plt.plot(time, phase)
    plt.xlabel('Time')
    plt.ylabel('Phase')
    plt.subplot(313)
    plt.plot(time, q_desired)
    plt.plot(time, q_actual)
    plt.xlabel('Time')
    plt.ylabel('Thigh_Angle')
    plt.savefig('results/All_info')
    plt.show()


def initialize_q_thigh(imu_data_buf):
    q_thigh_vec = np.zeros(10)
    for i in range(10):
        q_thigh_vec[i] = imu_data_buf[0]
        time.sleep(1e-2)
    q_thigh_init = np.mean(q_thigh_vec)
    print('Initialized q_thigh')
    return q_thigh_init


def read_angle_and_acc(imu_data_buf, q0):
    q_thigh = imu_data_buf[0] - q0
    acc_z = imu_data_buf[5] * 20
    return q_thigh, acc_z


def predict_qd(phase, f_joint_angle):
    joint_angles = f_joint_angle(phase)
    return joint_angles[1:]


def Position_control(q, q_desired, qd, qd_desired, qi, Kp, Ki, Kd):
    tau = Kp * (q_desired - q) + Kd * (qd_desired - qd) + Ki * qi
    return tau


if __name__ == '__main__':
    main()
