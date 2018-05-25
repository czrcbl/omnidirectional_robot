from main import make_S_traj, make_quad_traj
from serial_com import init_serial, send_command, recv_message,\
    send_control_signal, recv_message2
from main import wheel2states
import numpy as np
from scipy.io import savemat
import serial
from utils import now
import pickle
# ref = make_S_traj()
# print(ref)

# ref = make_quad_traj()
# print(ref)


def save_p(data, file_name):
    with open(file_name, 'w') as f:
        pickle.dump(data, f)


def load_p(file_name):
    with open(file_name, 'r') as f:
        data = pickle.load(f)
    return data


if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB0')
        ser.baudrate = 115200
        print('Port Openned:', ser.name)
    except Exception as e:
        print(e)

    N = 40  # Test Size
    test_n = 2

    if test_n == 0:
        control_signal = np.array([0, 2, -2])
        test_name = now() + '_frontal'
    elif test_n == 1:
        control_signal = np.array([-2, 1, 1])
        test_name = now() + '_normal'
    elif test_n == 2:
        control_signal = np.array([1, 1, 1])
        test_name = now() + '_angular'

    states_vec = np.empty(shape=(N, 3))
    control_signal_vec = np.empty(shape=(N, 3))
    wheel_vel_vec = np.empty(shape=(N, 3))
    ser_messages = []

    init_serial(ser)

    for n in range(N):
        data, ser_mess = recv_message2(ser)
        vel = np.array([data.m1_vel, data.m2_vel, data.m3_vel])
        print('wheel', vel)
        states = wheel2states(vel)
        print('states', states)
        send_control_signal(control_signal, ser)

        ser_messages.append(ser_mess)
        wheel_vel_vec[n, :] = vel.flatten()
        states_vec[n, :] = states.flatten()
        control_signal_vec[n, :] = control_signal.flatten()

    send_control_signal(np.array([0, 0, 0]), ser)
    savemat('openloop/' + test_name, {'control': control_signal_vec,
                                      'states': states_vec,
                                      'wheel_vel': wheel_vel_vec})
    save_p(ser_messages, 'openloop/' + test_name + 'pickle')
