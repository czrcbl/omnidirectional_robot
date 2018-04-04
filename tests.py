from main import make_S_traj, make_quad_traj
from serial_com import init_serial, send_command, recv_message, send_control_signal
from main import wheel2states
import numpy as np
from scipy.io import savemat
import serial
# ref = make_S_traj()
# print(ref)

# ref = make_quad_traj()
# print(ref)

try:
    ser = serial.Serial('/dev/ttyUSB0')
    ser.baudrate = 115200
    print('Port Openned:', ser.name)
except Exception as e:
    print(e)

N = 20
states_vec = np.empty(shape=(N, 3))
control_signal_vec = np.empty(shape=(N, 3))
wheel_vel_vec = np.empty(shape=(N, 3))

test_n = 2

if test_n == 0:
    # 1 Deslocamento frontal
    init_serial(ser)
    control_signal = np.array([0, 2, -2])
    for n in range(N):
        data = recv_message(ser)
        vel = np.array([data.m1_vel, data.m2_vel, data.m3_vel])
        print('wheel', vel)
        states = wheel2states(vel)
        print('states', states)
        send_control_signal(control_signal, ser)
        wheel_vel_vec[n, :] = vel.flatten()
        states_vec[n, :] = states.flatten()
        control_signal_vec[n, :] = control_signal.flatten()

    send_control_signal(np.array([0, 0, 0]), ser)
    savemat('openloop/frontal', {'control': control_signal_vec,
                                 'states': states_vec,
                                 'wheel_vel:': wheel_vel_vec})

elif test_n == 1:
    # 2 Deslocamento normal
    init_serial(ser)
    control_signal = np.array([-2, 1, 1])
    for n in range(N):
        data = recv_message(ser)
        vel = np.array([data.m1_vel, data.m2_vel, data.m3_vel])
        wheel_vel_vec[n, :] = vel.flatten()
        print('wheel', vel)
        states = wheel2states(vel)
        states_vec[n, :] = states.flatten()
        print('states', states)
        send_control_signal(control_signal, ser)
        control_signal_vec[n, :] = control_signal.flatten()

    send_control_signal(np.array([0, 0, 0]), ser)
    savemat('openloop/normal', {'control': control_signal_vec,
                                'states': states_vec,
                                'wheel_vel:': wheel_vel_vec})

elif test_n == 2:
    # 3 Deslocamento angular
    init_serial(ser)
    control_signal = np.array([1, 1, 1])
    for n in range(N):
        data = recv_message(ser)
        vel = np.array([data.m1_vel, data.m2_vel, data.m3_vel])
        wheel_vel_vec[n, :] = vel.flatten()
        print('wheel', vel)
        states = wheel2states(vel)
        states_vec[n, :] = states.flatten()
        print('states', states)
        send_control_signal(control_signal, ser)
        control_signal_vec[n, :] = control_signal.flatten()

    send_control_signal(np.array([0, 0, 0]), ser)
    savemat('openloop/angular', {'control': control_signal_vec,
                                 'states': states_vec,
                                 'wheel_vel:': wheel_vel_vec})
