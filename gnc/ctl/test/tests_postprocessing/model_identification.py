import pandas as pd
import numpy as np
import glob
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import curve_fit

TEST_RUN = '20210622_0747'
LOG_PATH = f'/Volumes/ros_logs/{TEST_RUN}/Simple_translation_40cm_print_step_0um_print_from_0cm/'

def main(test_run, log_folder):
    samples = glob.glob(log_folder+r'*')
    csv_files = [os.path.basename(test_case_path) for test_case_path in samples ]

    ref = pd.read_csv(LOG_PATH+r'oc-reference.csv')
    com = pd.read_csv(LOG_PATH + r'nc-ctl-command.csv')
    pmc = pd.read_csv(LOG_PATH + r'w-pmc-command.csv')
    vel = pd.read_csv(LOG_PATH + r'oc-truth-twist.csv')
    pos = pd.read_csv(LOG_PATH + r'oc-truth-pose.csv')

    ref.drop_duplicates(subset=['header.seq'], inplace=True)
    ref.reset_index(drop=True, inplace=True)
    ref['Time'] = ref['header.stamp.secs'] + ref['header.stamp.nsecs'] / 10e8
    com.drop_duplicates(subset=['header.seq'], inplace=True)
    com.reset_index(drop=True, inplace=True)
    com['Time'] = com['header.stamp.secs'] + com['header.stamp.nsecs'] / 10e8
    pmc.drop_duplicates(subset=['header.seq'], inplace=True)
    pmc.reset_index(drop=True, inplace=True)
    pmc['Time'] = pmc['header.stamp.secs'] + pmc['header.stamp.nsecs'] / 10e8
    vel.drop_duplicates(subset=['header.seq'], inplace=True)
    vel.reset_index(drop=True, inplace=True)
    vel['Time'] = vel['header.stamp.secs'] + vel['header.stamp.nsecs'] / 10e8
    pos.drop_duplicates(subset=['header.seq'], inplace=True)
    pos.reset_index(drop=True, inplace=True)
    pos['Time'] = pos['header.stamp.secs'] + pos['header.stamp.nsecs'] / 10e8

    '''plt.plot(vel['twist.linear.y'].diff().rolling(100).mean() * 62.5)
    plt.plot(com['wrench.force.y'] / 9.087)
    plt.show()
    plt.plot(vel['twist.linear.y'])
    plt.show()'''



    quat = pos[
        ['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
    rot = Rotation.from_quat(quat)
    rot_deg = rot.as_euler('xyz', degrees=True)
    rot_deg = pd.DataFrame(rot_deg, columns=['x', 'y', 'z'])
    a_ang_z = rot_deg.z.diff().diff()
    dt_pos = pos.Time.diff()
    a_ang_z = a_ang_z * np.pi / (dt_pos * 180)
    plt.plot(a_ang_z.rolling(10).mean().clip(-1.5, 1.5), label="acc from pos change")

    plt.plot((vel['twist.angular.z'].diff() / vel['Time'].diff()).rolling(10).mean().clip(-0.005, 0.005),
             label="acc from vel change")
    plt.plot((com['wrench.torque.z'].rolling(10).mean() / 0.1454).clip(-0.005, 0.005), label="acc from torque")
    plt.legend()
    plt.show()


    plt.plot(vel['twist.angular.z'].rolling(10).mean(), label="recorded vel")
    plt.plot((rot_deg.z.diff() * np.pi / (dt_pos * 180)).rolling(10).mean(), label="calc vel from pos")
    plt.legend()
    plt.show()


    pass

    plt.plot(pos['pose.position.x'])
    plt.show()
    model = get_pos_model()
    model_out_pos = []
    model_out_vel = []
    t_last = com['Time'][0] - model.dt
    for t_c, command in com[['Time', 'wrench.force.x']].values:
        t = t_c - t_last
        t_last = t_c
        y_temp, x_temp = model.step(command, t)
        model_out_pos.append(y_temp)
        model_out_vel.append(x_temp[1,0])
    plt.plot(model_out_vel, label="modeled velocity")
    plt.plot(vel['twist.linear.x'], label="recorded velocity")
    plt.legend()
    plt.show()

    pos_x = pos[['Time', 'pose.position.x']]
    vel_x = vel[['Time', 'twist.linear.x']]
    com_x = com[['Time', 'wrench.force.x']]

    vel_and_com_x = pd.merge(vel_x, com_x, on="Time")
    dt = vel_and_com_x.Time.diff()[1:]
    vel_and_com_x = vel_and_com_x[1:]
    vel_and_com_x.Time = dt
    data_in = vel_and_com_x[:-1].values
    data_out = vel_and_com_x[1:]['twist.linear.x'].values
    def vel_data(x, C):
        y = 0.99664271*x[:, 1] + x[:, 0] * x[:, 2] / 9.087 + C*np.sign(x[:, 1])*abs(x[:, 1])**0.5#abs(x[:, 1])
        return y

    parameters, covariance = curve_fit(vel_data, data_in, data_out)

    ff = 9.087 * vel_and_com_x['twist.linear.x'].diff() / vel_and_com_x.Time
    plt.plot(vel_and_com_x['wrench.force.x'], label="recorded force")
    plt.plot(ff, label="predicted force")
    plt.legend()
    plt.show()

    pass



class Model:
    def __init__(self, A, B, C, D, E=[[0,0], [0,0]], dt=1/62.5, m=None):
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.D = np.array(D)
        self.E = np.array(E)
        self.dt = dt
        self.m = m
        self.x = np.array([[0], [0]])

    def step(self, u, dt=None):
        if dt is None:
            #dt = self.dt
            self.A[0, 1] = self.dt
            self.B[1, 0] = self.dt / self.m
        else:
            self.A[0, 1] = dt
            self.B[1, 0] = dt / self.m
        x_next = np.dot(self.A, self.x) + np.dot(self.B, u) + np.dot(self.E, self.x*abs(self.x))
        y = np.dot(self.C, x_next)
        #print(y)
        #print(x_next)
        self.x = x_next
        return y[0, 0], x_next

def get_pos_model():
    m = 9.087 #9.087
    A = [[0., 1.], [0., 0.]]
    B = [[0.], [1.0 / m]]
    C = [[1., 0.]]
    D = 0
    dt = 1.0 / 62.5

    dA = [[1, dt], [0, 0.99664271]]
    dB = [[0.], [dt / m]]
    dC = C
    dD = D
    #drag_ = -0.5 * 1.05 * 0.092903 * 1.225
    drag_ = 0.00691741
    dE = [[0., 0.], [0, drag_]]
    model = Model(dA, dB, dC, dD, dE, dt, m)
    return model


if __name__ == "__main__":
    main(TEST_RUN, LOG_PATH)