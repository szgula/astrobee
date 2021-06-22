import pandas as pd
import numpy as np
import glob
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

TEST_RUN = '20210622_0643'
LOG_PATH = f'/Volumes/ros_logs/{TEST_RUN}/ModelIdentificationTest/'

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



if __name__ == "__main__":
    main(TEST_RUN, LOG_PATH)