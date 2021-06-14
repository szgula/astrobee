import bagpy
from bagpy import bagreader
import pandas as pd
#import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from collections import defaultdict
import pickle
import dataframe_image as dfi
import os


class CtlPostprocessing:
    def __init__(self, path_to_file):
        #path_to_bag = r'Simple_traslation_40cm.bag'
        b = bagreader(path_to_file)
        # change folder and sub-folders permission (needs to be "writable" by other)

        self.ctl = pd.read_csv(b.message_by_topic('gnc/ctl/command'))
        self.pmc = pd.read_csv(b.message_by_topic('hw/pmc/command'))
        self.reference = pd.read_csv(b.message_by_topic('loc/reference'))
        self.pose = pd.read_csv(b.message_by_topic('loc/truth/pose'))
        self.vel = pd.read_csv(b.message_by_topic('loc/truth/twist'))
        # FIXME all data contains duplicated and missing records - take a look at header.seq

    def calculate_effort_kpis(self):
        # Cumulative forces over whole test for each axis
        cum_forces = self.ctl[['wrench.force.x', 'wrench.force.y', 'wrench.force.z']].abs().sum().to_numpy()
        tot_force = cum_forces.sum()
        # Mean forces for update step for each axis
        mean_forces = self.ctl[['wrench.force.x', 'wrench.force.y', 'wrench.force.z']].abs().mean().to_numpy()
        # Cumulative torque over whole test for each axis
        cum_torque = self.ctl[['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']].abs().sum().to_numpy()
        tot_torque = cum_torque.sum()
        # Mean torque for update step for each axis
        mean_torque = self.ctl[['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']].abs().mean().to_numpy()

        # cumulative abs(force derivative) - this represents how jerky control signal is
        cum_forces_dif = self.ctl[['wrench.force.x', 'wrench.force.y', 'wrench.force.z']].diff().abs().sum().to_numpy()
        # mean abs(force derivative) - this represents how jerky control signal is
        mean_forces_dif = self.ctl[['wrench.force.x', 'wrench.force.y', 'wrench.force.z']].diff().abs().mean().to_numpy()
        # cumulative abs(force derivative) - this represents how jerky control signal is
        cum_torque_dif = self.ctl[['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']].diff().abs().sum().to_numpy()
        # mean abs(force derivative) - this represents how jerky control signal is
        mean_torque_dif = self.ctl[['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']].diff().abs().mean().to_numpy()

        output_xyz = {"cum_forces": cum_forces,
                  "mean_forces": mean_forces,
                  "cum_torque": cum_torque,
                  "mean_torque": mean_torque,
                  "cum_forces_dif": cum_forces_dif,
                  "mean_forces_dif": mean_forces_dif,
                  "cum_torque_dif": cum_torque_dif,
                  "mean_torque_dif": mean_torque_dif}
        output_aggregated = {
                  "tot_force": tot_force,
                  "tot_torque": tot_torque,
        }

        return output_xyz, output_aggregated


    def calculate_tracking_errors(self):
        pos = self.pose[['pose.position.x', 'pose.position.y', 'pose.position.z']]
        ref_pos = self.reference[['pose.position.x', 'pose.position.y', 'pose.position.z']]
        pos_diff = ref_pos - pos
        cum_pos_tracking_error = abs(pos_diff).sum().to_numpy()
        mean_pos_tracking_error = abs(pos_diff).mean().to_numpy()
        tot_pos_tracking_error = cum_pos_tracking_error.sum()

        quat = self.pose[['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        quat_ref = self.reference[['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        rot = Rotation.from_quat(quat)
        rot_ref = Rotation.from_quat(quat_ref)
        rot_deg = rot.as_euler('xyz', degrees=True)
        rot_deg_ref = rot_ref.as_euler('xyz', degrees=True)
        rot_diff = rot_deg_ref - rot_deg
        cum_rot_tracking_error = abs(rot_diff).sum(axis=0)
        mean_rot_tracking_error = abs(rot_diff).mean(axis=0)
        tot_rot_tracking_error = cum_rot_tracking_error.sum()

        output_xyz = {
            "cum_pos_tracking_error": cum_pos_tracking_error,
            "mean_pos_tracking_error": mean_pos_tracking_error,
            "cum_rot_tracking_error": cum_rot_tracking_error,
            "mean_rot_tracking_error": mean_rot_tracking_error,
        }
        output_aggregated = {
            "tot_pos_tracking_error": tot_pos_tracking_error,
            "tot_rot_tracking_error": tot_rot_tracking_error,
        }
        return output_xyz, output_aggregated

    @staticmethod
    def change_reference_step_signals(signal, change_threshold):
        ref_dif = signal.diff().abs() > change_threshold
        coord = np.where(ref_dif)
        coordinates = [(y, x) for x, y in zip(coord[0], coord[1])]
        coordinates.sort(key=lambda x: x[0])
        out = defaultdict(list)
        for y, x in coordinates:
            out[y].append(x)
        return out

    @staticmethod
    def get_basic_time_kpis(ref, real, time_series, type_):
        # MISSING steady state error - how to calculate it if we don't wait for signal settling?
        rise_time_start_threshold = 0.1  # 10% of (reference - start)
        rise_time_end_threshold = 0.9  # 90% of (reference - start)
        settling_time_threshold = 0.1  # +/-10% of (reference - start)
        position_reference_change_threshold = 0.05  # 5cm
        orientation_reference_change_threshold = np.deg2rad(5)  # 5deg
        if type_ == "position":
            threshold_ = position_reference_change_threshold
        elif type_ == "orientation":
            threshold_ = orientation_reference_change_threshold
        else:
            raise Exception("type needs to be position or orientation")

        ref_changes = CtlPostprocessing.change_reference_step_signals(ref, threshold_)
        kpi_values = []
        for column_id, value_change_ids in ref_changes.items():
            for switch_id, value_change_id in enumerate(value_change_ids):
                new_target_value = ref.iloc[value_change_id, column_id]
                """ we consider samples between <value_change_id, end_period_id> """
                end_period_id = value_change_ids[switch_id + 1] if switch_id + 1 < len(value_change_ids) else len(
                    real) - 1
                system_output = real.iloc[value_change_id:end_period_id, column_id]

                initial_value = real.iloc[value_change_id, column_id]
                diff = new_target_value - initial_value
                rise_time_start_value = rise_time_start_threshold * diff + initial_value
                rise_time_stop_value = rise_time_end_threshold * diff + initial_value
                settle_time_margin = abs(diff * settling_time_threshold)

                if new_target_value > initial_value:
                    rise_time_start_index = (system_output > rise_time_start_value).lt(True).idxmin()
                    rise_time_stop_index = (system_output > rise_time_stop_value).lt(True).idxmin()
                    overshoot = (system_output.max() - initial_value) / diff
                    overshoot = max(overshoot - 1, 0) * 100
                else:
                    rise_time_start_index = (system_output < rise_time_start_value).lt(True).idxmin()
                    rise_time_stop_index = (system_output < rise_time_stop_value).lt(True).idxmin()
                    overshoot = (system_output.min() - initial_value) / diff
                    overshoot = max(overshoot - 1, 0) * 100

                settling_time_stop_index = ((system_output - new_target_value).abs() > settle_time_margin).lt(
                    True).idxmax()
                # FIXME -> settling_time_stop_index many not exist
                rise_time = time_series[rise_time_stop_index] - time_series[rise_time_start_index]
                settling_time = time_series[settling_time_stop_index] - time_series[value_change_id]
                event_time = time_series[end_period_id] - time_series[value_change_id]
                single_event_kpi = {"typle": type_, "axis": column_id, "diff": diff, "overshoot": overshoot,
                                    "rise_time": rise_time, "settling_time": settling_time,
                                    "event_time": event_time}
                kpi_values.append(single_event_kpi)
        return kpi_values

    @staticmethod
    def get_eulers_from_quaternions(quat, as_pandas=False):
        rot = Rotation.from_quat(quat)
        rot_deg = rot.as_euler('xyz', degrees=True)
        if as_pandas:
            rot_deg = pd.DataFrame(rot_deg, columns=["rot_x", "rot_y", "rot_z"])
        return rot_deg

    def calculate_time_domain_errors_kpis(self):
        ref_pos = self.reference[['pose.position.x', 'pose.position.y', 'pose.position.z']]
        pos = self.pose[['pose.position.x', 'pose.position.y', 'pose.position.z']]
        kpi_values_lin = CtlPostprocessing.get_basic_time_kpis(ref_pos, pos, self.pose.Time, "position")

        quat = self.pose[
            ['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        quat_ref = self.reference[
            ['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        rot_deg = self.get_eulers_from_quaternions(quat, True)
        rot_deg_ref = self.get_eulers_from_quaternions(quat_ref, True)
        kpi_values_rot = CtlPostprocessing.get_basic_time_kpis(rot_deg_ref, rot_deg, self.pose.Time, "orientation")

        return kpi_values_lin + kpi_values_rot

    def plot_(self, folder_, name):
        # plot time signals
        c = ['b', 'g', 'r']
        t = self.pose.Time - min(self.pose.Time)
        t_ref = self.reference.Time - min(self.reference.Time)
        for idx, pos_signal_name in enumerate(['pose.position.x', 'pose.position.y', 'pose.position.z']):
            plt.plot(t,  self.pose[pos_signal_name], c[idx], label=pos_signal_name)
            plt.plot(t_ref, self.reference[pos_signal_name],  f'{c[idx]}--')
        plt.title("Position signals")
        plt.xlabel("Time [s]")
        plt.ylabel("Distance [m]")
        plt.legend()
        plt.savefig(f'{folder_}/pose_time_sign_{name}.png')
        plt.clf()

        quat = self.pose[
            ['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        quat_ref = self.reference[
            ['pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z', 'pose.orientation.w']].to_numpy()
        rot_deg = self.get_eulers_from_quaternions(quat, True)
        rot_deg_ref = self.get_eulers_from_quaternions(quat_ref, True)
        for idx, signal_name in enumerate(['rot_x', 'rot_y', 'rot_z']):
            plt.plot(t,  rot_deg[signal_name], c[idx], label=signal_name)
            plt.plot(t_ref, rot_deg_ref[signal_name],  f'{c[idx]}--')
        plt.title("Orientation signals")
        plt.xlabel("Time [s]")
        plt.ylabel("Rotation [deg]")
        plt.legend()
        plt.savefig(f'{folder_}/rot_time_sign_{name}.png')
        plt.clf()
        # 1 -> pos reference, pos
        # 2 -> rot reference, rot
        # 3 -> (past) cum effort

        t = self.ctl.Time - min(self.ctl.Time)
        for idx, pos_signal_name in enumerate(['wrench.force.x', 'wrench.force.y', 'wrench.force.z']):
            plt.plot(t,  self.ctl[pos_signal_name], c[idx], label=pos_signal_name)
        plt.title("Force ctl signals")
        plt.xlabel("Time [s]")
        plt.ylabel("Force [N]")
        plt.legend()
        plt.savefig(f'{folder_}/forces_time_sign_{name}.png')
        plt.clf()

        for idx, pos_signal_name in enumerate(['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']):
            plt.plot(t,  self.ctl[pos_signal_name], c[idx], label=pos_signal_name)
        plt.title("Torque ctl signals")
        plt.xlabel("Time [s]")
        plt.ylabel("Torque [N*m]")
        plt.legend()
        plt.savefig(f'{folder_}/torques_time_sign_{name}.png')
        plt.clf()

        for idx, pos_signal_name in enumerate(['wrench.force.x', 'wrench.force.y', 'wrench.force.z']):
            plt.plot(t,  self.ctl[pos_signal_name].abs().cumsum(), c[idx], label=pos_signal_name)
        plt.title("Accumulated forces")
        plt.xlabel("Time [s]")
        plt.ylabel("Accumulated force [N]")
        plt.legend()
        plt.savefig(f'{folder_}/forces_accumulated_time_sign_{name}.png')
        plt.clf()

        for idx, pos_signal_name in enumerate(['wrench.torque.x', 'wrench.torque.y', 'wrench.torque.z']):
            plt.plot(t,  self.ctl[pos_signal_name].abs().cumsum(), c[idx], label=pos_signal_name)
        plt.title("Accumulated torques")
        plt.xlabel("Time [s]")
        plt.ylabel("Accumulated torque [N*m]")
        plt.legend()
        plt.savefig(f'{folder_}/torques_accumulated_time_sign_{name}.png')
        plt.clf()


def main():
    samples_base_PID_no_print = [
        # ('plots', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210611_1053', 'Simple_rotation_66deg'),
        (r'/Volumes/ros_logs/20210611_1055', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210611_1110', 'Translation_with_rotation')
    ]

    # Base PID with 30cm print
    samples = [
        # ('plots', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210613_2118', 'Simple_rotation_66deg'),
        (r'/Volumes/ros_logs/20210613_2118', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210613_2118', 'Translation_with_rotation')
    ]
    sample = samples[2]
    name = sample[1]
    folder_ = sample[0]
    path = f'{folder_}/{name}.bag'
    kpi_extraction_folder = f'{folder_}/kpi_extracted'
    os.makedirs(kpi_extraction_folder, exist_ok=True)

    ctl_tests = CtlPostprocessing(path)
    kpi_effort, kpi_effort_gen = ctl_tests.calculate_effort_kpis()
    kpi_tracking, kpi_tracking_gen = ctl_tests.calculate_tracking_errors()
    kpi_time_prop = ctl_tests.calculate_time_domain_errors_kpis()
    ctl_tests.plot_(kpi_extraction_folder, name)

    dfi.export(pd.DataFrame(kpi_effort, index=['x', 'y', 'z']), f'{kpi_extraction_folder}/kpi_effort_{name}.png')
    dfi.export(pd.DataFrame(kpi_tracking, index=['x', 'y', 'z']), f'{kpi_extraction_folder}/kpi_tracking_{name}.png')
    dfi.export(pd.DataFrame(kpi_time_prop), f'{kpi_extraction_folder}/kpi_time_prop_{name}.png')

    kpi_aggregated = {
        "name": name,
        "kpi_effort": kpi_effort,
        "kpi_effort_gen": kpi_effort_gen,
        "kpi_tracking": kpi_tracking,
        "kpi_tracking_gen": kpi_tracking_gen,
        "kpi_time_prop": kpi_time_prop,
    }

    with open(f'{kpi_extraction_folder}/kpi_{name}.pickle', 'wb') as f:
        pickle.dump(kpi_aggregated, f)

    return ctl_tests


if __name__ == "__main__":
    ctl_test = main()
