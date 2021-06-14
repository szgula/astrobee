import os
import shutil
import numpy as np
import pandas as pd
import calendar
from datetime import datetime
from fpdf import FPDF

import matplotlib.pyplot as plt
import pickle


class PDF(FPDF):
    def __init__(self):
        super().__init__()
        self.WIDTH = 210
        self.HEIGHT = 297
        self.set_left_margin(30)
        self.base_intend = 30
        self.second_image_intend = self.base_intend + 10 + self.WIDTH / 3
        self.picture_size = self.WIDTH / 3

    def header(self):
        # Custom logo and positioning
        # Create an `assets` folder and put any wide and short image inside
        # Name the image `logo.png`
        # self.image('assets/logo.png', 10, 8, 33)
        self.set_font('Arial', 'B', 8)
        self.cell(self.WIDTH - 100)
        self.cell(60, 1, 'CTL report', 0, 0, 'R')
        self.ln(20)

    def footer(self):
        # Page numbers in the footer
        self.set_y(-15)
        self.set_font('Arial', 'I', 8)
        self.set_text_color(128)
        self.cell(0, 10, 'Page ' + str(self.page_no()), 0, 0, 'C')

    def print_first_page(self, folder, name):
        pose_file_name = f'{folder}pose_time_sign_{name}.png'
        rot_file_name = f'{folder}rot_time_sign_{name}.png'
        forces_file_name = f'{folder}forces_time_sign_{name}.png'
        torques_file_name = f'{folder}torques_time_sign_{name}.png'
        forces_acc_file_name = f'{folder}forces_accumulated_time_sign_{name}.png'
        torques_acc_file_name = f'{folder}torques_accumulated_time_sign_{name}.png'

        kpi_time_prop_file_name = f'{folder}kpi_time_prop_{name}.png'
        kpi_tracking_file_name = f'{folder}kpi_tracking_{name}.png'
        kpi_effort_file_name = f'{folder}kpi_effort_{name}.png'

        # Test case name
        self.set_font('Arial', 'B', size=10)
        self.set_xy(30, 20)
        self.cell(self.WIDTH - 60, 10, name)

        # Descprion
        pass

        # Time signals
        self.set_font('Arial', size=10)
        self.set_xy(30, 30)
        str_ = "Below are presented time signals (position and orientation respectivly). The continous line represent measured signal, the dashed (--) line is a reference singal."
        # self.cell(self.WIDTH - 60, 25, str_)
        self.write(7, str_)
        first_line_pictures = 45
        self.image(pose_file_name, self.base_intend, first_line_pictures, self.picture_size)
        self.image(rot_file_name, self.second_image_intend, first_line_pictures, self.picture_size)

        # The basic kpi
        base_kpi_line = 100
        self.set_xy(self.base_intend, base_kpi_line)
        str_ = "Basic KPIs:"
        self.write(7, str_)
        self.image(kpi_time_prop_file_name, self.base_intend, base_kpi_line + 10, self.WIDTH / 3)

        # Tracking kpi
        self.set_xy(self.second_image_intend, base_kpi_line)
        str_ = "Tracking KPIs:"
        self.write(7, str_)
        self.image(kpi_tracking_file_name, self.second_image_intend, base_kpi_line + 7, self.WIDTH / 3)

        # Effort kpi
        effort_kpi_line = base_kpi_line + 20
        self.set_xy(self.second_image_intend, effort_kpi_line)
        str_ = "Effort KPIs:"
        self.write(7, str_)
        self.image(kpi_effort_file_name, self.second_image_intend, effort_kpi_line + 7, self.WIDTH / 3)

        # Effort and control plots
        second_line_pictures = effort_kpi_line + 45
        self.set_xy(self.base_intend, second_line_pictures)
        str_ = "Control Plots:"
        self.write(7, str_)
        self.set_xy(self.second_image_intend, second_line_pictures)
        str_ = "Effort Plots:"
        self.write(7, str_)

        self.image(forces_file_name, self.base_intend, second_line_pictures + 10, self.picture_size)
        self.image(forces_acc_file_name, self.second_image_intend, second_line_pictures + 10, self.picture_size)

        third_line_pictures = second_line_pictures + 60
        self.image(torques_file_name, self.base_intend, third_line_pictures, self.picture_size)
        self.image(torques_acc_file_name, self.second_image_intend, third_line_pictures, self.picture_size)

        self.line(self.second_image_intend - 5, second_line_pictures, self.second_image_intend - 5,
                  third_line_pictures + 50)

    def print_page(self, folder, name):
        # Generates the report
        self.add_page()
        self.print_first_page(folder, name)


# Base PID with not print
samples_base_PID_no_print = [
        #('plots', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210611_1053', 'Simple_rotation_66deg'),
        (r'/Volumes/ros_logs/20210611_1055', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210611_1110', 'Translation_with_rotation')
    ]

# Base PID with 30cm print
samples = [
        #('plots', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210613_2118', 'Simple_rotation_66deg'),
        (r'/Volumes/ros_logs/20210613_2118', 'Simple_traslation_40cm'),
        (r'/Volumes/ros_logs/20210613_2118', 'Translation_with_rotation')
    ]

pdf = PDF()
for folder_, test_name in samples:
    pdf.print_page(f'{folder_}/kpi_extracted/', test_name)
pdf.output('Controller_report_30com_print_base_PID.pdf', 'F')
