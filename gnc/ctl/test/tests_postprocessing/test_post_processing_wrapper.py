import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from generate_report import PDF
from test_data_ros_bag_extraction import generate_kpi_wrapper
import glob

TEST_RUN = '20210617_2343'
LOG_PATH = f'/Volumes/ros_logs/{TEST_RUN}'

def main(test_run, log_folder):
    samples = glob.glob(log_folder+r'/*.bag')
    test_cases = [os.path.basename(test_case_path)[:-4] for test_case_path in samples ]


    for name in test_cases:
        print(f"Done: {name}")
        generate_kpi_wrapper(name, log_folder)
    
    pdf = PDF()
    for test_name in test_cases:
        pdf.print_page(f'{log_folder}/kpi_extracted/', test_name)
    output_name = f'Controller_report_{test_run}.pdf'
    output_path = f'{os.path.dirname(os.path.abspath(__file__))}/Reports/{output_name}'
    pdf.output(output_path, 'F')
    print(f"Done file: {output_path}")

if __name__ == "__main__":
    main(TEST_RUN, LOG_PATH)