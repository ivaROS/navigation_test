from __future__ import print_function
from __future__ import division
from builtins import filter
from builtins import next
from builtins import str
from builtins import range
from past.builtins import basestring
from builtins import object
from past.utils import old_div
import csv
import time
import math
import numpy as np
import copy
import os
import rosbag

def readFile(filename):
    expanded_filename = os.path.expanduser(filename)

    with open(expanded_filename, 'r') as csvfile:
        datareader = csv.DictReader(csvfile, restval='')

        result_list = []
        fieldnames = datareader.fieldnames
        for entry in datareader:
            should_warn=True
            for k,v in list(entry.items()):
                if k != v:
                    should_warn = False
                    break

            if should_warn:
                print("Warning! File [" + expanded_filename + "] appears to have extra fieldnames!")
            else:
                result_list.append(entry)
            
        return result_list

def getAvgSpeed(result_list):
    avg_speed_list = []

    for result in result_list:
        path_length = float(result["path_length"])
        path_time = float(result["time"])

        path_time = path_time / 1e9

        avg_speed = path_length / path_time

        avg_speed_list.append(avg_speed)

    avg_speed_total = sum(avg_speed_list) / len(avg_speed_list)

    return avg_speed_total, avg_speed_list


if __name__ == "__main__":
    filepath = '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/'
    filenames = [
                'results_1_sg_fast_0.4_p_k_nonstop_mr_fix8_1.1_orient_20_rangefix_2024-08-17 22:30:44.994437',
                'results_1_sg_fast_0.4_b_k_nonstop_mr_fix8_1.1_orient_20_rangefix_2024-08-17 17:36:49.357343',
                'results_1_sg_fast_0.4_p_ni_m_k_nonstop_mr_fix8_1.1_orient_20_rangefix_2024-08-18 02:34:11.845958',
                'results_1_sg_fast_0.4_b_ni_m_k_nonstop_mr_fix8_1.1_orient_20_rangefix_2024-08-17 06:33:41.721637',
                'results_1_sg_fast_0.4_p_ni_m_k_nonstop_po_mr_fix8_1.1_orient_20_rangefix_2024-08-21 23:12:29.664800',
                'results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_mr_fix8_1.1_orient_20_rangefix_2024-08-14 04:31:28.580234',
                'results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rangefix_2023-09-09 15:01:02.943943',
                'results_1_sg_fast_0.4_p_nonstop_po_mr_fix8_1.1_orient_20_rangefix_2024-08-18 18:55:06.681977',
                ]
    
    filepath = '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/'
    filenames = [
                'results_1_sg_fast_0.4_p_pose_mr_gp1_fix8_rfix_2024-08-20 18:58:41.367978',
                'results_1_sg_fast_0.4_b_pose_mr_gp1_fix8_rfix_2024-08-20 05:35:54.379717',
                'results_1_sg_fast_0.4_p_ni_m_k_mr_gp1_fix8_rfix_2024-08-21 03:42:49.399551',
                'results_1_sg_fast_0.4_b_ni_m_k_mr_gp1_fix8_rfix_2024-08-20 20:46:17.832509',
                'results_1_sg_fast_0.4_p_ni_m_k_po_mr_gp1_fix8_rfix_2024-08-21 19:23:10.100091',
                'results_1_sg_fast_0.4_b_ni_m_k_po_mr_gp1_fix8_rfix_2024-08-20 06:56:13.493027',
                'results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rfix_2023-09-10 04:13:35.513124',
                'results_1_sg_fast_0.4_p_pose_po_mr_gp1_fix8_rfix_2024-08-21 01:51:42.893069',
                ]
    
    for f in filenames:
        filename = filepath + f

        result_list = readFile(filename)
        avg_speed_total, avg_speed_list = getAvgSpeed(result_list)
        print(f)
        print("Average speed: " + str(avg_speed_total) + " m/s.")