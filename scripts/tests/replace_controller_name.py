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

def replace_controller(filename):
    expanded_filename = os.path.expanduser(filename)

    search_text = "bgap_mpc_casadi" # "potentialgap", "bgap_mpc_casadi", "potential_gap"

    sub1 = "results_1"
    sub2 = "_2023"
    # getting index of substrings
    idx1 = filename.index(sub1)
    idx2 = filename.index(sub2)
    append_text = filename[idx1 + len(sub1): idx2]
    replace_text = search_text + append_text

    with open(expanded_filename, 'r') as file:
    
        # Reading the content of the file
        # using the read() function and storing
        # them in a new variable
        data = file.read()

        if replace_text in data:
            print("Already renamed.")
            return
    
        # Searching and replacing the text
        # using the replace() function
        data = data.replace(search_text, replace_text)
    
    # Opening our text file in write only
    # mode to write the replaced content
    save_file = expanded_filename + "_rename"
    with open(save_file, 'w') as file:
    
        # Writing the replaced data in our
        # text file
        file.write(data)
        print("Renamed.")

        return save_file

def replace_scenario(filename):
    expanded_filename = os.path.expanduser(filename)

    search_text = ["full_sector_laser", "full_campus_obstacle", "full_fourth_floor_obstacle"]

    with open(expanded_filename, 'r') as file:
    
        # Reading the content of the file
        # using the read() function and storing
        # them in a new variable
        data = file.read()

        for s in search_text:
            sub = "full_"
            replace_text = s[s.index(sub) + len(sub):]
    
            # Searching and replacing the text
            # using the replace() function
            data = data.replace(s, replace_text)

    with open(expanded_filename, 'w') as file:
    
        # Writing the replaced data in our
        # text file
        file.write(data)
        print("Renamed.")
    

if __name__ == "__main__":
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 18:06:02.266878',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-02 01:33:35.785750',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_2_2023-08-09 18:30:58.142352',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_2023-08-14 21:06:13.967224',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_mpc_k_noc_nostop_2023-08-15 01:18:19.834283',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 21:49:18.317252']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_3_3.5_2_fix6_rangefix_2023-09-08 03:50:35.546407',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rangefix_2023-09-07 19:17:10.663522',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rangefix_2023-09-09 15:01:02.943943',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_3_3.5_2_fix6_1.1_orient_20_rangefix_2023-09-09 00:12:53.317953',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_2_3.5_0.5_fix6_1.1_orient_20_rangefix_2023-09-08 19:55:36.717437',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_1.1_rangefix_2023-09-08 13:01:36.542370',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_1.1_orient_20_rangefix_2023-09-09 03:05:53.425630',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_fix6_1.1_rangefix_2023-09-09 18:24:50.182957',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_fix6_1.1_orient_20_rangefix_2023-09-09 15:01:56.070250',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2023-09-08 05:14:04.053888']

    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 17:06:28.065570',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-06 01:49:17.732358',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_0.5_2023-08-08 03:13:14.843171',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_pose_noff_2023-08-01 00:59:52.661876',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_nostop_2023-08-15 17:47:55.757627',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 17:54:35.093422']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_4_4_0.5_fix6_rfix_2023-09-11 15:15:42.242858',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rfix_2023-09-10 15:33:53.546143',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rfix_2023-09-10 04:13:35.513124',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_4_4_0.5_fix6_orient_20_rfix_2023-09-12 00:03:59.885116',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_2_3.5_0.5_fix6_orient_20_rfix_2023-09-10 23:36:22.745163',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_rfix_2023-09-11 03:13:48.687762',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_orient_20_rfix_2023-09-11 01:28:55.923747',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_a5_fix6_rfix_2023-09-11 22:11:18.320517',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_a5_fix6_orient_20_rfix_2023-09-11 19:22:56.630124',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_a5_fix6_orient_20_rfix_2023-09-11 17:25:36.505512']

    for f in filenames:
        replace_controller(f)

    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_noc_2023-08-06 16:59:09.886338',
    #              '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:57:23.256144']

    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_noc_2023-08-06 16:25:35.877280',
    #              '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:11:30.600690']

    # for f in filenames:
    #     new_f = replace_controller(f)
    #     replace_scenario(new_f)
