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
            
        return result_list, fieldnames, expanded_filename

def getRepeatedNum(result_list):
    if len(result_list) == 0:
        return 0

    scenario_list = []
    seed_list = []
    repeated_num = 0
    seed_num = result_list[0]['seed']
    if 'world' in result_list[0].keys() and len(result_list[0]['world']) != 0:
        scenario = result_list[0]['world']
    elif 'scenario' in result_list[0].keys() and len(result_list[0]['scenario']) != 0:
        scenario = result_list[0]['scenario']

    for r in result_list:
        if 'world' in r.keys() and len(r['world']) != 0:
            s = r['world']
        elif 'scenario' in r.keys() and len(r['scenario']) != 0:
            s = r['scenario']

        if s not in scenario_list:
            scenario_list.append(s)
        if r['seed'] not in seed_list:
            seed_list.append(r['seed'])

        if r['seed'] == seed_num and s == scenario:
            repeated_num += 1

    return repeated_num, scenario_list, seed_list

def createFiles(result_list, fieldnames, expanded_filename, repeated_num, scenario_list, seed_list):
    results = []
    for i in range(repeated_num):
        r = []
        results.append(r)

    if len(result_list) == 0:
        return

    repeated_arr = np.zeros([len(scenario_list), len(seed_list)])
    
    for r in result_list:
        if 'world' in r.keys() and len(r['world']) != 0:
            s = r['world']
        elif 'scenario' in r.keys() and len(r['scenario']) != 0:
            s = r['scenario']
        scenario_idx = scenario_list.index(s)
        seed_idx = seed_list.index(r['seed'])
        repeated_arr[scenario_idx, seed_idx] += 1
        results[int(repeated_arr[scenario_idx, seed_idx]) - 1].append(r)

    for i in range(repeated_num):
        new_file_name = expanded_filename + '_' + str(i)
        with open(new_file_name, 'w', newline='') as new_csvfile:
            writer = csv.DictWriter(new_csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for final_r in results[i]:
                # print(final_r)
                writer.writerow(final_r)
    

if __name__ == "__main__":
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_3_3.5_2_fix6_rangefix_2023-09-08 03:50:35.546407_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rangefix_2023-09-07 19:17:10.663522_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rangefix_2023-09-09 15:01:02.943943_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_3_3.5_2_fix6_1.1_orient_20_rangefix_2023-09-09 00:12:53.317953_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_2_3.5_0.5_fix6_1.1_orient_20_rangefix_2023-09-08 19:55:36.717437_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_1.1_rangefix_2023-09-08 13:01:36.542370_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_1.1_orient_20_rangefix_2023-09-09 03:05:53.425630_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_fix6_1.1_rangefix_2023-09-09 18:24:50.182957_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_fix6_1.1_orient_20_rangefix_2023-09-09 15:01:56.070250_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2023-09-08 05:14:04.053888_rename']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_4_4_0.5_fix6_rfix_2023-09-11 15:15:42.242858_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rfix_2023-09-10 15:33:53.546143_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rfix_2023-09-10 04:13:35.513124_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_4_4_0.5_fix6_orient_20_rfix_2023-09-12 00:03:59.885116_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_2_3.5_0.5_fix6_orient_20_rfix_2023-09-10 23:36:22.745163_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_rfix_2023-09-11 03:13:48.687762_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_orient_20_rfix_2023-09-11 01:28:55.923747_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_a5_fix6_rfix_2023-09-11 22:11:18.320517_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_a5_fix6_orient_20_rfix_2023-09-11 19:22:56.630124_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_a5_fix6_orient_20_rfix_2023-09-11 17:25:36.505512_rename']

    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_rfix_2023-09-11 03:13:48.687762_rename']

    for f in filenames:
        result_list, fieldnames, expanded_filename = readFile(f)
        repeated_num, scenario_list, seed_list = getRepeatedNum(result_list)
        createFiles(result_list, fieldnames, expanded_filename, repeated_num, scenario_list, seed_list)
        print("Extracted one file.")

