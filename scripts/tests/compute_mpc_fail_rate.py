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

def getFailedRate(result_list):
    avg_mpc_fail_rate = 0
    avg_keyhole_fail_rate = 0
    mpc_fail_rate_lists = []
    keyhole_fail_rate_lists = []

    total_loops = 0
    for result in result_list:
        if 'stat_bag_file_path' in result.keys():
            bag_file = rosbag.Bag(f=result['stat_bag_file_path'])
        else:
            continue

        for topic, msg, t in bag_file.read_messages(topics=['stats']):
            total_num = msg.total_loop_num
            mpc_fail_num = msg.mpc_fail_num
            mpc_fail_rate = float(mpc_fail_num) / total_num
            keyhole_fail_num = msg.keyhole_fail_num
            keyhole_fail_rate = float(keyhole_fail_num) / total_num
            total_loops = total_loops + total_num

        mpc_fail_rate_lists.append(mpc_fail_rate)
        keyhole_fail_rate_lists.append(keyhole_fail_rate)
        bag_file.close()

    avg_mpc_fail_rate = sum(mpc_fail_rate_lists) / len(mpc_fail_rate_lists)
    avg_keyhole_fail_rate = sum(keyhole_fail_rate_lists) / len(keyhole_fail_rate_lists)

    return avg_mpc_fail_rate, avg_keyhole_fail_rate, mpc_fail_rate_lists, keyhole_fail_rate_lists, total_loops


if __name__ == "__main__":
    filepath = '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/mpc_stat/'
    # filename = filepath + 'results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2024-01-10 19:37:40.556109'
    # filename = filepath + 'results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_fix8_1.1_orient_20_rangefix_2024-07-30 06:20:18.987409'
    # filename = filepath + 'results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_fix8_1.1_orient_20_rangefix_2024-07-30 16:02:19.059796'
    # filename = filepath + 'results_1_sg_fast_0.4_p_ni_nonstop_mr_fix8_1.1_orient_20_rangefix_2025-02-19 14:20:49.642854' # baseline
    filename = filepath + 'results_1_sg_fast_0.4_p_ni_nonstop_mr_fix8_1.1_orient_20_rangefix_2025-03-02 15:16:28.188516'
    
    result_list = readFile(filename)
    avg_mpc_fail_rate, avg_keyhole_fail_rate, mpc_fail_rate_lists, keyhole_fail_rate_lists, total_loops = getFailedRate(result_list)
    print("Average MPC failure rates: " + str(round(avg_mpc_fail_rate * 100, 2)) + "% with [" + str(len(mpc_fail_rate_lists)) + "] tests and [" + str(total_loops) + "] loops.")
    print("Average Keyhole failure rates: " + str(round(avg_keyhole_fail_rate * 100, 2)) + "% with [" + str(len(keyhole_fail_rate_lists)) + "] tests and [" + str(total_loops) + "] loops.")