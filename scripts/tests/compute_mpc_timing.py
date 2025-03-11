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

import matplotlib.pyplot as plt

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

def getTimingStats(result_list):
    sgap_timing_lists = []
    traj_timing_lists = []
    keyhole_timing_lists = []
    mpc_timing_lists = []
    po_timing_lists = []
    total_timing_lists = []

    timing_dict = {}

    for result in result_list:
        if 'timing_bag_file_path' in result.keys():
            bag_file = rosbag.Bag(f=result['timing_bag_file_path'])
        else:
            continue

        if 'result' in result.keys():
            res = result['result']
            if res != "SUCCEEDED" and res != "ABORTED":
            # if res != "SUCCEEDED":
                continue
        else:
            continue

        for topic, msg, t in bag_file.read_messages(topics=['timing_stats']):
            sgap_timing_lists.append(msg.sg_time)
            traj_timing_lists.append(msg.traj_time)
            keyhole_timing_lists.append(msg.keyhole_time)
            mpc_timing_lists.append(msg.mpc_time)
            po_timing_lists.append(msg.po_time)
            total_timing_lists.append(msg.total_time)

    timing_dict['sg_time'] = sgap_timing_lists
    timing_dict['sg_mean'] = np.average(sgap_timing_lists)
    timing_dict['sg_std'] = np.std(sgap_timing_lists)
    timing_dict['traj_time'] = traj_timing_lists
    timing_dict['traj_mean'] = np.average(traj_timing_lists)
    timing_dict['traj_std'] = np.std(traj_timing_lists)
    timing_dict['keyhole_time'] = keyhole_timing_lists
    timing_dict['keyhole_mean'] = np.average(keyhole_timing_lists)
    timing_dict['keyhole_std'] = np.std(keyhole_timing_lists)
    timing_dict['mpc_time'] = mpc_timing_lists
    timing_dict['mpc_mean'] = np.average(mpc_timing_lists)
    timing_dict['mpc_std'] = np.std(mpc_timing_lists)
    timing_dict['po_time'] = po_timing_lists
    timing_dict['po_mean'] = np.average(po_timing_lists)
    timing_dict['po_std'] = np.std(po_timing_lists)
    timing_dict['total_time'] = total_timing_lists
    timing_dict['total_mean'] = np.average(total_timing_lists)
    timing_dict['total_std'] = np.std(total_timing_lists)

    print("Sg mean: " + str(timing_dict['sg_mean']) + "; std: " + str(timing_dict['sg_std']))
    print("Traj mean: " + str(timing_dict['traj_mean']) + "; std: " + str(timing_dict['traj_std']))
    print("Keyhole mean: " + str(timing_dict['keyhole_mean']) + "; std: " + str(timing_dict['keyhole_std']))
    print("MPC mean: " + str(timing_dict['mpc_mean']) + "; std: " + str(timing_dict['mpc_std']))
    print("PO mean: " + str(timing_dict['po_mean']) + "; std: " + str(timing_dict['po_std']))
    print("Total mean: " + str(timing_dict['total_mean']) + "; std: " + str(timing_dict['total_std']))

    print(sum(np.array(timing_dict['total_time']) > 200))

    return timing_dict

def drawTimingHist(data_list, title):
    plt.figure()
    plt.hist(data_list)
    plt.title(title)

def drawTimingFigs(data_dict):
    for key, value in data_dict.items():
        if 'time' in key:
            drawTimingHist(value, key)

    plt.show()


if __name__ == "__main__":
    filename = '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/timing_stat/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2024-01-19 19:06:37.932771'
    # filename = '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/timing_stat/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_acados_2024-01-25 02:16:05.411616'
    result_list = readFile(filename)
    timing_dict = getTimingStats(result_list)
    drawTimingFigs(timing_dict)