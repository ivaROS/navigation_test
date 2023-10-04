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

def getMPCFailedRate(result_list):
    avg_fail_rate = 0
    fail_rate_lists = []

    total_loops = 0
    for result in result_list:
        if 'bag_file_path' in result.keys():
            bag_file = rosbag.Bag(f=result['bag_file_path'])
        else:
            continue

        for topic, msg, t in bag_file.read_messages(topics=['stats']):
            total_num = msg.total_loop_num
            fail_num = msg.mpc_fail_num
            fail_rate = float(fail_num) / total_num
            total_loops = total_loops + total_num

        fail_rate_lists.append(fail_rate)
        bag_file.close()

    avg_fail_rate = sum(fail_rate_lists) / len(fail_rate_lists)

    return avg_fail_rate, fail_rate_lists, total_loops


if __name__ == "__main__":
    filename = '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_mpc_k_r_2023-08-09 23:33:08.399327'
    result_list = readFile(filename)
    avg_fail_rate, fail_rate_lists, total_loops = getMPCFailedRate(result_list)
    print("Average MPC failure rates: " + str(round(avg_fail_rate * 100, 2)) + "% with [" + str(len(fail_rate_lists)) + "] tests and [" + str(total_loops) + "] loops.")