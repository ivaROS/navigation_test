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
            
        return result_list

def getFailedList(result_list):
    failed_lists = {}

    for result in result_list:
        if 'world' in result.keys() and len(result['world']) != 0:
            scenario = result['world']
        elif 'scenario' in result.keys() and len(result['scenario']) != 0:
            scenario = result['scenario']

        result_case = result['result']
        seed = result['seed']

        if result_case != 'SUCCEEDED':
            if scenario in failed_lists.keys():
                if result_case in failed_lists[scenario].keys():
                    failed_lists[scenario][result_case].append(int(seed))
                else:
                    failed_lists[scenario][result_case] = [int(seed)]
            else:
                failed_lists[scenario] = {}
                failed_lists[scenario][result_case] = [int(seed)]

    for sk, sv in failed_lists.items():
        total_failed_num = 0
        for k, v in sv.items():
            total_failed_num += len(v)

        failed_lists[sk]['total'] = total_failed_num

    return failed_lists


if __name__ == "__main__":
    filename = '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_nostop_2023-08-15 17:47:55.757627'
    result_list = readFile(filename)
    failed_list = getFailedList(result_list)
    print(failed_list)