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


#standardize value formatting:
# Bools: 'True' and 'False'
# Numbers: remove all trailing 0's from decimals and remove decimal point if nothing after it
def formatString(value):
    if isinstance(value, bool):
        return str(value)
    else:
        try:
            num = float(value)
        except ValueError:
            return str(value)
        except TypeError:
            return value  # value isn't a string, a number, or a bool, leave it as is
        else:
            return ('%f' % num).rstrip('0').rstrip('.') #Copied from https://stackoverflow.com/a/2440786

def isMatch(entry, key, value):
    if key not in entry:
        return False
    else:
        v = entry[key]
        if isinstance(value, basestring) and v != value:
            return False
        elif v not in value:    #try adding str(v)
            return False
    return True


def convertToStrings(input):
    if input is not None and not isinstance(input, basestring):
        if isinstance(input, list):
            for i in range(len(input)):
                input[i] = convertToStrings(input[i])
        elif isinstance(input, set):
            temp = []
            while len(input) > 0:
                temp.append(convertToStrings(input.pop()))
            for v in temp:
                input.add(v)
        elif isinstance(input, dict):
            for key, value in list(input.items()):
                key = convertToStrings(key)
                input[key] = convertToStrings(value)
        else:
            input = formatString(input)
    return input


def filter(results, whitelist=None, blacklist=None, defaults=None):
    filtered_results = []
    convertToStrings(whitelist)
    convertToStrings(blacklist)
    convertToStrings(defaults)

    for entry in results:
        # First, apply consistent formatting to any numeric values
        for key,value in list(entry.items()):
            entry[key] = formatString(value)

        stillgood = True
        if whitelist is not None:
            for key, value in list(whitelist.items()):
                if not isMatch(entry, key, value):
                    stillgood = False
                    break
        if stillgood and blacklist is not None:
            for key, value in list(blacklist.items()):
                if isMatch(entry, key, value):
                    stillgood = False
                    break
        if stillgood and defaults is not None:
            for key, value in list(defaults.items()):
                if key not in entry or not entry[key]: #if the key wasn't included in the result fields or if the field was left blank for this entry
                    entry[key] = value
        if stillgood:
            filtered_results.append(entry)
    return filtered_results

def replace(results, replacements=None, replacements2=None):
    if replacements is not None:
        for entry in results:
            for key, value in list(replacements.items()):
                if key in entry:
                    if type(value) is dict:
                        if entry[key] in value:
                            entry[key] = value[entry[key]]
                    else:
                        entry[value] = entry.pop(key)
    elif replacements2 is not None:
        for keyname in results:
            if keyname in replacements2:
                results[keyname] = replacements2[keyname]

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



def getPrunedList(results, keys):
    return [{k: entry[k] for k in keys if k in entry} for entry in results]

def get_entries_for_key(cases, key):
    return list(set((case[key] for case in cases)))

def CalculateAgreement(results, result_keyname = "result"):
    seed_keyname = "seed"

    statistics = {}
    result_counts = {}

    for entry in results:
        seed = entry[seed_keyname]
        if seed not in statistics:
            seed_results = {}
            statistics[seed] = seed_results
        else:
            seed_results = statistics[seed]

        result = entry[result_keyname]
        if result not in seed_results:
            seed_results[result] = 1
        else:
            seed_results[result] += 1

        if result not in result_counts:
            result_counts[result] = 1
        else:
            result_counts[result] += 1


    #TODO: Add verification of equal number of combined results for each seed

    seeds_with_x_runs = {}
    for seed in statistics:
        seed_results = statistics[seed]
        seed_num_runs = sum(seed_results.values())

        if seed_num_runs not in seeds_with_x_runs:
            seeds_with_x_runs[seed_num_runs] = [seed]
        else:
            seeds_with_x_runs[seed_num_runs].append(seed)

        '''
        if num_runs is None:
            num_runs = seed_num_runs
            #print("Number of runs=" + str(num_runs))
        else:
            if seed_num_runs != num_runs:
                #print("Error! Seed [" + seed + "] contains (" + str(seed_num_runs) + "), not " + str(num_runs))
                #raise AssertionError("Error! Seed [" + seed + "] contains (" + str(seed_num_runs) + "), not " + str(num_runs))
                return None
        '''

    #num_runs = max((len(seeds_with_x_runs[v]) for v in seeds_with_x_runs.keys()))

    [num_runs, selected_seeds] = max(list(seeds_with_x_runs.items()), key=lambda num_runs_seeds: len(num_runs_seeds[1]))
    if num_runs == 1:
        return None, None

    #selected_seeds = seeds_with_x_runs[num_runs]

    num_seeds = len(selected_seeds)
    #print "Num seeds used: " + str(num_seeds)

    def agr(i):
        return (1.0/(num_runs*(num_runs-1))) * sum(n*(n-1) for n in list(statistics[i].values()))

    observed_agreement = (1.0/num_seeds) * sum(agr(i) for i in selected_seeds)
    #expected_agreement = ((1.0/(num_seeds*num_runs))**2) * sum(n**2 for n in result_counts.values())

    #agreement_coefficient = (observed_agreement - expected_agreement)/(1-expected_agreement)

    note = '(' + str(num_seeds) + '/' + str(len(list(statistics.keys()))) + ')'

    return (observed_agreement, note)

class ResultAnalyzer(object):

    def readFile(self, filename, whitelist = None, blacklist = None, defaults=None, replacements=None):
        result_list = readFile(filename)
        filtered_list = list(filter(result_list, whitelist=whitelist, blacklist=blacklist, defaults=defaults))
        convertToStrings(replacements)
        replace(results=filtered_list, replacements=replacements)
        self.results += filtered_list

    def readFiles(self, filenames, whitelist=None, blacklist = None, defaults=None, replacements=None):
        if isinstance(filenames, str):
          filenames = [filenames]
          
        for filename in filenames:
            self.readFile(filename, whitelist=whitelist, blacklist=blacklist, defaults=defaults, replacements=replacements)

    def clear(self):
        self.__init__()

    def getPrunedList(self, keys):
        return getPrunedList(self.results, keys=keys)

    def replace(self, replacements=None):
        replace(results=self.results, replacements=replacements)

    '''
    def getCases(self, has=None, hasnot=None):
        results = []
        for entry in self.results:
            stillgood = True
            if has is not None:
                for key,value in has.items():
                    if key not in entry or value!=entry[key]:
                        stillgood = False
                        break
            if hasnot is not None:
                for key,value in hasnot.items():
                    if key in entry and value==entry[key]:
                        stillgood = False
                        break

            if stillgood:
                results.append(entry)
        return results
    '''

    def getCases(self, whitelist=None, blacklist=None):
        return list(filter(self.results, whitelist=whitelist, blacklist=blacklist))

    def getCases2(self, whitelist=None, blacklist=None):
        return self.getCases(whitelist=whitelist, blacklist=blacklist)

    def addDefaults(self, defaults):
        self.results = list(filter(self.results, defaults=defaults))

    def getFailCases(self, controller):
        has = {'controller': controller}
        hasnot = {'result': 'SUCCEEDED'}
        results = self.getCases(whitelist=has, blacklist=hasnot)



        #gm = MultiMasterCoordinator()
        #gm.start()

        for result in results:
            print(result)
            #gm.task_queue.put(result)

    def getMaxTime(self):
        max_time = 0

        for entry in self.results:
            if 'time' in entry:
                time = entry['time']
                if time > max_time:
                    max_time = time
        print("Max time: " + str(max_time))


    def computeStatistics(self, independent, dependent):
        statistics = {}
        key_values = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent + dependent}
            conditionset = frozenset(list(condition.items()))
            
            #print conditionset
            
            if not conditionset in statistics:
                statistics[conditionset] = 1
            else:
                statistics[conditionset] = statistics[conditionset] + 1

            for key, value in list(condition.items()):
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)
        for num_barrels in key_values[independent[0]]:
           print(str(num_barrels) + " barrels:")
           for controller in sorted(key_values[independent[1]]):
                total = 0
                for result in key_values[dependent[0]]:
                    key = frozenset(list({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items()))
                    if key in statistics:
                        total+= statistics[key]
                print(controller + " controller:")
                for result in key_values[dependent[0]]:
                    key = frozenset(
                        list({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items()))
                    if key in sorted(statistics):
                        num = statistics[frozenset(list({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items()))]
                        print(result + ": " + str(num) + "\t" + str(float(num)/total))
                print("")

    def generateTable(self):
        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        for entry in self.results:
            condition = {key: entry[key] for key in ["controller", "scenario"] + ["result"]}
            conditionset = frozenset(list(condition.items()))

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = [int(entry["time"])]
                path_lengths[conditionset] = [float(entry["path_length"])]
            else:
                statistics[conditionset] = statistics[conditionset] + 1
                path_times[conditionset].append(int(entry["time"]))
                path_lengths[conditionset].append(float(entry["path_length"]))

            for key, value in list(condition.items()):
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)
        for scenario in key_values["scenario"]:
            print("")
            print("Scenario: " + str(scenario))

            print("")

            print(("| controller"), end=' ')
            for result in key_values["result"]:
                print((" | " + str(result)), end=' ')


            print("|")


            for i in range(len(key_values["result"])+1):
                print(("| -------"), end=' ')

            print("|")

            for controller in sorted(key_values["controller"]):
                total = 0
                for result in key_values["result"]:
                    key = frozenset(
                        list({"controller": controller, "scenario": scenario, "result": result}.items()))
                    if key in statistics:
                        total += statistics[key]

                print(("| " + str(controller)), end=' ')
                for result in key_values["result"]:
                    key = frozenset(
                        list({"controller": controller, "scenario": scenario, "result": result}.items()))
                    if key in sorted(statistics):
                        lookupkey = frozenset(list({"controller": controller, "scenario": scenario, "result": result}.items()))
                        num = statistics[lookupkey]
                        path_time = old_div(np.mean(np.array(path_times[lookupkey])),1e9)
                        path_length = np.mean(np.array(path_lengths[lookupkey]))

                        print(("| " + "{0:.1f}".format(100*float(num) / total) + "% (" + str(num) + ") " + "<br>" + "{0:.2f}".format(path_length) + "m" + " <br>" + "{0:.2f}".format(path_time) + "s"), end=' ')
                    else:
                        print(("| "), end=' ')
                print("|")

    def generateGenericTable(self, independent, dependent, whitelist=None, blacklist=None, replacements=None, order=None, check_agreement=False):
        #if type(x) is not str and isinstance(x, collections.Sequence)
        #remapped_keynames = {"SUCCEEDED":"SUCCEEDED", "path time":"path time", "seed":"seed", "path_length":"path_length", "common length":"common length", "common time":"common time"}
        #replace(results=remapped_keynames, replacements2=replacements)

        order = convertToStrings(order)

        def remap(term):
            return replacements[term] if term in replacements else term

        seed_keyname="seed"
        success_keyname="SUCCEEDED"
        path_length_keyname=remap("path length")
        path_time_keyname=remap("path time")
        common_length_keyname=remap("common length")
        common_time_keyname=remap("common time")
        agreement_keyname=remap("agreement")

        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        condition_lists = {}

        results = self.getCases(whitelist=whitelist, blacklist=blacklist)

        if len(results) == 0:
            print("Error! No results to analyze. Please check your filtering conditions")
            return

        for entry in results:
            condition = {key: entry[key] for key in independent + [dependent]}
            conditionset = frozenset(list(condition.items()))

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = {}
                path_lengths[conditionset] = {}
            else:
                statistics[conditionset] = statistics[conditionset] + 1

            if entry[seed_keyname] not in path_times[conditionset]:
                path_times[conditionset][entry[seed_keyname]] = []

            if entry[seed_keyname] not in path_lengths[conditionset]:
                path_lengths[conditionset][entry[seed_keyname]] = []

            path_times[conditionset][entry[seed_keyname]] += [int(entry["time"])]
            path_lengths[conditionset][entry[seed_keyname]] += [float(entry["path_length"])]

            for key, value in list(condition.items()):
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)

            independent_condition = {key: entry[key] for key in independent}
            independent_conditionset = frozenset(list(independent_condition.items()))
            if independent_conditionset not in condition_lists:
                condition_lists[independent_conditionset] = []
            condition_lists[independent_conditionset].append(entry)


        max_depth = len(independent)

        def processLayer(shared_conditions_dict={}, depth=0, shared_safe_keys=None):
            def sort(key_values, condition_name):
                if order is not None and condition_name in order:
                    if set(key_values[condition_name]) == set(order[condition_name]):
                        return order[condition_name]
                    else:
                        print("Error! order requested but does not contain all necessary keys")

                try:
                    #If values are all numbers, then make sure to sort numerically
                    sorted_numerically = [x for (_, x) in sorted([(float(x), x) for x in key_values[condition_name]], key=lambda pair: pair[0])]
                    return sorted_numerically
                except ValueError:
                    #Otherwise, sort alphabetically
                    return sorted(key_values[condition_name])

            if depth == max_depth:
                
                lookup_keys = []
                total=0
                for dependent_value in key_values[dependent]: #TODO: use all permutations of multiple dependents
                    key = frozenset(list(shared_conditions_dict.items()) + list({dependent: dependent_value}.items()))
                    if key in statistics:
                        total += statistics[key]

                #print("| " + str(controller)),
                for dependent_value in sort(key_values, dependent):
                    lookupkey = frozenset(list(shared_conditions_dict.items()) + list({dependent: dependent_value}.items()))
                    if lookupkey in statistics:
                        num = statistics[lookupkey]
                        #path_time = np.mean(np.array(path_times[lookupkey])) / 1e9
                        #path_length = np.mean(np.array(path_lengths[lookupkey]))

                        #print("| " + "{0:.1f}".format(100 * float(num) / total) + "% (" + str(
                        #    num) + ") " + "<br>" + "{0:.2f}".format(path_length) + "m" + " <br>" + "{0:.2f}".format(
                        #    path_time) + "s"),

                        print(("| " + "{0:.1f}".format(100 * float(num) / total) + "% (" + str(
                            num) + ") "), end=' ')
                    else:
                        print(("| "), end=' ')

                if check_agreement:
                    independent_key = frozenset(list(shared_conditions_dict.items()))
                    r = condition_lists[independent_key]
                    agr, note = CalculateAgreement(results=r, result_keyname='result')
                    agreement_details = False
                    print(("| " + (("{0:.3f}".format(agr) + (note if agreement_details else '')) if agr is not None else "N/A")), end=' ')
                    #print("| " + ("{0:.3f}".format(agr) if agr is not None else "N/A")),

                dependent_value= success_keyname
                lookupkey = frozenset(list(shared_conditions_dict.items()) + list({dependent: dependent_value}.items()))

                if lookupkey in statistics:
                
                    def reduce_list(lists):
                        return np.array([sum(nested_list)/float(len(nested_list)) for nested_list in lists])

                    # Print average path length and time over all successful cases
                    times = path_times[lookupkey]
                    times = reduce_list(list(times.values()))
                    path_time = old_div(np.mean(times), 1e9)
                    path_time_std = old_div(np.std(times), 1e9)

                    path_length = path_lengths[lookupkey]
                    path_lengthx = reduce_list(list(path_length.values()))
                    path_length = np.mean(path_lengthx)
                    path_length_std = np.std(path_lengthx)

                    print(("| " + "{0:.1f}".format(path_length) + "(" + str(path_time_std) + ") |" + "{0:.1f}".format(
                        path_time) + "(" + str(path_length_std) + ")"), end=' ')

                    # Now print the averages of shared successful cases
                    if shared_safe_keys is not None and len(shared_safe_keys) > 0:
                        times = [path_times[lookupkey][k] for k in shared_safe_keys]
                        times = reduce_list(times)
                        path_time = old_div(np.mean(times), 1e9)

                        path_length = [path_lengths[lookupkey][k] for k in shared_safe_keys]
                        path_length = reduce_list(path_length)
                        path_length = np.mean(path_length)

                        print(("| " + "{0:.1f}".format(path_length) + " |" + "{0:.1f}".format(
                            path_time) + ""), end=' ')
                    else:
                        print(("|"), end=' ')

                else:
                    print(("| "), end=' ')



                print("|")


            else:

                condition_name = independent[depth]
                safe_keys = None

                remapped_condition_name = remap(condition_name)

                if depth == max_depth-1:

                    dependent_value = success_keyname
                    if condition_name in key_values:
                        for condition_value in key_values[condition_name]:
                            lookupkey = frozenset(list(shared_conditions_dict.items()) + list({dependent: dependent_value, condition_name: condition_value}.items()))

                            if lookupkey in path_times:
                                condition_safe_keys = path_times[lookupkey]
                                condition_safe_keys = list(condition_safe_keys.keys())
                                if safe_keys is None:
                                    safe_keys = set(condition_safe_keys)
                                else:
                                    safe_keys = safe_keys.intersection(condition_safe_keys)
                            else:
                                pass

                    print("")
                    print(("| " + remapped_condition_name), end=' ')
                    for result in sort(key_values, dependent):
                        print((" | " + remap(str(result))), end=' ')

                    if check_agreement:
                        print(("| " + agreement_keyname), end=' ')
                        
                    print((" | " + path_length_keyname + " | " + path_time_keyname + " | " + common_length_keyname + " | " + common_time_keyname), end=' ')

                    print("|")

                    for i in range(len(key_values[dependent]) + (6 if check_agreement else 5)):
                        print(("| -------"), end=' ')

                    print("|")

                else:
                    print("")
                    print((remapped_condition_name + ":"), end=' ')

                singlevalue = len(key_values[condition_name]) == 1
                for condition_value in sort(key_values, condition_name):
                    cond_dict = copy.deepcopy(shared_conditions_dict)
                    cond_dict[condition_name]=condition_value

                    remapped_condition_value = remap(condition_value)
                    res_list = self.getCases(whitelist=cond_dict)
                    if len(res_list) > 0:

                        if depth == max_depth-1:
                          print(("| " + str(remapped_condition_value)), end=' ')
                        else:
                            if singlevalue:
                                print(" " + str(remapped_condition_value))
                            else:
                                print("\n")
                                print(remapped_condition_value + ":")

                        processLayer(cond_dict, depth+1, safe_keys)


        processLayer()
        print("")

    def CalculateAgreement(self, whitelist=None, blacklist=None):
        results = self.getCases(whitelist=whitelist, blacklist=blacklist)
        agreement_coefficient = CalculateAgreement(results=results, result_keyname="result")
        if agreement_coefficient is None:
            print('Error, unable to calculate agreement of results!')
        else:
            print("Full Agreement Coefficient = " + str(agreement_coefficient))
            keyname = 'pass_or_fail'
            for entry in results:
                entry[keyname] = 'pass' if entry['result']=='SUCCEEDED' else 'fail'
            agreement_coefficient = CalculateAgreement(results=results, result_keyname=keyname)
            print("Pass/Fail Agreement Coefficient = " + str(agreement_coefficient))


    def generateSingleTable(self):
        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        for entry in self.results:
            condition = {key: entry[key] for key in ["controller", "scenario"] + ["result"]}
            conditionset = frozenset(list(condition.items()))

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = [int(entry["time"])]
                path_lengths[conditionset] = [float(entry["path_length"])]
            else:
                statistics[conditionset] = statistics[conditionset] + 1
                path_times[conditionset].append(int(entry["time"]))
                path_lengths[conditionset].append(float(entry["path_length"]))

            for key, value in list(condition.items()):
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)

        print("")

        print(("| "), end=' ')
        for scenario in sorted(key_values["scenario"]):
            if scenario == "corridor_zigzag":
                print(("| corridor <br> zigzag"), end=' ')
            elif scenario == "corridor_zigzag_door":
                print(("| corridor <br> zigzag <br> door"), end=' ')
            else:
                print(("| " + str(scenario)), end=' ')
        print("|")

        for i in range(len(key_values["scenario"])+1):
            print(("| -------"), end=' ')
        print("|")

        for controller in sorted(key_values["controller"]):
            print(("| " + str(controller)), end=' ')
            for scenario in sorted(key_values["scenario"]):

                total = 0
                for result in key_values["result"]:
                    key = frozenset(
                        list({"controller": controller, "scenario": scenario, "result": result}.items()))
                    if key in statistics:
                        total += statistics[key]

                result = "SUCCEEDED"

                key = frozenset(
                    list({"controller": controller, "scenario": scenario, "result": result}.items()))
                if key in sorted(statistics):
                    lookupkey = frozenset(list({"controller": controller, "scenario": scenario, "result": result}.items()))
                    num = statistics[lookupkey]
                    path_time = old_div(np.mean(np.array(path_times[lookupkey])), 1e9)
                    path_length = np.mean(np.array(path_lengths[lookupkey]))

                    print(("| " + "{0:.1f}".format(100 * float(num) / total) + "% <br>" + "{0:.2f}".format(path_length) + "m"), end=' ')
                else:
                    print(("| 0.0%"), end=' ')
            print("|")




    def exists(self, scenario):
        pass

    def freezeSet(self, independent):
        self.frozen_set = []
        key_values = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent}
            conditionset = frozenset(list(condition.items()))
            self.frozen_set.append(conditionset)

    def getAverageTime(self, tasks):
        total_time = 0
        num_tasks = 0
        for task in tasks:
            t = int(task['time'])
            total_time += t
            num_tasks += 1
            
        avg_time = old_div(total_time,num_tasks) if num_tasks > 0 else 0
        print(': ' + str(old_div(avg_time,1e9))) #tasks[0]['controller'] +

    def contains(self, task):
        stripped_task = {str(key): str(task[key]) for key,value in list(task.items())}
        stripped_task = frozenset(list(stripped_task.items()))

        for entry in self.results:
            condition = {key: entry[key] for key,value in list(task.items()) if key in entry}
            conditionset = frozenset(list(condition.items()))
            if conditionset == stripped_task:
                if 'result' in entry: #and (entry['result'] == 'SUCCEEDED' or entry['result'] == 'BUMPER_COLLISION'):
                    return True

        return False

    def getMatchingResult(self, task):
        stripped_task = {str(key): str(task[key]) for key,value in list(task.items())}
        stripped_task = frozenset(list(stripped_task.items()))

        for entry in self.results:
            condition = {key: entry[key] for key,value in list(task.items())}
            conditionset = frozenset(list(condition.items()))
            if conditionset == stripped_task:
                return entry

        return None

    def __init__(self):
        self.fieldnames = []
        self.results = []

    def getCommonSuccessfulSeeds(self, controllers):
        statistics = {}
        good_seeds = []
        
        for seed in range(0,50):
            still_good = True
            
            for controller in controllers:
                task = {'scenario': 'sector', 'controller': controller, 'seed': seed, 'result': 'SUCCEEDED'}
                task = self.getMatchingResult(task)
                if task is None:
                    still_good = False
                    break
            
            if still_good:
                good_seeds.append(seed)
                print(str(seed) + ": good")
                
        return good_seeds


    def compareControllers(self, controller1, controller2):
        statistics = {}

        for seed in range(52,97):
            task1 = {'scenario': 'sector', 'controller': controller1, 'seed': seed}
            task2 = {'scenario': 'sector', 'controller': controller2, 'seed': seed}

            task1 = self.getMatchingResult(task1)
            if task1 is not None:
                task2 = self.getMatchingResult(task2)
                if task2 is not None:
                    condition = {task1['result']:task2['result']}
                    conditionset = frozenset(list(condition.items()))

                    # print conditionset

                    if not conditionset in statistics:
                        statistics[conditionset] = 1
                    else:
                        statistics[conditionset] = statistics[conditionset] + 1


        print(controller1 + " : " + controller2)
        for key,value in list(statistics.items()):
            print(str(next(iter(key))) + " : " + str(value))

