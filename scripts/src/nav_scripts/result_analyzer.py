import csv
import time
import math
import numpy as np
import copy

def filter(results, whitelist=None, blacklist=None):
    filtered_results = []
    for entry in results:
        stillgood = True
        if whitelist is not None:
            for key, value in whitelist.items():
                if key not in entry or entry[key] not in value or value not in entry[key]:
                    stillgood = False
                    break
        if blacklist is not None:
            for key, value in blacklist.items():
                if key in entry and entry[key] in value:
                    stillgood = False
                    break

        if stillgood:
            filtered_results.append(entry)
    return filtered_results

class ResultAnalyzer:

    def readFile(self, filename, whitelist = None, blacklist = None):
        with open(filename, 'rb') as csvfile:
            datareader = csv.DictReader(csvfile, restval='')

            result_list = []
            fieldnames = datareader.fieldnames
            for entry in datareader:
                result_list.append(entry)
        filtered_list = filter(result_list, whitelist=whitelist, blacklist=blacklist)
        self.results += filtered_list

    def readFiles(self, filenames, whitelist=None, blacklist = None):
        for filename in filenames:
            self.readFile(filename, whitelist=whitelist, blacklist=blacklist)

    def clear(self):
        self.__init__()

    def getPrunedList(self, keys):
        results = []
        for entry in self.results:
            {k: entry[k] for k in keys}

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

    def getFailCases(self, controller):
        has = {'controller': controller}
        hasnot = {'result': 'SUCCEEDED'}
        results = self.getCases(has=has, hasnot=hasnot)



        #gm = MultiMasterCoordinator()
        #gm.start()

        for result in results:
            print result
            #gm.task_queue.put(result)

    def getMaxTime(self):
        max_time = 0

        for entry in self.results:
            if 'time' in entry:
                time = entry['time']
                if time > max_time:
                    max_time = time
        print "Max time: " + str(max_time)


    def computeStatistics(self, independent, dependent):
        statistics = {}
        key_values = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent + dependent}
            conditionset = frozenset(condition.items())
            
            #print conditionset
            
            if not conditionset in statistics:
                statistics[conditionset] = 1
            else:
                statistics[conditionset] = statistics[conditionset] + 1

            for key, value in condition.items():
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)
        for num_barrels in key_values[independent[0]]:
           print str(num_barrels) + " barrels:"
           for controller in sorted(key_values[independent[1]]):
                total = 0
                for result in key_values[dependent[0]]:
                    key = frozenset({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())
                    if key in statistics:
                        total+= statistics[key]
                print controller + " controller:"
                for result in key_values[dependent[0]]:
                    key = frozenset(
                        {independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())
                    if key in sorted(statistics):
                        num = statistics[frozenset({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())]
                        print result + ": " + str(num) + "\t" + str(float(num)/total)
                print ""

    def generateTable(self):
        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        for entry in self.results:
            condition = {key: entry[key] for key in ["controller", "scenario"] + ["result"]}
            conditionset = frozenset(condition.items())

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = [int(entry["time"])]
                path_lengths[conditionset] = [float(entry["path_length"])]
            else:
                statistics[conditionset] = statistics[conditionset] + 1
                path_times[conditionset].append(int(entry["time"]))
                path_lengths[conditionset].append(float(entry["path_length"]))

            for key, value in condition.items():
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)
        for scenario in key_values["scenario"]:
            print ""
            print "Scenario: " + str(scenario)

            print ""

            print("| controller"),
            for result in key_values["result"]:
                print(" | " + str(result)),


            print("|")


            for i in range(len(key_values["result"])+1):
                print("| -------"),

            print("|")

            for controller in sorted(key_values["controller"]):
                total = 0
                for result in key_values["result"]:
                    key = frozenset(
                        {"controller": controller, "scenario": scenario, "result": result}.items())
                    if key in statistics:
                        total += statistics[key]

                print("| " + str(controller)),
                for result in key_values["result"]:
                    key = frozenset(
                        {"controller": controller, "scenario": scenario, "result": result}.items())
                    if key in sorted(statistics):
                        lookupkey = frozenset({"controller": controller, "scenario": scenario, "result": result}.items())
                        num = statistics[lookupkey]
                        path_time = np.mean(np.array(path_times[lookupkey]))/1e9
                        path_length = np.mean(np.array(path_lengths[lookupkey]))

                        print("| " + "{0:.1f}".format(100*float(num) / total) + "% (" + str(num) + ") " + "<br>" + "{0:.2f}".format(path_length) + "m" + " <br>" + "{0:.2f}".format(path_time) + "s"),
                    else:
                        print("| "),
                print("|")

    def generateGenericTable(self, independent, dependent):
        #if type(x) is not str and isinstance(x, collections.Sequence)

        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent + [dependent]}
            conditionset = frozenset(condition.items())

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = [int(entry["time"])]
                path_lengths[conditionset] = [float(entry["path_length"])]
            else:
                statistics[conditionset] = statistics[conditionset] + 1
                path_times[conditionset].append(int(entry["time"]))
                path_lengths[conditionset].append(float(entry["path_length"]))

            for key, value in condition.items():
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)

        max_depth = len(independent)

        def processLayer(shared_conditions_dict={}, depth=0):
            if depth == max_depth:

                lookup_keys = []
                total=0
                for dependent_value in key_values[dependent]: #TODO: use all permutations of multiple dependents
                    key = frozenset(shared_conditions_dict.items() + {dependent: dependent_value}.items())
                    if key in statistics:
                        total += statistics[key]

                #print("| " + str(controller)),
                for dependent_value in key_values[dependent]:
                    lookupkey = frozenset(shared_conditions_dict.items() + {dependent: dependent_value}.items())
                    if lookupkey in statistics:
                        num = statistics[lookupkey]
                        #path_time = np.mean(np.array(path_times[lookupkey])) / 1e9
                        #path_length = np.mean(np.array(path_lengths[lookupkey]))

                        #print("| " + "{0:.1f}".format(100 * float(num) / total) + "% (" + str(
                        #    num) + ") " + "<br>" + "{0:.2f}".format(path_length) + "m" + " <br>" + "{0:.2f}".format(
                        #    path_time) + "s"),

                        print("| " + "{0:.1f}".format(100 * float(num) / total) + "% (" + str(
                            num) + ") "),
                    else:
                        print("| "),

                dependent_value="SUCCEEDED"
                lookupkey = frozenset(shared_conditions_dict.items() + {dependent: dependent_value}.items())
                if lookupkey in statistics:
                    path_time = np.mean(np.array(path_times[lookupkey])) / 1e9
                    path_length = np.mean(np.array(path_lengths[lookupkey]))

                    print("| " + "{0:.2f}".format(path_length) + "m |" + "{0:.2f}".format(
                        path_time) + "s"),

                else:
                    print("| "),
                print("|")


            else:
                condition_name = independent[depth]

                if depth == max_depth-1:

                    print("")
                    print("| " + condition_name),
                    for result in key_values[dependent]:
                        print(" | " + str(result)),
                        
                    print(" | " + "path length | path time"),

                    print("|")

                    for i in range(len(key_values[dependent]) + 3):
                        print("| -------"),

                    print("|")

                else:
                    print("")
                    print(condition_name + ":")

                for condition_value in sorted(key_values[condition_name]):
                    if depth == max_depth-1:
                      print("| " + str(condition_value)),
                    else:
                        print("")
                        print(condition_value + ":")

                    cond_dict = copy.deepcopy(shared_conditions_dict)
                    cond_dict[condition_name]=condition_value

                    processLayer(cond_dict, depth+1)


        processLayer()

    def generateSingleTable(self):
        statistics = {}
        key_values = {}
        path_times = {}
        path_lengths = {}
        for entry in self.results:
            condition = {key: entry[key] for key in ["controller", "scenario"] + ["result"]}
            conditionset = frozenset(condition.items())

            # print conditionset

            if not conditionset in statistics:
                statistics[conditionset] = 1
                path_times[conditionset] = [int(entry["time"])]
                path_lengths[conditionset] = [float(entry["path_length"])]
            else:
                statistics[conditionset] = statistics[conditionset] + 1
                path_times[conditionset].append(int(entry["time"]))
                path_lengths[conditionset].append(float(entry["path_length"]))

            for key, value in condition.items():
                if not key in key_values:
                    key_values[key] = set()
                key_values[key].add(value)

        print ""

        print("| "),
        for scenario in sorted(key_values["scenario"]):
            if scenario == "corridor_zigzag":
                print("| corridor <br> zigzag"),
            elif scenario == "corridor_zigzag_door":
                print("| corridor <br> zigzag <br> door"),
            else:
                print("| " + str(scenario)),
        print "|"

        for i in range(len(key_values["scenario"])+1):
            print("| -------"),
        print "|"

        for controller in sorted(key_values["controller"]):
            print("| " + str(controller)),
            for scenario in sorted(key_values["scenario"]):

                total = 0
                for result in key_values["result"]:
                    key = frozenset(
                        {"controller": controller, "scenario": scenario, "result": result}.items())
                    if key in statistics:
                        total += statistics[key]

                result = "SUCCEEDED"

                key = frozenset(
                    {"controller": controller, "scenario": scenario, "result": result}.items())
                if key in sorted(statistics):
                    lookupkey = frozenset({"controller": controller, "scenario": scenario, "result": result}.items())
                    num = statistics[lookupkey]
                    path_time = np.mean(np.array(path_times[lookupkey])) / 1e9
                    path_length = np.mean(np.array(path_lengths[lookupkey]))

                    print("| " + "{0:.1f}".format(100 * float(num) / total) + "% <br>" + "{0:.2f}".format(path_length) + "m"),
                else:
                    print("| 0.0%"),
            print("|")




    def exists(self, scenario):
        pass

    def freezeSet(self, independent):
        self.frozen_set = []
        key_values = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent}
            conditionset = frozenset(condition.items())
            self.frozen_set.append(conditionset)

    def getAverageTime(self, tasks):
        total_time = 0
        num_tasks = 0
        for task in tasks:
            t = int(task['time'])
            total_time += t
            num_tasks += 1
            
        avg_time = total_time/num_tasks if num_tasks > 0 else 0
        print  ': ' + str(avg_time/1e9) #tasks[0]['controller'] +

    def contains(self, task):
        stripped_task = {str(key): str(task[key]) for key,value in task.items()}
        stripped_task = frozenset(stripped_task.items())

        for entry in self.results:
            condition = {key: entry[key] for key,value in task.items() if key in entry}
            conditionset = frozenset(condition.items())
            if conditionset == stripped_task:
                if 'result' in entry: #and (entry['result'] == 'SUCCEEDED' or entry['result'] == 'BUMPER_COLLISION'):
                    return True

        return False

    def getMatchingResult(self, task):
        stripped_task = {str(key): str(task[key]) for key,value in task.items()}
        stripped_task = frozenset(stripped_task.items())

        for entry in self.results:
            condition = {key: entry[key] for key,value in task.items()}
            conditionset = frozenset(condition.items())
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
                print str(seed) + ": good"
                
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
                    conditionset = frozenset(condition.items())

                    # print conditionset

                    if not conditionset in statistics:
                        statistics[conditionset] = 1
                    else:
                        statistics[conditionset] = statistics[conditionset] + 1


        print controller1 + " : " + controller2
        for key,value in statistics.items():
            print str(next(iter(key))) + " : " + str(value)

