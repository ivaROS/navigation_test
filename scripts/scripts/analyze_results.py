import csv
import time
from gazebo_master import MultiMasterCoordinator
import math
import numpy as np

def filter(results, whitelist=None, blacklist=None):
    filtered_results = []
    for entry in results:
        stillgood = True
        if whitelist is not None:
            for key, value in whitelist.items():
                if key not in entry or entry[key] not in value:
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
            print "Scenario: " + str(scenario)

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

                        print("| " + str(float(num) / total) + "(" + str(num) + ") " + str(path_time) + " " + str(path_length)),
                    else:
                        print("| "),
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
            condition = {key: entry[key] for key,value in task.items()}
            conditionset = frozenset(condition.items())
            if conditionset == stripped_task:
                if 'result' in entry and (entry['result'] == 'SUCCEEDED' or entry['result'] == 'BUMPER_COLLISION'):
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



if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()

    filenames = ['/home/justin/Documents/dl_gazebo_results_2018-02-20 14:17:20.349670',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-19 20:17:06.041463',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 15:18:36.378260',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 17:39:02.442583',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 19:55:37.855977'] #Initial set of runs with different controllers

    #filenames = ['/home/justin/Documents/dl2_gazebo_results_2018-02-21 13:40:16.659915']   #repeated baselines, trying to ensure that recovery behaviors disabled

    #filenames= ['/home/justin/Documents/dl2_gazebo_results_2018-02-26 22:00:58.731302', '/home/justin/Documents/dl2_gazebo_results_2018-02-27 21:44:43.554072' ] #Reran brute_force, turned out bug in decimation propagation
    filenames = ['/home/justin/Documents/dl2_gazebo_results_2018-03-02 20:12:04.419906' ] #reran brute_force after fixing some bugs, still doesn't look good


    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-03-10 14:01:47.717608', #dwa, teb, pips_dwa, pips_ni.
                 '/home/justin/Documents/dl3_gazebo_results_2018-03-10 16:41:40.875418', #rl_single and propagated pips_dwa
                 '/home/justin/Documents/dl3_gazebo_results_2018-03-10 18:35:11.674445', #rl_goal (first half)
                 '/home/justin/Documents/dl3_gazebo_results_2018-03-10 18:54:08.393278'  #rl_goal (2nd half)
                 ]

    #analyzer.readFiles(filenames=filenames, whitelist={'controller':['dwa','teb']})

    #filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-03-11 01:02:06.128521', '/home/justin/Documents/dl3_gazebo_results_2018-03-12 23:31:55.077168' ]

    #analyzer.readFiles(filenames=filenames)

    filenames= ['/home/justin/Documents/dl3_gazebo_results_2018-03-13 00:14:11.273737',  #missing ones from above files; all of 52:97
                '/home/justin/Documents/dl3_gazebo_results_2018-07-30 16:28:24.900163'    #egocylindrical 52:97
                ]

    filenames= ['/home/justin/Documents/dl3_gazebo_results_2018-07-30 18:35:12.794631', #previous bumper collision cases, now successes
                '/home/justin/Documents/dl3_gazebo_results_2018-07-30 18:50:09.987252'   #the rest of the 52:97 cases for egocylindrical
                ]

    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-07-30 20:08:55.373085'
                 ]
    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-07-30 21:15:57.399391'] #depth & ec dwa, standard dwa, teb 0:100

    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-07-31 18:54:52.888438']   #egocylindrical receding horizon 52:97

    filenames3 = ['/home/justin/Documents/dl3_gazebo_results_2018-08-01 18:57:16.943644']    #pips_ec_rh','depth_pips_dwa','egocylindrical_pips_dwa','dwa','teb' 0:100, 'sector' is really sector_laser (though called sector; need to change that)

    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-08-09 19:50:47.599175',    #egocylindrical_pips_dwa','dwa', plus no-recovery versions, campus 0:100, sector 0:26
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-10 14:24:53.367459',    #sector 26:64
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-10 19:33:55.865587',    #sector 64:100
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-13 20:52:35.074099']    #campus 0 barrels (0:100), 10 barrels (0:36); also included teb

    filenames.extend(['/home/justin/Documents/dl3_gazebo_results_2018-08-14 20:44:02.928378' #continuation of above set: campus 10 barrels 36:100 for all; plus teb for campus 20 barrels 0:100 and sector 0:100
     ,'/home/justin/Documents/dl3_gazebo_results_2018-08-15 12:49:36.399513' #missing case from above file; teb timing out (oscillating)
    # ,'/home/justin/Documents/dl3_gazebo_results_2018-08-15 12:56:04.487060' #repeat of above case; success this time...
     ])

    '/home/justin/Documents/dl3_gazebo_results_2018-08-15 13:32:10.591359' #52:97 depth_pips_dwa in sector; way worse than older results, so something's definitely wrong with recent updates to the controller

    '/home/justin/Documents/dl3_gazebo_results_2018-08-15 14:07:12.527471' #sector egocylindrical_pips_dwa (52:97); only 1 at a time; not as bad

    ['/home/justin/Documents/dl3_gazebo_results_2018-08-16 20:44:07.080949']    #pips_ec_rh & egocylindrical_pips_dwa (plus no recovery versions) sector (0:100) LOCAL-COSTMAP ENABLED (and going forward unless otherwise stated)

    ['/home/justin/Documents/dl3_gazebo_results_2018-08-17 12:44:37.019381']   #egocylindrical_pips_dwa localcostmap 0:40, 1 at a time, observed most of them with gazebo + rviz. Far better results than previous

    #Only 1 at a time until stated otherwise:
    filenames2 = ([
    '/home/justin/Documents/dl3_gazebo_results_2018-08-17 14:01:40.070974' #egocylindrical_pips_dwa sector (40:100)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-08-17 17:01:08.244777' #pips_ec_rh sector (40:94)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-08-17 20:50:04.918590' #egocylindrical_pips_dwa and pips_ec_rh (w/ and w/out recovery), teb, dwa. sector (0:33)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-08-18 16:23:36.669100' #sector; egocylindrical_pips_dwa & pips_ec_rh w/ recovery (33:40); egocylindrical_pips_dwa & pips_ec_rh w/out recovery, teb, dwa. sector (33:61)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-08-20 19:43:12.088761'
    ])

    seeds = [str(i) for i in range(0,50)] #(52,97)
    #analyzer.readFiles(filenames=filenames, whitelist={'seed':seeds, 'scenario':'sector', 'controller':['dwa','teb']}) #, blacklist={'controller':'teb'}

    filenames2.extend([
        '/home/justin/Documents/dl3_gazebo_results_2018-08-21 19:38:03.047302'  #multiclass, goal_regression, baseline_rl_goal sector (0,50)
    ])

    filenames4 = [
    '/home/justin/Documents/dl3_gazebo_results_2018-08-27 21:44:33.801033', #pips_ec_rh, sector 0:100, circle trajectories!
    '/home/justin/Documents/dl3_gazebo_results_2018-08-29 22:06:17.605848' #rl_goal, sector,0:100, circles
    ]

    #Added improved trajectory cropping at this point?

    filenames5 = [
        '/home/justin/Documents/dl3_gazebo_results_2018-08-30 21:25:55.858736' #pips_ec_rh sector (0:18)
        ,'/home/justin/Documents/dl3_gazebo_results_2018-08-30 22:37:07.640068' #pips_ec_rh sector (19:50), rl_single (0:50)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-08-31 10:48:22.570544' #rl_goal, multiclass, regression_goal sector (0:100)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-09-01 12:31:01.897767' # rl_goal_no_recovery, multiclass_no_recovery, pips_ec_rh_no_recovery, rl_single_no_recovery sector (0:44)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-09-01 17:50:40.516866' # rl_goal_no_recovery, multiclass_no_recovery, pips_ec_rh_no_recovery, rl_single_no_recovery sector (44:50)
    ,'/home/justin/Documents/dl3_gazebo_results_2018-09-01 18:21:55.618628' #pips_ec_rh_no_recovery_(paths: 5,25) (0:50) sector
    ,'/home/justin/Documents/dl3_gazebo_results_2018-09-02 00:01:01.691753' #pips_ec_rh_no_recovery_(paths: 9,15,19) (0:50) sector
    ]

    filenames6 = [
        '/home/justin/Documents/dl3_gazebo_results_2018-09-05 23:39:14.838354'
        ,'/home/justin/Documents/dl3_gazebo_results_2018-09-06 10:26:10.204236'
        ,'/home/justin/Documents/dl3_gazebo_results_2018-09-06 14:37:27.059749'
        ,'/home/justin/Documents/dl3_gazebo_results_2018-09-06 18:03:06.165308'
        ,'/home/justin/Documents/dl3_gazebo_results_2018-09-06 18:22:51.647290'
    ]

    #analyzer.readFiles(filenames=filenames5, whitelist={'seed':seeds, 'scenario':'sector'}) #, blacklist={'controller':'teb'}

    analyzer.readFiles(filenames=filenames6)

    analyzer.computeStatistics(independent=['scenario', 'controller'], dependent=['result'])
    analyzer.generateTable()

    #controllers = ['pips_ec_rh','pips_ec_rh_no_recovery', 'pips_ec_rh_no_recovery_5', 'pips_ec_rh_no_recovery_9', 'pips_ec_rh_no_recovery_15', 'pips_ec_rh_no_recovery_19', 'pips_ec_rh_no_recovery_25', 'rl_goal', 'multiclass', 'rl_single', 'egocylindrical_pips_dwa', 'goal_regression','teb', 'dwa', 'multiclass_no_recovery', 'rl_single_no_recovery']

    #goodseeds = analyzer.getCommonSuccessfulSeeds(controllers = controllers)
    
    #analyzer.clear()
    #analyzer.readFiles(filenames=filenames2, whitelist={'seed':goodseeds, 'scenario':'sector', 'controller':controllers}) #, blacklist={'controller':'teb'}

    '''
    for controller in controllers:
        print controller
        res = analyzer.getCases(has={'controller':controller, 'result':'SUCCEEDED'}) #, 'seed':seeds
        analyzer.getAverageTime(res)
    '''

    # analyzer.clear()
    # analyzer.readFiles(filenames=filenames, whitelist={'seed':seeds, 'scenario':'campus'})
    # analyzer.computeStatistics(independent=['num_barrels', 'controller'], dependent=['result'])


    #analyzer.compareControllers('egocylindrical_pips_dwa','pips_dwa')

    '''
    master = MultiMasterCoordinator()
    master.start()

    for a in range(0, 150):
        for controller in ['pips_ni', 'multiclass_propagated', 'rl_goal', 'rl_single', 'regression_goal',
                           'regression_goal_propagated', 'multiclass', "pips_dwa_propagated", "pips_dwa", 'teb',
                           'dwa']:  # 'rl_goal'
            for repetition in range(1):
                task = {'scenario': 'sector', 'controller': controller, 'seed': a}

                if not analyzer.contains(task):
                    master.task_queue.put(task)
                    print task







    master.waitToFinish()
    master.shutdown()


    #analyzer.computeStatistics(independent=['num_barrels','controller'], dependent=['result'])

    #analyzer.getFailCases(controller='brute_force')
    #analyzer.getMaxTime()

    '''
