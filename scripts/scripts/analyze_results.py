import csv
import time
from gazebo_master import MultiMasterCoordinator


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

    def exists(self, scenario):
        pass

    def freezeSet(self, independent):
        self.frozen_set = []
        key_values = {}
        for entry in self.results:
            condition = {key: entry[key] for key in independent}
            conditionset = frozenset(condition.items())
            self.frozen_set.append(conditionset)


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

    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-08-01 18:57:16.943644']    #pips_ec_rh','depth_pips_dwa','egocylindrical_pips_dwa','dwa','teb' 0:100, 'sector' is really sector_laser (though called sector; need to change that)

    filenames = ['/home/justin/Documents/dl3_gazebo_results_2018-08-09 19:50:47.599175',    #egocylindrical_pips_dwa','dwa', plus no-recovery versions, campus 0:100, sector 0:26
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-10 14:24:53.367459',    #sector 26:64
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-10 19:33:55.865587',    #sector 64:100
                 '/home/justin/Documents/dl3_gazebo_results_2018-08-13 20:52:35.074099']    #campus 0 barrels (0:100), 10 barrels (0:36); also included teb

    filenames.extend(['/home/justin/Documents/dl3_gazebo_results_2018-08-14 20:44:02.928378' #continuation of above set: campus 10 barrels 36:100 for all; plus teb for campus 20 barrels 0:100 and sector 0:100
     ,'/home/justin/Documents/dl3_gazebo_results_2018-08-15 12:49:36.399513' #missing case from above file; teb timing out (oscillating)
    # ,'/home/justin/Documents/dl3_gazebo_results_2018-08-15 12:56:04.487060' #repeat of above case; success this time...
     ])

    '/home/justin/Documents/dl3_gazebo_results_2018-08-15 13:32:10.591359' #52:97 depth_pips_dwa in sector; way worse than older results, so something's definitely wrong with recent updates to the controller

    '/home/justin/Documents/dl3_gazebo_results_2018-08-15 14:07:12.527471' #sector egocylindrical_pips_dwa (52:97); only 1 at a time; not too bad

    seeds = [str(i) for i in range(0,100)] #(52,97)
    analyzer.readFiles(filenames=filenames, whitelist={'seed':seeds, 'scenario':'sector'}) #, blacklist={'controller':'teb'}

    analyzer.computeStatistics(independent=['scenario', 'controller'], dependent=['result'])

    analyzer.clear()
    analyzer.readFiles(filenames=filenames, whitelist={'seed':seeds, 'scenario':'campus'})
    analyzer.computeStatistics(independent=['num_barrels', 'controller'], dependent=['result'])


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
