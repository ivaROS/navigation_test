import csv
import time
from gazebo_master import MultiMasterCoordinator


class ResultAnalyzer:


    def readFile(self, filename):
        with open(filename, 'rb') as csvfile:
            datareader = csv.DictReader(csvfile, restval='')

            result_list = []
            fieldnames = datareader.fieldnames
            for entry in datareader:
                result_list.append(entry)
        self.results += result_list

    def readFiles(self, filenames):
        for filename in filenames:
            self.readFile(filename)

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
           for controller in key_values[independent[1]]:
                total = 0
                for result in key_values[dependent[0]]:
                    key = frozenset({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())
                    if key in statistics:
                        total+= statistics[key]
                print controller + " controller:"
                for result in key_values[dependent[0]]:
                    key = frozenset(
                        {independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())
                    if key in statistics:
                        num = statistics[frozenset({independent[1]: controller, independent[0]: num_barrels, dependent[0]: result}.items())]
                        print result + ": " + str(num) + "\t" + str(float(num)/total)
                print ""

    def exists(self, scenario):
        pass


    def __init__(self):
        self.fieldnames = []
        self.results = []



if __name__ == "__main__":

    filenames = ['/home/justin/Documents/dl_gazebo_results_2018-02-20 14:17:20.349670',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-19 20:17:06.041463',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 15:18:36.378260',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 17:39:02.442583',
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 19:55:37.855977'] #Initial set of runs with different controllers

    #filenames = ['/home/justin/Documents/dl2_gazebo_results_2018-02-21 13:40:16.659915']   #repeated baselines, trying to ensure that recovery behaviors disabled

    #filenames= ['/home/justin/Documents/dl2_gazebo_results_2018-02-26 22:00:58.731302', '/home/justin/Documents/dl2_gazebo_results_2018-02-27 21:44:43.554072' ] #Reran brute_force, turned out bug in decimation propagation
    filenames = ['/home/justin/Documents/dl2_gazebo_results_2018-03-02 20:12:04.419906' ] #reran brute_force after fixing some bugs, still doesn't look good

    start_time = time.time()
    analyzer = ResultAnalyzer()
    analyzer.readFiles(filenames=filenames)

    analyzer.computeStatistics(independent=['num_barrels','controller'], dependent=['result'])

    analyzer.getFailCases(controller='brute_force')

