import csv
import time



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
                 '/home/justin/Documents/dl_gazebo_results_2018-02-20 19:55:37.855977']

    start_time = time.time()
    analyzer = ResultAnalyzer()
    analyzer.readFiles(filenames=filenames)

    analyzer.computeStatistics(independent=['num_barrels','controller'], dependent=['result'])

