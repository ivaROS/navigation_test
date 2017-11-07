import csv
import time

def getDict(dict, key, isList):
    if key not in dict:
        if isList:
            dict[key] = []
        else:
            dict[key] = {}
    return dict[key]

def readFile(filename):
    with open(filename, 'rb') as csvfile:
        datareader = csv.DictReader(csvfile, restval='')

        result_list = []
        fieldnames = datareader.fieldnames
        for entry in datareader:
            result_list.append(entry)

    return result_list, fieldnames

def groupResults(result_list, grouping_fields):
    scenarios = {}

    for result in result_list:
        dict = scenarios
        for field in grouping_fields:
            if field in result:
                islast = (field == grouping_fields[-1])
                dict = getDict(dict, result[field],isList=islast)
        dict.append(result)

    return scenarios


class ResultAnalyzer:


    def readFile(self, filename):
        self.results, self.fieldnames = readFile(filename=filename)

    # Sort first by scene, then by num_barrels, then by seed, then by controller
    # So: list of scenes; each scene contains
    def groupResults(self, grouping_fields):
        grouped_results = groupResults(result_list=self.results, grouping_fields=grouping_fields)
        return grouped_results



    #Ex: for each scenario/num_barrel, controller: report break down of results
    # go through all results, if level = "result", get len of dict and add to scenario/num_barrel/controller/result field
    def computeStatistics(self, grouping_fields):
        grouped_results = self.groupResults(grouping_fields=grouping_fields)


    def __init__(self):
        self.fieldnames = None
        self.results = None



if __name__ == "__main__":

    filename = '/home/justin/Documents/gazebo_results_2017-11-06 19:55:13.337258'
    grouping_fields = ["scenario", "num_barrels", "seed", "controller", "result"]

    start_time = time.time()
    analyzer = ResultAnalyzer()
    analyzer.readFile(filename=filename)
    analyzer.groupResults(grouping_fields=grouping_fields)

    grouping_fields = ["scenario", "num_barrels", "controller", "result"]
    analyzer.computeStatistics(grouping_fields=grouping_fields)