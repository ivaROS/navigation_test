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


def computeStatistics(results):
    res = {}
    total = 0
    for key in results:
        num = len(results[key])
        res[key] = num
        total += num

    for key in res:
        print key + "\t" + str(res[key])


def iterateLevels(results, grouping_fields, level):
    if level < len(grouping_fields) - 1:
        print grouping_fields[level] + "s:"
        for key in results:
            print key
            iterateLevels(results=results[key],grouping_fields=grouping_fields,level=level + 1)

    else:
        computeStatistics(results=results)

def getLevel(results, grouping_fields, level):
    if level > 0:
        for key in results:
            for res in getLevel(results=results[key],grouping_fields=grouping_fields,level=level - 1):
                yield res
    else:
        yield results




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


        iterateLevels(results=grouped_results,grouping_fields=grouping_fields,level=0)


    def getDifferentiatingScenarios(self, grouping_fields):
        grouped_results = self.groupResults(grouping_fields=grouping_fields)

        # Returns dict containing results from all controllers for the same scenario
        for result_group in getLevel(grouped_results, grouping_fields=grouping_fields, level= len(grouping_fields)-1):
            #print result_group
            res = set()
            scene = None
            for controller in result_group:
                res.add(result_group[controller][0]["result"])  #Note: this is only valid if identical runs have same result
                if scene is None:
                    scene = result_group[controller][0]['scenario'] + ":" + result_group[controller][0]['num_barrels'] + ":" + result_group[controller][0]['seed']
            if len(res) > 1:
                print scene

    def getFailScenarios(self, grouping_fields, controllers=None):
        grouped_results = self.groupResults(grouping_fields=grouping_fields)

        print "Scenarios failed by " + ("All" if controllers is None else str(controllers)) + ":"
        # Returns dict containing results from all controllers for the same scenario
        for result_group in getLevel(grouped_results, grouping_fields=grouping_fields, level= len(grouping_fields)-1):
            #print result_group
            res = set()
            res2 = set()
            scene = None
            for controller in result_group:
                if controllers is None or controller in controllers:
                    res.add(result_group[controller][0]["result"])  #Note: this is only valid if identical runs have same result
                    if scene is None:
                        scene = result_group[controller][0]['scenario'] + ":" + result_group[controller][0]['num_barrels'] + ":" + result_group[controller][0]['seed']
                        scene = result_group[controller][0]
                else:
                    if 'SUCCEEDED' not in result_group[controller][0]["result"]:
                        res.add('SUCCEEDED')

            if 'SUCCEEDED' not in res:
                print scene



    def __init__(self):
        self.fieldnames = None
        self.results = None



if __name__ == "__main__":

    filename = '/home/justin/Documents/gazebo_results_2017-11-29 19:41:30.871462'
    grouping_fields = ["scenario", "num_barrels", "seed", "target_id","controller", "result",]

    start_time = time.time()
    analyzer = ResultAnalyzer()
    analyzer.readFile(filename=filename)
    analyzer.groupResults(grouping_fields=grouping_fields)

    grouping_fields = ["scenario", "num_barrels", "controller", "result"]
    analyzer.computeStatistics(grouping_fields=grouping_fields)

    grouping_fields = ["scenario", "num_barrels", "controller", "result"]
    #analyzer.getDifferentiatingScenarios(grouping_fields=grouping_fields)

    #analyzer.getFailScenarios(grouping_fields=grouping_fields, controllers=["pips_dwa", "octo_dwa"])

    #analyzer.getFailScenarios(grouping_fields=grouping_fields)

    #analyzer.getFailScenarios(grouping_fields=grouping_fields, controllers=["octo_dwa"])

    analyzer.getFailScenarios(grouping_fields=grouping_fields, controllers=["pips_dwa"])