#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer


if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()

    filenames=['/home/justin/simulation_data/results_2019-08-27 22:53:45.456742', '/home/justin/simulation_data/results_2019-08-28 21:46:41.612063']

    filenames=['/home/justin/simulation_data/results_2019-08-31 21:41:55.024900'] #.25 threshold

    analyzer.readFiles(filenames = filenames, blacklist={'controller': 'laser_classifier_weighted_2d_no_neg'})

    filenames=['/home/justin/simulation_data/results_2019-09-03 00:27:43.205849'] #.5 threshold, w/collision checking

    analyzer.readFiles(filenames = filenames)

    analyzer.clear()
    filenames = ['/home/justin/simulation_data/results_2019-09-04 23:39:17.669680'] #p2d, nothing altered
    analyzer.readFiles(filenames=filenames, whitelist={'controller': ['p2d']})

    #
    # analyzer.readFiles(filenames=filenames)
    #
    # filenames2=['/home/justin/Downloads/box_turtle1','/home/justin/Downloads/turtlebot1']
    #
    # analyzer.readFiles(filenames=filenames2,whitelist={'controller':'dwa'})

    #filenames3=['/data/fall2018/chapter_experiments/chapter_experiments_2019-03-05 22:44:59.048367']

    #analyzer.readFiles(filenames=filenames)
    #analyzer.computeStatistics(independent=['scenario', 'controller'], dependent=['result'])

    #analyzer.generateTable()

    #analyzer.generateGenericTable(independent=['scenario', 'robot', 'controller'], dependent='result')

    #analyzer.generateGenericTable(independent=[ 'scenario','controller'], dependent='result')

    analyzer.clear()
    filenames = ['/home/justin/simulation_data/results_2019-09-06 05:04:56.753298'] #p2d and classifier, nothing altered
    #analyzer.readFiles(filenames=filenames, whitelist={})
    #analyzer.generateGenericTable(independent=[ 'scenario','controller'], dependent='result')

    analyzer.readFiles(filenames=['/home/justin/simulation_data/results_2020-05-27 01:25:54.203382'])
    analyzer.generateGenericTable(independent=['controller', 'scenario','min_obstacle_spacing'], dependent='result')

    # analyzer.generateSingleTable()


    #analyzer.compareControllers('egocylindrical_pips_dwa','dwa')

