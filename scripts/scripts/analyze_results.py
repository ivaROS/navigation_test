#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer


if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()

    filenames=['/home/justin/Downloads/chapter_experiments_2019-01-25 22:10:11.548983']
    #
    # analyzer.readFiles(filenames=filenames)
    #
    # filenames2=['/home/justin/Downloads/box_turtle1','/home/justin/Downloads/turtlebot1']
    #
    # analyzer.readFiles(filenames=filenames2,whitelist={'controller':'dwa'})

    #filenames3=['/data/fall2018/chapter_experiments/chapter_experiments_2019-03-05 22:44:59.048367']

    analyzer.readFiles(filenames=filenames)
    #analyzer.computeStatistics(independent=['scenario', 'controller'], dependent=['result'])

    #analyzer.generateTable()

    #analyzer.generateGenericTable(independent=['scenario', 'robot', 'controller'], dependent='result')

    analyzer.generateGenericTable(independent=['controller','robot', 'scenario'], dependent='result')


    # analyzer.generateSingleTable()


    #analyzer.compareControllers('egocylindrical_pips_dwa','dwa')

