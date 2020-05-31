#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer


if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()
    filenames = ['~/simulation_data/results_2019-10-15 18:14:25.927053']
    analyzer.readFiles(filenames=filenames, whitelist={})
    analyzer.generateGenericTable(independent=['scenario','min_obstacle_spacing','controller'], dependent='result')
