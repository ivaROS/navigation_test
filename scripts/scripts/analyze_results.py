#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer


if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()
    filenames = ['~/simulation_data/results_2019-09-06 05:04:56.753298']
    analyzer.readFiles(filenames=filenames, whitelist={})
    analyzer.generateGenericTable(independent=['scenario','controller'], dependent='result')