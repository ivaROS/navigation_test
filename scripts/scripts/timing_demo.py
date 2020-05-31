#!/usr/bin/env python
import execution


demo_location = "demo"
host_pkg = 'nav_scripts'

log_path = execution.run_test(rosbag_launch_dir = 'launch/rosbag', tests = ["egoteb"], logging_location = demo_location, test_host_pkg = host_pkg)
execution.parse_results(demo_location, host_pkg, log_path)