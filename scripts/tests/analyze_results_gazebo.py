#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer


if __name__ == "__main__":
    start_time = time.time()
    analyzer = ResultAnalyzer()

    # filenames=['/home/justin/simulation_data/results_2019-08-27 22:53:45.456742', '/home/justin/simulation_data/results_2019-08-28 21:46:41.612063']

    # filenames=['/home/justin/simulation_data/results_2019-08-31 21:41:55.024900'] #.25 threshold

    # analyzer.readFiles(filenames = filenames, blacklist={'controller': 'laser_classifier_weighted_2d_no_neg'})

    # filenames=['/home/justin/simulation_data/results_2019-09-03 00:27:43.205849'] #.5 threshold, w/collision checking

    # analyzer.readFiles(filenames = filenames)

    # analyzer.clear()
    # filenames = ['/home/justin/simulation_data/results_2019-09-04 23:39:17.669680'] #p2d, nothing altered
    # analyzer.readFiles(filenames=filenames, whitelist={'controller': ['p2d']})

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
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_mpc_k_2023-08-01 23:10:09.062030']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-02 01:33:35.785750']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_mpc_k_2023-08-02 03:49:25.642351']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_mpc_k_noc_2023-08-05 22:51:39.206927']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 21:49:18.317252']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_2023-08-03 19:18:19.215754']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_noc_2023-08-06 16:59:09.886338']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:57:23.256144']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_2_2023-08-09 18:30:58.142352']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 18:06:02.266878']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_mpc_k_nostop_2023-08-15 01:18:19.834283']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 18:06:02.266878_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-02 01:33:35.785750_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_2_2023-08-09 18:30:58.142352_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_b_pose_noff_2023-08-14 21:06:13.967224_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_mpc_k_noc_nostop_2023-08-15 01:18:19.834283_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 21:49:18.317252_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_noc_2023-08-06 16:59:09.886338_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:57:23.256144_rename']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_3_3.5_2_fix6_rangefix_2023-09-08 03:50:35.546407']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rangefix_2023-09-07 19:17:10.663522']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rangefix_2023-09-09 15:01:02.943943']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_3_3.5_2_fix6_1.1_orient_20_rangefix_2023-09-09 00:12:53.317953']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_2_3.5_0.5_fix6_1.1_orient_20_rangefix_2023-09-08 19:55:36.717437']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_1.1_rangefix_2023-09-08 13:01:36.542370']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_1.1_orient_20_rangefix_2023-09-09 03:05:53.425630']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_fix6_1.1_rangefix_2023-09-09 18:24:50.182957']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_fix6_1.1_orient_20_rangefix_2023-09-09 15:01:56.070250']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2023-09-08 05:14:04.053888']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_3_3.5_2_fix6_rangefix_2023-09-08 03:50:35.546407_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rangefix_2023-09-07 19:17:10.663522_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rangefix_2023-09-09 15:01:02.943943_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_3_3.5_2_fix6_1.1_orient_20_rangefix_2023-09-09 00:12:53.317953_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_pose_2_3.5_0.5_fix6_1.1_orient_20_rangefix_2023-09-08 19:55:36.717437_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_1.1_rangefix_2023-09-08 13:01:36.542370_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_1.1_orient_20_rangefix_2023-09-09 03:05:53.425630_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_fix6_1.1_rangefix_2023-09-09 18:24:50.182957_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_fix6_1.1_orient_20_rangefix_2023-09-09 15:01:56.070250_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/turtlebot/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_fix6_1.1_orient_20_rangefix_2023-09-08 05:14:04.053888_rename']
    
    analyzer.readFiles(filenames=filenames)
    analyzer.generateGenericTable(independent=['scenario','controller'], dependent='result')
    # analyzer.generateGenericTable(independent=['controller'], dependent='result')

    new_filenames = []
    for i in range(5):
        new_filenames = []
        for f in filenames:
            new_filenames.append(f + '_' + str(i))

        print(new_filenames)
        analyzer.readFiles(filenames=new_filenames)
        analyzer.generateGenericTable(independent=['scenario','controller'], dependent='result')
    
    #analyzer.readFiles(filenames=filenames, whitelist={})
    #analyzer.generateGenericTable(independent=[ 'scenario','controller'], dependent='result')

    

    # analyzer.generateSingleTable()


    #analyzer.compareControllers('egocylindrical_pips_dwa','dwa')

