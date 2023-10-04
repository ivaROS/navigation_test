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
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_ni_mpc_k_2023-07-31 17:10:49.397191']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_ni_pose_2023-07-31 18:31:19.888271']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_pose_noff_2023-08-01 00:59:52.661876']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_mpc_k_2023-07-31 19:54:52.859190']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_p_ni_mpc_k_2023-07-31 21:35:04.636084']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_p_mpc_k_2023-08-01 00:20:03.252407']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_mpc_k_2023-08-01 17:52:56.379359']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-06 01:49:17.732358']
    # filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_2023-08-01 20:46:24.670210']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_noc_2023-08-05 21:09:37.932434']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 17:54:35.093422']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_noc_2023-08-06 16:25:35.877280']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:11:30.600690']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_pose_noff_1_1_1_2023-08-08 02:22:27.165724']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_0.5_2023-08-08 03:13:14.843171']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 17:06:28.065570']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_nostop_2023-08-15 17:47:55.757627']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_mpc_k_nostop_2023-08-14 17:06:28.065570_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_ni_pose_2023-08-06 01:49:17.732358_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_b_pose_noff_3_3.5_0.5_2023-08-08 03:13:14.843171_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_b_pose_noff_2023-08-01 00:59:52.661876_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_mpc_k_nostop_2023-08-15 17:47:55.757627_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_sg_fast_0.4_p_ni_pose_noc_2023-08-05 17:54:35.093422_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_noc_2023-08-06 16:25:35.877280_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v1/results_1_p_pose_noff_nopo_noc_2023-08-06 20:11:30.600690_rename']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_4_4_0.5_fix6_rfix_2023-09-11 15:15:42.242858']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rfix_2023-09-10 15:33:53.546143']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rfix_2023-09-10 04:13:35.513124']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_4_4_0.5_fix6_orient_20_rfix_2023-09-12 00:03:59.885116']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_2_3.5_0.5_fix6_orient_20_rfix_2023-09-10 23:36:22.745163']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_rfix_2023-09-11 03:13:48.687762']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_orient_20_rfix_2023-09-11 01:28:55.923747']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_a5_fix6_rfix_2023-09-11 22:11:18.320517']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_a5_fix6_orient_20_rfix_2023-09-11 19:22:56.630124']
    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_a5_fix6_orient_20_rfix_2023-09-11 17:25:36.505512']

    filenames = ['/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_4_4_0.5_fix6_rfix_2023-09-11 15:15:42.242858_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_fix6_rfix_2023-09-10 15:33:53.546143_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_pose_noff_2_3.5_0.5_po_1.2_1_fix6_rfix_2023-09-10 04:13:35.513124_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_4_4_0.5_fix6_orient_20_rfix_2023-09-12 00:03:59.885116_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_pose_noff_2_3.5_0.5_fix6_orient_20_rfix_2023-09-10 23:36:22.745163_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_pose_fix6_rfix_2023-09-11 03:13:48.687762_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_pose_fix6_orient_20_rfix_2023-09-11 01:28:55.923747_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_p_ni_mpc_k_nonstop_a5_fix6_rfix_2023-09-11 22:11:18.320517_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_a5_fix6_orient_20_rfix_2023-09-11 19:22:56.630124_rename',
                 '/home/shiyu/simulation_data/sgap_benchmark/stdr/v2/results_1_sg_fast_0.4_b_ni_mpc_k_nonstop_po_1.2_1_a5_fix6_orient_20_rfix_2023-09-11 17:25:36.505512_rename']
    
    # analyzer.readFiles(filenames=filenames)
    # analyzer.generateGenericTable(independent=['world','controller'], dependent='result')
    # analyzer.generateGenericTable(independent=['controller'], dependent='result')

    new_filenames = []
    for i in range(5):
        new_filenames = []
        for f in filenames:
            new_filenames.append(f + '_' + str(i))

        print(new_filenames)
        analyzer.readFiles(filenames=new_filenames)
        analyzer.generateGenericTable(independent=['world','controller'], dependent='result')

    #analyzer.readFiles(filenames=filenames, whitelist={})
    #analyzer.generateGenericTable(independent=[ 'scenario','controller'], dependent='result')

    # analyzer.generateSingleTable()


    #analyzer.compareControllers('egocylindrical_pips_dwa','dwa')

