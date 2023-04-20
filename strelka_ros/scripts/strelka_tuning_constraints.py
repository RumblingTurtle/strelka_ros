import os
import roslaunch
import rospy
import numpy as np
from scipy.optimize import minimize, Bounds
from unitreepy.robots.a1.constants import STAND_ANGLES, INIT_ANGLES
import tf
from tf.transformations import quaternion_matrix
import time

def track_follower(max_velocity_x: float, max_curvature_threshold: float):
    
    start_time = time.time()
    
    # Launch full pipeline
    launch_path = os.path.expanduser(
        '~')+"/catkin_ws/src/strelka_ros/strelka_ros/launch/a1_full_pipeline.launch"

    cli_args = [launch_path,'load_params:=false']

    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    # Common
    rospy.set_param('/common/gait', 'trot')
    rospy.set_param('/common/blind', False)
    rospy.set_param('/common/perfect_odometry', True)
    rospy.set_param('/common/sim_slowdown', 1.0)

    # High Command
    rospy.set_param('/high_command/use_move_base', False)
    rospy.set_param('/high_command/velocity_x', max_velocity_x)
    rospy.set_param('/high_command/velocity_y', 0.0)
    rospy.set_param('/high_command/velocity_yaw', 0.0)
    rospy.set_param('/high_command/foot_height', 0.08)
    rospy.set_param('/high_command/body_height', 0.27)

    # MPC
    rospy.set_param('/mpc/mpc_step_dt', 0.02)
    rospy.set_param('/mpc/mpc_horizon_steps', 15)

    # Body planner

    rospy.set_param('/body_planner/height_lpf_freq', 0.9) # Stairs - 30, Sparse - 0.9
    rospy.set_param('/body_planner/pitch_lpf_freq', 0.9) # Stairs - 10, Sparse - 0.9

    # Foot planner

    rospy.set_param('/foot_planning/foothold_search_radius',0.1)
    rospy.set_param('/foot_planning/upd_footholds_continuously', False)
    rospy.set_param('/foot_planning/max_curvature_threshold', max_curvature_threshold)
    rospy.set_param('/foot_planning/sdf_threshold', 0.02)
    rospy.set_param('/foot_planning/height_difference_threshold', 0.12)
    rospy.set_param('/foot_planning/max_dist_to_nominal', 0.3)

    parent.start()

    rospy.init_node("FailureListener")
    listener = tf.TransformListener()


    r = rospy.Rate(1000)

    succesful_path_x = 2

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/trunk', '/odom', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        angle =  np.degrees(np.arccos(quaternion_matrix(rot)[2,2]))
        if abs(angle) > 40 or trans[2] < -0.5:
            succesful_path_x = trans[0]
            print("Robot failed the track")
            break
        if (time.time() - start_time) > 100:
            succesful_path_x = trans[0]
            break
    parent.shutdown()
    return succesful_path_x

max_velocity_x_range = list(map(float,np.arange(0.17,0.29, 0.01)))

max_curvature_range = list(map(float,np.arange(0.8,1.0, 0.01)))

for i in range(len(max_velocity_x_range)):
    for j in range(len(max_curvature_range)):
        x1_cost = track_follower(max_velocity_x_range[i], max_curvature_range[j])
        time.sleep(5)
        x2_cost = track_follower(max_velocity_x_range[i], max_curvature_range[j])
        time.sleep(5)
        x3_cost = track_follower(max_velocity_x_range[i], max_curvature_range[j])
        mean_cost = (x1_cost + x2_cost) / 2
        file = open("results.txt", "a")
        file.write(str(mean_cost) + ' --- ' + ' '.join([str(param) for param in [max_velocity_x_range[i], max_curvature_range[j]]]) + '\n')
        time.sleep(10)
