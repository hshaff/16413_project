from __future__ import print_function
from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from Graph import SearchNode, Path
from utils import *
from traj import get_opt_traj
import math
import numpy as np 
import os
import sys
import argparse

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['padm_project', 'padm_project/ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, unit_from_theta, get_joint_positions, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_collision_data, get_configuration, quat_from_euler, euler_from_quat, is_pose_close, get_joint, get_joint_position, joint_from_name, get_joints, get_joint_name, get_aabb_center, get_aabb, get_euler, get_all_links, get_link_names, get_link_parent, create_attachment, get_joint_names, get_joint_limits

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from padm_project.src.world import World
from padm_project.src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, open_surface_joints, get_surface_obstacles, \
    surface_from_name #, translate_linearly

def main_plan():
    plan = ''
    parser = PDDL_Parser()
    domain_filename = 'kitchenDomain.pddl'
    problem_filename = 'pb1.pddl'
    parser.parse_domain(domain_filename)
    parser.parse_problem(problem_filename)
    B = BFS(parser.state, parser.actions, parser.positive_goals, parser.negative_goals)
    print(B.path)
    world = create_world()
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    move_into_position(world)
    start_pose = get_link_pose(world.robot, tool_link)

    for i, activity in enumerate(B.path):
        print('activity:', activity)
        print('Start joint pos', get_joint_positions(world.robot, ik_joints))
        goal_pose = get_goal_pose(activity, world)
        path = rrt(start_pose, random_pose, world.robot, ik_joints, tool_link, goal_pose) #add custom bounds to rrt sampling
        current_pose = start_pose
        if not path:
            # RRT did not find a path, move closer to the activity goal
            print('Current Pose:', current_pose)
            print('Goal Pose:', goal_pose)
            pos_update = translate_linearly(world, 0.01, rot=np.pi/2)
            set_joint_positions(world.robot, world.base_joints, pos_update)
            for j in range(60):
                pos_update = translate_linearly(world, 0.01)
                set_joint_positions(world.robot, world.base_joints, pos_update)
            pos_update = translate_linearly(world, 0.01, rot=-np.pi/2)
            set_joint_positions(world.robot, world.base_joints, pos_update)
            for j in range(20):
                pos_update = translate_linearly(world, 0.01)
                set_joint_positions(world.robot, world.base_joints, pos_update)
            wait_for_user()
            start_pose = get_link_pose(world.robot, tool_link)
            current_pose = start_pose
            path = rrt(start_pose, random_pose, world.robot, ik_joints, tool_link, goal_pose) #add custom bounds to rrt sampling

        for p in path:
            end_pose = p
            for pose in interpolate_poses(current_pose, end_pose, pos_step_size=0.01):
                conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
                set_joint_positions(world.robot, ik_joints, conf)
                if B.path[i-1] == 'pickup-spam':
                    entity_name = 'potted_meat_can1'
                    body = world.get_body(entity_name)
                    set_pose(body, pose)
                elif B.path[i-1] == 'pickup-sugar':
                    entity_name = 'sugar_box0'
                    body = world.get_body(entity_name)
                    set_pose(body, pose)
            current_pose = end_pose
        perform_actions(activity, world, current_pose)
        print(activity,' complete')
        #wait_for_user()
def test_optimal_traj():
    plan = ''
    parser = PDDL_Parser()
    domain_filename = 'kitchenDomain.pddl'
    problem_filename = 'pb1.pddl'
    parser.parse_domain(domain_filename)
    parser.parse_problem(problem_filename)
    B = BFS(parser.state, parser.actions, parser.positive_goals, parser.negative_goals)
    print(B.path)
    world = create_world()
    tool_link = link_from_name(world.robot, 'panda_hand')
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    move_into_position(world)
    start_pose = get_link_pose(world.robot, tool_link)

    traj = get_opt_traj()

    for conf in traj:
        set_joint_positions(world.robot, ik_joints, conf)
        wait_for_user()

def main():
    #main_plan()
    test_optimal_traj()
    return 

if __name__ == "__main__":
    main()