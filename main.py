from __future__ import print_function
from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from Graph import SearchNode, Path
import math
import numpy as np 
import os
import sys
import argparse

sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['padm_project', 'padm_project/ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_collision_data

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from padm_project.src.world import World
from padm_project.src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly


def find_ff_heuristic(states, predicates, actions, pos_goal,  neg_goal):
    """Given initial state, predicates, actions, and goals, generates a relaxed planning graph and returns the 
    fast forward heuristic
    """
    neg_start_states = set()
    for p in predicates:
        if p not in states:
            neg_start_states.add(p)


    start_node = SearchNode([states, frozenset(neg_start_states)])
    q = [start_node]
    ff_heuristic = 0
    while q:
        current_node = q.pop(0)
        pos_states = current_node.state[0]
        neg_states = current_node.state[1]
        if check_goal((pos_states, neg_states), pos_goal, neg_goal, ff=True):
            return ff_heuristic
        if ff_heuristic > 20:
            return float('inf')

        total_add_effects = frozenset(current_node.state[0])
        total_del_effects = frozenset(current_node.state[1])
        for a in actions:
            if can_perform_action(current_node.state, a, ff=True):
                total_add_effects = frozenset.union(total_add_effects, a.add_effects)
                total_del_effects = frozenset.union(total_del_effects, a.del_effects)
        next_state = [total_add_effects, total_del_effects]
        next_node = SearchNode(next_state, current_node)
        ff_heuristic += 1
        q.append(next_node)
    return False

def EFHC(states, predicates, actions, pos_goal, neg_goal):
    current_node = SearchNode(states)
    while True:
        next_best = [] #(next_node, ff)
        current_ff = find_ff_heuristic(current_node.state, predicates, actions, pos_goal, neg_goal)
        for a in actions:
            if can_perform_action(current_node.state, a):
                diff = current_node.state.difference(a.del_effects)
                next_state = frozenset.union(diff, a.add_effects)
                next_node = SearchNode(next_state, current_node, action=a.name)
                next_ff = find_ff_heuristic(next_node.state, predicates, actions, pos_goal, neg_goal)
                #print(next_ff)
                if next_best:
                    if next_ff < next_best[0][1]:
                        next_best = [[next_node, next_ff]]
                    elif next_ff == next_best[0][1]:
                        next_best.append([next_node, next_ff])
                else:
                    next_best.append((next_node, next_ff))
        for i in next_best:
            print(i[1])
        if len(next_best) > 1:
            return NotImplementedError
        else:
            current_node = next_best[0][0]
        if check_goal(current_node.state, pos_goal, neg_goal):
            return Path(current_node)
                


def BFS(states, actions, pos_goal, neg_goal):
    start_node = SearchNode(states)
    q = [start_node]
    visited = set()
    while q:
        current_node = q.pop(0)
        if check_goal(current_node.state, pos_goal, neg_goal):
            return Path(current_node)
        for a in actions:
            if can_perform_action(current_node.state, a):
                diff = current_node.state.difference(a.del_effects)
                next_state = frozenset.union(diff, a.add_effects)
                next_node = SearchNode(next_state, current_node, action=a.name)
                if next_node.state not in visited:
                    q.append(next_node)
                    visited.add(next_state)
    return False

def check_goal(state, positive_goals, negative_goals, ff=False):
    """return true if state satisfies goal state"""
    if ff == False:
        if positive_goals.intersection(state) != positive_goals:
            return False
        if negative_goals.intersection(state):
            return False
        return True     
    else:
        pos_states = state[0]
        neg_states = state[1]
        if positive_goals.intersection(pos_states) != positive_goals:
            return False
        if negative_goals.intersection(neg_states) != negative_goals:
            return False
        return True


def can_perform_action(state, action, ff=False):
    """
    Given an action and state, return true true if the action can be performed, false o.w. 
    """
    if ff == False:
        if action.positive_preconditions.intersection(state) != action.positive_preconditions:
            return False
        if action.negative_preconditions.intersection(state):
            return False
        return True
    else:
        pos_states = state[0]
        neg_states = state[1]
        if action.positive_preconditions.intersection(pos_states) != action.positive_preconditions:
            return False
        if action.negative_preconditions.intersection(neg_states) != action.negative_preconditions:
            return False
        return True


def solve():
    # Solve the activity planning problem using EHC
    return 

UNIT_POSE2D = (0., 0., 0.)

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def motion_planner_main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    print("Going to use IK to go from a sample start state to a goal state\n")
    for i in range(2):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(world.robot, world.arm_joints, conf)
        wait_for_user()
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        end_pose = multiply(start_pose, Pose(Point(z=1.0)))
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
    print("Going to operate the base without collision checking")
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        #print(get_collision_data(world.robot))
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        if (i % 30 == 0):
            wait_for_user()
    wait_for_user()
    world.destroy()

def main():
    plan = ''
    parser = PDDL_Parser()
    domain_filename = 'kitchenDomain.pddl'
    problem_filename = 'pb1.pddl'
    parser.parse_domain(domain_filename)
    parser.parse_problem(problem_filename)

   # G = generate_graph(parser.state, parser.actions, parser.predicates)
    # plan = solve()
    # print(plan)
    B = BFS(parser.state, parser.actions, parser.positive_goals, parser.negative_goals)
    print(B.path)
    # Call motion planner
    motion_planner_main()
    # ff = find_ff_heuristic(parser.state, parser.predicates, parser.actions, parser.positive_goals, parser.negative_goals)
    # print(ff)
    # HC =  EFHC(parser.state, parser.predicates, parser.actions, parser.positive_goals, parser.negative_goals)
    # print(HC)
    return 

if __name__ == "__main__":
    main()