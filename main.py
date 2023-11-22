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
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_collision_data, get_configuration, quat_from_euler, euler_from_quat, is_pose_close, get_joint, get_joint_position, joint_from_name, get_joints, get_joint_name, get_aabb_center, get_aabb, get_euler

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

# def motion_planner_main():
#     print('Random seed:', get_random_seed())
#     print('Numpy seed:', get_numpy_seed())

#     np.set_printoptions(precision=3, suppress=True)
#     world = World(use_gui=True)
#     sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
#     spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
#     wait_for_user()
#     world._update_initial()
#     tool_link = link_from_name(world.robot, 'panda_hand')
#     joints = get_movable_joints(world.robot)
#     print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
#     print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
#     sample_fn = get_sample_fn(world.robot, world.arm_joints)
#     print("Going to use IK to go from a sample start state to a goal state\n")
#     for i in range(2):
#         print('Iteration:', i)
#         conf = sample_fn()
#         print('CONF', conf)
#         set_joint_positions(world.robot, world.arm_joints, conf)
#         for joint, value in zip(world.arm_joints, conf):
#             print('JOINT', joint, 'Value', value)
#         wait_for_user()
#         #print('config', get_configuration(world.robot))
#         ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
#         start_pose = get_link_pose(world.robot, tool_link)
#         print('STRT', start_pose)
#         end_pose = multiply(start_pose, Pose(Point(z=1.0)))
#         print('endpose', end_pose)
#         for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.001):
#             print('pose', pose)
#             conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
#             print('conf2', conf)

#             if conf is None:
#                 print('Failure!')
#                 wait_for_user()
#                 break
#             set_joint_positions(world.robot, ik_joints, conf)
#     print("Going to operate the base without collision checking")
#     for i in range(100):
#         goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
#         #print(get_collision_data(world.robot))
#         set_joint_positions(world.robot, world.base_joints, goal_pos)
#         if (i % 30 == 0):
#             wait_for_user()
#     wait_for_user()
#     world.destroy()

def rrt(start_pose, sample_fn, goal_pose=Pose(point=(1.5,0,0))):
    print('RRT Goal',goal_pose)
    print('RRT start',start_pose)
    V = set([start_pose])
    E = set()
    G = (V,E)
    path = 0
    temp_path = []
    while True:
        x_rand = sample_fn()  #
        #print("x_rand", x_rand)
        x_nearest = nearest(G, x_rand)
        #print('x_nearest', x_nearest)
        x_new = steer(x_nearest, x_rand)
        #print('x_new', x_new)
        if True: #obstacle avoidance goes here
            V.add(x_new)
            E.add((x_nearest, x_new))
            #print(x_new)
        #temp_path.append(x_new)
            #if is_pose_close(x_new, goal_pose, pos_tolerance=1e-1, ori_tolerance=1*np.pi):  # goal detection
            if is_close(x_new, goal_pose):
                path, num_nodes, G = extract_path(G, x_new, start_pose)
                print(num_nodes, len(path))
                return path
    #return temp_path

def extract_path(G, x_end, start_pose):
    V = G[0]
    E = G[1]
    path = [x_end]
    current_node = x_end
    while True:
        if current_node == start_pose:
            return path[::-1], len(V), (V,E)
        for edge in E:
            if edge[1] == current_node:
                path.append(edge[0])
                current_node = edge[0]


def nearest(G, x_rand):
    V = G[0]
    E = G[1]
    closest_dist = float('inf')
    x_nearest = None
    for pose in V:
        x, y, z = pose[0][0], pose[0][1], pose[0][2]  # only use position for now
        x_rand_x, x_rand_y, x_rand_z = x_rand[0][0], x_rand[0][1], x_rand[0][2]
        dist = np.sqrt((x-x_rand_x)**2 + (y - x_rand_y)**2 + (z - x_rand_z)**2)
        #dist = np.sqrt((v[0]-x_rand[0])**2 + (v[1]-x_rand[1])**2 + (v[2]-x_rand[2])**2 + (v[3]-x_rand[3])**2 + (v[4]-x_rand[4])**2 +(v[5]-x_rand[5])**2 + (v[6]-x_rand[6])**2)
        if dist < closest_dist:
            closest_dist = dist 
            x_nearest = pose 
    return x_nearest

def steer(x_nearest, x_rand):
    d = .05
    orientation = x_nearest[1]  # untouched for now
    x1, y1, z1 = x_nearest[0][0], x_nearest[0][1], x_nearest[0][2]  # only use position for now
    x2, y2, z2 = x_rand[0][0], x_rand[0][1], x_rand[0][2]
    line_len = np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
    #line_len = np.sqrt((x_nearest[0]-x_rand[0])**2 + (x_nearest[1]-x_rand[1])**2 + (x_nearest[2]-x_rand[2])**2 + (x_nearest[3]-x_rand[3])**2 + (x_nearest[4]-x_rand[4])**2 +(x_nearest[5]-x_rand[5])**2 + (x_nearest[6]-x_rand[6])**2)
    if line_len < d:
        factor = 1
    else:
        factor = d / line_len

    normal_rand = (x2 - x1, y2 - y1, z2 - z1)
    x,y,z = factor*normal_rand[0] + x1, factor*normal_rand[1] + y1, factor*normal_rand[2] + z1
    #normal_rand = (x_rand[0] - x_nearest[0], x_rand[1] - x_nearest[1], x_rand[2] - x_nearest[2], x_rand[3] - x_nearest[3], x_rand[4] - x_nearest[4], x_rand[5] - x_nearest[5], x_rand[6] - x_nearest[6])
   # x_new = (factor*normal_rand[0] + x_nearest[0], factor*normal_rand[1] + x_nearest[1], factor*normal_rand[2] + x_nearest[2], factor*normal_rand[3] + x_nearest[3], factor*normal_rand[4] + x_nearest[4], factor*normal_rand[5] + x_nearest[5], factor*normal_rand[6] + x_nearest[6])
    #pnew = Point(x,y,z)
    return ((x,y,z), orientation)

def random_pose():
    minx = 0
    maxx = 2
    miny = -1
    maxy = 1
    minz = -1
    maxz = 1
    mintheta = -np.pi 
    maxtheta = np.pi 

    x = np.random.uniform(minx, maxx)
    y = np.random.uniform(miny, maxy)
    z = np.random.uniform(minz, maxz)
    theta1 = np.random.uniform(mintheta, maxtheta)
    theta2 = np.random.uniform(mintheta, maxtheta)
    theta3 = np.random.uniform(mintheta, maxtheta)

    euler = Euler(theta1, theta2, theta3)
    quat = quat_from_euler(euler)
    point = Point(x, y, z)

    p = Pose(point, euler)
    return (tuple(p[0]), tuple(p[1]))

def is_close(pose, goal, tol=0.1):
    """ 
    Compare euclidean distance of a pose to the goal. 
    is_pose_close only checks that each value is within a distance
    """
    x1,y1,z1 = pose[0]
    x2,y2,z2 = goal[0]
    dist = np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)
    if dist <= tol:
        return True 
    else:
        return False

def get_goal_pose(activity, world):
    if activity == 'open-drawer':   # goal destination -- drawer
        surface_name = 'indigo_drawer_top'
        surface_aabb = compute_surface_aabb(world, surface_name)   
    
    elif activity == 'pickup-spam': # goal destination -- spam
        entity_name = 'potted_meat_can1'
        body = world.get_body(entity_name)
        aabb = get_aabb(body)
        pos = get_aabb_center(aabb)
        rot = get_euler(body)
        pose = Pose(pos, rot)
    
    elif activity == 'place-spam':  # goal destination -- drawer
        entity_name = 'potted_meat_can1'
        surface_name = 'indigo_drawer_top'
        pose = pose2d_on_surface(world, entity_name, surface_name)
   
    elif activity == 'pickup-sugar':    # goal destination -- stovetop
        entity_name = 'sugar_box0'
        body = world.get_body(entity_name)
        aabb = get_aabb(body)
        pos = get_aabb_center(aabb)
        rot = get_euler(body)
        pose = Pose(pos, rot)
    
    elif activity == 'place-sugar': # goal destination -- counter
        entity_name = 'sugar_box0'
        surface_name = 'indigo_tmp'
        pose = pose2d_on_surface(world, entity_name, surface_name)
    
    elif activity == 'close-drawer':    # goal destination -- drawer
        surface_name = 'indigo_drawer_top'
        surface_aabb = compute_surface_aabb(world, surface_name)
    
    else: 
        print('Failure! No activity found.')
        world.destroy()
    
    return pose


def test_motion_planner():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())
    #print('pose', random_pose())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))


    print('Beginning RRT')
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    #print('tool link', tool_link)
    joints = get_movable_joints(world.robot)
    #print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    #print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    ##print('IK Joints', get_ik_joints(world.robot, PANDA_INFO, tool_link))
    #print('gripper Joints', [get_joint_name(world.robot, joint) for joint in world.gripper_joints])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)

    start_pose = get_link_pose(world.robot, tool_link)
    #print('Start pose', type(start_pose))
    #print('gripper Joints', [get_joint_name(world.robot, joint) for joint in world.gripper_joints])
    
    for i in range(100):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        set_joint_positions(world.robot, world.base_joints, goal_pos)
    wait_for_user()

    activity = 'pickup-spam'
    goal_pose = get_goal_pose(activity, world)
    print('goal_pose:', goal_pose)

    path = rrt(start_pose, random_pose, goal_pose)
    current_pose = start_pose
    for p in path:
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        end_pose = p
        #conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        #wait_for_user()
        #print('current_pose', current_pose[0])
        #print('end_pose', end_pose[0])
        for pose in interpolate_poses(current_pose, end_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                #wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
        current_pose = end_pose
    wait_for_user()
    print('FINAL POSE', get_link_pose(world.robot, tool_link))


        




        #for pose in interpolate_poses(current_pose, end_pose, pos_step_size=0.01):
        #    conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
         #   if conf is None:
          #      print('Failure!')
           #     wait_for_user()
            #    break
           # set_joint_positions(world.robot, ik_joints, conf)
        #current_pose = end_pose



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
    #motion_planner_main()
    test_motion_planner()
    # ff = find_ff_heuristic(parser.state, parser.predicates, parser.actions, parser.positive_goals, parser.negative_goals)
    # print(ff)
    # HC =  EFHC(parser.state, parser.predicates, parser.actions, parser.positive_goals, parser.negative_goals)
    # print(HC)
    return 

if __name__ == "__main__":
    main()