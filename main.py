from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from Graph import SearchNode, Path
import math
import numpy as np 

def generate_relaxed_plan_graph(states, predicates, actions, pos_goal,  neg_goal):
    neg_start_states = set()
    for p in predicates:
        if p not in states:
            neg_start_states.add(p)


    start_node = SearchNode([states, frozenset(neg_start_states)])
    q = [start_node]
    visited = set()
    # while q:
    while q:
        current_node = q.pop(0)
        print('CURR', current_node.state)
        pos_states = current_node.state[0]
        neg_states = current_node.state[1]
        if check_goal((pos_states, neg_states), pos_goal, neg_goal, ff=True):
            return Path(current_node)

        total_add_effects = frozenset(current_node.state[0])
        total_del_effects = frozenset(current_node.state[1])
        for a in actions:
            if can_perform_action(current_node.state, a, ff=True):
                print(a.name, a.del_effects)
                #diff = current_node.state.symmetric_difference(a.del_effects)
                total_add_effects = frozenset.union(total_add_effects, a.add_effects)
                total_del_effects = frozenset.union(total_del_effects, a.del_effects)
                # if next_node.state not in visited:
                #     q.append(next_node)
                #     visited.add(next_state)
        next_state = [total_add_effects, total_del_effects]
        #print('NEXT', next_state)
        next_node = SearchNode(next_state, current_node)
        q.append(next_node)
    return False


def generate_graph(states, actions, predicates):
    num_predicates = len(predicates)
    print('states', states)
    # print('actions', actions)
    # print('predicates', predicates)
    next_state = states
    for a in actions:
        next_state = states
        if can_perform_action(states, a):
            print(a.name, a.del_effects)
            next_state.union(next_state, a.add_effects, a.del_effects)
            print(next_state)

    return

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
                diff = current_node.state.symmetric_difference(a.del_effects)
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


def find_ff_heuristic():
    return 

def solve():
    # Solve the activity planning problem using EHC
    return 

def main():
    plan = ''
    parser = PDDL_Parser()
    domain_filename = 'blockworld.pddl'
    problem_filename = 'pb1.pddl'
    parser.parse_domain(domain_filename)
    parser.parse_problem(problem_filename)

   # G = generate_graph(parser.state, parser.actions, parser.predicates)
    # plan = solve()
    # print(plan)
    B = BFS(parser.state, parser.actions, parser.positive_goals, parser.negative_goals)
    print(B.path)
    relaxed = generate_relaxed_plan_graph(parser.state, parser.predicates, parser.actions, parser.positive_goals, parser.negative_goals)
    print(relaxed.path)
    return 

if __name__ == "__main__":
    main()