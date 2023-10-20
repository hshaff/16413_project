from pddl_parser_root.pddl_parser.planner import Planner
from pddl_parser_root.pddl_parser.PDDL import PDDL_Parser
from graph import SearchNode, Path
import math
import numpy as np 

def generate_graph(states, actions, predicates):
    num_predicates = len(predicates)
    for s in states: 
        node_state = print(np.zeros(num_predicates))
    return

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

    G = generate_graph(parser.state, parser.actions, parser.predicates)
    plan = solve()
    print(plan)
    return 

if __name__ == "__main__":
    main()