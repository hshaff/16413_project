# 16413_project README
PADM Project Section 1 -- Activity Planning

Assumptions: 
In the process of designing the problem, we made the following assumptions about the environment.
1) There is one drawer that can be opened
2) There is one countertop that objects can be placed on
3) There is one stovetop that objects can be placed on
4) The arm of the robot can only hold one object at a time (meaning the robot cannot be holding an object while opening the drawer, for example)
5) The drawer begins closed and empty
6) The only objects in the environment that can be picked up and placed down are the sugar box and the spam box

In the following phases of the project, the solution can be made more robust by including additional drawers, countertops, and stovetops if the above assumptions are incorrect. For simplicity when designing the problem, we assumed that the only objects that can be picked up or placed were the sugar and the spam. This can also be modified by adding additional predicates if this assumption is incorrect.

Files: 
    blockworld.pddl -- The PDDL domain script
    pb1.pddl --  The PDDL problem script
    main.py -- The main run script
    graph.py -- The script that defines the SearchNode and Path classes

Key Functions: 
    main() -- Runs the PDDL parser and calls the planner function to generate an activity plan
    generate_graph() -- Generates the graph from the initial state to the goal state
    BFS() -- Implements the BFS search algorithm and returns an instance of the Path class defining the path from the initial state to the goal state
    check_goal() -- Checks the set of true predicates at the current state and returns true if the goal conditions are satisfied
    can_perform_action() -- Checks if the action can be performed based on the set of preconditions that are true at a state
    find_ff_heuristic() -- Generates the relaxed plan graph to find the FF heuristic
    solve() -- Function to solve the planning problem using Enforced Hill Climbing (EHC)
    SearchNode class -- Each instance of the class stores the state, parent, and action taken to get to the state
    Path class -- Each instance returns a list of the actions taken to get from the initial state to the goal state

    Actions:
        pickup-sugar -- the arm picks up the sugar box if the arm is not already holding an object and the sugar is on the       stovetop
        place-sugar -- the arm places the sugar on the countertop if the arm is holding an object, the sugar is no longer on the stovetop, and the countertop is clear
        pickup-spam -- the arm picks up the spam box if the arm is not already holding an object and the spam is on the countertop
        place-spam -- the arm places the spam in the drawer if the arm is holding an object, the drawer has been opened, and the spam is no longer on the countertop
        open-drawer -- the arm opens the drawer if the drawer is closed and the arm is not already holding an object
        close-drawer -- the arm closes the drawer if the drawer is open and the arm is not already holding an object

Solver:
For the activity planning solver, we planned to implement both BFS and Enforced Hill Climbing (EHC) with the Fast-Forward (FF) heuristic. We wanted to first implement BFS to make sure that we were able to generate a graph from our parsed PDDL files and that the predicates and actions we defined were sufficient to solve the problem. We defined each state in the graph as the set of positive predicates, ie. the predicates that are not false while in the state. Each node in the graph is stored as an instance of the SearchNode class. For each possible action, defined in the domain file, the function checks if the action is possible based on the set of preconditions. If an action can be performed and the state has not yet been visited, the node is added to the queue to be searched. When the goal state is popped from the queue, an instance of the Path class is returned, which returns the optimal path from the initial state to the goal.

Then we began implementing EHC using the FF heuristic. In order to determine the FF heuristic, we need to generate a relaxed plan graph. A relaxed plan graph looks at the set of preconditions that are true in the initial state, checks which actions can be applied based on the satisfied preconditions, and updates the set of facts based on the effects of those actions. The relaxed plan graph relaxes the problem constraints by ignoring delete effects, therefore it is typically a good heuristic since it underestimates the number of actions that must be taken to reach the goal. After determining the FF heuristic, we can implement an EHC search, which greedily takes actions toward the goal and never backtracks. 

However, while trying to generate the relaxed plan graph to get the heuristic value, we realized that the way we designed the predicates makes it such that the goal state is unreachable from some initial actions. For example, for simplicity we decided that the sugar can only be moved from the stovetop to the countertop. Therefore, the FF heuristic will never reach the goal state if the first action it takes is pick-up-sugar. This could be fixed by making the predicates more robust to any scenario that we might see in the environment. 

Challenges:
The main challenges that we faced throughout this first section of the project stemmed from the design decisions that we had to make while formulating the domain and problem files. The first challenge was defining the predicates. Too many predicates seems to make the problem more complicated than necessary. However, we wanted to make sure that we were defining enough predicates to properly constrain the problem and make sure the solution is complete. Another challenge we faced was overconstraining when defining the preconditions for each action. For example, logically we know that we want the drawer to have been previously opened before we pick up the spam box. However, the way we designed the problem, the preconditions for picking up the spam box should only be that the arm is empty and the spam box is on the countertop. The solver will do the rest of the planning for us if we design the constraints (preconditions and effects) correctly. 

When defining the predicates, we focused on making sure the solution was complete rather than thinking about the positive and negative preconditions at each state. When beginning to implement the EHC with FF heuristic planner, in order to determine the heuristic, we began forming a relaxed planning graph. For some states, there were only negative preconditions, which does not add to the set of facts in the relaxed plan graph. We had to change the way we parsed the preconditions to include both positive and negative preconditions to get around this issue. A lesson learned would be to plan with the goal in mind. We should have first decided what planner we were going to use, then decided what we needed for that planner, and worked our way backward to determine how to define the preconditions in order to avoid this challenge in the future.

PADM Project Section 2 -- Motion Planning
TODO

PADM Project Section 3 -- Trajectory Optimization
TODO
