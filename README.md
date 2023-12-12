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
    kitchenDomain.pddl -- The PDDL domain script
    pb1.pddl --  The PDDL problem script
    main.py -- The main run script
    graph.py -- The script that defines the SearchNode and Path classes

Key Functions: 
    main() -- Runs the PDDL parser and calls the planner function to generate an activity plan
    find_ff_heuristic() -- Generates the relaxed plan graph to find the FF heuristic, the number of actions until the goal conditions are satisfied
    EFHC() -- Function to solve the planning problem using Enforced Hill Climbing (EHC) with the FF heuristic
    BFS() -- Implements the BFS search algorithm and returns an instance of the Path class defining the path from the initial state to the goal state
    check_goal() -- Checks the set of true predicates at the current state and returns true if the goal conditions are satisfied
    can_perform_action() -- Checks if the action can be performed based on the set of preconditions that are true at a state
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
a. Assumptions
A decision that had to be made while implementing the motion planner was the distance from the object that the arm has to reach in order for it to be close enough to grab the object. We set the position tolerance to 0.1 and the orientation tolerance to pi. This was used to determine when RRT was complete and had found a valid path to the goal region.

One simplification that we made when implementing the motion planner was moving the base such that the drawer and the spam are within reach of the robot arm. We set a maximum number of iterations in our RRT implementation to 1000. If the maximum number of iterations is exceeded and no path has been found to the goal, we assume that the goal region is out of reach of the robot arm and the base is moved toward the goal. RRT is then rerun to find a valid path to the goal.

Originally our RRT implementation was taking quite a long time to run. We added goal biasing to the RRT implementation to bias the RRT solution toward the goal. This showed drastic improvement in RRT runtime.

b. Key Files, Key Functions, Motion Planner
The main script that runs our implementation is main.py. The key functions that have been added to perform motion planning are as follows: 
main_plan() -- The main script that executes the project code. The script creates the environment, generate the activity plan, and then performs the motion planner.
rrt() -- executes the RRT motion planner algorithm
check_goal() -- checks if RRT has finished and found a valid path to the goal
translate_linearly() -- moves the robot base to the starting position where the drawer is within reach of the robot arm
add_ycb() -- adds the ycb object to the environment
pose2d_on_surface() -- sets an object on a surface
add_sugar_box() -- adds the sugar box to the environment
add_spam_box() -- adds the spam box to the environment
get_sample_fn() -- function that samples poses in the range of the limits of the motion of the arm
extract_path() -- extracts the path generated by RRT that gets the arm from the starting pose to a pose in the goal region, near the target object to move or pick up
nearest() -- finds the nearest node in the RRT graph using euclidean distance
steer() -- incorporates dynamics to steer the motion of the arm in the direction of the goal
is_close() -- checks if the node added to the RRT path is close enough to the goal for RRT to return
get_goal_pose() -- returns the goal pose for each activity in the activity plan
perform_actions() -- when RRT has completed and the arm has been moved to the goal location, the function performs the current action
create_world() -- initializes the environment
move_into_position() -- moves the robot into the starting position

We chose to implement RRT for our motion planner. Based on the action to complete, we generate a goal region around an object, we use RRT to randomly sample the space and generate a feasible path to reach the goal region. Once in the goal region, the arm is able to grab the object and continue with the next activity. 

c. Integration Details
In order to use the motion planner to complete the assigned tasks, it is necessary to integrate with the activity planner implementation from the previous section of the project. Our approach first parses the PDDL scripts and calls the activity planner. Then, for each activity in the activity plan, there is a starting position/pose and a goal region. These are then used as the starting node and goal region for our RRT implementation. Once the goal region is reached, the activity is executed and the same process is repeated for every action in the activity plan.

d. Video of Motion Plan Execution
TODO - when the implementation is complete, a GIF of the execution will be added here.

e. Challenges
The first challenge we faced was figuring out how to work with the pybullet library. It took a good amount of time to get familiar with the provided functions and how to use them to control the motion of the robot and the objects in the environment. It was helpful to add print statements and change some code in the minimal_example.py script in the PADM Project repository to gain intuition about how the given environment works. 

The largest challenge was figuring out how to adapt RRT to this kitchen environment problem. We had to figure how to randomly sample the space to move from the arm's starting position toward the goal region. We also had to decide how to determine when we are close enough to an object to claim that we are able to grab the object we are searching for. In addition, we had to figure out how to use the pybullet library to determine if the randomly generated poses were valid motions for the arm to make. 

PADM Project Section 3 -- Trajectory Optimization
a. Key Files, Functions, and the Solver
The function that implements our trajectory optimization solver is traj_opt.py. We implemented the method outlined in Chapter 6 Example 6.7 in Russ Tedrake's Robotic Manipulation textbook (https://manipulation.csail.mit.edu/trajectories.html#section2). The example demonstrates a simple implementation of trajectory optimization with collision avoidance. Our implementation does not do collision avoidance but the implementation could easily be augmented with the additional collision-avoidance constraints if we were to include it in the future.

In our traj_opt.py script, the optimization problem is initialized, constrained, and solved. Then there are two additional functions, bezier() and bezier_path(). The bezier_path function repeatedly calls the bezier function to define a bezier curve for a given duration of time at each timestep, using the solution to our optimization problem as the points used to calculate the bezier curve. This generates a smooth trajectory of joint angles for the robot to execute.

The problem is set up as a pydrake MathematicalProgram and the Solve function in pydrake.solvers was used to solve the optimization problem. 

b. Optimization Problem
The decision variables (x0, x1, x2) are each a vector of 7 elements with continuous domains, representing each of the 7 joint angles. Therefore, there are 7*3 = 21 output values from the trajectory optimization solution.

Our objective function is minimizing the sum of the 2-norms of each adjacent set of sample points in the problem. For example, with three domain variables (x0, x1, x2), the objective functions are as follows: 
- min(np.sqrt((x0-x1).dot(x0-x1)))
- min(np.sqrt((x2-x1).dot(x2-x1)))
This objective function optimizes the trajectory by minimizing the total change in the joint angles between subsequent joint configurations. This would eliminate any motion that is not in the shortest feasible path from the starting pose to the goal pose.

This optimization problem is subject to two constraints. The first constraint is that the first decision variable is equal to the starting pose. The second constraint is that the last decision variable is equal to the final pose. 

If we were to add in the collision avoidance, the third set of constraints would constrain the norm of each of the decision variables such that the obstacle is avoided. However, we have not implemented that constraint.

c. Mathematical Constraint Problem
Objective: $$min \sum_{n=0}^{N-1} \| x_{n+1} - x_{n} \|^2$$
Subject to: $$x_0$$ = initial_configuration 
            $$x_N$$ = final_configuration

d. Challenges
By far, the largest challenge we faced with this section of the project was getting drake and pydrake set up. After attempting all suggested methods of installation, we were unable to get pydrake properly installed on either of our Ubuntu VMs. However, we were able to successfully pip install drake and import pydrake without issues on our MacOS computers. In order to implement and test this section of the project we had to get a bit creative. We selected a motion whose trajectory we wanted to optimize. We took the starting pose and the goal pose for that trajectory and set those as the initial configuration and final configuration in the traj_opt.py script. Then we performed the trajectory optimization as explained in sections b-c above. In order to test that our optimization was performing as expected, we copied the optimal trajectory output poses from the bezier_path() function to a script called traj.py, which has a function called get_opt_traj(). We committed this to our repository and were able to pull it to our Ubuntu VMs to test that the trajectory optimization was working as expected. We executed the consecutive joint angles with the robot arm in the simulation environment and saw that the arm executes what appears to be the optimal trajectory for the motion that we chose to optimize.

e. GIF/Vido 
GIF/video of the robot executing the plan and embedded in the README

f. Comparison to Motion Planner
The largest difference between the sample based motion planner and the trajectory optimization is that there is no longer randomized motion. Every call to the trajectory optimization will produce the same optimal path from the starting pose to the goal pose. In the sample based motion planner, the points in space were randomly sampled and the motions from the starting pose to the goal pose for each activity varied quite a bit. Also, the runtime for the sample based motion planner varied much more than for the trajectory optimization. The sample based motion planner would find a path to the goal but would sometime get lucky with its choice of randomly sampled points and get there quickly, or it could take much longer with poorly sampled points. Adding the goal biasing significantly decreased the runtime, but it still varied a good amount from run to run. The trajectory optimization took the same amount of time to run each time.