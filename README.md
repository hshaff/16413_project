# 16.413 Final Project README
### Hannah Shafferman and Kyle Sonandres
## PADM Project Section 1 - Activity Planning

### Assumptions 
In the process of designing the problem, we made the following assumptions about the environment.
1) There is one drawer that can be opened
2) There is one countertop that objects can be placed on
3) There is one stovetop that objects can be placed on
4) The arm of the robot can only hold one object at a time (meaning the robot cannot be holding an object while opening the drawer, for example)
5) The drawer begins closed and empty
6) The only objects in the environment that can be picked up and placed down are the sugar box and the spam box

In the following phases of the project, the solution can be made more robust by including additional drawers, countertops, and stovetops if the above assumptions are incorrect. For simplicity when designing the problem, we assumed that the only objects that can be picked up or placed were the sugar and the spam. This can also be modified by adding additional predicates if this assumption is incorrect.

### Files  
    kitchenDomain.pddl -- The PDDL domain script  
    pb1.pddl --  The PDDL problem script  
    main.py -- The main run script  
    graph.py -- The script that defines the SearchNode and Path classes  

### Key Functions   
    main() -- Runs the PDDL parser and calls the planner function to generate an activity plan  
    find_ff_heuristic() -- Generates the relaxed plan graph to find the FF heuristic, the number of actions until the goal conditions are satisfied  
    EFHC() -- Function to solve the planning problem using Enforced Hill Climbing (EHC) with the FF heuristic  
    BFS() -- Implements the BFS search algorithm and returns an instance of the Path class defining the path from the initial state to the goal state  
    check_goal() -- Checks the set of true predicates at the current state and returns true if the goal conditions are satisfied  
    can_perform_action() -- Checks if the action can be performed based on the set of preconditions that are true at a state  
    SearchNode class -- Each instance of the class stores the state, parent, and action taken to get to the state  
    Path class -- Each instance returns a list of the actions taken to get from the initial state to the goal state  

### Actions
    pickup-sugar -- the arm picks up the sugar box if the arm is not already holding an object and the sugar is on the stovetop  
    place-sugar -- the arm places the sugar on the countertop if the arm is holding an object, the sugar is no longer on the stovetop, and the countertop is clear  
    pickup-spam -- the arm picks up the spam box if the arm is not already holding an object and the spam is on the countertop  
    place-spam -- the arm places the spam in the drawer if the arm is holding an object, the drawer has been opened, and the spam is no longer on the countertop  
    open-drawer -- the arm opens the drawer if the drawer is closed and the arm is not already holding an object  
    close-drawer -- the arm closes the drawer if the drawer is open and the arm is not already holding an object  

### Solver  
For the activity planning solver, we planned to implement both BFS and Enforced Hill Climbing (EHC) with the Fast-Forward (FF) heuristic. We wanted to first implement BFS to make sure that we were able to generate a graph from our parsed PDDL files and that the predicates and actions we defined were sufficient to solve the problem. We defined each state in the graph as the set of positive predicates, ie. the predicates that are not false while in the state. Each node in the graph is stored as an instance of the SearchNode class. For each possible action, defined in the domain file, the function checks if the action is possible based on the set of preconditions. If an action can be performed and the state has not yet been visited, the node is added to the queue to be searched. When the goal state is popped from the queue, an instance of the Path class is returned, which returns the optimal path from the initial state to the goal.

Then we began implementing EHC using the FF heuristic. In order to determine the FF heuristic, we need to generate a relaxed plan graph. A relaxed plan graph looks at the set of preconditions that are true in the initial state, checks which actions can be applied based on the satisfied preconditions, and updates the set of facts based on the effects of those actions. The relaxed plan graph relaxes the problem constraints by ignoring delete effects, therefore it is typically a good heuristic since it underestimates the number of actions that must be taken to reach the goal. After determining the FF heuristic, we can implement an EHC search, which greedily takes actions toward the goal and never backtracks. 

However, while trying to generate the relaxed plan graph to get the heuristic value, we realized that the way we designed the predicates makes it such that the goal state is unreachable from some initial actions. For example, for simplicity we decided that the sugar can only be moved from the stovetop to the countertop. Therefore, the FF heuristic will never reach the goal state if the first action it takes is pick-up-sugar. This could be fixed by making the predicates more robust to any scenario that we might see in the environment. 

### Challenges  
The main challenges that we faced throughout this first section of the project stemmed from the design decisions that we had to make while formulating the domain and problem files. The first challenge was defining the predicates. Too many predicates seems to make the problem more complicated than necessary. However, we wanted to make sure that we were defining enough predicates to properly constrain the problem and make sure the solution is complete. Another challenge we faced was overconstraining when defining the preconditions for each action. For example, logically we know that we want the drawer to have been previously opened before we pick up the spam box. However, the way we designed the problem, the preconditions for picking up the spam box should only be that the arm is empty and the spam box is on the countertop. The solver will do the rest of the planning for us if we design the constraints (preconditions and effects) correctly. 

When defining the predicates, we focused on making sure the solution was complete rather than thinking about the positive and negative preconditions at each state. When beginning to implement the EHC with FF heuristic planner, in order to determine the heuristic, we began forming a relaxed planning graph. For some states, there were only negative preconditions, which does not add to the set of facts in the relaxed plan graph. We had to change the way we parsed the preconditions to include both positive and negative preconditions to get around this issue. A lesson learned would be to plan with the goal in mind. We should have first decided what planner we were going to use, then decided what we needed for that planner, and worked our way backward to determine how to define the preconditions in order to avoid this challenge in the future.

## PADM Project Section 2 - Motion Planning  
### Assumptions  
A decision that had to be made while implementing the motion planner was the distance from the object that the arm has to reach in order for it to be close enough to grab the object. We set the position tolerance to 0.1 and the orientation tolerance to pi. This was used to determine when RRT was complete and had found a valid path to the goal region.

One simplification that we made when implementing the motion planner was moving the base such that the drawer and the spam are within reach of the robot arm. We set a maximum number of iterations in our RRT implementation to 1000. If the maximum number of iterations is exceeded and no path has been found to the goal, we assume that the goal region is out of reach of the robot arm and the base is moved toward the goal. RRT is then rerun to find a valid path to the goal.

In order to perform the actions 'pickup-sugar' and 'pickup-spam' we found it was easiest to set the location of the object to the same location as the gripper of the robot arm for each iteration of the movement. Our code checks if the previous activity was either 'pickup-sugar' or 'pickup-spam' and if so, it updates the location of either the sugar box or the spam box as the robot arm moves toward the location where the box is being placed. 

Originally our RRT implementation was taking quite a long time to run. We added goal biasing to the RRT implementation to bias the RRT solution toward the goal. This showed drastic improvement in RRT runtime.

### Key Files, Key Functions, Motion Planner  
Main Script:  

    main_plan() -- The main script that executes the project code. The script creates the environment, generates the activity plan, and then performs the motion planner.  
RRT Helper Functions:  

    rrt() -- executes the RRT motion planner algorithm  
    random_pose() -- function that returns a pose generated by randomly sampling an (x, y, z) position and euler angle from the space  
    nearest() -- finds the nearest node in the RRT graph using euclidean distance  
    steer() -- takes a step in the direction of the goal from the nearest node in the graph and outputs a new pose to be added to the graph   
    is_close() -- checks if the node added to the RRT path is close enough to the goal for RRT to return     
    extract_path() -- extracts the path generated by RRT that gets the arm from the starting pose to a pose in the goal region, near the target object to move or pick up  
Motion Planner Helper Functions:  

    get_goal_pose() -- returns the goal pose for each activity in the activity plan  
    perform_actions() -- when RRT has completed and the arm has been moved to the goal location, the function performs the current action  
Environment Helper Functions:  

    create_world() -- initializes the simulation environment  
    add_ycb() -- adds the ycb object to the environment  
    pose2d_on_surface() -- sets an object on a surface  
    add_sugar_box() -- adds the sugar box to the environment  
    add_spam_box() -- adds the spam box to the environment  
    move_into_position() -- moves the robot into the starting position, calls translate_linearly ()  
    translate_linearly() -- moves the robot base to the starting position where the drawer is within reach of the robot arm  

We found many of the built-in pybullet functions quite helpful, specifically the functions that get and set joint positions and handle the kinematics of the robot arm. The World class provided in the project repository along with some of its helper functions were useful in working with the simulation environment.  

We chose to implement RRT for our motion planner. Based on the next action for the robot to complete, we generate a goal pose and RRT randomly samples the space to generate a feasible path to the goal. Once close enough to the goal, the action can be completed and the motion planner continues with the next activity generated by the activity planner. 

### Integration Details  
In order to use the motion planner to complete the assigned tasks, it is necessary to integrate with the activity planner implementation from the previous section of the project. Our approach first parses the PDDL scripts and calls the activity planner. Then, for each activity in the activity plan, there is a starting position/pose and a goal pose. For example, for the action 'pickup-spam', the starting pose is the location of the handle of the drawer from the previous activity 'open-drawer', and the goal pose is the location of the spam box on the countertop. These poses are then used as the starting node and goal region for our RRT implementation. Once RRT returns a valid path from the starting pose to the goal region, the activity is executed and the same process is repeated for the remaining actions in the activity plan.

### Video of Motion Plan Execution  
The video of the robot arm executing the motion plan can be viewed here: https://youtu.be/8CDxfg7Tu9A.  

One simplification we made was starting the robot in a position where the arm is able to reach the first goal pose, which in our plan was the handle of the drawer in order to open it. After the robot completes the 'open-drawer' activity, it continues with the activity plan to pick up the spam box and place it in the drawer. After those two actions are complete, the activity plan tells the robot to plan a path to the location of the sugar box in order to pick it up and move it to the countertop. However, the sugar box is out of reach of the robot arm. In the video of the motion planner execution, it can be seen that the robot pauses for a while after placing the spam box in the drawer. In our RRT implementation, we set a maximum number of iterations for RRT to run before it returns. If RRT returns no solution, we assume that the goal is out of reach of the arm. This long pause is due to the sugar box being out of reach of the robot arm. During the pause, the planner is searching for a path to the goal, however no such path exists so an empty path is returned after RRT reaches the maximum number of iterations. Then, we move the robot base toward the goal, in this case to the left, so that the goal pose is within reach of the robot arm before continuing on with our motion planner. Then RRT is able to find a path to continue on with the activity plan.

### Challenges  
One challenge we faced was learning how to interface with the pybullet library. There was a fair amount of effort that went into familiarizing ourselves with the simulation environment, the provided functions, and how to control the motion of the robot and the objects in the environment. It was helpful to start with the minimal_example.py script in the PADM Project repository and add to that as a starting point to gain intuition about how the environment works. 

The larger challenge we faced was figuring out how to adapt RRT to the kitchen environment problem. We had to define how to randomly sample the space in order to generate a feasible path from the starting position toward the goal region. We also had to define when we are close enough to an object to claim that we are able to grab it. In addition, we had to use the built-in pybullet functions to determine if the randomly generated poses were valid motions for the arm to make. In our RRT implementation in class, we assumed that as long as a randomly sampled point did not have any collisions we could add it to the path. However, we now have to incorporate the dynamics of the arm into our RRT implementation.

## PADM Project Section 3 - Trajectory Optimization  
### Key Files, Functions, and the Solver  
The function that implements our trajectory optimization solver is traj_opt.py. We implemented the method outlined in Chapter 6 Example 6.7 in Russ Tedrake's Robotic Manipulation textbook (https://manipulation.csail.mit.edu/trajectories.html#section2). The example demonstrates a simple implementation of trajectory optimization with collision avoidance. Our implementation does not do collision avoidance but the implementation could easily be augmented with the additional collision-avoidance constraints if we were to include it in the future.

In our traj_opt.py script, the optimization problem is initialized, constrained, and solved. Two additional functions are defined to simulate dynamics, bezier() and bezier_path(). Bezier() defines a generic 3-point bezier curve. Bezier_path() generates a sequence of T joint configuratons by repeatedly calling bezier() using the solution to our optimization problem as control points, where T is a user specified number of time steps. The result is a smooth trajectory of joint angles for the robot to execute. 

The problem is set up as a pydrake MathematicalProgram and the Solve function in pydrake.solvers was used to solve the optimization problem. 

### Optimization Problem  
The decision variables (x0, x1, x2) are each a vector of 7 elements with continuous domains, representing each of the 7 joint angles. Therefore, there are 7*3 = 21 output values from the trajectory optimization solution.

Our objective function is minimizing the sum of the 2-norms of each adjacent set of sample points in the problem. For example, with three domain variables (x0, x1, x2), the objective function is as follows: 
- min(np.sqrt((x0-x1).dot(x0-x1))+ np.sqrt((x2-x1).dot(x2-x1)))
  
This objective function optimizes the trajectory by minimizing the total change in the joint angles between subsequent joint configurations. This would eliminate any motion that is not in the shortest feasible path from the starting pose to the goal pose.  

This optimization problem is subject to two constraints. The first constraint is that the first decision variable is equal to the starting configuration. The second constraint is that the last decision variable is equal to the final configuration. These start and end configurations are extracted from the trajectory generated by RRT. 

If we were to add in the collision avoidance, the third set of constraints would constrain the norm of each of the decision variables such that the obstacle is avoided. However, we have not implemented that constraint.

### Mathematical Constraint Problem  
Objective: $$min \sum_{n=0}^{N-1} \| x_{n+1} - x_{n} \|^2$$
Subject to: $$x_0 = \text{initial configuration} $$
            $$x_N = \text{final configuration}$$

### Challenges  
By far, the largest challenge we faced with this section of the project was getting drake and pydrake set up. After attempting all suggested methods of installation, we were unable to get pydrake properly installed on either of our Ubuntu VMs. However, we were able to successfully pip install drake and import pydrake without issues on our MacOS computers. In order to implement and test this section of the project we had to get a bit creative. We selected a motion whose trajectory we wanted to optimize. We took the starting pose and the goal pose for that trajectory and set those as the initial configuration and final configuration in the traj_opt.py script. Then we performed the trajectory optimization as explained in the sections above. In order to test that our optimization was performing as expected, we copied the optimal trajectory output poses from the bezier_path() function to a script called traj.py, which has a function called get_opt_traj(). We committed this to our repository and were able to pull it to our Ubuntu VMs to test that the trajectory optimization was working as expected. We executed the consecutive joint angles with the robot arm in the simulation environment and saw that the arm executes what appears to be the optimal trajectory for the motion that we chose to optimize.

Properly forming the optimization problem was also a challenge. Earlier iterations of the above implementation with more decision variables led to the solver either producting an error or not finding an optimal solution. In other instances, the solver would find an optimal solution, but the formulation was wrong in the sense that the result was clearly not what we had intended for the solver to produce. Lastly, it was challenging to find a good representation of dynamics. We ultimatly decided to model the trajectory as a bezier curve and seperate it from the optimization routine for simplicity and to not overconstrain the solver. 

### GIF/Video   
The video of the robot arm executing the optimized trajectory can be viewed here: https://youtu.be/-LebyVfHDpc  

We chose to optimize the trajectory that the robot arm must take to complete the action 'close-drawer.' We chose this trajectory because it was the longest path that the robot arm had to take when completing the motion planning task and it was clear that RRT found a suboptimal path from the starting pose to the goal pose for this trajectory. In the video of the robot arm executing this optimal path, the robot arm travels through the table/stovetop to reach the drawer. We are not doing any collision detection/avoidance in our trajectory optimization so this behavior is expected. Since we are giving the optimization problem a starting pose above the surface and a final pose below the surface, the optimal trajectory would be the path that cuts directly through the surface to get from the starting pose to the goal pose.

### Comparison to Motion Planner  
The largest difference between the sample based motion planner and the trajectory optimization is that there is no longer randomized motion. Every call to the trajectory optimization will produce the same optimal path from the starting pose to the goal pose. In the sample based motion planner, the points in space were randomly sampled and the motions from the starting pose to the goal pose for each activity varied quite a bit. Also, the runtime for the sample based motion planner varied much more than for the trajectory optimization. The sample based motion planner would find a path to the goal but would sometime get lucky with its choice of randomly sampled points and get there quickly, or it could take much longer with poorly sampled points. Adding the goal biasing significantly decreased the runtime, but it still varied a good amount from run to run. The trajectory optimization took the same amount of time to run each time.

## Conclusion
In this project, we explored how activity planning, motion planning, and trajectory optimization are used in tandem to create a fully autonomous solution. We began with a PDDL program to define actions and states, later parsing it and performing a search algorithm to generate the activity plan. We then integrated the activity plan with our chosen motion planner, RRT, by choosing goal and start inputs based on the current activity. Lastly, we learned to formulate a smoother, more efficienct trajecotry through trajectory optimization using an initial guess from RRT. We formulated a mathematical program with constraints and objective functions, and called a solver to generate an optimal solution for the trajectory. In the end, concepts from throughout the class were combined to generate a complete solution. In addition, we learned about different tools that we are able to utilize, such as the pybullet library, drake, and the PDDL parser library, to assist in generating our solution.

We learned about different activity planning, motion planning, and trajectory optimization solutions throughout this semester. In this project, we were able to explore how the three work together to create a fully autonomous solution. We started with writing our first PDDL program for the activity planner and learning how to use the PDDL parser to solve the activity plan in python. Then we explored how to integrate the activity plan with our motion planner to determine the starting location and goal location for the RRT implementation. Most recently in class we have been discussing constraint satisfaction programs (CSPs). In order to perform the trajectory optimization, we had to generate a CSP to solve for the optimal trajectory for one of the motions that we performed in the motion planner. This project allowed us to put all of the concepts that we learned in class together to generate a complete solution. In addition, we learned about different tools that we are able to utilize, such as the pybullet library, drake, and the PDDL parser library, to assist in generating our solution.
