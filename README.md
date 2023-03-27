# Comp0129-CW1-team3

This working folder contains team 3's submission to Module COMP0129 coursework 1. 

# Contributor

- Jingyuan Wei (jeffery.wei.19@ucl.ac.uk)

- Xiang Long (xiang.long.22@ucl.ac.uk)
- Zuxun Wang (zuxun.wang.22@ucl.ac.uk)

# Licenses
This project is licensed under the terms of the MIT license.

# Work Distribution
| Name | Time Spent | Contribution (Percentage) |
|:--------:|:--------:|:--------:|
| Jingyuan Wei | 48 hours | 33.34% |
| Xiang Long | 48 hours | 33.34% |
| Zuxun Wang | 48 hours | 33.34% |

Coursework tasks are divided and distributed equally to each team member. Every team member contributes equally to the coursework.

We work as a group for each task to implement the code and communicate via Teams, but mainly one man writing while others give ideas and detect bugs on the fly. After that, we have the rest two members do either code reviewing or testing.

# Setup
1. Download the submission **cw1_team_3** folder from Moodle COMP0129 coursework 1 tab under

2. Move the folder into the working directory under the **comp0129_s23_robot/src** directory. 

    ```bash
    ~/comp0129_s23_robot/src/cw1_team_3
    ```

    

3. The compulsory packages we used in this coursework are listed in the **CMakeLists.txt** file. Those essential packages are:
    - roscpp
    - std_msgs
    - geometry_msgs
    - sensor_msgs
    - moveit_ros_planning
    - moveit_ros_planning_interface
    - tf2
    - tf2_ros
    - pcl_ros
    - pcl_conversions

4. Now you can run the coursework with:

    ```bash
    cd ~/comp0129_s23_robot
    catkin build
    source devel/setup.bash
    roslaunch cw1_team3 run_solution.launch
    ```

5. Before running a task. Open a separate terminal:

	```bash
	source devel/setup.bash
	```

# Run Tasks

To begin task 1, call:

```bash
rosservice call /tasl 1
```

To begin task 2, call:

```bash
rosservice call /tasl 2
```

To begin task 3, call:

```bash
rosservice call /tasl 3
```

The script will run and output the results to the initial terminal.

For task 3, we make each basket have a capacity of 3, meaning that a basket can only contain 3 boxes.
 
There could be a scenario in which a box collides with other boxes, leading to the other boxes not being put into the basket. In that case, we could try to scan the world again after finishing each round to find those collided boxes until all boxes can find their corresponding basket.

If you encounter any errors, refer to the README file.



