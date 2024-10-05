Overview
This ROS 2 package, periodic_tasks, implements a scheduling system that runs three periodic tasks at different frequencies (1 Hz, 0.5 Hz, and 0.2 Hz). The tasks are executed periodically, with their start times, finish times, and response times recorded in real-time. Additionally, the system monitors whether any task misses its deadline and logs any such occurrences.

Key Features
Task Scheduling: Three tasks are executed periodically at different intervals:

Task 1: Executes every 1 second.
Task 2: Executes every 2 seconds.
Task 3: Executes every 5 seconds.
Real-time Logging: For each task, the following information is logged:

Task ID and Job Number
Start and End Times
Response Time (time taken to complete the task)
Deadline Misses (if the response time exceeds the deadline)
Deadline Monitoring: Each task has a pre-set deadline, and if the task exceeds its deadline, a warning is logged indicating a missed deadline.

Dependencies
ROS 2 (Iron release or similar)
rclcpp (ROS 2 C++ client library)

README
ROS 2 Periodic Tasks
Introduction
The periodic_tasks package demonstrates task scheduling in ROS 2 by running three tasks at different intervals. The tasks log their execution times, and the system checks whether any tasks miss their deadlines. This is useful for applications that require real-time task management and monitoring in robotic systems.

Features
Executes three tasks at different rates:
Task 1 runs every 1 second.
Task 2 runs every 2 seconds.
Task 3 runs every 5 seconds.
Logs start time, finish time, response time, and missed deadlines.
Monitors and warns about deadline violations.
Dependencies
Before using the package, ensure the following dependencies are met:

ROS 2 (Iron release or a compatible version)
rclcpp (C++ client library for ROS 2)
Installation
Follow these steps to install and build the package in your ROS 2 workspace:

Clone the Repository
Navigate to your ROS 2 workspace's src folder (e.g., ~/ros2_ws/src), and clone the repository:

bash
git clone <repository_url>
Build the Package
Navigate back to the root of your ROS 2 workspace and use colcon to build the package:

bash
cd ~/ros2_ws
colcon build --packages-select periodic_tasks
Source the Workspace
After building the package, source the workspace setup script to update your environment:

bash
source install/setup.bash
Running the Node
Once the package is built and sourced, you can run the periodic_tasks node using the following command:

bash
Copy code
ros2 run periodic_tasks periodic_tasks_node
Code Explanation
The PeriodicTaskExecutor class, defined in the periodic_tasks_node node, sets up three periodic tasks using timers that run at different frequencies. Each task has a unique ID and deadline. When the task is executed, the response time is calculated by measuring the time between task release and task completion.

Main Components
Task 1: Runs every 1 second with a 1-second deadline.
Task 2: Runs every 2 seconds with a 2-second deadline.
Task 3: Runs every 5 seconds with a 5-second deadline.
For each task, the node logs:

Task ID and Job Number (a counter of how many times the task has run)
Release time, Completion time, and Response time (in seconds)
A warning message if a task misses its deadline
Logging Function
The node records the start, finish, and response time for each task in the log using RCLCPP_INFO. It also checks if the task exceeded its deadline and logs a warning if necessary using RCLCPP_WARN.

Main Loop
The program runs continuously using rclcpp::spin, allowing the periodic tasks to execute as scheduled.

Future Enhancements
Add more tasks with varying priorities.
Implement priority-based scheduling.
Integrate dynamic task addition/removal during runtime.
