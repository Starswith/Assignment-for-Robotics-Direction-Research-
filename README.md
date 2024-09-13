# Assignment-for-Robotics-Direction-Research
A test with questions by the teacher applied to ros2.


## 1. Preparation
### 1.1 Create a new workspace
First create a new workspace,and give it a name. (eg.called "catkin_ws")
### 1.2 Download "src"
Download the **"src"** package I shared and place it in the workspace you just created.
### 1.3 Setup environment
Go to the file package "catkin_ws", open the terminal in the file directory and create a workspace with colcon.
```
cd catkin_ws
colcon build
```
At this point the prep work is complete and the workspace is ready, then the programme can be run.

## 2. Running code
### 2.1 Running "driver_node"
Entering the file package "catkin_ws" to run the terminal.
```
. install/setup.bash   ###Refresh the setup.bash program
ros2 run driver driver_node   ###Run the "driver_node" program in the "driver" directory
```
### 2.2 Running "process_node"
Still entering the file package "catkin_ws" to run the terminal.
```
. install/setup.bash   ###Refresh the setup.bash program
ros2 run process process_node   ###Run the "process_node" program in the "process" directory
```

## 3. Observe the results of running the programme
### 3.1 Find topics
You have to make sure that the two programmes are running when you do this.You can use "ctrl+alt+t" to run the terminal.
```
ros2 topic list
```
If the topic is posted properly, you can see two topics including **"/targrt"** and **"/command"**.
### 3.2 Running rqt_plot
If you want to observe an image of the programme running on the X/Y/Z axes, you can use the rqt software to draw a viewable image.
```
rqt_plot/target/target.x   ###Output image of target.x
```
If the program is running correctly you will see the visualisation image generating a change in coordinates about the x. Similarly, using this command you can see images for other coordinates.
```
rqt_plot/target/target.y
rqt_plot/target/target.z
rqt_plot/command/command.x
rqt_plot/command/command.y
rqt_plot/command/command.z
```
You can also add x/y/z topics to the plot at the same time and observe the graphs of all three in one image.The principle is to add each topic separately and then run all of them, so I won't go into too much detail here.

---
If you have any questions, please feel free to comment to me or contact me at my email.([link](mc45219@um.edu.mo))




















