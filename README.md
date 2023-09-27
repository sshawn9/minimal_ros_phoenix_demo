# minimal_ros_phoenix_demo
Embodying the resilience of the legendary phoenix, this module ensures that ROS nodes rise from the ashes of an unexpected roscore termination. Designed with robustness in mind, `ros_phoenix` automatically restores and resumes operation of ROS nodes when roscore is rebooted, minimizing downtime and ensuring consistent performance across robotic applications.

Here, we demonstrate a minimal example of a subscriber and publisher. A separate thread is used solely to monitor the working status of roscore and reconstruct related services when necessary.

In ROS 1, roscore serves as the central node providing naming, registration, and discovery services. Once nodes have found each other and established a connection, their communication becomes direct, bypassing roscore. This means that even if roscore fails, nodes that have already connected can still communicate.

Nevertheless, in this scenario, as soon as we detect that roscore has ceased to function, we immediately stop publishing/subscribing. Of course, with simple modifications, you can configure the publisher/subscriber not to halt immediately when roscore goes down but to maintain established connections, reinitiating all services only when roscore restarts.

## Usage
- make sure you have ros installed already.
```bash
git clone https://github.com/sshawn9/minimal_ros_phoenix_demo.git
cd minimal_ros_phoenix_demo
mkdir build
cd build
cmake ..
make

# to run the minimal publisher demo
./ros_phoenix_pub_demo

# to run the minimal subscriber demo
./ros_phoenix_sub_demo

# Feel free to toggle roscore on or off, to see what will happens.
```
