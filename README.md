# 🤖 Robotics Basics — Learning ROS2 from Scratch

Welcome to **Robotics Basics**, a personal learning repository that documents my step-by-step journey into **ROS2** and **robotics fundamentals**.

This repo follows along with the [Robotics Basics YouTube Playlist](https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy&si=0nixJGJgkpDcUwBm) and serves as a practical notebook for mastering robotics concepts that power real-world robots.

---

## 🎯 Goal

To build a solid foundation in:
- **ROS2 fundamentals** (Nodes, Topics, Services, Actions, Parameters)
- **Robot communication & architecture**
- **Sensor integration**
- **Basic simulations and controls**

These experiments and learnings will lead to a larger applied project:  
🧭 **Project 1 — ROS2 Autonomous TurtleBot** *(Coming soon in a separate repo)*
🧭 **Project 2 — ROS2 + AI Perception Robot** *(Coming soon in a separate repo)*
🧭 **Project 3 — Voice & LLM-Controlled Robot Assistant** *(Coming soon in a separate repo)*

---

## 🧠 Key Topics Covered

| Concept                  | Description |
|--------------------------|--------------|
| **ROS2 Nodes**           | Independent modules that perform robot tasks |
| **Topics**               | Communication channels between nodes |
| **Services & Actions**   | Request-response and goal-oriented communication |
| **Parameters**           | Configurable runtime variables |
| **Launch Files**         | Automate multi-node setups |
| **Packages**             | Modular structure for robotics projects |

---

## 🚀 Getting Started

### 1️⃣ Clone the Repository
```bash
git clone https://github.com/shimmer0909/Robotics-basics.git
cd Robotics-basics

### 2️⃣ Build in a ROS2 Workspace
mkdir -p ~/ros2_ws/src
cp -r Robotics-basics/* ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install (To get changes in code auto reflected)
source install/setup.bash

### 3️⃣ Run Example Nodes
ros2 run <package_name> <node_name>

## 📁 Terminal Commands

| Concept                  | Command |
|--------------------------|--------------|
| **Topic Graphs**         | rqt_graph |
| **Inbuild Talker Node**  | ros2 run demo_nodes_cpp talker |
| **Inbuild Listener Node**| ros2 run demo_nodes_cpp listener |

### **TOPICS**
| **List Topics**          | ros2 topic list |
| **Get Topic Info**       | ros2 topic info /<topic_name> | ros2 topic info /chatter |
| **Show Interface**       | ros2 interface show <topic publisher type> | ros2 interface show std_msgd/msg/String |
| **Topic Output**         | ros2 topic echo /<topic_name> | ros2 topic echo /chatter
| **Topic Publish Rate**   | ros2 topic hz /<topic_name> | ros2 topic hz /turtle1/pose

### **SERVICES**
| **List Services**        | ros2 service list |
| **Service Type**         | ros2 service type /<service_name> | ros2 service type /add_two_ints |
| **Call a Service**       | ros2 service call /<service_name> <service_type> "{<interface_request>}" | ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{'a': 2, 'b': 5}" |

### **TURTLESIM**
| **Run Node**             | ros2 run turtlesim turtlesim_node |
| **Move Turtle**          | ros2 run turtlesim turtle_teleop_key |

### **MISCELLANEOUS**
| **List All Running Nodes** | ros2 node list |
| **Use Custom ROS2 Node**   | source ros2_ws/install/setup.bash in ~/.bashrc |
| **Create Package in ros2_ws** | cd ros2_ws/src | ros2 pkg create <robot_function_name> --build-type <ament_python/ament_cmake> --dependencies <name> | ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy

## 📁 Possible Error Fixes

| Concept                  | Command |
|--------------------------|--------------|
| **Node name not visible**| source ros2_ws/src/install/setup.bash |
| **[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7471: open_and_lock_file failed**  | rm -rf /dev/shm/fastrtps* |

