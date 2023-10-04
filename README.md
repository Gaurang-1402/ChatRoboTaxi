# ChatRoboTaxi: Control Mobile Robot with Natural Language

ChatRoboTaxi is a project that merges Language Logic Models (LLMs) with the control of self driving cars. With it, users can operate self driving cars using simple, natural language commands. The project is built using ROS2 (Robot Operating System) Foxy, and its simulations run within the Gazebo environment, providing a realistic platform for self driving car behavior.

Accompanying ChatRoboTaxi is a user-centric web application that presents users with a straightforward interface to input their commands. This melding of advanced technology and a user-friendly design creates an accessible and intuitive experience for controlling self driving cars.

While ChatRoboTaxi currently functions within a simulated environment for thorough testing and development, it is designed with real-world applications in mind. The system can translate natural language instructions into actual robotic commands when required.


Key features include:
- A user-friendly web application that provides an interactive interface for self driving car control
[![output-onlinegiftools-11-1.gif](https://i.postimg.cc/CxY3yS7g/output-onlinegiftools-11-1.gif)](https://postimg.cc/nCRRB8NR)
- Full control over the self driving car's directional movement, including forward, backward, left, and right commands
[![output-onlinegiftools-12-1.gif](https://i.postimg.cc/wM8WhQbp/output-onlinegiftools-12-1.gif)](https://postimg.cc/XGLcWdbH)
- Support for commands in multiple languages
[![output-onlinegiftools-13-1.gif](https://i.postimg.cc/Dw9bfpWn/output-onlinegiftools-13-1.gif)](https://postimg.cc/87bCtHB0)

## ROSGPT Architecture

![Screenshot from 2023-07-04 20-49-50](https://github.com/Gaurang-1402/ChatDrones/assets/71042887/f3534fd5-1ac8-4d55-8e67-fb5f6c0ddf8d)

1. The first component, "rosgpt.py", serves as the primary translator. As a REST server in a ROS2 node, it receives instructions in the form of POST requests, then processes these instructions into structured JSON commands using the ChatGPT API. Once the translation is complete, the commands are published on the /voice_cmd topic, ready for the next stage.

2. The next component is "rosgpt_client_node.py", a ROS2 client node that acts as a liaison between the user and rosgpt.py. It sends POST requests with the user's commands to the ROSGPT REST server and awaits the transformed JSON commands, displaying them upon receipt.

3. Another key component is "rosgpt_client.py", which fulfills a similar role to rosgpt_client_node.py. The main difference is that this script functions solely as a REST client for ROSGPT, without the ROS2 node involvement.

4. Once the commands are translated, they are received by "rosgptparser_robotaxi.py". This script, dubbed the ROSGPTParser, executes the commands. It subscribes to the /voice_cmd topic, receives the JSON commands, parses them, and then carries out the necessary self driving car steering.

## Getting started

Clone the repository

```
mkdir ros_ws
cd ros_ws
git clone <repo_url>
```

Install the `virtualenv` package:

```
python3 -m pip install --user virtualenv

```


Create and activate a virtual environment:

```
python3 -m virtualenv ROS2SDC_VENV
source ROS2SDC_VENV/bin/activate
```


Install the necessary Python modules:

```
pip3 install -r Repo_resources/installation_requirements_python.txt
```
rosgptparser_robotaxi
```
mkdir -p ~/.gazebo/models
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y ros-foxy-gazebo-ros-pkgs

```


## Building the Project

Copy models to your `.gazebo/models` directory:

```
cp -r self_driving_car_pkg/models/* ~/.gazebo/models
```


Notify Colcon to ignore `ROS2SDC_VENV` during the build:

```
touch ROS2SDC_VENV/COLCON_IGNORE
```


Build the project:

```
colcon build
```


## Running the Project

Open a new terminal:

```
source ROS2SDC_VENV/bin/activate
source install/setup.sh
ros2 launch self_driving_car_pkg world_gazebo.launch.py


```


## Installing Additional Dependencies

Prepare the Gazebo models directory and install the required packages:


Install rosgpt libraries from the rosgpt folder

```
cd ~/src/rosgpt
pip3 install -r requirements.txt
```

Install ROS requirements

```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

```
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=<rosdistro> -y
```


Add your OpenAI API Key in your ```.bashrc``` as an environment variable 

```
echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```


## Running ROSGPT

First, navigate to the root directory of your workspace and build the project

```
cd ~/ros_ws
colcon build --symlink-install
```
Now run each of these commands on new terminals

```
source ROS2SDC_VENV/bin/activate
source install/setup.sh
ros2 run rosgpt rosgpt

```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```

```
source ROS2SDC_VENV/bin/activate
source install/setup.sh
ros2 run rosgpt rosgptparser_robotaxi

```


Note: Please replace `<repository_url>` and `<your_api_key>` with the actual repository URL and your OpenAI API key, respectively.



## Credits
Simulation adapted from: https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
I am deeply appreciative of these individuals for sharing their work to build on top of!
