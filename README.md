# socialrobot_knowledge

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

The Socialrobot project - knowledge module.

- Maintainer status: maintained
- Maintainers
  - Jaeyun Lee (wodbs9522@gmail.com)
  - Sangui Lee (rmrlrmrl124@gmail.com)
- Author
  - Seokjun Lee (dltjrwns4127@gmail.com)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_knowledge.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
1. [Overview](#overview)
2. [Installation methods](#installation-methods)
   1. [Install manually](#install-manually)
3. [Dependencies](#dependencies)
   1. [Frameworks](#frameworks)
   2. [Third-party libraries](#third-party-libraries)
   3. [Social Robot Project Modules](#social-robot-project-modules)
   4. [Hardware requirements](#hardware-requirements)
4. [Quick start](#quick-start)
5. [Features](#features)
   1. [Example](#example)
6. [Nodes](#nodes)
   1. [{context_manager}](#node1-context_manager)
   2. [{context_listener}](#node2-context_listener)
   3. [{perception_listener}](#node3-perception_listener)
   4. [{context_saver}](#node4-context_saver)

</div>
</div>

---

## Overview

The Socialrobot project - knowledge module.

- context_manager
- rosjava_custom_srv
- rosjava_triple_msgs
- perception_listener

## Installation methods

### Install manually

#### Step 1: Installation

1. Install the ROS. [Instructions for Ubuntu 16.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
   
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3. Install rosjava packages and dependencies
   ```
   sudo apt install openjdk-8-jdk openjfx
   sudo apt-get install ros-kinetic-rosjava
   ```
4. Install SWI-prolog
   
   ```
   #Add the ppa ppa:swi-prolog/stable to your system’s software sources:
   sudo add-apt-repository ppa:swi-prolog/stable
   sudo apt-get update
   sudo apt-get install swi-prolog swi-prolog-java
   ```
   

5. if you want to visualize the ontology
   ```
   sudo apt-get install ros-<rosdistro>-rosbridge-server
   pip install tornado
   pip install selenium
   ```
   
   download geckodriver(version 0.25) from https://github.com/mozilla/geckodriver/releases
   ```
   sudo mv /---path---/geckodriver /usr/local/bin
   ```

#### Step 2: Setup

1. export environment variables to `~/.bashrc`
   ```
   export SWI_HOME_DIR="/usr/lib/swi-prolog"
   export LD_PRELOAD="$LD_PRELOAD:$SWI_HOME_DIR/lib/x86_64-linux/libswipl.so"
   export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SWI_HOME_DIR/bin/x86_64-linux/:$SWI_HOME_DIR/lib/x86_64-linux/"
   ```

#### About issues

- build error about `org.ros.rosjava_messages`: generate rosjava messages.

   ```
   genjava_message_artifacts --verbose -p std_msgs geometry_msgs octomap_msgs moveit_msgs vision_msgs rosjava_custom_srv
   ```

- Prolog version compatibility with jpl.jar (or jpl-<version>.jar)
   
   ```
   cp /usr/lib/swi-prolog/lib/jpl.jar socialrobot_knowledge/context_manager/context_manager/lib/
   ```

- multiple executable same files

   add the below to socialrobot_knowledge/context_manager/context_manager/build.gradle
   ```
   task cleanScripts(dependsOn: 'installDist') {
   doLast {
      file('build/scripts').deleteDir()
   }
   }

   installDist.finalizedBy(cleanScripts)
   ```

## Dependencies

### Frameworks

- ROS Kinetic/Melodic
  - rosbridge_server
  - rosjava

### Third-party libraries

- SWI-prolog
- geckodriver (for visualizing the ontology)
- tornado (for visualizing the ontology)
- selenium (for visualizing the ontology)

### Social Robot Project Modules

- None

### Hardware requirements

This package does not require any hardware device.

## Quick start 

From `commands` file:

```sh
rosrun context_manager context_manager org.ros.rosjava_context_manager.ContextManager
rosrun context_manager context_manager org.ros.rosjava_context_manager.ContextListener
    #type 'social_robot' or 'skku_robot' on the ContextListener's console for init
rosrun perception_listener perception_listener.py 
rosrun context_manager context_manager org.ros.rosjava_context_manager.ContextSaver

#for visualizing ontology
roslaunch rosbridge_server rosbridge_websocket.launch 
rosrun pywebtest talker.py 
```

## Features

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

### Example

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

## Nodes

### {context_manager}

<div style="padding-left:40px;">

#### Published Topics

- context_manager/monitor/provision_for_tm ([rosjava_custom_srv/MonitorServiceResponse]
  - Query result on topic.
- context_manager/monitor/service ([rosjava_custom_srv/MonitorSimilarService]
  - Query result on service.

#### Messages

- {QueryServiceRequest}.msg
  - query (`string`)
    - Service name to request to the manager.
  - manager (`string`)
    - Manager to request service.
	
- {QueryServiceResponse}.msg
  - result (`string`)
    - Service request result.
	
- {MainServiceRequest}.msg
  - predicate (`string list`)
    - Predicate to query.
  - param1 (`string list`)
    - Parameter to query.
  - param2 (`string list`)
    - Parameter to query.
  - param3 (`string list`)
    - Parameter to query.
  - param4 (`string list`)
    - Parameter to query.
  - status (`int32 list`)
    - Status to query.
  - manager (`string list`)
    - Manager to query.
	
- {Monitor}.msg
  - predicate (`string`)
    - Predicate to response.
  - param1 (`string`)
    - Parameter to response.
  - param2 (`string`)
    - Parameter to response.
  - param3 (`string`)
    - Parameter to response.
  - param4 (`string`)
    - Parameter to response.
	
- {MonitorServiceRequest}.msg
  - predicate (`string`)
    - Predicate to query.
  - param1 (`string`)
    - Parameter to query.
  - param2 (`string`)
    - Parameter to query.
  - param3 (`string`)
    - Parameter to query.
  - param4 (`string`)
    - Parameter to query.
  - status (`int32`)
    - Status to query.
  - manager (`string`)
    - Manager to query.
	
- {MonitorServiceResponse}.msg
  - predicate (`string`)
    - Predicate to response.
  - param1 (`string`)
    - Parameter to response.
  - param2 (`string`)
    - Parameter to response.
  - param3 (`string`)
    - Parameter to response.
  - param4 (`string`)
    - Parameter to response.
  - status (`int32`)
    - Status to response.
  - manager (`string`)
    - Manager to response.

#### Services

- {context_manager/monitor/service} (rosjava_custom_srv/MonitorSimilarService.srv)
  - Service that responds to queries received from Task Manager.

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- predicate (`string`)
  - Predicate to query.
- param1 (`string`)
  - Parameter to query.
- param2 (`string`)
  - Parameter to query.
- param3 (`string`)
  - Parameter to query.
- param4 (`string`)
  - Parameter to query.
- status (`int32`)
  - Status to query.
- manager (`string`)
  - Manager to query.

</div>
<div style="flex:50%; padding-left:10px;">

Response

- response (`MonitorServiceResponse[]`)
  - Response list.
  
</div>
</div>

</div>

### {context_listener}

<div style="padding-left:40px;">

#### Subscribed Topics

	
</div>

### {perception_listener}

<div style="padding-left:40px;">

#### Subscribed Topics

- /objects_infos ([std_msgs/Float32MultiArray])
  - current perception of objects.
- /visual_robot_perceptions ([std_msgs/Float32MultiArray])
  - Position of mobile robot.

</div>

</div>

### {context_saver}

<div style="padding-left:40px;">

</div>

</div>

---

- [[Go to the Social Robot Project Main]][SRP_main]
