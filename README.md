# RobEthiChor

TODO: Description

<p align="center">
  <img src="docs/ros_implementation.png" alt="robethichor architecture">
</p>

## Download, installation and running
Clone the repository:
```
git clone https://github.com/gianlucafilippone/robethichor.git
```

### Running on Docker
```
cd robethichor
docker build -t robethichor .
docker run robethichor
```

### Running on a local machine (computer/robot)
#### Prerequisites
- Ubuntu 22.04
- ROS 2 (Humble recommended) with development tools
- Rosdep

#### Installation
Robethichor needs to be built using `colcon` in order to run.

First create a folder for the ROS workspace, e.g.:
```
mkdir robethichor_ws
```

Move the downloaded repository into the `src` folder inside the workspace:

```
mv robethichor robethichor_ws/src
```

Install dependencies using Rosdep:
> [!NOTE]
> Rosdep must be installed and initialized before building the ROS package.
> Install Rosdep:
> ```
> apt-get update
> apt-get install python3-rosdep
> ```
>
> Initialize Rosdep:
> ```
> sudo rosdep init
> rosdep update
> ```

```
cd robethichor_ws
apt-get update
rosdep install --from-paths src -y --ignore-src
```

> [!IMPORTANT]
> Before building the package, remember to configure the ethical implications and the disposition activation according to the system requirements. Also, the Mission Controller component must be completed by providing the planning and task execution. Further details [here](#configuration)

Build the package:
```
colcon build
source install/setup.bash
```

#### Running
This repository provides a bash script for starting Robethichor:
```
cd src
chmod +x launch.bash
./launch.bash
```

> [!NOTE]
> By default, Robethichor will run its nodes with namespace `robethichor` and will expose its RESTful endpoints on the port `5000`. Additional configuration may be provided by runnig with parameters:
> ```
> ./launch.bash --namespace <namespace> --port <port>
> ```

As an alternative, Robethichor can be launched using the ROS launchfile:
```
ros2 launch robethichor robethichor_launch.py ns:=<namespace> port:=<port> ethical_implication_file:=<full/path/to/ethical/implication/file.json> disposition_activation_file:=<full/path/to/disposition/activation/file.json> log_output_file:=<full/path/to/log/file>
```

## Connector interface
The Connector node exposes endpoints are exposed to provide user-related informations.

### User Status
Example:

```json
{
  "s1": true,
  "s2": true,
  "s3": true,
  "s4": false,
  "s5": false
}
```

### Ethic profile
Example:

```json
{
    "c1": {
        "d1": 3,
        "d2": 2,
        "d3": 1
},
    "c2": {
        "d1": 2,
        "d2": 4,
        "d4": 2,
    }
}
```

### Goal
```json
{
    "goal": "goal name"
}
```

## Configuration
RobEthiChor needs to be configured to fit the application domain.

The Mission Controller node needs to be configured/implemented/substituted to aptly control the robot.
Also, the model of ethical implications and disposition activations need to be configured as follows.

### Ethical implications
Configured through the `ethical_implications.json` file.

### Disposition activations
Configured through the `disposition_activation.json` file.

## Context management
### Providing context information and updates
```json
{
    "c1": "Context Value 1",
    "c2": "Context Value 2",
    "c3": "Context Value 3"
}
```