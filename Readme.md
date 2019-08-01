# Multi-Agent Programming Contest Workspace

Technische Universität Berlin - DAI-Labor - http://www.dai-labor.de/

Contest homepage: https://multiagentcontest.org/2019/

This is the main workspace for the TUB participation in the Multi-Agent Contest (MAPC) 2019.
It applies the ROS Hybrid Behaviour Planner Framework (RHBP) on top of the ROS (Robot Operating System) framework.

## Setup and Install

### Dependencies

MASSim requires JAVA 11 and Maven.

Maven:
```
sudo apt install maven default-jdk
```

Java 11 in Ubuntu (tested in 16.04):

```bash
sudo add-apt-repository ppa:openjdk-r/ppa
sudo apt-get update
sudo apt install openjdk-11-jdk
```

Maybe you also have to update your default java version with
```bash
sudo update-alternatives --config java
sudo update-alternatives --config javac

```
and configure your JAVA_HOME variable in your ~/.bashrc by adding the following line at the end of the file.

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
```

RHBP requires ROS. Follow instructions for ROS Kinetic here: http://wiki.ros.org/kinetic/Installation/Ubuntu
```
sudo apt install ros-kinetic-desktop python-pip
pip install --user lindypy
```

### Expected directory Structure

The MAPC simulation server (MASSim) from `https://github.com/agentcontest/massim_2019` 

is integrated as a submodule into this workspace.

***Directory/Workspace Structure***

The project structure working with MASSim sources should look like below. In () a short description of the content is provided.

* your workspace
    * mapc_workspace
        * src (ROS source packages)
            * rhbp (the used decison-making and planning framework (git submodule)) https://gitlab.tubit.tu-berlin.de/rhbp/rhbp
            * mapc_ros_bridge (node implementation that works as a proxy between ROS and MASSim. It converts all simulation perception and creates all required topics.)
            * mac_rhbp_example (simple example using RHBP, ROS and mac_ros_bridge) (git submodule)
        * script (useful scripts, you might want to add this directory to your PATH variable)
        * third-party
            * massim (MASSim root directory (git submodule)) https://github.com/agentcontest/massim_2019
                * server
                    * conf (directory with the simulation configs)
    
### Clone and build

1. `git clone --recursive git@gitlab.tubit.tu-berlin.de:aaip-ss19/mapc_workspace.git`
2. `cd mapc_workspace`
3. `catkin_make`
4. Build massim: `cd third-party/massim` and `mvn install`
5. If you want to use the RHBP rqt plugin you have to start rqt once with `rqt --force-discover`

## Execution and further Documentation

*ATTENTION*: If you are executing any ROS commands that are making use of our packages you need to execute once `source devel/setup.bash` on the particular terminal beforehand.

### massim Server

You can start the MASSim server with the following command.

Running directly from the sources in `third-party/massim` with `scripts/start_massim_src.sh`
This requires that you have once executed `mvn install` in the massim sources root.
After you have executed the script you are asked for selecting a server configuration. You can start with the `SampleConfig` but I recommend to use one of our configs such as `SampleConfig2`. To make these configs available please run `scripts/copy_server_confs.sh`. *Attention!* Do not forget to press enter to start the match after the first initialisation stage is finished.

It is also possible to control the server in execution with some simple commands:
https://github.com/agentcontest/massim_2019/blob/master/docs/server.md#commands

### mapc_ros_bridge

For a basic test of your setup execute the MASSim server as descriped above and launch the mapc_ros_bridge with some dummy agents afterwards through the following command.

```bash
roslaunch mapc_ros_bridge mapc_ros_bridge_example.launch 
```

Now, you should be able to see your agents randomly moving in the MASSim monitor application http://localhost:8000/

### Trouble Shooting

#### Empty Git submodules

If you forgot to clone the mapc_workspace with the option `--recursive` you need to initialize the
Git submodules manually with `git submodule update --init --recursive`

#### rqt

If the RHBP plugins are not available in the rqt menu or are somehow behaving strangely it might help to make sure that the correct source for the plugin
is used (or found) and not a different potentially older version.

```
source devel/setup.bash
rqt --force-discover
```

#### Too many threads

Due to the extensive usage of threads by ROS it is possible that you are reaching the OS limits. 

Possible exception:
```
Exception in thread Thread-1:
Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
    self.run()
  File "/usr/lib/python2.7/threading.py", line 754, in run
    self.__target(*self.__args, **self.__kwargs)
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/registration.py", line 275, in start
    self.run()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/registration.py", line 318, in run
    t.start()
  File "/usr/lib/python2.7/threading.py", line 736, in start
    _start_new_thread(self.__bootstrap, ())
error: can't start new thread
```
To increase the static thread limit edit following file
`/etc/systemd/logind.conf`
and add following configuration line `UserTasksMax=100000`
After this a reboot is required.
