# mapc_ros_bridge

 ROS package that includes a proxy ROS node that works as a bridge between the MASSim simulation server and ROS. 
 It converts all simulation perception and creates all required topics from the configuration.
 
 Each agent needs an individual mapc_ros_bridge node instance to connect to the MASSim server. Please see launch files for example configurations and possible configuration parameters.
 
# Topics

The mac_ros_bridge provides a communication bridge per agent in the MAPC scenario.

The most important topics are `/bridge_node_AGENT_NAME/request_action` that is triggered by the MASSim `REQUEST-ACTION` 
on every simulation cycle per agent and `/bridge_node_AGENT_NAME/generic_action` that is subscribed by the 
mac_ros_bridge to receive the action to be executed by the agent. The `request_action` message does also already include most of the
perception.

The following topics are available.

## Simulation:

* `/bridge_node_AGENT_NAME/request_action` : Called before every simulation cycle. Trigger for the decision-making. Also includes perception.
* `/bridge_node_AGENT_NAME/start`: Called when a simulation round is started, here you probably want to setup your agent.
* `/bridge_node_AGENT_NAME/bye`: message when all matches have been run, just before the server closes all sockets
* `/bridge_node_AGENT_NAME/end`: Message after each simulation


## Perception

Specific individual perception topics (same information as in `/bridge_node_AGENT_NAME/request_action`). Feel free to create additional topics if required.

* `/bridge_node_AGENT_NAME/entity`: Local entity perception of the agent
* `/bridge_node_AGENT_NAME/agent`: Message for an individual agent

