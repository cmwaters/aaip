#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import rospy

import traceback

from mapc_ros_bridge.msg import RequestAction, GenericAction, Agent,  Bye, SimStart, SimEnd, Goal, Entity, Team, \
    Position, Obstacle, Task, Requirement, AgentMsg, EntityMsg, Dispenser, Block

import socket
import threading
import time
import errno

import json
import yaml

import collections


class MapcMessageType(object):
    """
    MAPC protocol message types
    """
    AUTH_REQ = "auth-request"
    AUTH_RES = "auth-response"
    SIM_START = "sim-start"
    SIM_END = "sim-end"
    REQ_ACTION = "request-action"
    RES_ACTION = "action"
    BYE = "bye"


class MapcMessage(object):
    """
    Container class for simplified MAPC protocol JSON conversion.
    """

    def __init__(self, type,  **kwargs):
        self.type = type
        self.content = kwargs

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)


class MapcRosBridge(threading.Thread):
    """
    Proxy server that converts the communication with the Multi-Agent Programming Contest Server 2017 to ROS topics
    """

    SOCKET_TIMEOUT = 2  # general socket timeout
    RETRY_DELAY = 1.0
    RECV_SIZE = 8192
    SEPARATOR = b'\0'

    AGENT_DEFAULT_NAME = 'agentA1'  # TODO adjust this if you want to debug a particular agent from PyCharm

    def __init__(self, name=None):
        """
        :param name: agent name, leave empty to get name property from roslaunch
        """
        super(MapcRosBridge, self).__init__()
        rospy.logdebug("MapcRosBridge::init")

        self._socket = None

        node_name = 'bridge_node_'

        if name:
            node_name += name
        else:
            node_name += self.AGENT_DEFAULT_NAME

        rospy.init_node(node_name, anonymous=False, log_level=rospy.INFO)

        if not name:
            self._agent_name = rospy.get_param('~agent_name', self.AGENT_DEFAULT_NAME)
        else:
            self._agent_name = rospy.get_param('~agent_name', name)

        server_ip = rospy.get_param('~server_ip', 'localhost')
        server_port = rospy.get_param('~server_port', 12300)
        self._server_address = (server_ip, server_port)

        self._agent_pw = rospy.get_param('~password', '1')

        rospy.logdebug("Server: %s Port: %d Agent: %s Password: %s", server_ip, server_port, self._agent_name,
                       self._agent_pw)

        self.message_id = -1

        # this is all perception that the agent is going to publish

        self._pub_request_action = rospy.Publisher('~request_action', RequestAction, queue_size=1)
        self._pub_sim_start = rospy.Publisher('~start', SimStart, queue_size=1, latch=True)
        self._pub_sim_end = rospy.Publisher('~end', SimEnd, queue_size=1, latch=False)
        self._pub_bye = rospy.Publisher('~bye', Bye, queue_size=1, latch=True)

        #specific publishers TODO extend if necessary with additional specific publishers
        self._pub_agent = rospy.Publisher('~agent', AgentMsg, queue_size=1, latch=True)
        self._pub_entity = rospy.Publisher('~entity', EntityMsg, queue_size=1, latch=True)

        # this is the expected action reply
        rospy.Subscriber("~generic_action", GenericAction, self.callback_generic_action)

    def connect(self):
        """
        connect to contest server
        :return: bool True for success
        """
        try:
            rospy.logdebug("Connecting...%s", self._agent_name)
            self._socket = socket.create_connection(self._server_address, MapcRosBridge.RETRY_DELAY)
            self._socket.settimeout(None)  # enable blocking mode until simulation starts
            return True
        except OSError as error:
            rospy.logerr('OSError connecting to {}: {}'.format(self._server_address, error))
            return False
        except socket.timeout:
            rospy.logerr('socket.timeout: Error connecting to {}: {}'.format(self._server_address, "timeout"))
            return False
        except socket.error as e:
            rospy.logerr('socket.error: Error connecting to {}: {}'.format(self._server_address, e))
            return False

    def reconnect(self):
        """
        Reconnect to contest server
        """
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
            self._socket.close()
        except OSError:
            pass
        time.sleep(MapcRosBridge.RETRY_DELAY)
        rospy.loginfo('Reconnecting...%s', self._agent_name)
        self._socket = None
        while not self.connect():
            time.sleep(MapcRosBridge.RETRY_DELAY)
        self.authenticate()

    def authenticate(self):
        """
        Authenticate on the contest server
        """
        rospy.logdebug("Authenticate...%s", self._agent_name)

        msg = MapcMessage(type=MapcMessageType.AUTH_REQ, user=self._agent_name, pw=self._agent_pw)

        self._socket.send(msg.to_json() + MapcRosBridge.SEPARATOR)

    def handle_message(self, data):
        """
        Handle server messages
        :param data: json message
        """
        try:
            message = yaml.safe_load(data)  # avoids a conversion to unicode strings as json.dumps

            type = message['type']
            content = message['content']
            if type == MapcMessageType.REQ_ACTION:
                self._request_action(message=content)
            elif type == MapcMessageType.SIM_START:
                self._sim_start(message=content)
            elif type == MapcMessageType.AUTH_RES:
                rospy.loginfo("%s: Authentication: %s", self._agent_name, content['result'])
            elif type == MapcMessageType.SIM_END:
                self._sim_end(message=content)
            elif type == MapcMessageType.BYE:
                self._bye(message=content)
        except Exception as e:
            rospy.logerr(traceback.format_exc())

    def _request_action(self, message):
        """
        Handle request action message (parse and publish)
        :param message: message content as dict
        """

        timestamp = long(message.get('time'))
        perception = message['percept']

        self.message_id = message.get('id')
        rospy.logdebug("request-action: perception id = %s", self.message_id)

        msg = RequestAction()
        msg.time = timestamp
        msg.simulation_step = int(message.get('step'))
        msg.deadline = long(message.get('deadline'))

        msg.team = self._parse_team(perception)
        agent = self._parse_agent(message)
        msg.agent = agent
        msg.tasks = self._parse_tasks(perception)
        msg.goals = self._parse_goals(perception)
        msg.obstacles = self._parse_obstacles(perception)
        entities, dispensers, blocks = self._parse_things(perception)
        msg.entities = entities
        msg.dispensers = dispensers
        msg.blocks = blocks

        self._pub_request_action.publish(msg)

        # specific publishers # TODO extend if necessary
        if self._pub_agent.get_num_connections() > 0:
            self._pub_agent.publish(AgentMsg(time=timestamp, agent=agent))

        if self._pub_entity.get_num_connections() > 0:
            self._pub_entity.publish(EntityMsg(time=timestamp, entities=entities))

        # this is for debugging only
        # self._send_action(action_type="move", params={"direction":"e"})

    def _sim_end(self, message):
        """
        Handle sim end message
        :param message: message content as dict
        """
        timestamp = long(message.get('time'))
        ranking = int(message.get('ranking'))
        score = int(message.get('score'))

        rospy.logdebug("ranking= %d", ranking)
        rospy.logdebug("score= %d", score)

        msg = SimEnd()
        msg.ranking = ranking
        msg.score = score
        msg.time = timestamp
        self._pub_sim_end.publish(msg)

    def _sim_start(self, message):
        """
        Handle sim start message
        :param message: message content as dict
        """

        timestamp = long(message.get('time'))

        percept = message.get('percept')

        steps = int(percept.get('steps'))
        rospy.logdebug("sim-start: steps = %s", steps)

        msg = SimStart()
        msg.time = timestamp
        msg.steps = steps
        msg.team = percept.get('team')
        msg.vision_range = int(percept.get('vision'))

        self._pub_sim_start.publish(msg)

    def _bye(self, message):
        """
        Handle bye message
        :param message: message content as dict
        """
        msg = Bye()
        self._pub_bye.publish(msg)
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _send_action(self, action_type, params={}):
        """
        send action reply back to MASSIM server
        :param action_type: action type that should be executed
        :param params: dictionary with optional parameters, depending on the action type
        """
        msg = MapcMessage(type=MapcMessageType.RES_ACTION, id=self.message_id)
        msg.content["type"] = action_type

        # add parameters, keys are not used in the moment but might be useful
        # for debugging, type checks or future extensions
        msg.content["p"] = params.values()

        self._socket.send(msg.to_json() + MapcRosBridge.SEPARATOR)

    def run(self):
        """
        Agent main thread
        """
        rospy.logdebug("MapcRosBridge::run")
        while not self.connect():
            time.sleep(MapcRosBridge.RETRY_DELAY)

        self.authenticate()
        msg_buffer = b''
        while not rospy.is_shutdown():
            try:
                data = self._socket.recv(MapcRosBridge.RECV_SIZE)
            except socket.timeout:
                rospy.logerr("socket timeout")
                self.reconnect()
                msg_buffer = b''
                continue
            except socket.error as e:
                if e.errno == errno.ECONNRESET:
                    # connection closed by server
                    data = b''
                else:
                    rospy.logerr(('Socket error: {}'.format(e)))
                    self.reconnect()
                    msg_buffer = b''
                    continue
            if len(data) == 0:
                rospy.logerr('Connection closed by server')
                self.reconnect()
                msg_buffer = b''
                continue
            else:
                msg_buffer += data
                index = msg_buffer.find(MapcRosBridge.SEPARATOR)
                while index != -1:
                    self.handle_message(msg_buffer[0:index])
                    msg_buffer = msg_buffer[index + 1:]
                    index = msg_buffer.find(MapcRosBridge.SEPARATOR)

    def callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        rospy.logdebug("MapcRosBridge::callback_generic_action %s", msg)

        params = {}
        for key_value in msg.params:
            params[key_value.key] = key_value.value

        params = collections.OrderedDict(sorted(params.items()))

        self._send_action(action_type=msg.action_type, params=params)

    # various scenario specific parsing methods

    def _parse_team(self, perception):

        team = Team()

        team.score = perception.get('score')

        return team

    def _parse_agent(self, message):
        perception = message['percept']

        agent = Agent()

        agent.last_action = perception.get('lastAction')
        agent.last_action_result = perception.get('lastActionResult')
        agent.name = self._agent_name
        agent.last_action_params = perception.get('lastActionParams')

        return agent

    def _parse_tasks(self, perception):

        tasks = []

        tasks_msg = perception.get('tasks')

        for task in tasks_msg:
            requirements = []
            for req in task['requirements']:
                requirements.append(Requirement(pos=Position(req['x'], req['y']), type=req['type'], details=req['details']))
            tasks.append(Task(name=task['name'], reward=task['reward'], deadline=task['deadline'], requirements=requirements))

        return tasks

    def _parse_obstacles(self, perception):

        obstacle = []

        obstacle_msg = perception.get('terrain').get('obstacle')

        if obstacle_msg:
            for g in obstacle_msg:
                obstacle.append(Obstacle(Position(g[0], g[1])))

        return obstacle

    def _parse_goals(self, perception):

        goals = []

        goals_msg = perception.get('terrain').get('goal')

        if goals_msg:
            for g in goals_msg:
                goals.append(Goal(Position(g[0], g[1])))

        return goals

    def _parse_things(self, perception):
        entities = []
        dispensers = []
        blocks = []

        things_msg = perception.get('things')

        for thing in things_msg:
            if thing['type'] == 'entity':
                entities.append(Entity(pos=Position(thing['x'], thing['y'])))
            if thing['type'] == 'dispenser':
                dispensers.append(Dispenser(pos=Position(thing['x'], thing['y']), type=thing['details']))
            if thing['type'] == 'block':
                blocks.append(Block(pos=Position(thing['x'], thing['y']), type=thing['details']))

        return entities, dispensers, blocks


if __name__ == '__main__':
    rospy.logdebug("mac_ros_bridge_node::main")
    try:
        bridge = MapcRosBridge().start()
        # bridge = MapcRosBridge("agentA1").start()
        # bridge = MapcRosBridge("agentA2").start()
        # bridge = MapcRosBridge("agentA3").start()
        # bridge = MapcRosBridge("agentA4").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentA5").start()
        # bridge = MapcRosBridge("agentA6").start()
        # bridge = MapcRosBridge("agentA7").start()
        # bridge = MapcRosBridge("agentA8").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentA9").start()
        # bridge = MapcRosBridge("agentA10").start()
        # bridge = MapcRosBridge("agentA11").start()
        # bridge = MapcRosBridge("agentA12").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentA13").start()
        # bridge = MapcRosBridge("agentA14").start()
        # bridge = MapcRosBridge("agentA15").start()
        # bridge = MapcRosBridge("agentA16").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        #
        # bridge = MapcRosBridge("agentB1").start()
        # bridge = MapcRosBridge("agentB2").start()
        # bridge = MapcRosBridge("agentB3").start()
        # bridge = MapcRosBridge("agentB4").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentB5").start()
        # bridge = MapcRosBridge("agentB6").start()
        # bridge = MapcRosBridge("agentB7").start()
        # bridge = MapcRosBridge("agentB8").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentB9").start()
        # bridge = MapcRosBridge("agentB10").start()
        # bridge = MapcRosBridge("agentB11").start()
        # bridge = MapcRosBridge("agentB12").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        # bridge = MapcRosBridge("agentB13").start()
        # bridge = MapcRosBridge("agentB14").start()
        # bridge = MapcRosBridge("agentB15").start()
        # bridge = MapcRosBridge("agentB16").start()
        # time.sleep(MapcRosBridge.RETRY_DELAY)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
