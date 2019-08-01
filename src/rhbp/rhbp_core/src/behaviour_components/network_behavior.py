'''
Created on 05.01.2017

@author: rieger
'''

from behaviour_components.conditions import create_condition_from_effect
from behaviour_components.behaviours import BehaviourBase
from behaviour_components.goals import OfflineGoal
from behaviour_components.managers import Manager

from utils.deprecation import deprecated

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME + '.behaviours')


class NetworkBehaviour(BehaviourBase):
    """
    Behavior, which encapsulates an additional manager and behaviors.
    This allows to build hierarchies of hybrid behaviour planners.
    """

    MANAGER_POSTFIX = "Manager"
    TYPE_STRING = "Network"

    def __init__(self, name, requires_execution_steps=True,
                 only_running_for_deciding_interruptible=Manager.USE_ONLY_RUNNING_BEHAVIOURS_FOR_INTERRUPTIBLE_DEFAULT_VALUE,
                 correlations=None, always_update_activation=False,
                 guarantee_decision=False,
                 **kwargs):
        """
        :param correlations: tuple <Effect>
        :param name: name of the behaviour that is also used to create the sub manager name together with the NetworkBehaviour.MANAGER_POSTFIX
        :param requires_execution_steps: whether the execution steps should be caused from the parent manager or not.
                If not, the step method must be called manually
        :param always_update_activation: if set to True the entire activation calculation of the sub manager is updated on each behaviour computation update
        :param guarantee_decision: if there are executable behaviours in the local network, adjust the thresholds until
               at least one behaviour is selected
        :param kwargs: args for the manager, except the prefix arg
        """
        super(NetworkBehaviour, self).__init__(name=name, requires_execution_steps=requires_execution_steps, **kwargs)
        if "interruptable" in kwargs:
            rhbplog.logwarn("Interruptable parameter will be ignored in a NetworkBehaviour. Interruptable attribute is "
                            "evaluated based on the running or registered parameters, "
                            "see 'only_running_for_deciding_interruptible'")
        self.requires_execution_steps = requires_execution_steps
        self.always_update_activation = always_update_activation
        self.guarantee_decision = guarantee_decision
        manager_args = {}
        manager_args.update(kwargs)
        manager_args['prefix'] = self.get_manager_prefix()
        self.__manager = Manager(enabled=False,
                                 use_only_running_behaviors_for_interRuptible=only_running_for_deciding_interruptible,
                                 **manager_args)

        self.__goal_name_prefix = name + "/Goals/"
        self.__goal_counter = 0

        if correlations is not None:
            self.add_effects(correlations)

    def get_manager_prefix(self):
        """
        Return the manager prefix generated by the behaviour name and the MANAGER_POSTFIX
        :return: the manager prefix str
        """
        return self._name + '/' + NetworkBehaviour.MANAGER_POSTFIX

    def __generate_goal_name(self, effect):
        """
        :param effect: instance of type  Effect
        :return: unique name for goal
        """
        # x as separator between counter an sensor names, to prevent conflict, caused by unusual names
        name = self.__goal_name_prefix + str(self.__goal_counter) + 'X' + effect.sensor_name
        self.__goal_counter += 1
        return name

    def _create_goal(self, sensor, effect, goal_name):
        """
        Generate goals, which made the manager trying to work infinitely on the given effect,
         until the network is stopped. Therefore the goal shouldn't reachable (except the goal for boolean effects)
        :param sensor: instance of type Sensor
        :param effect: instance of type  Effect
        :param goal_name: unique name for the goal
        :return: a goal, which causes the manager to work on the effect during the whole time
        :raises RuntimeError: if the creation of a goal for an effect of this
                type is not possible
        """

        try:
            condition = create_condition_from_effect(effect=effect, sensor=sensor)
            return OfflineGoal(name=goal_name, planner_prefix=self.get_manager_prefix(), permanent=True,
                               conditions={condition})
        except RuntimeError:
            raise RuntimeError(msg="Can't create goal for effect type '" +
                                   effect.sensor_type + "'.Overwrite the method _create_goal to handle the type")

    @deprecated
    def add_correlations(self, correlations):
        """
        Adds the given effects to the correlations of this Behavior. 
        DEPRECATED: Use *add_effects* instead
        :param correlations: list of Effects
        """
        self.add_effects(correlations)

    @deprecated
    def add_correlations_and_goals(self, sensor_correlations):
        """
        Adds the given effects to the correlations of this Behavior. 
        Furthermore creates a goal for each Effect and registers it at the nested Manager
        DEPRECATED: Use *add_effects_and_goals* instead
        :param sensor_correlations: list of tuples of (Sensor, Effect)
        """
        self.add_effects_and_goals(sensor_correlations)

    def add_effects(self, effects):
        """
        Adds the given effects to this Behavior. 
        :param effects: list of Effects
        """
        self._correlations.extend(effects)

    def add_effects_and_goals(self, sensor_effect):
        """
        Adds the given effects to the correlations of this Behavior. 
        Furthermore creates a goal for each Effect and registers it at the nested Manager
        :param sensor_effect: list of tuples of (Sensor, Effect)
        """
        for sensor, effect in sensor_effect:
            goal_name = self.__generate_goal_name(effect)
            goal = self._create_goal(sensor=sensor, effect=effect, goal_name=goal_name)
            self.__manager.add_goal(goal)
            self._correlations.append(effect)

    def add_goal(self, goal):
        """
        Adds the given goal to nested manager
        :param goal: AbstractGoalRepresentation
        """
        self.__manager.add_goal(goal)

    def updateComputation(self, manager_step):
        super(NetworkBehaviour, self).updateComputation(manager_step)

        # only trigger the update if not already activated because then it would be executed anyhow
        if self.always_update_activation and not self.__manager.enabled:
            self.__manager.update_activation(plan_if_necessary=False)

        if not self._isExecuting:
            self.__manager.send_discovery()

    def do_step(self):
        self.__manager.step(guarantee_decision=self.guarantee_decision)

    def start(self):
        self.__manager.enable()

    def stop(self):
        self.__manager.disable()

    def _is_interruptible(self):
        return self.__manager.is_interruptible()
