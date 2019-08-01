#! /usr/bin/env python2
"""
Created on 13.04.2015

@author: wypler, hrabia

This is just the planner/manager node executable, the particular implementation has been extracted to the ManagerNode
class to have it as well available in other packages.

"""
import sys
from behaviour_components.manager_node import ManagerNode
from rospy.exceptions import ROSInterruptException

import utils.rhbp_logging
rhbplog = utils.rhbp_logging.LogManager(logger_name=utils.rhbp_logging.LOGGER_DEFAULT_NAME)


if __name__ == '__main__':

    prefix = ""

    for arg in sys.argv:
        if arg.startswith('prefix:='):
            prefix = arg[len('prefix:='):]
            break

    node = ManagerNode(manager_prefix=prefix)

    try:
        node.run()
    except ROSInterruptException:
        rhbplog.loginfo("Planner node shut down")
