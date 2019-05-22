# -*- coding: utf-8 -*-
#
# Robonomics control related nodes.
#

from . import listener
from . import executor
from . import LiabilityExecutionsPersistence
from . import player_service


def listener_node():
    listener.Listener().spin()


def executor_node():
    executor.Executor().spin()


def persistence_node():
    LiabilityExecutionsPersistence.LiabilityExecutionsPersistence().spin()


def player_service_node():
    player_service.PlayerService().spin()
