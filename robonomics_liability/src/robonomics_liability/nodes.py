# -*- coding: utf-8 -*-
#
# Robonomics control related nodes.
#

from .listener import Listener
from .executor import Executor
from .LiabilityExecutionsPersistence import LiabilityExecutionsPersistence
from .player_service import PlayerService


def listener_node():
    Listener().spin()


def executor_node():
    Executor().spin()


def persistence_node():
    LiabilityExecutionsPersistence().spin()


def player_service_node():
    PlayerService().spin()
