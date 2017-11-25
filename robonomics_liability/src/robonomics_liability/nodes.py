# -*- coding: utf-8 -*-
#
# Robonomics control related nodes.
#

from . import listener 
from . import executor

def listener_node():
    listener.Listener().spin()

def executor_node():
    executor.Executor().spin()
