# -*- coding: utf-8 -*-
#
# Robonomics control related nodes.
#

from .distribution import Distribution

def distribution_node():
    Distribution().spin()
