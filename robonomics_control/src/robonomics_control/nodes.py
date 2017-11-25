# -*- coding: utf-8 -*-
#
# Robonomics control related nodes.
#

from . import distribution

def distribution_node():
    distribution.Distribution().spin()
