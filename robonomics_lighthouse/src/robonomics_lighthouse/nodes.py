# -*- coding: utf-8 -*-
#
# Robonomics market related nodes.
#

from . import lighthouse
from . import matcher


def matcher_node():
    matcher.Matcher().spin()


def lighthouse_node():
    lighthouse.Lighthouse().spin()
