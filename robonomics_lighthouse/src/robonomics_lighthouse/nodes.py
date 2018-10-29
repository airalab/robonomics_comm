# -*- coding: utf-8 -*-
#
# Robonomics market related nodes.
#

from . import lighthouse 
from . import infochan
from . import matcher


def infochan_node():
    infochan.InfoChan().spin()


def matcher_node():
    matcher.Matcher().spin()


def lighthouse_node():
    lighthouse.Lighthouse().spin()
