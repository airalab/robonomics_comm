# -*- coding: utf-8 -*-
#
# Robonomics market related nodes.
#

from . import lighthouse 
from . import infochan
from . import matcher
from . import signer

def infochan_node():
    infochan.InfoChan().spin()

def signer_node():
    signer.Signer().spin()

def matcher_node():
    matcher.Matcher().spin()

def lighthouse_node():
    lighthouse.Lighthouse().spin()
