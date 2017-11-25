# -*- coding: utf-8 -*-
#
# Robonomics market related nodes.
#

from . import market
from . import signer
from . import matcher
from . import generator

def market_node():
    market.Market().spin()

def signer_node():
    signer.Signer().spin()

def generator_node():
    generator.Generator().spin()

def matcher_node():
    matcher.Matcher().spin()
