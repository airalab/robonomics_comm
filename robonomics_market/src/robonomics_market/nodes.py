# -*- coding: utf-8 -*-

from market import Market
from distribution import Distribution

def market_node():
    Market().spin()

def distribution_node():
    Distribution().spin()
