# -*- coding: utf-8 -*-

from market import Market
from signer import Signer
from distribution import Distribution

def market_node():
    Market().spin()

def signer_node():
    Signer().spin()

def distribution_node():
    Distribution().spin()
