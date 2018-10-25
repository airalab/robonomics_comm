# -*- coding: utf-8 -*-
#
# ethereum common related nodes.
#

from . import erc20_services
from . import eth_services

def erc20_node():
    erc20_services.ERC20Services().spin()

def eth_node():
    eth_services.ETHServices().spin()
