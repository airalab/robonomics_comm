# -*- coding: utf-8 -*-
#
# ethereum common related nodes.
#

from .erc20_services import ERC20Services
from .eth_services import ETHServices
from .signer import Signer


def erc20_node():
    ERC20Services().spin()


def eth_node():
    ETHServices().spin()


def signer_node():
    Signer().spin()
