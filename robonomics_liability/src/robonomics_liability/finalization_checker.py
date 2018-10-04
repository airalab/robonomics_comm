import rospy
from web3 import Web3, HTTPProvider
from ens import ENS


class FinalizationChecker:
    def __init__(self, abi, web3_http_provider="http://localhost:8545", ens_contract=None):

        http_provider = HTTPProvider(web3_http_provider)
        self.ens = ENS(http_provider, addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)
        self.abi = abi

    def finalized(self, address):
        try:
            c = self.web3.eth.contract(address, abi=self.abi)
            return c.call().isFinalized()
        except Exception as e:
            rospy.logwarn("Failed check %s finalization with exception: %s", address, e)
            return None
