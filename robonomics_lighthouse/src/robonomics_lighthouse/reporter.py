# -*- coding: utf-8 -*-
#
# Robonomics results settler node. 
#

from robonomics_lighthouse.msg import Result
from web3 import Web3, HTTPProvider
from ens import ENS
from binascii import hexlify, unhexlify
from base58 import b58decode
import rospy, json

class Reporter:
    def __init__(self):
        '''
            Market reporter initialisation.
        '''
        rospy.init_node('robonomics_reporter')

        http_provider = HTTPProvider(rospy.get_param('~web3_http_provider'))
        self.web3 = Web3(http_provider, ens=ENS(http_provider, addr=rospy.get_param('~ens_contract', None)))

        self.liability_abi = json.loads(rospy.get_param('~liability_abi'))
        lighthouse_abi = json.loads(rospy.get_param('~lighthouse_abi'))
        lighthouse_contract = rospy.get_param('~lighthouse_contract')
        self.lighthouse = self.web3.eth.contract(lighthouse_contract, abi=lighthouse_abi)

        self.account = rospy.get_param('~account_address', self.web3.eth.accounts[0])

        rospy.Subscriber('infochan/incoming/result', Result, self.settlement)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def settlement(self, msg):
        '''
            Settle incoming result.
        '''
        liability = self.web3.eth.contract(msg.liability, abi=self.liability_abi)
        data = liability.functions.finalize(
            b58decode(msg.result),
            msg.signature,
            False).buildTransaction({'gas': 1000000})['data']
        tx = self.lighthouse.functions.to(msg.liability, data).transact({'from': self.account})
        rospy.loginfo('Result submitted at %s', self.web3.toHex(tx))
