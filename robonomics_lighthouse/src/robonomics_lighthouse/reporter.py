# -*- coding: utf-8 -*-
#
# Robonomics results settler node. 
#

from robonomics_lighthouse.msg import Result
from web3 import Web3, HTTPProvider
from binascii import hexlify, unhexlify
import rospy, json

class Reporter:
    def __init__(self):
        '''
            Market reporter initialisation.
        '''
        rospy.init_node('robonomics_reporter')

        http_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))

        self.liability_abi = json.loads(rospy.get_param('~liability_abi'))
        lighthouse_abi = json.loads(rospy.get_param('~lighthouse_abi'))
        lighthouse_address = rospy.get_param('~lighthouse_contract')
        self.lighthouse = self.web3.eth.contract(lighthouse_address, abi=lighthouse_abi)

        self.account = rospy.get_param('~eth_account_address')
        self.account = self.web3.eth.accounts[0] if len(self.account) == 0 else self.account

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
        v, r, s = msg.signature[64], msg.signature[0:32], msg.signature[32:64]
        liability = self.web3.eth.contract(msg.liability, abi=self.liability_abi)
        data = liability.functions.setResult(msg.result, v, r, s).buildTransaction({'gas': 1000000})['data'] 
        self.lighthouse.transact({'gas': 300000, 'from': self.account}).to(msg.liability, data)
        rospy.loginfo('Result submitted with data: {0}'.format(data))
