# -*- coding: utf-8 -*-
#
# Robonomics market order signer node.
#

from robonomics_market.msg import Ask, Bid
from web3 import Web3, HTTPProvider
from base58 import b58decode
import rospy, os

def askhash(msg):
    types = ['bytes32', 'bytes32', 'uint256', 'uint256', 'bytes32']
    return Web3.soliditySha3(types,
            [ b58decode(msg.model)[2:]
            , b58decode(msg.objective)[2:]
            , msg.cost * msg.count
            , msg.lighthouseFee
            , msg.salt ])

def bidhash(msg):
    types = ['bytes32', 'uint256', 'uint256', 'uint256', 'bytes32']
    return Web3.soliditySha3(types,
            [ b58decode(msg.model)[2:]
            , msg.cost * msg.count
            , msg.lighthouseFee
            , msg.validatorFee
            , msg.salt ])

class Signer:
    def __init__(self):
        '''
            Market order signer initialisation.
        '''
        rospy.init_node('robonomics_signer')
        http_provider = rospy.get_param('~web3_http_provider')
        self.web3     = Web3(HTTPProvider(http_provider))
        self.account  = self.web3.eth.accounts[0]

        self.signed_ask = rospy.Publisher('sending/ask', Ask, queue_size=10)
        self.signed_bid = rospy.Publisher('sending/bid', Bid, queue_size=10)

        def sign_ask(msg):
            msg.salt = os.urandom(32)
            msg.signature = self.web3.eth.sign(self.account, askhash(msg))
            self.signed_ask.publish(msg)
        rospy.Subscriber('signing/ask', Ask, sign_ask)

        def sign_bid(msg):
            msg.salt = os.urandom(32)
            msg.signature = self.web3.eth.sign(self.account, bidhash(msg))
            self.signed_bid.publish(msg)
        rospy.Subscriber('signing/bid', Bid, sign_bid)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
