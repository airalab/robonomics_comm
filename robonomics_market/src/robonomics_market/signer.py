# -*- coding: utf-8 -*-
#
# Robonomics market order signer node.
#

from robonomics_market.msg import Ask, Bid
from web3 import Web3, HTTPProvider
from base58 import b58decode
import rospy, os

def askdata(msg):
    msgdata  = b58decode(msg.model)
    msgdata += b58decode(msg.objective)
    msgdata += msg.cost.to_bytes(32, byteorder='big')
    msgdata += msg.count.to_bytes(32, byteorder='big')
    msgdata += msg.fee.to_bytes(32, byteorder='big')
    msgdata += msg.salt
    return msgdata

def biddata(msg):
    msgdata  = b58decode(msg.model)
    msgdata += msg.cost.to_bytes(32, byteorder='big')
    msgdata += msg.count.to_bytes(32, byteorder='big')
    msgdata += msg.fee.to_bytes(32, byteorder='big')
    msgdata += msg.salt
    return msgdata

class Signer:
    def __init__(self):
        '''
            Market order signer initialisation.
        '''
        rospy.init_node('robonomics_signer')
        http_provider = rospy.get_param('web3_http_provider', 'http://localhost:8545')
        self.web3     = Web3(HTTPProvider(http_provider))
        self.account  = '0x4af013AfBAdb22D8A88c92D68Fc96B033b9Ebb8a' #self.web3.eth.accounts[0]

        self.signed_ask = rospy.Publisher('sending/ask', Ask, queue_size=10)
        self.signed_bid = rospy.Publisher('sending/bid', Bid, queue_size=10)

        def sign_ask(msg):
            msg.salt = os.urandom(15)
            msg.signature = self.web3.eth.sign(self.account, data=askdata(msg))
            self.signed_ask.publish(msg)
        rospy.Subscriber('signing/ask', Ask, sign_ask)

        def sign_bid(msg):
            msg.salt = os.urandom(15)
            msg.signature = self.web3.eth.sign(self.account, data=biddata(msg))
            self.signed_bid.publish(msg)
        rospy.Subscriber('signing/bid', Bid, sign_bid)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
