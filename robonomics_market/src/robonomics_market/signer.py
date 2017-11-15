# -*- coding: utf-8 -*-
#
# Robonomics market order signer node.
#

from robonomics_market.msg import Ask, Bid
from web3 import Web3, HTTPProvider
from binascii import b2a_hex
from base58 import b58decode
import rospy, os

def ask_hexdata(msg):
    msgdata  = b2a_hex(b58decode(msg.model))
    msgdata += b2a_hex(b58decode(msg.objective))
    msgdata += '%0.64X' % msg.cost
    msgdata += '%0.64X' % msg.count
    msgdata += '%0.64X' % msg.fee
    msgdata += msg.salt
    return '0x'+msgdata

def bid_hexdata(msg):
    msgdata  = b2a_hex(b58decode(msg.model))
    msgdata += '%0.64X' % msg.cost
    msgdata += '%0.64X' % msg.count
    msgdata += '%0.64X' % msg.fee
    msgdata += msg.salt[2:]
    return '0x'+msgdata

class Signer:
    def __init__(self):
        '''
            Market order signer initialisation.
        '''
        rospy.init_node('robonomics_signer')
        http_provider = rospy.get_param('web3_http_provider', 'http://localhost:8545')
        self.web3     = Web3(HTTPProvider(http_provider))
        self.account  = self.web3.eth.accounts[0]

        self.signed_ask = rospy.Publisher('sending/ask', Ask, queue_size=10)
        self.signed_bid = rospy.Publisher('sending/bid', Bid, queue_size=10)

        def randsalt():
            return b2a_hex(os.urandom(15))

        def sign_ask(msg):
            msg.salt = '0x'+randsalt()
            msgdata  = ask_hexdata(msg)
            msg.signature = self.web3.eth.sign(self.account, hexstr=msgdata)
            self.signed_ask.publish(msg)
        rospy.Subscriber('signing/ask', Ask, sign_ask)

        def sign_bid(msg):
            msg.salt = '0x'+randsalt()
            msgdata  = bid_hexdata(msg)
            msg.signature = self.web3.eth.sign(self.account, hexstr=msgdata)
            self.signed_bid.publish(msg)
        rospy.Subscriber('signing/bid', Bid, sign_bid)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
