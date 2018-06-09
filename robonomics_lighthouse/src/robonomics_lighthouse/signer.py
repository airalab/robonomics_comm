# -*- coding: utf-8 -*-
#
# Robonomics lighthouse signer node.
#

from robonomics_lighthouse.msg import Ask, Bid, Result
from web3 import Web3, HTTPProvider
from base58 import b58decode
import rospy, os
import binascii

def askhash(msg):
    types = [ 'bytes'
            , 'bytes'
            , 'address'
            , 'uint256'
            , 'address'
            , 'uint256'
            , 'uint256'
            , 'bytes32' ]
    return Web3.soliditySha3(types,
            [ b58decode(msg.model)
            , b58decode(msg.objective)
            , msg.token
            , msg.cost
            , msg.validator
            , msg.validatorFee
            , msg.deadline
            , msg.nonce ])

def bidhash(msg):
    types = [ 'bytes'
            , 'bytes'
            , 'address'
            , 'uint256'
            , 'uint256'
            , 'uint256'
            , 'bytes32' ]
    return Web3.soliditySha3(types,
            [ b58decode(msg.model)
            , b58decode(msg.objective)
            , msg.token
            , msg.cost
            , msg.lighthouseFee
            , msg.deadline
            , msg.nonce ])

def reshash(msg):
    types = [ 'address'
            , 'bytes' ]
    return Web3.soliditySha3(types, [msg.liability, b58decode(msg.result)])

class Signer:
    def __init__(self):
        '''
            Lighthouse signer initialisation.
        '''
        rospy.init_node('robonomics_signer')
        http_provider = rospy.get_param('~web3_http_provider')
        self.web3     = Web3(HTTPProvider(http_provider))
        self.account  = rospy.get_param('~account_address', self.web3.eth.accounts[0])

        self.signed_ask = rospy.Publisher('sending/ask', Ask, queue_size=10)
        self.signed_bid = rospy.Publisher('sending/bid', Bid, queue_size=10)
        self.signed_res = rospy.Publisher('sending/result', Result, queue_size=10)

        def sign_ask(msg):
            msg.nonce = os.urandom(32)
            msg.signature = self.web3.eth.sign(self.account, askhash(msg))
            rospy.loginfo('askhash: %s signature: %s', binascii.hexlify(askhash(msg)), binascii.hexlify(msg.signature))
            self.signed_ask.publish(msg)
        rospy.Subscriber('signing/ask', Ask, sign_ask)

        def sign_bid(msg):
            msg.nonce = os.urandom(32)
            msg.signature = self.web3.eth.sign(self.account, bidhash(msg))
            rospy.loginfo('bidhash: %s signature: %s', binascii.hexlify(bidhash(msg)), binascii.hexlify(msg.signature))
            self.signed_bid.publish(msg)
        rospy.Subscriber('signing/bid', Bid, sign_bid)

        def sign_res(msg):
            msg.signature = self.web3.eth.sign(self.account, reshash(msg))
            rospy.loginfo('reshash: %s signature: %s', binascii.hexlify(reshash(msg)), binascii.hexlify(msg.signature))
            self.signed_res.publish(msg)
        rospy.Subscriber('signing/result', Result, sign_res)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
