# -*- coding: utf-8 -*-
#
# Robonomics lighthouse signer node.
#

from robonomics_lighthouse.msg import Ask, Bid, Result
from web3 import Web3
from base58 import b58decode
import rospy, os
import binascii
from eth_account.messages import defunct_hash_message
from ethereum_common import eth_keyfile_helper

def askhash(msg):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'address',
             'uint256',
             'uint256',
             'bytes32']
    return Web3.soliditySha3(types, [b58decode(msg.model),
                                     b58decode(msg.objective),
                                     msg.token,
                                     msg.cost,
                                     msg.validator,
                                     msg.validatorFee,
                                     msg.deadline,
                                     msg.nonce])

def bidhash(msg):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'uint256',
             'uint256',
             'bytes32']
    return Web3.soliditySha3(types, [b58decode(msg.model),
                                     b58decode(msg.objective),
                                     msg.token,
                                     msg.cost,
                                     msg.lighthouseFee,
                                     msg.deadline,
                                     msg.nonce])

def reshash(msg):
    types = ['address',
             'bytes']
    return Web3.soliditySha3(types, [msg.liability, b58decode(msg.result)])

class Signer:
    def __init__(self):
        '''
            Lighthouse signer initialisation.
        '''
        rospy.init_node('robonomics_signer')
        self.web3 = Web3()

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        __keyfile = rospy.get_param('~keyfile')
        __keyfile_password_file = rospy.get_param('~keyfile_password_file')

        __keyfile_helper = eth_keyfile_helper.KeyfileHelper(__keyfile, keyfile_password_file=__keyfile_password_file)
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.signed_ask = rospy.Publisher('sending/ask', Ask, queue_size=10)
        self.signed_bid = rospy.Publisher('sending/bid', Bid, queue_size=10)
        self.signed_res = rospy.Publisher('sending/result', Result, queue_size=10)

        #TODO: make tests when local sign will be implemented
        def sign_ask(msg):
            msg.nonce = os.urandom(32)
            signed_hash = self.__account.signHash(defunct_hash_message(askhash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('askhash: %s signature: %s', binascii.hexlify(askhash(msg)), binascii.hexlify(msg.signature))
            self.signed_ask.publish(msg)
        rospy.Subscriber('signing/ask', Ask, sign_ask)

        def sign_bid(msg):
            msg.nonce = os.urandom(32)
            signed_hash = self.__account.signHash(defunct_hash_message(bidhash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('bidhash: %s signature: %s', binascii.hexlify(bidhash(msg)), binascii.hexlify(msg.signature))
            self.signed_bid.publish(msg)
        rospy.Subscriber('signing/bid', Bid, sign_bid)

        def sign_res(msg):
            signed_hash = self.__account.signHash(defunct_hash_message(reshash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('reshash: %s signature: %s', binascii.hexlify(reshash(msg)), binascii.hexlify(msg.signature))
            self.signed_res.publish(msg)
        rospy.Subscriber('signing/result', Result, sign_res)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
