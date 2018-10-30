# -*- coding: utf-8 -*-
#
# Robonomics signer node.
#

from robonomics_msgs.msg import Demand, Offer, Result
from web3 import Web3
from base58 import b58decode
import rospy, os
import binascii
from eth_account.messages import defunct_hash_message
from . import eth_keyfile_helper


def demand_hash(msg):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'address',
             'address',
             'uint256',
             'uint256',
             'bytes32']
    return Web3.soliditySha3(types, [b58decode(msg.model),
                                     b58decode(msg.objective),
                                     msg.token,
                                     msg.cost,
                                     msg.lighthouse,
                                     msg.validator,
                                     msg.validatorFee,
                                     msg.deadline,
                                     msg.nonce])


def offer_hash(msg):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'address',
             'address',
             'uint256',
             'uint256',
             'bytes32']
    return Web3.soliditySha3(types, [b58decode(msg.model),
                                     b58decode(msg.objective),
                                     msg.token,
                                     msg.cost,
                                     msg.validator,
                                     msg.lighthouse,
                                     msg.lighthouseFee,
                                     msg.deadline,
                                     msg.nonce])

def result_hash(msg):
    types = ['address',
             'bytes',
             'bool']
    return Web3.soliditySha3(types, [msg.liability, b58decode(msg.result), msg.success])


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

        self.signed_demand = rospy.Publisher('sending/demand', Demand, queue_size=10)
        self.signed_offer  = rospy.Publisher('sending/offer',  Offer, queue_size=10)
        self.signed_result = rospy.Publisher('sending/result', Result, queue_size=10)

        #TODO: make tests when local sign will be implemented
        def sign_demand(msg):
            msg.nonce = os.urandom(32)
            signed_hash = self.__account.signHash(defunct_hash_message(demand_hash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('askhash: %s signature: %s', binascii.hexlify(demand_hash(msg)), binascii.hexlify(msg.signature))
            self.signed_demand.publish(msg)
        rospy.Subscriber('signing/demand', Demand, sign_demand)

        def sign_offer(msg):
            msg.nonce = os.urandom(32)
            signed_hash = self.__account.signHash(defunct_hash_message(offer_hash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('bidhash: %s signature: %s', binascii.hexlify(offer_hash(msg)), binascii.hexlify(msg.signature))
            self.signed_offer.publish(msg)
        rospy.Subscriber('signing/offer', Offer, sign_offer)

        def sign_result(msg):
            signed_hash = self.__account.signHash(defunct_hash_message(result_hash(msg)))
            msg.signature = signed_hash.signature
            rospy.loginfo('reshash: %s signature: %s', binascii.hexlify(result_hash(msg)), binascii.hexlify(msg.signature))
            self.signed_result.publish(msg)
        rospy.Subscriber('signing/result', Result, sign_result)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
