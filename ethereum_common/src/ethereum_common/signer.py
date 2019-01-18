# -*- coding: utf-8 -*-
#
# Robonomics signer node.
#

from robonomics_msgs.msg import Demand, Offer, Result
from web3 import Web3
from robonomics_msgs import robonomicsMessageUtils
import rospy, os
import binascii
from eth_account.messages import defunct_hash_message
from . import eth_keyfile_helper

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
            message_hash = robonomicsMessageUtils.demand_hash(msg)
            signed_hash = self.__account.signHash(defunct_hash_message(message_hash))
            msg.signature = signed_hash.signature
            rospy.loginfo('askhash: %s signature: %s', binascii.hexlify(message_hash), binascii.hexlify(msg.signature))
            self.signed_demand.publish(msg)
        rospy.Subscriber('signing/demand', Demand, sign_demand)

        def sign_offer(msg):
            msg.nonce = os.urandom(32)
            message_hash = robonomicsMessageUtils.offer_hash(msg)
            signed_hash = self.__account.signHash(defunct_hash_message(message_hash))
            msg.signature = signed_hash.signature
            rospy.loginfo('bidhash: %s signature: %s', binascii.hexlify(message_hash), binascii.hexlify(msg.signature))
            self.signed_offer.publish(msg)
        rospy.Subscriber('signing/offer', Offer, sign_offer)

        def sign_result(msg):
            message_hash = robonomicsMessageUtils.result_hash(msg)
            signed_hash = self.__account.signHash(defunct_hash_message(message_hash))
            msg.signature = signed_hash.signature
            rospy.loginfo('reshash: %s signature: %s', binascii.hexlify(message_hash), binascii.hexlify(msg.signature))
            self.signed_result.publish(msg)
        rospy.Subscriber('signing/result', Result, sign_result)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
