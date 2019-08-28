# -*- coding: utf-8 -*-
#
# Robonomics signer node.
#

from robonomics_msgs.msg import Demand, Offer, Result
from ethereum_common.msg import Address, UInt256
from web3 import Web3, HTTPProvider
from web3.exceptions import StaleBlockchain
from threading import Lock
from ens import ENS
from robonomics_msgs import robonomicsMessageUtils
import rospy
import binascii
import json
from eth_account.messages import defunct_hash_message
from . import eth_keyfile_helper


class Signer:
    def __init__(self):
        '''
            Lighthouse signer initialisation.
        '''
        rospy.init_node('robonomics_signer')

        ens_contract = rospy.get_param('~ens_contract', None)
        web3_http_provider = rospy.get_param('~web3_http_provider')
        http_provider = HTTPProvider(web3_http_provider)

        self.ens = ENS(http_provider, addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        __factory_contract_abi = rospy.get_param('~factory_contract_abi')
        __factory_contract = rospy.get_param('~factory_contract')
        self.factory = None

        __keyfile = rospy.get_param('~keyfile')
        __keyfile_password_file = rospy.get_param('~keyfile_password_file')

        __keyfile_helper = eth_keyfile_helper.KeyfileHelper(__keyfile, keyfile_password_file=__keyfile_password_file)
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.signed_demand = rospy.Publisher('sending/demand', Demand, queue_size=10)
        self.signed_offer  = rospy.Publisher('sending/offer',  Offer, queue_size=10)
        self.signed_result = rospy.Publisher('sending/result', Result, queue_size=10)

        self.__factory_initialization_lock = Lock()

        def get_initialized_factory():
            if self.factory is None:
                try:
                    factory_abi = json.loads(__factory_contract_abi)
                    factory_address = self.ens.address(__factory_contract)
                    try:
                        self.__factory_initialization_lock.acquire()
                        if self.factory is None:
                            self.factory = self.web3.eth.contract(factory_address, abi=factory_abi)
                    finally:
                        self.__factory_initialization_lock.release()

                    return self.factory
                except StaleBlockchain as e:
                    rospy.logwarn("Failed to initialize factory cause exception: %s", e)
            else:
                return self.factory

        def get_nonce_by_address(address):
            try:
                nonce = get_initialized_factory().call().nonceOf(address.address)
                return UInt256(nonce)
            except Exception as e:
                rospy.logerr("Failed to get nonce by address %s with exception: %s", address.address, e)
                try:
                    self.__factory_initialization_lock.acquire()
                    self.factory = None
                finally:
                    self.__factory_initialization_lock.release()

        def sign_demand(msg):
            msg.sender = Address(self.__account.address)
            msg = robonomicsMessageUtils.convert_msg_ens_names_to_addresses(msg, web3=self.web3)
            current_nonce = get_nonce_by_address(msg.sender)
            message_hash = robonomicsMessageUtils.demand_hash(msg, current_nonce)
            signed_hash = self.__account.signHash(defunct_hash_message(message_hash))
            msg.signature = signed_hash.signature
            msg.nonce = UInt256(uint256=str(current_nonce.uint256))
            rospy.loginfo('askhash: %s signature: %s', binascii.hexlify(message_hash), binascii.hexlify(msg.signature))
            self.signed_demand.publish(msg)
        rospy.Subscriber('signing/demand', Demand, sign_demand)

        def sign_offer(msg):
            msg.sender = Address(self.__account.address)
            msg = robonomicsMessageUtils.convert_msg_ens_names_to_addresses(msg, web3=self.web3)
            current_nonce = get_nonce_by_address(msg.sender)
            message_hash = robonomicsMessageUtils.offer_hash(msg, current_nonce)
            signed_hash = self.__account.signHash(defunct_hash_message(message_hash))
            msg.signature = signed_hash.signature
            msg.nonce = UInt256(uint256=str(current_nonce.uint256))
            rospy.loginfo('bidhash: %s signature: %s', binascii.hexlify(message_hash), binascii.hexlify(msg.signature))
            self.signed_offer.publish(msg)
        rospy.Subscriber('signing/offer', Offer, sign_offer)

        def sign_result(msg):
            msg = robonomicsMessageUtils.convert_msg_ens_names_to_addresses(msg, web3=self.web3)
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
