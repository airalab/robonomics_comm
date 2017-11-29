# -*- coding: utf-8 -*-
#
# Robonomics market Bid/Ask matcher node. 
#

from robonomics_market.msg import Bid, Ask
from web3 import Web3, HTTPProvider
from base58 import b58decode
from binascii import hexlify
import rospy, json

from . import signer 

class Matcher:
    bids = {}
    asks = {}

    def __init__(self):
        '''
            Market distribution node initialisation.
        '''
        rospy.init_node('robonomics_matcher')

        http_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))

        builder_abi = json.loads(rospy.get_param('~builder_contract_abi'))
        builder_address = rospy.get_param('~builder_contract_address')
        self.builder = self.web3.eth.contract(builder_address, abi=builder_abi)

        rospy.Subscriber('market/incoming/bid', Bid, lambda x: self.match(bid=x))
        rospy.Subscriber('market/incoming/ask', Ask, lambda x: self.match(ask=x))

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def match(self, bid=None, ask=None):
        '''
            Make bids and asks intersection.
        '''
        assert(not bid or not ask)

        if not ask:
            h = hash((bid.model, bid.cost, bid.count, bid.fee))
            self.bids[h] = bid
        else:
            h = hash((ask.model, ask.cost, ask.count, ask.fee))
            self.asks[h] = ask

        try:
            if not ask:
                h = hash((bid.model, bid.cost, bid.count, bid.fee))
                ask = self.asks[h]
            else:
                h = hash((ask.model, ask.cost, ask.count, ask.fee))
                bid = self.bids[h]

            rospy.loginfo('Match found: %s <=> %s', ask, bid)

            self.new_liability(ask, bid)
            del self.asks[h]
            del self.bids[h]

        except KeyError:
            rospy.loginfo('No match found')

    def new_liability(self, ask, bid):
        '''
            Create liability contract for matched ask & bid.
        '''
        param = [ask.cost, ask.count, ask.fee]
        sign = [ ask.salt
               , bytes(31) + bytes([ask.signature[64]])
               , ask.signature[0:32]
               , ask.signature[32:64]
               , bid.salt
               , bytes(31) + bytes([bid.signature[64]])
               , bid.signature[0:32]
               , bid.signature[32:64] ]
        self.builder.transact().create(b58decode(ask.model), b58decode(ask.objective), param, sign)
        rospy.loginfo('Liability contract created')
