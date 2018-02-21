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
            if h in self.bids:
                self.bids[h].append(bid)
            else:
                self.bids[h] = [bid]
        else:
            h = hash((ask.model, ask.cost, ask.count, ask.fee))
            if h in self.asks:
                self.asks[h].append(ask)
            else:
                self.asks[h] = [ask]

        try:
            if not ask:
                h = hash((bid.model, bid.cost, bid.count, bid.fee))
                ask = self.asks[h].pop()
            else:
                h = hash((ask.model, ask.cost, ask.count, ask.fee))
                bid = self.bids[h].pop()

            prlot = lambda lot: '| {0} Mod: {1} Cst: {2} Cnt: {3} Fee: {4} |'.format(
                                'Ask Obj: '+lot.objective if hasattr(lot, 'objective') else 'Bid',
                                lot.model, lot.cost, lot.count, lot.fee)
            rospy.loginfo('Matched %s & %s', prlot(ask), prlot(bid))
            self.new_liability(ask, bid)

        except (KeyError, IndexError):
            rospy.loginfo('No match for: %s', prlot(ask or bid))


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
