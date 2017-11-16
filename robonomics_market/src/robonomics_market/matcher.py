# -*- coding: utf-8 -*-
#
# Robonomics market Bid/Ask matcher node. 
#

from robonomics_market.msg import Bid, Ask
from web3 import Web3, HTTPProvider
from base58 import b58decode
import rospy, json

class Order:
    def __init__(self, msg):
        self.msg = msg

    def __cmp__(self, other):
        return cmp((self.msg.model, self.msg.cost, self.msg.count, self.msg.fee),
                   (other.msg.model, other.msg.cost, other.msg.count, other.msg.fee))

    def __repr__(self):
        return '{0}'.format(self.msg)

class Matcher:
    bids = set()
    asks = set()

    def __init__(self):
        '''
            Market distribution node initialisation.
        '''
        rospy.init_node('robonomics_distribution')

        http_provider = rospy.get_param('web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))

        security_abi = json.loads(rospy.get_param('~security_contract_abi'))
        security_address = rospy.get_param('~security_contract_address')
        self.security = self.web3.eth.contract(security_address, abi=security_abi)

        def incoming_bid(msg):
            self.bids.add(Order(msg))
            self.match()
        rospy.Subscriber('incoming/bid', Bid, incoming_bid)

        def incoming_ask(msg):
            self.asks.add(Order(msg))
            self.match()
        rospy.Subscriber('incoming/ask', Ask, incoming_ask)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def match(self):
        '''
            Make bids and asks intersection, 
            if call `match` on every insertion to bids/asks then
            intersection should have only two elements or nothing.
        '''
        eqs = self.bids & self.asks
        if len(eqs) == 0:
            return

        assert(len(eqs) == 2)

        rospy.loginfo('Match found: %s', eqs)

        try:
            eqs[0].msg.objective
            self.new_liability(eqs[0], eqs[1])
            self.asks.remove(eqs[0])
            self.bids.remove(eqs[1])

        except AttributeError:
            self.new_liability(eqs[1], eqs[0])
            self.asks.remove(eqs[1])
            self.bids.remove(eqs[0])

    def new_liability(self, ask, bid):
        '''
            Create liability contract for matched ask & bid.
        '''
        assert(ask.model == bid.model
                and ask.cost == bid.cost
                and ask.count == bid.count
                and ask.fee == bid.fee) 

        self.security.transact().create(b58decode(ask.model),
                                        b58decode(ask.objective), 
                                        ask.cost,
                                        ask.count,
                                        ask.fee,
                                        ask.salt,
                                        ask.signature,
                                        bid.salt,
                                        bid.signature)
        rospy.loginfo('Liability created')
