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

class Order:
    def __init__(self, msg):
        self.msg = msg

    def __hash__(self):
        return hash((self.msg.model, self.msg.cost, self.msg.count, self.msg.fee))

    def __eq__(self, other):
        return (self.msg.model, self.msg.cost, self.msg.count, self.msg.fee) == (other.msg.model, other.msg.cost, other.msg.count, other.msg.fee)

    def __repr__(self):
        return '{0}'.format(self.msg)

class Matcher:
    bids = {}
    asks = {}

    def __init__(self):
        '''
            Market distribution node initialisation.
        '''
        rospy.init_node('robonomics_matcher')

        http_provider = rospy.get_param('web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))

        security_abi = json.loads(rospy.get_param('~security_contract_abi'))
        security_address = rospy.get_param('~security_contract_address')
        self.security = self.web3.eth.contract(security_address, abi=security_abi)

        def incoming_bid(msg):
            rospy.logdebug('Incoming bid: %s', msg)
            self.match(bid=msg)
        rospy.Subscriber('incoming/bid', Bid, incoming_bid)

        def incoming_ask(msg):
            rospy.logdebug('Incoming ask: %s', msg)
            self.match(ask=msg)
        rospy.Subscriber('incoming/ask', Ask, incoming_ask)

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
            self.bids[hash(Order(bid))] = Order(bid)
        else:
            self.asks[hash(Order(ask))] = Order(ask)

        try:
            if not ask:
                ask = self.asks[hash(Order(bid))].msg
            else:
                bid = self.bids[hash(Order(ask))].msg

            rospy.loginfo('Match found: %s <=> %s', ask, bid)

            self.new_liability(ask, bid)
            del self.asks[Order(ask)]
            del self.bids[Order(bid)]

        except KeyError:
            rospy.loginfo('No match found')

    def new_liability(self, ask, bid):
        '''
            Create liability contract for matched ask & bid.
        '''
        assert(ask.model == bid.model
                and ask.cost == bid.cost
                and ask.count == bid.count
                and ask.fee == bid.fee)

        rospy.loginfo('model: %s', hexlify(b58decode(ask.model)).decode('utf-8'))
        rospy.loginfo('obj: %s', hexlify(b58decode(ask.objective)).decode('utf-8'))
        rospy.loginfo('siga: %s', hexlify(ask.signature).decode('utf-8'))
        rospy.loginfo('sigb: %s', hexlify(bid.signature).decode('utf-8'))
        rospy.loginfo('salta: %s', hexlify(ask.salt).decode('utf-8'))
        rospy.loginfo('saltb: %s', hexlify(bid.salt).decode('utf-8'))
        rospy.loginfo('hasha: %s', hexlify(signer.askhash(ask)).decode('utf-8'))
        rospy.loginfo('hashb: %s', hexlify(signer.bidhash(bid)).decode('utf-8'))

        self.security.transact().create(b58decode(ask.model),
                                        b58decode(ask.objective),
                                        [ask.cost, ask.count, ask.fee],
                                        ask.salt,
                                        ask.signature,
                                        bid.salt,
                                        bid.signature)
        rospy.loginfo('Liability contract created')
