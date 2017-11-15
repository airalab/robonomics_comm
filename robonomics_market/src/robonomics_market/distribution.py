# -*- coding: utf-8 -*-
#
# Robonomics market distribution controller.
#

from web3 import Web3, HTTPProvider
from web3.contract import ConciseContract
import numpy as np
import rospy

def desired_distribution(self, cap_vector, rob_vector):
    '''
        Desired robot distribution according to investor capital distribution,
        ref http://ensrationis.com/smart-factory-and-capital/
    '''
    return cap_vector * (sum(rob_vector) + 1) / sum(cap_vector)

def distribution_error(self, cap_vector, rob_vector):
    '''
        Robot distribution error according to current capital and robot distribution,
        ref http://ensrationis.com/smart-factory-and-capital/
    '''
    return desired_distribution(cap_vector, rob_vector) - rob_vector

class Distribution:
    INVESTORS_CONTRACT_ABI = []
    INVESTORS_CONTRACT_ADDRESS = ""
    robots = {}

    def __init__(self):
        '''
            Market distribution node initialisation.
        '''
        rospy.init_node('robonomics_distribution')
        http_provider = rospy.get_param('web3_http_provider', 'http://localhost:8545')
        self.web3 = Web3(HTTPProvider(http_provider))
        self.ns = ENS.fromWeb3(self.web3)
        self.investors = self.web3.eth.contract(self.INVESTORS_CONTRACT_ABI,
                                                self.INVESTORS_CONTRACT_ADDRESS,
                                                ContractFactoryClass=ConciseContract)
        self.market_list = rospy.get_param('supported_models')
        self.current_market = rospy.Publisher('current_market', std_msgs.msg.String, queue_size=10)
        self.subscribe_new_bids()

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def subscribe_new_bids(self):
        '''
            Subscribe to incoming bids and register new robots by markets.
        '''
        def ecrecover(order):
            # TODO: ecrecover contract call
            return 'NONE'

        def incoming_bid(msg):
            if msg.model in self.market_list:
                self.robots[msg.model].add(ecrecover(msg))
                self.update_current_market()

        rospy.Subscriber('incoming/bid', Bid, incoming_bid)

    def update_current_market(self):
        '''
            Market distribution control rule.
            Choose market by capital proportional robot distribution,
            ref http://ensrationis.com/smart-factory-and-capital/
        '''
        rospy.info('Input market list is %s', self.market_list)

        cap = [self.investors.totalCap(m) for m in self.market_list]
        rospy.info('Capitalization vector is %s', cap)

        rob = [len(self.robots[m]) for m in self.market_list]
        rospy.info('Real robot distribution is %s', rob)

        err = distribution_error(np.array(cap), np.array(rob))
        rospy.info('Robot distribution error is %s', err)

        maxi = np.argmax(err)
        rospy.info('Maximal error index is %d', maxi)

        self.current_market.publish(self.market_list[maxi])
