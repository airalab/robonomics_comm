# -*- coding: utf-8 -*-
#
# Robonomics liability tracking node.
#

from robonomics_liability.msg import Liability
from base58 import b58decode, b58encode
from web3 import Web3, HTTPProvider
from threading import Timer
import rospy, json

class Listener:
    def __init__(self):
        '''
            Robonomics liability tracking node initialisation.
        '''
        rospy.init_node('robonomics_liability_listener')

        http_provider = rospy.get_param('~web3_http_provider')
        self.web3 = Web3(HTTPProvider(http_provider))

        self.poll_interval = rospy.get_param('~poll_interval', 5)

        self.liability = rospy.Publisher('incoming', Liability, queue_size=10)

        builder_abi = json.loads(rospy.get_param('~builder_contract_abi'))
        builder_address = rospy.get_param('~builder_contract_address')
        self.builder = self.web3.eth.contract(builder_address, abi=builder_abi)

        self.liability_abi = json.loads(rospy.get_param('~liability_contract_abi'))

        def liability_result(msg):
            rospy.loginfo('Set result `%s` to %s', msg.result, msg.address)
            try:
                l = self.web3.eth.contract(msg.address, abi=self.liability_abi)
                l.transact({'gas': 100000}).setResult(b58decode(msg.result))
            except Exception as e:
                rospy.logerr(e)
        rospy.Subscriber('result', LiabilityResult, liability_result)
        
    def liability_read(self, address):
        '''
            Read liability from blockchain to message.
        '''
        c = self.web3.eth.contract(address, abi=self.liability_abi)
        msg = Liability()
        msg.address = address
        msg.model = b58encode(c.call().model())
        msg.objective = b58encode(c.call().objective())
        msg.promisee = c.call().promisee()
        msg.promisor = c.call().promisor()
        msg.cost = c.call().cost()
        msg.count = c.call().count()
        msg.fee = c.call().fee()
        rospy.logdebug('New liability readed: %s', msg)
        return msg

    def spin(self):
        '''
            Waiting for the new liabilities.
        '''
        liability_filter = self.builder.eventFilter('Builded')
        def liability_filter_thread():
            for entry in liability_filter.get_new_entries():
                self.liability.publish(self.liability_read(entry['args']['instance']))
            Timer(self.poll_interval, liability_filter_thread).start()
        liability_filter_thread()

        rospy.spin()
