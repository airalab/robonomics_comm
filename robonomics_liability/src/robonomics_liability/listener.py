# -*- coding: utf-8 -*-
#
# Robonomics liability tracking node.
#

from robonomics_msgs.msg import Result
from ipfs_common.msg import Multihash
from robonomics_liability.msg import Liability
from web3 import Web3, HTTPProvider, WebsocketProvider
from ens import ENS
from threading import Timer
import rospy, json, time, multihash
from std_msgs.msg import String
from . import finalization_checker
from persistent_queue import PersistentQueue


class Listener:
    def __init__(self):
        '''
            Robonomics liability tracking node initialisation.
        '''
        rospy.init_node('robonomics_liability_listener')

        web3_http_provider = rospy.get_param('~web3_http_provider')
        http_provider = HTTPProvider(web3_http_provider)

        web3_ws_provider = rospy.get_param('~web3_ws_provider')
        ws_provider = WebsocketProvider(web3_ws_provider)

        ens_contract = rospy.get_param('~ens_contract', None)

        self.ens = ENS(http_provider, addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)

        self.web3ws = Web3(ws_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        self.poll_interval = rospy.get_param('~poll_interval', 5)

        self.liability = rospy.Publisher('incoming', Liability, queue_size=10)

        self.create_liability_filter()

        self.liability_abi = json.loads(rospy.get_param('~liability_contract_abi'))

        self.liability_finalization_checker = finalization_checker.FinalizationChecker(self.liability_abi,
                                                                                       web3_http_provider=web3_http_provider,
                                                                                       ens_contract=ens_contract)
        self.finalized = rospy.Publisher('finalized', String, queue_size=10)

        self.liabilities_queue = PersistentQueue('robonomics_liability_listener.queue')

        self.result_handler()

    def create_liability_filter(self):
        try:
            factory_abi = json.loads(rospy.get_param('~factory_contract_abi'))
            factory_address = self.ens.address(rospy.get_param('~factory_contract'))
            factory = self.web3ws.eth.contract(factory_address, abi=factory_abi)
            self.liability_filter = factory.eventFilter('NewLiability')
        except Exception as e:
            rospy.logwarn("Failed to create liability filter with exception: \"%s\"", e)

    def result_handler(self):
        result = rospy.Publisher('infochan/eth/signing/result', Result, queue_size=10)

        def liability_finalize(msg):
            is_finalized = False
            while is_finalized is not True:
                result.publish(msg)
                time.sleep(30)
                # TODO: move sleep time to rop parameter with 30 seconds by default
                is_finalized = self.liability_finalization_checker.finalized(msg.liability)
            self.finalized.publish(msg.liability)

        rospy.Subscriber('result', Result, liability_finalize)

    def liability_read(self, address):
        '''
            Read liability from blockchain to message.
        '''
        c = self.web3.eth.contract(address, abi=self.liability_abi)
        msg = Liability()

        model_mh = Multihash()
        model_mh.multihash = multihash.decode(c.call().model()).encode('base58').decode()

        objective_mh = Multihash()
        objective_mh.multihash = multihash.decode(c.call().objective()).encode('base58').decode()

        msg.address = address
        msg.model = model_mh
        msg.objective = objective_mh
        msg.promisee.address = c.call().promisee()
        msg.promisor.address = c.call().promisor()
        msg.lighthouse.address = c.call().lighthouse()
        msg.token.address = c.call().token()
        msg.cost.uint256 = c.call().cost()
        msg.validator.address = c.call().validator()
        msg.validatorFee.address = c.call().validatorFee()
        rospy.logdebug('New liability readed: %s', msg)
        return msg

    def spin(self):
        '''
            Waiting for the new liabilities.
        '''

        def liability_filter_thread():
            try:
                for entry in self.liability_filter.get_new_entries():
                    liability_address = entry['args']['liability']
                    self.liabilities_queue.push(liability_address)
                    rospy.loginfo("New liability added to queue: %s", liability_address)
            except Exception as e:
                rospy.logerr('listener liability filter exception: %s', e)
                self.create_liability_filter()
            Timer(self.poll_interval, liability_filter_thread).start()

        def liabilities_queue_handler():
            entry = self.liabilities_queue.peek()
            if entry is not None:
                try:
                    self.liability.publish(self.liability_read(entry))
                    self.liabilities_queue.pop()
                    rospy.loginfo("Liability read successfully: %s", entry)
                except Exception as e:
                    rospy.logerr('Liability %s read exception: %s', entry, e)
            Timer(self.poll_interval, liabilities_queue_handler).start()

        liability_filter_thread()
        liabilities_queue_handler()
        rospy.spin()
