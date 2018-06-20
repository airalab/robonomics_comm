# -*- coding: utf-8 -*-
#
# Robonomics lighthouse functionality.
#

from robonomics_lighthouse.msg import Deal, Result
from web3 import Web3, HTTPProvider
from ens import ENS
from binascii import hexlify, unhexlify
from base58 import b58decode
import rospy, json

from .quota import * 

class Lighthouse:
    def __init__(self):
        '''
            Market reporter initialisation.
        '''
        rospy.init_node('robonomics_lighthouse')

        http_provider = HTTPProvider(rospy.get_param('~web3_http_provider'))
        ens = ENS(http_provider, addr=rospy.get_param('~ens_contract', None))
        self.web3 = Web3(http_provider, ens=ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        self.liability_abi = json.loads(rospy.get_param('~liability_abi'))

        lighthouse_abi = json.loads(rospy.get_param('~lighthouse_abi'))
        lighthouse_contract = rospy.get_param('~lighthouse_contract')
        self.lighthouse = self.web3.eth.contract(lighthouse_contract, abi=lighthouse_abi)

        self.account = rospy.get_param('~account_address', self.web3.eth.accounts[0])

        rospy.Subscriber('infochan/incoming/result', Result, self.settle_result)
        rospy.Subscriber('deal', Deal, self.settle_deal)

        self.setupGasStrategy()
        self.createQuotaManager()

    def setupGasStrategy(self):
        def gas_price_strategy(web3, transaction_params):
            # TODO: adaptive gas price strategy
            return Web3.toWei(4, 'gwei')
        self.web3.eth.setGasPriceStrategy(gas_price_strategy)

    def createQuotaManager(self):
        def transact(q):
            tx = q.get()
            try:
                tx['gas'] = self.web3.eth.estimateGas(tx)
                rospy.loginfo('Transaction GAS %d', tx['gas'])
                txhash = self.web3.eth.sendTransaction(tx) 
                rospy.loginfo('Transaction sended at %s', Web3.toHex(txhash))

                while not self.web3.eth.getTransactionReceipt(txhash):
                    rospy.sleep(15)
                rospy.loginfo('Transaction mined at %s', Web3.toHex(txhash))
            except Exception as e:
                rospy.logerr('Broken transaction: %s', e)
            q.task_done()

        def marker():
            m = self.lighthouse.call().marker()
            q = self.lighthouse.call().quota()
            keepalive = self.lighthouse.call().timeoutBlocks() + self.lighthouse.call().keepaliveBlock()

            rospy.loginfo('Lighthouse m: %d q: %d k: %d, b: %d', m, q, keepalive, self.web3.eth.blockNumber)

            if self.web3.eth.blockNumber > keepalive:
                # Timeout
                return True

            if q == 0:
                try:
                    # Next is me
                    return self.lighthouse.call().members(m + 1) == self.account
                except:
                    # Me on the start of list
                    return self.lighthouse.call().members(0) == self.account

            return False

        def quota():
            if self.lighthouse.call().members(self.lighthouse.call().marker()) == self.account:
                q = self.lighthouse.call().quota()
                if q > 0:
                    return q
            return self.lighthouse.call().quotaOf(self.account)

        self.manager = QuotaManager(quota, transact, marker) 
        self.manager.start()

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()

    def settle_deal(self, msg):
        '''
            Create liability contract for matched ask & bid.
        '''
        liability = self.web3.eth.contract('0x0000000000000000000000000000000000000000', abi=self.liability_abi)

        def encodeAsk(msg):
            args = [ b58decode(msg.model)
                   , b58decode(msg.objective)
                   , msg.token
                   , msg.cost
                   , msg.validator
                   , msg.validatorFee
                   , msg.deadline
                   , msg.nonce
                   , msg.signature ]
            return '0x' + liability.functions.ask(*args).buildTransaction()['data'][10:]

        def encodeBid(msg):
            args = [ b58decode(msg.model)
                   , b58decode(msg.objective)
                   , msg.token
                   , msg.cost
                   , msg.lighthouseFee
                   , msg.deadline
                   , msg.nonce
                   , msg.signature ]
            return '0x' + liability.functions.bid(*args).buildTransaction()['data'][10:]

        tx = self.lighthouse.functions.createLiability(encodeAsk(msg.ask), encodeBid(msg.bid))\
            .buildTransaction({'gas': 1000000, 'from': self.account})
        self.manager.put(tx)

    def settle_result(self, msg):
        '''
            Settle incoming result.
        '''
        liability = self.web3.eth.contract(msg.liability, abi=self.liability_abi)
        data = liability.functions.finalize(
            b58decode(msg.result),
            msg.signature,
            False).buildTransaction({'gas': 1000000})['data']
        tx = self.lighthouse.functions.to(msg.liability, data)\
            .buildTransaction({'gas': 1000000, 'from': self.account})
        self.manager.put(tx)
