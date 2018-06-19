from web3 import Web3, HTTPProvider
from ens import ENS
from ethereum_common.msg import *
from ethereum_common.srv import *
from threading import Timer
from json import loads
import rospy

ABI = loads('[{"constant":false,"inputs":[{"name":"_spender","type":"address"},{"name":"_value","type":"uint256"}],"name":"approve","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"totalSupply","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"_from","type":"address"},{"name":"_to","type":"address"},{"name":"_value","type":"uint256"}],"name":"transferFrom","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[{"name":"_owner","type":"address"}],"name":"balanceOf","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"_to","type":"address"},{"name":"_value","type":"uint256"}],"name":"transfer","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[{"name":"_owner","type":"address"},{"name":"_spender","type":"address"}],"name":"allowance","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"anonymous":false,"inputs":[{"indexed":true,"name":"_from","type":"address"},{"indexed":true,"name":"_to","type":"address"},{"indexed":false,"name":"_value","type":"uint256"}],"name":"Transfer","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"_owner","type":"address"},{"indexed":true,"name":"_spender","type":"address"},{"indexed":false,"name":"_value","type":"uint256"}],"name":"Approval","type":"event"}]')

def transferEvent(args):
    m = TransferEvent()
    m.args_from.address  = args['_from']
    m.args_to.address    = args['_to']
    m.args_value.uint256 = str(args['_value'])
    return m

def approvalEvent(args):
    m = ApprovalEvent()
    m.args_owner.address   = args['_owner']
    m.args_spender.address = args['_spender']
    m.args_value.uint256   = str(args['_value'])
    return m

def strToAddress(s):
    a = Address()
    a.address = s
    return a

class Node:
    def __init__(self):
        rospy.init_node('erc20_node', anonymous=True)

        http_provider = HTTPProvider(rospy.get_param('~web3_http_provider'))
        self.ens = ENS(http_provider, addr=rospy.get_param('~ens_contract', None))
        self.web3 = Web3(http_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        token_address = self.ens.address(rospy.get_param('~token_contract')) 
        self.erc20 = self.web3.eth.contract(token_address, abi=ABI)

        self.transfer = rospy.Publisher('event/transfer', TransferEvent, queue_size=10)
        self.approval = rospy.Publisher('event/approval', ApprovalEvent, queue_size=10)

        rospy.Service('transfer', Transfer,
            lambda m: TransferResponse(self.erc20.functions.transfer(m.to.address, int(m.value.uint256)).transact()))

        rospy.Service('transfer_from', TransferFrom,
            lambda m: TransferFromResponse(self.erc20.functions.transferFrom(m.owner.address, m.to.address, int(m.value.uint256)).transact()))

        rospy.Service('approve', Approve,
            lambda m: ApproveResponse(self.erc20.functions.approve(m.spender.address, int(m.value.uint256)).transact()))

        rospy.Service('accounts', Accounts,
            lambda m: AccountsResponse(list(map(strToAddress, self.web3.eth.accounts))))

    def spin(self):
        transfer_filter = self.erc20.eventFilter('Transfer')
        approval_filter = self.erc20.eventFilter('Approval')

        def filter_thread():
            for e in transfer_filter.get_new_entries():
                self.transfer.publish(transferEvent(e['args']))

            for e in approval_filter.get_new_entries():
                self.approval.publish(approvalEvent(e['args']))

            Timer(1, filter_thread).start()
        filter_thread()

        rospy.spin()
