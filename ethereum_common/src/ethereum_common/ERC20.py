from web3 import Web3
from ethereum_common.msg import *
from ethereum_common.srv import *
from threading import Timer
from json import loads
from ethereum_common.eth_keyfile_helper import KeyfileHelper
from ethereum_common.eth_parity_utils import ParityUtils
import rospy, time

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

        __keyfile_helper = KeyfileHelper(rospy.get_param('~keyfile'),
                                         keyfile_password_file=rospy.get_param('~keyfile_password_file'))
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.parity_utils = ParityUtils(self.__account,
                                        rospy.get_param('~web3_http_provider'),
                                        rospy.get_param('~ens_contract', None),
                                        rospy.get_param('~token_contract'),
                                        ABI)
        self.erc20 = self.parity_utils.getERC20Contract()
        self.initialize_event_filters()
        self.web3 = Web3()

        self.transfer = rospy.Publisher('event/transfer', TransferEvent, queue_size=10)
        self.approval = rospy.Publisher('event/approval', ApprovalEvent, queue_size=10)

        def transfer_handler(m):
            nonce = self.parity_utils.getAccountTransactionCount()
            tx = self.erc20.functions.transfer(m.to.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.parity_utils.signAndSendTransaction(tx=tx)
            return TransferResponse(sent_out)
        rospy.Service('transfer', Transfer,
                      transfer_handler)

        def transfer_from_handler(m):
            nonce = self.parity_utils.getAccountTransactionCount()
            tx = self.erc20.functions.transferFrom(m.owner.address, m.to.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.parity_utils.signAndSendTransaction(tx=tx)
            return TransferFromResponse(sent_out)
        rospy.Service('transfer_from', TransferFrom,
                      transfer_from_handler)

        def approve_handler(m):
            nonce = self.parity_utils.getAccountTransactionCount()
            tx = self.erc20.functions.approve(m.spender.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.parity_utils.signAndSendTransaction(tx=tx)
            return ApproveResponse(sent_out)
        rospy.Service('approve', Approve,
                      approve_handler)

        rospy.Service('accounts', Accounts,
            lambda m: AccountsResponse([strToAddress(self.__account.address)]))

    def initialize_event_filters(self):
        try:
            self.transfer_filter = self.erc20.eventFilter('Transfer')
            self.approval_filter = self.erc20.eventFilter('Approval')
        except Exception as e:
            rospy.logwarn("Failed to reinitialize erc20 event filters with exception: \"%s\"", e)

    def spin(self):
        def filter_thread():
            try:
                for e in self.transfer_filter.get_new_entries():
                    self.transfer.publish(transferEvent(e['args']))

                for e in self.approval_filter.get_new_entries():
                    self.approval.publish(approvalEvent(e['args']))
            except Exception as e:
                rospy.logerr('ecr20 node filters get entries exception: %s', e)
                time.sleep(10)
                self.initialize_event_filters()
            Timer(1, filter_thread).start()
        filter_thread()

        rospy.spin()
