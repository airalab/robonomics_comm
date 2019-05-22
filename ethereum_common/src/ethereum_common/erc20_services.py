from web3 import Web3
from ethereum_common.msg import ApprovalEvent, TransferEvent
from ethereum_common.srv import Transfer, TransferResponse, TransferFrom, TransferFromResponse, Approve, ApproveResponse, AccountBalance, AccountBalanceResponse, Balance, BalanceResponse, AccountToAddressAllowance, AccountToAddressAllowanceResponse, Allowance, AllowanceResponse
from threading import Timer
from json import loads
from ethereum_common.eth_keyfile_helper import KeyfileHelper
from ethereum_common.eth_utils import ETHUtils
import rospy
import time
import ethereum_common.type_converters as type_converter

ERC20_TOKEN_ABI = loads('[{"constant":true,"inputs":[],"name":"name","outputs":[{"name":"","type":"string"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"spender","type":"address"},{"name":"value","type":"uint256"}],"name":"approve","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"totalSupply","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"from","type":"address"},{"name":"to","type":"address"},{"name":"value","type":"uint256"}],"name":"transferFrom","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"decimals","outputs":[{"name":"","type":"uint8"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"spender","type":"address"},{"name":"addedValue","type":"uint256"}],"name":"increaseAllowance","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[{"name":"to","type":"address"},{"name":"value","type":"uint256"}],"name":"mint","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[{"name":"value","type":"uint256"}],"name":"burn","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[{"name":"owner","type":"address"}],"name":"balanceOf","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"from","type":"address"},{"name":"value","type":"uint256"}],"name":"burnFrom","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[],"name":"symbol","outputs":[{"name":"","type":"string"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":false,"inputs":[{"name":"account","type":"address"}],"name":"addMinter","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[],"name":"renounceMinter","outputs":[],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[{"name":"spender","type":"address"},{"name":"subtractedValue","type":"uint256"}],"name":"decreaseAllowance","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":false,"inputs":[{"name":"to","type":"address"},{"name":"value","type":"uint256"}],"name":"transfer","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"nonpayable","type":"function"},{"constant":true,"inputs":[{"name":"account","type":"address"}],"name":"isMinter","outputs":[{"name":"","type":"bool"}],"payable":false,"stateMutability":"view","type":"function"},{"constant":true,"inputs":[{"name":"owner","type":"address"},{"name":"spender","type":"address"}],"name":"allowance","outputs":[{"name":"","type":"uint256"}],"payable":false,"stateMutability":"view","type":"function"},{"inputs":[{"name":"_initial_supply","type":"uint256"}],"payable":false,"stateMutability":"nonpayable","type":"constructor"},{"anonymous":false,"inputs":[{"indexed":true,"name":"account","type":"address"}],"name":"MinterAdded","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"account","type":"address"}],"name":"MinterRemoved","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"from","type":"address"},{"indexed":true,"name":"to","type":"address"},{"indexed":false,"name":"value","type":"uint256"}],"name":"Transfer","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"owner","type":"address"},{"indexed":true,"name":"spender","type":"address"},{"indexed":false,"name":"value","type":"uint256"}],"name":"Approval","type":"event"}]')


class ERC20Services:
    def __init__(self):
        rospy.init_node('erc20_node', anonymous=True)

        __keyfile_helper = KeyfileHelper(rospy.get_param('~keyfile'),
                                         keyfile_password_file=rospy.get_param('~keyfile_password_file'))
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.eth_utils = ETHUtils(self.__account,
                                  rospy.get_param('~web3_http_provider'),
                                  rospy.get_param('~web3_ws_provider'),
                                  rospy.get_param('~ens_contract', None),
                                  rospy.get_param('~token_contract'),
                                  ERC20_TOKEN_ABI)
        self.erc20 = self.eth_utils.erc20Contract
        self.erc20_ws = self.eth_utils.erc20ContractWS
        self.factory_address = self.eth_utils.getAddressByName(rospy.get_param('~factory_contract'))

        self.initialize_event_filters()
        self.web3 = Web3()

        self.transfer = rospy.Publisher('event/transfer', TransferEvent, queue_size=10)
        self.approval = rospy.Publisher('event/approval', ApprovalEvent, queue_size=10)

        def transfer_handler(m):
            nonce = self.eth_utils.getAccountTransactionCount()
            tx = self.erc20.functions.transfer(m.to.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.eth_utils.signAndSendTransaction(tx=tx)
            return TransferResponse(sent_out)
        rospy.Service('transfer', Transfer,
                      transfer_handler)

        def transfer_from_handler(m):
            nonce = self.eth_utils.getAccountTransactionCount()
            tx = self.erc20.functions.transferFrom(m.owner.address, m.to.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.eth_utils.signAndSendTransaction(tx=tx)
            return TransferFromResponse(sent_out)
        rospy.Service('transfer_from', TransferFrom,
                      transfer_from_handler)

        def approve_handler(m):
            nonce = self.eth_utils.getAccountTransactionCount()
            tx = self.erc20.functions.approve(m.spender.address, int(m.value.uint256))\
                .buildTransaction({'nonce': nonce})
            sent_out = self.eth_utils.signAndSendTransaction(tx=tx)
            return ApproveResponse(sent_out)
        rospy.Service('approve', Approve,
                      approve_handler)

        def erc20_balance_handler(m):
            return AccountBalanceResponse(type_converter.strToUInt256(self.eth_utils.getTokenBalance(m.account.address)))
        rospy.Service('erc20_balance', AccountBalance, erc20_balance_handler)

        def erc20_balance_of_handler(m):
            return BalanceResponse(type_converter.strToUInt256(self.eth_utils.getTokenBalance(self.__account.address)))
        rospy.Service('erc20_balance_of', Balance, erc20_balance_of_handler)

        def account_to_address_erc20_allowance_handler(m):
            amount_allowance = self.eth_utils.getAllowance(m.account.address, m.to.address)
            return AccountToAddressAllowanceResponse(type_converter.strToUInt256(amount_allowance))
        rospy.Service('erc20_allowance', AccountToAddressAllowance, account_to_address_erc20_allowance_handler)

        def erc20_allowance_of_handler(m):
            # account from keyfile to default factory allowance
            amount_allowance = self.eth_utils.getAllowance(self.__account.address, self.factory_address)
            return AllowanceResponse(type_converter.strToUInt256(amount_allowance))
        rospy.Service('erc20_allowance_of', Allowance, erc20_allowance_of_handler)

    def initialize_event_filters(self):
        try:
            self.transfer_filter = self.erc20_ws.eventFilter('Transfer')
            self.approval_filter = self.erc20_ws.eventFilter('Approval')
        except Exception as e:
            rospy.logwarn("Failed to reinitialize erc20 event filters with exception: \"%s\"", e)

    def spin(self):
        def filter_thread():
            try:
                for e in self.transfer_filter.get_new_entries():
                    self.transfer.publish(type_converter.filterEntryToTransferEvent(e['args']))

                for e in self.approval_filter.get_new_entries():
                    self.approval.publish(type_converter.filterEntryToApprovalEvent(e['args']))
            except Exception as e:
                rospy.logerr('erc20 node filters get entries exception: %s', e)
                time.sleep(10)
                self.initialize_event_filters()
            Timer(1, filter_thread).start()
        filter_thread()

        rospy.spin()
