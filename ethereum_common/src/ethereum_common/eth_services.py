from ethereum_common.srv import *
from ethereum_common.eth_keyfile_helper import KeyfileHelper
from ethereum_common.eth_utils import ETHUtils
from ethereum_common.type_converters import *
import rospy
import ethereum_common.type_converters as type_converter


class ETHServices:
    def __init__(self):
        rospy.init_node('eth_node', anonymous=True)

        __keyfile_helper = KeyfileHelper(rospy.get_param('~keyfile'),
                                         keyfile_password_file=rospy.get_param('~keyfile_password_file'))
        self.__account = __keyfile_helper.get_local_account_from_keyfile()

        self.eth_utils = ETHUtils(self.__account,
                                        rospy.get_param('~web3_http_provider'),
                                        rospy.get_param('~ens_contract', None))

        def accounts_handler(m):
            return AccountsResponse([type_converter.strToAddress(self.__account.address)])
        rospy.Service('accounts', Accounts, accounts_handler)

        def account_eth_balance_handler(m):
            return AccountBalanceResponse(type_converter.strToUInt256(self.eth_utils.getBalance(m.account.address)))
        rospy.Service('account_eth_balance', AccountBalance, account_eth_balance_handler)

        def eth_balance_handler(m):
            return BalanceResponse(type_converter.strToUInt256(self.eth_utils.getBalance(self.__account.address)))
        rospy.Service('eth_balance', Balance, eth_balance_handler)

        def eth_current_block_number_handler(m):
            block = self.eth_utils.getCurrentBlockNumber
            return BlockNumberResponse(block)
        rospy.Service('current_block', BlockNumber, eth_current_block_number_handler)

    def spin(self):
        '''
            Waiting for the new messages.
        '''
        rospy.spin()
