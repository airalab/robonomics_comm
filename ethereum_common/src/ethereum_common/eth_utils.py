from web3 import Web3, HTTPProvider
from ens import ENS


class ETHUtils:
    def __init__(self, account, web3_http_provider, ens_contract, erc20_token=None, abi=None):
        http_provider = HTTPProvider(web3_http_provider)
        self.ens = ENS(http_provider,  addr=ens_contract)
        self.web3 = Web3(http_provider, ens=self.ens)

        from web3.middleware import geth_poa_middleware
        # inject the poa compatibility middleware to the innermost layer
        self.web3.middleware_stack.inject(geth_poa_middleware, layer=0)
        self.ens.web3.middleware_stack.inject(geth_poa_middleware, layer=0)

        if erc20_token is not None and abi is not None:
            token_address = self.ens.address(erc20_token)
            self.erc20 = self.web3.eth.contract(token_address, abi=abi)
            self.erc20_balance_delimiter = (10 ** self.erc20.functions.decimals().call())

        self.__account = account

    @property
    def erc20Contract(self):
        return self.erc20

    def getAddressByName(self, name):
        return self.ens.address(name)

    def getAccountTransactionCount(self):
        return self.web3.eth.getTransactionCount(self.__account.address)

    def signTransaction(self, tx):
        return self.__account.signTransaction(tx)

    def sendRawTransaction(self, raw_tx):
        return self.web3.eth.sendRawTransaction(raw_tx)

    def signAndSendTransaction(self, tx):
        signed_tx = self.signTransaction(tx=tx)
        return self.sendRawTransaction(raw_tx=signed_tx.rawTransaction)

    def getTransaction(self, tx_hash):
        return self.web3.eth.getTransaction(tx_hash)

    def getCurrentBlockNumber(self):
        block = self.getBlock('latest')
        return self.__getBlockNumberFromBlock(block)


    def __getBlockNumberFromBlock(self, block):
        return block['number']

    def getBlock(self, number):
        return self.web3.eth.getBlock(number)

    def getBalance(self, account):
        return self.web3.fromWei(self.web3.eth.getBalance(account), 'ether')

    def getTokenBalance(self, account):
        return self.erc20.functions.balanceOf(account).call() / self.erc20_balance_delimiter

    def getAllowance(self, account, address):
        return self.erc20.functions.allowance(account, address).call()