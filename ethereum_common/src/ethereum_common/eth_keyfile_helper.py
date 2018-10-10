from web3 import Web3
from eth_account.account import Account


class KeyfileHelper:
    def __init__(self, keyfile, keyfile_password_file=None, password=None):
        self.web3 = Web3()

        self.__keyfile = keyfile
        self.__keyfile_password_file = keyfile_password_file
        self.__password = password
        self.__localAccount = Account.privateKeyToAccount(self.__get_private_key_from_keyfile())

    def get_local_account_from_keyfile(self):
        return self.__localAccount

    def __get_private_key_from_keyfile(self):
        def get_password():
            if self.__password is not None:
                return self.__password
            else:
                with open(self.__keyfile_password_file, 'r') as keyfile:
                    return str(keyfile.readline()).strip('\n\r')

        with open(self.__keyfile, 'r') as keyfile:
            encrypted_key = keyfile.read()
            return self.web3.eth.account.decrypt(encrypted_key, get_password())
