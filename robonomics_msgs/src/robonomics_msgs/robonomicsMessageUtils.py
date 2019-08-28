# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result message utils
#
from web3 import Web3
from web3.auto import w3
from robonomics_msgs.msg import Demand, Offer, Result
from eth_account.messages import defunct_hash_message
import multihash
from web3.utils.normalizers import (
    abi_ens_resolver
)


def resolve_ens_name_to_address(name_address, web3):
    (abi_type, resolved) = abi_ens_resolver(w3=web3, abi_type="address", val=name_address)
    return resolved


def convert_msg_ens_names_to_addresses(msg, web3=None):
    if web3 is not None:
        if isinstance(msg, Offer) or isinstance(msg, Demand):
            msg.token.address = resolve_ens_name_to_address(msg.token.address, web3)
            msg.lighthouse.address = resolve_ens_name_to_address(msg.lighthouse.address, web3)
            msg.validator.address = resolve_ens_name_to_address(msg.validator.address, web3)
            msg.sender.address = resolve_ens_name_to_address(msg.sender.address, web3)
        elif isinstance(msg, Result):
            msg.liability.address = resolve_ens_name_to_address(msg.liability.address, web3)
    return msg


def demand_hash(msg, nonce):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'address',
             'address',
             'uint256',
             'uint256',
             'uint256',
             'address']
    return Web3.soliditySha3(types, [multihash.decode(msg.model.multihash.encode(), "base58").encode(),
                                     multihash.decode(msg.objective.multihash.encode(), "base58").encode(),
                                     msg.token.address,
                                     int(msg.cost.uint256),
                                     msg.lighthouse.address,
                                     msg.validator.address,
                                     int(msg.validatorFee.uint256),
                                     int(msg.deadline.uint256),
                                     int(nonce.uint256),
                                     msg.sender.address])


def offer_hash(msg, nonce):
    types = ['bytes',
             'bytes',
             'address',
             'uint256',
             'address',
             'address',
             'uint256',
             'uint256',
             'uint256',
             'address']
    return Web3.soliditySha3(types, [multihash.decode(msg.model.multihash.encode(), 'base58').encode(),
                                     multihash.decode(msg.objective.multihash.encode(), 'base58').encode(),
                                     msg.token.address,
                                     int(msg.cost.uint256),
                                     msg.validator.address,
                                     msg.lighthouse.address,
                                     int(msg.lighthouseFee.uint256),
                                     int(msg.deadline.uint256),
                                     int(nonce.uint256),
                                     msg.sender.address])


def result_hash(msg):
    types = ['address',
             'bytes',
             'bool']
    return Web3.soliditySha3(types, [msg.liability.address,
                                     multihash.decode(msg.result.multihash.encode(), 'base58').encode(),
                                     msg.success])


def get_result_msg_sender_address(result):
    if isinstance(result, Result):
        message_hash = defunct_hash_message(result_hash(result))
    else:
        raise TypeError("Cannot get signer for message %s of type %s", result, type(result))

    signature = result.signature
    recovered_hash = w3.eth.account.recoverHash(message_hash, signature=signature)
    return recovered_hash
