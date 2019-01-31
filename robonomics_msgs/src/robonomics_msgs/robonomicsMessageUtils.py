# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result message utils
#
from web3 import Web3
from web3.auto import w3
from robonomics_msgs.msg import Result
from eth_account.messages import defunct_hash_message
import multihash


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
