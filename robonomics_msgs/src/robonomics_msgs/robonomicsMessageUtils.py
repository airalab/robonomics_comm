# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result message utils
#
from web3 import Web3
from web3.auto import w3
from robonomics_msgs.msg import Demand, Offer, Result, AddedOrderFeedback, AddedPendingTransactionFeedback
from eth_account.messages import defunct_hash_message
import base58
from web3.utils.normalizers import (
    abi_ens_resolver
)
from web3.utils.ens import is_ens_name
from web3.exceptions import InvalidAddress
from binascii import hexlify, unhexlify
import eth_utils.address
import re

def __to_checksum_address(address, web3):
    if eth_utils.address.is_address(address):
        if eth_utils.address.is_checksum_address(address):
            return address
        else:
            return eth_utils.address.to_checksum_address(address)
    elif (re.match("^4[a-zA-Z0-9]+$", address) is not None) and (len(address) == 48):
        return address
    elif is_ens_name(address):
        return __resolve_ens_name_to_address(address, web3=web3)
    else:
        raise InvalidAddress("%a is not valid address or ENS name" % address)


def __resolve_ens_name_to_address(ens_name, web3):
    (abi_type, resolved) = abi_ens_resolver(w3=web3, abi_type="address", val=ens_name)
    return resolved


def convert_msg_ens_names_to_addresses(msg, web3=None):
    if web3 is not None:
        if isinstance(msg, Offer) or isinstance(msg, Demand):
            msg.token.address = __to_checksum_address(msg.token.address, web3)
            msg.lighthouse.address = __to_checksum_address(msg.lighthouse.address, web3)
            msg.validator.address = __to_checksum_address(msg.validator.address, web3)
            msg.sender.address = __to_checksum_address(msg.sender.address, web3)
        elif isinstance(msg, Result):
            msg.liability.address = __to_checksum_address(msg.liability.address, web3)
    return msg


def demand_hash(msg, nonce):
    if (re.match("^4[a-zA-Z0-9]+$", msg.liability.address) is not None) and (len(msg.liability.address) == 48):
        address_type = 'string'
    else:
        address_type = 'address'
    types = ['bytes',
             'bytes',
             address_type,
             'uint256',
             address_type,
             address_type,
             'uint256',
             'uint256',
             'uint256',
             address_type]
    return Web3.soliditySha3(types, [base58.b58decode(msg.model.multihash),
                                     base58.b58decode(msg.objective.multihash),
                                     msg.token.address,
                                     int(msg.cost.uint256),
                                     msg.lighthouse.address,
                                     msg.validator.address,
                                     int(msg.validatorFee.uint256),
                                     int(msg.deadline.uint256),
                                     int(nonce.uint256),
                                     msg.sender.address])


def offer_hash(msg, nonce):
    if (re.match("^4[a-zA-Z0-9]+$", msg.liability.address) is not None) and (len(msg.liability.address) == 48):
        address_type = 'string'
    else:
        address_type = 'address'
    types = ['bytes',
             'bytes',
             address_type,
             'uint256',
             address_type,
             address_type,
             'uint256',
             'uint256',
             'uint256',
             address_type]
    return Web3.soliditySha3(types, [base58.b58decode(msg.model.multihash),
                                     base58.b58decode(msg.objective.multihash),
                                     msg.token.address,
                                     int(msg.cost.uint256),
                                     msg.validator.address,
                                     msg.lighthouse.address,
                                     int(msg.lighthouseFee.uint256),
                                     int(msg.deadline.uint256),
                                     int(nonce.uint256),
                                     msg.sender.address])


def result_hash(msg):
    if (re.match("^4[a-zA-Z0-9]+$", msg.liability.address) is not None) and (len(msg.liability.address) == 48):
        address_type = 'string'
    else:
        address_type = 'address'
    types = [address_type,
            'bytes',
            'bool']
    return Web3.soliditySha3(types, [msg.liability.address,
                                     base58.b58decode(msg.result.multihash),
                                     msg.success])


def get_result_msg_sender_address(result):
    if isinstance(result, Result):
        message_hash = defunct_hash_message(result_hash(result))
    else:
        raise TypeError("Cannot get signer for message %s of type %s", result, type(result))

    signature = result.signature
    recovered_hash = w3.eth.account.recoverHash(message_hash, signature=signature)
    return recovered_hash


def offer2dict(b):
    return {
        'model': b.model.multihash,
        'objective': b.objective.multihash,
        'token': b.token.address,
        'cost': int(b.cost.uint256),
        'validator': b.validator.address,
        'lighthouse': b.lighthouse.address,
        'lighthouseFee': int(b.lighthouseFee.uint256),
        'deadline': int(b.deadline.uint256),
        'sender': b.sender.address,
        'nonce': int(b.nonce.uint256) if b.nonce.uint256 else '',
        'signature': hexlify(b.signature).decode('utf-8')
    }


def demand2dict(a):
    return {
        'model': a.model.multihash,
        'objective': a.objective.multihash,
        'token': a.token.address,
        'cost': int(a.cost.uint256),
        'lighthouse': a.lighthouse.address,
        'validator': a.validator.address,
        'validatorFee': int(a.validatorFee.uint256),
        'deadline': int(a.deadline.uint256),
        'sender': a.sender.address,
        'nonce': int(a.nonce.uint256) if a.nonce.uint256 else '',
        'signature': hexlify(a.signature).decode('utf-8')
    }


def res2dict(r):
    return {
        'liability': r.liability.address,
        'result': r.result.multihash,
        'success': r.success,
        'signature': hexlify(r.signature).decode('utf-8')
    }


def addedOrderFeedback2dict(f):
    return {
        'order': hexlify(f.order).decode('utf-8'),
        'accepted': int(f.accepted.uint256),
        'signature': hexlify(f.signature).decode('utf-8')
    }


def addedPendingTransactionFeedback2dict(f):
    return {
        'tx': f.tx.txhash
    }


def dict2demand(m):
    msg = Demand()

    msg.model.multihash = m['model']
    msg.objective.multihash = m['objective']
    msg.token.address = m['token']
    msg.cost.uint256 = str(m['cost'])
    msg.lighthouse.address = m['lighthouse']
    msg.validator.address = m['validator']
    msg.validatorFee.uint256 = str(m['validatorFee'])
    msg.deadline.uint256 = str(m['deadline'])
    msg.sender.address = m['sender']
    msg.nonce.uint256 = str(m['nonce'])
    if m['signature']:
        msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2offer(m):
    msg = Offer()

    msg.model.multihash = m['model']
    msg.objective.multihash = m['objective']
    msg.token.address = m['token']
    msg.cost.uint256 = str(m['cost'])
    msg.validator.address = m['validator']
    msg.lighthouse.address = m['lighthouse']
    msg.lighthouseFee.uint256 = str(m['lighthouseFee'])
    msg.deadline.uint256 = str(m['deadline'])
    msg.sender.address = m['sender']
    msg.nonce.uint256 = str(m['nonce'])
    if m['signature']:
        msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2res(m):
    msg = Result()

    msg.liability.address = m['liability']
    msg.result.multihash = m['result']
    msg.success = m['success']
    if m['signature']:
        msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2addedOrderFeedback(m):
    msg = AddedOrderFeedback()

    msg.order = unhexlify(m['order'].encode('utf-8'))
    msg.accepted.uint256 = str(m['accepted'])
    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2addedPendingTransactionFeedback(m):
    msg = AddedPendingTransactionFeedback()

    msg.tx.txhash = m['tx']
    return msg
