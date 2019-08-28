# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result ipfs message converter
#

from robonomics_msgs.msg import Demand, Offer, Result
from ethereum_common.msg import Address, UInt256
from ipfs_common.msg import Multihash
from binascii import unhexlify
import voluptuous as v
import rospy


@v.message('wrong hexadecimal field value')
def isHexIntNotZero(arg):
    if not int(arg, 16) > 0:
        raise v.Invalid('value is zero')
    return arg


isIpfsBase58Hash = v.All(str, v.Length(min=46, max=46), v.Match(r'^[a-zA-Z0-9]+$'))
isHexAddress = v.All(str, v.Length(min=42, max=42), v.Match(r'^0x[a-fA-F0-9]+$'))
isEnsName = v.All(str, v.Any(v.Match(r'^.+\.eth$'), v.Match(r'^.+\.sid$'), v.Match(r'^.+\.test$')))
isAddress = v.Any(
    isEnsName,
    isHexAddress
)

schemaAskBid = v.All(
    v.Any(
        v.Schema({
            v.Required('validatorFee'): v.All(int)}, extra=v.ALLOW_EXTRA),
        v.Schema({
            v.Required('lighthouseFee'): v.All(int)}, extra=v.ALLOW_EXTRA)
    ),
    v.Schema({
        v.Exclusive('validatorFee', 'XOR1'): object,
        v.Exclusive('lighthouseFee', 'XOR1'): object,

        v.Required('model'): isIpfsBase58Hash,
        v.Required('objective'): isIpfsBase58Hash,

        v.Required('lighthouse'): isAddress,
        v.Required('validator'): isAddress,
        v.Required('token'): isAddress,
        v.Required('cost'): v.All(int),
        v.Required('deadline'): v.All(int),
        v.Required('sender'): isAddress,
        v.Required('nonce'): v.All(int),
        v.Required('signature'): isHexIntNotZero()
    })
)

schemaResult = v.Schema({
    v.Required('liability'): isAddress,
    v.Required('result'): isIpfsBase58Hash,
    v.Required('success'): v.All(bool),
    v.Required('signature'): isHexIntNotZero()
})

schemaAskBidResult = v.Any(
    schemaAskBid,
    schemaResult
)


def dict2ask(m):
    msg = Demand()

    msg.model = Multihash()
    msg.model.multihash = m['model']

    msg.objective = Multihash()
    msg.objective.multihash = m['objective']

    msg.token = Address()
    msg.token.address = m['token']

    msg.cost = UInt256()
    msg.cost.uint256 = str(m['cost'])

    msg.lighthouse = Address()
    msg.lighthouse.address = m['lighthouse']

    msg.validator = Address()
    msg.validator.address = m['validator']

    msg.validatorFee = UInt256()
    msg.validatorFee.uint256 = str(m['validatorFee'])

    msg.deadline = UInt256()
    msg.deadline.uint256 = str(m['deadline'])

    msg.sender = Address()
    msg.sender.address = m['sender']

    msg.nonce = UInt256()
    msg.nonce.uint256 = str(m['nonce'])

    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2bid(m):
    msg = Offer()

    msg.model = Multihash()
    msg.model.multihash = m['model']

    msg.objective = Multihash()
    msg.objective.multihash = m['objective']

    msg.token = Address()
    msg.token.address = m['token']

    msg.cost = UInt256()
    msg.cost.uint256 = str(m['cost'])

    msg.validator = Address()
    msg.validator.address = m['validator']

    msg.lighthouse = Address()
    msg.lighthouse.address = m['lighthouse']

    msg.lighthouseFee = UInt256()
    msg.lighthouseFee.uint256 = str(m['lighthouseFee'])

    msg.deadline = UInt256()
    msg.deadline.uint256 = str(m['deadline'])

    msg.sender = Address()
    msg.sender.address = m['sender']

    msg.nonce = UInt256()
    msg.nonce.uint256 = str(m['nonce'])

    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2res(m):
    msg = Result()

    msg.liability = Address()
    msg.liability.address = m['liability']

    msg.result = Multihash()
    msg.result.multihash = m['result']

    msg.success = m['success']
    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def validateForAskBidResultBySchema(abr_msg):
    try:
        return schemaAskBidResult(abr_msg)
    except v.MultipleInvalid as e:
        rospy.logerr("Message validation error: %s", str(e))
        return None


def convertMessage(ipfsMessage):
    validatedBySchema = validateForAskBidResultBySchema(ipfsMessage)
    if not (validatedBySchema is None):
        if 'validatorFee' in validatedBySchema:
            # rospy.logwarn('DEBUG: Message %s is valid Ask message', ipfsMessage)
            return dict2ask(validatedBySchema)
        elif 'lighthouseFee' in validatedBySchema:
            # rospy.logwarn('DEBUG: Message %s is valid Bid ipfs message', ipfsMessage)
            return dict2bid(validatedBySchema)
        else:
            # rospy.logwarn('DEBUG: Message %s is valid Result ipfs message', ipfsMessage)
            return dict2res(validatedBySchema)

    rospy.logwarn('Message %s is not valid Ask, Bid or Result ipfs message', ipfsMessage)
    return None
