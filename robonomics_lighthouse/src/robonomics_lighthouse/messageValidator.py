# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result ipfs message converter
#

from robonomics_msgs.msg import Demand, Offer, Result
from binascii import unhexlify
import voluptuous as v
import rospy


@v.message('wrong hexadecimal field value')
def isHexIntNotZero(arg):
    if not int(arg, 16) > 0:
        raise v.Invalid('value is zero')
    return arg


isIpfsBase58Hash = v.All(str, v.Length(min=46, max=46), v.Match(r'^[a-zA-Z0-9]+$'))
isHexadecimalString = v.All(str, v.Match(r'^0x[a-fA-F0-9]+$'))

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

        v.Required('validator'): isHexadecimalString,
        v.Required('model'): isIpfsBase58Hash,
        v.Required('objective'): isIpfsBase58Hash,
        v.Required('token'): isHexadecimalString,
        v.Required('cost'): v.All(int),
        v.Required('deadline'): v.All(int),
        v.Required('nonce'): isHexIntNotZero(),
        v.Required('signature'): isHexIntNotZero()
    })
)

schemaResult = v.Schema({
    v.Required('liability'): isHexadecimalString,
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
    msg.model = m['model']
    msg.objective = m['objective']
    msg.token = m['token']
    msg.cost = m['cost']
    msg.validator = m['validator']
    msg.validatorFee = m['validatorFee']
    msg.deadline = m['deadline']
    msg.nonce = unhexlify(m['nonce'].encode('utf-8'))
    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2bid(m):
    msg = Offer()
    msg.model = m['model']
    msg.objective = m['objective']
    msg.token = m['token']
    msg.cost = m['cost']
    msg.validator = m['validator']
    msg.lighthouseFee = m['lighthouseFee']
    msg.deadline = m['deadline']
    msg.nonce = unhexlify(m['nonce'].encode('utf-8'))
    msg.signature = unhexlify(m['signature'].encode('utf-8'))
    return msg


def dict2res(m):
    msg = Result()
    msg.liability = m['liability']
    msg.result = m['result']
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
