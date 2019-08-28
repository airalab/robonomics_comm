# -*- coding: utf-8 -*-
#
# Robonomics Demand/Offer/Result ipfs message converter
#

from .robonomicsMessageUtils import offer2dict, demand2dict, res2dict, dict2offer, dict2demand, dict2res
from robonomics_msgs.msg import Demand, Offer, Result
import voluptuous as v
import rospy


@v.message('wrong hexadecimal field value')
def isHexIntNotZeroCanBeEmpty(arg):
    if arg and not int(arg, 16) > 0:
        raise v.Invalid('value is zero')
    return arg


@v.message('wrong decimal field value')
def isPositiveDecIntCanBeEmpty(arg):
    if arg and not arg >= 0:
        raise v.Invalid('value is negative')
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
        v.Required('nonce'): isPositiveDecIntCanBeEmpty(),
        v.Required('signature'): isHexIntNotZeroCanBeEmpty()
    })
)

schemaResult = v.Schema({
    v.Required('liability'): isAddress,
    v.Required('result'): isIpfsBase58Hash,
    v.Required('success'): v.All(bool),
    v.Required('signature'): isHexIntNotZeroCanBeEmpty()
})

schemaAskBidResult = v.Any(
    schemaAskBid,
    schemaResult
)


def isDemandFieldsCorrect(msg):
    try:
        assert isinstance(msg, Demand)
        schemaAskBid(demand2dict(msg))
        return True
    except Exception as e:
        rospy.logerr("Fields of demand message %s is not correct. Exception is %s", msg, e)
        return False


def isOfferFieldsCorrect(msg):
    try:
        assert isinstance(msg, Offer)
        schemaAskBid(offer2dict(msg))
        return True
    except Exception as e:
        rospy.logerr("Fields of offer message %s is not correct. Exception is %s", msg, e)
        return False


def isResultFieldsCorrect(msg):
    try:
        assert isinstance(msg, Result)
        schemaResult(res2dict(msg))
        return True
    except Exception as e:
        rospy.logerr("Fields of result message %s is not correct. Exception is %s", msg, e)
        return False


def msg_is_offer_demand_result(abr_msg):
    try:
        return schemaAskBidResult(abr_msg)
    except v.MultipleInvalid as e:
        rospy.logerr("Message validation error: %s", str(e))
        return None


def convertMessage(ipfsMessage):
    validatedBySchema = msg_is_offer_demand_result(ipfsMessage)
    if not (validatedBySchema is None):
        if 'validatorFee' in validatedBySchema:
            # rospy.logwarn('DEBUG: Message %s is valid Ask message', ipfsMessage)
            return dict2demand(validatedBySchema)
        elif 'lighthouseFee' in validatedBySchema:
            # rospy.logwarn('DEBUG: Message %s is valid Bid ipfs message', ipfsMessage)
            return dict2offer(validatedBySchema)
        else:
            # rospy.logwarn('DEBUG: Message %s is valid Result ipfs message', ipfsMessage)
            return dict2res(validatedBySchema)

    rospy.logwarn('Message %s is not valid Ask, Bid or Result ipfs message', ipfsMessage)
    return None
