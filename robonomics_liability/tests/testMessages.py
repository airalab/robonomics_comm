from robonomics_msgs.msg import Demand, Offer, Result
from ethereum_common.msg import Address, UInt256
from ipfs_common.msg import Multihash
from binascii import unhexlify

# ask
validAskDict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "validator": "0x0000000000000000000000000000000000000000",
    "validatorFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b"
}

def getValidAsk():
    a = Demand()
    a.model = Multihash()
    a.model.multihash = validAskDict['model']
    a.objective = Multihash()
    a.objective.multihash = validAskDict['objective']
    a.token = Address()
    a.token.address = validAskDict['token']
    a.cost = UInt256()
    a.cost.uint256 = str(validAskDict['cost'])
    a.lighthouse = Address()
    a.lighthouse.address = validAskDict['lighthouse']
    a.validator = Address()
    a.validator.address = validAskDict['validator']
    a.validatorFee = UInt256()
    a.validatorFee.uint256 = str(validAskDict['validatorFee'])
    a.deadline = UInt256()
    a.deadline.uint256 = str(validAskDict['deadline'])
    a.sender = Address()
    a.sender.address = validAskDict['sender']
    a.nonce = UInt256()
    a.nonce.uint256 = str(validAskDict['nonce'])
    a.signature = unhexlify(validAskDict['signature'].encode('utf-8'))
    return a

#bid
validBidDict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "validator": '0x0000000000000000000000000000000000000000',
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "lighthouseFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"
}


def getValidBid():
    b = Offer()
    b.model = Multihash()
    b.model.multihash = validBidDict['model']
    b.objective = Multihash()
    b.objective.multihash = validBidDict['objective']
    b.token = Address()
    b.token.address = validBidDict['token']
    b.cost = UInt256()
    b.cost.uint256 = str(validBidDict['cost'])
    b.validator = Address()
    b.validator.address = validBidDict['validator']
    b.lighthouse = Address()
    b.lighthouse.address = validBidDict['lighthouse']
    b.lighthouseFee = UInt256()
    b.lighthouseFee.uint256 = str(validBidDict['lighthouseFee'])
    b.deadline = UInt256()
    b.deadline.uint256 = str(validBidDict['deadline'])
    b.sender = Address()
    b.sender.address = validAskDict['sender']
    b.nonce = UInt256()
    b.nonce.uint256 = str(validAskDict['nonce'])
    b.signature = unhexlify(validBidDict['signature'].encode('utf-8'))
    return b

# res
validResDict = {
    'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
    'success': True,
    'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
    'signature': 'b14567adc9078cf838233f97568075fc706459d7e874407faecc90b3b82dc7b3604643d97720da197ba7fdda7728db003f6cbae8a59f8af176dd39c3766faeac2d'
}

def getValidRes():
    r = Result()
    r.liability = Address()
    r.liability.address = validResDict['liability']
    r.result = Multihash()
    r.result.multihash = validResDict['result']
    r.success = validResDict['success']
    r.signature = unhexlify(validResDict['signature'].encode('utf-8'))
    return r


#invalid data
someInvalidMsgDict = 'asdddddddd: asdaaaa'

#ask with wrong field lighthouseFee
invalidAsk1Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "deadline": 9999999,
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "lighthouseFee": 0,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
    "validator": "0x0000000000000000000000000000000000000000",
    "validatorFee": 0
}

#bid with validatorFee field
invalidBid1Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "validator": "0x0000000000000000000000000000000000000000",
    "validatorFee": 0,
    "lighthouseFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

#res with deadline field
invalidRes1Dict = {
    'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
    'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
    'succcess': False,
    "deadline": 9999999,
    'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#ask without validator
invalidAsk2Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "cost": 1,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
    "nonce": 777,
    "validatorFee": 0
}

#bid without lighthouseFee
invalidBid2Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "validator": '0x0000000000000000000000000000000000000000',
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"
}

#res without result
invalidRes2Dict = {
    'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
    'success': True,
    'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'
}

#ask with wrong sender
invalidAsk3Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F3",
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
    "validator": "0x0000000000000000000000000000000000000000",
    "nonce": 777,
    "validatorFee": 0
}

#bid with invalid token
invalidBid3Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0xZ3BAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "validator": '0x0000000000000000000000000000000000000000',
    "lighthouse": "0x0000000000000000000000000000000000000000",
    "lighthouseFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"
}

# res without success flag
invalidRes3Dict = {
    'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
    'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
    'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'
}


#ask without lighthouse field
invalidAsk4Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "validator": "0x0000000000000000000000000000000000000000",
    "validatorFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b"
}

#bid withouse lighthouse field
invalidBid4Dict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "validator": '0x0000000000000000000000000000000000000000',
    "lighthouseFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"
}

validAskWithENSNamesDict = {
    "model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
    "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
    "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
    "cost": 1,
    "lighthouse": "airalab.lighthouse.5.robonomics.sid",
    "validator": "0x0000000000000000000000000000000000000000",
    "validatorFee": 0,
    "deadline": 9999999,
    "sender": "0x004ec07d2329997267Ec62b4166639513386F32E",
    "nonce": 777,
    "signature": "260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b"
}
