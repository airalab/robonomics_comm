from robonomics_lighthouse.msg import Ask, Bid, Result
from binascii import unhexlify

#ask
validAskDict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
            "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
            "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
            "cost":1,
            "validator": "0x0000000000000000000000000000000000000000",
            "validatorFee": 0,
            "deadline":9999999,
            "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
            "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b"
                }

def getValidAsk():
    a = Ask()
    a.model = validAskDict['model']
    a.objective = validAskDict['objective']
    a.token = validAskDict['token']
    a.cost = validAskDict['cost']
    a.validator = validAskDict['validator']
    a.validatorFee = validAskDict['validatorFee']
    a.deadline = validAskDict['deadline']
    a.nonce = unhexlify(validAskDict['nonce'].encode('utf-8'))
    a.signature = unhexlify(validAskDict['signature'].encode('utf-8'))
    return a

#bid
validBidDict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
            "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
            "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
            "cost": 1,
            "validator": '0x0000000000000000000000000000000000000000',
            "lighthouseFee": 0,
            "deadline": 9999999,
            "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
            "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}


def getValidBid():
    b = Bid()
    b.model = validBidDict['model']
    b.objective = validBidDict['objective']
    b.token = validBidDict['token']
    b.cost = validBidDict['cost']
    b.validator = validBidDict['validator']
    b.lighthouseFee = validBidDict['lighthouseFee']
    b.deadline = validBidDict['deadline']
    b.nonce = unhexlify(validBidDict['nonce'].encode('utf-8'))
    b.signature = unhexlify(validBidDict['signature'].encode('utf-8'))
    return b


# res
validResDict = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
                'success': True,
            'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
            'signature': 'b14567adc9078cf838233f97568075fc706459d7e874407faecc90b3b82dc7b3604643d97720da197ba7fdda7728db003f6cbae8a59f8af176dd39c3766faeac2d'}

def getValidRes():
    r = Result()
    r.liability = validResDict['liability']
    r.result = validResDict['result']
    r.success = validResDict['success']
    r.signature = unhexlify(validResDict['signature'].encode('utf-8'))
    return r


#invalid data
someInvalidMsgDict = 'asdddddddd: asdaaaa'

#ask with wrong field lighthouseFee
invalidAsk1Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
                "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
                "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
                "cost":1,
                "deadline":9999999,
                "lighthouseFee": 0,
                "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
                "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
                "validator":"0x0000000000000000000000000000000000000000",
                "validatorFee":0}

#bid with validatorFee field
invalidBid1Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
                "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
                "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
                "cost": 1,
                "validator":"0x0000000000000000000000000000000000000000",
                "validatorFee": 0,
                "lighthouseFee": 0,
                "deadline": 9999999,
                "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
                "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

#res with deadline field
invalidRes1Dict = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
                'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
                   'succcess': False,
                "deadline": 9999999,
                'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#ask without validator
invalidAsk2Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost":1,
               "deadline":9999999,
               "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
               "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
                   "validatorFee":0
                   }

#bid without lighthouseFee
invalidBid2Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost": 1,
               "validator": '0x0000000000000000000000000000000000000000',
               "deadline": 9999999,
               "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
               "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

#res without result
invalidRes2Dict = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
                   'success': True,
               'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#ask with zero nonce
invalidAsk3Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost":1,
               "deadline":9999999,
               "nonce":"0",
               "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
               "validator":"0x0000000000000000000000000000000000000000",
                   "validatorFee":0
                   }

#bid with invalid token
invalidBid3Dict = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token": "0xZ3BAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost": 1,
               "validator": '0x0000000000000000000000000000000000000000',
               "lighthouseFee": 0,
               "deadline": 9999999,
               "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
               "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

# res without success flag
invalidRes3Dict = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
            'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
            'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

