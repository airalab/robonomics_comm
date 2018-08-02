from robonomics_lighthouse.msg import Ask, Bid, Result
from robonomics_lighthouse import messageValidator

#ask
validAsk = {"model":"QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
            "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
            "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
            "cost":1,
            "deadline":9999999,
            "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
            "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
            "validator":"0x0000000000000000000000000000000000000000",
            "validatorFee":0
            }
#bid
validBid = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
            "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
            "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
            "cost": 1,
            "lighthouseFee": 0,
            "deadline": 9999999,
            "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
            "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}
#res
validRes = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
            'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
            'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#invalid data
someInvalidMsg = 'asdddddddd: asdaaaa'

#ask with wrong field lighthouseFee
invalidAsk1 = {"model":"QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost":1,
               "deadline":9999999,
               "lighthouseFee": 0,
               "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
               "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
               "validator":"0x0000000000000000000000000000000000000000",
               "validatorFee":0
               }
#bid with validator field
invalidBid1 = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost": 1,
               "lighthouseFee": 0,
               "deadline": 9999999,
               "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
               "validator":"0x0000000000000000000000000000000000000000",
               "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

#res with deadline field
invalidRes1 = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
               'result': 'Qmd32ebrLAsXPFQQK2LDWY6ekcvz1yDC62jGskUFNEDSbA',
               "deadline": 9999999,
               'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#ask without validator
invalidAsk2 = {"model":"QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective":"Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token":"0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost":1,
               "deadline":9999999,
               "nonce":"0ae083b24c0fe66711a43f374296aaee523d0b0c0546bf7164d743196efa1847",
               "signature":"260edc32bba1da3d2e7cbc15c8063ddf3327bc6cb92df3e6ab296d48116f49315f8406eb32e114c075c06261b47a9515c804ab82e623b5b0b26d42b77d04242d1b",
               "validatorFee":0
               }

#bid without lighthouseFee
invalidBid2 = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token": "0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost": 1,
               "deadline": 9999999,
               "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
               "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}

#res without result
invalidRes2 = {'liability': '0xcC4f10c72908D7b8A0eB7fBa70f00b135b3f97d7',
               'signature': '1392513328b145d7ce53b7cedb0bb6064fe5f85c740e3d3c98610303345390aa5a6250ea7751bc72a6ef37caab19adc14e5cde027c4ffb183a7c0895a3d82f7c1b'}

#ask with zero nonce
invalidAsk3 = {"model":"QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
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
invalidBid3 = {"model": "QmfCcLKrTCuXsf6bHbVupVv4zsbs6kjqTQ7DRftGqMLjdW",
               "objective": "Qmbm3o2wkqseSEi5F69CPAuDrsKnrwTJ3HN5FVLPgLHKUm",
               "token": "0xZ3BAF1d511Adf5098511B5c5B39e1F1b506C1AFE",
               "cost": 1,
               "lighthouseFee": 0,
               "deadline": 9999999,
               "nonce": "3f24032d1b02ab7a095bd1113be3eaf7ef4f84af0b32bd91acaf1b7d457dc5e4",
               "signature": "869d99495e019479dd0470546b2a2499c8154f6a1febd4b20a24d28079e27393023b4e9b38969223de93cc6807d162653335f7dd66ebbc492b020d2a812002c71b"}


class TestMessageConverter:
    def setUp(self):
        self.multiplier = 2
    def teardown(self):
        pass

    def test_ValidAsk(self):
        f = messageValidator.convertMessage(validAsk)
        assert isinstance(f, Ask)
    def test_ValidBid(self):
        f = messageValidator.convertMessage(validBid)
        assert isinstance(f, Bid)
    def test_ValidRes(self):
        f = messageValidator.convertMessage(validRes)
        assert isinstance(f, Result)


    def test_Wrong(self):
        f = messageValidator.convertMessage(someInvalidMsg)
        assert f is None

    def test_InvalidAsk1(self):
        f = messageValidator.convertMessage(invalidAsk1)
        assert f is None
    def test_InvalidBid1(self):
        f = messageValidator.convertMessage(invalidBid1)
        assert f is None
    def test_InvalidRes1(self):
        f = messageValidator.convertMessage(invalidRes1)
        assert f is None

    def test_InvalidAsk2(self):
        f = messageValidator.convertMessage(invalidAsk2)
        assert f is None
    def test_InvalidBid2(self):
        f = messageValidator.convertMessage(invalidBid2)
        assert f is None
    def test_InvalidRes2(self):
        f = messageValidator.convertMessage(invalidRes2)
        assert f is None

    def test_InvalidAsk3(self):
        f = messageValidator.convertMessage(invalidAsk3)
        assert f is None
    def test_InvalidBid3(self):
        f = messageValidator.convertMessage(invalidBid3)
        assert f is None