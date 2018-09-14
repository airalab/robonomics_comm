import unittest
from robonomics_lighthouse import infochan
from tests.robonomics_lighthouse import testMessages


class TestSigner(unittest.TestCase):
    def setUp(self):
        self.multiplier = 2

    def test_ask2dict(self):
        self.assertEqual(testMessages.validAskDict, infochan.ask2dict(testMessages.getValidAsk()))

    def test_bid2dict(self):
        self.assertEqual(testMessages.validBidDict, infochan.bid2dict(testMessages.getValidBid()))

    def test_res2dict(self):
        self.assertEqual(testMessages.validResDict, infochan.res2dict(testMessages.getValidRes()))
