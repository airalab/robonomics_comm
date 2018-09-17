#!/usr/bin/env python

import unittest, rostest, sys
from robonomics_lighthouse import signer
from robonomics_lighthouse import messageValidator
from robonomics_lighthouse.test import testMessages

PKG = 'robonomics_lighthouse'
NAME = 'test_signer'


class TestSigner(unittest.TestCase):

    def test_askhash(self):
        self.assertEqual(bytearray.fromhex('4f7cae2f478f7052b61e7bbd52cc8b70f7b483004d236ea370acd75620fd1000'),
                         signer.askhash(messageValidator.dict2ask(testMessages.validAskDict)))

    def test_bidhash(self):
        self.assertEqual(bytearray.fromhex('1977a06ea783dfd99ee0efa85a59deadc5b80cedc15074d487e65e4bf3580828'),
                         signer.bidhash(messageValidator.dict2bid(testMessages.validBidDict)))

    def test_reshash(self):
        self.assertEqual(bytearray.fromhex('b79fade824c5785bc17347a3c6abe50de87aa4b0414281f9010c05ea3ea7c805'),
                         signer.reshash(messageValidator.dict2res(testMessages.validResDict)))

    def test_invalidRes1_reshash(self):
        self.assertNotEqual(bytearray.fromhex('b79fade824c5785bc17347a3c6abe50de87aa4b0414281f9010c05ea3ea7c666'),
                            signer.reshash(messageValidator.dict2res(testMessages.validResDict)))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSigner, sys.argv)
