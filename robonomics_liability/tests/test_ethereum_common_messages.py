#!/usr/bin/env python

import unittest
import rostest
import sys
from ethereum_common.msg import UInt256

PKG = 'robonomics_liability'
NAME = 'test_ethereum_common_messages'


class TestEthereumCommonMessages(unittest.TestCase):
    def setUp(self):
        self.multiplier = 2

    def test_UInt256_comparison(self):
        u1 = UInt256("100")
        u2 = UInt256("101")
        u3 = UInt256("101")

        # test comparisons with UInt256
        self.assertGreater(u2, u1)
        self.assertLess(u1, u2)

        self.assertGreaterEqual(u2, u1)
        self.assertGreaterEqual(u3, u1)
        self.assertGreaterEqual(u3, u2)
        self.assertGreaterEqual(u2, u3)

        self.assertLessEqual(u1, u2)
        self.assertLessEqual(u1, u3)
        self.assertLessEqual(u3, u2)
        self.assertLessEqual(u2, u3)

        self.assertEqual(u2, u3)

        # comparison with int
        self.assertGreater(u2, 100)
        self.assertLess(u1, 101)

        self.assertGreaterEqual(u2, 100)
        self.assertGreaterEqual(u3, 100)
        self.assertGreaterEqual(u3, 101)

        self.assertLessEqual(u1, 101)
        self.assertLessEqual(u3, 101)

        self.assertEqual(u1, 100)

    def test_UInt256_arithmetic(self):
        u1 = UInt256("100")
        u2 = UInt256("101")

        u1 += 1
        self.assertEqual(u1, u2)

        u1 = UInt256("100")
        u3 = u1 + 1
        self.assertEqual(u3, u2)

        u3 = u1 + UInt256("1")
        self.assertEqual(u3, u2)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestEthereumCommonMessages, sys.argv)


