from ethereum_common.msg import *

def strToAddress(s):
    a = Address()
    a.address = s
    return a

def strToUInt256(s):
    u = UInt256()
    u.uint256 = str(s)
    return u

def filterEntryToTransferEvent(args):
    m = TransferEvent()
    m.args_from.address  = args['_from']
    m.args_to.address    = args['_to']
    m.args_value.uint256 = str(args['_value'])
    return m

def filterEntryToApprovalEvent(args):
    m = ApprovalEvent()
    m.args_owner.address   = args['_owner']
    m.args_spender.address = args['_spender']
    m.args_value.uint256   = str(args['_value'])
    return m