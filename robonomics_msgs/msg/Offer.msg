# Behaviour model IPFS hash
ipfs_common/Multihash   model        # Base58
# Task IPFS hash
ipfs_common/Multihash   objective     # Base58
# Operational token
ethereum_common/Address token        # address
# Execution cost
ethereum_common/UInt256 cost         # int
# Validator network address
ethereum_common/Address validator     # address
# lighthouse network address
ethereum_common/Address lighthouse    #address
# Settlement fee
ethereum_common/UInt256 lighthouseFee # int 
# Deadline block number
ethereum_common/UInt256 deadline
# Order nonce value 
uint8[] nonce       # hex
# Order signature
uint8[] signature   # hex
