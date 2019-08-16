# Behaviour model IPFS hash
ipfs_common/Multihash   model         # Base58
# Task IPFS hash
ipfs_common/Multihash   objective     # Base58
# Operational token
ethereum_common/Address token         # address
# Execution cost
ethereum_common/UInt256 cost          # int
# lighthouse network address
ethereum_common/Address lighthouse    #address
# Validator network address
ethereum_common/Address validator     # address
# Validator network fee
ethereum_common/UInt256 validatorFee  # int
# Deadline block number
ethereum_common/UInt256 deadline      # int
# Demand sender address
ethereum_common/Address sender     # address
# Nonce
ethereum_common/UInt256 nonce         # nonce
# Order signature
uint8[]                 signature    # hex
