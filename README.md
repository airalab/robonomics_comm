Robonomics communication stack 
==============================

Robonomics communication ROS package stack contains common packages as a core of communication process.
Communication in robonomics consist of two parts: decentralized messaging and consensus economical transactions.

**Installation** process is similar to any ROS catkin package set.

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/airalab/robonomics_comm
catkin_init_workspace && cd .. && catkin_make 
```

Robonomics market
-----------------

Market package implements protocol part about robonomics markets.
The robonomics market is decentralized and use smart contracts and
blockchain arbitrary system for orders closing.

#### Nodes

##### Market node 

This node provide simple decentralized orders exchange through p2p networks.

Command: `rosrun robonomics_market market_node`

##### Signer node

This node sign order by sender private key. 

Command: `rosrun robonomics_market signer_node`

##### Generator node

This node generate market orders by linear function like `Q = a - k * P`. 

Command: `rosrun robonomics_market generator_node`

##### Distribution node

This node implements proportional capital market distribution approach, detailed
explanation of this approach available [here](http://ensrationis.com/smart-factory-and-capital/).

Command: `rosrun robonomics_market distribution_node`

##### Matcher node

This node track all the orders in p2p network and match it by some criterias, and when its equvalent,
the result of Ask-Bid matching can be settled down on blockchain as liability smart contract.

Command: `rosrun robonomics_market matcher_node`
