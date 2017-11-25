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

Robonomics liability
--------------------



Robonomics market
-----------------

Market package implements protocol part about robonomics markets.
The robonomics market is decentralized and use smart contracts and
blockchain arbitrary system for orders closing.

Robonomics control
------------------

This packages implements robonomics control rules described at [article](http://ensrationis.com/smart-factory-and-capital/).
