Robonomics communication stack 
==============================
[![Build Status](https://travis-ci.org/airalab/robonomics_comm.svg?branch=master)](https://travis-ci.org/airalab/robonomics_comm)
![BSD3 License](http://img.shields.io/badge/license-BSD3-brightgreen.svg)

> This is reference implementation of robot economics protocol.

Robonomics communication stack contains a set of ROS packages for economical communication purposes.

**Installation** process is similar to any ROS catkin package set.

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/airalab/robonomics_comm
catkin_init_workspace && cd .. && catkin_make 
```

> `robonomics_comm` also can be builded with Nix, `nix build -f release.nix`.

There is a script made especially to simplify installation and usage of robonomics_comm

Download the following [script](http://bootstrap.aira.life/robonomics.sh) and run it:

```
wget http://bootstrap.aira.life/robonomics.sh
chmod +x ./robonomics.sh
./robonomics.sh init        # for the first time only
./robonomics.sh mainnet     # or sidechain for Sidechain network
```

Robonomics liability
--------------------

Liability package implements protocol part about robot liability
smart contract actions. It provide methods for robot task/result
store, delivery and interpretation.

Robonomics control
------------------

This packages implements robonomics control rules described at [article](http://ensrationis.com/smart-factory-and-capital/).

Examples
--------

```bash
# Launch liability
. ws/devel/setup.bash
roslaunch robonomics_liability liability.launch \
    keyfile:="$WORKSPACE/keyfile" \
    keyfile_password_file:="$WORKSPACE/keyfile_password_file" \
    web3_http_provider:="https://mainnet.infura.io/v3/cd7368514cbd4135b06e2c5581a4fff7" \
    web3_ws_provider:="wss://mainnet.infura.io/ws"
```

Testing
-------

[Rostest](http://wiki.ros.org/rostest) framework is used for Robonomics communication stack testing.

Run test command:

```
make test
```

> [Nix](https://nixos.org/nix/) is requred for testing.
