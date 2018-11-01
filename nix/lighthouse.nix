{ stdenv
, mkRosPackage
, ros_comm
, python3Packages
, ipfs
, robonomics_comm_ethereum_common
, robonomics_comm_ipfs_common
, robonomics_comm_msgs
}:

let
  pname = "robonomics_lighthouse";
  version = "0.6.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_lighthouse; 

  propagatedBuildInputs = with python3Packages;
  [ ros_comm ipfs pexpect base58 web3 voluptuous
    robonomics_comm_ethereum_common
    robonomics_comm_ipfs_common
    robonomics_comm_msgs ];

  meta = with stdenv.lib; {
    description = "Robonomics lighthouse support for ROS";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
