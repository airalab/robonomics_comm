{ stdenv
, mkRosPackage
, python3Packages
, ros_comm
, robonomics_comm_msgs
}:

let
  pname = "ipfs_common";
  version = "0.0.1";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../ipfs_common;

  propagatedBuildInputs = with python3Packages; [ ros_comm ipfsapi robonomics_comm_msgs ];

  meta = with stdenv.lib; {
    description = "Robonomics IPFS integration node";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru maintainers.strdn ];
  };
}
