{ stdenv
, mkRosPackage
, robonomics_comm_msgs
, robonomics_comm_control
, robonomics_comm_liability
, robonomics_comm_lighthouse
, robonomics_comm_ethereum_common
, robonomics_comm_ipfs_common
}:

let
  pname = "robonomics_comm";
  version = "0.3.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_comm;

  propagatedBuildInputs =
  [ robonomics_comm_msgs
    robonomics_comm_control
    robonomics_comm_liability
    robonomics_comm_lighthouse
    robonomics_comm_ethereum_common
    robonomics_comm_ipfs_common
  ];

  meta = with stdenv.lib; {
    description = "Robonomics communication stack meta-package";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
