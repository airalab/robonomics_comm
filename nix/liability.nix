{ stdenv
, mkRosPackage
, robonomics_comm_msgs
, python3Packages
}:

let
  pname = "robonomics_liability";
  version = "0.4.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_liability; 

  propagatedBuildInputs = with python3Packages; [ robonomics_comm_msgs web3 ipfsapi ];

  meta = with stdenv.lib; {
    description = "Robot liability support for ROS";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
