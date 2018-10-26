{ stdenv
, mkRosPackage
, robonomics_comm_msgs
, python3Packages
}:

let
  pname = "robonomics_control";
  version = "0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_control; 

  propagatedBuildInputs = with python3Packages; [ robonomics_comm_msgs web3 numpy ];

  meta = with stdenv.lib; {
    description = "Set of robonomics control algorithms";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
