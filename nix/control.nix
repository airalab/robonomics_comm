{ stdenv
, mkRosPackage
, fetchFromGitHub
, robonomics_comm_lighthouse
, python3Packages
}:

let
  pname = "robonomics_control";
  version = "0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_control; 

  propagatedBuildInputs = with python3Packages; [ robonomics_comm_lighthouse web3 numpy ];

  meta = with stdenv.lib; {
    description = "Set of robonomics control algorithms";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
