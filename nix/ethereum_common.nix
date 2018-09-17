{ stdenv
, mkRosPackage
, fetchFromGitHub
, robonomics_comm_lighthouse
, python3Packages
, ros_comm
}:

let
  pname = "ethereum_common";
  version = "0.1.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../ethereum_common;

  propagatedBuildInputs = with python3Packages; [ ros_comm web3 ];

  meta = with stdenv.lib; {
    description = "Commonly used Ethereum communication nodes";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
