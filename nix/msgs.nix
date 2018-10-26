{ stdenv
, mkRosPackage
, ros_comm
}:

let
  pname = "robonomics_msgs";
  version = "0.0.0";

in mkRosPackage rec {
  name = "${pname}-${version}";

  src = ../robonomics_msgs; 

  propagatedBuildInputs = [ ros_comm ];

  meta = with stdenv.lib; {
    description = "Robonomics communication messages";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
