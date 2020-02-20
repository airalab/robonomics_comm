{ stdenv
, ros_comm
, mkRosPackage
, python3Packages
, python3
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "robonomics_comm";
  version = "master";

  src = ./.;

  propagatedBuildInputs = with python3Packages;
  [ ros_comm web3 base58 voluptuous python-persistent-queue ipfsapi ipfshttpclient setuptools];

  postInstall = ''
    patch $out/lib/${python3.libPrefix}/site-packages/ethereum_common/msg/_UInt256.py $src/ethereum_common/msg/_UInt256.py.patch
  '';

  meta = with stdenv.lib; {
    description = "Robonomics communication stack";
    homepage = http://github.com/airalab/robonomics_comm;
    license = licenses.bsd3;
    maintainers = [ maintainers.akru ];
  };
}
