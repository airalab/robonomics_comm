{ rev    ? "3bb32e42e1291870f2fec6b96cedded157070576"             # The Git revision of nixpkgs to fetch
, sha256 ? "1s5shhcvvfy88xirf80xh3crlkgxd71vs53vyl0b30ip9c2y5kpy" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

rec {
  nixpkgs = import (builtins.fetchTarball {
    url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
    inherit sha256;
  }) { inherit system; };
}
