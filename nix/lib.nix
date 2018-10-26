{ rev    ? "bd564e4c80f2ee9658bfea888704e792fbc58669"             # The Git revision of nixpkgs to fetch
, sha256 ? "1a1lif48k8idcdh147icfzhg4ckpmd49hq2jhqiy13nw1sb89sdb" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

rec {
  nixpkgs = import (builtins.fetchTarball {
    url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
    inherit sha256;
  }) { inherit system; };
}
