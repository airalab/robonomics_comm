{ rev    ? "282e60c4f304a8def25f3d031305d1637434f9ba"             # The Git revision of nixpkgs to fetch
, sha256 ? "1h20malr7ghcaqfql6fgf72dkgkclqpxix4d0qbrpjm44nsz1f48" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
