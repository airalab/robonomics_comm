{ rev    ? "812f01d8ef840b2d5a11e6e84872f66f0e86ec54"             # The Git revision of nixpkgs to fetch
, sha256 ? "1n43js09fx9jav27652mkmkkxh01675wnv8bj1mnj4p62cwricsf" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
