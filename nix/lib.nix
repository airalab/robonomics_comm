{ rev    ? "60f643ca403345e7a75aad7eaba9a44f04cc1823"             # The Git revision of nixpkgs to fetch
, sha256 ? "1sjpaqrjggqiwn797mn5ycd0znq8byqyvgnbzifw1mf692xfmm40" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

rec {
  nixpkgs = import (builtins.fetchTarball {
    url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
    inherit sha256;
  }) { inherit system; };
}
