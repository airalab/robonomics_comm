{ rev    ? "06103c4647ee4d40936d123a796e45c6a18b8863"             # The Git revision of nixpkgs to fetch
, sha256 ? "0w0qfsmcqdilj9c6jxx89q7jlc7bl25zlapfk8djn0hl7vgab2ik" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
