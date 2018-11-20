{ rev    ? "99ad3291fd7f1ebbc4818ddce3ed99693bdee1d7"             # The Git revision of nixpkgs to fetch
, sha256 ? "1lmlqiwmxw2ri2g0kvqdc19n0qkscwzyd68rgzb3j8n25rziiars" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
