{ rev    ? "558e1fe078a04b93a2a833d9ddbc142c76b78716"             # The Git revision of nixpkgs to fetch
, sha256 ? "05irjsbinh3s5id1zqqs46sl77jgr008s1ff72pm84niyihxh0nk" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
