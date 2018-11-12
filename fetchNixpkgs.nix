{ rev    ? "59ca70ddf9704518814e782a95e1f06124f955df"             # The Git revision of nixpkgs to fetch
, sha256 ? "1xgr3qivganvxfyqr4l2y1359vvdylcf296jrj1v0zck931dfpg1" # The SHA256 of the downloaded data
, system ? builtins.currentSystem                                 # This is overridable if necessary
}:

import (builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}) { inherit system; }
