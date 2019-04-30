{ rev    ? "e760716c65e7223c5d3dd4ac8a629504b387e0d1"             # The Git revision of nixpkgs to fetch
, sha256 ? "0wir84c90n78kc51hpg3ps422p9wh479c62sq9201l0i80lih3l7" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
