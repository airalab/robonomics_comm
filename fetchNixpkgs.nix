{ rev    ? "f8c451acaa0495a00f507ef72da479f8789eda2c"             # The Git revision of nixpkgs to fetch
, sha256 ? "0h8imd7zg6yx63yn06wb5ka75mwd3x90g41w2n1zvahx9gsrrdk7" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
