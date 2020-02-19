{ rev    ? "acb4d2362e801259500bc150950b58b0642341e7"             # The Git revision of nixpkgs to fetch
, sha256 ? "0pw33qv1zxy0zaandwv1fvwz9wp2faa12i6i2ilayxxzdi4683i8" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
