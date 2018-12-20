{ rev    ? "2aa81442f00bd287af3d883feb0d69578bfa7df1"             # The Git revision of nixpkgs to fetch
, sha256 ? "0c19ym0fwpj8jg8dmrgyfvcca68qmlmfigpp8ghw5yfg2kpbqmh4" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
