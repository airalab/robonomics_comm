{ rev    ? "2004b8435b49ebd95cde5f23cb3b7fb75a04ab16"             # The Git revision of nixpkgs to fetch
, sha256 ? "1da82r97g0hdrsrvv0zgmhkbgb0zkga0pvpg3c377m89chqjx1cb" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
