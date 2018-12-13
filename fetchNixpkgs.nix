{ rev    ? "72ccb7fc71a87b328628b400c72867dfccc58796"             # The Git revision of nixpkgs to fetch
, sha256 ? "06jn1zwkk5nvpi6fljfk827498r5866jdclsb39sgdbjvk6jcrfs" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
