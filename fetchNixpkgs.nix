{ rev    ? "e8ea32f3a1ccf19bce6c7e20678b0a308d4731b6"             # The Git revision of nixpkgs to fetch
, sha256 ? "12z0ndyhdzymc4adlsp00h5543n32gnwagaxw6fc2kwrmhqgafln" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
