{ rev    ? "6164a1bcc2777da7ea4e986b6683d4f426260989"             # The Git revision of nixpkgs to fetch
, sha256 ? "07fg192npdnwnfs2mv8wg3339al5yz4l0h1shl54967ylck613s6" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
