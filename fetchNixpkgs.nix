{ rev    ? "907c71e6477831833cf0e81ef10202c0e9a253dd"             # The Git revision of nixpkgs to fetch
, sha256 ? "0nlldjzjrs1zxz6s280q4lx9nb3n4jywhwmq5ngqfmkazg3nkj6d" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
