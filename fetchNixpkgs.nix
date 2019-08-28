{ rev    ? "593878eee93a4d2ae0d50d8d70822cfdb75606ea"             # The Git revision of nixpkgs to fetch
, sha256 ? "00vz0lf8zid95jpy82pf2rq3jiv9zyxll6ck64b8n88ky64pwmz1" # The SHA256 of the downloaded data
}:

builtins.fetchTarball {
  url = "https://github.com/airalab/airapkgs/archive/${rev}.tar.gz";
  inherit sha256;
}
