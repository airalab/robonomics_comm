{ nixpkgs ? import ./fetchNixpkgs.nix { }
, system ? builtins.currentSystem
}:

let
  makeTest = import "${nixpkgs}/nixos/tests/make-test.nix";
  pkgs = import nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
  test = makeTest (import ./tests.nix { robonomics_comm = package; });
}
