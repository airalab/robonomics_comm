{ nixpkgs ? import ./fetchNixpkgs.nix { }
, system ? builtins.currentSystem
}:

let
  liabilityTest = import "${nixpkgs}/nixos/tests/liability.nix";
  pkgs = import nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
  test = liabilityTest { inherit package; };
}
