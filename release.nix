{ nixpkgs ? import ./fetchNixpkgs.nix { } }:

rec {
  robonomics_comm = nixpkgs.callPackage ./default.nix { };
}
