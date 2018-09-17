{ lib ? import ./nix/lib.nix { }
}:

let
  callPackage = lib.nixpkgs.callPackage;

in rec {
  robonomics_comm_ethereum_common = callPackage ./nix/ethereum_common.nix { };
  robonomics_comm_lighthouse = callPackage ./nix/lighthouse.nix { };
  robonomics_comm_liability = callPackage ./nix/liability.nix { };
  robonomics_comm_control = callPackage ./nix/control.nix { };
  robonomics_comm = callPackage ./nix/default.nix { };
}
