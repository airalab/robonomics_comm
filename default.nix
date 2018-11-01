{ lib ? import ./nix/lib.nix { }
}:

let
  callPackage = lib.nixpkgs.callPackage;

in rec {
  robonomics_comm_msgs = callPackage ./nix/msgs.nix { };
  robonomics_comm_ethereum_common = callPackage ./nix/ethereum_common.nix {  };
  robonomics_comm_ipfs_common = callPackage ./nix/ipfs_common.nix {
    inherit robonomics_comm_msgs;
   };
  robonomics_comm_lighthouse = callPackage ./nix/lighthouse.nix {
    inherit robonomics_comm_ipfs_common;
  };
  robonomics_comm_liability = callPackage ./nix/liability.nix { };
  robonomics_comm_control = callPackage ./nix/control.nix { };
  robonomics_comm = callPackage ./nix/default.nix {
    inherit robonomics_comm_ipfs_common;
  };
}
