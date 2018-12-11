{ robonomics_comm }:
{ pkgs, ... } :

let
  chain_spec = ''
{
    "name": "LiabilityTest",
    "engine": {
        "authorityRound": {
            "params": {
                "stepDuration": "5",
                "validators" : {
                    "list": [
                        "0x00Bd138aBD70e2F00903268F3Db08f2D25677C9e"
                    ]
							  }
            }
        }
    },
    "params": {
        "gasLimitBoundDivisor": "0x400",
        "maximumExtraDataSize": "0x20",
        "minGasLimit": "0x1388",
        "networkID" : "0x2323",
        "eip155Transition": 0,
        "validateChainIdTransition": 0,
        "eip140Transition": 0,
        "eip211Transition": 0,
        "eip214Transition": 0,
        "eip658Transition": 0
    },
    "genesis": {
        "seal": {
            "authorityRound": {
                "step": "0x0",
                "signature": "0x0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
            }
        },
        "difficulty": "0x1",
        "gasLimit": "0x8000000"
    },
    "accounts": {
        "0x0000000000000000000000000000000000000001": { "balance": "1", "builtin": { "name": "ecrecover", "pricing": { "linear": { "base": 3000, "word": 0 } } } },
        "0x0000000000000000000000000000000000000002": { "balance": "1", "builtin": { "name": "sha256", "pricing": { "linear": { "base": 60, "word": 12 } } } },
        "0x0000000000000000000000000000000000000003": { "balance": "1", "builtin": { "name": "ripemd160", "pricing": { "linear": { "base": 600, "word": 120 } } } },
        "0x0000000000000000000000000000000000000004": { "balance": "1", "builtin": { "name": "identity", "pricing": { "linear": { "base": 15, "word": 3 } } } },
        "0x004ec07d2329997267Ec62b4166639513386F32E": { "balance": "10000000000000000000000" }
    }
}
  '';
  node0_parity_config = ''
[parity]
chain = "/etc/liability-test-chain.json"
base_path = "/var/lib/parity/parity0"
keys_path = "/etc/keys"
[network]
port = 30300
[rpc]
port = 10545
apis = ["web3", "eth", "net", "personal", "parity", "parity_set", "traces", "rpc", "parity_accounts"]
[websockets]
port = 10546
[account]
unlock = ["0x004ec07d2329997267ec62b4166639513386f32e"]
password = ["/etc/keys/chain.psk"]
[mining]
engine_signer = "0x00Bd138aBD70e2F00903268F3Db08f2D25677C9e"
reseal_on_txs = "none"
'';

  user_account_private_key = "3df8095dfbae93d8c7f1143b217a483d57a7f745e2542425dfe2fa25264cb2e8";

  XRT_address = "0xDfE46C9140819979D4d02297EC37DAa224f512f2";

  ENS_address = "0x5F3DBa5e45909D1bf126aA0aF0601B1a369dbFD7";

  lighthouse_contract = "test.lighthouse.4.robonomics.eth";

in rec {
  name = "liability";
  meta = with pkgs.stdenv.lib.maintainers; {
    maintainers = [ strdn akru ];
  };

  machine =
    { config, pkgs, ... }:
    {
      virtualisation.memorySize = 4096;
      virtualisation.writableStoreUseTmpfs = false;
      networking.firewall.enable = false;

    };

  nodes = {

    liability_node =
      { config, ... }:

      {
        virtualisation.memorySize = 4096;
        virtualisation.writableStoreUseTmpfs = false;

        security.pam.loginLimits = [{
          domain = "*";
          type = "soft";
          item = "nofile";
          value = "4096";
        }];

        environment.systemPackages = with pkgs;
        [ robonomics_comm
          robonomics_contracts
          robonomics-tools
          parity
          stack
          ipfs ];

        services.ipfs = {
          enable = true;
          pubsubExperiment = true;
          defaultMode = "norouting";
        };

        services.liability = {
          enable = true;
          package = robonomics_comm;
          web3_http_provider = "http://127.0.0.1:10545";
          web3_ws_provider = "ws://127.0.0.1:10546";
          lighthouse = "test.lighthouse.4.robonomics.eth";
          ens = "0x5F3DBa5e45909D1bf126aA0aF0601B1a369dbFD7";
          keyfile = "/etc/keys/LiabilityTest/user.keyfile";
          keyfile_password_file = "/etc/keys/user.psk";
          enable_aira_graph = false;
        };

        services.parity = {
          enable = true;
          chain = "liability-test-chain";
          configText = node0_parity_config;
        };


        environment.etc."keys/LiabilityTest/authority.keyfile".text = ''
{"id":"40b4c6db-bc6b-b47e-c828-c088ba93fa90","version":3,"crypto":{"cipher":"aes-128-ctr","cipherparams":{"iv":"868d22fb7b90185c47260a82ec36526e"},"ciphertext":"7f5cdd6ee2c5dc7804433d8dfb321af597fed90c171e8c0200134c20ae9a581a","kdf":"pbkdf2","kdfparams":{"c":10240,"dklen":32,"prf":"hmac-sha256","salt":"8fdb34a12a51cd7f70d23bbabaec24c712421ca307611419df6d46a212b66889"},"mac":"6e0f65a427a8483c0993d4727f24872445d3294750112fa95fa7670acc4ab77f"},"address":"00bd138abd70e2f00903268f3db08f2d25677c9e","name":"","meta":"{}"}'';

        environment.etc."keys/LiabilityTest/user.keyfile".text = ''
{"id":"a49b3696-add3-9b4c-982c-14e54f31b5fb","version":3,"crypto":{"cipher":"aes-128-ctr","cipherparams":{"iv":"5f2294ec34165c900ffa72989af2cb16"},"ciphertext":"d6f1a16f2fbfe1c96b769c5ec77d1c238ef4ea8e4c0eda5f8493315f329440a1","kdf":"pbkdf2","kdfparams":{"c":10240,"dklen":32,"prf":"hmac-sha256","salt":"294f52b7b37a8b37fb1f3ca6453ca7a5628ce556b443b0cbdda96bbbfefecd0f"},"mac":"7f071f865a0ee9dead6b08516bd7ea58ad5852ad72e3edb933dce669808e0d8f"},"address":"004ec07d2329997267ec62b4166639513386f32e","name":"","meta":"{}"}'';


        environment.etc."keys/chain.psk".text = ''
node0
user'';
        environment.etc."keys/user.psk".text = ''user'';

        environment.etc."resolv.conf".text = "nameserver 8.8.8.8";
        environment.etc."liability-test-chain.json".text = chain_spec;

        environment.etc."xrtc-launch".text = "xrtd --private 3df8095dfbae93d8c7f1143b217a483d57a7f745e2542425dfe2fa25264cb2e8 --web3 http://localhost:10545 --ens 0x5F3DBa5e45909D1bf126aA0aF0601B1a369dbFD7 --chain 8995 --lighthouse test.lighthouse.4.robonomics.eth > /tmp/xrtd.log 2>&1 &";
        environment.etc."rostest-launch".text = "source ${robonomics_comm}/setup.bash && rostest -r robonomics_liability liability.test ens_contract:=0x5F3DBa5e45909D1bf126aA0aF0601B1a369dbFD7 lighthouse_contract:='test.lighthouse.4.robonomics.eth'  web3_http_provider:='http://127.0.0.1:10545'  test_token:=0xDfE46C9140819979D4d02297EC37DAa224f512f2";

        networking.firewall.enable = false;
      };
  };

  testScript = ''
    startAll;

    #parity
    $liability_node->waitForUnit("parity");
    $liability_node->waitForOpenPort("10545");
    $liability_node->waitForOpenPort("10546");

    #liability
    $liability_node->waitForUnit("liability");

    #all units started, run robonomics_contracts creation
    $liability_node->mustSucceed("
      truffle migrate --network testing
    ");

    #run xrtd. TODO: systemd service
    $liability_node->mustSucceed("
      xrtd --private 3df8095dfbae93d8c7f1143b217a483d57a7f745e2542425dfe2fa25264cb2e8 --web3 http://localhost:10545 --ens ${ENS_address} --chain 8995 --lighthouse ${lighthouse_contract} > /tmp/xrtd.log 2>&1 &
    ");

    $liability_node->mustSucceed("
      source ${robonomics_comm}/setup.bash && rostest -r robonomics_liability liability.test ens_contract:=${ENS_address} lighthouse_contract:=${lighthouse_contract}  web3_http_provider:='http://127.0.0.1:10545'  test_token:=0xDfE46C9140819979D4d02297EC37DAa224f512f2 >&2
    ");
  '';
}
