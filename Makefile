all: package

package:
	nix build --no-link -f release.nix $@

test:
	nix-build --no-link release.nix -A $@
