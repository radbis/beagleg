{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      stdenv
      pkg-config
      git
      lcov
      bear
      gtest
      valgrind
      ghostscript
      llvmPackages_19.clang-tools
      graphviz
    ];
  shellHook =
  ''
    # When compiling on a non-machine, switch off these options.
    export ARM_COMPILE_FLAGS=""
  '';
}
