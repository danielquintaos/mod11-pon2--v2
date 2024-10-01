{
  description = "ESP32-CAM Real-Time Face Detection Project";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {
      devShells.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          python3
          python3Packages.numpy
          python3Packages.opencv4
          python3Packages.pyserial
          platformio
        ];
      };
    };
}
