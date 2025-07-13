{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    ardrone-autonomy-flake = {
      # url = "github:SCOTT-HAMILTON/ardrone_autonomy";
      url = "git+file:./ardrone_autonomy";
      inputs.nixpkgs.follows = "nixpkgs";  # Ensure consistent nixpkgs
    };
  };
  outputs = { self, nix-ros-overlay, ardrone-autonomy-flake, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        # The overlay logic from your first snippet
        applyDistroOverlay = rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
            ) rosPackages;

        rosOverlay = final: prev: {
          ardrone-autonomy = ardrone-autonomy-flake.packages.${system}.default;
        };

        rosDistroOverlays = final: prev: {
          # Apply the overlay to the ROS packages from nix-ros-overlay
          rosPackages = applyDistroOverlay (rosOverlay) prev.rosPackages;
        };


        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default rosDistroOverlays ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            pkgs.colcon
            (pkgs.python3.withPackages (ps: with ps; [
              jupyter
              ipython
              numpy
              scipy
              matplotlib
              numba
              pandas
              tqdm
              pyqt5
              pyqtgraph
            ]))
            # ... other non-ROS packages
            (with pkgs.rosPackages.noetic; buildEnv {
              paths = [
                ros-core
                ardrone-autonomy
                rospy
                rviz
                plotjuggler
                plotjuggler-msgs
                plotjuggler-ros
                teleop-twist-joy
                # ardrone-tutorial
                # ... other ROS packages
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
