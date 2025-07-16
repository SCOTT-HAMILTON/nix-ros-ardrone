{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!

    ardrone-autonomy-flake = {
      # url = "github:SCOTT-HAMILTON/ardrone_autonomy";
      url = "github:SCOTT-HAMILTON/ardrone_autonomy/indigo-devel";
      inputs = {
        nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # Ensure consistent nixpkgs
        ardronelib = {
          # url = "git+file:/home/scott/GIT/nix-ros-ardrone/ardrone_autonomy/ardronelib";
          url = "github:SCOTT-HAMILTON/ardronelib/master";
          inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
        };
      };
    };

    camera-calibration-flake = {
      # url = "git+file:/home/scott/GIT/image_pipeline/camera_calibration";
      url = "github:SCOTT-HAMILTON/ros_camera_calibration/master";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # Ensure consistent nixpkgs
    };

    ros-foxglove-bridge-flake = {
      url = "github:SCOTT-HAMILTON/ros-foxglove-bridge";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
  };
  outputs = { self, nix-ros-overlay, nixpkgs, ardrone-autonomy-flake, camera-calibration-flake, ros-foxglove-bridge-flake }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        # The overlay logic from your first snippet
        applyDistroOverlay = rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
            ) rosPackages;

        rosDistroOverlays = final: prev: {
          rosPackages = applyDistroOverlay (rosFinal: rosPrev: rec {
            cv-bridge = rosPrev.cv-bridge.overrideAttrs (old: {
              buildInputs = (old.buildInputs or []) ++ [ final.opencv4 ];
            });
            ardrone-autonomy = ardrone-autonomy-flake.packages.${system}.default;
            mycamera-calibration = camera-calibration-flake.packages.${system}.default.overrideAttrs (
              old: {
                propagatedBuildInputs =
                  builtins.filter (pkg: (pkg.pname or "") != "ros-noetic-cv-bridge") (old.propagatedBuildInputs or [])
                  ++ [ cv-bridge ];
              }
            );
            rqt-image-view = rosPrev.rqt-image-view.overrideAttrs (old: {
              buildInputs = pkgs.lib.traceValFn (x: builtins.concatStringsSep ", " (map (pkg: pkg.pname or "NO_PNAME") x)) ((builtins.filter (pkg: (pkg.pname or "") != "ros-noetic-cv-bridge") (old.buildInputs or [])) ++ [ cv-bridge ]);
            });

            ros-foxglove-bridge = ros-foxglove-bridge-flake.packages.${system}.default;

            nix-ros-ardrone = rosFinal.callPackage ./package.nix {};
          }) prev.rosPackages;
        };
        
        opencvOverlay = self: super: {
          opencv4 = super.opencv4.override { enableGtk3 = true; };
        };

        pkgs = import nixpkgs {
          inherit system;
          overlays = [  nix-ros-overlay.overlays.default rosDistroOverlays opencvOverlay ];
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
              distutils
              opencv4
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
                rqt-image-view
                mycamera-calibration
                ros-foxglove-bridge
                nix-ros-ardrone
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
