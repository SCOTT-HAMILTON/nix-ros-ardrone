{
  inputs = {

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixpkgs-system.url = "github:NixOS/nixpkgs/25.05";
    flake-utils.url = "github:numtide/flake-utils";

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

    orb-slam3-ros-flake = {
      # url = "git+file:/home/scott/GIT/thien94/orb_slam3_ros";
      url = "github:SCOTT-HAMILTON/orb_slam3_ros";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };

    # nix-gl-flake = {
    #   url = "github:nix-community/nixGL";
    #   inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    # };

    # simple-pid-flake = {
      # url = "git+file:/home/scott/GIT/nix-ros-ardrone/simple-pid";
      # inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    # };
  };
  outputs = { self, flake-utils, nix-ros-overlay, nixpkgs, ardrone-autonomy-flake, camera-calibration-flake, ros-foxglove-bridge-flake, orb-slam3-ros-flake, nixpkgs-system }:
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
            # image-transport = prev.lib.traceValFn (x: "patched-image-transport=${x}") (rosFinal.callPackage ./image-transport/package.nix {});
            # cv-bridge = rosPrev.cv-bridge.overrideAttrs (old: {
            #   buildInputs = (old.buildInputs or []) ++ [ final.opencv4 ];
            # });
            ardrone-autonomy = ardrone-autonomy-flake.packages.${system}.default;
              # propagatedBuildInputs = prev.lib.traceValFn (x: builtins.concatStringsSep ", " (map (y: y.pname or "NO_PNAME") x) )
              #   (map (x: if x.pname == "ros-noetic-image-transport" then image-transport else x) old.propagatedBuildInputs);
            # });
            mycamera-calibration = camera-calibration-flake.packages.${system}.default.overrideAttrs (
              old: {
                # propagatedBuildInputs =
                #   builtins.filter (pkg: (pkg.pname or "") != "ros-noetic-cv-bridge") (old.propagatedBuildInputs or [])
                #   ++ [ cv-bridge ];
              }
            );
            # rqt-image-view = rosPrev.rqt-image-view.overrideAttrs (old: {
            #   buildInputs = pkgs.lib.traceValFn (x: builtins.concatStringsSep ", " (map (pkg: pkg.pname or "NO_PNAME") x)) ((builtins.filter (pkg: (pkg.pname or "") != "ros-noetic-cv-bridge") (old.buildInputs or [])) ++ [ cv-bridge ]);
            # });

            ros-foxglove-bridge = ros-foxglove-bridge-flake.packages.${system}.default;

            nix-ros-ardrone = rosFinal.callPackage ./package.nix {};

            orb-slam3-ros = orb-slam3-ros-flake.packages.${system}.default;
          }) prev.rosPackages;
        };
        
        opencvOverlay = self: super: {
          opencv4 = super.opencv4.override { enableGtk3 = true; };
        };

        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rosDistroOverlays
            # opencvOverlay
          ];
        };

        pkgsSystem = import nixpkgs-system {};

        mypython = (pkgs.python3.override {
          # Careful, we're using a different final and prev here!
          packageOverrides = final: prev: {
            simple-pid = prev.buildPythonPackage rec {
              pname = "simple-pid";
              version = "2.0.0";
              src = prev.fetchPypi {
                inherit pname version;
                sha256 = "sha256-t2ScuJEkNe9PL0+ZexDyuFdXvJ7nnZTE+rM/nTuE3Vs=";
              };
              propagatedBuildInputs = [
                prev.setuptools
              ];
              format = "pyproject";
            };
          };
        }).withPackages (ps: with ps; [
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
          simple-pid
        ]);
        rosEnv = (with pkgs.rosPackages.noetic; buildEnv {
              paths = [
                ros-core
                rviz # REQUIRES UNFREE freeimage package
                rospy
                plotjuggler
                plotjuggler-msgs
                plotjuggler-ros
                teleop-twist-joy
                ros-foxglove-bridge
                dynamic-reconfigure
                control-toolbox
                hector-trajectory-server
                
                ## cv_bridge collision
                # rqt-image-view
                # mycamera-calibration
                # rqt-reconfigure

                ## image_transport collision
                nix-ros-ardrone
                orb-slam3-ros
                ardrone-autonomy

                ## ROS Packages for running orb-slam3
                roscpp
                tf
                sensor-msgs
                (lib.traceValFn (x: "rosEnv direct image-transport = ${x}") image-transport)
              ];
            });
      in {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          shellHook = ''
            export ROS_PACKAGE_PATH="${rosEnv}/share"; # Required for ROS's pluginlib to find plugins
          '';
          packages = [
            pkgs.colcon
            # simple-pid-flake.packages.${system}.default
            # ... other non-ROS packages
            rosEnv 
            (pkgs.runCommand "mypython-wrapper" {
              nativeBuildInputs = [ pkgs.makeWrapper ];
            } ''
              mkdir -p $out/bin
              makeWrapper ${mypython}/bin/python $out/bin/mypython
            '')
            pkgsSystem.gvfs
          ];
        };

        packages.default = pkgs.rosPackages.noetic.nix-ros-ardrone;
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
