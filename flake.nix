{
  inputs = {

    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";

    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
    nixpkgs-system.url = "github:NixOS/nixpkgs/25.05";

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

    # camera-calibration-flake = {
    #   # url = "git+file:/home/scott/GIT/image_pipeline/camera_calibration";
    #   url = "github:SCOTT-HAMILTON/ros_camera_calibration/master";
    #   inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # Ensure consistent nixpkgs
    # };

    ros-foxglove-bridge-flake = {
      url = "github:SCOTT-HAMILTON/ros-foxglove-bridge";
      inputs.nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
  };
  outputs = { self, nix-ros-overlay, nixpkgs, ardrone-autonomy-flake, ros-foxglove-bridge-flake, nixpkgs-system }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        # The overlay logic from your first snippet
        applyDistroOverlay = rosOverlay: rosPackages:
          rosPackages
          // builtins.mapAttrs (
            rosDistro: rosPkgs: if rosPkgs ? overrideScope then rosPkgs.overrideScope rosOverlay else rosPkgs
            ) rosPackages;

        rosDistroOverlays = final: prev: {
          rosPackages = applyDistroOverlay (rosFinal: rosPrev: {
            ardrone-autonomy = ardrone-autonomy-flake.packages.${system}.default;
            ros-foxglove-bridge = ros-foxglove-bridge-flake.packages.${system}.default;
            nix-ros-ardrone = rosFinal.callPackage ./package.nix {};
            orb-slam2-ros = rosFinal.callPackage ./orb-slam2-ros/package.nix { };
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
                rqt-image-view
                # mycamera-calibration
                # rqt-reconfigure

                ## image_transport collision
                nix-ros-ardrone
                # orb-slam3-ros
                ardrone-autonomy

                # stella-vslam-ros
                orb-slam2-ros
                robot-localization
                image-view
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
            pkgs.gdb
            pkgs.valgrind
          ];
        };

        packages.default = pkgs.rosPackages.noetic.nix-ros-ardrone;
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
