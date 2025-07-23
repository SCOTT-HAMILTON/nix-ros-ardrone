{ lib
, stdenv
, cmake
, eigen
, yaml-cpp
, opencv
, spdlog
, sqlite
, g2o
, libGL
, libGLU
, fetchFromGitHub
# , nix-gitignore
}:

stdenv.mkDerivation rec {
  pname = "stella-vslam";
  version = "0.6.0";

  src = fetchFromGitHub {
    owner = "stella-cv";
    repo = "stella_vslam";
    rev = version;
    sha256 = "sha256-j7YLZ82Mnz+derNI/rZCYqwCIbTIipF3r/l4ZcOXSbY=";
    fetchSubmodules = true;
  };
  nativeBuildInputs = [ cmake ];
  buildInputs =  [ eigen yaml-cpp opencv spdlog sqlite g2o libGL libGLU ];

  meta = {
    description = "stella-vslam";
    license = with lib.licenses; [ mit ];
  };
}
