{ lib
, stdenv
, cmake
, stella-vslam
, sioclient
, protobuf
, opencv
, eigen
, g2o
, libGL
, libGLU
, yaml-cpp
, sqlite
, fetchFromGitHub
# , nix-gitignore
}:

stdenv.mkDerivation rec {
  pname = "socket-publisher";
  version = "0.0.2";

  src = fetchFromGitHub {
    owner = "stella-cv";
    repo = "socket_publisher";
    rev = version;
    sha256 = "sha256-4MxPi7C8kOQ7KDDRLbzi4O57RwQODc3UvQBB1f2HzLg=";
    fetchSubmodules = true;
  };
  patches = [ ./good.patch ];
  nativeBuildInputs = [ cmake ];
  buildInputs =  [ stella-vslam sioclient protobuf opencv eigen g2o libGL libGLU yaml-cpp sqlite ];

  meta = {
    description = "stella-vslam";
    license = with lib.licenses; [ mit ];
  };
}
