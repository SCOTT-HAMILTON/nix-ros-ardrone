{ ardrone-autonomy-src }: (final: prev:
{
  ardrone-autonomy = final.callPackage (ardrone-autonomy-src + "/package.nix") {};
})
