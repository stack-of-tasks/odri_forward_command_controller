{
  description = "A ros2 controller to send (position, velocity, effort, Kp, Kd) commands to an odri device";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs.odri-forward-command-controller = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./include
              ./odri_forward_command_controller_plugin.xml
              ./package.xml
              ./src
              ./test
            ];
          };
        };
      }
    );
}
