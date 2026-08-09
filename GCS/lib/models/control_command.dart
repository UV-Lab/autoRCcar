// lib/models/control_command.dart
// corresponds to autorccar_interfaces/msg/ControlCommand

class ControlCommand {
  final double speed;
  final double steeringAngle;

  const ControlCommand({
    this.speed = 0.0,
    this.steeringAngle = 0.0,
  });

  factory ControlCommand.fromRosMsg(Map<String, dynamic> msg) {
    return ControlCommand(
      speed: (msg['speed'] as num?)?.toDouble() ?? 0.0,
      steeringAngle: (msg['steering_angle'] as num?)?.toDouble() ?? 0.0,
    );
  }
}
