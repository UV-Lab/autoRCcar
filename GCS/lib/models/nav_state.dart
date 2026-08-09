// lib/models/nav_state.dart
// corresponds to autorccar_interfaces/NavState

class Vec3 {
  final double x, y, z;
  const Vec3({this.x = 0, this.y = 0, this.z = 0});

  factory Vec3.fromMap(Map<String, dynamic> m) => Vec3(
        x: (m['x'] as num?)?.toDouble() ?? 0,
        y: (m['y'] as num?)?.toDouble() ?? 0,
        z: (m['z'] as num?)?.toDouble() ?? 0,
      );
}

class Quaternion {
  final double x, y, z, w;
  const Quaternion({this.x = 0, this.y = 0, this.z = 0, this.w = 1});

  factory Quaternion.fromMap(Map<String, dynamic> m) => Quaternion(
        x: (m['x'] as num?)?.toDouble() ?? 0,
        y: (m['y'] as num?)?.toDouble() ?? 0,
        z: (m['z'] as num?)?.toDouble() ?? 0,
        w: (m['w'] as num?)?.toDouble() ?? 1,
      );
}

class NavState {
  final Vec3 origin;         // ECEF [m]
  final Vec3 position;       // ENU [m]
  final Vec3 velocity;       // ENU [m/s]
  final Quaternion quaternion;
  final Vec3 acceleration;
  final Vec3 angularVelocity;

  const NavState({
    required this.origin,
    required this.position,
    required this.velocity,
    required this.quaternion,
    required this.acceleration,
    required this.angularVelocity,
  });

  factory NavState.fromRosMsg(Map<String, dynamic> msg) {
    return NavState(
      origin: Vec3.fromMap(msg['origin'] ?? {}),
      position: Vec3.fromMap(msg['position'] ?? {}),
      velocity: Vec3.fromMap(msg['velocity'] ?? {}),
      quaternion: Quaternion.fromMap(msg['quaternion'] ?? {}),
      acceleration: Vec3.fromMap(msg['acceleration'] ?? {}),
      angularVelocity: Vec3.fromMap(msg['angular_velocity'] ?? {}),
    );
  }
}
