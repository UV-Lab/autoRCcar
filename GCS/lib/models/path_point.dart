// lib/models/path_point.dart

import 'package:latlong2/latlong.dart';

/// ENU coordinate waypoint (used internally for computation)
class WaypointENU {
  double x; // East [m]
  double y; // North [m]

  WaypointENU({required this.x, required this.y});
}

/// waypoint for map display (lat/lng)
class WaypointLLH {
  final LatLng latLng;
  final double x; // ENU East
  final double y; // ENU North

  const WaypointLLH({
    required this.latLng,
    required this.x,
    required this.y,
  });
}
