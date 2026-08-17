// lib/models/occupancy_grid.dart
// corresponds to nav_msgs/msg/OccupancyGrid
//
// note: rosbridge often sends int8[]/uint8[] array fields as
// base64-encoded strings rather than JSON arrays. both cases are handled.

import 'dart:convert';

class OccupancyGridMsg {
  final int width;
  final int height;
  final double resolution; // [m/cell]
  final double originX;    // origin in map frame [m]
  final double originY;
  final List<int> data;    // -1: unknown, 0: free, 100: occupied

  const OccupancyGridMsg({
    required this.width,
    required this.height,
    required this.resolution,
    required this.originX,
    required this.originY,
    required this.data,
  });

  factory OccupancyGridMsg.fromRosMsg(Map<String, dynamic> msg) {
    final info = msg['info'] as Map<String, dynamic>? ?? {};
    final origin = info['origin'] as Map<String, dynamic>? ?? {};
    final position = origin['position'] as Map<String, dynamic>? ?? {};

    final rawData = msg['data'];
    List<int> data;

    if (rawData is String) {
      // case where rosbridge encoded int8[]/uint8[] as a base64 string
      final bytes = base64.decode(rawData);
      // convert uint8 (0~255) to int8 (-128~127)
      data = bytes.map((b) => b > 127 ? b - 256 : b).toList();
    } else if (rawData is List) {
      data = rawData.map((e) => (e as num).toInt()).toList();
    } else {
      data = const [];
    }

    return OccupancyGridMsg(
      width: (info['width'] as num?)?.toInt() ?? 0,
      height: (info['height'] as num?)?.toInt() ?? 0,
      resolution: (info['resolution'] as num?)?.toDouble() ?? 0.05,
      originX: (position['x'] as num?)?.toDouble() ?? 0.0,
      originY: (position['y'] as num?)?.toDouble() ?? 0.0,
      data: data,
    );
  }
}
