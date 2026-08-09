// lib/screens/widgets/nav_status_panel.dart

import 'package:flutter/material.dart';
import '../../ros2/gcs_controller.dart';

class NavStatusPanel extends StatelessWidget {
  final GcsController ctrl;
  const NavStatusPanel({super.key, required this.ctrl});

  @override
  Widget build(BuildContext context) {
    final nav = ctrl.navState;

    return Container(
      padding: const EdgeInsets.all(12),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          _sectionTitle('NAV STATUS'),
          const SizedBox(height: 8),
          _navGroup('POS [m]', [
            ('E', nav?.position.x ?? 0),
            ('N', nav?.position.y ?? 0),
            ('U', nav?.position.z ?? 0),
          ], Colors.cyan),
          const SizedBox(height: 8),
          _navGroup('VEL [m/s]', [
            ('E', nav?.velocity.x ?? 0),
            ('N', nav?.velocity.y ?? 0),
            ('U', nav?.velocity.z ?? 0),
          ], Colors.greenAccent),
          const SizedBox(height: 8),
          _navGroup('ATT [deg]', [
            ('Roll', ctrl.rollDeg),
            ('Pitch', ctrl.pitchDeg),
            ('Yaw', ctrl.yawDeg),
          ], Colors.orangeAccent),
          const SizedBox(height: 12),
          _divider(),
          const SizedBox(height: 8),
          // show origin
          if (ctrl.hasOrigin) ...[
            _sectionTitle('ORIGIN (ECEF→LLH)'),
            const SizedBox(height: 6),
            _gpsRow('Lat', ctrl.originLatDeg ?? 0),
            _gpsRow('Lon', ctrl.originLonDeg ?? 0),
            const SizedBox(height: 8),
          ],
          // show vehicle GPS position
          // if (ctrl.vehiclePosition != null) ...[
          //   _sectionTitle('GPS POSITION'),
          //   const SizedBox(height: 6),
          //   _gpsRow('Lat', ctrl.vehiclePosition!.latitude),
          //   _gpsRow('Lon', ctrl.vehiclePosition!.longitude),
          // ],
        ],
      ),
    );
  }

  Widget _sectionTitle(String text) {
    return Text(
      text,
      style: const TextStyle(
        color: Colors.white54,
        fontSize: 10,
        fontWeight: FontWeight.bold,
        letterSpacing: 1.5,
      ),
    );
  }

  Widget _navGroup(String title, List<(String, double)> fields, Color color) {
    return Container(
      padding: const EdgeInsets.all(8),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.04),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            title,
            style: TextStyle(
              color: color,
              fontSize: 11,
              fontWeight: FontWeight.bold,
              letterSpacing: 1,
            ),
          ),
          const SizedBox(height: 6),
          Row(
            children: fields.map((f) => Expanded(
              child: _navRow(f.$1, f.$2, color),
            )).toList(),
          ),
        ],
      ),
    );
  }

Widget _navRow(String label, double value, Color color) {
  return Padding(
    padding: const EdgeInsets.symmetric(vertical: 2),
    child: Row(
      children: [
        Expanded(child: Container()),
        SizedBox(
          width: 30,
          child: Text(
            label,
            style: const TextStyle(color: Colors.white54, fontSize: 12),
          ),
        ),
        SizedBox(
          width: 70,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
            decoration: BoxDecoration(
              color: Colors.black.withOpacity(0.4),
              borderRadius: BorderRadius.circular(3),
            ),
            child: Text(
              value.toStringAsFixed(2),
              textAlign: TextAlign.right,
              style: TextStyle(
                color: color.withOpacity(0.9),
                fontSize: 12,
                fontFamily: 'monospace',
                fontWeight: FontWeight.w500,
              ),
            ),
          ),
        ),
      ],
    ),
  );
}

  Widget _gpsRow(String label, double value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 30,
            child: Text(label,
                style: const TextStyle(color: Colors.white54, fontSize: 12)),
          ),
          Expanded(
            child: Text(
              value.toStringAsFixed(7),
              textAlign: TextAlign.right,
              style: const TextStyle(
                color: Colors.lightBlueAccent,
                fontSize: 12,
                fontFamily: 'monospace',
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _divider() {
    return Container(height: 1, color: Colors.white12);
  }
}
