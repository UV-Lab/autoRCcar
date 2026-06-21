// lib/screens/widgets/enu_plot_view.dart
//
// Flutter port of the PyQt pyqtgraph position graph.
// displays current vehicle position/heading, trajectory, waypoints, and spline on an ENU (East/North) grid.
// follows the vehicle at center; supports zoom via mouse wheel.
// left-click: add waypoint / right-click: remove nearest waypoint

import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/gestures.dart';
import '../../ros2/gcs_controller.dart';

class EnuPlotView extends StatefulWidget {
  final GcsController ctrl;
  const EnuPlotView({super.key, required this.ctrl});

  @override
  State<EnuPlotView> createState() => _EnuPlotViewState();
}

class _EnuPlotViewState extends State<EnuPlotView> {
  double _scale = 20.0; // pixels per meter

  void _zoom(double factor) {
    setState(() {
      _scale = (_scale * factor).clamp(2.0, 300.0);
    });
  }

  /// convert screen coordinates to ENU (east, north)
  Offset _screenToEnu(
      Offset localPos, Size size, double centerEast, double centerNorth) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    final dx = localPos.dx - cx;
    final dy = cy - localPos.dy; // invert screen y so north is up
    final east = centerEast + dx / _scale;
    final north = centerNorth + dy / _scale;
    return Offset(east, north);
  }

  @override
  Widget build(BuildContext context) {
    final ctrl = widget.ctrl;
    final pos = ctrl.navState?.position;
    final east = pos?.x ?? 0.0;
    final north = pos?.y ?? 0.0;
    final yaw = ctrl.yawDeg;

    return Container(
      color: const Color(0xFF1A1A2E),
      child: Stack(
        children: [
          Positioned.fill(
            child: LayoutBuilder(
              builder: (context, constraints) {
                final size = Size(constraints.maxWidth, constraints.maxHeight);
                return Listener(
                  onPointerSignal: (event) {
                    if (event is PointerScrollEvent) {
                      final factor = event.scrollDelta.dy > 0 ? 0.9 : 1.1;
                      _zoom(factor);
                    }
                  },
                  onPointerDown: (event) {
                    final enu =
                        _screenToEnu(event.localPosition, size, east, north);

                    if (event.buttons & kPrimaryButton != 0) {
                      ctrl.addWaypointENU(enu.dx, enu.dy);
                    } else if (event.buttons & kSecondaryButton != 0) {
                      final thresholdM = 14 / _scale;
                      ctrl.removeWaypointNearENU(enu.dx, enu.dy, thresholdM);
                    }
                  },
                  child: CustomPaint(
                    size: size,
                    painter: _EnuGridPainter(
                      scale: _scale,
                      centerEast: east,
                      centerNorth: north,
                      yawDeg: yaw,
                      trajectory: ctrl.enuTrajectory,
                      waypoints:
                          ctrl.waypoints.map((w) => (w.x, w.y)).toList(),
                      splineENU: ctrl.splineENU,
                    ),
                  ),
                );
              },
            ),
          ),

          // zoom controls
          Positioned(
            top: 16,
            right: 16,
            child: Column(
              children: [
                _zoomButton(Icons.add, () => _zoom(1.25)),
                const SizedBox(height: 4),
                _zoomButton(Icons.remove, () => _zoom(1 / 1.25)),
              ],
            ),
          ),

          // current position / waypoint info
          Positioned(
            bottom: 16,
            left: 16,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.black.withOpacity(0.6),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    'E: ${east.toStringAsFixed(2)} m   N: ${north.toStringAsFixed(2)} m',
                    style: const TextStyle(
                      color: Colors.white70,
                      fontSize: 11,
                      fontFamily: 'monospace',
                    ),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    '좌클릭: Waypoint 추가  |  우클릭: 삭제  |  ${ctrl.waypoints.length}개',
                    style: const TextStyle(
                      color: Colors.white38,
                      fontSize: 10,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _zoomButton(IconData icon, VoidCallback onTap) {
    return InkWell(
      onTap: onTap,
      child: Container(
        width: 28,
        height: 28,
        decoration: BoxDecoration(
          color: Colors.black.withOpacity(0.5),
          borderRadius: BorderRadius.circular(4),
        ),
        child: Icon(icon, color: Colors.white70, size: 16),
      ),
    );
  }
}

class _EnuGridPainter extends CustomPainter {
  final double scale; // px per meter
  final double centerEast;
  final double centerNorth;
  final double yawDeg;
  final List<(double, double)> trajectory; // (east, north)
  final List<(double, double)> waypoints;  // (east, north)
  final List<(double, double)> splineENU;  // (east, north)

  _EnuGridPainter({
    required this.scale,
    required this.centerEast,
    required this.centerNorth,
    required this.yawDeg,
    required this.trajectory,
    required this.waypoints,
    required this.splineENU,
  });

  /// convert ENU (east, north) to screen coordinates (vehicle fixed at center, north up)
  Offset _toScreen(double east, double north, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    final dx = (east - centerEast) * scale;
    final dy = (north - centerNorth) * scale;
    return Offset(cx + dx, cy - dy);
  }

  @override
  void paint(Canvas canvas, Size size) {
    canvas.drawRect(
      Rect.fromLTWH(0, 0, size.width, size.height),
      Paint()..color = const Color(0xFF1A1A2E),
    );

    // ── auto-adjust grid spacing to stay between 40~160 px ──────────
    double gridStepM = 1.0;
    double pxPerGrid = scale * gridStepM;
    while (pxPerGrid < 40) {
      gridStepM *= 2;
      pxPerGrid = scale * gridStepM;
    }
    while (pxPerGrid > 160) {
      gridStepM /= 2;
      pxPerGrid = scale * gridStepM;
    }

    final gridPaint = Paint()
      ..color = Colors.white.withOpacity(0.08)
      ..strokeWidth = 1;
    final axisPaint = Paint()
      ..color = Colors.white.withOpacity(0.3)
      ..strokeWidth = 1.2;

    final halfWm = (size.width / 2) / scale;
    final halfHm = (size.height / 2) / scale;

    final eastStart = ((centerEast - halfWm) / gridStepM).floor() * gridStepM;
    final eastEnd = centerEast + halfWm;
    for (double e = eastStart; e <= eastEnd; e += gridStepM) {
      final p1 = _toScreen(e, centerNorth - halfHm, size);
      final p2 = _toScreen(e, centerNorth + halfHm, size);
      canvas.drawLine(p1, p2, e.abs() < 1e-6 ? axisPaint : gridPaint);
      _drawText(canvas, _fmt(e, gridStepM),
          Offset(p1.dx + 2, size.height - 16), Colors.white38, 9);
    }

    final northStart =
        ((centerNorth - halfHm) / gridStepM).floor() * gridStepM;
    final northEnd = centerNorth + halfHm;
    for (double n = northStart; n <= northEnd; n += gridStepM) {
      final p1 = _toScreen(centerEast - halfWm, n, size);
      final p2 = _toScreen(centerEast + halfWm, n, size);
      canvas.drawLine(p1, p2, n.abs() < 1e-6 ? axisPaint : gridPaint);
      _drawText(canvas, _fmt(n, gridStepM), Offset(4, p1.dy - 12),
          Colors.white38, 9);
    }

    // ── trajectory ──────────────────────────────────────────────
    if (trajectory.length >= 2) {
      final path = Path();
      final first = _toScreen(trajectory[0].$1, trajectory[0].$2, size);
      path.moveTo(first.dx, first.dy);
      for (int i = 1; i < trajectory.length; i++) {
        final p = _toScreen(trajectory[i].$1, trajectory[i].$2, size);
        path.lineTo(p.dx, p.dy);
      }
      canvas.drawPath(
        path,
        Paint()
          ..color = Colors.blueAccent.withOpacity(0.8)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2,
      );
    }

    // ── waypoint connection lines ──────────────────────────────────
    if (waypoints.length >= 2) {
      final path = Path();
      final first = _toScreen(waypoints[0].$1, waypoints[0].$2, size);
      path.moveTo(first.dx, first.dy);
      for (int i = 1; i < waypoints.length; i++) {
        final p = _toScreen(waypoints[i].$1, waypoints[i].$2, size);
        path.lineTo(p.dx, p.dy);
      }
      canvas.drawPath(
        path,
        Paint()
          ..color = Colors.black.withOpacity(0.8)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1,
      );
    }

    // ── spline path ───────────────────────────────────────
    if (splineENU.length >= 2) {
      final path = Path();
      final first = _toScreen(splineENU[0].$1, splineENU[0].$2, size);
      path.moveTo(first.dx, first.dy);
      for (int i = 1; i < splineENU.length; i++) {
        final p = _toScreen(splineENU[i].$1, splineENU[i].$2, size);
        path.lineTo(p.dx, p.dy);
      }
      canvas.drawPath(
        path,
        Paint()
          ..color = Colors.white.withOpacity(0.7)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2,
      );
    }

    // ── waypoint markers ─────────────────────────────────────
    for (int i = 0; i < waypoints.length; i++) {
      final p = _toScreen(waypoints[i].$1, waypoints[i].$2, size);
      canvas.drawCircle(p, 10, Paint()..color = Colors.white.withOpacity(0.9));
      canvas.drawCircle(
        p,
        10,
        Paint()
          ..color = Colors.black54
          ..style = PaintingStyle.stroke
          ..strokeWidth = 2,
      );
      _drawText(canvas, '${i + 1}', p - const Offset(3, 5), Colors.black, 10);
    }

    // ── vehicle marker (screen center, yaw direction) ───────────────────
    final vehiclePos = _toScreen(centerEast, centerNorth, size);
    canvas.save();
    canvas.translate(vehiclePos.dx, vehiclePos.dy);
    canvas.rotate(yawDeg * pi / 180.0); // 0°=north (up), clockwise

    final arrow = Path();
    arrow.moveTo(0, -10);
    arrow.lineTo(7, 8);
    arrow.lineTo(0, 3);
    arrow.lineTo(-7, 8);
    arrow.close();
    canvas.drawPath(arrow, Paint()..color = Colors.yellowAccent);
    canvas.drawPath(
      arrow,
      Paint()
        ..color = Colors.black.withOpacity(0.5)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1,
    );
    canvas.restore();

    // ── axis labels (E / N) ──────────────────────────────────
    final origin = _toScreen(0, 0, size);
    _drawText(canvas, 'E', Offset(size.width - 18, origin.dy - 16),
        Colors.cyanAccent, 13);
    _drawText(
        canvas, 'N', Offset(origin.dx + 6, 6), Colors.cyanAccent, 13);
  }

  String _fmt(double v, double step) {
    if (v.abs() < 1e-9) v = 0;
    return step < 1 ? v.toStringAsFixed(1) : v.toStringAsFixed(0);
  }

  void _drawText(
      Canvas canvas, String text, Offset offset, Color color, double size) {
    final tp = TextPainter(
      text: TextSpan(
        text: text,
        style: TextStyle(
          color: color,
          fontSize: size,
          fontFamily: 'monospace',
          fontWeight: FontWeight.w500,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, offset);
  }

  @override
  bool shouldRepaint(covariant _EnuGridPainter old) => true;
}
