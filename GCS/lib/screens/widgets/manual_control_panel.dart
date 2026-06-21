// lib/screens/widgets/manual_control_panel.dart

import 'dart:math';
import 'package:flutter/material.dart';
import '../../ros2/gcs_controller.dart';

class ManualControlPanel extends StatelessWidget {
  final GcsController ctrl;
  const ManualControlPanel({super.key, required this.ctrl});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(12),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          // ── vehicle top view ───────────────────────────────────────
          Row(
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              // left: vehicle top view (fixed width)
              SizedBox(
                width: 260,
                height: 180,
                child: CustomPaint(
                  painter: CarTopViewPainter(
                    speed: ctrl.pwmCommand.speed,
                    steeringAngle: ctrl.pwmCommand.steeringAngle,
                  ),
                ),
              ),
              // const SizedBox(width: 8),
              // right: PWM values (fixed width)
              SizedBox(
                width: 90,
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _hwDisplay('HW.Speed', ctrl.teleopSpeed),
                    const SizedBox(height: 12),
                    _hwDisplay('HW.Steer', ctrl.teleopSteer),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 70),

          // ── mode buttons (3) ──────────────────────────────────────
          Row(
            children: [
              Expanded(child: _modeButton('STOP', Icons.stop_circle, Colors.redAccent, () => ctrl.sendTeleopCommand(0))),
              const SizedBox(width: 6),
              Expanded(child: _modeButton('TELEOP', Icons.gamepad, Colors.orangeAccent, () => ctrl.sendTeleopCommand(1))),
              const SizedBox(width: 6),
              Expanded(child: _modeButton('AUTO', Icons.smart_toy, Colors.greenAccent, () => ctrl.sendTeleopCommand(2))),
            ],
          ),
          const SizedBox(height: 40),

          // ── d-pad ─────────────────────────────────────────────────
          _buildDpad(),
          const SizedBox(height: 40),

          // ── function buttons below d-pad ──────────────────────────
          Row(
            children: [
              Expanded(
                child: _funcButton(
                  'Speed Reset',
                  Icons.speed,
                  ctrl.isTeleop ? Colors.orange : Colors.white38,
                  ctrl.isTeleop ? () => ctrl.sendSpeedReset() : null,
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: _funcButton(
                  'Steer Reset',
                  Icons.rotate_right,
                  ctrl.isTeleop ? Colors.orange : Colors.white38,
                  ctrl.isTeleop ? () => ctrl.sendSteerReset() : null,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildDpad() {
    return Column(
      children: [
        // up
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _arrowButton(Icons.keyboard_arrow_up, 'UP', () => ctrl.teleopSpeedUp()),
          ],
        ),
        // left / center gap / right
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _arrowButton(Icons.keyboard_arrow_left, 'LEFT', () => ctrl.teleopSteerLeft()),
            const SizedBox(width: 4),
            _centerButton(),
            const SizedBox(width: 4),
            _arrowButton(Icons.keyboard_arrow_right, 'RIGHT', () => ctrl.teleopSteerRight()),
          ],
        ),
        // down
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _arrowButton(Icons.keyboard_arrow_down, 'DOWN', () => ctrl.teleopSpeedDown()),
          ],
        ),
      ],
    );
  }

  Widget _hwDisplay(String label, double value) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          label,
          style: const TextStyle(
            color: Colors.white54,
            fontSize: 10,
            fontWeight: FontWeight.bold,
            letterSpacing: 1.5,
          ),
        ),
        const SizedBox(height: 4),
        Container(
          width: double.infinity,
          padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
          decoration: BoxDecoration(
            color: Colors.black.withOpacity(0.4),
            borderRadius: BorderRadius.circular(4),
            border: Border.all(color: Colors.white12),
          ),
          child: Text(
            value.toStringAsFixed(1),
            style: const TextStyle(
              color: Colors.cyanAccent,
              fontSize: 16,
              fontFamily: 'monospace',
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    );
  }

  Widget _arrowButton(IconData icon, String label, VoidCallback? onPressed) {
    final isActive = ctrl.isTeleop;
    return SizedBox(
      width: 72,
      height: 72,
      child: ElevatedButton(
        style: ElevatedButton.styleFrom(
          backgroundColor: isActive
              ? Colors.orange.withOpacity(0.15)
              : Colors.white.withOpacity(0.04),
          foregroundColor: isActive ? Colors.orange : Colors.white24,
          side: BorderSide(
              color: isActive ? Colors.orange.withOpacity(0.5) : Colors.white12),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
          padding: EdgeInsets.zero,
        ),
        onPressed: isActive ? onPressed : null,
        child: Icon(icon, size: 36),
      ),
    );
  }

  Widget _centerButton() {
    return Container(
      width: 72,
      height: 72,
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.04),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white12),
      ),
    );
  }

  Widget _modeButton(String label, IconData icon, Color color, VoidCallback onPressed) {
    return SizedBox(
      height: 40,
      child: ElevatedButton.icon(
        style: ElevatedButton.styleFrom(
          backgroundColor: color.withOpacity(0.15),
          foregroundColor: color,
          side: BorderSide(color: color.withOpacity(0.5)),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
          padding: EdgeInsets.zero,
        ),
        icon: Icon(icon, size: 14),
        label: Text(label,
            style: const TextStyle(fontSize: 11, fontWeight: FontWeight.bold)),
        onPressed: onPressed,
      ),
    );
  }

  Widget _funcButton(String label, IconData icon, Color color, VoidCallback? onPressed) {
    return SizedBox(
      height: 40,
      child: ElevatedButton.icon(
        style: ElevatedButton.styleFrom(
          backgroundColor: color.withOpacity(0.15),
          foregroundColor: color,
          side: BorderSide(color: color.withOpacity(0.5)),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
          padding: EdgeInsets.zero,
        ),
        icon: Icon(icon, size: 14),
        label: Text(label,
            style: const TextStyle(fontSize: 11, fontWeight: FontWeight.bold)),
        onPressed: onPressed,
      ),
    );
  }
}

// ── vehicle top view CustomPainter ──────────────────────────────────
class CarTopViewPainter extends CustomPainter {
  final double speed;
  final double steeringAngle; // deg

  CarTopViewPainter({required this.speed, required this.steeringAngle});

  @override
  void paint(Canvas canvas, Size size) {
    final w = size.width;
    final h = size.height;
    final cx = w / 2;
    final cy = h / 2;

    // body proportions
    final carW = w * 0.28;
    final carH = h * 0.75;
    final wheelW = carW * 0.22;
    final wheelH = carH * 0.22;
    final steerDeg = (steeringAngle - 87) * (45.0 / 77.0); // neutral at 87, mapped to ±45°
    final steerRad = steerDeg * pi / 180.0;

    final bodyPaint = Paint()
      ..color = Colors.blueGrey.shade700
      ..style = PaintingStyle.fill;

    final borderPaint = Paint()
      ..color = Colors.blueGrey.shade300
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;

    final wheelPaint = Paint()
      ..color = Colors.grey.shade600
      ..style = PaintingStyle.fill;

    final wheelBorderPaint = Paint()
      ..color = Colors.grey.shade400
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.0;

    final steerWheelPaint = Paint()
      ..color = Colors.cyanAccent.shade700
      ..style = PaintingStyle.fill;

    // ── body ─────────────────────────────────────────────────────
    final bodyRect = RRect.fromRectAndRadius(
      Rect.fromCenter(center: Offset(cx, cy), width: carW, height: carH),
      const Radius.circular(6),
    );
    canvas.drawRRect(bodyRect, bodyPaint);
    canvas.drawRRect(bodyRect, borderPaint);

    // front indicator line
    final frontLinePaint = Paint()
      ..color = Colors.cyanAccent.withOpacity(0.6)
      ..strokeWidth = 2.0;
    canvas.drawLine(
      Offset(cx - carW * 0.3, cy - carH * 0.26),
      Offset(cx + carW * 0.3, cy - carH * 0.26),
      frontLinePaint,
    );

    // ── wheel positions ──────────────────────────────────────────
    final frontY = cy - carH * 0.28;
    final rearY = cy + carH * 0.28;
    final leftX = cx - carW * 0.62;
    final rightX = cx + carW * 0.62;

    // rear wheels (fixed)
    _drawWheel(canvas, Offset(leftX, rearY), wheelW, wheelH, 0, wheelPaint, wheelBorderPaint);
    _drawWheel(canvas, Offset(rightX, rearY), wheelW, wheelH, 0, wheelPaint, wheelBorderPaint);

    // front wheels (with steering angle applied)
    _drawWheel(canvas, Offset(leftX, frontY), wheelW, wheelH, steerRad, steerWheelPaint, wheelBorderPaint);
    _drawWheel(canvas, Offset(rightX, frontY), wheelW, wheelH, steerRad, steerWheelPaint, wheelBorderPaint);

    // ── steering angle text (above front wheels) ─────────────────
    _drawText(
      canvas,
      '${steeringAngle.toInt()}',
      Offset(cx, frontY - wheelH * 0.3),
      Colors.cyanAccent,
      14,
    );

    // ── speed text (body center) ──────────────────────────────────
    _drawText(
      canvas,
      '${speed.toInt()}',
      Offset(cx, cy),
      Colors.white,
      14,
    );
  }

  void _drawWheel(Canvas canvas, Offset center, double w, double h,
      double angle, Paint fill, Paint border) {
    canvas.save();
    canvas.translate(center.dx, center.dy);
    canvas.rotate(angle);
    final rect = RRect.fromRectAndRadius(
      Rect.fromCenter(center: Offset.zero, width: w, height: h),
      const Radius.circular(2),
    );
    canvas.drawRRect(rect, fill);
    canvas.drawRRect(rect, border);
    canvas.restore();
  }

  void _drawText(Canvas canvas, String text, Offset offset, Color color, double fontSize) {
    final tp = TextPainter(
      text: TextSpan(
        text: text,
        style: TextStyle(
          color: color,
          fontSize: fontSize,
          fontWeight: FontWeight.bold,
          fontFamily: 'monospace',
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    tp.paint(canvas, offset - Offset(tp.width / 2, tp.height / 2));
  }

  @override
  bool shouldRepaint(CarTopViewPainter old) =>
      old.speed != speed || old.steeringAngle != steeringAngle;
}
