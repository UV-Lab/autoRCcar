// lib/screens/widgets/system_monitor_panel.dart
//
// displays Jetson system status (CPU/memory/disk/temperature) + restart/shutdown controls.
// communicates with autorccar_util/process_manager_node on the vehicle via
// /util/system_status (subscribe) and /util/system_command (publish).

import 'package:flutter/material.dart';
import '../../ros2/gcs_controller.dart';
import '../../models/system_status.dart';

class SystemMonitorPanel extends StatelessWidget {
  final GcsController ctrl;
  const SystemMonitorPanel({super.key, required this.ctrl});

  @override
  Widget build(BuildContext context) {
    final status = ctrl.systemStatus;

    return Padding(
      padding: const EdgeInsets.all(12),
      child: Column(
        children: [
          if (status.isEmpty)
            const Padding(
              padding: EdgeInsets.symmetric(vertical: 8),
              child: Text(
                '/util/system_status 수신 대기 중...',
                style: TextStyle(color: Colors.white38, fontSize: 12),
              ),
            ),

          _metricBar(
            label: 'CPU',
            icon: Icons.memory,
            percent: status.cpuPercent,
          ),
          const SizedBox(height: 14),
          _metricBar(
            label: 'Memory',
            icon: Icons.storage,
            percent: status.memPercent,
            subtitle:
                '${status.memUsedMb.toStringAsFixed(0)} / ${status.memTotalMb.toStringAsFixed(0)} MB',
          ),
          const SizedBox(height: 14),
          _metricBar(
            label: 'Disk',
            icon: Icons.sd_storage,
            percent: status.diskPercent,
            subtitle:
                '${status.diskUsedGb.toStringAsFixed(1)} / ${status.diskTotalGb.toStringAsFixed(1)} GB',
          ),
          const SizedBox(height: 14),
          _temperatureCard(status.tempCelsius),

          const Spacer(),

          const Divider(color: Colors.white12),
          const SizedBox(height: 12),

          Row(
            children: [
              Expanded(
                child: _powerButton(
                  context,
                  label: 'Restart',
                  icon: Icons.restart_alt,
                  color: Colors.orangeAccent,
                  dialogTitle: 'Jetson 재부팅',
                  dialogMessage: '차량 컴퓨터(Jetson)를 재부팅하시겠습니까?\n실행 중인 모든 노드가 종료됩니다.',
                  onConfirm: () => ctrl.restartJetson(),
                ),
              ),
              const SizedBox(width: 8),
              Expanded(
                child: _powerButton(
                  context,
                  label: 'Shutdown',
                  icon: Icons.power_settings_new,
                  color: Colors.redAccent,
                  dialogTitle: 'Jetson 종료',
                  dialogMessage: '차량 컴퓨터(Jetson)를 종료하시겠습니까?\n다시 켜려면 차량 전원을 직접 조작해야 합니다.',
                  onConfirm: () => ctrl.shutdownJetson(),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  // ── color by level (0~60: green, 60~85: orange, 85+: red) ──
  Color _levelColor(double percent) {
    if (percent < 60) return Colors.greenAccent;
    if (percent < 85) return Colors.orangeAccent;
    return Colors.redAccent;
  }

  Color _tempColor(double tempC) {
    if (tempC < 60) return Colors.greenAccent;
    if (tempC < 80) return Colors.orangeAccent;
    return Colors.redAccent;
  }

  Widget _metricBar({
    required String label,
    required IconData icon,
    required double percent,
    String? subtitle,
  }) {
    final color = _levelColor(percent);
    final clamped = (percent / 100.0).clamp(0.0, 1.0);

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(icon, size: 14, color: Colors.white54),
            const SizedBox(width: 6),
            Text(
              label,
              style: const TextStyle(
                color: Colors.white70,
                fontSize: 12,
                fontWeight: FontWeight.bold,
              ),
            ),
            const Spacer(),
            Text(
              '${percent.toStringAsFixed(1)} %',
              style: TextStyle(
                color: color,
                fontSize: 12,
                fontFamily: 'monospace',
                fontWeight: FontWeight.bold,
              ),
            ),
          ],
        ),
        const SizedBox(height: 6),
        ClipRRect(
          borderRadius: BorderRadius.circular(4),
          child: LinearProgressIndicator(
            value: clamped,
            minHeight: 8,
            backgroundColor: Colors.white.withOpacity(0.08),
            valueColor: AlwaysStoppedAnimation<Color>(color),
          ),
        ),
        if (subtitle != null) ...[
          const SizedBox(height: 4),
          Text(
            subtitle,
            style: const TextStyle(
              color: Colors.white38,
              fontSize: 10,
              fontFamily: 'monospace',
            ),
          ),
        ],
      ],
    );
  }

  Widget _temperatureCard(double tempC) {
    final color = _tempColor(tempC);
    final valid = tempC >= 0;

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
      decoration: BoxDecoration(
        color: Colors.white.withOpacity(0.04),
        borderRadius: BorderRadius.circular(6),
        border: Border.all(color: color.withOpacity(0.3)),
      ),
      child: Row(
        children: [
          Icon(Icons.thermostat, size: 18, color: color),
          const SizedBox(width: 8),
          const Text(
            'CPU Temp',
            style: TextStyle(
              color: Colors.white70,
              fontSize: 12,
              fontWeight: FontWeight.bold,
            ),
          ),
          const Spacer(),
          Text(
            valid ? '${tempC.toStringAsFixed(1)} °C' : '-',
            style: TextStyle(
              color: color,
              fontSize: 14,
              fontFamily: 'monospace',
              fontWeight: FontWeight.bold,
            ),
          ),
        ],
      ),
    );
  }

  Widget _powerButton(
    BuildContext context, {
    required String label,
    required IconData icon,
    required Color color,
    required String dialogTitle,
    required String dialogMessage,
    required VoidCallback onConfirm,
  }) {
    return SizedBox(
      height: 44,
      child: ElevatedButton.icon(
        style: ElevatedButton.styleFrom(
          backgroundColor: color.withOpacity(0.15),
          foregroundColor: color,
          side: BorderSide(color: color.withOpacity(0.5)),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
        ),
        icon: Icon(icon, size: 18),
        label: Text(label,
            style: const TextStyle(fontSize: 13, fontWeight: FontWeight.bold)),
        onPressed: () => _confirm(
          context,
          title: dialogTitle,
          message: dialogMessage,
          color: color,
          onConfirm: onConfirm,
        ),
      ),
    );
  }

  void _confirm(
    BuildContext context, {
    required String title,
    required String message,
    required Color color,
    required VoidCallback onConfirm,
  }) {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF16213E),
        title: Text(title, style: const TextStyle(color: Colors.white, fontSize: 16)),
        content: Text(message, style: const TextStyle(color: Colors.white70, fontSize: 13)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel', style: TextStyle(color: Colors.white54)),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(backgroundColor: color.withOpacity(0.3)),
            onPressed: () {
              onConfirm();
              Navigator.pop(ctx);
            },
            child: Text('확인', style: TextStyle(color: color)),
          ),
        ],
      ),
    );
  }
}
