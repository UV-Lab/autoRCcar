import 'package:flutter/material.dart';
import '../../ros2/gcs_controller.dart';
import 'package:file_selector/file_selector.dart';
import 'dart:typed_data';

class ControlPanel extends StatefulWidget {
  final GcsController ctrl;
  final VoidCallback onSetOrigin;

  const ControlPanel({
    super.key,
    required this.ctrl,
    required this.onSetOrigin,
  });

  @override
  State<ControlPanel> createState() => _ControlPanelState();
}

class _ControlPanelState extends State<ControlPanel> {
  GcsController get ctrl => widget.ctrl;
  VoidCallback get onSetOrigin => widget.onSetOrigin;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(12),
      decoration: const BoxDecoration(
        border: Border(top: BorderSide(color: Colors.white12)),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          // _modeBanner(),
          // const SizedBox(height: 10),

          // clear all (single row)
          _gcsButton(
            label: 'Clear All',
            icon: Icons.clear_all,
            color: Colors.tealAccent,
            onPressed: ctrl.clearAll,
          ),
          const SizedBox(height: 6),
          
          // send path (single row)
          _gcsButton(
            label: 'Send Path',
            icon: Icons.send,
            color: Colors.tealAccent,
            onPressed: ctrl.waypoints.isNotEmpty ? ctrl.sendGlobalPath : null,
          ),
          const SizedBox(height: 6),

          // import path / export path (two columns)
          Row(
            children: [
              Expanded(
                child: _gcsButton(
                  label: 'Import Path',
                  icon: Icons.upload_file,
                  color: Colors.tealAccent,
                  onPressed: () => _importPath(context),
                  small: true,
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: _gcsButton(
                  label: 'Export Path',
                  icon: Icons.download,
                  color: Colors.tealAccent,
                  onPressed: ctrl.waypoints.isNotEmpty
                      ? () => _exportPath(context)
                      : null,
                  small: true,
                ),
              ),
            ],
          ),
          const SizedBox(height: 6),

          // set origin / set yaw (two columns)
          Row(
            children: [
              Expanded(
                child: _gcsButton(
                  label: 'Set Origin',
                  icon: Icons.my_location,
                  color: Colors.tealAccent,
                  onPressed: onSetOrigin,
                  small: true,
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: _gcsButton(
                  label: 'Set Yaw',
                  icon: Icons.rotate_right,
                  color: Colors.tealAccent,
                  onPressed: () => _showSetYawDialog(context),
                  small: true,
                ),
              ),
            ],
          ),
          const SizedBox(height: 6),

          // Start / Stop
          Row(
            children: [
              Expanded(
                child: _gcsButton(
                  label: 'START',
                  icon: Icons.play_arrow,
                  color: Colors.yellowAccent,
                  onPressed: ctrl.isTeleop ? null : ctrl.sendStart,
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: _gcsButton(
                  label: 'STOP',
                  icon: Icons.stop,
                  color: Colors.redAccent,
                  onPressed: ctrl.sendStop,
                ),
              ),
            ],
          ),
          const SizedBox(height: 4),

          if (ctrl.waypoints.isNotEmpty)
            Text(
              'Waypoints: ${ctrl.waypoints.length}  |  Spline: ${ctrl.splinePath.length}pts',
              style: const TextStyle(color: Colors.white38, fontSize: 10),
              textAlign: TextAlign.center,
            ),
        ],
      ),
    );
  }

  void _exportPath(BuildContext context) async {
    final now = DateTime.now();
    final timestamp =
        '${now.year}_${now.month.toString().padLeft(2, '0')}_${now.day.toString().padLeft(2, '0')}'
        '_${now.hour.toString().padLeft(2, '0')}_${now.minute.toString().padLeft(2, '0')}_${now.second.toString().padLeft(2, '0')}';

    final location = await getSaveLocation(
      suggestedName: 'waypoints_$timestamp.txt',
      acceptedTypeGroups: [
        const XTypeGroup(label: 'text', extensions: ['txt']),
      ],
    );
    if (location != null) {
      final content = ctrl.exportPathToText();
      final file = XFile.fromData(
        Uint8List.fromList(content.codeUnits),
        mimeType: 'text/plain',
      );
      await file.saveTo(location.path);
    }
  }

  void _importPath(BuildContext context) async {
    const XTypeGroup typeGroup = XTypeGroup(
      label: 'text',
      extensions: ['txt'],
    );
    final XFile? file = await openFile(acceptedTypeGroups: [typeGroup]);
    if (file != null) {
      final content = await file.readAsString();
      ctrl.importPathFromText(content);
    }
  }

  // Widget _modeBanner() {
  //   final isTeleop = ctrl.isTeleop;
  //   return Container(
  //     width: double.infinity,
  //     padding: const EdgeInsets.symmetric(vertical: 8),
  //     decoration: BoxDecoration(
  //       color: isTeleop
  //           ? Colors.orange.withOpacity(0.2)
  //           : Colors.green.withOpacity(0.2),
  //       borderRadius: BorderRadius.circular(6),
  //       border: Border.all(
  //         color: isTeleop ? Colors.orange : Colors.green,
  //         width: 1.5,
  //       ),
  //     ),
  //     child: Text(
  //       isTeleop ? 'TELEOP MODE' : 'AUTO MODE',
  //       textAlign: TextAlign.center,
  //       style: TextStyle(
  //         color: isTeleop ? Colors.orange : Colors.green,
  //         fontWeight: FontWeight.bold,
  //         fontSize: 13,
  //         letterSpacing: 1.5,
  //       ),
  //     ),
  //   );
  // }

  Widget _gcsButton({
    required String label,
    required IconData icon,
    required Color color,
    VoidCallback? onPressed,
    bool small = false,
  }) {
    final isDisabled = onPressed == null;
    return SizedBox(
      width: double.infinity,
      height: small ? 36 : 40,
      child: ElevatedButton.icon(
        style: ElevatedButton.styleFrom(
          backgroundColor:
              isDisabled ? Colors.white10 : color.withOpacity(0.15),
          foregroundColor: isDisabled ? Colors.white24 : color,
          side: BorderSide(
              color: isDisabled ? Colors.white12 : color.withOpacity(0.5)),
          shape:
              RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
          padding: EdgeInsets.zero,
        ),
        icon: Icon(icon, size: small ? 14 : 16),
        label: Text(
          label,
          style: TextStyle(
              fontSize: small ? 11 : 12, fontWeight: FontWeight.bold),
        ),
        onPressed: onPressed,
      ),
    );
  }

  void _showSetYawDialog(BuildContext context) {
    final controller = TextEditingController();
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF16213E),
        title: const Text('Set Yaw',
            style: TextStyle(color: Colors.white, fontSize: 16)),
        content: TextField(
          controller: controller,
          keyboardType: const TextInputType.numberWithOptions(
              signed: true, decimal: true),
          style: const TextStyle(color: Colors.white),
          decoration: const InputDecoration(
            labelText: '진북 기준 Yaw 각도 [deg]',
            labelStyle: TextStyle(color: Colors.white54),
            enabledBorder: OutlineInputBorder(
                borderSide: BorderSide(color: Colors.white24)),
            focusedBorder: OutlineInputBorder(
                borderSide: BorderSide(color: Colors.blueAccent)),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel',
                style: TextStyle(color: Colors.white54)),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
                backgroundColor: Colors.purpleAccent.withOpacity(0.3)),
            onPressed: () {
              final parsed = double.tryParse(controller.text);
              if (parsed != null) {
                double val = parsed;
                while (val > 180) val = val - 360;
                while (val <= -180) val = val + 360;
                ctrl.sendSetYaw(val);
                Navigator.pop(ctx);
              }
            },
            child: const Text('Set',
                style: TextStyle(color: Colors.purpleAccent)),
          ),
        ],
      ),
    );
  }
}