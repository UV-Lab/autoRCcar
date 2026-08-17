import 'dart:typed_data';
import 'package:flutter/material.dart';

class CameraOverlayWidget extends StatelessWidget {
  final Stream<Uint8List> imageStream;
  final bool isConnected;
  final bool isCameraActive;
  final VoidCallback onClose;

  const CameraOverlayWidget({
    super.key,
    required this.imageStream,
    required this.isConnected,
    required this.isCameraActive,
    required this.onClose,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 256,
      height: 192,
      decoration: BoxDecoration(
        color: Colors.black,
        border: Border.all(color: Colors.white24, width: 1),
        borderRadius: BorderRadius.circular(4),
        boxShadow: [
          BoxShadow(color: Colors.black.withOpacity(0.6), blurRadius: 8),
        ],
      ),
      child: Column(
        children: [
          _buildTitleBar(),
          Expanded(child: _buildVideoArea()),
        ],
      ),
    );
  }

  Widget _buildTitleBar() {
    return Container(
      height: 24,
      decoration: const BoxDecoration(
        color: Color(0xFF0F3460),
        borderRadius: BorderRadius.only(
          topLeft: Radius.circular(4),
          topRight: Radius.circular(4),
        ),
      ),
      child: Row(
        children: [
          const SizedBox(width: 8),
          const Icon(Icons.videocam, size: 13, color: Colors.tealAccent),
          const SizedBox(width: 5),
          const Text(
            'CAMERA',
            style: TextStyle(
              color: Colors.white70,
              fontSize: 10,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.5,
            ),
          ),
          const Spacer(),
          InkWell(
            onTap: onClose,
            child: const Padding(
              padding: EdgeInsets.symmetric(horizontal: 6, vertical: 4),
              child: Icon(Icons.close, size: 13, color: Colors.white38),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildVideoArea() {
    return StreamBuilder<Uint8List>(
      stream: imageStream,
      builder: (context, snapshot) {
        if (isCameraActive && snapshot.hasData) {
          return ClipRRect(
            borderRadius: const BorderRadius.only(
              bottomLeft: Radius.circular(4),
              bottomRight: Radius.circular(4),
            ),
            child: Image.memory(
              snapshot.data!,
              fit: BoxFit.cover,
              width: double.infinity,
              height: double.infinity,
              gaplessPlayback: true,
            ),
          );
        }
        return _buildPlaceholder();
      },
    );
  }

  Widget _buildPlaceholder() {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            isConnected ? Icons.videocam_off : Icons.wifi_off,
            color: Colors.white24,
            size: 30,
          ),
          const SizedBox(height: 6),
          Text(
            isConnected ? 'NO CAMERA SIGNAL' : 'NOT CONNECTED',
            style: const TextStyle(color: Colors.white24, fontSize: 9),
          ),
        ],
      ),
    );
  }
}
