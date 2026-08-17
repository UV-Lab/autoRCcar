// lib/screens/widgets/occupancy_grid_view.dart
//
// converts /occupancy_grid (nav_msgs/OccupancyGrid) to a pixel image for display.
// even large grids (e.g. 1000x1000) are fast because they are converted to ui.Image once and redrawn.
// supports pan/zoom via InteractiveViewer.

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';
import '../../ros2/gcs_controller.dart';
import '../../models/occupancy_grid.dart';

class OccupancyGridView extends StatefulWidget {
  final GcsController ctrl;
  const OccupancyGridView({super.key, required this.ctrl});

  @override
  State<OccupancyGridView> createState() => _OccupancyGridViewState();
}

class _OccupancyGridViewState extends State<OccupancyGridView> {
  ui.Image? _image;
  int _lastVersion = -1;
  bool _building = false;

  @override
  void initState() {
    super.initState();
    _maybeRebuild();
  }

  @override
  void didUpdateWidget(covariant OccupancyGridView oldWidget) {
    super.didUpdateWidget(oldWidget);
    _maybeRebuild();
  }

  void _maybeRebuild() {
    final grid = widget.ctrl.occupancyGrid;
    if (grid == null) return;
    if (widget.ctrl.occupancyGridVersion == _lastVersion || _building) return;

    _building = true;
    _lastVersion = widget.ctrl.occupancyGridVersion;

    _buildImage(grid).then((img) {
      _building = false;
      if (!mounted) return;
      setState(() => _image = img);
      // if new data arrived while building, rebuild
      if (widget.ctrl.occupancyGridVersion != _lastVersion) {
        _maybeRebuild();
      }
    });
  }

  Future<ui.Image> _buildImage(OccupancyGridMsg grid) async {
    final w = grid.width;
    final h = grid.height;
    final pixels = Uint8List(w * h * 4);

    for (int row = 0; row < h; row++) {
      // OccupancyGrid data row 0 = origin (min y) → bottom in image
      // flip vertically so +y (north) is at the top of the screen
      final imgRow = h - 1 - row;
      for (int col = 0; col < w; col++) {
        final v = grid.data[row * w + col];
        int gray;
        int alpha = 255;
        if (v < 0) {
          // unknown
          gray = 60;
          alpha = 200;
        } else {
          final occ = v.clamp(0, 100);
          gray = (255 * (100 - occ) / 100).round();
        }
        final idx = (imgRow * w + col) * 4;
        pixels[idx] = gray;
        pixels[idx + 1] = gray;
        pixels[idx + 2] = gray;
        pixels[idx + 3] = alpha;
      }
    }

    final completer = Completer<ui.Image>();
    ui.decodeImageFromPixels(
      pixels,
      w,
      h,
      ui.PixelFormat.rgba8888,
      (image) => completer.complete(image),
    );
    return completer.future;
  }

  @override
  Widget build(BuildContext context) {
    final grid = widget.ctrl.occupancyGrid;

    if (grid == null || _image == null) {
      return Container(
        color: const Color(0xFF1A1A2E),
        child: const Center(
          child: Text(
            '/occupancy_grid 토픽 수신 대기 중...',
            style: TextStyle(color: Colors.white38, fontSize: 13),
          ),
        ),
      );
    }

    return Container(
      color: Colors.black,
      child: Stack(
        children: [
          Positioned.fill(
            child: InteractiveViewer(
              minScale: 0.2,
              maxScale: 20,
              child: Center(
                child: SizedBox(
                  width: grid.width.toDouble(),
                  height: grid.height.toDouble(),
                  child: CustomPaint(
                    painter: _GridImagePainter(_image!),
                  ),
                ),
              ),
            ),
          ),
          // info display
          Positioned(
            bottom: 16,
            left: 16,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.black.withOpacity(0.6),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Text(
                '${grid.width} x ${grid.height}  |  ${grid.resolution.toStringAsFixed(3)} m/cell',
                style: const TextStyle(
                  color: Colors.white70,
                  fontSize: 11,
                  fontFamily: 'monospace',
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _GridImagePainter extends CustomPainter {
  final ui.Image image;
  _GridImagePainter(this.image);

  @override
  void paint(Canvas canvas, Size size) {
    paintImage(
      canvas: canvas,
      rect: Rect.fromLTWH(0, 0, size.width, size.height),
      image: image,
      fit: BoxFit.fill,
      filterQuality: FilterQuality.none,
    );
  }

  @override
  bool shouldRepaint(covariant _GridImagePainter oldDelegate) =>
      oldDelegate.image != image;
}
