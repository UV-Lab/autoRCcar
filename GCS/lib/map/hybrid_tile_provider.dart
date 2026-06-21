// lib/map/hybrid_tile_provider.dart
//
// flutter_map TileProvider: tries OSM online first; falls back to MBTiles on failure or offline.
// references the online status cached by ConnectivityService so that
// offline mode skips the network and reads MBTiles immediately (minimizes latency)

import 'dart:async';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:http/http.dart' as http;

import 'connectivity_service.dart';
import 'mbtiles_service.dart';

class HybridTileProvider extends TileProvider {
  final MbtilesService mbtiles;
  final ConnectivityService connectivity;
  final Duration onlineTimeout;

  HybridTileProvider(
    this.mbtiles,
    this.connectivity, {
    this.onlineTimeout = const Duration(seconds: 3),
  });

  @override
  ImageProvider getImage(TileCoordinates coordinates, TileLayer options) {
    return HybridTileImage(
      z: coordinates.z,
      x: coordinates.x,
      y: coordinates.y,
      mbtiles: mbtiles,
      isOnline: connectivity.isOnline,
      onlineTimeout: onlineTimeout,
    );
  }
}

class HybridTileImage extends ImageProvider<HybridTileImage> {
  final int z;
  final int x;
  final int y;
  final MbtilesService mbtiles;
  final bool isOnline;
  final Duration onlineTimeout;

  const HybridTileImage({
    required this.z,
    required this.x,
    required this.y,
    required this.mbtiles,
    required this.isOnline,
    required this.onlineTimeout,
  });

  @override
  Future<HybridTileImage> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<HybridTileImage>(this);
  }

  @override
  ImageStreamCompleter loadImage(
      HybridTileImage key, ImageDecoderCallback decode) {
    return MultiFrameImageStreamCompleter(
      codec: _load(decode),
      scale: 1.0,
      debugLabel: 'tile/$z/$x/$y (${isOnline ? "online" : "offline"})',
    );
  }

  Future<ui.Codec> _load(ImageDecoderCallback decode) async {
    Uint8List? bytes;

    // only attempt network if determined to be online (skip directly to mbtiles when offline)
    if (isOnline) {
      try {
        final url = 'https://tile.openstreetmap.org/$z/$x/$y.png';
        final res = await http
            .get(Uri.parse(url),
                headers: {'User-Agent': 'autorccar_gcs/1.0'})
            .timeout(onlineTimeout);
        if (res.statusCode == 200) {
          bytes = res.bodyBytes;
        }
      } catch (_) {
        // individual tile fetch failed — fall back to mbtiles
      }
    }

    // offline MBTiles fallback
    bytes ??= await mbtiles.getTile(z, x, y);

    // neither source available — return transparent tile
    bytes ??= _transparentPng;

    final buffer = await ui.ImmutableBuffer.fromUint8List(bytes);
    return decode(buffer);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) return true;
    return other is HybridTileImage &&
        other.z == z &&
        other.x == x &&
        other.y == y &&
        other.isOnline == isOnline;
  }

  @override
  int get hashCode => Object.hash(z, x, y, isOnline);
}

/// 1×1 transparent PNG returned when both online and offline sources fail
final Uint8List _transparentPng = Uint8List.fromList([
  0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
  0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4, 0x89, 0x00, 0x00, 0x00,
  0x0A, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0x63, 0x00, 0x01, 0x00, 0x00,
  0x05, 0x00, 0x01, 0x0D, 0x0A, 0x2D, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x49,
  0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82,
]);
