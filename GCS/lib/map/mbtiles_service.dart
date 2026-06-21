// lib/map/mbtiles_service.dart
//
// reads tile data from a .mbtiles (sqlite) file bundled in app assets.
// on first run, copies assets → local file and opens with sqflite (ffi).
// also reads minzoom/maxzoom/bounds (center coordinates) from the metadata table.

import 'dart:io';
import 'dart:math';
import 'dart:typed_data';

import 'package:flutter/services.dart' show rootBundle;
import 'package:latlong2/latlong.dart';
import 'package:path_provider/path_provider.dart';
import 'package:sqflite_common_ffi/sqflite_ffi.dart';

class MbtilesService {
  Database? _db;
  bool _initFailed = false;

  int? minZoom;
  int? maxZoom;

  /// center coordinates derived from bounds (center of minlon,minlat,maxlon,maxlat)
  LatLng? centerLatLng;

  /// path to the mbtiles file bundled in assets
  static const String _assetPath = 'assets/map.mbtiles';

  /// filename for the local copy
  static const String _localFileName = 'map.mbtiles';

  Future<void> init() async {
    try {
      sqfliteFfiInit();
      databaseFactory = databaseFactoryFfi;

      final dir = await getApplicationSupportDirectory();
      final dbPath = '${dir.path}${Platform.pathSeparator}$_localFileName';
      final file = File(dbPath);

      if (!await file.exists()) {
        final data = await rootBundle.load(_assetPath);
        await file.writeAsBytes(
          data.buffer.asUint8List(data.offsetInBytes, data.lengthInBytes),
          flush: true,
        );
      }

      _db = await databaseFactory.openDatabase(
        dbPath,
        options: OpenDatabaseOptions(readOnly: true),
      );

      await _loadMetadata();
    } catch (e) {
      // mbtiles asset missing or load failed → offline map unavailable
      _initFailed = true;
      _db = null;
    }
  }

  /// reads minzoom/maxzoom/bounds from the metadata table.
  /// falls back to estimating from the tiles table if not found.
  Future<void> _loadMetadata() async {
    final db = _db;
    if (db == null) return;

    try {
      final rows = await db.query(
        'metadata',
        columns: ['name', 'value'],
        where: 'name IN (?, ?, ?, ?)',
        whereArgs: ['minzoom', 'maxzoom', 'bounds', 'center'],
      );

      for (final row in rows) {
        final name = row['name'] as String?;
        final value = '${row['value']}';

        switch (name) {
          case 'minzoom':
            minZoom = int.tryParse(value);
            break;
          case 'maxzoom':
            maxZoom = int.tryParse(value);
            break;
          case 'bounds':
            // "minlon,minlat,maxlon,maxlat"
            final parts = value.split(',').map((s) => double.tryParse(s.trim())).toList();
            if (parts.length == 4 && parts.every((p) => p != null)) {
              final minLon = parts[0]!;
              final minLat = parts[1]!;
              final maxLon = parts[2]!;
              final maxLat = parts[3]!;
              centerLatLng = LatLng(
                (minLat + maxLat) / 2.0,
                (minLon + maxLon) / 2.0,
              );
            }
            break;
          case 'center':
            // some generators provide center directly as "lon,lat"
            final parts = value.split(',').map((s) => double.tryParse(s.trim())).toList();
            if (parts.length >= 2 && parts[0] != null && parts[1] != null) {
              centerLatLng = LatLng(parts[1]!, parts[0]!);
            }
            break;
        }
      }
    } catch (_) {
      // ignore if metadata table is absent
    }

    // if minzoom/maxzoom is absent, query directly from the tiles table
    if (minZoom == null || maxZoom == null) {
      try {
        final rows = await db.rawQuery(
          'SELECT MIN(zoom_level) AS minz, MAX(zoom_level) AS maxz FROM tiles',
        );
        if (rows.isNotEmpty) {
          minZoom ??= rows.first['minz'] as int?;
          maxZoom ??= rows.first['maxz'] as int?;
        }
      } catch (_) {}
    }

    // if center is absent, estimate from tile bounds at the max zoom level
    if (centerLatLng == null && maxZoom != null) {
      try {
        final rows = await db.rawQuery(
          'SELECT MIN(tile_column) AS minx, MAX(tile_column) AS maxx, '
          'MIN(tile_row) AS miny, MAX(tile_row) AS maxy '
          'FROM tiles WHERE zoom_level = ?',
          [maxZoom],
        );
        if (rows.isNotEmpty) {
          final minX = rows.first['minx'] as int?;
          final maxX = rows.first['maxx'] as int?;
          final minY = rows.first['miny'] as int?;
          final maxY = rows.first['maxy'] as int?;
          if (minX != null && maxX != null && minY != null && maxY != null) {
            final z = maxZoom!;
            final cx = (minX + maxX + 1) / 2.0;
            // tile_row uses TMS (inverted) coordinates; convert to XYZ
            final cyTms = (minY + maxY + 1) / 2.0;
            final cyXyz = (1 << z) - cyTms;
            centerLatLng = _tileToLatLng(cx, cyXyz, z);
          }
        }
      } catch (_) {}
    }
  }

  LatLng _tileToLatLng(double x, double y, int z) {
    final n = 1 << z;
    final lon = x / n * 360.0 - 180.0;
    final latRad = atanSinh(pi * (1 - 2 * y / n));
    final lat = latRad * 180.0 / pi;
    return LatLng(lat, lon);
  }

  double atanSinh(double x) {
    // atan(sinh(x))
    final e2x = exp(2 * x);
    final sinhX = (e2x - 1) / (2 * exp(x));
    return atan(sinhX);
  }

  bool get isAvailable => _db != null && !_initFailed;

  /// returns PNG/JPEG bytes for XYZ coordinates (z, x, y)
  /// MBTiles uses the TMS scheme (y-axis inverted)
  Future<Uint8List?> getTile(int z, int x, int y) async {
    final db = _db;
    if (db == null) return null;

    final tmsY = (1 << z) - 1 - y;

    try {
      final result = await db.query(
        'tiles',
        columns: ['tile_data'],
        where: 'zoom_level = ? AND tile_column = ? AND tile_row = ?',
        whereArgs: [z, x, tmsY],
        limit: 1,
      );
      if (result.isEmpty) return null;

      Uint8List bytes = result.first['tile_data'] as Uint8List;

      // decompress if gzip-compressed (e.g. vector tiles)
      if (bytes.length > 2 && bytes[0] == 0x1F && bytes[1] == 0x8B) {
        bytes = Uint8List.fromList(gzip.decode(bytes));
      }
      return bytes;
    } catch (_) {
      return null;
    }
  }

  Future<void> close() async {
    await _db?.close();
    _db = null;
  }
}
