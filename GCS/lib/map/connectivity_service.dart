// lib/map/connectivity_service.dart
//
// periodically checks and caches OSM tile server reachability.
// pre-determines online/offline status so HybridTileProvider does not
// wait for a timeout on every tile request.

import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;

class ConnectivityService extends ChangeNotifier {
  static const _checkUrl = 'https://tile.openstreetmap.org/0/0/0.png';
  static const _checkInterval = Duration(seconds: 15);
  static const _checkTimeout = Duration(seconds: 2);

  bool _isOnline = true; // assume online until first check completes
  bool _checked = false;
  bool _disposed = false;
  Timer? _timer;

  bool get isOnline => _isOnline;
  bool get checked => _checked;

  ConnectivityService() {
    _check();
    _timer = Timer.periodic(_checkInterval, (_) => _check());
  }

  Future<void> _check() async {
    bool result;
    try {
      final res = await http.get(Uri.parse(_checkUrl)).timeout(_checkTimeout);
      result = res.statusCode == 200;
    } catch (_) {
      result = false;
    }

    if (_disposed) return;

    final firstCheck = !_checked;
    _checked = true;

    if (result != _isOnline || firstCheck) {
      _isOnline = result;
      notifyListeners();
    }
  }

  /// immediately re-check (e.g. when the user presses a retry button)
  Future<void> recheck() => _check();

  @override
  void dispose() {
    _disposed = true;
    _timer?.cancel();
    super.dispose();
  }
}
