import 'package:flutter/material.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart';
import 'package:provider/provider.dart';
import '../ros2/gcs_controller.dart';
import '../ros2/rosbridge_service.dart' as rb;
import 'widgets/nav_status_panel.dart';
import 'widgets/control_panel.dart';
import 'widgets/connection_dialog.dart';
import 'widgets/manual_control_panel.dart';
import '../map/hybrid_tile_provider.dart';
import '../map/connectivity_service.dart';
import '../map/mbtiles_service.dart';
import 'widgets/occupancy_grid_view.dart';
import 'widgets/enu_plot_view.dart';
import 'widgets/run_panel.dart';
import 'widgets/system_monitor_panel.dart';
import 'widgets/llm_panel.dart';
import 'widgets/camera_overlay_widget.dart';

enum MapMode { osm, occupancyGrid, enuPlot }

class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  final MapController _mapController = MapController();
  bool _isSettingOrigin = false;
  double _currentZoom = 18.0;
  bool _showCamera = true;
  late GcsController _ctrl;

  MapMode _mapMode = MapMode.osm;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      _ctrl = context.read<GcsController>();
      _ctrl.onOriginSet = (latLng) {
        _mapController.move(latLng, 18);
      };
      _ctrl.enableCamera();
    });
  }

  @override
  void dispose() {
    _ctrl.disableCamera();
    super.dispose();
  }

  @override
    Widget build(BuildContext context) {
      final ctrl = context.watch<GcsController>();

      return Scaffold(
        backgroundColor: const Color(0xFF1A1A2E),
        appBar: _buildAppBar(ctrl),
        body: Row(
          children: [
            // ── map / occupancy grid / chart + LLM panel area ───────────
            Expanded(
              flex: 6,
              child: Column(
                children: [
                  _buildMapModeSelector(),
                  // map display area: reduced ratio to allow space for the LLM panel below
                  Expanded(
                    flex: 7,
                    child: switch (_mapMode) {
                      MapMode.osm => Stack(
                          children: [
                            _buildMap(ctrl),
                            _buildMapOverlay(ctrl),
                          ],
                        ),
                      MapMode.occupancyGrid => OccupancyGridView(ctrl: ctrl),
                      MapMode.enuPlot => EnuPlotView(ctrl: ctrl),
                    },
                  ),
                  // ── LLM natural language command interface ──────────────
                  LlmPanel(ctrl: ctrl),
                ],
              ),
            ),
            // ── right panel ────────────────────────────────────
            Container(
              width: 380,
              color: const Color(0xFF16213E),
              child: DefaultTabController(
                length: 4,
                child: Column(
                  children: [
                    const TabBar(
                      tabs: [
                        Tab(icon: Icon(Icons.navigation, size: 12), text: 'VEHICLE', height: 40),
                        Tab(icon: Icon(Icons.gamepad, size: 12), text: 'MANUAL', height: 40),
                        Tab(icon: Icon(Icons.play_circle, size: 12), text: 'RUN', height: 40),
                        Tab(icon: Icon(Icons.monitor_heart, size: 12), text: 'SYSTEM', height: 40),
                      ],
                      labelColor: Colors.white,
                      unselectedLabelColor: Colors.white38,
                      indicatorColor: Colors.tealAccent,
                      dividerColor: Colors.white12,
                      labelStyle: TextStyle(fontSize: 11),
                    ),
                    Expanded(
                      child: TabBarView(
                        children: [
                          // TAB 1: NAV STATUS + CONTROL BUTTONS
                          Column(
                            children: [
                              Expanded(child: NavStatusPanel(ctrl: ctrl)),
                              ControlPanel(
                                ctrl: ctrl,
                                onSetOrigin: _enterOriginMode,
                              ),
                            ],
                          ),
                          // TAB 2: MANUAL CONTROL
                          ManualControlPanel(ctrl: ctrl),
                          // TAB 3: RUN
                          RunPanel(ctrl: ctrl),
                          // TAB 4: SYSTEM
                          SystemMonitorPanel(ctrl: ctrl),
                        ],
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

  AppBar _buildAppBar(GcsController ctrl) {
    final isConnected = ctrl.connectionState == rb.ConnectionState.connected;
    final isConnecting = ctrl.connectionState == rb.ConnectionState.connecting;

    return AppBar(
      backgroundColor: const Color(0xFF0F3460),
      title: const Row(
        crossAxisAlignment: CrossAxisAlignment.baseline,
        textBaseline: TextBaseline.alphabetic,
        children: [
          Text('AutoRCcar GCS',
              style: TextStyle(color: Colors.white, fontSize: 28, fontWeight: FontWeight.bold)),
          SizedBox(width: 6),
          Text('\t© UV-Lab',
              style: TextStyle(color: Colors.white54, fontSize: 18)),
        ],
      ),
      actions: [
        // mode banner
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
            decoration: BoxDecoration(
              color: ctrl.isTeleop
                  ? Colors.orange.withOpacity(0.2)
                  : Colors.green.withOpacity(0.2),
              border: Border.all(
                color: ctrl.isTeleop ? Colors.orange : Colors.green,
              ),
              borderRadius: BorderRadius.circular(4),
            ),
            child: Text(
              ctrl.isTeleop ? 'TELEOP MODE' : 'AUTO MODE',
              style: TextStyle(
                color: ctrl.isTeleop ? Colors.orange : Colors.green,
                fontSize: 12,
                fontWeight: FontWeight.bold,
                letterSpacing: 1.5,
              ),
            ),
          ),
        ),
        // connection status indicator
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 8),
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
            decoration: BoxDecoration(
              color: isConnected
                  ? Colors.green.withOpacity(0.2)
                  : isConnecting
                      ? Colors.orange.withOpacity(0.2)
                      : Colors.red.withOpacity(0.2),
              border: Border.all(
                color: isConnected
                    ? Colors.green
                    : isConnecting
                        ? Colors.orange
                        : Colors.red,
              ),
              borderRadius: BorderRadius.circular(4),
            ),
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                Icon(
                  isConnected
                      ? Icons.wifi
                      : isConnecting
                          ? Icons.wifi_find
                          : Icons.wifi_off,
                  color: isConnected
                      ? Colors.green
                      : isConnecting
                          ? Colors.orange
                          : Colors.red,
                  size: 16,
                ),
                const SizedBox(width: 6),
                Text(
                  isConnected
                      ? 'CONNECTED'
                      : isConnecting
                          ? 'CONNECTING...'
                          : 'DISCONNECTED',
                  style: TextStyle(
                    color: isConnected
                        ? Colors.green
                        : isConnecting
                            ? Colors.orange
                            : Colors.red,
                    fontSize: 12,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ],
            ),
          ),
        ),
        // connection settings button
        IconButton(
          icon: const Icon(Icons.settings_ethernet, color: Colors.white70),
          tooltip: 'rosbridge 연결 설정',
          onPressed: () => _showConnectionDialog(context),
        ),
        const SizedBox(width: 8),
      ],
    );
  }

  Widget _buildMap(GcsController ctrl) {
    final mbtiles = context.read<MbtilesService>();
    final connectivity = context.watch<ConnectivityService>();

    // priority: vehicle position > mbtiles center > default
    final center = ctrl.vehiclePosition
        ?? mbtiles.centerLatLng
        ?? const LatLng(37.3595, 127.1052);

    // restrict to mbtiles zoom range only when offline
    double minZoom = 1;
    double maxZoom = 19;
    if (!connectivity.isOnline && mbtiles.isAvailable) {
      minZoom = (mbtiles.minZoom ?? 1).toDouble();
      maxZoom = (mbtiles.maxZoom ?? 19).toDouble();
    }

    return FlutterMap(
      mapController: _mapController,
      options: MapOptions(
        initialCenter: center,
        initialZoom: 18,
        minZoom: minZoom,
        maxZoom: maxZoom,
        onPositionChanged: (position, hasGesture) {
          setState(() {
            _currentZoom = position.zoom ?? _currentZoom;
          });
        },
        onTap: (tapPos, latLng) {
          if (_isSettingOrigin) {
            ctrl.setOrigin(latLng);
            setState(() => _isSettingOrigin = false);
            ScaffoldMessenger.of(context).showSnackBar(
              SnackBar(
                content: Text(
                  'Origin 설정: ${latLng.latitude.toStringAsFixed(6)}, ${latLng.longitude.toStringAsFixed(6)}',
                ),
                backgroundColor: Colors.green.shade700,
                duration: const Duration(seconds: 2),
              ),
            );
          } else {
            ctrl.addWaypoint(latLng);
          }
        },
      ),
      children: [
        // OSM tiles
        TileLayer(
          tileProvider: HybridTileProvider(
            context.read<MbtilesService>(),
            context.watch<ConnectivityService>(),
          ),
          userAgentPackageName: 'com.example.autorccar_gcs',
        ),
        // driving trajectory
        if (ctrl.trajectory.isNotEmpty)
          PolylineLayer(
            polylines: [
              Polyline(
                points: ctrl.trajectory,
                strokeWidth: 2.5,
                color: Colors.blueAccent.withOpacity(0.8),
              ),
            ],
          ),
        // spline path
        if (ctrl.splinePath.isNotEmpty)
          PolylineLayer(
            polylines: [
              Polyline(
                points: ctrl.splinePath,
                strokeWidth: 2.0,
                color: Colors.white.withOpacity(0.7),
              ),
            ],
          ),
        // waypoint connection lines
        if (ctrl.waypoints.length >= 2)
          PolylineLayer(
            polylines: [
              Polyline(
                points: ctrl.waypoints.map((w) => w.latLng).toList(),
                strokeWidth: 1.0,
                color: Colors.black.withOpacity(0.8),
              ),
            ],
          ),
        // waypoint markers
        MarkerLayer(
          markers: [
            // Waypoints
            ...ctrl.waypoints.asMap().entries.map((e) {
              final idx = e.key;
              final wp = e.value;
              return Marker(
                point: wp.latLng,
                width: 36,
                height: 36,
                child: GestureDetector(
                  onSecondaryTap: () => ctrl.removeWaypoint(idx),
                  child: Stack(
                    alignment: Alignment.center,
                    children: [
                      Container(
                        width: 24,
                        height: 24,
                        decoration: BoxDecoration(
                          color: Colors.white.withOpacity(0.9),
                          shape: BoxShape.circle,
                          border: Border.all(color: Colors.black54, width: 2),
                        ),
                      ),
                      Text(
                        '${idx + 1}',
                        style: const TextStyle(
                          fontSize: 10,
                          fontWeight: FontWeight.bold,
                          color: Colors.black,
                        ),
                      ),
                    ],
                  ),
                ),
              );
            }),
            // vehicle marker
            if (ctrl.vehiclePosition != null)
              Marker(
                point: ctrl.vehiclePosition!,
                width: 40,
                height: 40,
                child: Transform.rotate(
                  angle: _yawToMapAngle(ctrl.yawDeg),
                  child: const Icon(
                    Icons.navigation,
                    color: Colors.blueAccent,
                    size: 32,
                    shadows: [Shadow(color: Colors.black, blurRadius: 4)],
                  ),
                ),
              ),
            // origin marker
            if (ctrl.hasOrigin && ctrl.vehiclePosition == null)
              Marker(
                point: _mapController.camera.center,
                width: 20,
                height: 20,
                child: const Icon(Icons.add, color: Colors.red, size: 20),
              ),
          ],
        ),
      ],
    );
  }

  Widget _buildMapModeSelector() {
    return Container(
      color: const Color(0xFF16213E),
      height: 40,
      child: Row(
        children: [
          _mapModeButton('Map', MapMode.osm, Icons.map),
          _mapModeButton('Occupancy Grid', MapMode.occupancyGrid, Icons.grid_on),
          _mapModeButton('ENU Plot', MapMode.enuPlot, Icons.scatter_plot),
        ],
      ),
    );
  }

  Widget _mapModeButton(String label, MapMode mode, IconData icon) {
    final selected = _mapMode == mode;
    return SizedBox(
      height: 40,
      child: InkWell(
        onTap: () {
          final ctrl = context.read<GcsController>();
          if (mode == MapMode.occupancyGrid && _mapMode != MapMode.occupancyGrid) {
            ctrl.enableOccupancyGrid();
          } else if (mode != MapMode.occupancyGrid && _mapMode == MapMode.occupancyGrid) {
            ctrl.disableOccupancyGrid();
          }
          setState(() => _mapMode = mode);
        },
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 12),
          decoration: BoxDecoration(
            border: Border(
              bottom: BorderSide(
                color: selected ? Colors.tealAccent : Colors.transparent,
                width: 2,
              ),
            ),
          ),
          child: Row(
            mainAxisSize: MainAxisSize.min,
            mainAxisAlignment: MainAxisAlignment.start,
            children: [
              const SizedBox(width: 12),
              Icon(
                icon,
                size: 18,
                color: selected ? Colors.white : Colors.white38,
              ),
              const SizedBox(width: 6),
              Text(
                label,
                style: TextStyle(
                  fontSize: 14,
                  color: selected ? Colors.white : Colors.white38,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildMapOverlay(GcsController ctrl) {
    final connectivity = context.watch<ConnectivityService>();
    final isConnected = ctrl.connectionState == rb.ConnectionState.connected;

    return Stack(
      children: [
        // top-left: online/offline status
        if (connectivity.checked)
          Positioned(
            top: 16,
            left: 16,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
              decoration: BoxDecoration(
                color: connectivity.isOnline
                    ? Colors.green.withOpacity(0.7)
                    : Colors.red.withOpacity(0.8),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Row(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(
                    connectivity.isOnline ? Icons.wifi : Icons.wifi_off,
                    color: Colors.white,
                    size: 14,
                  ),
                  const SizedBox(width: 6),
                  Text(
                    connectivity.isOnline ? 'ONLINE MAP' : 'OFFLINE - LOCAL MAP',
                    style: const TextStyle(color: Colors.white, fontSize: 11, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
            ),
          ),
        // bottom-left: camera video + text overlay
        Positioned(
          bottom: 16,
          left: 16,
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: [
              // camera video (togglable)
              if (_showCamera)
                Padding(
                  padding: const EdgeInsets.only(bottom: 8),
                  child: CameraOverlayWidget(
                    imageStream: ctrl.cameraStream,
                    isConnected: isConnected,
                    isCameraActive: ctrl.processStatus['gscam'] == 'running',
                    onClose: () => setState(() => _showCamera = false),
                  ),
                ),
              // text overlay
              if (_isSettingOrigin)
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                  margin: const EdgeInsets.only(bottom: 8),
                  decoration: BoxDecoration(
                    color: Colors.orange.withOpacity(0.9),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: const Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.touch_app, color: Colors.white, size: 16),
                      SizedBox(width: 6),
                      Text(
                        '지도를 클릭하여 Origin 설정',
                        style: TextStyle(color: Colors.white, fontSize: 13),
                      ),
                    ],
                  ),
                ),
              if (!_isSettingOrigin && ctrl.hasOrigin)
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.6),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: Text(
                    '좌클릭: Waypoint 추가  |  우클릭: 삭제  |  ${ctrl.waypoints.length}개',
                    style: const TextStyle(color: Colors.white70, fontSize: 11),
                  ),
                ),
              if (!ctrl.hasOrigin && !_isSettingOrigin)
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
                  decoration: BoxDecoration(
                    color: Colors.red.withOpacity(0.7),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: const Text(
                    '⚠ ROS2 연결 후 Origin 자동 설정됩니다',
                    style: TextStyle(color: Colors.white, fontSize: 11),
                  ),
                ),
            ],
          ),
        ),
        // restore button when camera is hidden
        if (!_showCamera)
          Positioned(
            bottom: 50,
            left: 16,
            child: Tooltip(
              message: '카메라 영상 표시',
              child: InkWell(
                onTap: () => setState(() => _showCamera = true),
                child: Container(
                  padding: const EdgeInsets.all(6),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.6),
                    borderRadius: BorderRadius.circular(4),
                    border: Border.all(color: Colors.white24),
                  ),
                  child: const Icon(Icons.videocam, color: Colors.white54, size: 18),
                ),
              ),
            ),
          ),
        // bottom-right zoom display
        Positioned(
          bottom: 16,
          right: 16,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
            decoration: BoxDecoration(
              color: Colors.black.withOpacity(0.6),
              borderRadius: BorderRadius.circular(6),
            ),
            child: Text(
              'Zoom: ${_currentZoom.toStringAsFixed(1)}',
              style: const TextStyle(
                color: Colors.white70,
                fontSize: 11,
                fontFamily: 'monospace',
              ),
            ),
          ),
        ),
        // top-right GPS display
        if (ctrl.vehiclePosition != null)
          Positioned(
            top: 16,
            right: 16,
            child: Container(
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
              decoration: BoxDecoration(
                color: Colors.black.withOpacity(0.6),
                borderRadius: BorderRadius.circular(6),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.end,
                children: [
                  const Text(
                    'GPS POSITION',
                    style: TextStyle(
                        color: Colors.white54,
                        fontSize: 10,
                        fontWeight: FontWeight.bold,
                        letterSpacing: 1.5),
                  ),
                  const SizedBox(height: 4),
                  Text(
                    'Lat: ${ctrl.vehiclePosition!.latitude.toStringAsFixed(7)}',
                    style: const TextStyle(
                        color: Colors.lightBlueAccent,
                        fontSize: 12,
                        fontFamily: 'monospace'),
                  ),
                  Text(
                    'Lon: ${ctrl.vehiclePosition!.longitude.toStringAsFixed(7)}',
                    style: const TextStyle(
                        color: Colors.lightBlueAccent,
                        fontSize: 12,
                        fontFamily: 'monospace'),
                  ),
                ],
              ),
            ),
          ),
      ],
    );
  }

  double _yawToMapAngle(double yawDeg) {
    // yaw: clockwise from north [deg] → map rotation angle [rad]
    return -yawDeg * 3.14159265 / 180.0;
  }

  void _enterOriginMode() {
    setState(() => _isSettingOrigin = true);
  }

  void _showConnectionDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (ctx) => ConnectionDialog(
        onConnect: (host, port) {
          context.read<GcsController>().connect(host, port);
        },
      ),
    );
  }
}
