import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:latlong2/latlong.dart';
import '../models/nav_state.dart';
import '../models/path_point.dart';
import '../models/control_command.dart';
import '../math/user_geometry.dart';
import '../math/cubic_spline.dart';
import 'rosbridge_service.dart';
import '../models/occupancy_grid.dart';
import '../models/system_status.dart';
import '../models/llm_action.dart';
import '../config/process_definitions.dart';

class GcsController extends ChangeNotifier {
  final RosbridgeService _ros;

  // ── nav state ───────────────────────────────────────────
  NavState? navState;
  double rollDeg = 0, pitchDeg = 0, yawDeg = 0;

  // ── mode ────────────────────────────────────────────────
  bool isTeleop = false;
 
  // ── Control Command ────────────────────────────────
  ControlCommand controlCommand = const ControlCommand();

  // ── PWM Command ────────────────────────────────────
  ControlCommand pwmCommand = const ControlCommand();

  // ── map origin (LLH rad) ─────────────────────────────────
  // automatically set upon first reception of nav_topic origin (ECEF)
  List<double>? _originLLH;
  bool get hasOrigin => _originLLH != null;

  // origin latitude/longitude (for display)
  double? originLatDeg;
  double? originLonDeg;

  // ── ENU trajectory (E, N) - for position graph ─────────────────────
  List<(double, double)> enuTrajectory = [];

  Function(LatLng)? onOriginSet;

  // ── vehicle position (map) ─────────────────────────────────────
  LatLng? vehiclePosition;
  List<LatLng> trajectory = [];

  // ── Waypoints ───────────────────────────────────────────
  List<WaypointLLH> waypoints = [];

  // ── spline path ─────────────────────────────────────────
  List<LatLng> splinePath = [];

  // ── spline path (ENU) - for position graph ────────────────────
  List<(double, double)> splineENU = [];

  // ── connection state ────────────────────────────────────────────
  ConnectionState get connectionState => _ros.state;

  GcsController({String host = 'localhost', int port = 9090})
      : _ros = RosbridgeService(host: host, port: port) {
    _ros.onNavState = _onNavState;
    _ros.onTeleopMode = _onTeleopMode;
    _ros.onControlCommand = _onControlCommand;
    _ros.onPwmCommand = _onPwmCommand;
    _ros.onOccupancyGrid = _onOccupancyGrid; 
    _ros.onConnectionChanged = (_) => notifyListeners();
    _ros.onProcessStatus = _onProcessStatus;
    _ros.onSystemStatus = _onSystemStatus;
  }

  // ── process status (RUN tab) ───────────────────────────────
  Map<String, String> processStatus = {};

  // ── system status (SYSTEM tab) ──────────────────────────
  SystemStatus systemStatus = const SystemStatus();

  // ── Occupancy Grid ──────────────────────────────────
  OccupancyGridMsg? occupancyGrid;
  int occupancyGridVersion = 0;

  void _onProcessStatus(Map<String, String> status) {
    processStatus = status;
    notifyListeners();
  }

  void startProcess(String name) => _ros.publishProcessCommand(name, 'start');
  void stopProcess(String name) => _ros.publishProcessCommand(name, 'stop');

  // ── ROS connection ─────────────────────────────────────────────
  Future<void> connect(String host, int port) async {
    _ros.updateAddress(host, port);
    await _ros.connect();
  }

  void disconnect() => _ros.disconnect();

  void enableOccupancyGrid() => _ros.subscribeOccupancyGrid();
  void disableOccupancyGrid() => _ros.unsubscribeOccupancyGrid();

  // ── camera stream ─────────────────────────────────────────
  Stream<Uint8List> get cameraStream => _ros.cameraStream;

  void enableCamera({String topic = '/camera/image_raw/compressed'}) =>
      _ros.subscribeCamera(topic: topic);
  void disableCamera() => _ros.unsubscribeCamera();

  // ── nav callback ─────────────────────────────────────────────
  void _onNavState(NavState msg) {
    navState = msg;

    // convert quaternion to Euler angles (w,x,y,z order)
    final euler = quat2euler([
      msg.quaternion.w,
      msg.quaternion.x,
      msg.quaternion.y,
      msg.quaternion.z,
    ]);
    rollDeg = rad2deg(euler[0]);
    pitchDeg = rad2deg(euler[1]);
    yawDeg = rad2deg(euler[2]);

    // ── auto origin setup ──────────────────────────────────
    // convert nav_topic origin (ECEF XYZ) → LLH
    // treat as valid only when ECEF is not (0,0,0)
    final ecef = [msg.origin.x, msg.origin.y, msg.origin.z];
    final ecefNorm = ecef[0] * ecef[0] + ecef[1] * ecef[1] + ecef[2] * ecef[2];

    if (_originLLH == null && ecefNorm > 1e6) {
      // convert ECEF → LLH
      final llh = xyz2llh(ecef);
      _originLLH = llh;
      originLatDeg = rad2deg(llh[0]);
      originLonDeg = rad2deg(llh[1]);
      onOriginSet?.call(LatLng(originLatDeg!, originLonDeg!));
    }

    // ── update vehicle position ────────────────────────────────
    if (_originLLH != null) {
      final enu = [msg.position.x, msg.position.y, msg.position.z];
      final llh = enu2llh(enu, _originLLH!);
      vehiclePosition = LatLng(rad2deg(llh[0]), rad2deg(llh[1]));
      trajectory.add(vehiclePosition!);
      // ── accumulate ENU trajectory ────────────────────────────────
      enuTrajectory.add((msg.position.x, msg.position.y));
    }

    notifyListeners();
  }

  void _onSystemStatus(SystemStatus status) {
    systemStatus = status;
    notifyListeners();
  }

  void restartJetson() => _ros.publishSystemCommand('restart');
  void shutdownJetson() => _ros.publishSystemCommand('shutdown');

  void _onPwmCommand(ControlCommand cmd) {
    pwmCommand = cmd;
    notifyListeners();
  }

  void _onControlCommand(ControlCommand cmd) {
    controlCommand = cmd;
    notifyListeners();
  }

  void _onTeleopMode(bool teleop) {
    isTeleop = teleop;
    notifyListeners();
  }

  void sendTeleopCommand(int cmd) => _ros.publishTeleopCommand(cmd);

  void sendSpeedReset() {
    teleopSpeed = 0.0;
    _ros.publishTeleopControlCommand(teleopSpeed, teleopSteer);
    notifyListeners();
  }

  void sendSteerReset() {
    teleopSteer = 0.0;
    _ros.publishTeleopControlCommand(teleopSpeed, deg2rad(teleopSteer));
    notifyListeners();
  }

  // ── manual origin setup (map click) ────────────────────────────
  void setOrigin(LatLng latLng, {double altitude = 0}) {
    _originLLH = [
      deg2rad(latLng.latitude),
      deg2rad(latLng.longitude),
      altitude,
    ];
    originLatDeg = latLng.latitude;
    originLonDeg = latLng.longitude;
    trajectory.clear();
    notifyListeners();
  }

  void _onOccupancyGrid(OccupancyGridMsg grid) {
    occupancyGrid = grid;
    occupancyGridVersion++;
    notifyListeners();
  }

  // ── waypoint management ─────────────────────────────────────────
  void addWaypoint(LatLng latLng) {
    double ex = 0, ey = 0;
    if (_originLLH != null) {
      final llh = [deg2rad(latLng.latitude), deg2rad(latLng.longitude), 0.0];
      final enu = llh2enu(llh, _originLLH!);
      ex = enu[0];
      ey = enu[1];
    }
    waypoints.add(WaypointLLH(latLng: latLng, x: ex, y: ey));
    _updateSpline();
    notifyListeners();
  }

  void removeWaypoint(int index) {
    if (index >= 0 && index < waypoints.length) {
      waypoints.removeAt(index);
      _updateSpline();
      notifyListeners();
    }
  }

  /// add waypoint by direct coordinates from the ENU position graph
  void addWaypointENU(double east, double north) {
    LatLng latLng;
    if (_originLLH != null) {
      final llh = enu2llh([east, north, 0.0], _originLLH!);
      latLng = LatLng(rad2deg(llh[0]), rad2deg(llh[1]));
    } else {
      latLng = const LatLng(0, 0);
    }
    waypoints.add(WaypointLLH(latLng: latLng, x: east, y: north));
    _updateSpline();
    notifyListeners();
  }

  /// remove waypoint near (east, north) within thresholdM
  void removeWaypointNearENU(double east, double north, double thresholdM) {
    if (waypoints.isEmpty) return;
    int closestIdx = -1;
    double closestDist2 = double.infinity;
    for (int i = 0; i < waypoints.length; i++) {
      final dx = waypoints[i].x - east;
      final dy = waypoints[i].y - north;
      final d2 = dx * dx + dy * dy;
      if (d2 < closestDist2) {
        closestDist2 = d2;
        closestIdx = i;
      }
    }
    if (closestIdx != -1 && closestDist2 < thresholdM * thresholdM) {
      removeWaypoint(closestIdx);
    }
  }

  void clearAll() {
    waypoints.clear();
    splinePath.clear();
    splineENU.clear();
    trajectory.clear();
    enuTrajectory.clear();
    notifyListeners();
  }

  // ── spline computation ───────────────────────────────────────────
  void _updateSpline() {
    if (waypoints.length < 2) {
      splinePath.clear();
      splineENU.clear();
      return;
    }
    final xPts = waypoints.map((w) => w.x).toList();
    final yPts = waypoints.map((w) => w.y).toList();
    final result = calculateCubicSplinePath(xPts, yPts, ds: 0.5);
    if (result == null) {
      splinePath.clear();
      splineENU.clear();
      return;
    }

    splineENU = List.generate(result.x.length, (i) => (result.x[i], result.y[i]));

    if (_originLLH != null) {
      splinePath = splineENU.map((p) {
        final llh = enu2llh([p.$1, p.$2, 0.0], _originLLH!);
        return LatLng(rad2deg(llh[0]), rad2deg(llh[1]));
      }).toList();
    } else {
      splinePath = [];
    }
  }

  // ── teleop control values ─────────────────────────────────────
  double teleopSpeed = 0.0;
  double teleopSteer = 0.0;

  void teleopSpeedUp() {
    teleopSpeed += 0.1;
    _ros.publishTeleopControlCommand(teleopSpeed, teleopSteer);
    notifyListeners();
  }

  void teleopSpeedDown() {
    teleopSpeed -= 0.1;
    _ros.publishTeleopControlCommand(teleopSpeed, teleopSteer);
    notifyListeners();
  }

  void teleopSteerLeft() {
    teleopSteer += 1.0;
    _ros.publishTeleopControlCommand(teleopSpeed, deg2rad(teleopSteer));
    notifyListeners();
  }

  void teleopSteerRight() {
    teleopSteer -= 1.0;
    _ros.publishTeleopControlCommand(teleopSpeed, deg2rad(teleopSteer));
    notifyListeners();
  }

  // ── ROS Publish ───────────────────────────────────────────

  void sendStart() => _ros.publishCommand(1);
  void sendStop() => _ros.publishCommand(0);
  void sendSetYaw(double yaw) => _ros.publishSetYaw(yaw);

  void sendGlobalPath() {
    if (waypoints.isEmpty) return;
    final pathList = waypoints.map((w) => [w.x, w.y]).toList();
    _ros.publishGlobalPath(pathList);
  }

  // ── Import / Export ───────────────────────────────────────

  void importPathFromText(String content) {
    if (_originLLH == null) return;
    waypoints.clear();
    for (final line in content.split('\n')) {
      final parts = line.trim().split(RegExp(r'\s+'));
      if (parts.length >= 2) {
        final x = double.tryParse(parts[0]);
        final y = double.tryParse(parts[1]);
        if (x != null && y != null) {
          final llh = enu2llh([x, y, 0.0], _originLLH!);
          waypoints.add(WaypointLLH(
            latLng: LatLng(rad2deg(llh[0]), rad2deg(llh[1])),
            x: x,
            y: y,
          ));
        }
      }
    }
    _updateSpline();
    notifyListeners();
  }

  String exportPathToText() {
    return waypoints.map((w) => '${w.x} ${w.y}').join('\n');
  }

  // ── LLM action execution ─────────────────────────────────────────
  // converts LlmResult.actions returned by LlmService into GCS operations in order.
  // high-risk actions (restart_jetson, shutdown_jetson) are not executed immediately;
  // they are separated into pendingConfirmations for the caller (LlmPanel) to
  // execute via confirmPendingAction() after receiving user confirmation.
  LlmExecutionResult executeLlmActions(List<LlmAction> actions) {
    final errors = <String>[];
    final pending = <LlmAction>[];

    for (final action in actions) {
      if (action.requiresConfirmation) {
        pending.add(action);
        continue;
      }
      final error = _runAction(action);
      if (error != null) errors.add(error);
    }

    return LlmExecutionResult(errors: errors, pendingConfirmations: pending);
  }

  /// execute a high-risk action that has passed the LlmPanel confirmation dialog.
  /// returns null on success, or an error message on failure.
  String? confirmPendingAction(LlmAction action) => _runAction(action);

  /// run the actual action. returns null on success, error message on failure.
  String? _runAction(LlmAction action) {
    try {
      switch (action.type) {
        // ── VEHICLE tab ──────────────────────────────────────
        case 'add_waypoint':
          final x = _asDouble(action.params['x']);
          final y = _asDouble(action.params['y']);
          if (x == null || y == null) {
            return 'add_waypoint: x/y 값이 올바르지 않습니다.';
          }
          addWaypointENU(x, y);
          return null;

        case 'set_yaw':
          final yaw = _asDouble(action.params['yaw']);
          if (yaw == null) return 'set_yaw: yaw 값이 올바르지 않습니다.';
          sendSetYaw(yaw);
          return null;

        case 'start':
          sendStart();
          return null;

        case 'stop':
          sendStop();
          return null;

        case 'clear_all':
          clearAll();
          return null;

        // ── MANUAL tab (actions valid only in teleop mode are guarded) ──
        case 'teleop_mode':
          // mode: 0=STOP, 1=TELEOP, 2=AUTO
          final mode = action.params['mode'];
          final modeInt = mode is num
              ? mode.toInt()
              : int.tryParse(mode?.toString() ?? '');
          if (modeInt == null || modeInt < 0 || modeInt > 2) {
            return 'teleop_mode: mode 값은 0(STOP)/1(TELEOP)/2(AUTO) 중 하나여야 합니다.';
          }
          sendTeleopCommand(modeInt);
          return null;

        case 'speed_up':
          if (!isTeleop) return 'speed_up: TELEOP 모드에서만 사용할 수 있습니다.';
          teleopSpeedUp();
          return null;

        case 'speed_down':
          if (!isTeleop) return 'speed_down: TELEOP 모드에서만 사용할 수 있습니다.';
          teleopSpeedDown();
          return null;

        case 'steer_left':
          if (!isTeleop) return 'steer_left: TELEOP 모드에서만 사용할 수 있습니다.';
          teleopSteerLeft();
          return null;

        case 'steer_right':
          if (!isTeleop) return 'steer_right: TELEOP 모드에서만 사용할 수 있습니다.';
          teleopSteerRight();
          return null;

        case 'speed_reset':
          if (!isTeleop) return 'speed_reset: TELEOP 모드에서만 사용할 수 있습니다.';
          sendSpeedReset();
          return null;

        case 'steer_reset':
          if (!isTeleop) return 'steer_reset: TELEOP 모드에서만 사용할 수 있습니다.';
          sendSteerReset();
          return null;

        // ── RUN tab (id is validated against the kValidProcessIds whitelist) ──
        case 'start_process':
          final id = action.params['id'] as String?;
          if (id == null || !kValidProcessIds.contains(id)) {
            return 'start_process: 알 수 없는 프로세스 id "$id".';
          }
          startProcess(id);
          return null;

        case 'stop_process':
          final id = action.params['id'] as String?;
          if (id == null || !kValidProcessIds.contains(id)) {
            return 'stop_process: 알 수 없는 프로세스 id "$id".';
          }
          stopProcess(id);
          return null;

        // ── SYSTEM tab (high-risk: pre-separated in executeLlmActions,
        //    so reaching here only happens via confirmPendingAction) ──
        case 'restart_jetson':
          restartJetson();
          return null;

        case 'shutdown_jetson':
          shutdownJetson();
          return null;

        default:
          return '알 수 없는 명령: ${action.type}';
      }
    } catch (e) {
      return '${action.type} 실행 중 오류: $e';
    }
  }

  double? _asDouble(dynamic v) {
    if (v is num) return v.toDouble();
    if (v is String) return double.tryParse(v);
    return null;
  }

  /// current ENU position for providing location context to the LLM system prompt.
  double get currentEast => navState?.position.x ?? 0.0;
  double get currentNorth => navState?.position.y ?? 0.0;

  @override
  void dispose() {
    _ros.dispose();
    super.dispose();
  }
}
