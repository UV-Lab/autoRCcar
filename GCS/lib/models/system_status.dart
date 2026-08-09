// lib/models/system_status.dart
// corresponds to /util/system_status (std_msgs/String, JSON)

class SystemStatus {
  final double cpuPercent;
  final double memPercent;
  final double memUsedMb;
  final double memTotalMb;
  final double diskPercent;
  final double diskUsedGb;
  final double diskTotalGb;
  final double tempCelsius;

  const SystemStatus({
    this.cpuPercent = 0,
    this.memPercent = 0,
    this.memUsedMb = 0,
    this.memTotalMb = 0,
    this.diskPercent = 0,
    this.diskUsedGb = 0,
    this.diskTotalGb = 0,
    this.tempCelsius = 0,
  });

  /// whether data has not yet been received (initial state)
  bool get isEmpty => memTotalMb == 0 && diskTotalGb == 0;

  factory SystemStatus.fromJson(Map<String, dynamic> json) {
    return SystemStatus(
      cpuPercent: (json['cpu_percent'] as num?)?.toDouble() ?? 0,
      memPercent: (json['mem_percent'] as num?)?.toDouble() ?? 0,
      memUsedMb: (json['mem_used_mb'] as num?)?.toDouble() ?? 0,
      memTotalMb: (json['mem_total_mb'] as num?)?.toDouble() ?? 0,
      diskPercent: (json['disk_percent'] as num?)?.toDouble() ?? 0,
      diskUsedGb: (json['disk_used_gb'] as num?)?.toDouble() ?? 0,
      diskTotalGb: (json['disk_total_gb'] as num?)?.toDouble() ?? 0,
      tempCelsius: (json['temp_celsius'] as num?)?.toDouble() ?? 0,
    );
  }
}
