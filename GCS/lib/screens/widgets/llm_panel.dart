// lib/screens/widgets/llm_panel.dart
//
// bottom LLM interface: natural language input → API call → JSON action parsing → GCS control.
// single input field + single response text box (no conversation history).

import 'package:flutter/material.dart';
import '../../config/process_definitions.dart';
import '../../llm/llm_service.dart';
import '../../llm/llm_settings.dart';
import '../../models/llm_action.dart';
import '../../ros2/gcs_controller.dart';
import 'llm_settings_dialog.dart';

class LlmPanel extends StatefulWidget {
  final GcsController ctrl;
  const LlmPanel({super.key, required this.ctrl});

  @override
  State<LlmPanel> createState() => _LlmPanelState();
}

enum _ResponseStatus { idle, loading, success, partial, error }

class _LlmPanelState extends State<LlmPanel> {
  final _inputCtrl = TextEditingController();
  final _llmService = LlmService();

  LlmSettings? _settings;
  _ResponseStatus _status = _ResponseStatus.idle;
  String _responseMessage =
      '자연어로 명령을 입력하세요. 예: "3미터 앞으로 이동", "수동모드로 전환", "INS/GNSS 프로세스 시작", "Jetson 재부팅"';

  @override
  void initState() {
    super.initState();
    _loadSettings();
  }

  Future<void> _loadSettings() async {
    final s = await LlmSettings.load();
    if (mounted) setState(() => _settings = s);
  }

  Future<void> _openSettings() async {
    final updated = await showDialog<LlmSettings>(
      context: context,
      builder: (_) => LlmSettingsDialog(initialSettings: _settings),
    );
    if (updated != null && mounted) {
      setState(() => _settings = updated);
    }
  }

  Future<void> _submit() async {
    final text = _inputCtrl.text.trim();
    if (text.isEmpty) return;

    final settings = _settings;
    if (settings == null || !settings.isConfigured) {
      setState(() {
        _status = _ResponseStatus.error;
        _responseMessage = 'API 키가 설정되지 않았습니다. 우측 설정 아이콘에서 먼저 설정하세요.';
      });
      return;
    }

    setState(() {
      _status = _ResponseStatus.loading;
      _responseMessage = '처리 중...';
    });

    try {
      final raw = await _llmService.sendCommand(
        settings: settings,
        userInput: text,
        currentEast: widget.ctrl.currentEast,
        currentNorth: widget.ctrl.currentNorth,
        currentYawDeg: widget.ctrl.yawDeg,
        hasOrigin: widget.ctrl.hasOrigin,
        isTeleop: widget.ctrl.isTeleop,
        validProcessIds: kValidProcessIds.toList(),
      );

      final result = LlmResult.fromRawResponse(raw);

      if (result.actions.isEmpty) {
        setState(() {
          _status = _ResponseStatus.success;
          _responseMessage = result.message.isNotEmpty
              ? result.message
              : '실행할 명령이 없습니다.';
        });
        return;
      }

      final execResult = widget.ctrl.executeLlmActions(result.actions);
      final confirmedErrors = <String>[];

      // high-risk actions (restart_jetson/shutdown_jetson) require an additional confirmation before execution
      if (execResult.hasPending) {
        for (final action in execResult.pendingConfirmations) {
          final confirmed = await _confirmHighRiskAction(action);
          if (confirmed) {
            final err = widget.ctrl.confirmPendingAction(action);
            if (err != null) confirmedErrors.add(err);
          } else {
            confirmedErrors.add('${_describeAction(action)}: 사용자가 취소했습니다.');
          }
        }
      }

      final allErrors = [...execResult.errors, ...confirmedErrors];

      if (!mounted) return;
      setState(() {
        if (allErrors.isEmpty) {
          _status = _ResponseStatus.success;
          _responseMessage = result.message.isNotEmpty
              ? result.message
              : '${result.actions.length}개 명령을 실행했습니다.';
        } else {
          _status = _ResponseStatus.partial;
          _responseMessage =
              '${result.message}\n\n일부 명령 실행 실패:\n${allErrors.join('\n')}';
        }
      });
    } catch (e) {
      if (!mounted) return;
      setState(() {
        _status = _ResponseStatus.error;
        _responseMessage = e is LlmApiException ? e.message : '오류 발생: $e';
      });
    } finally {
      _inputCtrl.clear();
    }
  }

  String _describeAction(LlmAction action) => switch (action.type) {
        'restart_jetson' => 'Jetson 재부팅',
        'shutdown_jetson' => 'Jetson 종료',
        _ => action.type,
      };

  /// confirmation dialog before executing high-risk actions (restart/shutdown).
  /// returns true if confirmed, false if cancelled.
  Future<bool> _confirmHighRiskAction(LlmAction action) async {
    final label = _describeAction(action);
    final message = switch (action.type) {
      'restart_jetson' =>
        'LLM 명령으로 차량 컴퓨터(Jetson)를 재부팅하려 합니다.\n실행 중인 모든 노드가 종료됩니다. 계속할까요?',
      'shutdown_jetson' =>
        'LLM 명령으로 차량 컴퓨터(Jetson)를 종료하려 합니다.\n다시 켜려면 차량 전원을 직접 조작해야 합니다. 계속할까요?',
      _ => 'LLM이 고위험 명령($label)을 요청했습니다. 계속할까요?',
    };

    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: const Color(0xFF16213E),
        title: Row(
          children: [
            const Icon(Icons.warning_amber, color: Colors.redAccent, size: 20),
            const SizedBox(width: 8),
            Text(label,
                style: const TextStyle(color: Colors.white, fontSize: 16)),
          ],
        ),
        content: Text(message,
            style: const TextStyle(color: Colors.white70, fontSize: 13)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: const Text('Cancel', style: TextStyle(color: Colors.white54)),
          ),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
                backgroundColor: Colors.redAccent.withOpacity(0.3)),
            onPressed: () => Navigator.pop(ctx, true),
            child: const Text('확인', style: TextStyle(color: Colors.redAccent)),
          ),
        ],
      ),
    );
    return confirmed ?? false;
  }

  Color get _statusColor => switch (_status) {
        _ResponseStatus.idle => Colors.white38,
        _ResponseStatus.loading => Colors.amberAccent,
        _ResponseStatus.success => Colors.greenAccent,
        _ResponseStatus.partial => Colors.orangeAccent,
        _ResponseStatus.error => Colors.redAccent,
      };

  IconData get _statusIcon => switch (_status) {
        _ResponseStatus.idle => Icons.smart_toy_outlined,
        _ResponseStatus.loading => Icons.hourglass_top,
        _ResponseStatus.success => Icons.check_circle,
        _ResponseStatus.partial => Icons.warning_amber,
        _ResponseStatus.error => Icons.error_outline,
      };

  @override
  Widget build(BuildContext context) {
    final configured = _settings?.isConfigured ?? false;

    return Container(
      color: const Color(0xFF16213E),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(Icons.smart_toy, size: 14, color: Colors.tealAccent.withOpacity(0.8)),
              const SizedBox(width: 6),
              const Text(
                'LLM COMMAND',
                style: TextStyle(
                  color: Colors.white54,
                  fontSize: 10,
                  fontWeight: FontWeight.bold,
                  letterSpacing: 1.5,
                ),
              ),
              const Spacer(),
              if (_settings != null)
                Text(
                  configured
                      ? _settings!.provider.label
                      : 'API 키 미설정',
                  style: TextStyle(
                    color: configured ? Colors.white38 : Colors.orangeAccent,
                    fontSize: 10,
                  ),
                ),
              const SizedBox(width: 6),
              IconButton(
                icon: const Icon(Icons.settings, size: 16, color: Colors.white54),
                tooltip: 'LLM 연동 설정',
                onPressed: _openSettings,
                padding: EdgeInsets.zero,
                constraints: const BoxConstraints(minWidth: 28, minHeight: 28),
              ),
            ],
          ),
          const SizedBox(height: 6),
          Row(
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              Expanded(
                child: TextField(
                  controller: _inputCtrl,
                  enabled: _status != _ResponseStatus.loading,
                  onSubmitted: (_) => _submit(),
                  style: const TextStyle(color: Colors.white, fontSize: 13),
                  decoration: InputDecoration(
                    hintText: '예: 3미터 앞으로 이동해줘',
                    hintStyle: const TextStyle(color: Colors.white24, fontSize: 13),
                    isDense: true,
                    contentPadding:
                        const EdgeInsets.symmetric(horizontal: 10, vertical: 10),
                    filled: true,
                    fillColor: Colors.black.withOpacity(0.25),
                    border: OutlineInputBorder(
                      borderRadius: BorderRadius.circular(6),
                      borderSide: const BorderSide(color: Colors.white12),
                    ),
                    enabledBorder: OutlineInputBorder(
                      borderRadius: BorderRadius.circular(6),
                      borderSide: const BorderSide(color: Colors.white12),
                    ),
                    focusedBorder: OutlineInputBorder(
                      borderRadius: BorderRadius.circular(6),
                      borderSide: const BorderSide(color: Colors.tealAccent),
                    ),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              SizedBox(
                height: 38,
                child: ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.tealAccent.withOpacity(0.15),
                    foregroundColor: Colors.tealAccent,
                    side: BorderSide(color: Colors.tealAccent.withOpacity(0.5)),
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(6)),
                    padding: const EdgeInsets.symmetric(horizontal: 16),
                  ),
                  onPressed: _status == _ResponseStatus.loading ? null : _submit,
                  child: _status == _ResponseStatus.loading
                      ? const SizedBox(
                          width: 16,
                          height: 16,
                          child: CircularProgressIndicator(
                              strokeWidth: 2, color: Colors.tealAccent),
                        )
                      : const Text('전송',
                          style: TextStyle(
                              fontSize: 12, fontWeight: FontWeight.bold)),
                ),
              ),
            ],
          ),
          const SizedBox(height: 6),
          Container(
            width: double.infinity,
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
            decoration: BoxDecoration(
              color: Colors.black.withOpacity(0.25),
              borderRadius: BorderRadius.circular(6),
              border: Border.all(color: _statusColor.withOpacity(0.3)),
            ),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(_statusIcon, size: 14, color: _statusColor),
                const SizedBox(width: 8),
                Expanded(
                  child: Text(
                    _responseMessage,
                    style: TextStyle(
                      color: _statusColor.withOpacity(0.95),
                      fontSize: 12,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _inputCtrl.dispose();
    super.dispose();
  }
}
