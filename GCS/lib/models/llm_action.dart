// lib/models/llm_action.dart
// model representing JSON actions returned by the LLM.
// LLM responses must always be JSON strings in the form:
// {"actions": [ {action: ..., ...params}, ... ], "message": "..."}

import 'dart:convert';

class LlmAction {
  final String type; // add_waypoint | set_yaw | start | stop | clear_all |
                      // teleop_mode | speed_up | speed_down | steer_left |
                      // steer_right | speed_reset | steer_reset |
                      // start_process | stop_process |
                      // restart_jetson | shutdown_jetson
  final Map<String, dynamic> params;

  const LlmAction({required this.type, this.params = const {}});

  factory LlmAction.fromJson(Map<String, dynamic> json) {
    final type = json['action'] as String? ?? '';
    final params = Map<String, dynamic>.from(json)..remove('action');
    return LlmAction(type: type, params: params);
  }

  /// high-risk actions that can interrupt a running mission, such as vehicle computer restart/shutdown.
  /// these actions require an additional confirmation dialog in the LLM panel UI before execution.
  static const Set<String> highRiskTypes = {
    'restart_jetson',
    'shutdown_jetson',
  };

  bool get requiresConfirmation => highRiskTypes.contains(type);
}

class LlmResult {
  final List<LlmAction> actions;
  final String message; // natural language description/response shown to the user

  const LlmResult({required this.actions, required this.message});

  /// parse the raw LLM response.
  /// on parse failure, actions is an empty list and message shows the raw text as-is.
  factory LlmResult.fromRawResponse(String raw) {
    String cleaned = raw.trim();
    if (cleaned.startsWith('```')) {
      cleaned = cleaned.replaceFirst(RegExp(r'^```[a-zA-Z]*\s*'), '');
      cleaned = cleaned.replaceFirst(RegExp(r'```\s*$'), '');
      cleaned = cleaned.trim();
    }

    try {
      final decoded = jsonDecode(cleaned);
      if (decoded is! Map) {
        return LlmResult(actions: const [], message: raw);
      }
      final map = Map<String, dynamic>.from(decoded);

      final List<LlmAction> actions = [];
      final actionsJson = map['actions'];
      if (actionsJson is List) {
        for (final a in actionsJson) {
          if (a is Map) {
            actions.add(LlmAction.fromJson(Map<String, dynamic>.from(a)));
          }
        }
      }

      final message = map['message'] as String? ?? '';
      return LlmResult(actions: actions, message: message);
    } catch (_) {
      // if not JSON, display the raw text as the message (no actions)
      return LlmResult(actions: const [], message: raw);
    }
  }
}

/// result of GcsController.executeLlmActions().
/// - errors: list of reasons for actions that failed to execute immediately
/// - pendingConfirmations: list of high-risk actions (LlmAction.requiresConfirmation)
///   that have not yet been executed. the caller UI must receive user confirmation
///   and then execute them via GcsController.confirmPendingAction().
class LlmExecutionResult {
  final List<String> errors;
  final List<LlmAction> pendingConfirmations;

  const LlmExecutionResult({
    required this.errors,
    required this.pendingConfirmations,
  });

  bool get hasPending => pendingConfirmations.isNotEmpty;
  bool get hasErrors => errors.isNotEmpty;
}
