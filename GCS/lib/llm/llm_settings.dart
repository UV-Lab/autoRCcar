// lib/llm/llm_settings.dart
// LLM provider selection and API key save/load (SharedPreferences)

import 'package:shared_preferences/shared_preferences.dart';

enum LlmProvider { claude, openai }

extension LlmProviderX on LlmProvider {
  String get label => switch (this) {
        LlmProvider.claude => 'Claude (Anthropic)',
        LlmProvider.openai => 'OpenAI',
      };

  String get storageKey => switch (this) {
        LlmProvider.claude => 'claude',
        LlmProvider.openai => 'openai',
      };

  static LlmProvider fromStorageKey(String? key) {
    return LlmProvider.values.firstWhere(
      (p) => p.storageKey == key,
      orElse: () => LlmProvider.claude,
    );
  }
}

class LlmSettings {
  static const _kProviderKey = 'llm_provider';
  static const _kClaudeApiKey = 'llm_api_key_claude';
  static const _kOpenAiApiKey = 'llm_api_key_openai';
  static const _kClaudeModel = 'llm_model_claude';
  static const _kOpenAiModel = 'llm_model_openai';

  static const String defaultClaudeModel = 'claude-sonnet-4-6';
  static const String defaultOpenAiModel = 'gpt-4o-mini';

  final LlmProvider provider;
  final String claudeApiKey;
  final String openAiApiKey;
  final String claudeModel;
  final String openAiModel;

  const LlmSettings({
    required this.provider,
    required this.claudeApiKey,
    required this.openAiApiKey,
    required this.claudeModel,
    required this.openAiModel,
  });

  String get activeApiKey =>
      provider == LlmProvider.claude ? claudeApiKey : openAiApiKey;

  String get activeModel =>
      provider == LlmProvider.claude ? claudeModel : openAiModel;

  bool get isConfigured => activeApiKey.trim().isNotEmpty;

  static Future<LlmSettings> load() async {
    final prefs = await SharedPreferences.getInstance();
    return LlmSettings(
      provider: LlmProviderX.fromStorageKey(prefs.getString(_kProviderKey)),
      claudeApiKey: prefs.getString(_kClaudeApiKey) ?? '',
      openAiApiKey: prefs.getString(_kOpenAiApiKey) ?? '',
      claudeModel: prefs.getString(_kClaudeModel) ?? defaultClaudeModel,
      openAiModel: prefs.getString(_kOpenAiModel) ?? defaultOpenAiModel,
    );
  }

  Future<void> save() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString(_kProviderKey, provider.storageKey);
    await prefs.setString(_kClaudeApiKey, claudeApiKey);
    await prefs.setString(_kOpenAiApiKey, openAiApiKey);
    await prefs.setString(_kClaudeModel, claudeModel);
    await prefs.setString(_kOpenAiModel, openAiModel);
  }

  LlmSettings copyWith({
    LlmProvider? provider,
    String? claudeApiKey,
    String? openAiApiKey,
    String? claudeModel,
    String? openAiModel,
  }) {
    return LlmSettings(
      provider: provider ?? this.provider,
      claudeApiKey: claudeApiKey ?? this.claudeApiKey,
      openAiApiKey: openAiApiKey ?? this.openAiApiKey,
      claudeModel: claudeModel ?? this.claudeModel,
      openAiModel: openAiModel ?? this.openAiModel,
    );
  }
}
