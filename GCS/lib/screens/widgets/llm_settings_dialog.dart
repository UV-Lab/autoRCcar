// lib/screens/widgets/llm_settings_dialog.dart
//
// LLM integration settings dialog, styled the same as connection_dialog.dart.
// provider (Claude/OpenAI) selection + API key + model name input, saved via SharedPreferences.

import 'package:flutter/material.dart';
import '../../llm/llm_settings.dart';

class LlmSettingsDialog extends StatefulWidget {
  final LlmSettings? initialSettings;
  const LlmSettingsDialog({super.key, this.initialSettings});

  @override
  State<LlmSettingsDialog> createState() => _LlmSettingsDialogState();
}

class _LlmSettingsDialogState extends State<LlmSettingsDialog> {
  late LlmProvider _provider;
  final _claudeKeyCtrl = TextEditingController();
  final _openAiKeyCtrl = TextEditingController();
  final _claudeModelCtrl = TextEditingController();
  final _openAiModelCtrl = TextEditingController();
  bool _obscureKey = true;
  bool _loading = true;

  @override
  void initState() {
    super.initState();
    _init();
  }

  Future<void> _init() async {
    final s = widget.initialSettings ?? await LlmSettings.load();
    _provider = s.provider;
    _claudeKeyCtrl.text = s.claudeApiKey;
    _openAiKeyCtrl.text = s.openAiApiKey;
    _claudeModelCtrl.text = s.claudeModel;
    _openAiModelCtrl.text = s.openAiModel;
    if (mounted) setState(() => _loading = false);
  }

  @override
  Widget build(BuildContext context) {
    if (_loading) {
      return const AlertDialog(
        backgroundColor: Color(0xFF16213E),
        content: SizedBox(
          height: 80,
          child: Center(
            child: CircularProgressIndicator(color: Colors.tealAccent),
          ),
        ),
      );
    }

    return AlertDialog(
      backgroundColor: const Color(0xFF16213E),
      title: const Row(
        children: [
          Icon(Icons.smart_toy, color: Colors.tealAccent, size: 20),
          SizedBox(width: 8),
          Text('LLM 연동 설정',
              style: TextStyle(color: Colors.white, fontSize: 16)),
        ],
      ),
      content: SizedBox(
        width: 360,
        child: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                '자연어 명령 처리에 사용할 LLM 제공자를 선택하고 API 키를 입력하세요.\n'
                'API 키는 기기 로컬(SharedPreferences)에 저장되며 외부로 전송되지 않습니다.',
                style: TextStyle(color: Colors.white54, fontSize: 11),
              ),
              const SizedBox(height: 16),
              _providerSelector(),
              const SizedBox(height: 16),
              if (_provider == LlmProvider.claude) ..._claudeFields(),
              if (_provider == LlmProvider.openai) ..._openAiFields(),
            ],
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('Cancel', style: TextStyle(color: Colors.white54)),
        ),
        ElevatedButton(
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.tealAccent.withOpacity(0.3),
          ),
          onPressed: _save,
          child: const Text('Save', style: TextStyle(color: Colors.tealAccent)),
        ),
      ],
    );
  }

  Widget _providerSelector() {
    return Row(
      children: [
        Expanded(
          child: _providerChip(LlmProvider.claude),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: _providerChip(LlmProvider.openai),
        ),
      ],
    );
  }

  Widget _providerChip(LlmProvider p) {
    final selected = _provider == p;
    return InkWell(
      onTap: () => setState(() => _provider = p),
      borderRadius: BorderRadius.circular(6),
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 10),
        alignment: Alignment.center,
        decoration: BoxDecoration(
          color: selected
              ? Colors.tealAccent.withOpacity(0.15)
              : Colors.white.withOpacity(0.04),
          borderRadius: BorderRadius.circular(6),
          border: Border.all(
            color: selected ? Colors.tealAccent : Colors.white12,
          ),
        ),
        child: Text(
          p.label,
          style: TextStyle(
            color: selected ? Colors.tealAccent : Colors.white54,
            fontSize: 12,
            fontWeight: FontWeight.bold,
          ),
        ),
      ),
    );
  }

  List<Widget> _claudeFields() {
    return [
      _apiKeyField('Claude API Key', _claudeKeyCtrl, 'sk-ant-...'),
      const SizedBox(height: 10),
      _field('Model', _claudeModelCtrl, LlmSettings.defaultClaudeModel),
    ];
  }

  List<Widget> _openAiFields() {
    return [
      _apiKeyField('OpenAI API Key', _openAiKeyCtrl, 'sk-...'),
      const SizedBox(height: 10),
      _field('Model', _openAiModelCtrl, LlmSettings.defaultOpenAiModel),
    ];
  }

  Widget _apiKeyField(
      String label, TextEditingController ctrl, String hint) {
    return TextField(
      controller: ctrl,
      obscureText: _obscureKey,
      style: const TextStyle(color: Colors.white, fontSize: 13),
      decoration: InputDecoration(
        labelText: label,
        hintText: hint,
        labelStyle: const TextStyle(color: Colors.white54),
        hintStyle: const TextStyle(color: Colors.white24),
        enabledBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.white24)),
        focusedBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.tealAccent)),
        suffixIcon: IconButton(
          icon: Icon(
            _obscureKey ? Icons.visibility_off : Icons.visibility,
            color: Colors.white38,
            size: 18,
          ),
          onPressed: () => setState(() => _obscureKey = !_obscureKey),
        ),
      ),
    );
  }

  Widget _field(String label, TextEditingController ctrl, String hint) {
    return TextField(
      controller: ctrl,
      style: const TextStyle(color: Colors.white, fontSize: 13),
      decoration: InputDecoration(
        labelText: label,
        hintText: hint,
        labelStyle: const TextStyle(color: Colors.white54),
        hintStyle: const TextStyle(color: Colors.white24),
        enabledBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.white24)),
        focusedBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.tealAccent)),
      ),
    );
  }

  Future<void> _save() async {
    final settings = LlmSettings(
      provider: _provider,
      claudeApiKey: _claudeKeyCtrl.text.trim(),
      openAiApiKey: _openAiKeyCtrl.text.trim(),
      claudeModel: _claudeModelCtrl.text.trim().isEmpty
          ? LlmSettings.defaultClaudeModel
          : _claudeModelCtrl.text.trim(),
      openAiModel: _openAiModelCtrl.text.trim().isEmpty
          ? LlmSettings.defaultOpenAiModel
          : _openAiModelCtrl.text.trim(),
    );
    await settings.save();
    if (mounted) Navigator.pop(context, settings);
  }

  @override
  void dispose() {
    _claudeKeyCtrl.dispose();
    _openAiKeyCtrl.dispose();
    _claudeModelCtrl.dispose();
    _openAiModelCtrl.dispose();
    super.dispose();
  }
}
