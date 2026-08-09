// lib/screens/widgets/connection_dialog.dart

import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

class ConnectionDialog extends StatefulWidget {
  final Function(String host, int port) onConnect;

  const ConnectionDialog({super.key, required this.onConnect});

  @override
  State<ConnectionDialog> createState() => _ConnectionDialogState();
}

class _ConnectionDialogState extends State<ConnectionDialog> {
  final _hostCtrl = TextEditingController();
  final _portCtrl = TextEditingController(text: '9090');

  @override
  void initState() {
    super.initState();
    _loadSavedAddress();
  }

  Future<void> _loadSavedAddress() async {
    final prefs = await SharedPreferences.getInstance();
    setState(() {
      _hostCtrl.text = prefs.getString('rosbridge_host') ?? 'localhost';
      _portCtrl.text = (prefs.getInt('rosbridge_port') ?? 9090).toString();
    });
  }

  Future<void> _saveAddress(String host, int port) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('rosbridge_host', host);
    await prefs.setInt('rosbridge_port', port);
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      backgroundColor: const Color(0xFF16213E),
      title: const Row(
        children: [
          Icon(Icons.settings_ethernet, color: Colors.blueAccent, size: 20),
          SizedBox(width: 8),
          Text('rosbridge 연결',
              style: TextStyle(color: Colors.white, fontSize: 16)),
        ],
      ),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          const Text(
            'ROS2 PC에서 먼저 실행:\nros2 launch rosbridge_server rosbridge_websocket_launch.xml',
            style: TextStyle(color: Colors.white54, fontSize: 11),
          ),
          const SizedBox(height: 16),
          _field('Host / IP', _hostCtrl, '192.168.1.100'),
          const SizedBox(height: 10),
          _field('Port', _portCtrl, '9090',
              type: TextInputType.number),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text('Cancel', style: TextStyle(color: Colors.white54)),
        ),
        ElevatedButton(
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.blueAccent.withOpacity(0.3),
          ),
          onPressed: () {
            final host = _hostCtrl.text.trim();
            final port = int.tryParse(_portCtrl.text.trim()) ?? 9090;
            _saveAddress(host, port);
            widget.onConnect(host, port);
            Navigator.pop(context);
          },
          child: const Text('Connect',
              style: TextStyle(color: Colors.blueAccent)),
        ),
      ],
    );
  }

  Widget _field(String label, TextEditingController ctrl, String hint,
      {TextInputType type = TextInputType.text}) {
    return TextField(
      controller: ctrl,
      keyboardType: type,
      style: const TextStyle(color: Colors.white),
      decoration: InputDecoration(
        labelText: label,
        hintText: hint,
        labelStyle: const TextStyle(color: Colors.white54),
        hintStyle: const TextStyle(color: Colors.white24),
        enabledBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.white24)),
        focusedBorder: const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.blueAccent)),
      ),
    );
  }
}
