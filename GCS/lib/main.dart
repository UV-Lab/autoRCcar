import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'ros2/gcs_controller.dart';
import 'map/mbtiles_service.dart';
import 'map/connectivity_service.dart';
import 'screens/main_screen.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  final mbtiles = MbtilesService();
  await mbtiles.init();

  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => GcsController(host: 'localhost', port: 9090)),
        Provider<MbtilesService>.value(value: mbtiles),
        ChangeNotifierProvider(create: (_) => ConnectivityService()),
      ],
      child: const GcsApp(),
    ),
  );
}

class GcsApp extends StatelessWidget {
  const GcsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AutoRCCar GCS',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: Colors.blueAccent,
          brightness: Brightness.dark,
        ),
        useMaterial3: true,
        scaffoldBackgroundColor: const Color(0xFF1A1A2E),
      ),
      home: const MainScreen(),
    );
  }
}