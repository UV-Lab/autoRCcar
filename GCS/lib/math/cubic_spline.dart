// lib/math/cubic_spline.dart
// Dart port of cubic_spline.py

import 'dart:math';

class CubicSpline1D {
  late List<double> a, b, c, d, x;
  late int nx;

  CubicSpline1D(List<double> xs, List<double> ys) {
    x = List.from(xs);
    nx = xs.length;
    a = List.from(ys);
    b = [];
    c = [];
    d = [];

    List<double> h = List.generate(nx - 1, (i) => xs[i + 1] - xs[i]);

    // A matrix (nx x nx)
    List<List<double>> A = List.generate(nx, (_) => List.filled(nx, 0.0));
    A[0][0] = 1.0;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) A[i + 1][i + 1] = 2.0 * (h[i] + h[i + 1]);
      A[i + 1][i] = h[i];
      A[i][i + 1] = h[i];
    }
    A[0][1] = 0.0;
    A[nx - 1][nx - 2] = 0.0;
    A[nx - 1][nx - 1] = 1.0;

    // B vector
    List<double> B = List.filled(nx, 0.0);
    for (int i = 0; i < nx - 2; i++) {
      B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] -
          3.0 * (a[i + 1] - a[i]) / h[i];
    }

    c = _solveLinear(A, B);

    for (int i = 0; i < nx - 1; i++) {
      double di = (c[i + 1] - c[i]) / (3.0 * h[i]);
      double bi = (a[i + 1] - a[i]) / h[i] - h[i] / 3.0 * (2.0 * c[i] + c[i + 1]);
      d.add(di);
      b.add(bi);
    }
  }

  double? calcPosition(double xVal) {
    if (xVal < x.first || xVal > x.last) return null;
    int i = _findIndex(xVal);
    double dx = xVal - x[i];
    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
  }

  double? calcFirstDerivative(double xVal) {
    if (xVal < x.first || xVal > x.last) return null;
    int i = _findIndex(xVal);
    double dx = xVal - x[i];
    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * dx * dx;
  }

  double? calcSecondDerivative(double xVal) {
    if (xVal < x.first || xVal > x.last) return null;
    int i = _findIndex(xVal);
    double dx = xVal - x[i];
    return 2.0 * c[i] + 6.0 * d[i] * dx;
  }

  int _findIndex(double xVal) {
    int lo = 0, hi = x.length - 1;
    while (lo < hi - 1) {
      int mid = (lo + hi) ~/ 2;
      if (x[mid] <= xVal) lo = mid; else hi = mid;
    }
    return lo;
  }

  /// Gaussian elimination
  List<double> _solveLinear(List<List<double>> A, List<double> B) {
    int n = B.length;
    List<List<double>> aug = List.generate(
        n, (i) => [...A[i], B[i]]);

    for (int col = 0; col < n; col++) {
      int pivot = col;
      for (int row = col + 1; row < n; row++) {
        if (aug[row][col].abs() > aug[pivot][col].abs()) pivot = row;
      }
      var tmp = aug[col]; aug[col] = aug[pivot]; aug[pivot] = tmp;

      if (aug[col][col].abs() < 1e-12) continue;
      for (int row = col + 1; row < n; row++) {
        double factor = aug[row][col] / aug[col][col];
        for (int k = col; k <= n; k++) {
          aug[row][k] -= factor * aug[col][k];
        }
      }
    }

    List<double> sol = List.filled(n, 0.0);
    for (int i = n - 1; i >= 0; i--) {
      sol[i] = aug[i][n];
      for (int j = i + 1; j < n; j++) sol[i] -= aug[i][j] * sol[j];
      sol[i] /= aug[i][i];
    }
    return sol;
  }
}

class CubicSpline2D {
  late List<double> s;
  late CubicSpline1D sx, sy;
  late List<double> ds;

  CubicSpline2D(List<double> x, List<double> y) {
    s = _calcDistParam(x, y);
    sx = CubicSpline1D(s, x);
    sy = CubicSpline1D(s, y);
  }

  List<double> _calcDistParam(List<double> x, List<double> y) {
    ds = [];
    List<double> sList = [0.0];
    for (int i = 0; i < x.length - 1; i++) {
      double dxi = x[i + 1] - x[i];
      double dyi = y[i + 1] - y[i];
      double dist = sqrt(dxi * dxi + dyi * dyi);
      ds.add(dist);
      sList.add(sList.last + dist);
    }
    return sList;
  }

  List<double>? calcPosition(double sVal) {
    double? x = sx.calcPosition(sVal);
    double? y = sy.calcPosition(sVal);
    if (x == null || y == null) return null;
    return [x, y];
  }

  double? calcHeading(double sVal) {
    double? dx = sx.calcFirstDerivative(sVal);
    double? dy = sy.calcFirstDerivative(sVal);
    if (dx == null || dy == null) return null;
    return atan2(dy, dx);
  }

  double? calcCurvature(double sVal) {
    double? dx = sx.calcFirstDerivative(sVal);
    double? ddx = sx.calcSecondDerivative(sVal);
    double? dy = sy.calcFirstDerivative(sVal);
    double? ddy = sy.calcSecondDerivative(sVal);
    if (dx == null || ddx == null || dy == null || ddy == null) return null;
    double denom = pow(dx * dx + dy * dy, 1.5).toDouble();
    if (denom < 1e-12) return 0.0;
    return (ddy * dx - ddx * dy) / denom;
  }
}

class SplineResult {
  final List<double> x, y, heading, curvature, s;
  const SplineResult({
    required this.x,
    required this.y,
    required this.heading,
    required this.curvature,
    required this.s,
  });
}

SplineResult? calculateCubicSplinePath(
    List<double> xPoints, List<double> yPoints,
    {double ds = 0.5}) {
  if (xPoints.length < 2) return null;

  try {
    final csp = CubicSpline2D(xPoints, yPoints);
    double totalS = csp.s.last;

    List<double> refX = [], refY = [], refH = [], refK = [], refS = [];
    double sVal = 0.0;
    while (sVal <= totalS) {
      final pos = csp.calcPosition(sVal);
      if (pos != null) {
        refX.add(pos[0]);
        refY.add(pos[1]);
        refH.add(csp.calcHeading(sVal) ?? 0.0);
        refK.add(csp.calcCurvature(sVal) ?? 0.0);
        refS.add(sVal);
      }
      sVal += ds;
    }

    return SplineResult(x: refX, y: refY, heading: refH, curvature: refK, s: refS);
  } catch (e) {
    return null;
  }
}
