/**
 * @file Geofence.cpp
 * @brief Algoritmos geométricos compactos sem STL para ESP32.
 */
#include "Geofence.h"
#include <math.h>

bool Geofence::isInside(const GpsData& gps) const {
  if (!gps.valid || fence_.count < 3) return false;
  bool c = false;
  for (uint8_t i = 0, j = fence_.count - 1; i < fence_.count; j = i++) {
    const auto& pi = fence_.points[i];
    const auto& pj = fence_.points[j];
    if (((pi.lon > gps.lon) != (pj.lon > gps.lon)) &&
        (gps.lat < (pj.lat - pi.lat) * (gps.lon - pi.lon) / (pj.lon - pi.lon + 1e-12) + pi.lat)) {
      c = !c;
    }
  }
  return c;
}

double Geofence::distanceToSegmentMeters(double lat, double lon, const GeoPoint& a, const GeoPoint& b) const {
  // Distance from the point to the closest point of segment a-b, not to its endpoints.
  // Local equirectangular frame centred on the point: accurate for fence-sized edges.
  constexpr double R = 6371000.0;
  const double mPerDegLat = R * DEG_TO_RAD;
  const double mPerDegLon = mPerDegLat * cos(lat * DEG_TO_RAD);
  const double ax = (a.lon - lon) * mPerDegLon;
  const double ay = (a.lat - lat) * mPerDegLat;
  const double bx = (b.lon - lon) * mPerDegLon;
  const double by = (b.lat - lat) * mPerDegLat;
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) {
    t = -(ax * dx + ay * dy) / len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
  }
  const double cx = ax + t * dx;
  const double cy = ay + t * dy;
  return sqrt(cx * cx + cy * cy);
}

bool Geofence::isNearBoundary(const GpsData& gps, float warningMeters) const {
  if (!gps.valid || fence_.count < 3) return false;
  double best = 1e9;
  for (uint8_t i = 0; i < fence_.count; ++i) {
    const GeoPoint& a = fence_.points[i];
    const GeoPoint& b = fence_.points[(i + 1) % fence_.count];
    const double d = distanceToSegmentMeters(gps.lat, gps.lon, a, b);
    if (d < best) best = d;
  }
  return best <= warningMeters;
}
