#include <cassert>
#include <cmath>

#include "arduino_compat/Arduino.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif

#include "../../coleira/Geofence.cpp"

// Square pasture, ~1 km per side, near -15 lat. Longitude step chosen so edges are ~1000 m.
static Polygon makeFence() {
  const double dLon = 1000.0 / (111194.9 * std::cos(15.0 * 3.14159265358979 / 180.0));
  const double dLat = 1000.0 / 111194.9;
  Polygon p;
  p.count = 4;
  p.points[0] = {-15.0, -47.0};
  p.points[1] = {-15.0, -47.0 + dLon};
  p.points[2] = {-15.0 + dLat, -47.0 + dLon};
  p.points[3] = {-15.0 + dLat, -47.0};
  return p;
}

static GpsData at(double lat, double lon) {
  GpsData g;
  g.valid = true;
  g.lat = lat;
  g.lon = lon;
  return g;
}

int main() {
  Geofence fence;
  fence.setFence(makeFence());
  const double dLon = fence.fence().points[1].lon - fence.fence().points[0].lon;
  const double dLat = fence.fence().points[3].lat - fence.fence().points[0].lat;

  // 10 m inside the middle of the southern edge: ~500 m from either corner, 10 m from the edge.
  const GpsData midSouth = at(-15.0 + 10.0 / 111194.9, -47.0 + dLon / 2);
  assert(fence.isInside(midSouth));
  assert(fence.isNearBoundary(midSouth, 30.0f));
  assert(!fence.isNearBoundary(midSouth, 5.0f));

  // 10 m inside the middle of the western edge.
  const GpsData midWest = at(-15.0 + dLat / 2, -47.0 + dLon * 10.0 / 1000.0);
  assert(fence.isInside(midWest));
  assert(fence.isNearBoundary(midWest, 30.0f));

  // Centre of the pasture is ~500 m from every edge.
  const GpsData centre = at(-15.0 + dLat / 2, -47.0 + dLon / 2);
  assert(fence.isInside(centre));
  assert(!fence.isNearBoundary(centre, 30.0f));

  // Near a corner still works, and so does a point past the end of an edge.
  assert(fence.isNearBoundary(at(-15.0 + 5.0 / 111194.9, -47.0 + dLon * 0.005), 30.0f));
  assert(!fence.isNearBoundary(at(-15.0 - 200.0 / 111194.9, -47.0 - dLon * 0.2), 30.0f));

  return 0;
}
