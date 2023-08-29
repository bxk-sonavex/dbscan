#pragma once

#include <span>
#include <vector>

struct Point2f
{
  float x { 0.F };
  float y { 0.F };

  inline constexpr auto get(std::size_t dim) const {
    return dim == 0 ? x : y;
  }
};

struct Point3f
{
  float x { 0.F };
  float y { 0.F };
  float z { 0.F };

  inline constexpr auto get(std::size_t dim) const {
    switch (dim) {
      case 0: return x;
      case 1: return y;
      default: return z;
    }
  }
};

auto dbscan(const std::span<const Point2f>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
auto dbscan(const std::span<const Point3f>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
