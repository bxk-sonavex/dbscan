#include "dbscan.hpp"
#include <nanoflann/nanoflann.hpp>
#include <iterator>

template<typename Point>
struct Adaptor
{
  const std::span<const Point>& points;
  Adaptor(const std::span<const Point>& points) : points(points) { }

  /// CRTP helper method
  //inline const Derived& derived() const { return obj; }

  // Must return the number of data points
  inline std::size_t kdtree_get_point_count() const { return points.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    return points[idx].get(dim);
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

  auto const* elem_ptr(const std::size_t idx) const {
    return &points[idx].x;
  }
};

template<typename Point>
auto dbscan_(const std::span<const Point>& data, float eps, int min_pts)
{
  static_assert(std::is_same_v<Point, Point2f> or std::is_same_v<Point, Point3f>,
                "This only supports either 2D or 3D points");

  constexpr int dim = [] {
    if constexpr (std::is_same_v<Point, Point2f>) {
      return 2;
    }
    else if constexpr (std::is_same_v<Point, Point3f>) {
      return 3;
    }
  }();

  const auto adaptor = Adaptor(data);

  eps *= eps;
  using namespace nanoflann;
  using KDTree_t = KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, decltype(adaptor)>, decltype(adaptor), dim>;

  auto index = KDTree_t(dim, adaptor, KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  const auto n_points = adaptor.kdtree_get_point_count();
  auto visited = std::vector<bool>(n_points);
  auto clusters = std::vector<std::vector<size_t>>();
  auto matches = std::vector<std::pair<size_t, float>>();
  auto sub_matches = std::vector<std::pair<size_t, float>>();

  for (size_t i = 0; i < n_points; i++) {
    if (visited[i]) {
      continue;
    }

    index.radiusSearch(adaptor.elem_ptr(i), eps, matches, SearchParams(32, 0.F, false));
    if (matches.size() < static_cast<size_t>(min_pts)) {
      continue;
    }
    visited[i] = true;

    auto cluster = std::vector({ i });

    while (!matches.empty()) {
      auto nb_idx = matches.back().first;
      matches.pop_back();
      if (visited[nb_idx]) {
        continue;
      }
      visited[nb_idx] = true;

      index.radiusSearch(adaptor.elem_ptr(nb_idx), eps, sub_matches, SearchParams(32, 0.F, false));

      if (sub_matches.size() >= static_cast<size_t>(min_pts)) {
        std::copy(sub_matches.begin(), sub_matches.end(), std::back_inserter(matches));
      }
      cluster.push_back(nb_idx);
    }
    clusters.emplace_back(std::move(cluster));
  }

  return clusters;
}

auto dbscan(const std::span<const Point2f>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>
{
  return dbscan_(data, eps, min_pts);
}

auto dbscan(const std::span<const Point3f>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>
{
  return dbscan_(data, eps, min_pts);
}
