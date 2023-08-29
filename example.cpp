#include "dbscan.hpp"
#include <iostream>
#include <string>
#include <system_error>
#include <vector>
#include <utility>
#include <fstream>
#include <charconv>
#include <tuple>
#include <random>
#include <cstdint>
#include <optional>
#include <filesystem>

struct Color
{
  uint32_t r { 0 }, g { 0 }, b { 0 };
};

auto check_from_chars_error(std::errc err, std::string_view line, int line_counter)
{
  if (err == std::errc())
    return;

  if (err == std::errc::invalid_argument) {
    std::cerr << "Error: Invalid value \"" << line
              << "\" at line " << line_counter << "\n";
    std::exit(1);
  }

  if (err == std::errc::result_out_of_range) {
    std::cerr << "Error: Value \"" << line << "\"out of range at line "
              << line_counter << "\n";
    std::exit(1);
  }
}

auto push_values(std::vector<float>& store, std::string_view line, int line_counter)
{
  auto ptr = line.data();
  auto ec = std::errc();
  auto n_pushed = 0;

  while (ptr < line.data() + line.size()) {
    float value;
    const auto [p, ec] = std::from_chars(ptr, line.data() + line.size(), value);
    ptr = p + 1;
    check_from_chars_error(ec, line, line_counter);
    n_pushed++;
    store.push_back(value);
  }

  return n_pushed;
}

auto read_values(const std::filesystem::path& filepath)
{
  std::ifstream file(filepath);

  if (not file.good()) {
    std::perror(filepath.string().data());
    std::exit(2);
  }

  auto count = 0;

  auto points = std::vector<float>();
  auto dim = 0;

  while (not file.eof()) {
    count++;
    auto line = std::string();
    std::getline(file, line);

    if (not line.empty()) {
      auto n_pushed = push_values(points, line, count);

      if (count != 1) {
        if (n_pushed != dim) {
          std::cerr << "Inconsistent number of dimensions at line '" << count << "'\n";
          std::exit(1);
        }
      }
      dim = n_pushed;
    }
  }

  return std::make_tuple(points, dim);
}

template<typename T>
T to_num(std::string_view str)
{
  T value {};
  if (std::from_chars(str.data(), str.data() + str.size(), value).ec == std::errc()) {
    return value;
  }
  else {
    std::cerr << "Error converting value '" << str << "'\n";
    std::exit(1);
  }
}

// noise will be labelled as 0
auto label(const std::vector<std::vector<size_t>>& clusters, size_t pointCount)
{
  auto point_labels = std::vector<size_t>(pointCount);

  for (size_t i = 0; i < clusters.size(); i++) {
    for (const auto& p : clusters[i]) {
      point_labels[p] = i + 1;
    }
  }

  return point_labels;
}

auto dbscan2d(const std::span<const float>& data, float eps, int min_pts)
{
  auto points = std::vector<Point2f>(data.size() / 2);

  std::memcpy(points.data(), data.data(), sizeof(float) * data.size());

  auto clusters = dbscan(points, eps, min_pts);
  auto flat = label(clusters, points.size());

  for (size_t i = 0; i < points.size(); i++) {
    std::cout << points[i].x << ',' << points[i].y << ',' << flat[i] << '\n';
  }
}

auto dbscan3d(const std::span<const float>& data, float eps, int min_pts)
{
  auto points = std::vector<Point3f>(data.size() / 3);
  std::memcpy(points.data(), data.data(), sizeof(float) * data.size());

  const auto clusters = dbscan(points, eps, min_pts);
  const auto labels = label(clusters, points.size());

  const auto color_map = [&clusters] {
    std::random_device rd;
    auto distribution = std::uniform_int_distribution(0U, 255U);
    auto generator = std::mt19937(rd());

    auto color_map = std::vector<Color>(clusters.size() + 1);
    color_map.front() = { 0, 0, 0 }; // color of noise/outlier(s)
    for (size_t i = 1; i < color_map.size(); i++) {
      color_map[i] = { distribution(generator), distribution(generator), distribution(generator) };
    }
    return color_map;
  }();

  for (size_t i = 0; i < points.size(); i++) {
    const auto& color = color_map.at(labels[i]);
    std::cout << points[i].x << ',' << points[i].y << ',' << points[i].z
              << "," << labels[i]
              << ',' << color.r << ',' << color.g << ',' << color.b << '\n';
  }
}

int main(int argc, char** argv)
{
  if (argc != 4) {
    std::cerr << "usage: example <tsv file> <epsilon> <min points>\n";
    return EXIT_FAILURE;
  }

  const std::vector<std::string_view> args { argv, argv + argc };
  auto [values, dim] = read_values(args[1]);
  const auto epsilon = to_num<float>(args[2]);
  const auto min_pts = to_num<int>(args[3]);

  if (dim == 2) {
    dbscan2d(values, epsilon, min_pts);
  }
  else if (dim == 3) {
    dbscan3d(values, epsilon, min_pts);
  }

  return EXIT_SUCCESS;
}