#include <iostream>
#include <cmath> 
#include <string>
#include <stdexcept>
#include <iosfwd>
#include <vector>
#include <armadillo>

namespace brnelib
{
  /// @brief Specification of polar coordinates (what lidar uses)
  struct Polar
  {
    /// @brief Range in m
    double r = 0.0;
    /// @brief Bearing in radians
    double theta = 0.0;
  };

  /// @brief Circle representation in the 2D plane
  struct Circle
  {
    /// @brief X coordinate of origin (m)
    double x = 0.0;
    /// @brief Y coordinate of origin (m)
    double y = 0.0;
    /// @brief Radius of circle (m)
    double r = 0.0;
  };

  /// \brief output a Polar coordinate as "Theta: x r: y"
  /// os - stream to output to
  /// p - the coordinates to print
  std::ostream & operator<<(std::ostream & os, const Polar & p);

  /// \brief output a Circle as Center: (x,y) Radius: R"
  /// os - stream to output to
  /// c - the coordinates to print
  std::ostream & operator<<(std::ostream & os, const Circle & c);

  /// @brief Obtain the euclidean distance between two Polar coordinates
  /// @param p1 polar coordinate 1
  /// @param p2 polar coordinate 2
  /// @return distance between the two points
  double polarDistance(const Polar p1, const Polar p2);

  /// @brief simple check if point is at (0,0)
  /// @param p point
  /// @return true if the point is at the origin, false otherwise
  bool atOrigin(const Polar p);

}