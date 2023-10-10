#include "brnelib/brne.hpp"

namespace brnelib
{
    std::ostream & operator<<(std::ostream & os, const Polar & p){
        os << "Angle: " << p.theta << ",\tR: " << p.r;
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Circle & c){
      os << "Center: (" << c.x << ", " << c.y << ") Rad: " << c.r;
      return os;
    }

    double polarDistance(const Polar p1, const Polar p2){
      const auto x1 = p1.r*cos(p1.theta);
      const auto y1 = p1.r*sin(p1.theta);
      const auto x2 = p2.r*cos(p2.theta);
      const auto y2 = p2.r*sin(p2.theta);
      const auto dx = x2 - x1;
      const auto dy = y2 - y1;
      return sqrt(dx * dx + dy * dy);
    }

    bool atOrigin(const Polar p){
      return ((p.r == 0) && (p.theta == 0));
    }
}