#ifndef FOLLOW_THE_GAP_V0__CORNER_HPP_
#define FOLLOW_THE_GAP_V0__CORNER_HPP_

#include "follow_the_gap_v0/gap.hpp"

namespace FollowTheGap
{

/**
 * @brief Corner model used by fallback logic.
 *
 * A corner is represented as a specialized gap plus a semantic label telling
 * whether it should be interpreted as a left or right corner.
 */
class Corner : public Gap
{
public:
  enum class CornerTypes
  {
    kLeft,
    kRight
  };

  Corner() = delete;

  Corner(const Obstacle & o1, const Obstacle & o2, CornerTypes corner_type)
  : Gap(o1, o2), corner_type_(corner_type)
  {
  }

  CornerTypes CornerType() const
  {
    return corner_type_;
  }

private:
  CornerTypes corner_type_;
};

}  // namespace FollowTheGap

#endif  // FOLLOW_THE_GAP_V0__CORNER_HPP_
