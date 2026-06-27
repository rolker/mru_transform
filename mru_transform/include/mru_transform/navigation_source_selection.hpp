#ifndef MRU_TRANSFORM_NAVIGATION_SOURCE_SELECTION_HPP
#define MRU_TRANSFORM_NAVIGATION_SOURCE_SELECTION_HPP

#include <cstddef>
#include <vector>

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

namespace mru_transform
{

/// @brief Choose which navigation source should drive the fused output, using
/// strict priority-preference.
///
/// @p sensor_stamps holds the latest sample time of each source, in descending
/// priority order. The first source whose latest sample is fresh
/// (age < @p timeout, measured against @p now) is THE source: lower-priority
/// sources are consulted only when every higher-priority source is stale. The
/// selected source updates the output only when its latest sample is newer than
/// @p last_value_time (the stamp of the value last published); otherwise there
/// is no new data to emit on this call.
///
/// @return index into @p sensor_stamps to adopt, or -1 if no source should
/// update on this call.
///
/// This replaces the earlier "newest fresh sample wins" arbitration, where a
/// faster lower-priority source (e.g. the FCU at ~10 Hz) out-voted a fresh but
/// slower primary (e.g. the SBG INS at ~4 Hz): the priority loop fell through
/// whenever the primary carried no sample newer than a global high-water mark,
/// handing the slot to the next source. Stopping at the first fresh source
/// keeps the primary in control while it is healthy and fails over only on a
/// genuine dropout past @p timeout. See unh_echoboats_project11#339.
inline int selectNavigationSource(
  const std::vector<rclcpp::Time> &sensor_stamps,
  const rclcpp::Time &last_value_time,
  const rclcpp::Time &now,
  const rclcpp::Duration &timeout)
{
  for(std::size_t i = 0; i < sensor_stamps.size(); ++i)
  {
    if(now - sensor_stamps[i] < timeout)
    {
      // Highest-priority fresh source: it owns the output. Adopt only when it
      // carries a sample we have not published yet; never fall through to a
      // lower-priority source while this one is fresh.
      return sensor_stamps[i] > last_value_time ? static_cast<int>(i) : -1;
    }
    // Stale: fall through to the next-priority source.
  }
  return -1;
}

} // namespace mru_transform

#endif
