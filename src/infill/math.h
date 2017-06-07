#ifndef CURA_INFILL_MATH_H
#define CURA_INFILL_MATH_H

#include <cstdint>

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"


namespace cura {

/*
 * Calculates the distance between the two given points (with integer coordinates)
 */
static inline double p2p_dist(const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
    return std::sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y));
}


static inline bool is_point_in_path(const ClipperLib::IntPoint& point, const ClipperLib::Path& path)
{
    bool on_path = false;
    for (auto itr_point = path.begin(); itr_point != path.end(); ++itr_point)
    {
        if (point == *itr_point)
        {
            on_path = true;
            break;
        }
    }
    return on_path;
}


/*
 * Computes the direction of the given path. This only works with non-convex polygons.
 * Returns a positive number if it's clockwise, a negative number if it's counter-clockwise.
 */
static inline int32_t compute_path_direction(const ClipperLib::Path& path)
{
    int64_t sum = 0;
    for (auto itr_pt = path.begin(); itr_pt != path.end(); ++itr_pt)
    {
        if (itr_pt + 1 == path.end())
        {
            break;
        }

        const ClipperLib::IntPoint& p1 = *itr_pt;
        const ClipperLib::IntPoint& p2 = *(itr_pt + 1);

        sum += (p2.X - p1.X) * (p2.Y + p1.Y);
    }
    return sum > 0 ? 1 : -1;
}


/*
 * Reverse the direction of the given path and save it to the result path.
 */
static inline void reverse_path_direction(ClipperLib::Path& result_path, const ClipperLib::Path& original_path)
{
    for (auto itr_pt = original_path.crbegin(); itr_pt != original_path.crend(); ++itr_pt)
    {
        result_path << *itr_pt;
    }
}


/*
 * Sample the given line.
 */
static inline void sample_line(
    std::vector<ClipperLib::IntPoint>& sampled_point_list,
    const ClipperLib::IntPoint& p1,
    const ClipperLib::IntPoint& p2,
    int64_t sample_distance)
{
    const double line_length = p2p_dist(p1, p2);
    double unit_a = (p2.X - p1.X) / line_length;
    double unit_b = (p2.Y - p1.Y) / line_length;

    int64_t points_to_sample = std::round(line_length / sample_distance);
    const double step_distance = line_length / points_to_sample;
    const int64_t step_x = std::round(step_distance * unit_a);
    const int64_t step_y = std::round(step_distance * unit_b);

    ClipperLib::IntPoint start_point = p1;
    while (points_to_sample-- > 1)
    {
        ClipperLib::IntPoint end_point(start_point.X + step_x, start_point.Y + step_y); 

        sampled_point_list << end_point;
        start_point = end_point;
    }
}


/*
 * Create a sampled path based on the given sample distance. If the sample distance is finer than the given path,
 * It will generate a finer path out of the given path based on the given sample distance.
 */
static void create_sampled_path(ClipperLib::Path& result, const ClipperLib::Path& path, int64_t sample_distance = 300)
{
    // put in the first point
    auto itr_pt = path.begin();
    auto prev_itr_pt = itr_pt;
    result << *itr_pt;

    // handle all the edges
    for (++itr_pt; itr_pt != path.end(); ++itr_pt)
    {
        const ClipperLib::IntPoint& this_point = *itr_pt;
        const ClipperLib::IntPoint& prev_point = *prev_itr_pt;

        // sample this edge
        sample_line(result, prev_point, this_point, sample_distance);

        result << this_point;
        prev_itr_pt = itr_pt;
    }

    // sample the last edge
    ClipperLib::IntPoint start_point = path[path.size() - 1];
    ClipperLib::IntPoint last_point = path[0];
    sample_line(result, start_point, last_point, sample_distance);
}


} // namespace cura

#endif // CURA_INFILL_MATH_H
