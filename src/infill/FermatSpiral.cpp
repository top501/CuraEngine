/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <limits>
#include <algorithm>
#include <vector>

#ifndef NDEBUG
# include <iostream>
#endif // NDEBUG

#include "math.h"
#include "FermatSpiral.h"
#include "FermatSpiralMST.h"

using namespace cura;


/*
 * Starting from the given point, travel along the given path for the given distance,
 * and return the point.
 */
static ClipperLib::IntPoint get_point_on_path_after_distance(const ClipperLib::Path& path,
                                                             uint64_t start_point_index,
                                                             int64_t distance);


void FermatSpiralInfillGenerator::generateInfill(
    const Polygons& in_outline,
    Polygons& result_lines,
    const SliceMeshStorage* mesh)
{
    // construct the MST
    SpiralContourTree tree;
    tree.setPolygons(in_outline.getPaths());
    tree.constructTree();

    // TODO: connect the contours
}



ClipperLib::IntPoint get_point_on_path_after_distance(
    const ClipperLib::Path& path,
    uint64_t start_point_index,
    int64_t distance)
{
    ClipperLib::IntPoint result_point;
    
    assert (start_point_index < path.size());
    if (start_point_index >= path.size())
    {
        std::cerr << "start_point_index > path.size" << std::endl;
        start_point_index = start_point_index % path.size();
    }

    bool got_point = false;
    double accumulated_distance = 0;
    while (got_point)
    {
        for (auto itr_pt = path.begin() + start_point_index; itr_pt != path.end(); ++itr_pt)
        {
            if (itr_pt + 1 == path.end())
            {
                break;
            }
            const ClipperLib::IntPoint& start_point = *itr_pt;
            const ClipperLib::IntPoint& end_point = *(itr_pt + 1);

            const double line_length = p2p_dist(start_point, end_point);
            accumulated_distance += line_length;
            if (std::round(accumulated_distance) > distance)
            {
                // chop off
                const double distance_to_go = accumulated_distance - distance;

                const double unit_a = (end_point.X - start_point.X) / line_length;
                const double unit_b = (end_point.Y - start_point.Y) / line_length;
                const int64_t delta_x = std::round(distance_to_go * unit_a);
                const int64_t delta_y = std::round(distance_to_go * unit_b);

                result_point.X = delta_x;
                result_point.Y = delta_y;
                got_point = true;
                break;
            }
        }

        if (got_point)
        {
            break;
        }
    }
}
