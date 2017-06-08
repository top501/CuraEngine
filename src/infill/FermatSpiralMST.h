/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#ifndef CURA_INFILL_FREMATSPIRAL_MST_H
#define CURA_INFILL_FREMATSPIRAL_MST_H

#include <cstdint>

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"

/*
 * This file contains the minimum spanning tree logic for the Fermat Spiral infll.
 */
namespace cura {

/*
 * Data structure for the sprial-contour tree.
 * This class represents a node in that tree.
 */
struct SpiralContourNode
{
    uint32_t level;  // i - distance from the outmost boundary
    uint32_t index;  // j - the index number of the contour with the same distance
    ClipperLib::Path path;  // the polygon path of this contour
    ClipperLib::Path sampled_path;  // the same polygon path with finer sample size
    int32_t direction;  // positive for clockwise, negative for counter-clockwise
    struct SpiralContourNode * parent;  // parent node of this node
    int32_t type;   // connection type: I or II
    std::vector<struct SpiralContourNodeConnection *> to_child_connection_list; // a list of candidate connections towards a child contour
    std::vector<struct SpiralContourNodeConnection *> to_parent_connection_list; // a list of candidate connections towards a parent contour
};


/*
 * Represents an arc from p1 to p2.
 * Note that the edge is undirected.
 */
struct Arc
{
    ClipperLib::IntPoint p1;
    ClipperLib::IntPoint p2;
    int64_t point_count;
    double distance;
};


/*
 * Represents all edges between the parent node and the child node.
 * Note that the edge is undirected. 'parent' and 'child' are merely for better understanding.
 */
struct SpiralContourNodeConnection
{
    struct SpiralContourNode *parent_node;
    struct SpiralContourNode *child_node;
    int64_t weight;
    std::vector<struct Arc> arc_list;
};


/*
 * This class represents a sprial-contour tree. It takes a polygon object and generates a
 * minimum spanning tree (MST) based on 
 */
class SpiralContourTree
{
public:
    SpiralContourTree();
    ~SpiralContourTree();

    /*
     * Safely clears everything and releases all the allocated memories.
     */
    void clear();

    /*
     * Initialises the spiral-contour tree to process the given polygon paths.
     * Note that the given polygon paths must belong to a single object.
     */
    void setPolygons(const ClipperLib::Paths& paths);

    /*
     * Starts constructing a spiral-contour tree using all the added nodes and edges.
     * This function creates a minimum spanning tree (MST).
     */
    void constructTree();

    /*
     * Connects all the contours by generating inward and outward points.
     * This must be called after the MST is constructed because it uses the MST to
     * create connections.
     */
    ClipperLib::Path connectContours(struct SpiralContourNode *node = nullptr);

private:
    /*
     * This function is called after the tree is constructed. It uses depth-first search to
     * traverse through all the contours and determines the direction of all contours.
     */
    int32_t determineContourDirections(struct SpiralContourNode *node,
                                       int32_t parent_direction,
                                       bool is_parent_direction_set);

    /*
     * Adds the given SpiralContourNode into the tree node list and assigns
     * the given level number and an auto-generated index number for it.
     * 
     * param node  - The SpiralContourNode to be added.
     * param level - The level number this node belongs to.
     */
    void addNode(struct SpiralContourNode *node, uint32_t level);

    /*
     * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
     */
    void createNodes(uint32_t current_level, const ClipperLib::Paths& paths);

    void updateNodeType(struct SpiralContourNode *node);

    void addConnectionArc(struct SpiralContourNode *parent_node,
                          struct SpiralContourNode *child_node,
                          const struct Arc& arc);

    void computeConnections(struct SpiralContourNode *node_cij,
                            struct SpiralContourNode *node_cip1j,
                            struct SpiralContourNode *node_cip1k);

#ifndef NDEBUG
    /*
     * Prints the minimum spanning tree (MST).
     */
    void printMST(const struct SpiralContourNode *node, uint32_t level);
#endif // NDEBUG

private:
    std::vector<std::vector<struct SpiralContourNode *>> m_contour_node_list;  // indexed by [level, index]
    std::vector<struct SpiralContourNode *>           m_all_contour_node_list; // a flat list of all spiral contour nodes
    std::vector<struct SpiralContourNodeConnection *> m_all_node_connection_list;  // a flat list of all connections between all spiral contour nodes
    struct SpiralContourNode *m_tree_root;
};


} // namespace cura

#endif // CURA_INFILL_FREMATSPIRAL_MST_H
