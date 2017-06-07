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

#include "FermatSpiralMST.h"
#include "math.h"

using namespace cura;

//
// local data structures and ata
//
struct ConnectionSorter
{
    inline bool operator() (const struct SpiralContourNodeConnection * conn1, const struct SpiralContourNodeConnection * conn2)
    {
        return (conn1->weight < conn2->weight);
    }
};

//
// local functions
//
static bool shouldIncludeCij(const ClipperLib::IntPoint& cij,
                             struct SpiralContourNode *cip1j,
                             struct SpiralContourNode *cip1k);

static void reverseArcPoints(std::vector<struct Arc>& result_list,
                             const std::vector<struct Arc>& original_list);


SpiralContourTree::SpiralContourTree()
    : m_tree_root(nullptr)
{
    this->m_contour_node_list.clear();
    this->m_all_node_connection_list.clear();
}


SpiralContourTree::~SpiralContourTree()
{
    // safely clear everything
    clear();
}

/*
 * Safely clears everything and releases all the allocated memories.
 */
void SpiralContourTree::clear()
{
    m_tree_root = nullptr;

    // clear all connection nodes
    for (uint32_t i = 0; i < m_all_node_connection_list.size(); ++i)
    {
        delete m_all_node_connection_list[i];
    }
    m_all_node_connection_list.clear();

    // clear all tree nodes
    for (uint32_t i = 0; i < m_all_contour_node_list.size(); ++i)
    {
        delete m_all_contour_node_list[i];
    }
    m_all_contour_node_list.clear();

    m_contour_node_list.clear();
}


ClipperLib::Path connectContours(struct SpiralContourNode *node)
{
    if (node == nullptr)
    {
        node = this->m_tree_root;
    }

    // if this is not a parent node, handle the children nodes first and then connect
    for (auto itr_conn = this->to_child_connection_list.begin(); itr_conn != this->to_child_connection_list.end(); ++itr_conn)
    {
        this->connectContours((*itr_conn)->child_node);
    }

    // TODO: connect this node with its parent
    
}


void SpiralContourTree::constructTree()
{
    if (m_contour_node_list.size() == 0)
    {
#ifdef DEBUG_TREE
        std::cout << "no contour node to process." << std::endl;
#endif // DEBUG_TREE
        return;
    }

#ifdef DEBUG_TREE
    std::cout << "start constructing MST" << std::endl;
#endif // DEBUG_TREE

    // generate connections between nodes
    uint32_t i;
    uint32_t j;
    uint32_t jp;
    uint32_t k;
    for (i = 0; i < m_contour_node_list.size(); ++i)
    {
        // if there is no next level, do nothing
        if (i + 1 >= m_contour_node_list.size())
        {
            break;
        }

        for (j = 0; j < m_contour_node_list[i].size(); ++j)
        {
            struct SpiralContourNode *c_ij = m_contour_node_list[i][j];

            // if there is only one contour, then there will only be one connection
            if (m_contour_node_list[i + 1].size() == 1)
            {
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][0];
                this->computeConnections(c_ij, c_ip1_jp, nullptr);
                continue;
            }

            // get c[i+1,j'] and c[i+1,k]
            for (jp = 0; jp < m_contour_node_list[i + 1].size(); ++jp)
            {
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][jp];
                for (k = 0; k < m_contour_node_list[i + 1].size(); ++k)
                {
                    if (k == jp)
                    {
                        continue;
                    }
                    struct SpiralContourNode *c_ip1_k = m_contour_node_list[i + 1][k];
                    this->computeConnections(c_ij, c_ip1_jp, c_ip1_k);
                }
            }
        }
    }

    // sort the connections based on weight
    std::vector<struct SpiralContourNodeConnection *> sorted_connection_list = m_all_node_connection_list;
    std::sort(sorted_connection_list.begin(), sorted_connection_list.end(), ConnectionSorter());

    // create a minimum spanning tree (MST)
    std::vector<struct SpiralContourNode *> already_connected_node_list;
    already_connected_node_list.reserve(this->m_all_contour_node_list.size());
    uint32_t created_connection_count = 0;

#ifdef DEBUG_TREE
    std::cout << "total node count: " << m_all_contour_node_list.size() << std::endl;
    std::cout << "total connection count: " << m_all_node_connection_list.size() << std::endl;
    std::cout << "sorted connection count: " << sorted_connection_list.size() << std::endl;
#endif // DEBUG_TREE
    for (auto itr_conn = sorted_connection_list.begin(); itr_conn != sorted_connection_list.end(); ++itr_conn)
    {
        // if all nodes have been connected, no need to continue
        if (created_connection_count == this->m_all_contour_node_list.size() - 1)
        {
            break;
        }

        struct SpiralContourNode *parent_node = (*itr_conn)->parent_node;
        struct SpiralContourNode *child_node = (*itr_conn)->child_node;

        // make sure we don't create a cyclic link
        bool found_parent = false;
        bool found_child = false;
        for (auto itr_connected_node = already_connected_node_list.begin(); itr_connected_node != already_connected_node_list.end(); ++itr_connected_node)
        {
            if ((*itr_connected_node) == parent_node)
            {
                found_parent = true;
            }
            else if ((*itr_connected_node) == child_node)
            {
                found_child = true;
            }
            if (found_parent and found_child)
            {
                break;
            }
        }
        // make sure we don't create a cyclic link
        if (found_parent and found_child and child_node->parent != nullptr)
        {
            continue;
        }

        // set this connection
        parent_node->to_child_connection_list.push_back(*itr_conn);
        this->updateNodeType(parent_node);
        child_node->to_parent_connection_list.push_back(*itr_conn);
        this->updateNodeType(child_node);
        child_node->parent = parent_node;
        ++created_connection_count;

        if (!found_parent)
        {
            already_connected_node_list.push_back(parent_node);
        }
        if (!found_child)
        {
            already_connected_node_list.push_back(child_node);
        }
    }

    // determines and sets the path directions of all contours
    this->determineContourDirections(this->m_tree_root, 0, false);

#ifndef NDEBUG
    //std::cout << ">>>>>>>>> MST:" << std::endl;
    //this->printMST(this->m_tree_root, 0);
#endif // NDEBUG
}


/*
 * This function is called after the tree is constructed. It uses depth-first search to
 * traverse through all the contours and determines the direction of all contours.
 */
int32_t SpiralContourTree::determineContourDirections(
    struct SpiralContourNode *node,
    int32_t parent_direction,
    bool is_parent_direction_set)
{
    bool direction_need_to_change = false; // whether the direction of this contour needs to be changed
    int32_t node_direction = 0; // the new direction if the direction of this contour needs to be changed
    bool is_node_direction_known = false; // whether the direction of this contour is known

    // if the parent's direction is known, we simple use the parent's direction to determine the child's direction.
    if (is_parent_direction_set)
    {
        node_direction = -parent_direction;
        is_node_direction_known = true;
        direction_need_to_change = node->direction != node_direction;
    }

    struct SpiralContourNodeConnection *connection = nullptr;
    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        connection = *itr_conn;
        int32_t child_direction = this->determineContourDirections(connection->child_node, node_direction, is_node_direction_known);

        // if the parent's direction has not been determined yet, we determine the direction of this node using the first child's direction
        if (!is_node_direction_known)
        {
            node_direction = -child_direction;
            is_node_direction_known = true;
            direction_need_to_change = node->direction != node_direction;
        }
    }

    if (!is_node_direction_known)
    {
        node_direction = node->direction;
        direction_need_to_change = false;
    }

    // reverse the paths and arcs if needed
    if (direction_need_to_change)
    {
        // reverse this path
        ClipperLib::Path reversed_path;
        reverse_path_direction(reversed_path, node->path);

        // we by now have reversed the contour path direction.
        // because when we start connecting the spiral contours, we do it from bottom up,
        // so to make it convenient, we make sure that the child -> parent connections will
        // have their nearest arc list sorted in the same direction as this node.
        for (auto itr_conn = node->to_parent_connection_list.begin(); itr_conn != node->to_parent_connection_list.end(); ++itr_conn)
        {
            connection = *itr_conn;
            std::vector<struct Arc> reversed_arc_list;
            reverseArcPoints(reversed_arc_list, connection->arc_list);
            connection->arc_list = reversed_arc_list;
        }

        // set direction
        node->direction = node_direction;
    }

    return node->direction;
}


void SpiralContourTree::addConnectionArc(struct SpiralContourNode *parent_node, struct SpiralContourNode *child_node, const struct Arc& arc)
{
    struct SpiralContourNodeConnection *connection = nullptr;

    // find an existing connection object
    for (auto itr_conn = m_all_node_connection_list.begin(); itr_conn != m_all_node_connection_list.end(); ++itr_conn)
    {
        // TODO: check and optimise those checks
        if ((*itr_conn)->parent_node != parent_node)
        {
            continue;
        }
        if ((*itr_conn)->child_node != child_node)
        {
            continue;
        }

        connection = *itr_conn;
        break;
    }

    // create a new connection object if no existing can be found
    if (connection == nullptr)
    {
        connection = new struct SpiralContourNodeConnection;
        connection->parent_node = parent_node;
        connection->child_node = child_node;
        connection->weight = 0;
        connection->arc_list = std::vector<struct Arc>();
        this->m_all_node_connection_list.push_back(connection);
    }

    connection->arc_list.push_back(arc);
    connection->weight += arc.point_count;  // weight is the number of points
}


void SpiralContourTree::addNode(struct SpiralContourNode *node, uint32_t level)
{
    // make sure that the node list for this level exists
    while (level >= m_contour_node_list.size())
    {
        m_contour_node_list.push_back(std::vector<struct SpiralContourNode *>());
    }

    // assign level number and node id and add to list
    node->level = level;
    node->index = m_contour_node_list[level].size();

    this->m_contour_node_list[level].push_back(node);
    this->m_all_contour_node_list.push_back(node);
}


void SpiralContourTree::setPolygons(const ClipperLib::Paths& paths)
{
    if (paths.size() == 0)
    {
        return;
    }

    // the first path is the outline, so we can directly feed the whole paths to the
    // handling function and it will generate spiral contour c[0,0] for the outline
    // path automatically.
    createNodes(0, paths);

    // set root node
    this->m_tree_root = this->m_contour_node_list[0][0];
}


void SpiralContourTree::updateNodeType(struct SpiralContourNode *node)
{
    uint32_t connection_count = node->to_parent_connection_list.size() + node->to_child_connection_list.size();
    uint32_t type = connection_count <= 2 ? 1 : 2;
    node->type = type;
}


/*
 * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
 */
void SpiralContourTree::createNodes(uint32_t current_level, const ClipperLib::Paths& paths)
{
    for (auto itr_path = paths.begin(); itr_path != paths.end(); ++itr_path)
    {
        // create a child node for each Polygon Path and continue
        struct SpiralContourNode *child_node = new struct SpiralContourNode;
        memset(child_node, 0, sizeof(struct SpiralContourNode));

        // create the child node
        child_node->path = *itr_path;
        child_node->sampled_path = ClipperLib::Path();
        //create_sampled_path(child_node->sampled_path, child_node->path, 50);
        child_node->direction = compute_path_direction(child_node->path);
        child_node->parent = nullptr;
        child_node->to_child_connection_list = std::vector<struct SpiralContourNodeConnection *>();
        child_node->to_parent_connection_list = std::vector<struct SpiralContourNodeConnection *>();

        this->addNode(child_node, current_level);

        // create child nodes for this node (if any)
        ClipperLib::Paths child_node_paths;

        ClipperLib::ClipperOffset clipper(1.2, 10.0);  // TODO: make this configurable
        clipper.AddPath(*itr_path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        clipper.MiterLimit = 1.2;
        clipper.Execute(child_node_paths, -7.0);

        // create spiral contour tree nodes for this child node
        createNodes(current_level + 1, child_node_paths);
    }
}


void SpiralContourTree::computeConnections(
    struct SpiralContourNode *node_cij,
    struct SpiralContourNode *node_cip1j,
    struct SpiralContourNode *node_cip1k)
{
    bool is_in_nearest_area = false;
    int64_t nearest_point_count = 0;
    int64_t nearest_area_count = 0;
    ClipperLib::IntPoint nearest_area_start_point;

    auto itr_prev_ptr = node_cij->path.begin();

    // if there is only one contour on the lower level, just take the contour as the nearest area
    if (node_cip1k == nullptr)
    {
        struct Arc arc;
        arc.p1 = node_cij->path[0];
        arc.p2 = node_cij->path[node_cij->path.size() - 1];
        arc.point_count = node_cij->path.size();
        arc.distance = 0; // TODO: compute this

        // add edge to the tree
        this->addConnectionArc(node_cij, node_cip1j, arc);

        return;
    }

    // take points on this node and compute distance towards the other
    for (auto itr_pt_cij = node_cij->path.begin(); itr_pt_cij != node_cij->path.end(); ++itr_pt_cij)
    {
        bool has_smallest_dj_prime = shouldIncludeCij(*itr_pt_cij, node_cip1j, node_cip1k);

        if (has_smallest_dj_prime)
        {
            if (!is_in_nearest_area)
            {
                // mark this point as the starting point in the current nearest area.
                is_in_nearest_area = true;
                nearest_area_start_point = *itr_pt_cij;
            }
            ++nearest_point_count;
        }
        else if (!has_smallest_dj_prime && is_in_nearest_area)
        {
            // conclude this area
            struct Arc arc;
            arc.p1 = nearest_area_start_point;
            arc.p2 = *itr_prev_ptr;
            arc.point_count = nearest_point_count;
            arc.distance = 0; // TODO: compute this

            // add edge to the tree
            this->addConnectionArc(node_cij, node_cip1j, arc);

            is_in_nearest_area = false;
            nearest_point_count = 0;

            ++nearest_area_count;
        }

        itr_prev_ptr = itr_pt_cij;
    }

    // if there is still an open nearest area, we need to conclude it.
    if (is_in_nearest_area)
    {
        // conclude this area
        struct Arc arc;
        arc.p1 = nearest_area_start_point;
        arc.p2 = node_cij->path[node_cij->path.size() - 1];
        arc.point_count = nearest_point_count;
        arc.distance = 0; // TODO: compute this

        // add edge to the tree
        this->addConnectionArc(node_cij, node_cip1j, arc);

        is_in_nearest_area = false;
        nearest_point_count = 0;
    }
}


#ifndef NDEBUG
/*
 * Prints the minimum spanning tree (MST).
 */
void SpiralContourTree::printMST(const struct SpiralContourNode *node, uint32_t level)
{
    if (node == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i <= level; ++i)
    {
        std::cout << "-";
    }

    std::cout << "[" << node->level << "," << node->index << "]"
        << " t-" << node->type
        << " d= " << (node->direction > 0 ? "L" : "R")
        << std::endl;
    std::cout << " > to child connection list size = " << node->to_child_connection_list.size() << std::endl;

    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        const struct SpiralContourNodeConnection* connection = *itr_conn;
        printMST(connection->child_node, level + 1);
    }
}
#endif // NDEBUG


static bool shouldIncludeCij(
    const ClipperLib::IntPoint& cij,
    struct SpiralContourNode *cip1j,
    struct SpiralContourNode *cip1k)
{
    // no other contours on the same level as c[i+1,j].
    if (cip1k == nullptr)
    {
        return true;
    }

    bool has_smallest_dj_prime = true;
    for (auto itr_pt_cip1j = cip1j->path.begin(); itr_pt_cip1j != cip1j->path.end(); ++itr_pt_cip1j)
    {
        const double dj_prime = p2p_dist(cij, *itr_pt_cip1j);

        for (auto itr_pt_cip1k = cip1k->path.begin(); itr_pt_cip1k != cip1k->path.end(); ++itr_pt_cip1k)
        {
            const double dk_prime = p2p_dist(cij, *itr_pt_cip1k);
            if (dj_prime >= dk_prime)
            {
                has_smallest_dj_prime = false;
                break;
            }
        }

        if (!has_smallest_dj_prime)
        {
            break;
        }
    }

    return has_smallest_dj_prime;
}


static void reverseArcPoints(std::vector<struct Arc>& result_list, const std::vector<struct Arc>& original_list)
{
    result_list.reserve(original_list.size());
    for (auto itr_arc = original_list.crbegin(); itr_arc != original_list.crend(); ++itr_arc)
    {
        struct Arc new_arc = *itr_arc;
        new_arc.p1 = (*itr_arc).p2;
        new_arc.p2 = (*itr_arc).p1;
        result_list.push_back(new_arc);
    }
}
