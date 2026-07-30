#ifndef HBST_TYPES_H
#define HBST_TYPES_H

#define SRRG_HBST_HAS_OPENCV

#include "Thirdparty/srrg_hbst/types/binary_tree.hpp"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

// ObjectType is size_t, which represents the index of the keypoint in the KeyFrame
typedef srrg_hbst::BinaryMatchable256<size_t> HBSTMatchable;
typedef srrg_hbst::BinaryTree256<size_t> HBSTTree;
typedef srrg_hbst::BinaryNode256<size_t> HBSTNode;

}

#endif // HBST_TYPES_H
