/*
 * 按对象 id 排序的关联容器。
 *
 * 原代码中的 std::set<KeyFrame *> / std::map<KeyFrame *,...> 按指针地址排序，
 * 而 Android 的 scudo 分配器会随机化地址，于是每次运行时这些容器的遍历顺序都不同：
 * 浮点求和次序、g2o 边的插入次序、描述子中值的平局、共视权重相同时的排序都随之改变，
 * 导致同一段输入在两次运行中得到不同的关键帧数和轨迹。
 *
 * mnId 由进程内单调递增的静态计数器分配（KeyFrame.cc / MapPoint.cc，从不重置），
 * 对加载地图的对象同样如此，因此在进程生命周期内唯一，可作为全序键。
 * 比较器把 nullptr 视为最小元素，兼容原先把空指针放进集合的用法。
 */
#ifndef IDORDER_H
#define IDORDER_H

#include <set>
#include <map>
#include <cstddef>

namespace ORB_SLAM2
{

class KeyFrame;
class MapPoint;

template<class T>
struct IdLess
{
    bool operator()(const T* a, const T* b) const
    {
        if(!a) return b != nullptr;
        if(!b) return false;
        return a->mnId < b->mnId;
    }
};

typedef std::set<KeyFrame*, IdLess<KeyFrame> > KeyFrameSet;
typedef std::set<MapPoint*, IdLess<MapPoint> > MapPointSet;

template<class V> using KeyFrameMap = std::map<KeyFrame*, V, IdLess<KeyFrame> >;
template<class V> using MapPointMap = std::map<MapPoint*, V, IdLess<MapPoint> >;

} // namespace ORB_SLAM2

#endif // IDORDER_H