/**
 * 由Olsc于2025/10/21创建并开始进行修改
 */

 // 轻量级接口：从本地 SO 中访问内嵌资源文件
#pragma once

#include <cstddef>

namespace EmbeddedResources {

// 返回是否存在指定资源；如存在，设置 data/size 为只读内存视图
// 资源名称约定：
// - "ORBvoc.txt.arm.bin"
bool Get(const char* name, const unsigned char*& data, size_t& size);

}

 