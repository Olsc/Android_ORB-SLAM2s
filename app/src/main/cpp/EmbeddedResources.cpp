/**
 * 由Olsc于2025/10/21创建并开始进行修改
 */

 #include "EmbeddedResources.h"
#include <cstring>

// 这些符号由 objcopy 在构建时生成
// objcopy 创建 _binary_<filename>_start 和 _binary_<filename>_end 符号
// 注意：文件路径中的 '.' 和 '/' 会被替换为 '_'


// ORBvoc.txt.arm.bin
extern "C" {
    extern const unsigned char _binary_ORBvoc_txt_arm_bin_start[];
    extern const unsigned char _binary_ORBvoc_txt_arm_bin_end[];
}


namespace EmbeddedResources {

bool Get(const char* name, const unsigned char*& data, size_t& size)
{
    data = nullptr; 
    size = 0;
    if(!name) return false;

    if(strcmp(name, "ORBvoc.txt.arm.bin") == 0){
        data = _binary_ORBvoc_txt_arm_bin_start;
        size = _binary_ORBvoc_txt_arm_bin_end - _binary_ORBvoc_txt_arm_bin_start;
        return true;
    }

    return false;
}

}
