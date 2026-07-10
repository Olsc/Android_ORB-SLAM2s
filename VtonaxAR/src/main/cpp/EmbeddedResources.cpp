/**
 * 由Olsc于2025/10/21创建并开始进行修改
 */

 #include "EmbeddedResources.h"
#include <cstring>


extern "C" {
    // ORB 查找表
    extern const unsigned char _binary_ORB_LUT_bin_start[];
    extern const unsigned char _binary_ORB_LUT_bin_end[];
}

namespace EmbeddedResources {

bool Get(const char* name, const unsigned char*& data, size_t& size)
{
    data = nullptr; 
    size = 0;
    if(!name) return false;


    
    if(strcmp(name, "ORB_LUT.bin") == 0){
        data = _binary_ORB_LUT_bin_start;
        size = _binary_ORB_LUT_bin_end - _binary_ORB_LUT_bin_start;
        return true;
    }

    return false;
}

}
