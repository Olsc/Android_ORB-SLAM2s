// OpenCV stubs compiled directly into VtonaxAR_Engine
// Resolves symbols from opencv_core .a that would otherwise not be found

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utils/logger.hpp>

// === Forward declare DynamicLib (normally in plugin_loader.private.hpp) ===
namespace cv { namespace plugin { namespace impl {
class DynamicLib {
public:
    DynamicLib(const String& libPath);
    ~DynamicLib();
    bool isLoaded() const { return true; }
    void* getSymbol(const char* symbolName) const;
    String getName() const;
    void disableAutomaticLibraryUnloading() {}
    static String suffix() { return String(".so"); }
};
}}}
// =====================================================================

// Device constructor/destructor
namespace cv { namespace ocl {
Device::Device() CV_NOEXCEPT {}
Device::~Device() {}
const Device& Device::getDefault() { static Device d; return d; }
bool Device::hostUnifiedMemory() const { return false; }
}}

// Filesystem utils
namespace cv { namespace utils {
void getBinLocation(String&) {}
namespace fs {
String getParent(const String&) { return String(); }
String join(const String&, const String& b) { return b; }
}
}}

// zlib stubs
extern "C" {
void* gzopen(const char*, const char*) { return nullptr; }
int gzputs(void*, const char*) { return 0; }
int gzgets(void*, char*, int) { return 0; }
int gzclose(void*) { return 0; }
int gzeof(void*) { return 0; }
int gzwrite(void*, const void*, unsigned int) { return 0; }
int gzrewind(void*) { return 0; }
}

// Logging stubs
namespace cv { namespace utils { namespace logging { namespace internal {
LogTag* getGlobalLogTag() { return nullptr; }
void writeLogMessageEx(LogLevel, const char*, const char*, int, const char*, ...) {}
void writeLogMessageEx(LogLevel, const char*, const char*, int, const char*, const char*) {}
}}}}

// DynamicLib out-of-line definitions
namespace cv { namespace plugin { namespace impl {
DynamicLib::DynamicLib(const String&) {}
DynamicLib::~DynamicLib() {}
void* DynamicLib::getSymbol(const char*) const { return nullptr; }
String DynamicLib::getName() const { return String(); }
}}}
