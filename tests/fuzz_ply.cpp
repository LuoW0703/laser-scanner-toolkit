#include "lsc/core/io_utils.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "lsc_fuzz_input.ply";
    {
        std::ofstream f(path, std::ios::binary);
        f.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    }

    lsc::PointCloud cloud;
    (void)lsc::loadPLY(path.string(), cloud);
    std::filesystem::remove(path);
    return 0;
}
