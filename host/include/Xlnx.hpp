#pragma once
#include <string>
#include <xrt/experimental/xrt_xclbin.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_uuid.h>

struct not_load_t {};
inline constexpr not_load_t not_load;

class Xlnx {
    static xrt::uuid
    load_xclbin(xrt::device& device, const std::string& xclbin) {
        std::cout << "found device: "
                  << device.get_info<xrt::info::device::name>() << '\n';
        return device.load_xclbin(xclbin);
    }

    static xrt::uuid
    register_xclbin(xrt::device& device, const std::string& xclbin) {
        std::cout << "found device: "
                  << device.get_info<xrt::info::device::name>() << '\n';
        return device.register_xclbin(xrt::xclbin(xclbin));
    }

public:
    Xlnx() = default;

    Xlnx(unsigned int id, const std::string& xclbin)
        : device_(id), uuid_(load_xclbin(device_, xclbin)) {
    }

    Xlnx(const std::string& bdf, const std::string& xclbin)
        : device_(bdf), uuid_(load_xclbin(device_, xclbin)) {
    }

    Xlnx(unsigned int id, const std::string& xclbin, not_load_t)
        : device_(id), uuid_(register_xclbin(device_, xclbin)) {
    }

    Xlnx(const std::string& bdf, const std::string& xclbin, not_load_t)
        : device_(bdf), uuid_(register_xclbin(device_, xclbin)) {
    }

    auto device() -> xrt::device& {
        return device_;
    }

    auto uuid() -> xrt::uuid& {
        return uuid_;
    }

private:
    xrt::device device_;
    xrt::uuid   uuid_;
};
