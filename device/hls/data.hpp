#pragma once
#include <cstdint>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <hls_stream.h>

#define AXIS_ENABLE_KEEP_LAST AXIS_ENABLE_KEEP | AXIS_ENABLE_LAST

using axi_t     = ap_axiu<512, 0, 0, 0, AXIS_ENABLE_KEEP_LAST>;
using axi_bit_t = ap_axiu<1, 0, 0, 0, 0>;

struct axi_it {
    ap_uint<512> data;
    ap_uint<64>  keep;
    ap_uint<1>   last;

    axi_it() = default;

    axi_it(
        const ap_uint<512>& data,
        const ap_uint<64>&  keep,
        const ap_uint<1>    last)
        : data(data), keep(keep), last(last) {
    }

    explicit axi_it(const axi_t& axis)
        : axi_it(axis.data, axis.keep, axis.last) {
    }

    axi_it& operator=(const axi_t& axis) {
        data = axis.data;
        keep = axis.keep;
        last = axis.last;
        return *this;
    }
};

inline axi_t& operator>>=(axi_t& axis, ap_uint<16> shift) {
    axis.data >>= (shift << 3);
    axis.keep >>= shift;
    return axis;
}

inline axi_t operator>>(const axi_t& axis, ap_uint<16> shift) {
    axi_t res;
    res.data = axis.data >> (shift << 3);
    res.keep = axis.keep >> shift;
    res.last = axis.last;
    return res;
}

inline axi_it& operator>>=(axi_it& axisi, ap_uint<16> shift) {
    axisi.data >>= (shift << 3);
    axisi.keep >>= shift;
    return axisi;
}

inline axi_it operator>>(const axi_it& axisi, ap_uint<16> shift) {
    axi_it res;
    res.data = axisi.data >> (shift << 3);
    res.keep = axisi.keep >> shift;
    res.last = axisi.last;
    return res;
}

inline ap_uint<16> bswap(ap_uint<16> x) {
    return x << 8 | x >> 8;
}

inline ap_uint<32> bswap(ap_uint<32> x) {
    ap_uint<32> res;
    res(7, 0)   = x(31, 24);
    res(15, 8)  = x(23, 16);
    res(23, 16) = x(15, 8);
    res(31, 24) = x(7, 0);
    return res;
}

inline ap_uint<16> bitswap(ap_uint<16> x, ap_uint<8> l) {
    ap_uint<16> r = 0;
    for (auto i = 0; i < l; ++i) {
#pragma HLS UNROLL
        r(i, i) = x(l - 1 - i, l - 1 - i);
    }
    return r;
}

struct IpMeta {
    ap_uint<16> datalen; // for cal length of tcp
    ap_uint<64> addr;
};

struct TcpSportMeta {
    ap_uint<16> length; // just for cal expect seq
    ap_uint<32> sndseq;
    ap_uint<80> conn;
};

template <typename Integral>
constexpr bool has_single_bit(Integral x) {
    return (x & (x - 1)) == 0;
}

template <typename Integral>
constexpr uint16_t consteval_bit_width(Integral x) {
    if (x == 0) {
        return 1;
    }

    uint16_t width = 0;
    while (x != 0) {
        ++width;
        x >>= 1;
    }
    return width;
}