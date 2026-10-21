#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <libdeflate.h>
#include "data.hpp"

extern "C" void inflate(
    hls::stream<axi_bit_t>& notify_in,
    hls::stream<axi_t>&     data_in,
    hls::stream<axi_bit_t>& notify_out,
    hls::stream<axi_t>&     data_out);

using Bytes = std::vector<unsigned char>;

struct BitWriter {
    Bytes    bytes;
    unsigned bit_count = 0;

    void put(unsigned value, unsigned width) {
        for (unsigned bit = 0; bit < width; ++bit) {
            if (bit_count % 8 == 0)
                bytes.push_back(0);
            bytes.back() |= ((value >> bit) & 1) << (bit_count % 8);
            ++bit_count;
        }
    }

    void symbol(unsigned value, const std::vector<unsigned>& lengths) {
        unsigned counts[16] = {};
        unsigned codes[16]  = {};
        for (unsigned length : lengths) {
            if (length)
                ++counts[length];
        }
        for (unsigned length = 1; length < 16; ++length) {
            codes[length] = (codes[length - 1] + counts[length - 1]) << 1;
        }
        for (unsigned index = 0; index < value; ++index) {
            if (lengths[index])
                ++codes[lengths[index]];
        }
        for (unsigned bit = lengths[value]; bit > 0; --bit) {
            put((codes[lengths[value]] >> (bit - 1)) & 1, 1);
        }
    }
};

static std::vector<unsigned> fixed_lengths() {
    std::vector<unsigned> lengths(288, 8);
    std::fill(lengths.begin() + 144, lengths.begin() + 256, 9);
    std::fill(lengths.begin() + 256, lengths.begin() + 280, 7);
    return lengths;
}

static void fixed_block(BitWriter& writer, const Bytes& plain, bool last) {
    writer.put(last, 1);
    writer.put(1, 2);
    const auto lengths = fixed_lengths();
    for (unsigned byte : plain)
        writer.symbol(byte, lengths);
    writer.symbol(256, lengths);
}

static void stored_block(BitWriter& writer, const Bytes& plain, bool last) {
    writer.put(last, 1);
    writer.put(0, 2);
    while (writer.bit_count % 8)
        writer.put(0, 1);
    writer.put(plain.size(), 16);
    writer.put((~plain.size()) & 65535, 16);
    for (unsigned byte : plain)
        writer.put(byte, 8);
}

static void
dynamic_header(BitWriter& writer, const std::vector<unsigned>& lengths) {
    const unsigned order[19] =
        {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    writer.put(5, 3);
    writer.put(0, 5);
    writer.put(0, 5);
    writer.put(15, 4);
    for (unsigned symbol : order)
        writer.put(lengths[symbol], 3);
}

static Bytes dynamic_literals(const Bytes& plain, bool long_codes = false) {
    BitWriter             writer;
    std::vector<unsigned> code_lengths(19, 4);
    std::fill(code_lengths.begin() + 16, code_lengths.end(), 0);
    dynamic_header(writer, code_lengths);
    std::vector<unsigned> lengths(257, 9);
    lengths[256] = 1;
    if (long_codes) {
        std::fill(lengths.begin(), lengths.end(), 0);
        for (unsigned symbol = 0; symbol < 14; ++symbol)
            lengths[symbol] = symbol + 1;
        lengths[14] = lengths[256] = 15;
    }
    for (unsigned length : lengths)
        writer.symbol(length, code_lengths);
    writer.symbol(0, code_lengths);
    for (unsigned byte : plain)
        writer.symbol(byte, lengths);
    writer.symbol(256, lengths);
    return writer.bytes;
}

static Bytes repeat_zero_lengths(const Bytes& plain) {
    BitWriter             writer;
    std::vector<unsigned> code_lengths(19, 0);
    code_lengths[0] = code_lengths[1] = code_lengths[16] = 2;
    code_lengths[17] = code_lengths[18] = 3;
    dynamic_header(writer, code_lengths);
    writer.symbol(0, code_lengths);
    writer.symbol(18, code_lengths);
    writer.put(64 - 11, 7);
    writer.symbol(1, code_lengths);
    writer.symbol(17, code_lengths);
    writer.put(0, 3);
    writer.symbol(16, code_lengths);
    writer.put(0, 2);
    writer.symbol(18, code_lengths);
    writer.put(138 - 11, 7);
    writer.symbol(18, code_lengths);
    writer.put(46 - 11, 7);
    writer.symbol(1, code_lengths);
    writer.symbol(0, code_lengths);
    std::vector<unsigned> lengths(257, 0);
    lengths[65] = lengths[256] = 1;
    for (unsigned byte : plain)
        writer.symbol(byte, lengths);
    writer.symbol(256, lengths);
    return writer.bytes;
}

static Bytes exact_match(unsigned distance, Bytes& plain) {
    BitWriter writer;
    plain.resize(distance);
    for (unsigned index = 0; index < distance; ++index)
        plain[index] = index * 37;
    stored_block(writer, plain, false);
    writer.put(3, 3);
    writer.symbol(285, fixed_lengths());
    const unsigned bases[30] = {
        1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
        33,   49,   65,   97,   129,  193,  257,  385,   513,   769,
        1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
    unsigned symbol = 0;
    while (symbol < 29 && bases[symbol + 1] <= distance)
        ++symbol;
    writer.symbol(symbol, std::vector<unsigned>(32, 5));
    writer.put(distance - bases[symbol], symbol < 4 ? 0 : symbol / 2 - 1);
    writer.symbol(256, fixed_lengths());
    for (unsigned index = 0; index < 258; ++index)
        plain.push_back(plain[plain.size() - distance]);
    return writer.bytes;
}

static bool run_case(
    const char*  name,
    const Bytes& compressed,
    const Bytes& expected,
    bool         check_rate = false,
    unsigned     input_gap  = 0,
    bool         empty_tail = false,
    unsigned     rate_limit = 64) {
    auto*      decoder = libdeflate_alloc_decompressor();
    Bytes      reference(expected.size() + 1);
    size_t     actual_size = 0;
    const auto result      = libdeflate_deflate_decompress(
        decoder,
        compressed.data(),
        compressed.size(),
        reference.data(),
        reference.size(),
        &actual_size);
    libdeflate_free_decompressor(decoder);
    if (result != LIBDEFLATE_SUCCESS || actual_size != expected.size() ||
        !std::equal(expected.begin(), expected.end(), reference.begin())) {
        std::fprintf(stderr, "FAIL %s: invalid reference stimulus\n", name);
        return false;
    }
    hls::stream<axi_bit_t> notify_in, notify_out;
    hls::stream<axi_t>     data_in, data_out;
    axi_bit_t              notify;
    notify.data = 1;
    notify_in.write(notify);
    size_t       input_offset = 0;
    Bytes        output;
    bool         done            = false;
    unsigned     last_full_cycle = 0;
    unsigned     max_gap         = 0;
    const size_t timeout =
        20000 + expected.size() * 20 + compressed.size() * (input_gap + 1);
    for (size_t cycle = 0; cycle < timeout && !done; ++cycle) {
        if (input_offset < compressed.size() && cycle % (input_gap + 1) == 0) {
            axi_t axis;
            axis.data = ~ap_uint<512>(0);
            axis.keep = 0;
            for (unsigned byte = 0;
                 byte < 64 && input_offset < compressed.size();
                 ++byte) {
                axis.data(byte * 8 + 7, byte * 8) = compressed[input_offset++];
                axis.keep[byte]                   = 1;
            }
            axis.last = input_offset == compressed.size() && !empty_tail;
            data_in.write(axis);
        } else if (empty_tail && input_offset == compressed.size()) {
            axi_t axis;
            axis.data = 0;
            axis.keep = 0;
            axis.last = 1;
            data_in.write(axis);
            ++input_offset;
        }
        inflate(notify_in, data_in, notify_out, data_out);
        while (!data_out.empty()) {
            const axi_t axis = data_out.read();
            const auto  keep = axis.keep.to_uint64();
            if ((keep & (keep + 1)) != 0 ||
                (!axis.last && keep != ~uint64_t(0))) {
                std::fprintf(stderr, "FAIL %s: invalid output keep\n", name);
                return false;
            }
            if (!axis.last) {
                if (last_full_cycle)
                    max_gap =
                        std::max(max_gap, unsigned(cycle - last_full_cycle));
                last_full_cycle = cycle;
            }
            for (unsigned byte = 0; byte < 64; ++byte) {
                if (axis.keep[byte])
                    output.push_back(axis.data(byte * 8 + 7, byte * 8));
            }
            done = axis.last;
        }
        if (output.size() > expected.size())
            break;
    }
    for (unsigned cycle = 0; cycle < 4; ++cycle)
        inflate(notify_in, data_in, notify_out, data_out);
    const bool passed = done && output == expected && data_in.empty() &&
                        data_out.empty() && notify_in.empty() &&
                        notify_out.empty() &&
                        (!check_rate || max_gap <= rate_limit);
    std::printf(
        "%s %s: expected=%zu actual=%zu done=%d max_full_beat_gap=%u\n",
        passed ? "PASS" : "FAIL",
        name,
        expected.size(),
        output.size(),
        done,
        max_gap);
    return passed;
}

static bool
reference_ok(const char* name, const Bytes& compressed, const Bytes& expected) {
    auto*      decoder = libdeflate_alloc_decompressor();
    Bytes      reference(expected.size() + 1);
    size_t     actual_size = 0;
    const auto result      = libdeflate_deflate_decompress(
        decoder,
        compressed.data(),
        compressed.size(),
        reference.data(),
        reference.size(),
        &actual_size);
    libdeflate_free_decompressor(decoder);
    if (result != LIBDEFLATE_SUCCESS || actual_size != expected.size() ||
        !std::equal(expected.begin(), expected.end(), reference.begin())) {
        std::fprintf(stderr, "FAIL %s: invalid reference stimulus\n", name);
        return false;
    }
    return true;
}

static axi_t input_beat(const Bytes& compressed, size_t& offset) {
    axi_t axis;
    axis.data = ~ap_uint<512>(0);
    axis.keep = 0;
    for (unsigned byte = 0; byte < 64 && offset < compressed.size(); ++byte) {
        axis.data(byte * 8 + 7, byte * 8) = compressed[offset++];
        axis.keep[byte]                   = 1;
    }
    axis.last = offset == compressed.size();
    return axis;
}

// 输入过程中突然 notify(0): 复位请求在当前流结束 (回到 IDLE) 后生效,
// 清空全部解码状态并向下游转发 notify(0); 之后 notify(1) 重新开始的流
// 不能受前一条流残留状态的影响.
static bool run_reset_case(
    const char*  name,
    const Bytes& compressed_a,
    const Bytes& expected_a,
    const Bytes& compressed_b,
    const Bytes& expected_b) {
    if (!reference_ok(name, compressed_a, expected_a) ||
        !reference_ok(name, compressed_b, expected_b))
        return false;
    hls::stream<axi_bit_t> notify_in, notify_out;
    hls::stream<axi_t>     data_in, data_out;
    axi_bit_t              notify;
    notify.data = 1;
    notify_in.write(notify);

    Bytes        output_a, output_b;
    size_t       offset_a = 0, offset_b = 0;
    bool         a_done     = false;
    bool         b_done     = false;
    bool         b_started  = false;
    bool         reset_sent = false;
    bool         reset_seen = false;
    unsigned     resets     = 0;
    const size_t timeout    = 20000 +
                           (expected_a.size() + expected_b.size()) * 20 +
                           compressed_a.size() + compressed_b.size();
    for (size_t cycle = 0; cycle < timeout && !b_done; ++cycle) {
        if (!a_done && offset_a < compressed_a.size()) {
            data_in.write(input_beat(compressed_a, offset_a));
            // 数据刚开始输入 (A 还在解码) 就插入复位
            if (!reset_sent) {
                notify.data = 0;
                notify_in.write(notify);
                reset_sent = true;
            }
        } else if (a_done && reset_seen && !b_started) {
            notify.data = 1;
            notify_in.write(notify);
            b_started = true;
        } else if (b_started && offset_b < compressed_b.size()) {
            data_in.write(input_beat(compressed_b, offset_b));
        }
        inflate(notify_in, data_in, notify_out, data_out);
        while (!data_out.empty()) {
            const axi_t axis = data_out.read();
            const auto  keep = axis.keep.to_uint64();
            if ((keep & (keep + 1)) != 0 ||
                (!axis.last && keep != ~uint64_t(0))) {
                std::fprintf(stderr, "FAIL %s: invalid output keep\n", name);
                return false;
            }
            Bytes& out = a_done ? output_b : output_a;
            for (unsigned byte = 0; byte < 64; ++byte) {
                if (axis.keep[byte])
                    out.push_back(axis.data(byte * 8 + 7, byte * 8));
            }
            if (axis.last) {
                if (!a_done)
                    a_done = true;
                else
                    b_done = true;
            }
        }
        while (!notify_out.empty()) {
            const axi_bit_t n = notify_out.read();
            if (n.data) {
                std::fprintf(stderr, "FAIL %s: unexpected notify(1)\n", name);
                return false;
            }
            ++resets;
            reset_seen = true;
        }
    }
    for (unsigned cycle = 0; cycle < 4; ++cycle)
        inflate(notify_in, data_in, notify_out, data_out);
    const bool passed = a_done && b_done && reset_seen && resets == 1 &&
                        output_a == expected_a && output_b == expected_b &&
                        data_in.empty() && data_out.empty() &&
                        notify_in.empty() && notify_out.empty();
    std::printf(
        "%s %s: expected=%zu+%zu actual=%zu+%zu done=%d/%d resets=%u\n",
        passed ? "PASS" : "FAIL",
        name,
        expected_a.size(),
        expected_b.size(),
        output_a.size(),
        output_b.size(),
        a_done,
        b_done,
        resets);
    return passed;
}

static Bytes compress(const Bytes& input, unsigned level) {
    auto* compressor = libdeflate_alloc_compressor(level);
    Bytes result(libdeflate_deflate_compress_bound(compressor, input.size()));
    result.resize(libdeflate_deflate_compress(
        compressor,
        input.data(),
        input.size(),
        result.data(),
        result.size()));
    libdeflate_free_compressor(compressor);
    return result;
}

int main() {
    // 满拍间隔 <= 64: literal 每拍 1 字节, match 每拍 4 字节, 所以含
    // match 的流满拍可以比 64 拍更密; 慢于 64 拍才说明 literal 吞吐掉了
    const bool        check_rate = true;
    const std::string text =
        "A queryable object is a read-only collection of key/value pair where "
        "each key is a customization point object known as a query object. "
        "A query is an invocation of a query object with a queryable object as "
        "its first argument and a (possibly empty) set of additional "
        "arguments. "
        "A query imposes syntactic and semantic requirements on its "
        "invocations.";
    const Bytes original(text.begin(), text.end());
    if (!run_case("original-dynamic", compress(original, 6), original))
        return 1;
    Bytes literals(4096);
    for (size_t index = 0; index < literals.size(); ++index)
        literals[index] = index % 256;
    BitWriter fixed;
    fixed_block(fixed, literals, true);
    // 8/9 位 literal 走双码配对快路径: 满拍间隔应 <= 40 (约 2 B/cycle);
    // 15 位码字不在推测集合内, 回退到 1 B/cycle (<= 64)
    if (!run_case(
            "fixed-literals",
            fixed.bytes,
            literals,
            check_rate,
            0,
            false,
            40))
        return 1;
    if (!run_case(
            "dynamic-literals",
            dynamic_literals(literals),
            literals,
            check_rate,
            0,
            false,
            40))
        return 1;
    const Bytes long_literals(1024, 14);
    if (!run_case(
            "15-bit-literals",
            dynamic_literals(long_literals, true),
            long_literals,
            check_rate))
        return 1;
    const Bytes repeated(128, 65);
    if (!run_case(
            "repeat-zero-then-16",
            repeat_zero_lengths(repeated),
            repeated))
        return 1;
    for (unsigned size : {0, 1, 63, 64, 65, 511, 512, 513, 4096}) {
        const Bytes plain(literals.begin(), literals.begin() + size);
        BitWriter   stored;
        stored_block(stored, plain, true);
        const auto name = "stored-" + std::to_string(size);
        if (!run_case(name.c_str(), stored.bytes, plain))
            return 1;
    }
    BitWriter blocks;
    fixed_block(blocks, repeated, false);
    stored_block(blocks, {}, false);
    stored_block(blocks, literals, false);
    fixed_block(blocks, original, true);
    Bytes combined = repeated;
    combined.insert(combined.end(), literals.begin(), literals.end());
    combined.insert(combined.end(), original.begin(), original.end());
    if (!run_case("mixed-blocks", blocks.bytes, combined))
        return 1;
    if (!run_case("input-stalls", fixed.bytes, literals, false, 97))
        return 1;
    if (!run_case("empty-input-tail", fixed.bytes, literals, true, 0, true))
        return 1;
    // 输入中途 notify(0): 当前流正常结束并转发 notify(0) 后, 新流从零状态开始
    if (!run_reset_case(
            "reset-mid-input-fixed-dynamic",
            fixed.bytes,
            literals,
            compress(original, 6),
            original))
        return 1;
    if (!run_reset_case(
            "reset-mid-input-dynamic-fixed",
            compress(original, 6),
            original,
            fixed.bytes,
            literals))
        return 1;
    for (unsigned distance :
         {1,
          2,
          3,
          4,
          5,
          7,
          8,
          9,
          31,
          32,
          33,
          34,
          35,
          36,
          37,
          256,
          1024,
          32768}) {
        Bytes      plain;
        const auto encoded = exact_match(distance, plain);
        const auto name    = "exact-distance-" + std::to_string(distance);
        if (!run_case(name.c_str(), encoded, plain))
            return 1;
    }
    for (unsigned distance : {1, 2, 3, 4, 5, 8, 9, 31, 32, 33, 36, 256, 1024}) {
        Bytes plain(70000);
        for (size_t index = 0; index < plain.size(); ++index)
            plain[index] = (index % distance) * 37;
        const auto name = "match-pattern-" + std::to_string(distance);
        if (!run_case(name.c_str(), compress(plain, 6), plain))
            return 1;
    }
    unsigned random = 0x13579bdf;
    for (unsigned level : {0, 1, 6, 12}) {
        Bytes plain(65537);
        for (auto& byte : plain) {
            random ^= random << 13;
            random ^= random >> 17;
            random ^= random << 5;
            byte = random;
        }
        const auto name = "random-level-" + std::to_string(level);
        if (!run_case(name.c_str(), compress(plain, level), plain))
            return 1;
    }
    std::puts("PASS: all inflate cases");
    return 0;
}
