#pragma once
#include "data.hpp"

#define MAX_TCP_CONN 32

inline void filter_tcp(
    hls::stream<axi_t>&  in,
    hls::stream<IpMeta>& om,
    hls::stream<axi_it>& out) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1
    enum {
        STATE_IDLE,
        STATE_WORK,
        STATE_FLUSH
    };

    static ap_uint<2> state = STATE_IDLE;
    static ap_uint<4> ihl;  // IP 头长度, 以 4 字节为单位
    static ap_uint<1> pass; // 当前帧是否为 IPv4 TCP, 否则 WORK 只读不写
    static axi_it     prev; // 上一拍的原始数据

    // TCP 段从帧的第 14 + ihl * 4 字节开始. 每拍输出 {hi, lo} 右移这么多字节,
    // 各状态只选择移位器的输入, 移位器只有一个, 在 switch 之后.
    // 14 + ihl * 4 = 2 + (ihl + 3) * 4: 先固定右移 2 字节, 再按 4 字节移 ihl + 3 档.
    bool         emit    = false;
    ap_uint<512> hi      = 0;
    ap_uint<64>  hi_keep = 0;
    ap_uint<512> lo      = prev.data;
    ap_uint<64>  lo_keep = prev.keep;
    ap_uint<4>   sh      = ihl + 3;
    ap_uint<1>   last    = 1;

    switch (state) {
    case STATE_IDLE: {
        if (!in.empty()) {
            axi_t       tmp      = in.read();
            ap_uint<16> eth_type = tmp.data(111, 96);
            ap_uint<8>  ip_proto = tmp.data(191, 184);
            ap_uint<16> totlen   = bswap(ap_uint<16>(tmp.data(143, 128)));
            pass = eth_type == 0x0008 && ip_proto == 0x06;
            if (pass) {
                IpMeta meta;
                meta.addr = tmp.data(271, 208);
                meta.datalen =
                    totlen - (ap_uint<16>(tmp.data(115, 112)) << 2);
                om.write(meta);
            }
            ihl       = tmp.data(115, 112);
            prev.data = tmp.data;
            prev.keep = tmp.keep;
            lo        = tmp.data;
            lo_keep   = tmp.keep;
            if (tmp.last) {
                // 单拍帧可能带以太网最小帧填充, 按 IP 总长截断
                lo_keep &= ~(ap_uint<64>(-1) << (totlen(5, 0) + 14));
            }
            sh        = ap_uint<4>(tmp.data(115, 112)) + 3;
            emit      = pass && tmp.last;
            // 状态只取决于 last, 协议比较的结果寄存在 pass 中
            state = tmp.last ? STATE_IDLE : STATE_WORK;
        }
    } break;
    case STATE_FLUSH: {
        // 输出 WORK 末拍溢出的部分
        emit  = true;
        state = STATE_IDLE;
    } break;
    case STATE_WORK: {
        if (!in.empty()) {
            axi_t tmp = in.read();
            hi        = tmp.data;
            hi_keep   = tmp.keep;
            prev.data = tmp.data;
            prev.keep = tmp.keep;
            emit      = pass;
            // keep 连续: 末拍超过 14 + ihl * 4 字节即溢出.
            // 起始字节恒为 4k + 2, 只抽样这些位, 选择器只有 16 档
            ap_uint<16> keep_s;
            for (int i = 0; i < 16; ++i) {
#pragma HLS UNROLL
                keep_s[i] = tmp.keep[(i << 2) + 2];
            }
            ap_uint<1> spill = pass && keep_s[sh];
            last             = tmp.last && !spill;
            if (tmp.last) {
                // 余下部分在 FLUSH 输出, 每拍只写一次.
                // STATE_FLUSH = 2, STATE_IDLE = 0, 即 {spill, 0}
                state = (spill, ap_uint<1>(0));
            }
        }
    } break;
    }

    if (emit) {
        ap_uint<1024> data = (hi, lo);
        ap_uint<128>  keep = (hi_keep, lo_keep);
        axi_it        o;
        o.data = (data >> 16) >> (ap_uint<10>(sh) << 5);
        o.keep = (keep >> 2) >> (ap_uint<7>(sh) << 2);
        o.last = last;
        out.write(o);
    }
}

inline void filter_tcp_sport(
    hls::stream<IpMeta>&       meta_in,
    hls::stream<axi_it>&       data_in,
    ap_uint<16>                sport,
    hls::stream<TcpSportMeta>& meta_out,
    hls::stream<axi_it>&       data_out) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1
    enum {
        STATE_IDLE,
        STATE_HEADER,
        STATE_CONSUME,
        STATE_FLUSH
    };

    static ap_uint<2>   state = STATE_IDLE;
    static IpMeta       ip_meta;
    static TcpSportMeta tcp_meta;
    static ap_uint<4>   doff; // TCP 头长度, 以 4 字节为单位
    static ap_uint<1>   pass; // 当前包是否转发, 否则 CONSUME 只读不写
    static axi_it       prev; // 上一拍的原始数据

    // 载荷从 TCP 段的第 doff * 4 字节开始. 每拍输出 {hi, lo} 右移 doff 个字,
    // 各状态只选择移位器的输入, 移位器只有一个, 在 switch 之后.
    bool         emit    = false;
    ap_uint<512> hi      = 0;
    ap_uint<64>  hi_keep = 0;
    ap_uint<512> lo      = prev.data;
    ap_uint<64>  lo_keep = prev.keep;
    ap_uint<4>   sh      = doff;
    ap_uint<1>   last    = 1;

    switch (state) {
    case STATE_IDLE: {
        if (!meta_in.empty()) {
            meta_in.read(ip_meta);
            tcp_meta.conn(63, 0) = ip_meta.addr;
            state                = STATE_HEADER;
        }
    } break;
    case STATE_HEADER: {
        if (!data_in.empty()) {
            axi_it            tmp       = data_in.read();
            const ap_uint<16> port      = tmp.data(15, 0);
            const ap_uint<4>  len       = tmp.data(103, 100);
            const ap_uint<16> hdr_bytes = ap_uint<16>(len) << 2;
            pass = sport == port && ip_meta.datalen != hdr_bytes;
            if (pass) {
                tcp_meta.conn(79, 64) = tmp.data(31, 16);
                tcp_meta.length       = ip_meta.datalen - hdr_bytes;
                tcp_meta.sndseq       = bswap(ap_uint<32>(tmp.data(63, 32)));
                meta_out.write(tcp_meta);
            }
            doff = len;
            prev = tmp;
            lo   = tmp.data;
            lo_keep = tmp.keep;
            sh   = len;
            emit = pass && tmp.last;
            // 状态只取决于 last, 端口比较的结果寄存在 pass 中
            state = tmp.last ? STATE_IDLE : STATE_CONSUME;
        }
    } break;
    case STATE_CONSUME: {
        if (!data_in.empty()) {
            axi_it tmp = data_in.read();
            hi         = tmp.data;
            hi_keep    = tmp.keep;
            prev       = tmp;
            emit       = pass;
            // keep 连续: 末拍超过 doff * 4 字节即溢出 (丢弃的包不溢出).
            // 头长度是 4 的倍数, 只抽样这些位, 选择器只有 16 档
            ap_uint<16> keep_s;
            for (int i = 0; i < 16; ++i) {
#pragma HLS UNROLL
                keep_s[i] = tmp.keep[i << 2];
            }
            ap_uint<1> spill = pass && keep_s[doff];
            last             = tmp.last && !spill;
            if (tmp.last) {
                // 溢出部分在 FLUSH 输出, 每拍只写一次.
                // STATE_FLUSH = 3, STATE_IDLE = 0, 即 {2{spill}}
                state = (spill, spill);
            }
        }
    } break;
    case STATE_FLUSH: {
        // 输出 CONSUME 末拍溢出的部分
        emit  = true;
        state = STATE_IDLE;
    } break;
    }

    if (emit) {
        ap_uint<1024> data = (hi, lo);
        ap_uint<128>  keep = (hi_keep, lo_keep);
        axi_it        o;
        o.data = data >> (ap_uint<10>(sh) << 5);
        o.keep = keep >> (ap_uint<7>(sh) << 2);
        // 无效字节清 0, 下游 tcp_stream 拼接时可直接按字节 OR
        for (unsigned i = 0; i < 64; ++i) {
#pragma HLS UNROLL
            if (!o.keep[i]) {
                o.data((i << 3) + 7, i << 3) = 0;
            }
        }
        o.last = last;
        data_out.write(o);
    }
}
