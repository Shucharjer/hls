#pragma once
#include <ap_axi_sdata.h>
#include "data.hpp"

#ifndef CONN_SPORT
#define CONN_SPORT 65535u
#endif

#ifndef MAX_CONN
#define MAX_CONN 32u
#endif

#ifndef CONN_TIMEOUT_SEC
#define CONN_TIMEOUT_SEC 30
#endif

#ifndef CONN_TIMER_FREQ
#define CONN_TIMER_FREQ 300'000'000lu
#endif

using conn_id_t   = ap_uint<consteval_bit_width(MAX_CONN - 1)>;
using conn_size_t = ap_uint<consteval_bit_width(MAX_CONN)>;

using conn_seq_notify_t =
    ap_axiu<1, 32, 32, consteval_bit_width(MAX_CONN - 1), 0>;
struct seq_notify_it {
    ap_uint<1>  data; // 0: rst, 1: data
    ap_uint<32> user; // length
    ap_uint<32> id;   // seq
};

using conn_axi_t = ap_axiu<
    512,
    0,
    0,
    consteval_bit_width(MAX_CONN - 1),
    AXIS_ENABLE_KEEP_LAST>;

inline void connection_timer(
    hls::stream<conn_id_t>& notify_act,
    hls::stream<conn_id_t>& notify_rst) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1

    static conn_size_t i = 0;

    // 多一位, 保证超时前不会回绕
    using time_t =
        ap_uint<consteval_bit_width(CONN_TIMER_FREQ * CONN_TIMEOUT_SEC) + 1>;
    static time_t     timers[MAX_CONN]{};
    static ap_uint<1> active[MAX_CONN]{};
    // overdue[j] == (timers[j] >= 超时周期), 提前一拍算好
    static ap_uint<1> overdue[MAX_CONN]{};
#pragma HLS ARRAY_PARTITION variable = timers complete
#pragma HLS ARRAY_PARTITION variable = active complete
#pragma HLS ARRAY_PARTITION variable = overdue complete

    bool      act = false;
    conn_id_t n   = 0;
    if (!notify_act.empty()) {
        n   = notify_act.read();
        act = true;
    }

    // 每个连接每 MAX_CONN 拍轮询一次是否超时
    bool expire = active[i] && overdue[i];
    if (expire) {
        notify_rst.write(i);
    }

    // 各连接的计时器并行更新: 有数据时清零, 否则每拍加一
    for (unsigned j = 0; j < MAX_CONN; ++j) {
#pragma HLS UNROLL
        if (act && n == j) {
            active[j]  = 1;
            timers[j]  = 0;
            overdue[j] = 0;
        } else {
            if (expire && i == j) {
                active[j] = 0;
            }
            overdue[j] = timers[j] >= CONN_TIMER_FREQ * CONN_TIMEOUT_SEC - 1;
            timers[j]  = timers[j] + 1;
        }
    }

    if (i == MAX_CONN - 1) {
        i = 0;
    } else {
        i = i + 1;
    }
}

inline void connection_handler(
    hls::stream<conn_id_t>&         notify_rst,
    hls::stream<conn_id_t>&         notify_act,
    hls::stream<TcpSportMeta>&      meta_in,
    hls::stream<axi_it>&            data_in,
    hls::stream<conn_seq_notify_t>& notify_out,
    hls::stream<conn_axi_t>&        data_out) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1

    enum {
        WORK_STATE_IDLE,
        WORK_STATE_BUSY
    };

    enum {
        STATE_IDLE,
        STATE_VERIFY,
        STATE_FORWARD,
        STATE_DROP
    };

    static ap_uint<1>   work_state = WORK_STATE_IDLE;
    static ap_uint<3>   state      = STATE_IDLE;
    static TcpSportMeta meta; // 当前包
    static TcpSportMeta pend; // 预取的下一个包
    static ap_uint<1>   pend_valid = 0;
    static ap_uint<80>  conns[MAX_CONN]{};
#pragma HLS ARRAY_PARTITION variable = conns complete

    // 查表结果, 在查表拍寄存
    static ap_uint<MAX_CONN> hit;   // 匹配的连接 (至多一个)
    static ap_uint<MAX_CONN> empty; // 编号最小的空闲连接 (至多一个)
    static ap_uint<1> avail;
    static conn_id_t  curr;

    // 拍首 pend 为空才预取, 读 meta_in 不依赖本拍其它 fifo 的数据
    bool fetch = !pend_valid;

    // rst 同样先读入寄存器
    static conn_id_t  rst_id;
    static ap_uint<1> rst_valid = 0;
    bool              rst_fetch = !rst_valid;

    // 数据同样先读入寄存器, 本拍处理上一拍读入的数据
    static axi_it     beat;
    static ap_uint<1> beat_valid = 0;
    bool              beat_take  = false;

    // 查表使用拍首的连接表. 写连接表 (rst, VERIFY) 的拍不会查表.
    ap_uint<80> table[MAX_CONN];
#pragma HLS ARRAY_PARTITION variable = table complete
    for (unsigned i = 0; i < MAX_CONN; ++i) {
#pragma HLS UNROLL
        table[i] = conns[i];
    }

    // 本拍可以取下一个包的 meta: 空闲且无 rst, 或者当前包的最后一拍.
    // 包尾同拍完成下一个包的查表, 多拍包之间不再有空拍.
    bool next = false;

    switch (work_state) {
    case WORK_STATE_IDLE: {
        if (rst_valid) {
            conns[rst_id] = 0;
            rst_valid     = 0;
            conn_seq_notify_t n;
            n.data = 0;
            n.user = 0;
            n.id   = 0;
            n.dest = rst_id;
            notify_out.write(n);
        } else {
            next = true;
        }
    } break;
    case WORK_STATE_BUSY: {
        if (beat_valid) {
            axi_it tmp    = beat;
            bool   verify = state == STATE_VERIFY;
            beat_take     = true;
            if (verify) {
                // 确定连接, 同拍转发首拍. 命中则沿用, 否则取编号最小的空闲连接
                bool              found = hit != 0;
                ap_uint<MAX_CONN> pick  = found ? hit : empty;
                conn_id_t         id    = 0;
                for (unsigned i = 0; i < MAX_CONN; ++i) {
#pragma HLS UNROLL
                    if (pick[i]) {
                        id |= conn_id_t(i); // pick 为独热码, 或起来即编号
                        conns[i] = meta.conn;
                    }
                }
                curr = id;
                if (!found && !avail) {
                    state = STATE_DROP;
                } else {
                    notify_act.write(id);
                    conn_seq_notify_t n;
                    n.data = 1;
                    n.user = meta.length;
                    n.id   = meta.sndseq;
                    n.dest = id;
                    notify_out.write(n);
                    state = STATE_FORWARD;
                }
            }
            if (state == STATE_FORWARD) {
                conn_axi_t out;
                out.data = tmp.data;
                out.keep = tmp.keep;
                out.last = tmp.last;
                out.dest = curr;
                data_out.write(out);
            }
            if (tmp.last) {
                // VERIFY 拍写了连接表, 下一拍再查表;
                // 有待处理的 rst 时先回到 IDLE
                next       = !verify && !rst_valid;
                work_state = WORK_STATE_IDLE;
            }
        }
    } break;
    }

    if (next && pend_valid) {
        meta       = pend;
        pend_valid = 0;
        ap_uint<MAX_CONN> unused;
        for (unsigned i = 0; i < MAX_CONN; ++i) {
#pragma HLS UNROLL
            hit[i]    = table[i] == pend.conn;
            unused[i] = table[i] == 0;
        }
        avail      = unused != 0;
        empty      = unused & (~unused + 1);
        work_state = WORK_STATE_BUSY;
        state      = STATE_VERIFY;
    }

    if (fetch && !meta_in.empty()) {
        pend       = meta_in.read();
        pend_valid = 1;
    }

    if (rst_fetch && !notify_rst.empty()) {
        rst_id    = notify_rst.read();
        rst_valid = 1;
    }

    // 寄存器空或本拍被取走时读入下一个
    if (beat_take) {
        beat_valid = 0;
    }
    if (!beat_valid && !data_in.empty()) {
        beat       = data_in.read();
        beat_valid = 1;
    }
}

inline void connection_notify_dispatch(
    hls::stream<conn_seq_notify_t>& notify_in,
    hls::stream<seq_notify_it> (&notify_out)[MAX_CONN]) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1
    if (!notify_in.empty()) {
        conn_seq_notify_t n = notify_in.read();
        seq_notify_it     notify;
        notify.data = n.data;
        notify.user = n.user;
        notify.id   = n.id;
        for (conn_size_t i = 0; i < MAX_CONN; ++i) {
            if (i == n.dest) {
                notify_out[i].write(notify);
            }
        }
    }
}

inline void connection_data_dispatch(
    hls::stream<conn_axi_t>& data_in,
    hls::stream<axi_it> (&data_out)[MAX_CONN]) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1
    if (!data_in.empty()) {
        conn_axi_t n = data_in.read();
        axi_it     notify;
        notify.data = n.data;
        notify.keep = n.keep;
        notify.last = n.last;
        for (conn_size_t i = 0; i < MAX_CONN; ++i) {
            if (i == n.dest) {
                data_out[i].write(notify);
            }
        }
    }
}

// clang-format off
/// @brief TCP 重组
/// 定义 A, B, C 为有序数据包, 且预期收到 A 对应的数据:
/// 当接收到 A, B, C 时, 立即向下游输出, 总共 3 notify(1), 3 次数据;
/// 当接收到 A, C 时, 先正常发送 A, 在收到 C 时, 先向下游发送 notify(0), 再向下游发送 notify(1) 和数据;
/// 当接收到 B, A, C 时, 先缓存 B, 收到 A 时,
/// 向下游发送 notify(1) 和 data, 数据需要拼接, 最后正常发送 C.
/// 乱序包先缓存, 缺口补齐后再拼接下发; slots 用尽仍未补齐时,
/// 向下游发送 notify(0), 丢弃缓存并以当前包为新流起点.
/// slot 查表每拍只比较一个, 面积与 num_slots 基本无关.
/// 输入拍 keep 以外的字节须为 0 (由 filter_tcp_sport 保证), 拼接时直接按字节 OR.
// clang-format on
inline void tcp_stream(
    hls::stream<seq_notify_it>& notify_in,
    hls::stream<axi_it>&        data_in,
    hls::stream<ap_uint<1>>&    notify_out,
    hls::stream<axi_it>&        data_out) {
#pragma HLS INLINE   off
#pragma HLS PIPELINE II = 1

    constexpr auto num_slots  = 32u;
    constexpr auto slot_size  = 32;
    constexpr auto slot_bytes = 32 * 64;

    static ap_uint<512> buf[num_slots][slot_size];
    static ap_uint<32>  seqs[num_slots];
    static ap_uint<32>  ends[num_slots];
    static ap_uint<num_slots> valid = 0;
    static ap_uint<32>  expect   = 0;
    static ap_uint<1>   started  = 0; // 连接的首个包确定起始 seq
#pragma HLS BIND_STORAGE variable = seqs type = ram_1p impl = lutram
#pragma HLS BIND_STORAGE variable = ends type = ram_1p impl = lutram

    // 当前包
    static ap_uint<32> pend_end;
    static ap_uint<32> in_rem; // 当前包剩余字节
    static ap_uint<32> skip;
    static ap_uint<32> sav_seq;
    static ap_uint<32> sav_end;

    // 当前读取的缓存段
    static ap_uint<5>  slot;
    static ap_uint<6>  off;
    static ap_uint<12> rem;
    static ap_uint<32> seg_end;

    // 槽扫描 (每拍一个槽)
    static ap_uint<5>  scan_i;
    static ap_uint<1>  scan_hit_mode; // 0: 找空槽/重复, 1: 找后继段
    static ap_uint<32> scan_seq;
    static ap_uint<32> scan_end;
    static ap_uint<32> scan_plen;
    static ap_uint<5>  scan_free;
    static ap_uint<1>  scan_free_ok;
    static ap_uint<5>  scan_dup;
    static ap_uint<1>  scan_dup_ok;
    static ap_uint<5>  scan_hit;
    static ap_uint<1>  scan_hit_ok;
    static ap_uint<12> scan_len;

    // 输出拼接
    static ap_uint<1024> pbuf;
    static ap_uint<8>    plen   = 0;
    static ap_uint<1>    tail_pending = 0;
    // 追加级: 状态机只寄存本拍数据, 拼接/输出在下一拍
    static ap_uint<1>    svalid = 0;
    static ap_uint<512>  sdata;
    static ap_uint<7>    slen;

    enum {
        WORK_STATE_IDLE,
        WORK_STATE_BUSY
    };

    enum {
        STATE_FORWARD, // 数据连续, 透传当前数据
        STATE_SAVE,    // 发生乱序, 缓存当前数据
        STATE_FLUSH,   // 刷新缓存
        STATE_DROP,
        STATE_FINISH,  // 输出拼接缓冲的末拍
        STATE_SCAN,    // 逐槽查表
        STATE_RESYNC   // 缓存已满, 以当前包作为新流起点
    };

    static ap_uint<1> work_state = WORK_STATE_IDLE;
    static ap_uint<3> state       = STATE_FORWARD;

    bool   take = work_state == WORK_STATE_BUSY
               && (state == STATE_FORWARD || state == STATE_SAVE
                   || state == STATE_DROP)
               && !data_in.empty();
    axi_it tmp;
    if (take) {
        tmp = data_in.read();
    }

    bool         app      = false;
    ap_uint<512> app_data = 0;
    ap_uint<7>   app_len  = 0;
    bool         ntf      = false;
    ap_uint<1>   ntf_val  = 0;
    bool         fin_now  = false; // 本拍确认运行结束

    switch (work_state) {
    case WORK_STATE_IDLE: {
        if (!notify_in.empty()) {
            seq_notify_it n = notify_in.read();
            if (n.data) {
                if (!started) {
                    expect  = n.id;
                    started = 1;
                }
                // 0: 连续; MSB=0: 超前; MSB=1: 已转发过
                ap_uint<32> d    = n.id - expect;
                ap_uint<32> dend = n.id + n.user;
                if (d == 0) {
                    pend_end   = dend;
                    in_rem     = n.user;
                    skip       = 0;
                    ntf        = true;
                    ntf_val    = 1;
                    state      = STATE_FORWARD;
                    work_state = WORK_STATE_BUSY;
                } else if (d[31]) {
                    ap_uint<32> tail = dend - expect;
                    if (tail == 0 || tail[31]) {
                        state      = STATE_DROP;
                        work_state = WORK_STATE_BUSY;
                    } else {
                        // 与已转发数据部分重叠, 丢掉头部 (按节奏拍递减)
                        skip       = expect - n.id;
                        pend_end   = dend;
                        in_rem     = n.user;
                        ntf        = true;
                        ntf_val    = 1;
                        state      = STATE_FORWARD;
                        work_state = WORK_STATE_BUSY;
                    }
                } else {
                    // 乱序: 逐槽找重复或空闲槽
                    scan_seq      = n.id;
                    scan_end      = dend;
                    scan_plen     = n.user;
                    scan_i        = 0;
                    scan_hit_mode = 0;
                    scan_free_ok  = 0;
                    scan_dup_ok   = 0;
                    state         = STATE_SCAN;
                    work_state    = WORK_STATE_BUSY;
                }
            } else {
                // 重置 TCP 接收
                expect  = 0;
                started = 0;
                valid   = 0;
                plen         = 0;
                pbuf         = 0;
                tail_pending = 0;
                ntf     = true;
                ntf_val = 0;
            }
        }
    } break;
    case WORK_STATE_BUSY: {
        switch (state) {
        case STATE_FORWARD: {
            if (take) {
                ap_uint<7> n = in_rem > 64 ? ap_uint<7>(64)
                                           : ap_uint<7>(in_rem);
                in_rem -= n;
                ap_uint<512> d = tmp.data;
                if (skip >= n) {
                    skip -= n;
                    n = 0;
                    d = 0;
                } else if (skip != 0) {
                    d >>= (skip << 3);
                    n -= ap_uint<7>(skip);
                    skip = 0;
                }
                app      = true;
                app_data = d;
                app_len  = n;
                if (tmp.last) {
                    expect = pend_end;
                    // 无缓存段时只比对最后一个槽, 少等一拍
                    scan_i        = valid == 0 ? ap_uint<5>(num_slots - 1)
                                               : ap_uint<5>(0);
                    scan_hit_mode = 1;
                    scan_hit_ok   = 0;
                    state         = STATE_SCAN;
                }
            }
        } break;
        case STATE_SAVE: {
            if (take) {
                buf[slot][off] = tmp.data;
                off            = off + 1;
                if (tmp.last) {
                    seqs[slot]  = sav_seq;
                    ends[slot]  = sav_end;
                    valid[slot] = 1;
                    work_state  = WORK_STATE_IDLE;
                }
            }
        } break;
        case STATE_FLUSH: {
            ap_uint<7> n = rem > 64 ? ap_uint<7>(64) : ap_uint<7>(rem);
            app          = true;
            app_data     = buf[slot][off];
            app_len      = n;
            rem          = rem - n;
            off          = off + 1;
            if (rem == 0) {
                ap_uint<num_slots> left =
                    valid & ~(ap_uint<num_slots>(1) << slot);
                valid  = left;
                expect = seg_end;
                scan_i = left == 0 ? ap_uint<5>(num_slots - 1) : ap_uint<5>(0);
                scan_hit_mode = 1;
                scan_hit_ok   = 0;
                state         = STATE_SCAN;
            }
        } break;
        case STATE_SCAN: {
            bool v = valid[scan_i];
            if (scan_hit_mode == 0) {
                // 保存: 记录最低的重复槽/空闲槽
                if (!v && !scan_free_ok) {
                    scan_free    = scan_i;
                    scan_free_ok = 1;
                }
                if (v && seqs[scan_i] == scan_seq && !scan_dup_ok) {
                    scan_dup    = scan_i;
                    scan_dup_ok = 1;
                }
            } else if (v && seqs[scan_i] == expect && !scan_hit_ok) {
                scan_hit    = scan_i;
                scan_hit_ok = 1;
                scan_len    = ap_uint<12>(ends[scan_i] - seqs[scan_i]);
            }

            if (scan_i != num_slots - 1) {
                scan_i = scan_i + 1;
            } else if (scan_hit_mode == 0) {
                if (scan_dup_ok) {
                    slot       = scan_dup;
                    off        = 0;
                    sav_seq    = scan_seq;
                    sav_end    = scan_end;
                    state      = STATE_SAVE;
                } else if (scan_free_ok && scan_plen <= slot_bytes) {
                    slot       = scan_free;
                    off        = 0;
                    sav_seq    = scan_seq;
                    sav_end    = scan_end;
                    state      = STATE_SAVE;
                } else {
                    // slots 用尽且缺口未补齐: 通知下游断流
                    valid      = 0;
                    expect     = scan_seq;
                    pend_end   = scan_end;
                    in_rem     = scan_plen;
                    skip       = 0;
                    ntf        = true;
                    ntf_val    = 0;
                    state      = STATE_RESYNC;
                }
            } else if (scan_hit_ok) {
                slot    = scan_hit;
                off     = 0;
                rem     = scan_len;
                seg_end = expect + ap_uint<32>(scan_len);
                state   = STATE_FLUSH;
            } else {
                state   = STATE_FINISH;
                fin_now = true;
            }
        } break;
        case STATE_DROP: {
            if (take && tmp.last) {
                work_state = WORK_STATE_IDLE;
            }
        } break;
        case STATE_FINISH: {
            // 追加级在 switch 之后输出剩余数据
        } break;
        case STATE_RESYNC: {
            ntf     = true;
            ntf_val = 1;
            state   = STATE_FORWARD;
        } break;
        }
    } break;
    }

    // 追加级: 只用寄存的数据做拼接与输出
    if (svalid) {
        pbuf = (ap_uint<1024>(sdata) << (ap_uint<10>(plen) << 3)) | pbuf;
        plen = plen + slen;
        if (plen > 64) {
            axi_it o;
            o.data = pbuf(511, 0);
            o.keep = ap_uint<64>(-1);
            o.last = 0;
            data_out.write(o);
            pbuf >>= 512;
            plen  -= 64;
        }
    }

    // 运行结束: 排空拼接缓冲, 末拍带 tlast
    if (work_state == WORK_STATE_BUSY && (fin_now || tail_pending)) {
        if (plen > 64) {
            axi_it o;
            o.data = pbuf(511, 0);
            o.keep = ap_uint<64>(-1);
            o.last = 0;
            data_out.write(o);
            pbuf >>= 512;
            plen         -= 64;
            tail_pending  = 1;
        } else {
            axi_it o;
            o.data = pbuf(511, 0);
            o.keep = ap_uint<64>(-1) >> (64 - plen);
            o.last = 1;
            data_out.write(o);
            pbuf         = 0;
            plen         = 0;
            tail_pending = 0;
            work_state   = WORK_STATE_IDLE;
        }
    }

    svalid       = app;
    sdata        = app_data;
    slen         = app_len;

    if (ntf) {
        notify_out.write(ntf_val);
    }
}
