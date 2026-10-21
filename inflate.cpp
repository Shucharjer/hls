#include "data.hpp"
#ifdef INFLATE_TRACE
#include "print.hpp"
#else
#define nsprintf(...)
#endif

// 一个块结束: 最后一块则输出最后一拍, 否则读下一个块头
#define BLOCK_END             \
    if (last_block) {         \
        nstate = STATE_DONE;  \
    } else {                  \
        nstate = STATE_BLOCK; \
        sh_reg = 3;           \
    }

// 开始解压一个 huffman 块: FILL 还没有解出码字, 第一拍只解码
#define LITLEN_START                    \
    nstate     = STATE_DECOMPRESS_FILL; \
    pend_none  = 1;                     \
    is_literal = 0;                     \
    sh_reg     = 0;

// 一条压缩流处理完, 回到 IDLE 等下一个 notify.
// bitbuf/length/beat_bits 只在 switch 之后的补位口写 (同一变量只有一个写入点,
// 不在 switch 内写后被补位口读), 由下一条流开始时在 IDLE 清 0
#define STREAM_RESET              \
    work_state = WORK_STATE_IDLE; \
    nstate     = STATE_BLOCK;     \
    sh_reg     = 3;

namespace internal {

const uint16_t static_litlen_bl_count[16] =
    {0, 0, 0, 0, 0, 0, 0, 24, 152, 112, 0, 0, 0, 0, 0, 0};
const uint16_t static_litlen_next_code[16] =
    {0, 0, 0, 0, 0, 0, 0, 0, 48, 400, 1024, 2048, 4096, 8192, 16384, 32768};
const uint16_t static_litlen_offsets[16] =
    {0, 0, 0, 0, 0, 0, 0, 0, 24, 176, 288, 288, 288, 288, 288, 288};
const uint16_t static_litlen_symbols[288] = {
    256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270,
    271, 272, 273, 274, 275, 276, 277, 278, 279, 0,   1,   2,   3,   4,   5,
    6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,
    21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,
    36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,
    51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,
    66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
    81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,
    96,  97,  98,  99,  100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
    111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125,
    126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
    141, 142, 143, 280, 281, 282, 283, 284, 285, 286, 287, 144, 145, 146, 147,
    148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162,
    163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177,
    178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192,
    193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207,
    208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222,
    223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237,
    238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252,
    253, 254, 255};

// next_code + 该码长中 literal (symbol < 256) 的个数: 7 位码全是 256..279,
// 8 位码前 144 个 (0..143) 是 literal, 9 位码全是 literal (144..255)
const uint16_t static_litlen_lit_limit[16] =
    {0, 0, 0, 0, 0, 0, 0, 0, 192, 512, 1024, 2048, 4096, 8192, 16384, 32768};

const uint16_t static_dist_bl_count[16] =
    {0, 0, 0, 0, 0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint16_t static_dist_next_code[16] =
    {0, 0, 0, 0, 0, 0, 60, 120, 240, 480, 960, 1920, 3840, 7680, 15360, 30720};
const uint16_t static_dist_offsets[16] =
    {0, 0, 0, 0, 0, 0, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30};
const uint16_t static_dist_symbols[30] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};

// 固定宽度的位反转. 解码时对窗口只反转一次, 第 l 路码字取
// bitswap(win, 15) >> (15 - l) (l 展开后是常数移位), 不再按 l 各做一次
// 可变宽度反转: 反转是纯连线, 但原来的写法让 win 的每一位扇出到 15 路
// 可变 mux, 且每路比较器前多 1-2 级逻辑 (这是每拍一个 literal 的递归路径)
inline ap_uint<15> bitswap15(const ap_uint<15>& x) {
#pragma HLS INLINE
    ap_uint<15> r;
    for (int i = 0; i < 15; ++i) {
#pragma HLS UNROLL
        r[i] = x[14 - i];
    }
    return r;
}

inline ap_uint<7> bitswap7(const ap_uint<7>& x) {
#pragma HLS INLINE
    ap_uint<7> r;
    for (int i = 0; i < 7; ++i) {
#pragma HLS UNROLL
        r[i] = x[6 - i];
    }
    return r;
}

// 码字长度: lt[l] = 前 l 位 < limit[l], 随 l 单调 (一旦为真后面都为真),
// 第一个为真的 l 即码字长度. 直接从温度计码编码, 每一位只看几路, 不先算
// one-hot 的 hit 再做 15 位 OR 归约 (这是每拍一个 literal 的递归路径).
// 非法码字 (lt 全为 0) 得到 14, 与之前的 0 一样只是无意义的值
inline ap_uint<4> therm_len(const ap_uint<16>& lt) {
#pragma HLS INLINE
    ap_uint<4> len;
    len[3] = !lt[7];
    len[2] = (!lt[3] && lt[7]) || !lt[11];
    len[1] =
        (!lt[1] && lt[3]) || (!lt[5] && lt[7]) || (!lt[9] && lt[11]) || !lt[13];
    len[0] = lt[1] || (!lt[2] && lt[3]) || (!lt[4] && lt[5]) ||
             (!lt[6] && lt[7]) || (!lt[8] && lt[9]) || (!lt[10] && lt[11]) ||
             (!lt[12] && lt[13]) || (!lt[14] && lt[15]);
    return len;
}

} // namespace internal

extern "C" void inflate(
    hls::stream<axi_bit_t>& notify_in,
    hls::stream<axi_t>&     data_in,
    hls::stream<axi_bit_t>& notify_out,
    hls::stream<axi_t>&     data_out) {

#pragma HLS INTERFACE axis port = notify_in
#pragma HLS INTERFACE axis port = notify_out
#pragma HLS INTERFACE axis port = data_in depth = 512
#pragma HLS INTERFACE axis port = data_out depth = 512
#pragma HLS PIPELINE                       II    = 1
#pragma HLS INTERFACE ap_ctrl_none         port  = return

    // 下标为 symbol - 256 (即 symbol 低 5 位), [0] 是 EOB, [30]/[31] 非法, 补 0
    // 补齐到 32 项, 非法 symbol 也不会越界
    const ap_uint<9> length_base[32] = {
        0,  3,  4,  5,  6,  7,  8,  9,  10,  11,  13,  15,  17,  19,  23, 27,
        31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258, 0,  0};
    const ap_uint<3> length_extra[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
                                         1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4,
                                         4, 4, 4, 5, 5, 5, 5, 0, 0, 0};
    // [30]/[31] 非法, 补 0
    const ap_uint<15> dist_base[32] = {
        1,    2,    3,    4,    5,    7,     9,     13,    17,  25,   33,
        49,   65,   97,   129,  193,  257,   385,   513,   769, 1025, 1537,
        2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577, 0,   0};
    // (1 << dist_extra) - 1, 在 DIST 中与 dist_extra 一起查表, COPY_START
    // 不用现算 mask
    const ap_uint<13> dist_mask[32] = {
        0,    0,    0,    0,    1,    1,    3,    3,    7,   7,   15,
        15,   31,   31,   63,   63,   127,  127,  255,  255, 511, 511,
        1023, 1023, 2047, 2047, 4095, 4095, 8191, 8191, 0,   0};
    // dist_base - 4 (按 15 位取模), DIST 中算 fe_pos + 4 - base
    const ap_uint<15> dist_base_m4[32] = {
        32765, 32766, 32767, 0,    1,    3,     5,     9,     13,    21,   29,
        45,    61,    93,    125,  189,  253,   381,   509,   765,   1021, 1533,
        2045,  3069,  4093,  6141, 8189, 12285, 16381, 24573, 32764, 32764};
    const ap_uint<4> dist_extra[32] = {0,  0,  0,  0,  1,  1,  2,  2,  3, 3, 4,
                                       4,  5,  5,  6,  6,  7,  7,  8,  8, 9, 9,
                                       10, 10, 11, 11, 12, 12, 13, 13, 0, 0};
    // 放寄存器 (常数 mux), 避免 ROM 读延迟让查表结果晚一拍
#pragma HLS ARRAY_PARTITION variable = length_base complete
#pragma HLS ARRAY_PARTITION variable = length_extra complete
#pragma HLS ARRAY_PARTITION variable = dist_base complete
#pragma HLS ARRAY_PARTITION variable = dist_extra complete
#pragma HLS ARRAY_PARTITION variable = dist_mask complete
#pragma HLS ARRAY_PARTITION variable = dist_base_m4 complete
    // 固定块的 litlen 表项, 与动态块 litlen_table_symbols 的编码相同
    // (literal 为字节值, length 码为 {length_extra(3), length_base(9)}, EOB 为
    // 0), 下标为 internal::static_litlen_symbols 的下标. FILL 中与动态表
    // 一样 288 选 1, 不再按下标分段计算 (加法和比较会串在查表之前)
    const ap_uint<12> static_litlen_entry[288] = {
        0,    3,    4,    5,    6,    7,    8,    9,    10,   523,  525,  527,
        529,  1043, 1047, 1051, 1055, 1571, 1579, 1587, 1595, 2115, 2131, 2147,
        0,    1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,
        12,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,
        24,   25,   26,   27,   28,   29,   30,   31,   32,   33,   34,   35,
        36,   37,   38,   39,   40,   41,   42,   43,   44,   45,   46,   47,
        48,   49,   50,   51,   52,   53,   54,   55,   56,   57,   58,   59,
        60,   61,   62,   63,   64,   65,   66,   67,   68,   69,   70,   71,
        72,   73,   74,   75,   76,   77,   78,   79,   80,   81,   82,   83,
        84,   85,   86,   87,   88,   89,   90,   91,   92,   93,   94,   95,
        96,   97,   98,   99,   100,  101,  102,  103,  104,  105,  106,  107,
        108,  109,  110,  111,  112,  113,  114,  115,  116,  117,  118,  119,
        120,  121,  122,  123,  124,  125,  126,  127,  128,  129,  130,  131,
        132,  133,  134,  135,  136,  137,  138,  139,  140,  141,  142,  143,
        2163, 2691, 2723, 2755, 2787, 258,  0,    0,    144,  145,  146,  147,
        148,  149,  150,  151,  152,  153,  154,  155,  156,  157,  158,  159,
        160,  161,  162,  163,  164,  165,  166,  167,  168,  169,  170,  171,
        172,  173,  174,  175,  176,  177,  178,  179,  180,  181,  182,  183,
        184,  185,  186,  187,  188,  189,  190,  191,  192,  193,  194,  195,
        196,  197,  198,  199,  200,  201,  202,  203,  204,  205,  206,  207,
        208,  209,  210,  211,  212,  213,  214,  215,  216,  217,  218,  219,
        220,  221,  222,  223,  224,  225,  226,  227,  228,  229,  230,  231,
        232,  233,  234,  235,  236,  237,  238,  239,  240,  241,  242,  243,
        244,  245,  246,  247,  248,  249,  250,  251,  252,  253,  254,  255,
    };
#pragma HLS ARRAY_PARTITION variable = static_litlen_entry complete
    const ap_uint<5> code_length_order[19] =
        {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

    enum {
        WORK_STATE_IDLE,
        WORK_STATE_BUSY
    };

    enum {
        BLOCK_TYPE_UNCOMPRESSED    = 0,
        BLOCK_TYPE_STATIC_HUFFMAN  = 1,
        BLOCK_TYPE_DYNAMIC_HUFFMAN = 2,
        BLOCK_TYPE_INVALID         = 3
    };

    enum {
        STATE_BLOCK,
        // 固定块: 把常数码表整块写入寄存器表, 之后解码路径上不再有
        // static_block 选择 (每路比较前少一级 mux, 也少了 static_block
        // 这个高扇出寄存器)
        STATE_LOAD_STATIC_TABLES,
        STATE_DYNAMIC_HUFFMAN_HEADER,
        STATE_DYNAMIC_HUFFMAN_CODE_LENS,

        STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_CAL_COUNT,
        STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_SYMBOL_OFFSET,
        STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_POS,
        STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_SYMBOLS,

        STATE_DYNAMIC_HUFFMAN_DECODE_TABLE,
        STATE_DYNAMIC_HUFFMAN_DECODE_TABLE_FILL,

        STATE_DYNAMIC_HUFFMAN_BUILD_FILL_LENGTHS,

        STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_CAL_COUNT,
        STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_SYMBOL_OFFSET,
        STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_POS,
        STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_SYMBOLS,

        STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_CAL_COUNT,
        STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_SYMBOL_OFFSET,
        STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_POS,
        STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_SYMBOLS,

        // 解码 (前端): literal 每拍一个 (FILL), match 为 FILL -> LEN -> DIST
        // -> COPY_START, COPY_START 推出 match token 并解出下一个码字.
        // 拷贝由后端按 token 进行, 与解码并行
        STATE_DECOMPRESS_FILL,
        STATE_DECOMPRESS_LEN,
        STATE_DECOMPRESS_DIST,
        STATE_DECOMPRESS_COPY_START,

        STATE_STORED_ALIGN,
        STATE_STORED_HEADER,
        STATE_STORED_COPY,

        // 输出最后一拍 (last = 1), 丢弃剩余输入直到 tlast
        STATE_DONE,
        STATE_DRAIN,

        STATE_NUM
    };
    static_assert(STATE_NUM <= 64, "state is ap_uint<6>");

    static ap_uint<2> work_state = WORK_STATE_IDLE;
    static ap_uint<6> state      = STATE_BLOCK;

    // 本条流的 tlast 拍已从 data_in 读出
    static ap_uint<1> last;
    // 输入暂存级: data_in -> in_* -> beat_buf. 读口之后只做寄存, keep 的
    // 掩码和 clz 在搬入 beat_buf 时从寄存器算, 不和读握手串在一拍
    static ap_uint<1> in_valid = 0;
    // state == STATE_DRAIN 的寄存器副本, 只由 DONE/DRAIN 和 last 决定, 与
    // state 同步更新. 直接比较 state 时, 6 位比较串在 data_in 的读握手之前
    static ap_uint<1>   in_drain = 0;
    static ap_uint<512> in_data;
    static ap_uint<64>  in_keep;
    // bitbuf 中的有效位数. 只在 length < 需求 (最大 57) 时补 64 位, 最大 127
    static ap_uint<7> length = 0;
    // 可变移位器的宽度决定 LUT 和时序, 所以 bitbuf 只留 128 位,
    // 整拍数据先放 beat_buf (只做常数移位), 由补位口每次补 64 位
    // length 以上的位恒为 0, 补位用 OR 写入 (见 "唯一的补位口")
    static ap_uint<128> bitbuf = 0;
    static ap_uint<512> beat_buf;
    // beat_buf 各字节的 keep, 与 beat_buf 同步右移 (每次补位移 8 位)
    static ap_uint<64> beat_keep;
    // beat_buf 中剩余的有效位数, [0, 512]
    static ap_uint<10> beat_bits = 0;
    // beat_bits == 0 及 eos = last && !in_valid && beat_bits == 0 (输入已
    // 全部进入 bitbuf) 的寄存器副本, 与 beat_bits/in_valid/last 同步写入.
    // 它们决定 do_shift/do_fill, 而后者选择 128 位 移位器的移位量; 在本拍用 10
    // 位比较现算会串在移位器之前
    static ap_uint<1> beat_zero = 1;
    static ap_uint<1> eos_reg   = 0;

    static ap_uint<1> last_block;
    // 固定块: 只用于 symbols 表项选择 (288 项整块写入会让 HLS 调度时间
    // 爆炸, 常数表项用 static_litlen_entry 直接选). 解码用的 limit/base/
    // lit_limit 已在 STATE_LOAD_STATIC_TABLES 预载, 解码路径上没有该选择
    static ap_uint<1> static_block;
    // [257, 286]
    static ap_uint<9> hlit;
    // [1, 32]
    static ap_uint<6> hdist;
    // hlit - 1 / hdist - 1, 建表时与自增前的 sidx 比较, [256, 285] / [0, 31]
    static ap_uint<9> hlit_last;
    static ap_uint<5> hdist_last;
    // [4, 19]
    static ap_uint<5> hclen;
    // hclen * 3, [12, 57]
    static ap_uint<6> hclen3;

    // limit = next_code + bl_count, base = offsets - next_code (按下标位宽取模)
    // 解码时 next_code <= code < limit 即命中, symbol 下标为 base + code.
    // next_code[l] = limit[l - 1] << 1, 所以 code_l >= next_code[l] 等价于
    // code_(l-1) >= limit[l - 1], 下界由上一路的上界比较得到, 不需要存
    // next_code
    static ap_uint<5> codelen_table_bl_count[16];
    // codelen 码长最大 7, 合法码表下 limit[l] <= 2^l <= 128, 8 位即可
    // (SYMBOL_OFFSET 仍写 l = 8..15, 截断后不会被解码使用)
    static ap_uint<8> codelen_table_limit[16];
    static ap_uint<5> codelen_table_offsets[16];
    static ap_uint<5> codelen_table_base[16];
    static ap_uint<5> codelen_table_symbols[19];
    // codelen_table_symbols 中 symbol < 16 (码长本身) 的 one-hot, 16/17/18 为
    // 0. 码长写口按项累加 bl_count 时直接用, 不用对查表结果再做 4-16 译码
    static ap_uint<16> codelen_table_oh[19];

    static ap_uint<9>  litlen_table_bl_count[16];
    static ap_uint<16> litlen_table_limit[16];
    static ap_uint<9>  litlen_table_offsets[16];
    static ap_uint<9>  litlen_table_base[16];
    // 表项: literal (symbol < 256) 为字节值; length 码 (symbol >= 257) 为
    // {length_extra(3), length_base(9)}, EOB 为 0. FILL 查一次表就得到输出字节
    // 或 copy_len/extra 位数, 不再经过 length_base/length_extra 的第二级查表
    static ap_uint<12> litlen_table_symbols[288];
    // 上一拍 litlen_idx 对应表项的 length_base, LEN 使用
    static ap_uint<9> len_base;
    // 每个码长中 literal (symbol < 256) 的个数, 及 lit_limit = next_code +
    // lit_count. 同一码长内 symbol 按值递增分配码字, literal 排在最前, 所以
    // 解码时 code < lit_limit[l] 即为 literal, 不需要等查表结果
    static ap_uint<9>  litlen_table_lit_count[16];
    static ap_uint<16> litlen_table_lit_limit[16];
    // DECODE_SYMBOL 得到的码字是否 literal, FILL 用它决定是否输出,
    // 不依赖本拍查表得到的 symbol
    static ap_uint<1> is_literal;
    // FILL 中还没有解出码字 (刚进入 FILL), 本拍只解码, 不输出/不移位
    static ap_uint<1> pend_none;
    // 动态块中 symbol 256 的码长, 写码长时记录 (合法流中必不为 0)
    static ap_uint<4> litlen_eob_len;
    // 动态块中 symbol 256 在 litlen_table_symbols 中的下标. 256 在其码长内
    // 紧跟 literal 之后, 即 offsets[eob_len] + lit_count[eob_len], 建表时
    // 算好. 固定块中为 0. FILL 用 litlen_idx 与它比较判断 EOB, 不在 15 路
    // 解码中各做一次 16 位相等比较
    static ap_uint<9> litlen_eob_idx;

    static ap_uint<6>  dist_table_bl_count[16];
    static ap_uint<16> dist_table_limit[16];
    static ap_uint<6>  dist_table_offsets[16];
    static ap_uint<5>  dist_table_base[16];
    // 按解码下标 (canonical position) 直接索引的打包表, DIST 一次 32:1 读
    // 得到全部 dist 信息, 不再先查 symbol 再查 base/extra:
    //   [14:0]  base, [29:15] base_m4, [42:30] mask, [46:43] extra,
    //   [47]    near (symbol <= 9)
    static ap_uint<48> dist_table_entry[32];

    static ap_uint<9> huffman_tmp[16];
    // 不加 ARRAY_PARTITION 时数组实现为 RAM (只有全部访问都是常数下标时才会被
    // 自动分区), 每拍多次访问会把 II 限制为 8. 以下各表放寄存器:
    // bl_count/lit_count/huffman_tmp 每拍有动态下标的读改写,
    // limit/lit_limit/base/offsets 解码时 15 路并行读,
    // symbols 若用 BRAM, 读延迟 1 拍, 解出的 symbol 下一拍就决定状态跳转, II =
    // 2
#pragma HLS ARRAY_PARTITION variable = codelen_table_bl_count complete
#pragma HLS ARRAY_PARTITION variable = codelen_table_limit complete
#pragma HLS ARRAY_PARTITION variable = codelen_table_offsets complete
#pragma HLS ARRAY_PARTITION variable = codelen_table_base complete
#pragma HLS ARRAY_PARTITION variable = codelen_table_symbols complete
#pragma HLS ARRAY_PARTITION variable = codelen_table_oh complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_bl_count complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_limit complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_offsets complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_base complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_symbols complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_lit_count complete
#pragma HLS ARRAY_PARTITION variable = litlen_table_lit_limit complete
#pragma HLS ARRAY_PARTITION variable = dist_table_bl_count complete
#pragma HLS ARRAY_PARTITION variable = dist_table_limit complete
#pragma HLS ARRAY_PARTITION variable = dist_table_offsets complete
#pragma HLS ARRAY_PARTITION variable = dist_table_base complete
#pragma HLS ARRAY_PARTITION variable = dist_table_entry complete
#pragma HLS ARRAY_PARTITION variable = huffman_tmp complete

    // 填 symbols 的两级流水 (三张表共用, 见 switch 之后):
    //   A (*_FILL_SYMBOLS 状态内): 读码长, 算表项, 写入 fs_*
    //   B (下一拍, switch 之后): pos = huffman_tmp[fs_len]++, symbols[pos] =
    //   fs_entry
    // 若在一拍内完成 (码长 -> huffman_tmp -> 288 路写译码), HLS 会把写
    // symbols 排到流水线后级, 再前递给第一级的解码查表, 前递 mux 把建表和
    // 解码串成一条长路径. B 级在状态跳走后的下一拍完成, 最早的读在其后:
    // CODELEN 为 FILL_SYMBOLS -> DECODE_TABLE -> DECODE_TABLE_FILL (读)
    static ap_uint<1> fs_valid = 0;
    // 0: codelen, 1: litlen, 2: dist
    static ap_uint<2>  fs_tbl;
    static ap_uint<4>  fs_len;
    static ap_uint<12> fs_entry;

    // [0, hlit + hdist]
    static ap_uint<9> lidx;
    // lidx == lidx_last, 即本拍写入的是最后一个 code length. 在码长写口
    // 与 lidx 同步算好 (那里的结果只喂寄存器), 状态跳转直接读, lidx 的 9 位
    // 比较就不在状态写 mux 之前了
    static ap_uint<1> at_last;
    // 最后一个 code length 的下标 hlit + hdist - 1, 在 HEADER 预先算好,
    // 写长度时用自增前的 lidx 比较, 不在关键路径上串两次加法, [257, 317]
    static ap_uint<9> lidx_last;
    // 上一个写入的 code length, 给 symbol 16 用
    static ap_uint<4> prev_len;
    // symbol 16/17/18 剩余的重复次数及重复值, 每拍写一个, [0, 138]
    static ap_uint<8> rep_cnt;
    static ap_uint<4> rep_val;
    // build table 时的 symbol 下标, [0, 286]
    static ap_uint<9> sidx;
    // SYMBOL_OFFSET 每拍处理一个 code length, 跨拍保存累加值
    // [1, 15]
    static ap_uint<4>  acc_l;
    static ap_uint<16> acc_code;
    // [0, 288]
    static ap_uint<9> acc_offset;

    static ap_uint<3> code_lengths[19];
    // CAL_COUNT 并行读全部 19 项, FILL_SYMBOLS 动态下标读, 放寄存器
#pragma HLS ARRAY_PARTITION variable = code_lengths complete
    static ap_uint<4> litlen_lengths[288];
    static ap_uint<4> dist_lengths[32];
    // litlen_lengths 放 LUTRAM: FILL_SYMBOLS 的 A 级读出码长, 读延迟 1 拍,
    // 下一拍 B 级才用 (fs_*), 读出的码长直接进 B 级, 依赖它的写仍在第一级.
    // 放寄存器时 288 选 1 的读 mux 约 1.5K LUT. dist_lengths 只有 32 项,
    // 放寄存器
#pragma HLS BIND_STORAGE variable = litlen_lengths type = ram_s2p impl = lutram
#pragma HLS ARRAY_PARTITION variable = dist_lengths complete

    // litlen 解码得到的 symbol 表下标, 288 选 1 的查表放到 DECOMPRESS_FILL,
    // 不和解码的比较/归约串在同一拍, [0, 287]
    static ap_uint<9> litlen_idx;
    // 同上, codelen 的查表放到 DECODE_TABLE_FILL, [0, 18]
    static ap_uint<5> codelen_idx;
    // dist 解码得到的 canonical position, DIST 用它在打包表里一次读出
    // base/mask/extra/near, [0, 31]
    static ap_uint<5> dist_idx;
    // 解码得到的码字长度, 由 *_FILL 状态从 bitbuf 中移去, [1, 15]
    static ap_uint<4> dec_len;
    // 下一拍可变移位器的移位量, 在跳转到对应状态时写入,
    // 使移位器输入直接来自寄存器
    static ap_uint<6> sh_reg = 3;
    // sh_reg 的副本: 窗口选择 (两组 mux) 和 128 位移位器各用一个, 避免
    // 一个寄存器同时驱动这两大块 (PnR 上 fanout ~160). 每拍在 switch 后
    // 统一复制新值
    static ap_uint<6> sh_w15 = 3;
    static ap_uint<6> sh_w89 = 3;
    static ap_uint<6> sh_buf = 3;

    // length/dist 解码得到的复制长度和距离. stored 块也用 copy_len 计剩余字节
    // length [3, 258], stored [0, 65535]
    static ap_uint<16> copy_len;
    // stored 块剩余字节数为 0, 由写 copy_len 的地方同步更新. 用它代替
    // copy_len != 0 的 16 位比较, 该比较会排在 do_shift -> 移位器之前
    static ap_uint<1> copy_zero;
    // dist_base 的低 6 位, near 时 COPY_START 加上 extra 得到距离 [1, 32]
    static ap_uint<6> copy_dist;
    // DIST 中预算: 源起始地址 fe_pos - base 及 fe_pos + 4 - base, extra bits
    // 的 mask, 是否 near (dist <= 32, 即 dist symbol <= 9)
    static ap_uint<15> copy_rbase;
    static ap_uint<15> copy_rbase4;
    static ap_uint<13> copy_mask;
    static ap_uint<1>  copy_near;
    // 前端已推出 token 的总字节数 (模 32K), 即该 token 在后端开始时的
    // bufoff. 用它在前端算 match 的源地址, 后端不用再做减法
    static ap_uint<15> fe_pos = 0;

    // 前端 -> 后端的 token 队列, 深度 4, 放寄存器. 前端每拍至多推 1 个,
    // 后端每拍至多取 1 个. tq_count[2] 即队列满 (count == 4)
    static ap_uint<1> tq_match[4];
    // literal: 字节值; match: 长度 [3, 258]
    static ap_uint<9> tq_len[4];
    // literal token 的第二字节及其字节数 (1 或 2); match token 不用
    static ap_uint<8> tq_b1[4];
    static ap_uint<3> tq_nlit[4];
    static ap_uint<1> tq_near[4];
    // near 时的距离 [1, 32] (按 6 位取模, 32 为 0)
    static ap_uint<6> tq_dist[4];
    // far 时的源起始地址及其 + 4
    static ap_uint<15> tq_raddr[4];
    static ap_uint<15> tq_raddr4[4];
#pragma HLS ARRAY_PARTITION variable = tq_match complete
#pragma HLS ARRAY_PARTITION variable = tq_len complete
#pragma HLS ARRAY_PARTITION variable = tq_b1 complete
#pragma HLS ARRAY_PARTITION variable = tq_nlit complete
#pragma HLS ARRAY_PARTITION variable = tq_near complete
#pragma HLS ARRAY_PARTITION variable = tq_dist complete
#pragma HLS ARRAY_PARTITION variable = tq_raddr complete
#pragma HLS ARRAY_PARTITION variable = tq_raddr4 complete
    static ap_uint<2> tq_head  = 0;
    static ap_uint<2> tq_tail  = 0;
    static ap_uint<3> tq_count = 0;
    // tq_count != 0 的寄存器副本, 与 tq_count 同步写入. 后端是否取 token
    // 决定本拍输出几个字节, 3 位比较会串在输出拍的写位置之前
    static ap_uint<1> tq_nempty = 0;

    // 后端: 每拍输出 n 个字节 (literal 1, match 每拍至多 4)
    // 正在拷贝 match, 剩余字节数, 本拍输出数 min(4, be_len), 本拍是最后一拍
    static ap_uint<1> be_busy = 0;
    static ap_uint<9> be_len;
    static ap_uint<3> be_n;
    static ap_uint<1> be_end;
    // near: 第 j 个输出字节在 hist 中的位置 (0 为最近输出的字节)
    static ap_uint<1> be_near;
    static ap_uint<5> be_idx[4];
#pragma HLS ARRAY_PARTITION variable = be_idx complete
    // far: 源起始地址低 2 位, 下一次读各 bank 的字地址 (起始地址及 + 4)
    static ap_uint<2>  be_rot;
    static ap_uint<13> be_w0;
    static ap_uint<13> be_w1;

    // 正在拼装的输出拍, outoff 为已写入的字节数, [0, 67]. 第 64..67 字节
    // 存放超出当前拍的字节, 下一拍写出当前拍后移到开头, 不需要回绕
    static ap_uint<544> out_data;
    static ap_uint<7>   outoff = 0;

    // LZ77 窗口, 按地址模 4 交错放 4 个 bank, 每拍读写连续 4 字节. 一读一写
    static ap_uint<8> window[4][8192];
#pragma HLS ARRAY_PARTITION variable = window dim = 1 complete
#pragma HLS BIND_STORAGE variable = window type = ram_s2p impl = bram
    // dist <= 32 时从 hist 取, 读 window 的 match 距离 >= 33, 读出的字节至少在
    // 7 拍之前写入, 不存在需要 HLS 保证的跨迭代 RAW
#pragma HLS DEPENDENCE variable = window inter false
    // 下一个写入的字节地址, 及其字地址 + 1
    static ap_uint<15> bufoff = 0;
    static ap_uint<13> buf_w1 = 1;
    // 各 bank 上一拍读出的字节
    static ap_uint<8> be_rd[4];
#pragma HLS ARRAY_PARTITION variable = be_rd complete
    // 最近输出的 32 个字节, hist(7, 0) 为最近一个. near 拷贝的源字节可能
    // 是上一拍刚输出的, 直接从寄存器取
    static ap_uint<256> hist;

    // fifo bram has same latency with fifo lutram, but it is bigger
    static ap_uint<16> lookup[512];
#pragma HLS BIND_STORAGE variable = lookup type = fifo impl = bram

    nsprintf("length: %u\n", length.to_uint());

    switch (work_state) {
    case WORK_STATE_IDLE: {
        if (!notify_in.empty()) {
            axi_bit_t n = notify_in.read();
            if (n.data) {
                // 开始一条流: 清掉上一条流残留的填充位
                work_state = WORK_STATE_BUSY;
                last       = 0;
                length     = 0;
                bitbuf     = 0;
                beat_bits  = 0;
                beat_zero  = 1;
                eos_reg    = 0;
                in_valid   = 0;
                in_drain   = 0;
            } else {
                state     = STATE_BLOCK;
                sh_reg    = 3;
                sh_w15    = 3;
                sh_w89    = 3;
                sh_buf    = 3;
                outoff    = 0;
                bufoff    = 0;
                buf_w1    = 1;
                fe_pos    = 0;
                tq_head   = 0;
                tq_tail   = 0;
                tq_count  = 0;
                tq_nempty = 0;
                be_busy   = 0;
                length    = 0;
                bitbuf    = 0;
                beat_bits = 0;
                beat_zero = 1;
                eos_reg   = 0;
                in_valid  = 0;
                in_drain  = 0;
                notify_out.write(n);
            }
        }
    } break;
    case WORK_STATE_BUSY: {
        // 所有状态共用一个可变移位器, 在 switch 之后统一移位.
        const ap_uint<128> cur_bitbuf    = bitbuf;
        const ap_uint<7>   cur_length    = length;
        const ap_uint<10>  cur_beat_bits = beat_bits;
        const ap_uint<1>   cur_last      = last;
        // 本拍开始时的 state; switch 之后 state 已被写成下一状态
        const ap_uint<6> cur_state = state;
        // 移位量 sh_reg 在跳转到本状态时已写好, 各分支只给出 do_shift
        const ap_uint<6> sh = sh_buf;
        // 输入已全部进入 bitbuf. 此时 length 可能小于解码所需的位数
        // (最后一个码字 + 填充), 解码状态不再 poll, 直接用剩余的位解
        const ap_uint<1> cur_beat_zero = beat_zero;
        const ap_uint<1> eos           = eos_reg;
        const ap_uint<1> lack          = cur_length < 48 && !eos;
        // token 队列满: 推 token 的状态 (FILL/COPY_START/STORED_COPY) 整拍
        // 停住. 只看寄存器 tq_count 的最高位
        const ap_uint<1> tq_full = tq_count[2];
        // FILL 中上一拍解出的码字是否 EOB (见 litlen_eob_idx)
        const ap_uint<1> is_eob = litlen_idx == litlen_eob_idx;
        // 本拍是否移位, 只由本拍开始时的寄存器决定, 不经过分支内的数据 (如查表
        // 得到的 symbol). 写成在各分支内 do_shift = 1 时, HLS
        // 把它合并成一个跨分支 的 phi, 选择条件带上查表结果,
        // 查表就串到了移位器前面
        ap_uint<1> do_shift = 0;
        switch (state) {
        case STATE_BLOCK:
            do_shift = length >= 3;
            break;
        case STATE_DYNAMIC_HUFFMAN_HEADER:
            do_shift = length >= 14;
            break;
        case STATE_DYNAMIC_HUFFMAN_CODE_LENS:
            do_shift = length >= hclen3;
            break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_FILL_LENGTHS:
            do_shift = sh != 0;
            break;
        case STATE_DECOMPRESS_FILL:
        case STATE_DECOMPRESS_COPY_START:
            do_shift = !lack && !tq_full;
            break;
        case STATE_DYNAMIC_HUFFMAN_DECODE_TABLE_FILL:
        case STATE_DECOMPRESS_LEN:
        case STATE_DECOMPRESS_DIST:
        case STATE_STORED_ALIGN:
            do_shift = 1;
            break;
        case STATE_STORED_HEADER:
            do_shift = length >= 32;
            break;
        case STATE_STORED_COPY:
            do_shift = !copy_zero && length >= 8 && !tq_full;
            break;
        default:
            break;
        }
        // 必须补位才能继续 (位数不够, 本拍什么都不做), 与各状态分支内 "补位"
        // 注释处 的条件相同. 输入已读完时据此判断流被截断
        ap_uint<1> need_poll = 0;
        switch (state) {
        case STATE_BLOCK:
            need_poll = length < 3;
            break;
        case STATE_DYNAMIC_HUFFMAN_HEADER:
            need_poll = length < 14;
            break;
        case STATE_DYNAMIC_HUFFMAN_CODE_LENS:
            need_poll = length < hclen3;
            break;
        case STATE_DYNAMIC_HUFFMAN_DECODE_TABLE:
            need_poll = length < 14 && !eos;
            break;
        case STATE_DECOMPRESS_FILL:
        case STATE_DECOMPRESS_COPY_START:
            need_poll = lack;
            break;
        case STATE_STORED_HEADER:
            need_poll = length < 32;
            break;
        case STATE_STORED_COPY:
            need_poll = !copy_zero && length < 8;
            break;
        default:
            break;
        }
        const ap_uint<1> do_poll = !cur_length[6] && !eos;
        // DRAIN 丢弃输入, 与补位共用唯一的 data_in.read
        const ap_uint<1> do_drain = in_drain;
        // 推 token 的使能, 只由本拍开始时的寄存器决定 (与 do_shift 相同),
        // 不经过分支内的数据: FILL 推上一拍解出的 literal, COPY_START 推
        // match, STORED_COPY 推 bitbuf 低 8 位
        const ap_uint<1> st_fill   = cur_state == STATE_DECOMPRESS_FILL;
        const ap_uint<1> st_cstart = cur_state == STATE_DECOMPRESS_COPY_START;
        const ap_uint<1> st_stored = cur_state == STATE_STORED_COPY;
        const ap_uint<1> fe_push =
            (st_fill && !lack && !tq_full && !pend_none && is_literal) ||
            (st_cstart && !lack && !tq_full) ||
            (st_stored && !copy_zero && length >= 8 && !tq_full);
        // litlen 表项: literal 为字节值, length 码为 {extra(3), base(9)}.
        // 解码用的 limit/base/lit_limit 在固定块已预载, 只有 symbols 表项
        // 按 static_block 在两路 288:1 读之间选一次 (不在解码递归路径上)
        const ap_uint<12> dyn_entry =
            static_block ? static_litlen_entry[litlen_idx]
                         : litlen_table_symbols[litlen_idx];
        const ap_uint<8>  fill_byte = ap_uint<8>(dyn_entry(7, 0));
        // FILL 遇到 length 码时的 extra 位数
        const ap_uint<3> ll_ext = ap_uint<3>(dyn_entry(11, 9));
        // length_base 每拍无条件记入 len_base, LEN 用上一拍 (FILL) 的值:
        // FILL 到 LEN 之间 litlen_idx 不变. 不在 LEN 中现查, 288 选 1 就不
        // 串在长度加法之前
        const ap_uint<9> cur_len_base = len_base;
        len_base                      = ap_uint<9>(dyn_entry(8, 0));
        // 解码窗口候选. FILL 双码推测时 sh 最多 24 (F + lenZ, F=8/9);
        // LEN 最多 5, COPY_START 最多 13. 两组 32 候选:
        //   win15[c]  = 偏移 c 的 15 位窗口 (标量表 & LEN/COPY_START)
        //   win89[c]  = 偏移 c+8 的 16 位窗口, 低 15 位是偏移 c+8 的窗口,
        //               高 15 位是偏移 c+9 的窗口 (F=8/9 配对)
        // 每组都是 32:1, 比 64 选 1 少一级; 也不用 sh+8 的加法
        ap_uint<15> win15[32];
        ap_uint<16> win89[32];
#pragma HLS ARRAY_PARTITION variable = win15 complete
#pragma HLS ARRAY_PARTITION variable = win89 complete
    loop_win_cand:
        for (ap_uint<6> c = 0; c < 32; ++c) {
#pragma HLS UNROLL
            win15[c] = cur_bitbuf(c + 14, c);
            win89[c] = cur_bitbuf(c + 23, c + 8);
        }
        const ap_uint<15> win   = win15[sh_w15(4, 0)];
        const ap_uint<16> w89   = win89[sh_w89(4, 0)];
        const ap_uint<15> win8  = ap_uint<15>(w89(14, 0));
        const ap_uint<15> win9  = ap_uint<15>(w89(15, 1));
        // 窗口只反转一次, 第 l 路码字是 rev 的常数切片
        const ap_uint<15> rev = internal::bitswap15(win);
        // COPY_START: 移去 dist extra bits, 源地址 = (fe_pos - base) - extra
        const ap_uint<13> dext     = cur_bitbuf(12, 0) & copy_mask;
        const ap_uint<15> m_raddr  = copy_rbase - dext;
        const ap_uint<15> m_raddr4 = copy_rbase4 - dext;
        const ap_uint<6>  m_dist   = copy_dist + dext(5, 0);
        // 本拍开始时的 match 长度和 near (copy_len 在 switch 之后、copy_near
        // 在 DIST 中会被改写, 读写串在一起会把改写的运算带到推 token 一侧)
        const ap_uint<9> m_len  = copy_len(8, 0);
        const ap_uint<1> m_near = copy_near;
        // LEN: match 长度 = length_base (FILL 解出的 litlen_idx 未改变, 查表)
        // + extra bits (sh_reg 位, [0, 5]). 在 switch 之前无条件算, 由 switch
        // 之后的 copy_len 写口按 cur_state 选择; 在 switch 分支里赋值时, 分支
        // 合流处的 phi 再加一级 mux. extra 的 mask 按常数表取
        const ap_uint<5> len_mask[8] = {0, 1, 3, 7, 15, 31, 0, 0};
#pragma HLS ARRAY_PARTITION variable = len_mask complete
        const ap_uint<16> len_val =
            ap_uint<16>(cur_len_base) + (cur_bitbuf(4, 0) & len_mask[sh(2, 0)]);
        // 推出的 token
        const ap_uint<9> push_len = st_cstart   ? m_len
                                    : st_stored ? ap_uint<9>(cur_bitbuf(7, 0))
                                                : ap_uint<9>(fill_byte);

        // 后端本拍: 拷贝中则输出 match 的下一组字节, 否则取一个 token
        // (literal 本拍输出, match 本拍只做准备并发出第一次读)
        const ap_uint<1>  tq_empty  = !tq_nempty;
        const ap_uint<1>  pop       = !be_busy && !tq_empty;
        const ap_uint<1>  h_match   = tq_match[tq_head];
        const ap_uint<9>  h_len     = tq_len[tq_head];
        const ap_uint<8>  h_b1      = tq_b1[tq_head];
        const ap_uint<3>  h_nlit    = tq_nlit[tq_head];
        const ap_uint<1>  h_near    = tq_near[tq_head];
        const ap_uint<6>  h_dist    = tq_dist[tq_head];
        const ap_uint<15> h_raddr   = tq_raddr[tq_head];
        const ap_uint<15> h_raddr4  = tq_raddr4[tq_head];
        const ap_uint<1>  pop_lit   = pop && !h_match;
        const ap_uint<1>  pop_match = pop && h_match;
        // 本拍输出的字节数及字节 (literal token 1-2 字节, match 每拍至多 4)
        const ap_uint<3> out_n =
            be_busy ? be_n : (pop_lit ? ap_uint<3>(h_nlit) : ap_uint<3>(0));
        ap_uint<8> out_b[4];
#pragma HLS ARRAY_PARTITION variable = out_b complete
    loop_out_bytes:
        for (ap_uint<3> j = 0; j < 4; ++j) {
#pragma HLS UNROLL
            const ap_uint<5> hi = be_idx[j];
            const ap_uint<8> hb =
                hist(ap_uint<8>(hi) * 8 + 7, ap_uint<8>(hi) * 8);
            const ap_uint<8> fb = be_rd[ap_uint<2>(be_rot + j)];
            const ap_uint<8> lb = j == 0   ? ap_uint<8>(h_len(7, 0))
                                  : j == 1 ? h_b1
                                           : ap_uint<8>(0);
            out_b[j]           = !be_busy ? lb : be_near ? hb : fb;
        }
        // 后端空闲 (队列空且不在拷贝), DONE 在此之后才输出最后一拍
        // outoff > 64 时本拍要先写出满的一拍, DONE 的最后一拍等下一拍再写
        const ap_uint<1> be_idle =
            !be_busy && tq_empty && !(outoff[6] && outoff(5, 0) != 0);
        // 只由寄存器决定
        const ap_uint<1> do_flush = cur_state == STATE_DONE && be_idle;
        // 填 symbols 流水的 A 级输出, 只由 *_FILL_SYMBOLS 给出, 在 switch
        // 之后写入 fs_*
        ap_uint<1>  fill_valid = 0;
        ap_uint<2>  fill_tbl   = 0;
        ap_uint<4>  fill_len   = 0;
        ap_uint<12> fill_entry = 0;
        // B 级读本拍开始时的 huffman_tmp. 放在 switch 之后读, 会读到
        // *_FILL_POS 在 switch 内刚写的值, 把两者串成一条组合路径
        const ap_uint<9> fs_pos = huffman_tmp[fs_len];
        // 码长写口 (switch 之后) 的使能和值, 只由 cur_state 的比较和寄存器
        // 查表决定. 在 switch 的分支里赋值时, HLS 在 switch 之后为它们
        // 合成跨分支的 phi, 状态译码就串在 bl_count 的累加之前
        const ap_uint<5>  cl_sym = codelen_table_symbols[codelen_idx];
        const ap_uint<16> cl_oh  = codelen_table_oh[codelen_idx];
        const ap_uint<1>  st_dtf =
            cur_state == STATE_DYNAMIC_HUFFMAN_DECODE_TABLE_FILL;
        const ap_uint<1> st_fl =
            cur_state == STATE_DYNAMIC_HUFFMAN_BUILD_FILL_LENGTHS;
        const ap_uint<1>  wl_en  = (st_dtf && !cl_sym[4]) || st_fl;
        const ap_uint<4>  wl_val = st_fl ? rep_val : ap_uint<4>(cl_sym(3, 0));
        const ap_uint<16> wl_oh =
            st_fl ? ap_uint<16>(ap_uint<16>(1) << rep_val) : cl_oh;

        // litlen / dist 解码器, 在 switch 之前算, FILL/LEN/COPY_START 共用.
        // 解码窗口 = bitbuf 跳过 sh_reg 位之后的 15 位: FILL 中 sh_reg 是上一拍
        // 解出、本拍才移去的码字长度, 解码与移位并行, 每拍出一个 literal;
        // LEN 中 sh_reg 是本拍移去的 length extra bits; COPY_START 中为
        // dist 码字 + extra bits (DIST 合并后, 最多 28).

        // 16 个候选码字 (rev 的常数切片, l 位码字只含低 l 位), 按码长索引
        ap_uint<15> code_l[16];
#pragma HLS ARRAY_PARTITION variable = code_l complete
    loop_rev_slices:
        for (ap_uint<5> l = 0; l <= 15; ++l) {
#pragma HLS UNROLL
            code_l[l] = ap_uint<15>(rev >> (15 - l));
        }

        // litlen: 由温度计码得到码长, 再按码长取 base/lit_limit 和码字.
        // 不再做 15 路 hit/lane_idx 归约: 命中路的索引就是 base[len] + code
        ap_uint<4> ll_len;
        ap_uint<9> ll_idx = 0;
        ap_uint<1> ll_lit;
        {
            ap_uint<16> lt  = 0; // lt[0] = 0
            ap_uint<15> hit = 0, lit = 0;
            // 固定块已预载入同一组寄存器表. base + code 与比较并行算,
            // 命中路的 lane_idx 按位 OR 归约; literal 判断同样并行后用
            // hit 归约. 不用 "按码长选 base/lit_limit 再比较/相加": 实测
            // 多串一级 mux + 加法/比较, PnR 反而更慢
            ap_uint<9> lane_idx[15];
        loop_litlen_decode:
            for (ap_uint<5> l = 1; l <= 15; ++l) {
#pragma HLS UNROLL
                lt[l]           = code_l[l] < litlen_table_limit[l];
                hit[l - 1]      = lt[l] && !lt[l - 1];
                lit[l - 1]      = code_l[l] < litlen_table_lit_limit[l];
                lane_idx[l - 1] = ap_uint<9>(litlen_table_base[l] + code_l[l]);
            }
            // 该码长的 literal 标志按码长从 lit 中选, 不做 hit & lit 的
            // 15 位归约: 后者串在 compare -> hit 之后, 比 therm_len 选
            // lit 慢, 且在 sh_reg 的关键路径上 (use_pair 依赖 lit0)
            ap_uint<16> lit_vec = 0;
            for (ap_uint<5> b = 1; b <= 15; ++b) {
#pragma HLS UNROLL
                lit_vec[b] = lit[b - 1];
            }
            ll_len = internal::therm_len(lt);
            ll_lit = lit_vec[ll_len];
        loop_litlen_decode_idx:
            for (ap_uint<4> b = 0; b < 9; ++b) {
#pragma HLS UNROLL
                ap_uint<15> v = 0;
                for (ap_uint<5> l = 0; l < 15; ++l) {
#pragma HLS UNROLL
                    v[l] = lane_idx[l][b];
                }
                ll_idx[b] = (v & hit).or_reduce();
            }
        }
        // 固定偏移推测: 只在首码长度恰为 8 或 9 时配对 (def late 常见
        // literal 码长; 固定块 literal 也全在 8/9). 两个候选并行解码,
        // 选中后的总长 F + lenZ 预先算好, sh_reg 只多一级 2:1 选择
        ap_uint<4> z_len[2];
        ap_uint<9> z_idx[2];
        ap_uint<1> z_lit[2];
        ap_uint<15> z_rev[2];
#pragma HLS ARRAY_PARTITION variable = z_len complete
#pragma HLS ARRAY_PARTITION variable = z_idx complete
#pragma HLS ARRAY_PARTITION variable = z_lit complete
#pragma HLS ARRAY_PARTITION variable = z_rev complete
        z_rev[0] = internal::bitswap15(win8);
        z_rev[1] = internal::bitswap15(win9);
    loop_z_cand:
        for (ap_uint<2> k = 0; k < 2; ++k) {
#pragma HLS UNROLL
            const ap_uint<15> crev = z_rev[k];
            ap_uint<16> lt  = 0; // lt[0] = 0
            ap_uint<15> hit = 0, lit = 0;
            ap_uint<9> lane_idx[15];
        loop_z_decode:
            for (ap_uint<5> l = 1; l <= 15; ++l) {
#pragma HLS UNROLL
                const ap_uint<15> c = ap_uint<15>(crev >> (15 - l));
                lt[l]           = c < litlen_table_limit[l];
                hit[l - 1]      = lt[l] && !lt[l - 1];
                lit[l - 1]      = c < litlen_table_lit_limit[l];
                lane_idx[l - 1] = ap_uint<9>(litlen_table_base[l] + c);
            }
            z_len[k] = internal::therm_len(lt);
            // 与 ll_lit 相同: 按码长选 lit, 不做 hit & lit 归约
            ap_uint<16> lit_vec = 0;
            for (ap_uint<5> b = 1; b <= 15; ++b) {
#pragma HLS UNROLL
                lit_vec[b] = lit[b - 1];
            }
            z_lit[k] = lit_vec[z_len[k]];
            ap_uint<9> idx = 0;
        loop_z_decode_idx:
            for (ap_uint<4> b = 0; b < 9; ++b) {
#pragma HLS UNROLL
                ap_uint<15> v = 0;
                for (ap_uint<5> l = 0; l < 15; ++l) {
#pragma HLS UNROLL
                    v[l] = lane_idx[l][b];
                }
                idx[b] = (v & hit).or_reduce();
            }
            z_idx[k] = idx;
        }
        // 首码 Y 是 literal 且长度恰为 8/9 时, 偏移 F 处的码字就是下一个
        // 码字 Z; 配对消费 F + lenZ, Z 成为下一拍 pending.
        // 8/9 只差最低位, 用 ll_len[0] 选候选, 不做两次相等比较
        const ap_uint<1> len89 =
            ll_len[3] && !ll_len[2] && !ll_len[1];
        const ap_uint<1> sel9   = ll_len[0];
        const ap_uint<1> use_pair = ll_lit && len89;
        // F + len 用 16 项常数表, 不在关键路径上做加法
        const ap_uint<5> plus8[16] = {8,  9,  10, 11, 12, 13, 14, 15,
                                      16, 17, 18, 19, 20, 21, 22, 23};
        const ap_uint<5> plus9[16] = {9,  10, 11, 12, 13, 14, 15, 16,
                                      17, 18, 19, 20, 21, 22, 23, 24};
#pragma HLS ARRAY_PARTITION variable = plus8 complete
#pragma HLS ARRAY_PARTITION variable = plus9 complete
        const ap_uint<5> total8 = plus8[z_len[0]];
        const ap_uint<5> total9 = plus9[z_len[1]];
        // {use_pair, sel9} 一次选出 pair 总长或非 pair 的 len0:
        // use_pair=0 时两路都是 ll_len, 省掉后面的 select + phi
        ap_uint<5> sh_tab[4];
        ap_uint<2> sh_idx;
#pragma HLS ARRAY_PARTITION variable = sh_tab complete
        sh_tab[0] = ap_uint<5>(ll_len);
        sh_tab[1] = ap_uint<5>(ll_len);
        sh_tab[2] = total8;
        sh_tab[3] = total9;
        sh_idx[1] = use_pair;
        sh_idx[0] = sel9;
        const ap_uint<5> sh_pair = sh_tab[sh_idx];
        // 下一个 pending 的 litlen 下标同样一次 4:1 选出:
        // use_pair=0 时两路都是当前 Y 的下标
        ap_uint<9> idx_tab[4];
#pragma HLS ARRAY_PARTITION variable = idx_tab complete
        idx_tab[0]               = ll_idx;
        idx_tab[1]               = ll_idx;
        idx_tab[2]               = z_idx[0];
        idx_tab[3]               = z_idx[1];
        const ap_uint<9> litlen_next = idx_tab[sh_idx];
        // is_literal 同样一次 4:1 选出 (非 pair 时是 LL 的 Y)
        ap_uint<1> lit_tab[4];
#pragma HLS ARRAY_PARTITION variable = lit_tab complete
        lit_tab[0]               = ll_lit;
        lit_tab[1]               = ll_lit;
        lit_tab[2]               = z_lit[0];
        lit_tab[3]               = z_lit[1];
        const ap_uint<1> lit_next = lit_tab[sh_idx];
        // FILL 推 pair (X, Y): X 的字节来自 dyn_entry, Y 的字节本拍读表
        const ap_uint<1> fill_pair = st_fill && is_literal && use_pair;
        const ap_uint<12> y_entry =
            static_block ? static_litlen_entry[ll_idx]
                         : litlen_table_symbols[ll_idx];
        const ap_uint<8> push_b1 =
            fill_pair ? ap_uint<8>(y_entry(7, 0)) : ap_uint<8>(0);
        const ap_uint<3> push_nlit = fill_pair ? ap_uint<3>(2) : ap_uint<3>(1);
        // dist: 同上, 用 dist 表, 共用同一组 code_l
        ap_uint<4> dd_len;
        ap_uint<5> dd_idx = 0;
        {
            ap_uint<16> lt  = 0; // lt[0] = 0
            ap_uint<15> hit = 0;
            // base + code 与比较并行, 命中路的 lane_idx 按位 OR 归约:
            // 比 "按码长选 base/code 再相加" 少一级 mux + 加法 (后者是
            // 这一段的关键路径)
            ap_uint<5> lane_idx[15];
        loop_dist_decode:
            for (ap_uint<5> l = 1; l <= 15; ++l) {
#pragma HLS UNROLL
                lt[l]           = code_l[l] < dist_table_limit[l];
                hit[l - 1]      = lt[l] && !lt[l - 1];
                lane_idx[l - 1] = ap_uint<5>(dist_table_base[l] + code_l[l]);
            }
            dd_len = internal::therm_len(lt);
        loop_dist_decode_idx:
            for (ap_uint<3> b = 0; b < 5; ++b) {
#pragma HLS UNROLL
                ap_uint<15> v = 0;
                for (ap_uint<5> l = 0; l < 15; ++l) {
#pragma HLS UNROLL
                    v[l] = lane_idx[l][b];
                }
                dd_idx[b] = (v & hit).or_reduce();
            }
        }

        // 状态跳转写在局部 nstate 上, switch 之后只写一次 state. 直接在
        // switch 的二十多个分支里写 state, HLS 会把它们合成一条很长的写
        // 优先级链 (每级一个 5 位 mux), 排在最后写口的使能之前
        // switch 内会写、switch 之后要读的静态变量, 先取本拍开始时的值.
        // switch 之后读会读到 switch 内刚写的值, 把写入的运算 (如 HEADER
        // 的加法) 串到读的一侧. 这些变量都在较早的状态写入, 较晚的状态
        // 使用, 不需要同拍的新值
        const ap_uint<9> cur_hlit      = hlit;
        const ap_uint<9> cur_lidx_last = lidx_last;
        ap_uint<5>       cur_codelen_offsets[16];
        ap_uint<9>       cur_litlen_offsets[16];
        ap_uint<6>       cur_dist_offsets[16];
    loop_cur_offsets:
        for (ap_uint<5> i = 0; i < 16; ++i) {
#pragma HLS UNROLL
            cur_codelen_offsets[i] = codelen_table_offsets[i];
            cur_litlen_offsets[i]  = litlen_table_offsets[i];
            cur_dist_offsets[i]    = dist_table_offsets[i];
        }
        ap_uint<6> nstate = cur_state;
        switch (state) {
        case STATE_BLOCK: {
            if (length < 3) {
                // 补位: do_poll 在 switch 前按相同条件算好
            } else {
                last_block = bitbuf(0, 0);
                nsprintf("last block: %u\n", last_block.to_uchar());
                ap_uint<2> type = bitbuf(2, 1);
                switch (type) {
                case BLOCK_TYPE_UNCOMPRESSED: {
                    nsprintf("uncompressed\n");
                    // 输入按字节进入 bitbuf, length 的低 3 位就是到字节边界前
                    // 剩余的位数. 移去块头 3 位后再移去这些位对齐
                    const ap_uint<7> rest = length - 3;
                    nstate                = STATE_STORED_ALIGN;
                    sh_reg                = rest(2, 0);
                } break;
                case BLOCK_TYPE_STATIC_HUFFMAN: {
                    nsprintf("static huffman\n");
                    // 固定码表 (RFC 1951 3.2.6) 已预先算好
                    // 不建表, 下一拍把 internal::static_* 常数整块写入
                    // litlen/dist 寄存器表, 之后解码只有一套表
                    nstate = STATE_LOAD_STATIC_TABLES;
                    sh_reg = 0;
                } break;
                case BLOCK_TYPE_DYNAMIC_HUFFMAN:
                    nsprintf("dynamic huffman\n");
                    static_block = 0;
                    nstate       = STATE_DYNAMIC_HUFFMAN_HEADER;
                    sh_reg       = 14;
                    break;
                default: // 非法块类型, 结束本条流
                    nstate = STATE_DONE;
                    break;
                }
            }
        } break;
        case STATE_LOAD_STATIC_TABLES: {
            nsprintf("load_static_tables\n");
            // 固定块: 解码用的 16 项 limit/base/lit_limit 常数表整块写入
            // 与动态块相同的寄存器表, 解码路径上不再有 static_block 选择.
            // symbols 表项仍由 static_litlen_entry 直接选择 (整块写 288 项
            // 会让 HLS 调度时间从 ~30s 爆炸到 ~9min). bl_count/lit_count/
            // offsets 不写 (固定块不建表)
        loop_static_litlen_tbl:
            for (ap_uint<5> l = 1; l <= 15; ++l) {
#pragma HLS UNROLL
                litlen_table_limit[l] =
                    internal::static_litlen_next_code[l] +
                    internal::static_litlen_bl_count[l];
                litlen_table_base[l] = internal::static_litlen_offsets[l] -
                                       internal::static_litlen_next_code[l];
                litlen_table_lit_limit[l] =
                    internal::static_litlen_lit_limit[l];
            }
        loop_static_dist_tbl:
            for (ap_uint<5> l = 1; l <= 15; ++l) {
#pragma HLS UNROLL
                dist_table_limit[l] = internal::static_dist_next_code[l] +
                                      internal::static_dist_bl_count[l];
                dist_table_base[l] = internal::static_dist_offsets[l] -
                                     internal::static_dist_next_code[l];
            }
        loop_static_dist_sym:
            for (ap_uint<5> i = 0; i < 30; ++i) {
#pragma HLS UNROLL
                // 固定块 dist 码全部为 5 位
                ap_uint<48> e = 0;
                e(14, 0)      = dist_base[i](14, 0);
                e(29, 15)     = dist_base_m4[i];
                e(42, 30)     = dist_mask[i];
                e(46, 43)     = dist_extra[i];
                e[47]         = i <= 9;
                dist_table_entry[i] = e;
            }
            static_block = 1;
            // 固定块中 EOB (symbol 256) 在下标 0
            litlen_eob_len = 7;
            litlen_eob_idx = 0;
            LITLEN_START;
        } break;
        case STATE_DYNAMIC_HUFFMAN_HEADER: // needs 14 bits
        {
            if (length < 14) {
                // 补位: do_poll 在 switch 前按相同条件算好
            } else {
                const ap_uint<5> hc = 4 + bitbuf(13, 10);
                hlit                = 257 + bitbuf(4, 0);
                hdist               = 1 + bitbuf(9, 5);
                hlit_last           = 256 + bitbuf(4, 0);
                hdist_last          = bitbuf(9, 5);
                // hlit + hdist - 1 = 257 + HLIT + HDIST
                lidx_last = 257 + bitbuf(4, 0) + bitbuf(9, 5);
                hclen     = hc;
                hclen3    = hc * 3;
                nstate    = STATE_DYNAMIC_HUFFMAN_CODE_LENS;
                sh_reg    = hc * 3;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_CODE_LENS: {
            nsprintf("dynamic_huffman_code_lens\n");
            // consume [12, 57] bits
            if (length < hclen3) {
                // 补位: do_poll 在 switch 前按相同条件算好
            } else {
                // 前 hclen 个按 code_length_order 写入, 其余为 0. i < hclen
                // 用掩码表示, 不对每个 i 各做一次比较
                const ap_uint<19> valid = ~(~ap_uint<19>(0) << hclen);
            loop_fill_codelen:
                for (ap_uint<5> i = 0; i < 19; ++i) {
#pragma HLS UNROLL
                    code_lengths[code_length_order[i]] =
                        bitbuf(i * 3 + 2, i * 3) & -ap_uint<3>(valid[i]);
                }

            // rst tables
            loop_rst_codelen_bl_cnt:
                for (ap_uint<5> i = 0; i < 16; ++i) {
#pragma HLS UNROLL
                    codelen_table_bl_count[i] = 0;
                }
                // litlen/dist 的 bl_count 由码长写口在 CODE_LENS 清 0

                sidx   = 0;
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_CAL_COUNT;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_CAL_COUNT: {
            nsprintf("dynamic_huffman_build_codelen_table_cal_count\n");
            // 每拍累加一个 symbol, 与 litlen/dist 在码长写口累加相同.
            // 19 个比较在一拍内求和会串成加法链, 成为关键路径
            // bl_count[0] 已在 CODE_LENS 清 0, 长度为 0 的不计
            const ap_uint<3> l = code_lengths[sidx(4, 0)];
            if (l > 0) {
                ++codelen_table_bl_count[l];
            }
            // 用自增前的值比较, 不在同一拍内先加再比 (写后读)
            const ap_uint<1> sidx_end = sidx == 18;
            ++sidx;
            if (sidx_end) {
                acc_l      = 1;
                acc_code   = 0;
                acc_offset = 0;
                nstate =
                    STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_SYMBOL_OFFSET;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_SYMBOL_OFFSET: {
            nsprintf("dynamic_huffman_build_codelen_table_symbol_offset\n");
            // 每拍处理一个 l, 避免 15 级串行加法
            const ap_uint<16> code =
                (acc_code + codelen_table_bl_count[acc_l - 1]) << 1;
            codelen_table_limit[acc_l]   = code + codelen_table_bl_count[acc_l];
            codelen_table_offsets[acc_l] = acc_offset;
            codelen_table_base[acc_l]    = acc_offset - code;
            acc_code                     = code;
            acc_offset += codelen_table_bl_count[acc_l];
            // 用自增前的值比较
            const ap_uint<1> acc_end = acc_l == 15;
            ++acc_l;
            if (acc_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_POS;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_POS: {
            nsprintf("dynamic_huffman_build_codelen_table_fill_pos\n");
            // huffman_tmp 在 switch 之后统一写 (见 B 级)
            sidx   = 0;
            nstate = STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_SYMBOLS;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_SYMBOLS: {
            nsprintf("dynamic_huffman_build_codelen_table_fill_symbols\n");
            // 每拍处理一个 symbol, 展开后 19 级 huffman_tmp 自增链会限制 II.
            // 本拍只读码长, 下一拍写表 (fs_valid)
            const ap_uint<3> l = code_lengths[sidx(4, 0)];
            fill_valid         = l > 0;
            fill_tbl           = 0;
            fill_len           = l;
            fill_entry         = sidx;
            // 用自增前的值比较, 不在同一拍内先加再比 (写后读)
            const ap_uint<1> sidx_end = sidx == 18;
            ++sidx;
            if (sidx_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_DECODE_TABLE;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_DECODE_TABLE: {
            nsprintf("dynamic_huffman_decode_table\n");
            // 码字最长 7 位 + extra bits 最长 7 位 (symbol 18), 一次备足 14 位,
            // FILL 不需要再 poll, bitbuf 的写使能就不依赖查表得到的 symbol.
            // eos 时剩余位可能不足 14 (合法流中只差最后的填充位), 直接解
            if (length < 14 && !eos) {
                // 补位: do_poll 在 switch 前按相同条件算好
            } else {
                // 只算 dec_len 和 symbol 下标, 移位和查表放到 FILL
                // 有上下界时至多一个 l 命中, 按位 OR 归约, 不用 if 避免 select
                // 链 lt[l] = code_l < limit[l], 随 l 单调 (一旦为真后面都为真),
                // 第一个为真的 l 即码字长度: hit = lt[l] && !lt[l - 1]
                // code length code 只有 3 bit (RFC 1951), 码长最大 7, 只需 7 路
                const ap_uint<7> rev = internal::bitswap7(bitbuf(6, 0));
                ap_uint<7> hit = 0;
                ap_uint<8> lt  = 0; // lt[0] = 0
                ap_uint<5> lane_base[7];
                ap_uint<5> lane_code[7];
            loop_decode_table:
                for (ap_uint<4> l = 1; l <= 7; ++l) {
#pragma HLS UNROLL
                    const ap_uint<7> code = ap_uint<7>(rev >> (7 - l));
                    lt[l]                 = code < codelen_table_limit[l];
                    hit[l - 1]            = lt[l] && !lt[l - 1];
                    lane_base[l - 1]      = codelen_table_base[l];
                    lane_code[l - 1]      = ap_uint<5>(code);
                }
                ap_uint<5> idx = 0;
                ap_uint<5> base_sel = 0;
                ap_uint<5> code_sel = 0;
            loop_decode_table_sel:
                for (ap_uint<4> l = 0; l < 7; ++l) {
#pragma HLS UNROLL
                    const ap_uint<5> m = -ap_uint<5>(hit[l]);
                    base_sel |= lane_base[l] & m;
                    code_sel |= lane_code[l] & m;
                }
                idx = ap_uint<5>(base_sel + code_sel);
                // 第 l 路命中 (hit[l-1]) 时 dec_len = l, 每一位是固定若干路的
                // OR
                dec_len[0]  = (hit & 0x55).or_reduce();
                dec_len[1]  = (hit & 0x66).or_reduce();
                dec_len[2]  = (hit & 0x78).or_reduce();
                dec_len[3]  = 0;
                codelen_idx = idx;
                nstate      = STATE_DYNAMIC_HUFFMAN_DECODE_TABLE_FILL;
                // FILL 先移去码字; 16/17/18 的 extra bits 在 FILL_LENGTHS
                // 第一拍 移去, 这样 sh_reg 不依赖 symbol, 不加长解码路径
                sh_reg = dec_len;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_DECODE_TABLE_FILL: {
            nsprintf("dynamic_huffman_decode_table_fill\n");
            // bitbuf 尚未移去 dec_len, 本状态统一移去码字 (sh_reg = dec_len).
            // 16/17/18 的 extra bits 从 dec_len 处读, 由 FILL_LENGTHS
            // 第一拍移去 DECODE_TABLE 已保证 length >= 14 >= dec_len + extra,
            // 不需要 poll
            // symbol 来自查表, 不作为分支条件 (switch (symbol) 的各分支在
            // switch 之后合流, 选择条件会带上查表结果, 加长所有合流处的 mux),
            // 全部用 select. 非重复码由 switch 之后的码长写口写入.
            // 不是重复码时写 rep_cnt/rep_val/sh_reg 无影响: 下一状态
            // (DECODE_TABLE 或建表) 不移位, 也不读 rep_*
            const ap_uint<5> sym  = cl_sym;
            const ap_uint<1> rep  = sym[4];
            const ap_uint<1> is16 = sym == 16;
            const ap_uint<1> is17 = sym == 17;
            const ap_uint<7> ext =
                ap_uint<14>(cur_bitbuf(13, 0)) >> dec_len(2, 0);
            rep_cnt = is16   ? ap_uint<8>(3 + ext(1, 0))
                      : is17 ? ap_uint<8>(3 + ext(2, 0))
                             : ap_uint<8>(11 + ext(6, 0));
            rep_val = is16 ? prev_len : ap_uint<4>(0);
            sh_reg  = is16   ? ap_uint<6>(2)
                      : is17 ? ap_uint<6>(3)
                             : ap_uint<6>(7);
            nstate  = rep ? STATE_DYNAMIC_HUFFMAN_BUILD_FILL_LENGTHS
                      : at_last
                          ? STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_CAL_COUNT
                          : STATE_DYNAMIC_HUFFMAN_DECODE_TABLE;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_FILL_LENGTHS: {
            nsprintf("dynamic_huffman_build_fill_lengths\n");
            // 第一拍移去 extra bits (sh_reg = 2/3/7), 之后 sh_reg = 0 空移
            sh_reg = 0;
            // 每拍写一个重复的 code length (码长写口按 st_fl 写)
            // 用自减前的值比较, 不在同一拍内先减再比
            const ap_uint<1> rep_end = rep_cnt == 1;
            --rep_cnt;
            if (at_last) {
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_CAL_COUNT;
            } else if (rep_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_DECODE_TABLE;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_CAL_COUNT: {
            nsprintf("dynamic_huffman_build_litlen_table_cal_count\n");
            // bl_count 已在码长写口累加 ([0] 恒为 0)
            acc_l      = 1;
            acc_code   = 0;
            acc_offset = 0;
            nstate     = STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_SYMBOL_OFFSET;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_SYMBOL_OFFSET: {
            nsprintf("dynamic_huffman_build_litlen_table_symbol_offset\n");
            // 每拍处理一个 l, 避免 15 级串行加法
            const ap_uint<16> code =
                (acc_code + litlen_table_bl_count[acc_l - 1]) << 1;
            litlen_table_limit[acc_l] = code + litlen_table_bl_count[acc_l];
            litlen_table_lit_limit[acc_l] =
                code + litlen_table_lit_count[acc_l];
            litlen_table_offsets[acc_l] = acc_offset;
            if (acc_l == litlen_eob_len) {
                litlen_eob_idx = acc_offset + litlen_table_lit_count[acc_l];
            }
            litlen_table_base[acc_l] = acc_offset - code;
            acc_code                 = code;
            acc_offset += litlen_table_bl_count[acc_l];
            // 用自增前的值比较
            const ap_uint<1> acc_end = acc_l == 15;
            ++acc_l;
            if (acc_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_POS;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_POS: {
            nsprintf("dynamic_huffman_build_litlen_table_fill_pos\n");
            // huffman_tmp 在 switch 之后统一写 (见 B 级)
            sidx   = 0;
            nstate = STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_SYMBOLS;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_SYMBOLS: {
            nsprintf("dynamic_huffman_build_litlen_table_fill_symbols\n");
            // 每拍处理一个 symbol, 本拍只读码长、算表项, 下一拍写表 (fs_valid)
            const ap_uint<4> l = litlen_lengths[sidx];
            // length 码的表项按 symbol 低 5 位 (即 symbol - 256) 查常数表
            const ap_uint<5> li = sidx(4, 0);
            fill_valid          = l > 0;
            fill_tbl            = 1;
            fill_len            = l;
            fill_entry          = sidx < 256 ? ap_uint<12>(sidx)
                                             : ap_uint<12>(
                                          (ap_uint<12>(length_extra[li]) << 9) |
                                          length_base[li]);
            // 用自增前的值比较 (hlit_last = hlit - 1, 在 HEADER 预先算好)
            const ap_uint<1> sidx_end = sidx == hlit_last;
            ++sidx;
            if (sidx_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_CAL_COUNT;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_CAL_COUNT: {
            nsprintf("dynamic_huffman_build_dist_table_cal_count\n");
            // bl_count 已在码长写口累加
            acc_l      = 1;
            acc_code   = 0;
            acc_offset = 0;
            nstate     = STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_SYMBOL_OFFSET;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_SYMBOL_OFFSET: {
            nsprintf("dynamic_huffman_build_dist_table_symbol_offset\n");
            // 每拍处理一个 l, 避免 15 级串行加法
            const ap_uint<16> code = (acc_code + dist_table_bl_count[acc_l - 1])
                                     << 1;
            dist_table_limit[acc_l]   = code + dist_table_bl_count[acc_l];
            dist_table_offsets[acc_l] = acc_offset;
            dist_table_base[acc_l]    = acc_offset - code;
            acc_code                  = code;
            acc_offset += dist_table_bl_count[acc_l];
            // 用自增前的值比较
            const ap_uint<1> acc_end = acc_l == 15;
            ++acc_l;
            if (acc_end) {
                nstate = STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_POS;
            }
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_POS: {
            nsprintf("dynamic_huffman_build_dist_table_fill_pos\n");
            // huffman_tmp 在 switch 之后统一写 (见 B 级)
            sidx   = 0;
            nstate = STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_SYMBOLS;
        } break;
        case STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_SYMBOLS: {
            nsprintf("dynamic_huffman_build_dist_table_fill_symbols\n");
            // 每拍处理一个 symbol, 本拍只读码长, 下一拍写表 (fs_valid)
            const ap_uint<4> l = dist_lengths[sidx(4, 0)];
            fill_valid         = l > 0;
            fill_tbl           = 2;
            fill_len           = l;
            fill_entry         = sidx;
            // 用自增前的值比较 (hdist_last = hdist - 1, 在 HEADER 预先算好)
            const ap_uint<1> sidx_end = sidx == hdist_last;
            ++sidx;
            if (sidx_end) {
                LITLEN_START;
            }
        } break;
        case STATE_DECOMPRESS_FILL: {
            nsprintf("decompress_fill\n");
            // 稳态每拍一个 literal: 输出上一拍解出的 literal (litlen_idx 查表),
            // 同时移去它的码字 (sh_reg), 并从窗口 (跳过 sh_reg 位)
            // 解出下一个码字. 进入本状态而还没有解出码字时 pend_none = 1,
            // sh_reg = 0, 只解码 是否输出/移位/跳转只看本拍开始时的
            // pend_none/is_literal/is_eob, 查表结果只作为数据.
            // literal 由 switch 之后的推 token 口推出 (fe_push)
            if (!lack && !tq_full) {
                // length 码的 extra 位数 ll_ext 在 switch 之前算好
                // (表项见建表)
                const ap_uint<3> e_ext = ll_ext;
                if (pend_none) {
                    // 第一个码字: 只解出, 下一拍输出
                    litlen_idx = ll_idx;
                    is_literal = ll_lit;
                    pend_none  = 0;
                    sh_reg     = ll_len;
                } else if (is_literal) {
                    if (use_pair) {
                        // X, Y 本拍推出; Z 成为下一拍 pending, 窗口跳过 Y+Z
                        litlen_idx = litlen_next;
                        is_literal = lit_next;
                        sh_reg     = sh_pair;
                    } else {
                        // Y 是 literal 但长度不是 8/9 (或不够位数): 只推 X,
                        // Y 成为 pending, 下一拍继续尝试配对
                        // (litlen_next/lit_next/sh_pair 此时选出的就是 Y)
                        litlen_idx = litlen_next;
                        is_literal = lit_next;
                        sh_reg     = sh_pair;
                    }
                } else if (is_eob) {
                    nsprintf("end of block\n");
                    BLOCK_END;
                } else {
                    // length 码: extra bits 在 LEN 中读 (sh_reg = extra 位数)
                    // copy_len 在 LEN 中查表 (litlen_idx 不变), 288 选 1 的
                    // 结果不再经过本状态的跳转 mux
                    sh_reg = e_ext;
                    nstate = STATE_DECOMPRESS_LEN;
                }
            } else {
                // 补位 (do_poll 在 switch 前按相同条件算好) 或队列满
            }
        } break;
        case STATE_DECOMPRESS_LEN: {
            nsprintf("decompress_len\n");
            // 移去 length extra bits (sh_reg, [0, 5]), 同时从窗口解出其后的
            // dist 码字. match 长度 len_val 由 switch 之后的写口写入 copy_len
            dist_idx = dd_idx;
            sh_reg   = dd_len;
            nstate   = STATE_DECOMPRESS_DIST;
        } break;
        case STATE_DECOMPRESS_DIST: {
            nsprintf("decompress_dist\n");
            // 移去 dist 码字, 按 canonical position 一次读打包表得到
            // base/mask/extra/near (原来的 symbol -> base/extra 多级查表
            // 合成一次 32:1 读)
            const ap_uint<48> de = dist_table_entry[dist_idx];
            copy_dist            = de(5, 0);
            copy_rbase           = fe_pos - de(14, 0);
            copy_rbase4          = fe_pos - de(29, 15);
            copy_mask            = de(42, 30);
            copy_near            = de[47];
            sh_reg               = ap_uint<6>(de(46, 43));
            nstate               = STATE_DECOMPRESS_COPY_START;
        } break;
        case STATE_DECOMPRESS_COPY_START: {
            nsprintf("decompress_copy_start\n");
            // 移去 dist 码字 + extra bits (sh_reg, [1, 28]), 推出 match
            // token (源地址等在 switch 之前算好), 同时从窗口 (跳过 sh_reg
            // 位) 解出下一个码字, 与 FILL 相同. 位数不够或队列满时整拍停住
            if (!lack && !tq_full) {
                litlen_idx = ll_idx;
                is_literal = ll_lit;
                pend_none  = 0;
                sh_reg     = ll_len;
                nstate     = STATE_DECOMPRESS_FILL;
            }
        } break;
        case STATE_STORED_ALIGN: {
            nsprintf("stored_align\n");
            // 移去到字节边界的 [0, 7] 位 (sh_reg 在 BLOCK 中已算好)
            nstate = STATE_STORED_HEADER;
            sh_reg = 32;
        } break;
        case STATE_STORED_HEADER: {
            nsprintf("stored_header\n");
            // LEN (16) + NLEN (16), 不校验 NLEN
            if (length < 32) {
                // 补位: do_poll 在 switch 前按相同条件算好
            } else {
                nstate = STATE_STORED_COPY;
                sh_reg = 8;
            }
        } break;
        case STATE_STORED_COPY: {
            nsprintf("stored_copy\n");
            // 每拍原样输出一个字节, LEN = 0 时 (如 sync flush) 直接结束
            // 字节由 switch 之后的推 token 口推出 (fe_push)
            if (copy_zero) {
                BLOCK_END;
            } else if (length < 8 || tq_full) {
                // 补位 (do_poll 在 switch 前按相同条件算好) 或队列满
            } else {
                // copy_len/copy_zero 在 switch 之后统一写
            }
        } break;
        case STATE_DONE: {
            nsprintf("done\n");
            // 等后端输出完队列中的 token 后, 输出最后一拍 (可能为空拍,
            // keep = 0), do_flush 在 switch 前按 state/be_idle 算
            if (!be_idle) {
                // 等后端
            } else if (cur_last) {
                STREAM_RESET;
            } else {
                // 压缩流在 tlast 之前结束 (如 zlib/gzip 尾部), 丢弃剩余输入
                nstate = STATE_DRAIN;
            }
        } break;
        case STATE_DRAIN: {
            nsprintf("drain\n");
            // 复用补位口读 data_in (只能有一处 data_in.read, 否则两处读被排在
            // 流水线不同级, II = 2). 读完 tlast 那一拍后结束
            // do_drain 为 in_drain 寄存器
            if (cur_last) {
                STREAM_RESET;
            }
        } break;
        default:
            break;
        }

        // 输入已读完但还需要更多位: 流被截断, 输出已解出的数据并结束.
        // 补位的状态在 switch 内不写 nstate, 这里是唯一的覆盖点
        state = (need_poll && eos) ? ap_uint<6>(STATE_DONE) : nstate;

        // sh_reg 的窗口/移位器副本, 下一拍使用
        sh_w15 = sh_reg;
        sh_w89 = sh_reg;
        sh_buf = sh_reg;

        // copy_len/copy_zero 的唯一写口, 选择条件只来自寄存器: LEN 写
        // match 长度, STORED_HEADER (位数足够时) 写 LEN, STORED_COPY 推出
        // 一个字节时减 1. 在 switch 的分支里写时, 分支合流处的 mux 串在
        // LEN 的 288 选 1 查表和加法之后
        {
            const ap_uint<1> st_len = cur_state == STATE_DECOMPRESS_LEN;
            const ap_uint<1> st_sh =
                cur_state == STATE_STORED_HEADER && cur_length >= 32;
            const ap_uint<16> cur_copy_len = copy_len;
            const ap_uint<16> dec          = cur_copy_len - 1;
            const ap_uint<1>  dec_zero     = cur_copy_len == 1;
            copy_len                       = st_len ? len_val
                                             : st_sh ? ap_uint<16>(cur_bitbuf(15, 0))
                                             : (st_stored && fe_push) ? dec
                                                                      : cur_copy_len;
            copy_zero = st_sh ? ap_uint<1>(cur_bitbuf(15, 0) == 0)
                        : (st_stored && fe_push) ? dec_zero
                                                 : copy_zero;
        }
        // DONE 且未读到 tlast 时进入 DRAIN, DRAIN 读到 tlast 后离开
        in_drain = ((cur_state == STATE_DONE && be_idle) ||
                    cur_state == STATE_DRAIN) &&
                   !cur_last;

        // 唯一的码长写口 (lidx 在此自增), switch 内读到的 lidx 是自增前的值
        // 码长写口. bl_count 每项每拍无条件写一次: CODE_LENS 清 0, 否则按
        // one-hot 累加 (不按 wl_val 动态下标读改写, 那样 16 选 1 的读 mux
        // 串在 wl_val 之后). [0] 不累加 (长度为 0 的 symbol 不计), 建表时
        // 读到的 bl_count[0] 恒为 0. lidx/prev_len/at_last 在 BLOCK 清 0
        {
            const ap_uint<1> rst_cnt =
                cur_state == STATE_DYNAMIC_HUFFMAN_CODE_LENS;
            const ap_uint<1> rst_idx = cur_state == STATE_BLOCK;
            const ap_uint<1> to_lit  = lidx < cur_hlit;
            const ap_uint<1> is_lit  = lidx < 256;
        loop_write_length_count:
            for (ap_uint<5> i = 0; i < 16; ++i) {
#pragma HLS UNROLL
                const ap_uint<1> inc = wl_en && wl_oh[i] && i != 0;
                litlen_table_bl_count[i] =
                    rst_cnt ? ap_uint<9>(0)
                            : ap_uint<9>(
                                  litlen_table_bl_count[i] + (inc && to_lit));
                litlen_table_lit_count[i] =
                    rst_cnt ? ap_uint<9>(0)
                            : ap_uint<9>(
                                  litlen_table_lit_count[i] +
                                  (inc && to_lit && is_lit));
                dist_table_bl_count[i] =
                    rst_cnt
                        ? ap_uint<6>(0)
                        : ap_uint<6>(dist_table_bl_count[i] + (inc && !to_lit));
            }
            if (wl_en) {
                if (to_lit) {
                    litlen_lengths[lidx] = wl_val;
                    if (lidx == 256)
                        litlen_eob_len = wl_val;
                } else {
                    dist_lengths[lidx - cur_hlit] = wl_val;
                }
            }
            // at_last: 下一次写入的是最后一个 (lidx + 1 == cur_lidx_last)
            at_last  = rst_idx ? ap_uint<1>(0)
                       : wl_en ? ap_uint<1>(lidx + 1 == cur_lidx_last)
                               : at_last;
            prev_len = rst_idx ? ap_uint<4>(0) : wl_en ? wl_val : prev_len;
            lidx     = rst_idx ? ap_uint<9>(0) : ap_uint<9>(lidx + wl_en);
        }

        // 填 symbols 流水的 B 级: 只读本拍开始时的 fs_* (上一拍 A 级的输出),
        // 状态已跳走时也照常完成最后一个 symbol
        // huffman_tmp 每项每拍无条件写一次, 选择条件只来自寄存器 (state,
        // fs_valid, fs_len): *_FILL_POS 载入 offsets, B 级自增一项.
        // 在 switch 内的分支里写会给每项生成跨分支的写使能 phi 链,
        // 把 FILL 等状态的分支条件串进来
        const ap_uint<1> fp_codelen =
            cur_state == STATE_DYNAMIC_HUFFMAN_BUILD_CODELEN_TABLE_FILL_POS;
        const ap_uint<1> fp_litlen =
            cur_state == STATE_DYNAMIC_HUFFMAN_BUILD_LITLEN_TABLE_FILL_POS;
        const ap_uint<1> fp_dist =
            cur_state == STATE_DYNAMIC_HUFFMAN_BUILD_DIST_TABLE_FILL_POS;
    loop_huffman_tmp:
        for (ap_uint<5> i = 0; i < 16; ++i) {
#pragma HLS UNROLL
            const ap_uint<9> cur = huffman_tmp[i];
            huffman_tmp[i] = fp_codelen  ? ap_uint<9>(cur_codelen_offsets[i])
                             : fp_litlen ? ap_uint<9>(cur_litlen_offsets[i])
                             : fp_dist   ? ap_uint<9>(cur_dist_offsets[i])
                             : (fs_valid && fs_len == i)
                                 ? ap_uint<9>(fs_pos + 1)
                                 : cur;
        }
        if (fs_valid) {
            switch (fs_tbl) {
            case 0:
                codelen_table_symbols[fs_pos(4, 0)] = fs_entry(4, 0);
                codelen_table_oh[fs_pos(4, 0)] =
                    fs_entry[4] ? ap_uint<16>(0)
                                : ap_uint<16>(ap_uint<16>(1) << fs_entry(3, 0));
                break;
            case 1:
                litlen_table_symbols[fs_pos] = fs_entry;
                break;
            default: {
                // dist 表按 canonical position 打包写, 解码时按 dd_idx 直接读
                const ap_uint<5> sym = fs_entry(4, 0);
                ap_uint<48>      e   = 0;
                e(14, 0)             = dist_base[sym](14, 0);
                e(29, 15)            = dist_base_m4[sym];
                e(42, 30)            = dist_mask[sym];
                e(46, 43)            = dist_extra[sym];
                e[47]                = sym <= 9;
                dist_table_entry[fs_pos(4, 0)] = e;
            } break;
            }
        }
        fs_valid = fill_valid;
        fs_tbl   = fill_tbl;
        fs_len   = fill_len;
        fs_entry = fill_entry;

        // token 队列: 推入 (前端) 与取出 (后端), 写入的位置 (tail) 与本拍
        // 读出的位置 (head) 不同 (队列空时不取, 满时不推)
        if (fe_push) {
            tq_match[tq_tail]  = st_cstart;
            tq_len[tq_tail]    = push_len;
            tq_b1[tq_tail]     = push_b1;
            tq_nlit[tq_tail]   = push_nlit;
            tq_near[tq_tail]   = m_near;
            tq_dist[tq_tail]   = m_dist;
            tq_raddr[tq_tail]  = m_raddr;
            tq_raddr4[tq_tail] = m_raddr4;
        }
        // 自增/自减结果从寄存器并行算好, fe_push/pop 只做选择. 写成
        // count + push - pop 时, push 的判断串在两级加法之前
        {
            const ap_uint<2>  tail1   = tq_tail + 1;
            const ap_uint<2>  head1   = tq_head + 1;
            const ap_uint<3>  cur_cnt = tq_count;
            const ap_uint<3>  cnt_p1  = cur_cnt + 1;
            const ap_uint<3>  cnt_m1  = cur_cnt - 1;
            const ap_uint<15> pos_l   = fe_pos + m_len;
            const ap_uint<15> pos_1   = fe_pos + 1;
            const ap_uint<15> pos_n   = fe_pos + push_nlit;
            tq_tail                   = fe_push ? tail1 : tq_tail;
            tq_head                   = pop ? head1 : tq_head;
            // AND-OR 选择, 写成 select 时 HLS 会折回成 count + push - pop
            const ap_uint<1> inc  = fe_push && !pop;
            const ap_uint<1> dec  = pop && !fe_push;
            const ap_uint<1> hold = !inc && !dec;
            tq_count              = (cnt_p1 & -ap_uint<3>(inc)) |
                       (cnt_m1 & -ap_uint<3>(dec)) |
                       (cur_cnt & -ap_uint<3>(hold));
            tq_nempty =
                fe_push || cur_cnt[2] || cur_cnt[1] || (tq_nempty && !pop);
            fe_pos = !fe_push    ? fe_pos
                     : st_cstart ? pos_l
                     : st_stored ? pos_1
                                 : pos_n;
        }

        // 读窗口: 起始地址的字节在 bank rot 起, bank b < rot 的字节在下一个
        // 字. 每拍都读, 读出的字节下一拍用.
        // 放在后端状态更新之前, 读本拍开始时的 be_*
        {
            const ap_uint<2>  rot = be_busy ? be_rot : h_raddr(1, 0);
            const ap_uint<13> w0 =
                be_busy ? be_w0 : ap_uint<13>(h_raddr(14, 2));
            const ap_uint<13> w1 =
                be_busy ? be_w1 : ap_uint<13>(h_raddr4(14, 2));
        loop_win_read:
            for (ap_uint<3> bk = 0; bk < 4; ++bk) {
#pragma HLS UNROLL
                be_rd[bk] = window[bk][bk >= rot ? w0 : w1];
            }
        }

        // 后端状态. 取到 match 时: 长度、near 的 hist 位置 (第 j 个字节的
        // 源为 j % d 个周期之前, 即 hist[d - 1 - j % d]), far 的读地址
        if (pop_match) {
            be_busy = 1;
            be_len  = h_len;
            be_n    = h_len > 4 ? ap_uint<3>(4) : ap_uint<3>(h_len);
            be_end  = h_len <= 4;
            be_near = h_near;
        loop_be_idx:
            for (ap_uint<3> j = 0; j < 4; ++j) {
#pragma HLS UNROLL
                const ap_uint<6> d = h_dist == 0 ? ap_uint<6>(32) : h_dist;
                const ap_uint<2> m =
                    d == 1   ? ap_uint<2>(0)
                    : d == 2 ? ap_uint<2>(j & 1)
                    : d == 3 ? ap_uint<2>(j == 3 ? ap_uint<3>(0) : j)
                             : ap_uint<2>(j);
                be_idx[j] = d - 1 - m;
            }
            be_rot = h_raddr(1, 0);
            be_w0  = h_raddr(14, 2) + 1;
            be_w1  = h_raddr4(14, 2) + 1;
        } else if (be_busy) {
            // 本拍是最后一拍 (本拍开始时的 be_end) 则结束拷贝
            const ap_uint<1> was_end = be_end;
            const ap_uint<9> rest    = be_len - 4;
            be_len                   = rest;
            be_n    = rest > 4 ? ap_uint<3>(4) : ap_uint<3>(rest);
            be_end  = rest <= 4;
            be_busy = !was_end;
            be_w0   = be_w0 + 1;
            be_w1   = be_w1 + 1;
        }
        // 唯一的输出口: 本拍 out_n 个字节 out_b[0..n) 写入窗口、hist、输出拍
        {
            const ap_uint<2> boff = bufoff(1, 0);
        loop_win_write:
            for (ap_uint<3> bk = 0; bk < 4; ++bk) {
#pragma HLS UNROLL
                const ap_uint<2> j = bk - boff;
                if (j < out_n) {
                    window[bk]
                          [bk >= boff ? ap_uint<13>(bufoff(14, 2)) : buf_w1] =
                              out_b[j];
                }
            }
            const ap_uint<15> nb = bufoff + out_n;
            bufoff               = nb;
            buf_w1               = nb(14, 2) + 1;
        }
        {
            // hist 左移 n 个字节, 最后输出的字节在最低位: 5 种常数移位按
            // out_n 选择
            const ap_uint<256> h1 = (hist << 8) | ap_uint<256>(out_b[0]);
            const ap_uint<256> h2 =
                (hist << 16) | ap_uint<256>((ap_uint<16>(out_b[0]), out_b[1]));
            const ap_uint<256> h3 =
                (hist << 24) |
                ap_uint<256>((ap_uint<24>(out_b[0]), out_b[1], out_b[2]));
            const ap_uint<256> h4 =
                (hist << 32) |
                ap_uint<256>((out_b[0], out_b[1], out_b[2], out_b[3]));
            hist = out_n == 0   ? hist
                   : out_n == 1 ? h1
                   : out_n == 2 ? h2
                   : out_n == 3 ? h3
                                : h4;
        }
        // 输出拍: 本拍开始时 outoff > 64 (或 = 64 且本拍有新字节) 时, 低 64
        // 字节原样写出, 第 64..67 字节移到开头. 本拍的第 j 个字节放在
        // (outoff mod 64) + j, 不回绕, 至多到第 67 字节. 正好 64 字节时等到
        // 下一个字节或 DONE 才写出, DONE 写出时 last = 1, 不多出空拍.
        // 只读本拍开始时的 outoff, outoff 只写一次
        {
            const ap_uint<7> cur_outoff = outoff;
            const ap_uint<6> o6         = cur_outoff(5, 0);
            const ap_uint<1> ready = cur_outoff[6] && (o6 != 0 || out_n != 0);
            // 写入位置的范围 [o6, o6 + n), 用两个温度计掩码表示, 不按字节
            // 各做一次减法和比较
            const ap_uint<7>  o6n  = o6 + out_n;
            const ap_uint<68> ge   = ~ap_uint<68>(0) << o6;
            const ap_uint<68> lt   = ~(~ap_uint<68>(0) << o6n);
            const ap_uint<68> take = ge & lt;
            // 第 p 个字节若写入, 其值为 out_b[(p - o6) % 4], 只依赖 p % 4,
            // 先按 o6 % 4 旋转一次
            ap_uint<8> rb[4];
#pragma HLS ARRAY_PARTITION variable = rb complete
        loop_out_rot:
            for (ap_uint<3> k = 0; k < 4; ++k) {
#pragma HLS UNROLL
                rb[k] = out_b[ap_uint<2>(k - o6(1, 0))];
            }
            ap_uint<544> merged;
        loop_out_merge:
            for (ap_uint<7> p = 0; p < 68; ++p) {
#pragma HLS UNROLL
                // 写出当前拍后, 第 64..67 字节移到第 0..3 字节. 第 4 字节
                // 及以后在 outoff 之后, 内容无关
                const ap_uint<8> old =
                    (p < 4 && ready)
                        ? ap_uint<8>(out_data(p * 8 + 519, p * 8 + 512))
                        : ap_uint<8>(out_data(p * 8 + 7, p * 8));
                merged(p * 8 + 7, p * 8) = take[p] ? rb[p(1, 0)] : old;
            }
            if (ready || do_flush) {
                axi_t o;
                o.data = out_data(511, 0);
                o.keep = (ready || cur_outoff == 64)
                             ? ~ap_uint<64>(0)
                             // DONE 时 outoff < 64, o6 == outoff, 复用 ge
                             : ap_uint<64>(~ge(63, 0));
                o.last = do_flush;
                data_out.write(o);
            }
            out_data = merged;
            // 下一拍的 outoff: 各种 out_n 的结果从寄存器并行算好, out_n
            // 只做选择
            ap_uint<7> nxt_ready[5], nxt_keep[5];
#pragma HLS ARRAY_PARTITION variable = nxt_ready complete
#pragma HLS ARRAY_PARTITION variable = nxt_keep complete
        loop_outoff_next:
            for (ap_uint<3> k = 0; k < 5; ++k) {
#pragma HLS UNROLL
                nxt_ready[k] = o6 + k;
                nxt_keep[k]  = cur_outoff + k;
            }
            outoff = do_flush ? ap_uint<7>(0)
                     : ready  ? nxt_ready[out_n]
                              : nxt_keep[out_n];
        }

        // 唯一的右移位器 + 补位口, 无条件计算, 结果按 do_shift/do_fill 选择.
        // 若写成 if (do_shift) { bitbuf = cur_bitbuf >> sh; }, HLS 会把移位
        // 下沉到每个状态的分支里, 生成多份 128 位移位器
        //
        // bitbuf 中 length 以上的位恒为 0 (复位清 0, 右移补 0, 补入的 64 位
        // 按剩余位数清 0), 所以用一次可变左移 + OR 写入, 不需要按 length
        // 展开 64 个写入位置
        //
        // 以下判断只读本拍开始时的 cur_beat_bits/cur_last, 不读本拍刚写的值
        // (写后读会把多段运算串成一条组合路径). beat_bits/beat_buf/last 各只在
        // 下面写一次
        //
        // 移位器的输入只有寄存器 (cur_bitbuf, sh_reg, length), 无条件移位,
        // do_shift 只在最后选择结果. 若先算 shamt = do_shift ? sh : 0,
        // do_shift 的判断 (如 length >= 8) 就串到了移位器前面.
        //
        // 补位位置 = 移位后的长度 (do_shift ? length - sh : length). 两种位置
        // 各用一个移位器并行算 (移位量只来自寄存器), do_shift 只在最后选择.
        // 若先选位置再移位, do_shift (各状态的长度比较 + 跨状态合流) 就串在
        // 128 位移位器之前. 用 AND-OR 选择, 写成 select 时会被折回成
        // shl(x, select(...)). 补位时 length < 64, 移位量只需 6 位
        const ap_uint<1>   do_fill   = do_poll && !cur_beat_zero;
        const ap_uint<128> shifted   = cur_bitbuf >> sh;
        const ap_uint<7>   remaining = cur_length - sh;
        const ap_uint<128> consumed  = do_shift ? shifted : cur_bitbuf;
        // do_fill 只由寄存器决定, 先作用在数据上, 不补位时两个移位结果都为 0.
        // beat_buf 搬入时不按 keep 清 0, 这里只对补入的 64 位按对应的 8 个
        // keep 位清掉无效字节, 保证 bitbuf 中 length 以上为 0. 掩码是寄存器
        // beat_keep 的逐位展开, 不在移位器之前现算
        ap_uint<64> fill_mask;
    loop_fill_mask:
        for (ap_uint<7> b = 0; b < 64; ++b) {
#pragma HLS UNROLL
            fill_mask[b] = do_fill && beat_keep[b >> 3];
        }
        const ap_uint<128> fill_data =
            ap_uint<128>(beat_buf(63, 0) & fill_mask);
        const ap_uint<128> fill_shift = fill_data << remaining(5, 0);
        const ap_uint<128> fill_keep  = fill_data << cur_length(5, 0);
        const ap_uint<128> sel_shift  = -ap_uint<128>(do_shift);
        bitbuf = consumed | (fill_shift & sel_shift) | (fill_keep & ~sel_shift);
        // beat_bits 不足 64 时高位为 0, 多补的 0 不会被计入 length
        const ap_uint<7> fill_bits = !do_fill ? ap_uint<7>(0)
                                     : cur_beat_bits < 64
                                         ? ap_uint<7>(cur_beat_bits(6, 0))
                                         : ap_uint<7>(64);
        length = do_shift ? ap_uint<7>(remaining + fill_bits)
                          : ap_uint<7>(cur_length + fill_bits);

        // 输入暂存级. beat_buf 为空且暂存有数据时搬入 (move), 暂存为空或本拍
        // 搬走时读下一拍. DRAIN 时一直读 (覆盖暂存) 直到 tlast.
        // data_in.read 只有一处, 两处分开写会被排在流水线不同级, II = 2
        const ap_uint<1> cur_in_valid = in_valid;
        const ap_uint<1> move         = cur_in_valid && cur_beat_zero;
        const ap_uint<1> do_read =
            !cur_last && (!cur_in_valid || cur_beat_zero || do_drain) &&
            !data_in.empty();
        // beat_zero/in_valid/eos_reg 的下一拍值, 与 beat_bits/in_valid/last
        // 的写入一一对应. move 要求 beat_zero, do_fill 要求 !beat_zero, 互斥
        ap_uint<1> next_last  = cur_last;
        ap_uint<1> next_valid = cur_in_valid && !move;
        ap_uint<1> next_zero  = cur_beat_zero;
        if (move) {
            // keep 以外的字节不清 0, 补位时按 beat_keep 清 (见 fill_mask)
            beat_buf  = in_data;
            beat_keep = in_keep;
            beat_bits = (64 - __builtin_clzll(in_keep)) << 3;
            next_zero = in_keep == 0;
            nsprintf("beat_bits: %u\n", beat_bits.to_uint());
        } else if (do_fill) {
            beat_bits = cur_beat_bits - fill_bits;
            next_zero = cur_beat_bits <= 64;
            beat_buf >>= 64;
            beat_keep >>= 8;
        }
        if (do_read) {
            const axi_t tmp = data_in.read();
            in_data         = tmp.data;
            in_keep         = tmp.keep;
            last            = tmp.last;
            next_last       = tmp.last;
            next_valid      = 1;
        }
        in_valid  = next_valid;
        beat_zero = next_zero;
        // eos_reg = next_last && !next_valid && next_zero. do_read 要求
        // !cur_last, 而读入后 next_valid = 1, 所以读与不读结果相同, 不经过
        // data_in 的握手和读出的 tlast
        eos_reg = cur_last && !(cur_in_valid && !move) && next_zero;
    } break;
    }
}
