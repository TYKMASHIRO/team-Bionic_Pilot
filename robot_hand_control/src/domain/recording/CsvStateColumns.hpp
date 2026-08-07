#pragma once

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "robotics/domain/states/CombinedRobotState.hpp"

namespace robotics::domain {

/**
 * @brief states.csv 固定 110 列契约（列名单一来源）。
 *
 * CsvRecordSink（阶段4 写）与 TrajectoryCsvLoader（阶段5 读）共用本契约，
 * 保证"录制写出的列序"与"轨迹读回的列序"永远一致。
 *
 * 缺席编码（与 CsvRecordSink 一致）：设备 valid=false 时数值列写 0、
 * 字符串/矢量列写空串、*_valid=0；消费方以 *_valid 判定在场。
 */
struct CsvStateColumns {
    /// 110 列列名（顺序即 states.csv 表头列序）。
    static const std::vector<std::string>& names();

    /// 列数（恒 110）。
    static std::size_t count() { return names().size(); }

    /// 写一行：header=true 写表头，否则写数据行（不含末尾换行）。
    /// 表头与数据行由同一列名序列驱动，永远对齐。
    static bool write_row(std::ostream& os, const CombinedRobotState& c,
                          bool header);

    /**
     * @brief 解析一行数据。
     * @param header 表头行（split 结果）；@param line 数据行。
     * @param out    解析结果。
     * @return false = 表头缺列 / 行内字段数不足 / 数值解析失败。
     */
    static bool parse_row(const std::vector<std::string>& header,
                          const std::string& line, CombinedRobotState& out);
};

/// CSV 字段转义：含逗号/引号/换行时双引号包裹、内部引号加倍。
/// 供 events.csv / commands.csv 落盘与轨迹写入复用。
std::string csv_escape_shared(const std::string& v);

/// CSV 行按逗号切分（states.csv 数值/分号字段不含引号，无需转义解析）。
std::vector<std::string> csv_split(const std::string& line);

}  // namespace robotics::domain
