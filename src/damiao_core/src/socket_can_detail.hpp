#pragma once

#include <damiao_core/types.hpp>
#include <linux/can.h>
#include <cstddef>

namespace damiao
{
namespace detail
{

// 将 Linux 原生帧转换为核心帧，并拒绝非经典 CAN 标准数据帧。
Result<CanFrame> decode_native_frame(const can_frame& frame, std::size_t bytes,
    SteadyClock::time_point received_at);
// 将经过范围校验的核心帧转换为 Linux 原生经典 CAN 帧。
Result<can_frame> encode_native_frame(const CanFrame& frame);
// 非阻塞取得接口所有权锁，并将锁文件描述符交给调用方管理。
Status acquire_lock(const std::string& path, int& descriptor);
// 幂等关闭接口所有权锁文件描述符。
void release_lock(int& descriptor) noexcept;

}  // namespace detail
}  // namespace damiao
