#pragma once

#include "motor_scanner.hpp"

#include <damiao_core/bus.hpp>
#include <damiao_core/types.hpp>

#include <cstddef>
#include <vector>

namespace damiao_tools
{

const char* error_name(damiao::ErrorCode code);
const char* bus_state_name(damiao::BusState state);
void print_status_error(const damiao::Status& status);
void print_motor_state(const damiao::Result<damiao::MotorState>& result);
void print_motor_list(const std::vector<DiscoveredMotor>& motors, const std::string& can_interface,
    std::size_t registered_count, std::size_t selected_list_index);

}  // namespace damiao_tools
