// Copyright 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file kuka_rsi_driver/rsi_config.h
 * \brief Definition of mappings between interfaces, internal structures and XML entries
 *
 * \author  Robert Wilbrandt wilbrandt@fzi.de
 * \date    2025-03-06
 */
#ifndef KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED

#include <array>
#include <hardware_interface/hardware_info.hpp>
#include <string>
#include <vector>

namespace kuka_rsi_driver {

/*! \brief Definition of all state and command interfaces
 */
struct InterfaceConfig
{
  //! Fully qualified names of all joint position state interfaces
  std::vector<std::string> joint_position_state_interfaces;
  //! Fully qualified names of all joint effort state interfaces
  std::vector<std::string> joint_effort_state_interfaces;
  //! Fully qualified names of all joint position command interfaces
  std::vector<std::string> joint_position_command_interfaces;

  //! Fully qualified names of all tcp sensor state interfaces
  std::array<std::string, 7> tcp_state_interfaces;

  //! Fully qualified name of robot state state interface
  std::string robot_state_state_interface;
  //! Fully qualified name of speed scaling state interface
  std::string speed_scaling_state_interface;
};

class RsiConfig
{
public:
  RsiConfig(const hardware_interface::HardwareInfo& info);

  const InterfaceConfig& interfaceConfig() const;

private:
  void verifyComponent(const hardware_interface::ComponentInfo& component,
                       const std::string& component_function,
                       const std::vector<std::string>& expected_state_interfaces,
                       const std::vector<std::string>& expected_command_interfaces) const;

  InterfaceConfig m_interface_config;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
