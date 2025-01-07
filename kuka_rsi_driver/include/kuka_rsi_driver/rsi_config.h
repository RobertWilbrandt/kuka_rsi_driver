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
 * \brief Configuration of additional RSI signals that should be handled
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-01-07
 */
#ifndef KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
#define KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED

#include <string>
#include <vector>

namespace kuka_rsi_driver {

enum class RsiType
{
  BOOL,
  DOUBLE,
  LONG
};

struct RsiElement
{
  RsiElement(const std::string& name, RsiType type, std::size_t index);

  std::string name;

  RsiType type;
  std::size_t index;
};

struct RsiAttributeElement
{
  explicit RsiAttributeElement(const std::string& name);
  RsiAttributeElement(const std::string& name, const std::vector<RsiElement>& attributes);

  std::string name;
  std::vector<RsiElement> attributes;
};


class CommunicationConfig
{
public:
  CommunicationConfig() = default;

  void addAttributeElement(const std::string& name,
                           const std::vector<std::string>& attribute_names,
                           const std::vector<RsiType>& attribute_types);
  void addElement(const std::string& name, RsiType type);

  [[nodiscard]] std::size_t numBoolSignals() const;
  [[nodiscard]] std::size_t numDoubleSignals() const;
  [[nodiscard]] std::size_t numLongSignals() const;

  const std::vector<RsiAttributeElement>& attributeElements() const;
  const std::vector<RsiElement>& elements() const;

private:
  std::size_t& numSignals(RsiType type);

  std::size_t m_bool_cnt   = 0;
  std::size_t m_double_cnt = 0;
  std::size_t m_long_cnt   = 0;

  std::vector<RsiAttributeElement> m_attribute_elements;
  std::vector<RsiElement> m_elements;
};

struct RsiConfig
{
  CommunicationConfig transmission_config;
  CommunicationConfig reception_config;
};

} // namespace kuka_rsi_driver

#endif // KUKA_RSI_DRIVER_RSI_CONFIG_H_INCLUDED
