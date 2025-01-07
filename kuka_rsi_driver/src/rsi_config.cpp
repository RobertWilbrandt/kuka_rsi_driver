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

/*!\file rsi_config.cpp
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-01-07
 */
#include "kuka_rsi_driver/rsi_config.h"

#include <stdexcept>

namespace kuka_rsi_driver {

RsiElement::RsiElement(const std::string& name, RsiType type, std::size_t index)
  : name{name}
  , type{type}
  , index{index}
{
}

RsiAttributeElement::RsiAttributeElement(const std::string& name)
  : name{name}
{
}

RsiAttributeElement::RsiAttributeElement(const std::string& name,
                                         const std::vector<RsiElement>& attributes)
  : name{name}
  , attributes{attributes}
{
}


void CommunicationConfig::addAttributeElement(const std::string& name,
                                              const std::vector<std::string>& attribute_names,
                                              const std::vector<RsiType>& attribute_types)
{
  RsiAttributeElement attribute_element{name};

  if (attribute_names.size() != attribute_types.size())
  {
    throw std::runtime_error{"Attribute names and attribute sizes must be the same size"};
  }
  for (std::size_t i = 0; i < attribute_names.size(); ++i)
  {
    attribute_element.attributes.emplace_back(
      attribute_names[i], attribute_types[i], numSignals(attribute_types[i])++);
  }

  m_attribute_elements.push_back(attribute_element);
}

void CommunicationConfig::addElement(const std::string& name, RsiType type)
{
  m_elements.emplace_back(name, type, numSignals(type)++);
}

std::size_t CommunicationConfig::numBoolSignals() const
{
  return m_bool_cnt;
}

std::size_t CommunicationConfig::numDoubleSignals() const
{
  return m_double_cnt;
}

std::size_t CommunicationConfig::numLongSignals() const
{
  return m_long_cnt;
}

const std::vector<RsiAttributeElement>& CommunicationConfig::attributeElements() const
{
  return m_attribute_elements;
}

const std::vector<RsiElement>& CommunicationConfig::elements() const
{
  return m_elements;
}

std::size_t& CommunicationConfig::numSignals(RsiType type)
{
  switch (type)
  {
    case RsiType::BOOL:
      return m_bool_cnt;
    case RsiType::DOUBLE:
      return m_double_cnt;
    case RsiType::LONG:
      return m_long_cnt;
    default:
      throw std::runtime_error{"Invalid RSI type"};
  }
}

} // namespace kuka_rsi_driver
