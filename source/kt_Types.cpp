/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "kt_Types.h"

namespace karto
{

  ParameterManager::ParameterManager()
  {
  }

  ParameterManager::~ParameterManager()
  {
    Clear();
  }

  void ParameterManager::Add(AbstractParameter* pParameter)
  {
    if(pParameter != NULL)
    {
      if(m_ParameterLookup.find(pParameter->GetName()) == m_ParameterLookup.end())
      {
        m_Parameters.push_back(pParameter);

        m_ParameterLookup[pParameter->GetName()] = pParameter;
      }
      else
      {
        m_ParameterLookup[pParameter->GetName()]->SetValueFromString(pParameter->GetValueAsString());

        assert(false);
      }
    }
  }

  AbstractParameter* ParameterManager::Get(const std::string& rName)
  {
    if(m_ParameterLookup.find(rName) != m_ParameterLookup.end())
    {
      return m_ParameterLookup[rName];
    }

    std::cout << "Unable to get parameter: " << rName << std::endl;

    return NULL;
  }

  void ParameterManager::Clear()
  {
    forEach(karto::ParameterVector, &m_Parameters)
    {
      delete *iter;
    }

    m_Parameters.clear();

    m_ParameterLookup.clear();
  }

  const ParameterVector& ParameterManager::GetParameterVector() const 
  {
    return m_Parameters;
  }

  AbstractParameter* ParameterManager::operator()(const std::string& rName)
  {
    return Get(rName);
  }

}
