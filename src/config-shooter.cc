// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-constrained-planner.
//
// hpp-constrained-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-constrained-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-constrained-planner.  If not, see
// <http://www.gnu.org/licenses/>.

#include <KineoWorks2/kwsConfig.h>
#include <hpp/model/device.hh>

#include "hpp/constrained/config-shooter.hh"

namespace hpp {
  namespace constrained {
    ConfigShooter::~ConfigShooter() {}

    /// Generate a random configuration about a given configuration
    void ConfigShooter::shoot (CkwsConfig& io_config) const
    {
      double sigma = standardDeviation_;
      io_config.randomStep (sigma);
    }
    
    void ConfigShooter::standardDeviation (const double& sigma)
    {
      standardDeviation_ = sigma;
    }

    double ConfigShooter::standardDeviation () const
    {
      return standardDeviation_;
    }

    /// Create object and return shared pointer
    ConfigShooterShPtr ConfigShooter::create (hpp::model::DeviceShPtr robot)
    {
      ConfigShooter* ptr = new ConfigShooter (robot);
      ConfigShooterShPtr shPtr (ptr);
      ConfigShooterWkPtr wkPtr (shPtr);

      ptr->init (wkPtr);
      return shPtr;
    }
     
    ConfigShooter::ConfigShooter (hpp::model::DeviceShPtr robot) :
      robot_ (robot), standardDeviation_ (0.01)
    {
    }

    void ConfigShooter::init (ConfigShooterWkPtr wkPtr)
    {
      weakPtr_ = wkPtr;
    }
  } // namespace constrained
} //namespace hpp
