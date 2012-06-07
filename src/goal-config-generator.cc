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
#include <kwsIO/kwsioConfig.h>
#include <hpp/util/exception.hh>
#include <hpp/util/debug.hh>
#include <hpp/model/fwd.hh>

#include "hpp/constrained/config-shooter.hh"
#include "hpp/constrained/goal-config-generator.hh"

namespace hpp {
  namespace constrained {
    GoalConfigGenerator::~GoalConfigGenerator() {}

    GoalConfigGeneratorShPtr GoalConfigGenerator::create
    (hpp::model::DeviceShPtr robot)
    {
      GoalConfigGenerator* ptr = new GoalConfigGenerator (robot);
      GoalConfigGeneratorShPtr shPtr (ptr);
      GoalConfigGeneratorWkPtr wkPtr (shPtr);

      ptr->init (wkPtr);
      return shPtr;
    }

    GoalConfigGenerator::GoalConfigGenerator (hpp::model::DeviceShPtr robot) :
      ConfigProjector (robot)
    {
      configShooter_ = ConfigShooter::create (robot);
    }

    void GoalConfigGenerator::init (GoalConfigGeneratorWkPtr wkPtr)
    {
      weakPtr_ = wkPtr;
    }

    bool GoalConfigGenerator::generate (CkwsConfig& io_config)
    {
      unsigned int nb_try=0;
      unsigned int nb_maxTry=10;
      while (nb_try < nb_maxTry) { //Shoot config
	nb_try++;
	CkwsConfigShPtr randomConfig = CkwsConfig::create(io_config);
	configShooter ()->shoot (*randomConfig);

	if (project(*randomConfig) == KD_OK) {
	  //Projection worked
	  hppDout (info, "config projected: " << *randomConfig);
	  model::DeviceShPtr robot (getRobot());
	  robot->configValidators ()->validate (*randomConfig);
	    if(randomConfig->isValid ()) {
	    //Configuration is valid
	    hppDout (info, "Configuration valid.");
	    io_config = *randomConfig;
	    return true;
	  } else {
	    hppDout (info, "Configuration unvalid.");
	  }
	} else {
	  hppDout (info, "Random config: " << *randomConfig);
	}
      }
      hppDout (info, "Failed to generate goal configuration");
      return false;
    }

    void GoalConfigGenerator::configShooter
    (const ConfigShooterShPtr& configShooter)
    {
      configShooter_ = configShooter;
    }

    ConfigShooterConstShPtr GoalConfigGenerator::configShooter () const
    {
      return configShooter_;
    }
  } // namespace constrained
} //namespace hpp
