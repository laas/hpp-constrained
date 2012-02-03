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

    void GoalConfigGenerator::generate (const CkwsConfig& io_config)
    {
      bool success = false;
      unsigned int nb_try=0;
      unsigned int nb_maxTry=10;
      while ( (nb_try < nb_maxTry) && (!success) ) { //Shoot config
	nb_try++;
	CkwsConfigShPtr randomConfig = CkwsConfig::create(io_config);
	configShooter ()->shoot (*randomConfig);

	if (project(*randomConfig) == KD_OK) {
	  //Projection worked
	  hppDout (info, "config: " << *randomConfig);
	  model::DeviceShPtr robot (getRobot());
	  robot->setCurrentConfig(*randomConfig);
	  if(!robot->collisionTest()) { //Configuration is collision free
	    return;
	  }
	} else {
	  hppDout (info, "Random config: " << *randomConfig);
	}
      }
      if (!success) {
	HPP_THROW_EXCEPTION (Exception,
			     "Failed to generate goal configuration");
      }
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
