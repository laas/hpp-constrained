// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-constrained.
//
// hpp-constrained is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-constrained is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-constrained.  If not, see <http://www.gnu.org/licenses/>.

#include <limits>

#include <hpp/util/debug.hh>
#include <kwsIO/kwsioConfig.h>

#include "hpp/constrained/config-extendor.hh"

namespace hpp {
  namespace constrained {

    ConfigExtendor::ConfigExtendor(hpp::model::DeviceShPtr robot):
      ConfigProjector(robot), configConstraintPushed_ (false)
    {
      vectorN config = robot->currentConfiguration();
      vectorN mask (robot->numberDof(),1);
      configConstraint_ =
	new ChppGikConfigurationConstraint(*robot, config, mask);

    }

    ConfigExtendor::~ConfigExtendor()
    {
      if (configConstraint_) delete configConstraint_;
    }

    CkwsConfigShPtr
    ConfigExtendor::extendOneStep(const CkwsConfig & extendTo,
				  const CkwsConfig & extendFrom)
    {
      /* Putting dynamic robot in extendFrom configuration */
      std::vector<double> dofs;
      extendFrom.getDofValues(dofs);
      vectorN jrlCfg(dofs.size());
      robot_->kwsToJrlDynamicsDofValues(dofs,jrlCfg);
      robot_->currentConfiguration(jrlCfg);
      robot_->computeForwardKinematics();

      return (extendOneStep(extendTo));
    }

    CkwsConfigShPtr
    ConfigExtendor::extendOneStep(const CkwsConfig & extendTo)
    {
      CkwsConfigShPtr resCfg;

      vectorN currentConfig(robot_->numberDof());
      currentConfig = robot_->currentConfiguration();

      std::vector<double> kwsdofs;
      extendTo.getDofValues(kwsdofs);
      vectorN jrlextendTo(kwsdofs.size());
      robot_->kwsToJrlDynamicsDofValues(kwsdofs,jrlextendTo);

      configConstraint_->target(jrlextendTo);
      configConstraint_->computeValue();
      double initialValue = norm_2(configConstraint_->value());

      pushConfigConstraint ();
      if(!optimizeOneStep(1.)) {
	return resCfg;
      }
      popConfigConstraint ();

      currentConfig = robot_->currentConfiguration();
      if ( project(currentConfig) != KD_OK) {
#ifdef HPP_DEBUG	
	std::vector <double> dofs (robot_->countDofs ());
	robot_->jrlDynamicsToKwsDofValues (currentConfig, dofs);
	CkwsConfig cfg(robot_, dofs);
#endif
	hppDout (info, "failed to project configuration: " << cfg);
	return resCfg;
      }
      configConstraint_->computeValue();
      double newValue  = norm_2(configConstraint_->value());

      if (newValue > initialValue - progressThreshold_) {
	hppDout (info, "distance to config did not decrease.");
	return resCfg;
      }

      if (!areConstraintsSatisfied()) {
	return resCfg;
      }

      robot_->jrlDynamicsToKwsDofValues(currentConfig,kwsdofs);

      resCfg = CkwsConfig::create(robot_,kwsdofs);
      cache_.insert(*resCfg);

      return resCfg;
    }

    void ConfigExtendor::pushConfigConstraint ()
    {
      assert (!configConstraintPushed_);
      soc_.push_back (configConstraint_);
      configConstraintPushed_ = true;
    }
    void ConfigExtendor::popConfigConstraint ()
    {
      assert (configConstraintPushed_);
      soc_.pop_back ();
      configConstraintPushed_ = false;
    }

  } //end of namespace constrained
} //end of namespace hpp
