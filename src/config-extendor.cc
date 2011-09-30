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

#include <hpp/constrained/config-extendor.hh>

namespace hpp {
  namespace constrained {

    ConfigExtendor::ConfigExtendor(hpp::model::DeviceShPtr robot):
      ConfigProjector(robot)
    {
      extensionStep_=1e-2; //Default value

      vectorN config = robot->currentConfiguration();
      vectorN mask (robot->numberDof(),1);
      for(unsigned int i=0; i<6; i++){
	mask(i)=0;
      }
      configConstraint_ = new ChppGikConfigurationConstraint(*robot, config, mask);
    }

    ConfigExtendor::~ConfigExtendor()
    {
    }

    CkwsConfigShPtr
    ConfigExtendor::extendOneStep(const CkwsConfig & extendTo,
				  const CkwsConfig & extendFrom)
    {
      robot_->hppSetCurrentConfig(extendFrom);
      return (extendOneStep(extendTo));
    }

    CkwsConfigShPtr
    ConfigExtendor::extendOneStep(const CkwsConfig & extendTo)
    {
      CkwsConfigShPtr currentConfig;
      robot_->getCurrentConfig(currentConfig);
      std::vector<double> deltaConfig;
      extendTo.getDofValues(deltaConfig);

      for(unsigned int i=0; i<deltaConfig.size(); i++){
	deltaConfig[i]-=currentConfig->dofValue(i);
      }

      double delta = robot_->distance()->distance(*currentConfig,extendTo);
      double epsilon = delta/extensionStep_;
  
      std::vector<double> targetConfig(deltaConfig.size());
      vectorN jrlTargetConfig(deltaConfig.size());

      for(unsigned int i=0; i<deltaConfig.size(); i++){
	double dofValue = currentConfig->dofValue(i);
	dofValue += epsilon*deltaConfig[i];
	targetConfig[i]=dofValue;
      }

      robot_->kwsToJrlDynamicsDofValues(targetConfig,jrlTargetConfig);
  
      configConstraint_->target(jrlTargetConfig);
      configConstraint_->computeValue();

      addConstraint(configConstraint_);


      unsigned int n = 0;

      double constraintValue = std::numeric_limits<double>::infinity();
      bool didConstraintDecrease = true;
      bool isConstraintSolved = false;
      bool optimReturnOK = true;

      while ( (n < maxOptimizationSteps_)
	      && (!isConstraintSolved)
	      && didConstraintDecrease 
	      && optimReturnOK ) {
	configConstraint_->computeValue();
	double value = norm_2(configConstraint_->value());
	if (value < solveThreshold_) { 
	  isConstraintSolved = true;
	}
	else {
	  didConstraintDecrease = ( value < constraintValue - progressThreshold_);
	  constraintValue = value;
	}
	optimReturnOK = optimizeOneStep();
	n++;
      }
  
      removeLastConstraint();

      if (!areConstraintsSatisfied()) {
	currentConfig.reset();
	return currentConfig;
      }

      robot_->getCurrentConfig(*currentConfig);
      return currentConfig;
    }


    void
    ConfigExtendor::setExtensionStep(double i_extensionStep)
    {
      extensionStep_ = i_extensionStep;
    }

    double
    ConfigExtendor::getExtensionStep()
    {
      return extensionStep_;
    }

  } //end of namespace constrained
} //end of namespace hpp
