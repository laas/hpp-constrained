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

#include <hpp/constrained/config-projector.hh>

#include <hpp/model/joint.hh>

namespace hpp {
  namespace constrained {

    ConfigProjector::ConfigProjector(hpp::model::DeviceShPtr i_robot)
    {
      robot_ = i_robot;

      solver_ = new ChppGikSolver(*robot_);


      vectorN weightVector(robot_->numberDof());
      for(unsigned int i=0; i< robot_->numberDof(); i++)
	weightVector(i) = 1;
      solver_->weights(weightVector);

      soc_.clear();

      maxOptimizationSteps_ = 20; //Default value
      solveThreshold_ = 1e-3;
      progressThreshold_ = 1e-4;
    }

    ConfigProjector::~ConfigProjector()
    {
      delete solver_;
  
      for(unsigned int i = 0; i < soc_.size(); i++) {
	delete soc_[i];
      }
      soc_.clear();
    }

    void
    ConfigProjector::resetConstraints()
    {
      for(unsigned int i = 0; i < soc_.size(); i++) {
	delete soc_[i];
      }
      soc_.clear();
    }

    void
    ConfigProjector::setConstraints(std::vector<CjrlGikStateConstraint *> i_soc)
    {
      resetConstraints();
      for(unsigned int i=0; i<i_soc.size(); i++) {
	soc_.push_back(i_soc[i]);
      }
    }

    void
    ConfigProjector::addConstraint(CjrlGikStateConstraint* newConstraint)
    {
      soc_.push_back(newConstraint);
    }

    void
    ConfigProjector::removeLastConstraint()
    {
      soc_.pop_back();
    }

    void
    ConfigProjector::removeConstraint(CjrlGikStateConstraint* rmConstraint)
    {
      std::vector<CjrlGikStateConstraint*>::iterator it;
      for(it = soc_.begin(); it!= soc_.end(); it++) {
	if ( (*it) == rmConstraint ) soc_.erase(it);
      }
    }

    ChppGikSolver*
    ConfigProjector::getGikSolver()
    {
      return solver_;
    }

    ktStatus
    ConfigProjector::project(CkwsConfig & io_config)
    {
      robot_->hppSetCurrentConfig(io_config);
  
      std::vector<double> constraintValues(soc_.size(), std::numeric_limits<double>::infinity());
      bool didOneConstraintDecrease = true;
      bool optimReturnOK = true;
      unsigned int n = 0; //Optimization iterations

      while ( (n < maxOptimizationSteps_) 
	      && didOneConstraintDecrease
	      && optimReturnOK
	      && (!areConstraintsSatisfied()) ) {
	didOneConstraintDecrease = false;
	for (unsigned int i=0; i<soc_.size();i++) {
	  double value = norm_2(soc_[i]->value());
	  if (value > solveThreshold_) { //This constraint is not solved, has it decreased?
	    didOneConstraintDecrease = 
	      didOneConstraintDecrease || (value < constraintValues[i] - progressThreshold_);
	  }
	  constraintValues[i] = value;
	}
	optimReturnOK = optimizeOneStep();
	n++;
      }
    
      if (!areConstraintsSatisfied()) {
	return KD_ERROR;
      }
  
      vectorN jrlCfg = robot_->currentConfiguration();
      robot_->hppSetCurrentConfig(jrlCfg);
      robot_->getCurrentConfig(io_config);
      return KD_OK;
    }

    void
    ConfigProjector::setMaxNumberOptimizationSteps(unsigned int i_nbSteps)
    {
      maxOptimizationSteps_ = i_nbSteps;
    }

    unsigned int
    ConfigProjector::getMaxNumberOptimizationSteps()
    {
      return maxOptimizationSteps_ ;
    }

    hpp::model::DeviceShPtr
    ConfigProjector::getRobot()
    {
      return robot_;
    }

    bool
    ConfigProjector::areConstraintsSatisfied()
    {
      for(unsigned int i=0; i<soc_.size(); i++){
	soc_[i]->computeValue();
	double value = norm_2(soc_[i]->value());
	if (value > solveThreshold_) {
	  return false;
	}
      }
      return true;
    }

    bool
    ConfigProjector::optimizeOneStep()
    {
      CjrlJoint * rootJoint = robot_->getRootJoint()->jrlJoint();

      /* Prepare the linear system */
      for(unsigned int i=0; i<soc_.size(); i++) {
	soc_[i]->jacobianRoot(*rootJoint);
	soc_[i]->computeJacobian();
	soc_[i]->computeValue();
      }

      solver_->solve(soc_);

      const vectorN newConfig = solver_->solution();
  
      if ( isnan(norm_2(newConfig)) )
	return false;

      robot_->hppSetCurrentConfig(newConfig);
  
      return true;
    }

  } //end of namespace constrained
} //end of namespace hpp
