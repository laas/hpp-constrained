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

#include <hpp/util/debug.hh>
#include <hpp/model/joint.hh>
#include <kwsIO/kwsioConfig.h>

#include "../src/constraint-set.hh"

namespace hpp {
  namespace constrained {

    ConfigProjector::ConfigProjector(hpp::model::DeviceShPtr i_robot) :
      robot_ (i_robot), solver_ (new ChppGikSolver(*robot_)),
      soc_ (), maxOptimizationSteps_ (20), solveThreshold_ (1e-3),
      progressThreshold_ (1e-4), cache_ (),
      constraintSet_ (new ConstraintSet (*robot_.get ()))

    {
      vectorN weightVector(robot_->numberDof());
      for(unsigned int i=0; i< robot_->numberDof(); i++)
	weightVector(i) = 1;
      solver_->weights(weightVector);
      // Insert a constraint set as unique task. All other tasks will be
      // inserted in the constraint set.
      soc_.push_back (constraintSet_);
    }

    ConfigProjector::~ConfigProjector()
    {
      resetConstraints();
      delete solver_;
      soc_.clear();
      cache_.clear();
    }

    void
    ConfigProjector::resetConstraints()
    {
      constraintSet_->resetConstraints ();
      cache_.clear();
    }

    void
    ConfigProjector::setConstraints(std::vector<CjrlGikStateConstraint *> i_soc)
    {
      resetConstraints();
      for(unsigned int i=0; i<i_soc.size(); i++) {
	addConstraint (i_soc [i]);
      }
    }

    std::vector<CjrlGikStateConstraint *>
    ConfigProjector::getConstraints()
    {
      return constraintSet_->getConstraints ();
    }

    void
    ConfigProjector::addConstraint(CjrlGikStateConstraint* newConstraint)
    {
      hppDout (info, "Constraint dimension: " << newConstraint->dimension ());
      constraintSet_->addConstraint (newConstraint);
      hppDout (info, "Constraint set dimension: " <<
	       constraintSet_->dimension ());
    }

    void
    ConfigProjector::removeLastConstraint()
    {
      constraintSet_->removeLastConstraint ();
    }

    void
    ConfigProjector::removeConstraint(CjrlGikStateConstraint* rmConstraint)
    {
      constraintSet_->removeConstraint (rmConstraint);
    }

    ChppGikSolver*
    ConfigProjector::getGikSolver()
    {
      return solver_;
    }

    /// In case of failure, the input/output configuration is not
    /// modified.
    ktStatus
    ConfigProjector::project(vectorN & jrlConfig)
    {
      robot_->currentConfiguration(jrlConfig);
      robot_->computeForwardKinematics();

      std::vector<double> constraintValues
	(soc_.size(),std::numeric_limits<double>::infinity());
      int didOneConstraintDecrease = 2;
      bool optimReturnOK = true;
      unsigned int n = 0; //Optimization iterations
      double lambda = .1;
      double lambdaMax = .95;
      bool satisfied = false;

      while ( (n < maxOptimizationSteps_)
	      && didOneConstraintDecrease
	      && optimReturnOK
	      && (!(satisfied = areConstraintsSatisfied())) ) {
	didOneConstraintDecrease --;
	for (unsigned int i=0; i<soc_.size();i++) {
	  double value = norm_2(soc_[i]->value());
	  if (value > solveThreshold_) {
	    //This constraint is not solved, has it decreased?
	    if (value < constraintValues[i]) didOneConstraintDecrease = 3;
	  }
	  constraintValues[i] = value;
	}
	optimReturnOK = optimizeOneStep(lambda);
	// make lambda tend to lambdaMax
	hppDout (info, lambda);
	lambda = lambdaMax - .8*(lambdaMax - lambda);
	n++;
      }

      if (!areConstraintsSatisfied()) {
#ifdef HPP_DEBUG
	std::vector <double> kwsDofVector (robot_->countDofs ());
	robot_->jrlDynamicsToKwsDofValues(robot_->currentConfiguration(),
					  kwsDofVector);
	CkwsConfig kwsConfig (robot_, kwsDofVector);
#endif
	hppDout (info, "Projection failed: " << kwsConfig);
	return KD_ERROR;
      }
      hppDout (info, "Projection succeeded.");

      jrlConfig = robot_->currentConfiguration();
      return KD_OK;
    }

    /// In case of failure, the input/output configuration is not
    /// modified.
    ktStatus
    ConfigProjector::project(CkwsConfig & io_config)
    {
      if (cache_.find(io_config)!=cache_.end())
	return KD_OK;

      std::vector<double> dofs;
      io_config.getDofValues(dofs);
      vectorN jrlCfg(dofs.size());
      robot_->kwsToJrlDynamicsDofValues(dofs,jrlCfg);

      if (project(jrlCfg) != KD_OK)
	return KD_ERROR;

      robot_->jrlDynamicsToKwsDofValues(jrlCfg,dofs);

      io_config.setDofValues(dofs);
      cache_.insert(io_config);
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
      bool res = true;
      for(unsigned int i=0; i<soc_.size(); i++){
	soc_[i]->computeValue();
	double value = norm_2(soc_[i]->value());
	if (value > solveThreshold_) {
	  res = false;
	}
      }
      return res;
    }

    bool
    ConfigProjector::optimizeOneStep (double lambda)
    {
      CjrlJoint * rootJoint = robot_->getRootJoint()->jrlJoint();

      /* Prepare the linear system */
      for(unsigned int i=0; i<soc_.size(); i++) {
	soc_[i]->jacobianRoot(*rootJoint);
	soc_[i]->computeJacobian();
	soc_[i]->computeValue();
      }

      solver_->solve(soc_, lambda);
      const vectorN newConfig = solver_->solution();

      assert (!isnan(norm_2(newConfig)) );

      robot_->currentConfiguration(newConfig);
      robot_->computeForwardKinematics();
      return true;
    }

  } //end of namespace constrained
} //end of namespace hpp
