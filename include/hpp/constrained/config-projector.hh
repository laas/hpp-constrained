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

#ifndef HPP_CONSTRAINED_CONFIG_PROJECTOR_HH
#define HPP_CONSTRAINED_CONFIG_PROJECTOR_HH

#include <vector>
#include <set>

#include <KineoWorks2/kwsConfig.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/gik/core/solver.hh>
#include <hpp/model/device.hh>

#include <hpp/constrained/fwd.hh>

namespace hpp {
  namespace constrained {

    struct cfgcomp {
      bool operator() (const CkwsConfig & cfg1,const CkwsConfig & cfg2) const
      {
	cfg1.isValidItself (); cfg2.isValidItself ();
	if (cfg1.isEquivalent (cfg2)) return false;
	for (unsigned int i=0;i<cfg1.device()->countDofs();i++) {
	  if ( cfg1.dofValue(i) < cfg2.dofValue(i) )
	    return true;
	}
	return false;
      }
    };

    /// Projector onto the sub-manifold defined by non linear constraints

    /// Objects of this class project a given configuration of a robot onto the
    /// sub-manifold of the configuration space defined by some non-linear
    /// constraints.

    /// The non-linear constraints are handled by methods:
    /// ConfigProjector::setConstraints,
    /// ConfigProjector::addConstraint,
    /// ConfigProjector::resetConstraints,
    /// ConfigProjector::removeLastConstraint.

    /// The constraints are gathered into an object of type ConstraintSet and
    /// solved by ChppGikSolver solver as a single constraint: no
    /// prioritization.
    class ConfigProjector
    {
    public:

      typedef std::set<CkwsConfig,cfgcomp> cache_t;
      /// Constructor
      ConfigProjector(hpp::model::DeviceShPtr i_robot);

      /// Destructor
      virtual ~ConfigProjector();

      /// Empty the stack of constraints and delete the corresponding objects.
      virtual void resetConstraints();

      /// Sets the stack of constraints.
      virtual void setConstraints(std::vector<CjrlGikStateConstraint *> i_soc);

      /// Get the stack of constraints.
      virtual std::vector<CjrlGikStateConstraint *> getConstraints();

      /// Add a constraint.

      /// All constraints have the same priority.
      virtual void addConstraint(CjrlGikStateConstraint* newConstraint);

      /// Removes the last constraint from soc_.
      virtual void removeLastConstraint();

      /// Removes a constraint from soc_.
      virtual void removeConstraint(CjrlGikStateConstraint* rmConstraint);

      /// Get the inverse kinematics solver.
      ChppGikSolver* getGikSolver();

      /// Project a config on the constrained manifold.
      /// @param io_config input/output configuration
      /// @return KD_OK | KD_ERROR  the projection might fail
      /// \note In case of failure, the input/output configuration is not
      /// modified.
      ktStatus project(CkwsConfig & io_config);


      /// Projects a config on the constrained manifold.
      /// @param io_config input/output configuration in jrl-dynamics manner
      /// @return KD_OK | KD_ERROR  the projection might fail
      /// \note In case of failure, the input/output configuration is not
      /// modified.
      ktStatus project(vectorN & jrlConfig);

      /// Sets the maximum number of iterations when projecting a configuration
      /// @param i_nbSteps New maximum value
      void setMaxNumberOptimizationSteps(unsigned int i_nbSteps);

      /// Gets the maximum number of iterations when projecting a configuration
      /// @return o_nbSteps Maximum value
      unsigned int getMaxNumberOptimizationSteps();

      /// Gets the associated robot.
      /// @return o_robot Associated robot
      hpp::model::DeviceShPtr getRobot();

    protected:
      /// Checks if the set of constraints is satisfied
      /// @return true | false
      /// Configuration is robot current configuration.
      bool areConstraintsSatisfied();

      /// Performs one step of optimization.
      /// \param lambda Coefficient to apply to configuration update <= 1.
      /// @return true | false if no progress is made.
      virtual bool optimizeOneStep (double lambda);

    protected:
      /// Pointer to robot
      hpp::model::DeviceShPtr robot_;

      /// Inverse kinematic solver
      ChppGikSolver* solver_;

      /// Stack of constraints solved by the solver
      std::vector<CjrlGikStateConstraint*> soc_;

      /// Maximum number of optimization steps
      unsigned int maxOptimizationSteps_;

      /// Threshold under which a constraint is  considered solved
      double solveThreshold_;

      /// Threshold under which a constraint is  considered as progressing
      double progressThreshold_;

      cache_t cache_;
      /// Set of constraints with the same priority level
      ConstraintSet* constraintSet_;
    };
  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_CONFIG_PROJECTOR_HH
