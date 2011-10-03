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

#include <KineoWorks2/kwsConfig.h>

# include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/gik/core/solver.hh>
#include <hpp/model/device.hh>

#include <hpp/constrained/fwd.hh>

namespace hpp {
  namespace constrained {
    class ConfigProjector
    {
    public:
      /**
       * \brief Constructor
       */
      ConfigProjector(hpp::model::DeviceShPtr i_robot);

      /**
       * \brief Destructor
       */
      ~ConfigProjector();
 
      /**
       * \brief Empty the stack of constraints and delete the corresponding objects.
       */
      void
      resetConstraints();

      /**
       * \brief Sets the stack of constraints.
       */
      void
      setConstraints(std::vector<CjrlGikStateConstraint *> i_soc);

      /**
       * \brief Adds a constraint to soc_.
       */
      void
      addConstraint(CjrlGikStateConstraint* newConstraint);

      /**
       * \brief Removes the last constraint from soc_.
       */
      void
      removeLastConstraint();

      /**
       * \brief Removes a constraint from soc_.
       */
      void
      removeConstraint(CjrlGikStateConstraint* rmConstraint);

      /** 
       * \brief Get the inverse kinematics solver.
       */
      ChppGikSolver*
      getGikSolver();

      /**
       * \brief
       * Projects a config on the constrained manifold.
       * @param io_config input/output configuration
       * @return KD_OK | KD_ERROR  the projection might fail
       */
      ktStatus
      project(CkwsConfig & io_config);

      /**
       * \brief Sets the maximum number of optimization steps performed when projecting a configuration
       * @param i_nbSteps New maximum value
       */
      void
      setMaxNumberOptimizationSteps(unsigned int i_nbSteps);

      /**
       * \brief Gets the maximum number of optimization steps performed when projecting a configuration
       * @return o_nbSteps Maximum value
       */
      unsigned int
      getMaxNumberOptimizationSteps();

      /**
       * \brief Gets the associated robot.
       * @return o_robot Associated robot
       */
      hpp::model::DeviceShPtr
      getRobot();

    protected:
      /**
       * \brief Checks if the stack of constraints is solved in the current robot configuration.
       * @return true | false
       */
      bool 
      areConstraintsSatisfied();

      /**
       * \brief Performs one step of optimization. Returns false if no progress is made.
       * @return true | false
       */
      virtual bool
      optimizeOneStep();
      
    protected:
      /**
       * \brief Pointer to robot
       */
      hpp::model::DeviceShPtr robot_;

      /**
       * \brief Inverse kinematic solver
       */
      ChppGikSolver* solver_;

      /**
       * \brief Stack of constraints solved by the solver
       */
      std::vector<CjrlGikStateConstraint*> soc_;

      /**
       * \brief Maximum number of optimization steps
       */
      unsigned int maxOptimizationSteps_;
  
      /**
       * \brief Threshold under which a constraint is  considered solved
       */
      double solveThreshold_;

      /**
       * \brief Threshold under which a constraint is  considered as progressing
       */
      double progressThreshold_;
    };
  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_CONFIG_PROJECTOR_HH
