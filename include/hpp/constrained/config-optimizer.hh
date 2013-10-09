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

#ifndef HPP_CONSTRAINED_CONFIG_OPTIMIZER_HH
#define HPP_CONSTRAINED_CONFIG_OPTIMIZER_HH

#include <utility>
#include <queue>
#include <vector>

# include <KineoWorks2/kwsPathPlanner.h>

#include "hpp/constrained/fwd.hh"
#include "hpp/constrained/config-extendor.hh"

HPP_KIT_PREDEF_CLASS (CkwsPath);

namespace hpp {
  namespace constrained {
    /**
     * \brief Optimize a config while staying on a constrained submanifold
     * and ensuring collision avoidance. The optimization consists in getting
     * closer to a reference configuration.
     */

    typedef std::pair<CkwsConfigShPtr, double> ConfigWithCost;

    struct compareConfigsWithCost {
      bool operator() (const ConfigWithCost& cfg1,
		       const ConfigWithCost& cfg2) const
      {
	return (cfg1.second > cfg2.second);
      }
    };

    typedef std::priority_queue <ConfigWithCost,std::vector <ConfigWithCost>,
				 compareConfigsWithCost> ConfigQueue;

    HPP_KIT_PREDEF_CLASS (ConfigOptimizer);
    class ConfigOptimizer : public CkwsPathPlanner
    {
    public:

      /// Return shared pointer to new object.
      /// @param robot
      /// @param i_extendor Configuration extendor defining the constrained
      /// manifold
      /// @param i_goalConfig Reference configuration
      static ConfigOptimizerShPtr create (hpp::model::DeviceShPtr robot,
					  ConfigExtendor* i_extendor,
					  CkwsConfigShPtr i_goalConfig);
      /**
       * \brief Destructor
       */
      ~ConfigOptimizer();

      virtual ktStatus doPlan (const CkwsPathConstShPtr& i_path,
			       const std::vector< bool >& i_stableConfigs,
			       CkwsPathShPtr& o_path);

      /**
       * \brief Optimize a configuration
       * @param i_cfg Configuration to optimize
       * @return o_path Collision-free path linking i_cfg
       * to the optimized config. Can be empty if no optimization was found.
       */
      CkwsPathShPtr
      optimizeConfig(CkwsConfigShPtr i_cfg);

      /**
       * \brief Get the reference configuration
       * @return o_config
       */
      CkwsConfigShPtr
      getGoalConfig();

      /**
       * \brief Gets the config extendor defining the constrained submanifold
       * @return o_configExtendor
       */
      ConfigExtendor *
      getConfigExtendor();

    protected:

      /// Constructor
      /// @param robot
      /// @param i_extendor Configuration extendor defining the constrained
      /// manifold
      /// @param i_goalConfig Reference configuration
      ConfigOptimizer (hpp::model::DeviceShPtr robot,
		       ConfigExtendor* i_extendor,
		       CkwsConfigShPtr i_goalConfig);

      void init (ConfigOptimizerWkPtr wkPtr);

      /// Shoot configuration in a neighbourhood of an input configuration.

      /// Computes distance to goalConfig_
      /// @param i_config Configuration around which we shoot
      /// @param o_queue result priority queue
      /// @param nbConfigs Number of configs to generate
      void
      shootRandomConfigsWithCost(CkwsConfigShPtr i_config,
				 ConfigQueue & o_queue,
				 unsigned int nbConfigs);

      /**
       * \brief Computes and returns the cost of a configuration
       */
      double
      cost(CkwsConfigShPtr i_config);

    private:
      /**
       * \brief Pointer to robot.
       */
      hpp::model::DeviceShPtr robot_;

      /**
       * \brief Pointer to config extendor.
       */
      ConfigExtendor * extendor_;

      /**
       * \brief Pointer to reference configuration
       */
      CkwsConfigShPtr goalConfig_;

      /**
       * \brief Extension step in local optimization
       */
      double step_;

      /**
       * \brief Kineo Distance operator
       */
      CkwsMetricShPtr distance_;

     /**
       * \brief Number of random configurations shot at each stage of optimization
       */
      unsigned int nbRandCfgs_;

      double progressThreshold_;

      unsigned int maxOptimizationSteps_;

    };
  } //end of namespace constrained
} //end of namespace hpp

#endif //HPP_CONSTRAINED_CONFIG_OPTIMIZER_HH
