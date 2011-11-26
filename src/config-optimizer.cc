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

#include <map>

#include <KineoWorks2/kwsDiffusionShooter.h>

#include <hpp/constrained/config-optimizer.hh>

namespace hpp {
  namespace constrained {

    ConfigOptimizer::ConfigOptimizer(hpp::model::DeviceShPtr robot,
				     ConfigExtendor * i_extendor,
				     CkwsConfigShPtr i_goalConfig):
      robot_(robot),
      extendor_(i_extendor),
      goalConfig_(i_goalConfig)
    {
      step_ = 0.01; //Default value
      distance_ = CkwsDistance::create();
      nbRandCfgs_ = 20; //Default value
    }

    ConfigOptimizer::~ConfigOptimizer()
    {
    }

    CkwsConfigShPtr
    ConfigOptimizer::getGoalConfig()
    {
      return goalConfig_;
    }

    ConfigExtendor *
    ConfigOptimizer::getConfigExtendor()
    {
      return extendor_;
    }
    
    CkwsPathShPtr
    ConfigOptimizer::optimizeConfig(CkwsConfigShPtr i_cfg)
    {
      //DEBUG
      std::cout << "Entering optimizeConfig()..." << std::endl;
      std::cout << "\tInitial cost: " << cost(i_cfg) << std::endl;

      CkwsPathShPtr resultPath = CkwsPath::create(robot_);

      //First, try to extend towards goalConfig_
      // Stop if: (1) a collision occurs, OR
      //          (2) we reach a local minimum

      CkwsSteeringMethodShPtr sm = robot_->steeringMethod();
       CkwsValidatorDPCollisionShPtr dpValidator = 
	robot_->directPathValidators()->retrieve<CkwsValidatorDPCollision> ();

       CkwsConfigShPtr newConfig = extendor_->extendOneStep ( *goalConfig_,
							      *i_cfg);
       
       CkwsConfigShPtr startCfg = i_cfg;
       bool configIsValid = true;
       bool dpIsValid = true;

      while ( newConfig
	      && configIsValid
	      && dpIsValid ) 
	{
	  configIsValid = newConfig->isValid();
	  if ( configIsValid ) {
	    if (newConfig->isEquivalent(*startCfg)) {
		configIsValid = false;
	    }
	    else {
	      CkwsDirectPathShPtr dp = 
		sm->makeDirectPath(*startCfg,*newConfig);
	      if (!dp) {
		dpIsValid = false;
	      }
	      else {
		dpValidator->validate(*dp);
		dpIsValid = dp->isValid();
	      } 
	      if ( dpIsValid ) {
		if ( resultPath->appendDirectPath(dp) != KD_OK )
		  dpIsValid = false;

		startCfg = newConfig;
		newConfig = extendor_->extendOneStep ( *i_cfg );
	      }
	    }
	  }
	}


      //DEBUG
      if (resultPath->isEmpty()) {
	std::cout << "\tNo direct extension" << std::endl;
      }
      else {
	std::cout << "\tCost after direct extension: " << cost(resultPath->configAtEnd()) << std::endl ;
      }

      if (!newConfig) { //No extension was found, we have reached a collision free local optimum

	//DEBUG
	std::cout << "\tReturn after direct extension" << std::endl;

	return resultPath;
      }

      //Try to extend in random directions until no improvement is found

      CkwsConfigShPtr currentConfig = (resultPath->isEmpty())? i_cfg : resultPath->configAtEnd();
      double currentCost = cost(currentConfig);
      bool didProgress = true;

      //DEBUG
      unsigned int nbOptSteps = 0;

      while (didProgress) {

	//DEBUG
	nbOptSteps++;

	didProgress = false;
	ConfigQueue q;

	shootRandomConfigsWithCost(currentConfig,q,nbRandCfgs_);

	while ( !q.empty() && !didProgress ) {
	  CkwsConfigShPtr randConfig = q.top().first;
	  q.pop();

	  CkwsConfigShPtr newConfig = extendor_->extendOneStep ( *randConfig, *currentConfig);
	  if (newConfig->isValid() && (cost(newConfig) < currentCost) ){
	    if (!(newConfig->isEquivalent(*currentConfig))) {
	      CkwsDirectPathShPtr dp = sm->makeDirectPath(*currentConfig, *newConfig);
	      dpIsValid = false;
	      if (dp) {
		dpValidator->validate(*dp);
		dpIsValid = dp->isValid();
	      } 
	      if ( dpIsValid ) {
		if ( resultPath->appendDirectPath(dp) == KD_OK ) {
		  didProgress = true;
		  currentConfig = newConfig;
		  currentCost = cost(currentConfig);
		}
	      }
	    }
	  }
	}
      }

      //DEBUG
      std::cout << "\tAfter " << nbOptSteps << " steps, cost is: " << cost(resultPath->configAtEnd()) << std::endl ;

      return resultPath;
    }
    
    double
    ConfigOptimizer::cost(CkwsConfigShPtr i_config)
    {
      CkwsConfigShPtr tmpCfg = CkwsConfig::create(*i_config);
      for(unsigned int i=0;i<6;i++) tmpCfg->dofValue(i, goalConfig_->dofValue(i));
      return distance_->distance(*i_config,*goalConfig_);
    }

    void
    ConfigOptimizer::shootRandomConfigsWithCost(CkwsConfigShPtr i_config,
						ConfigQueue & o_queue,
						unsigned int nbConfigs)
    {
      for(unsigned int i=0;i<nbConfigs;i++) {
	CkwsConfigShPtr randomConfig = CkwsConfig::create(*i_config);
	CkwsDiffusionShooter::gaussianShoot(*randomConfig,step_);
	double dist = cost(randomConfig);
	ConfigWithCost cfg(randomConfig,dist);
	o_queue.push(cfg);
      }
    }

 } //end of namespace constrained
} //end of namespace hpp
