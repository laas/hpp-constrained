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

#ifndef HPP_CONSTRAINED_ROADMAP_BUILDER_HH
#define HPP_CONSTRAINED_ROADMAP_BUILDER_HH

#include <KineoWorks2/kwsDiffusingRdmBuilder.h>

#include <hpp/constrained/fwd.hh>
#include <hpp/constrained/config-extendor.hh>

namespace hpp {
  namespace constrained {
    template<class T = CkwsDiffusingRdmBuilder >
    class RoadmapBuilder : public T
    {
    public:
      /**
       * \brief Destructor.
       */
      ~RoadmapBuilder();
      /**
       * \brief Constructor.
       * @param i_roadmap The roadmap to build
       * @param i_extendor Configuration extendor used by roadmap builder
       * @param i_picker The picker used to pick diffusion nodes
       * @param i_shooter The shooter used to generate the random configurations
       * @return o_roadmapBuilder Newly allocated roadmap builder
       */
      static
      KIT_SHARED_PTR ( RoadmapBuilder<T> )
      create  ( const CkwsRoadmapShPtr& i_roadmap,
		ConfigExtendor* i_extendor,
		const CkwsDiffusionNodePickerShPtr &i_picker = CkwsPickerBasic::create(),
		const CkwsDiffusionShooterShPtr &i_shooter = CkwsShooterRoadmapBox::create() );

      /**
       * \brief Copy constructor.
       * @param inRdmBuilder The roadmap builder to copy.
       * @return o_roadmapBuilder New roadmap builder
       */
      static
      KIT_SHARED_PTR ( RoadmapBuilder<T> )
      createCopy ( const  KIT_SHARED_PTR_CONST ( RoadmapBuilder<T> ) &inRdmBuilder );

      /**
       * \brief Gets the config extendor used to generate constrained configurations
       * @return o_configExtendor Config extendor used by the roadmap builder
       */
      ConfigExtendor* getConfigExtendor();

    protected:
      /**
       * \brief Constructor
       * @param i_roadmap Roadmap to build
       * @param i_extendor Configuration extendor used by roadmap builder
       */
      RoadmapBuilder( const CkwsRoadmapShPtr& i_roadmap,
		      ConfigExtendor * i_extendor);

      /**
       * \brief Initialization
       * @param         i_weakPtr : weak pointer to the object itself
       * @return        KD_OK | KD_ERROR
       */
      ktStatus
      init ( const KIT_WEAK_PTR ( RoadmapBuilder<T> ) & i_weakPtr );

      /// Extend the roadmap from a node towards a configuration.
      /// @param i_node Node to extend from/to.
      /// @param i_cfg Configuration to extend to/from.
      /// @param i_direction ROADMAP_TO_NODE or NODE_TO_ROADMAP
      /// @param i_param parameters passed to the extend method
      /// @retval o_roadmapBoxWasIncreased whether roadmap box increased
      /// @return New node added to the roadmap (can be NULL)
      virtual CkwsNodeShPtr extend (const CkwsNodeShPtr& i_node,
				    const CkwsConfig& i_cfg,
				    CkwsRoadmapBuilder::EDirection i_direction,
				    const CkitParameterMapConstShPtr& i_params,
				    bool& o_roadmapBoxWasIncreased);

    private:
      /**
       * \brief Weak pointer to the object.
       */
      KIT_WEAK_PTR ( RoadmapBuilder<T> ) wkPtr_;

      /**
       * \brief Pointer to a config extendor.
       */
      ConfigExtendor * extendor_;

    };

    template<class T>
    RoadmapBuilder<T>::~RoadmapBuilder()
    {
    }

    template<class T>
    KIT_SHARED_PTR ( RoadmapBuilder<T> )
    RoadmapBuilder<T>::create  ( const CkwsRoadmapShPtr& i_roadmap,
				 ConfigExtendor * i_extendor,
				 const CkwsDiffusionNodePickerShPtr &i_picker,
				 const CkwsDiffusionShooterShPtr &i_shooter )
    {
      RoadmapBuilder<T> * newPtr = new RoadmapBuilder<T> ( i_roadmap, i_extendor );
      KIT_SHARED_PTR ( RoadmapBuilder<T> ) newShPtr ( newPtr );
      KIT_WEAK_PTR ( RoadmapBuilder<T> ) newWkPtr ( newShPtr );

      if ( newPtr->init ( newWkPtr ) != KD_OK )
	{
	  newShPtr.reset();
	  return newShPtr;
	}
      newPtr->diffusionNodePicker ( i_picker );
      newPtr->diffusionShooter ( i_shooter );

      return newShPtr;
    }

    template<class T>
    KIT_SHARED_PTR ( RoadmapBuilder<T> )
    RoadmapBuilder<T>::createCopy ( const  KIT_SHARED_PTR_CONST ( RoadmapBuilder<T> ) &inRdmBuilder )
    {
      return RoadmapBuilder<T>::create ( inRdmBuilder->roadmap(),
					 inRdmBuilder->getConfigExtendor(),
					 inRdmBuilder->diffusionNodePicker(),
					 inRdmBuilder->diffusionShooter() );
    }

    template<class T>
    ConfigExtendor *
    RoadmapBuilder<T>::getConfigExtendor()
    {
      return extendor_;
    }

    template<class T>
    RoadmapBuilder<T>::RoadmapBuilder( const CkwsRoadmapShPtr& i_roadmap,
				       ConfigExtendor * i_extendor):
      T ( i_roadmap ),
      extendor_ ( i_extendor )
    {
    }

    template<class T>
    ktStatus
    RoadmapBuilder<T>::init ( const KIT_WEAK_PTR ( RoadmapBuilder<T> ) & i_weakPtr )
    {
      ktStatus res = T::init ( i_weakPtr ) ;
      if ( res == KD_OK )
	wkPtr_ = i_weakPtr;
      return res;
    }

    template<class T> CkwsNodeShPtr
    RoadmapBuilder<T>::extend (const CkwsNodeShPtr& i_node,
			       const CkwsConfig& i_cfg,
			       CkwsRoadmapBuilder::EDirection i_direction,
			       const CkitParameterMapConstShPtr& i_params,
			       bool& o_roadmapBoxWasIncreased)
    {
      CkwsDeviceShPtr device = T::roadmap()->device();
      CkwsSteeringMethodShPtr sm = device->steeringMethod();
      CkwsValidatorDPCollisionShPtr dpValidator =
	device->directPathValidators()->retrieve<CkwsValidatorDPCollision> ();

      CkwsConfig startCfg = i_node->config ();

      CkwsConfigShPtr newConfig = extendor_->extendOneStep ( i_cfg,
							     startCfg);
      CkwsNodeShPtr currentNode = i_node;
      CkwsNodeShPtr lastAddedNode;

      bool configIsValid = true;
      bool dpIsValid = true;

      while ( newConfig
	      && configIsValid
	      && dpIsValid )
	{
	  configIsValid = newConfig->isValid();
	  if ( configIsValid ) {
	    if (newConfig->isEquivalent(currentNode->config())) {
	      newConfig = extendor_->extendOneStep ( i_cfg );
	    }
	    else {
	      CkwsDirectPathShPtr dp =
		sm->makeDirectPath(startCfg,*newConfig);
	      if (!dp) {
		dpIsValid = false;
	      }
	      else {
		dpValidator->validate(*dp);
		dpIsValid = dp->isValid();
	      }
	      if ( dpIsValid ) {
		lastAddedNode = T::roadmapNode ( *newConfig );

		if ( T::roadmapLink ( currentNode, lastAddedNode, dp) != KD_OK)
		  dpIsValid = false;

		currentNode = lastAddedNode;
		newConfig = extendor_->extendOneStep ( i_cfg );
	      }

	    }
	  }
	}
      return lastAddedNode;
    }


    typedef RoadmapBuilder<CkwsDiffusingRdmBuilder> DiffusingRoadmapBuilder;
    typedef RoadmapBuilder<CkwsIPPRdmBuilder> IppRoadmapBuilder;
    KIT_POINTER_DEFS ( DiffusingRoadmapBuilder );
    KIT_POINTER_DEFS ( IppRoadmapBuilder );

  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_ROADMAP_BUILDER_HH
