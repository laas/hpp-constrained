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

#ifndef HPP_CONSTRAINED_KWS_CONSTRAINT_HH
#define HPP_CONSTRAINED_KWS_CONSTRAINT_HH

#include <KineoWorks2/kwsConstraint.h>

#include <hpp/constrained/fwd.hh>

namespace hpp {
  namespace constrained {
    class KwsConstraint : public CkwsConstraint
    {
    public:
      /**
       *  \brief Destructor.
       */
      ~KwsConstraint();

      /**
       * \brief Constructor.
       * @param  i_projector The configuration projector used by the constraint
       * @return o_kwsConstraint Newly allocated kineo constraint
       */
      static 
      KwsConstraintShPtr
      create (const std::string & i_name,
	      ConfigProjector * i_projector);

      /**
       * \brief Copy constructor.
       * @param i_KwsConstraint The kineo constraint to copy
       * @return o_kwsconstraint New kineo constraint
       */
      static 
      KwsConstraintShPtr
      createCopy ( const KwsConstraintConstShPtr &i_KwsConstrain);

      // inherited -- for doc see parent class
      virtual CkwsValidatorShPtr clone() const;

      /**
       * \brief Gets the configuration projector.
       * @return o_configProjector Configuration projector
       */
      ConfigProjector *
      getConfigProjector() const;

    protected:
      /**
       * \brief Constructor
       * @param  i_projector The configuration projector used by the constraint
       */
      KwsConstraint(const std::string & i_name,
		    ConfigProjector * i_projector);

      /**
       * \brief Initialization
       * @param         i_weakPtr : weak pointer to the object
       * @return        KD_OK | KD_ERROR
       */
      ktStatus 
      init ( const KwsConstraintWkPtr  & i_weakPtr );

    protected:
      virtual ktStatus
      doApply (CkwsConfig& io_config,
	       const CkitParameterMapConstShPtr& parameterMap) const;

    private:
      /**
       * \brief Weak pointer to the object.
       */
      KwsConstraintWkPtr wkPtr_;

      /**
       * \brief Pointer to a config projector.
       */ 
      ConfigProjector * projector_;

    };
  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_KWS_CONSTRAINT_HH
