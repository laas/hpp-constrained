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

#include <hpp/constrained/kws-constraint.hh>
#include <hpp/constrained/config-projector.hh>


namespace hpp {
  namespace constrained {
    KwsConstraint::~KwsConstraint()
    {
    }

    KwsConstraintShPtr
    KwsConstraint::create (const std::string & i_name,
			   ConfigProjector * i_projector)
    {
      KwsConstraint * newPtr = new KwsConstraint(i_name, i_projector);
      KwsConstraintShPtr newShPtr ( newPtr );
      KwsConstraintWkPtr wkPtr ( newShPtr );
      if ( newPtr->init(wkPtr) != KD_OK ) newShPtr.reset();
      return newShPtr;
    }

    KwsConstraintShPtr
    KwsConstraint::createCopy ( const KwsConstraintConstShPtr &i_KwsConstraint)
    {
      return create(i_KwsConstraint->name(),i_KwsConstraint->getConfigProjector());
    }
    
    CkwsValidatorShPtr KwsConstraint::clone() const
    {
      return  create(name(),getConfigProjector());
    }

    ConfigProjector *
    KwsConstraint::getConfigProjector() const
    {
      return projector_;
    }

    KwsConstraint::KwsConstraint(const std::string & i_name,
				 ConfigProjector * i_projector):
      CkwsConstraint(i_name,i_projector->getRobot()),
      projector_(i_projector)
    {
    }

    ktStatus
    KwsConstraint::init ( const KwsConstraintWkPtr  & i_weakPtr )
    {
      ktStatus success = this->CkwsConstraint::init( i_weakPtr );
      if ( success == KD_OK ) wkPtr_ = i_weakPtr;
      return success;
    }

    ktStatus KwsConstraint::
    doApply(CkwsConfig& io_config,
	    const CkitParameterMapConstShPtr& parameterMap) const
    {
       return (projector_->project(io_config));
    }

  } //end of namespace constrained
} //end of namespace hpp
