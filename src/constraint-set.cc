// Copyright (C) 2012 CNRS
// Author: Florent Lamiraux
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

#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include "../src/constraint-set.hh"

namespace hpp {
  namespace constrained {
    using boost::numeric::ublas::subrange;
    using boost::numeric::ublas::noalias;

    ConstraintSet::~ConstraintSet ()
    {
    }

    ConstraintSet::ConstraintSet (CjrlDynamicRobot& robot) :
      robot_ (robot), constraints_ (), activeDofs_ (robot.numberDof (), 0),
      value_ (), jacobian_ ()

    {
      resizeValueAndJacobian ();
    }

    ConstraintSet::ConstraintSet (const ConstraintSet& constraintSet) :
      robot_ (constraintSet.robot_), constraints_ (constraintSet.constraints_),
      activeDofs_ (constraintSet.activeDofs_),
      value_ (constraintSet.value_),
      jacobian_ (constraintSet.jacobian_)
    {
    }

    void ConstraintSet::resetConstraints()
    {
      constraints_.clear ();
      resizeValueAndJacobian ();
    }

    void ConstraintSet::addConstraint (CjrlGikStateConstraint* constraint)
    {
      constraints_.push_back (constraint);
      resizeValueAndJacobian ();
    }

    void ConstraintSet::removeLastConstraint ()
    {
      constraints_.pop_back ();
      resizeValueAndJacobian ();
    }

    void ConstraintSet::removeConstraint(CjrlGikStateConstraint* rmConstraint)
    {
      std::vector<CjrlGikStateConstraint*>::iterator it;
      for(it = constraints_.begin(); it!= constraints_.end(); it++) {
	if ((*it) == rmConstraint) {
	  delete rmConstraint;
	  constraints_.erase(it);
	}
      }
      resizeValueAndJacobian ();
    }

    CjrlGikStateConstraint* ConstraintSet::clone() const
    {
      return new ConstraintSet (*this);
    }

    CjrlDynamicRobot& ConstraintSet::robot()
    {
      return robot_;
    }
    void ConstraintSet::jacobianRoot(CjrlJoint& inJoint)
    {
      for (iterator_t it = constraints_.begin (); it != constraints_.end ();
	   it++) {
	CjrlGikStateConstraint* constraint = *it;
	constraint->jacobianRoot (inJoint);
      }
    }

    vectorN& ConstraintSet::influencingDofs ()
    {
      return activeDofs_;
    }

    ConstraintSet::constraints_t ConstraintSet::getConstraints () const
    {
      return constraints_;
    }

    unsigned int ConstraintSet::dimension () const
    {
      return value_.size ();
    }

    const vectorN& ConstraintSet::value ()
    {
      return value_;
    }

    const matrixNxP& ConstraintSet::jacobian ()
    {
      return jacobian_;
    }

    void ConstraintSet::computeInfluencingDofs()
    {
      for (size_type i=0; i<robot_.numberDof (); i++) {
	for (iterator_t it = constraints_.begin (); it != constraints_.end ();
	     it++) {
	  CjrlGikStateConstraint* constraint = *it;
	  if (constraint->influencingDofs () [i] != 0) {
	    activeDofs_ [i] = 1.;
	    break;
	  }
	}
      }
    }

    void ConstraintSet::computeValue()
    {
      size_type firstRow = 0;
      size_type lastRow;
      for (iterator_t it = constraints_.begin (); it != constraints_.end ();
	   it++) {
	(*it)->computeValue ();
	lastRow = firstRow + (*it)->dimension ();
	noalias (subrange (value_, firstRow, lastRow)) = (*it)->value ();
	firstRow = lastRow;
      }
    }

    void ConstraintSet::computeJacobian()
    {
      size_type firstRow = 0;
      size_type lastRow;
      for (iterator_t it = constraints_.begin (); it != constraints_.end ();
	   it++) {
	(*it)->computeJacobian ();
	lastRow = firstRow + (*it)->dimension ();
	noalias (subrange (jacobian_, firstRow, lastRow, 0,
			   robot_.numberDof ())) = (*it)->jacobian ();
	firstRow = lastRow;
      }
    }

    void ConstraintSet::resizeValueAndJacobian ()
    {
      size_type nbRows = 0;
      size_type nbCols = robot_.numberDof ();
      for (iterator_t it = constraints_.begin (); it != constraints_.end ();
	   it++) {
	CjrlGikStateConstraint* constraint = *it;
	nbRows += constraint->dimension ();
      }
      value_.resize (nbRows);
      jacobian_.resize (nbRows, nbCols);
    }
  } //end of namespace constrained
} //end of namespace hpp
