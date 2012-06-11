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

#ifndef HPP_CONSTRAINED_CONSTRAINT_SET_HH
# define HPP_CONSTRAINED_CONSTRAINT_SET_HH

# include <vector>
# include <jrl/mal/matrixabstractlayer.hh>
# include <gikTask/jrlGikStateConstraint.h>

namespace hpp {
  namespace constrained {
    /// Set of constraints with same priority
    class ConstraintSet : public CjrlGikStateConstraint
    {
    public:
      typedef std::vector <CjrlGikStateConstraint*> constraints_t;
      typedef constraints_t::iterator iterator_t;
      typedef unsigned int size_type;
      /// \name Add and remove constraints
      /// @{

      /// Remove all constraints
      void resetConstraints();
      /// Add a constraint
      void addConstraint (CjrlGikStateConstraint* constraint);
      /// Remove last constraint
      void removeLastConstraint ();
      /// Remove a given constraint
      void removeConstraint(CjrlGikStateConstraint* rmConstraint);
      /// @}

      virtual ~ConstraintSet ();
      /// Constructor
      ConstraintSet (CjrlDynamicRobot& robot);
      // Copy constructor
      ConstraintSet (const ConstraintSet& set);
      /// Clone the object
      virtual CjrlGikStateConstraint* clone() const;
      /// Associated robot.
      CjrlDynamicRobot& robot();
      /// \brief Select the joint in the robot that serves as root for
      /// computation of jacobians.
      void jacobianRoot(CjrlJoint& inJoint);

      /// Get the influencing dofs vector.

      //// the influencing dofs vector is a binary vector whose size
      //// matches the robot configuration size, where an element with
      //// value 1 indicates that the corresponding degree of freedom
      //// can modify the value of this constraint and an element with
      //// value 0 cannot.
      vectorN& influencingDofs();

      constraints_t getConstraints () const;

      /// \name Value and Jacobian
      /// \{
      unsigned int dimension() const;

      /// Get the constraint value.
      virtual const vectorN& value();

      /// Get the constraint Jacobian
      virtual const matrixNxP& jacobian();

      /// \}
      /// \name Computations
      /// \{

      /// Compute the influencing dofs vector.
      void computeInfluencingDofs();

      /// Compute the value of the constraint.
      void computeValue();

      /// Compute the Jacobian of the constraint
      ///
      /// Jacobian is computed with respect to the internal degrees of
      /// freedom and to a selected root joint (see method
      /// CjrlGikStateConstraint::jacobianRoot()).
      void computeJacobian();
      ///@}
    private:
      void resizeValueAndJacobian ();
      CjrlDynamicRobot& robot_;
      constraints_t constraints_;
      vectorN activeDofs_;
      vectorN value_;
      matrixNxP jacobian_;

    }; // class ConstraintSet
  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_CONSTRAINT_SET_HH
