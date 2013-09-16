// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-constrained.
//
// hpp-constrained is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-constrained is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-constrained.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CONSTRAINED_GOAL_CONFIG_GENERATOR_HH
# define HPP_CONSTRAINED_GOAL_CONFIG_GENERATOR_HH

# include <hpp/model/device.hh>

# include "hpp/constrained/config-projector.hh"
# include "hpp/constrained/fwd.hh"

namespace hpp {
  namespace constrained {
    /// Generate a goal random goal configuration about a given configuration
    class GoalConfigGenerator : public ConfigProjector
    {
    public:
      virtual ~GoalConfigGenerator();

      /// Generate a goal configuration about a given configuration
      /// \param io_config initial guess,
      /// \retval io_config result,
      /// \return true if success, false otherwise
      /// Shoot up to 10 random configuration and return the first one that
      /// satisfies the constraints and that is valid.
      virtual bool generate (CkwsConfig& io_config);

      /// Set configuration shooter
      void configShooter (const ConfigShooterShPtr& configShooter);

      /// Get config shooter
      ConfigShooterConstShPtr configShooter () const;

      /// Create object and return shared pointer
      static GoalConfigGeneratorShPtr create (hpp::model::DeviceShPtr robot);
    protected:
      /// Constructor that takes a projector onto the goal submanifold.
      GoalConfigGenerator (hpp::model::DeviceShPtr robot);
      /// Initialization
      void init (GoalConfigGeneratorWkPtr wkPtr);

    private:
      GoalConfigGeneratorWkPtr weakPtr_;
      ConfigShooterShPtr configShooter_;
    }; // class GoalConfigGenerator
  } // namespace constrained
} //namespace hpp
#endif // HPP_CONSTRAINED_GOAL_CONFIG_GENERATOR_HH
