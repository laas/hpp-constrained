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

#ifndef HPP_CONSTRAINED_CONFIG_SHOOTER_HH
# define HPP_CONSTRAINED_CONFIG_SHOOTER_HH

# include <hpp/model/fwd.hh>
# include "hpp/constrained/fwd.hh"

namespace hpp {
  namespace constrained {
    /// Shoot a random configuration about a given configuration
    class ConfigShooter
    {
    public:
      virtual ~ConfigShooter();

      /// Generate a goal configuration about a given configuration
      
      /// Configuration is shoot according to a Gaussian law centered at
      /// the input configuration. The default standard deviation (0.01)
      /// can be changed by calling ConfigShooter::standardDeviation setter.
      virtual void shoot (CkwsConfig& io_config) const;

      void standardDeviation (const double& sigma);
      double standardDeviation () const;

      /// Create object and return shared pointer
      static ConfigShooterShPtr create (hpp::model::DeviceShPtr robot);
    protected:
      /// Constructor that takes a projector onto the goal submanifold.
      ConfigShooter (hpp::model::DeviceShPtr robot);
      /// Initialization
      void init (ConfigShooterWkPtr wkPtr);

    private:
      model::DeviceShPtr robot_;
      double standardDeviation_;
      ConfigShooterWkPtr weakPtr_;
    }; // class ConfigShooter
  } // namespace constrained
} //namespace hpp
#endif // HPP_CONSTRAINED_CONFIG_SHOOTER_HH
