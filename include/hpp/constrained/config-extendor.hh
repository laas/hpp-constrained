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

#ifndef HPP_CONSTRAINED_CONFIG_EXTENDOR_HH
#define HPP_CONSTRAINED_CONFIG_EXTENDOR_HH

#include <hpp/constrained/fwd.hh>
#include <hpp/constrained/config-projector.hh>

#include <hpp/gik/constraint/configuration-constraint.hh>

namespace hpp {
  namespace constrained {
    class ConfigExtendor : public ConfigProjector
    {
    public:
      /**
       * \brief Constructor
       */
      ConfigExtendor(hpp::model::DeviceShPtr robot);
      
      /**
       * \brief Destructor
       */
      ~ConfigExtendor();

      /**
       * \brief Performs one elementary extension on a constrained manifold.
       * @param extendTo Configuration towards which the extension is performed
       * @param extendFrom Configuration from which the extension is performed, 
       * can be omitted, the extension then starts from the current configuration
       * @return o_config Extended config. Can be NULL if no extension is possible
       */
      CkwsConfigShPtr
      extendOneStep(CkwsConfig & extendTo,
		    CkwsConfig & extendFrom);

      /**
       * \brief Performs one elementary extension on a constrained manifold.
       * @param extendTo Configuration towards which the extension is performed
       * @return o_config Extended config. Can be NULL if no extension is possible
       */
      CkwsConfigShPtr
      extendOneStep(CkwsConfig & extendTo);

      /**
       * \brief Sets the length of extensions
       */
      void
      setExtensionStep(double i_extensionStep);

      /**
       * \brief Gets the length of extensions
       */
      double
      getExtensionStep();

    private:
      /**
       * \brief Length of one extension, homogeneous to the result of CkwsDistance::distance()
       */
      double extensionStep_;

      /**
       * \brief Configuration constraint, used to explore a constrained manifold with an inverse kinematics solver
       */
      ChppGikConfigurationConstraint * configConstraint_;
    };
  } //end of namespace constrained
} //end of namespace hpp

#endif // HPP_CONSTRAINED_CONFIG_EXTENDOR_HH
