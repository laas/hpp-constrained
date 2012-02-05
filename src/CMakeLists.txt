# Copyright 2011, Sebastien Dalibard, CNRS
#
# This file is part of hpp-constrained.
# hpp-constrained is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-constrained is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-constrained. If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

SET(${PACKAGE_NAME}_SOURCES
  config-extendor.cc
  config-optimizer.cc
  config-projector.cc
  config-shooter.cc
  constraint-set.cc
  constraint-set.hh
  goal-config-generator.cc
  kws-constraint.cc
  )
 
ADD_LIBRARY(hpp-constrained 
  SHARED 
  ${${PACKAGE_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(hpp-constrained hpp-model)
PKG_CONFIG_USE_DEPENDENCY(hpp-constrained hpp-gik)

INSTALL(TARGETS hpp-constrained
  DESTINATION lib)
