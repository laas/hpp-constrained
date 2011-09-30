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

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)


SET(PROJECT_NAME hpp-constrained)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION "Package dedicated to motion planning on constrained manifolds.")
SET(PROJECT_URL "https://github.com/laas/hpp-constrained")


SET(${PROJECT_NAME}_HEADERS
  ${CMAKE_SOURCE_DIR}/include/hpp/constrained/fwd.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/constrained/config-projector.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/constrained/config-extendor.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/constrained/roadmap-builder.hh
  ${CMAKE_SOURCE_DIR}/include/hpp/constrained/kws-constraint.hh
  )

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("hpp-model")
ADD_REQUIRED_DEPENDENCY("hpp-gik")

# Search for Boost.
SET(BOOST_COMPONENTS
  filesystem system thread program_options unit_test_framework)
SEARCH_FOR_BOOST()

ADD_SUBDIRECTORY(src)

# Documentation produced by doxygen
INCLUDE(FindDoxygen)
IF(DOXYGEN_FOUND)
  add_subdirectory(doc)
ENDIF(DOXYGEN_FOUND)

SETUP_PROJECT_FINALIZE()
