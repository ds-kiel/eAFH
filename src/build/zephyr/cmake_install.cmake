# Install script for directory: /Users/valentin/research/official_repos/eAFH/src/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/valentin/research/toolchains/gcc-arm-none-eabi-10.3-2021.07/bin/arm-none-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/arch/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/lib/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/soc/arm/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/boards/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/subsys/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/drivers/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/cmsis/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/atmel/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/altera/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/canopennode/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/civetweb/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/esp-idf/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/fatfs/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/cypress/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/infineon/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/nordic/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/openisa/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/nuvoton/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/microchip/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/silabs/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/st/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/stm32/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/ti/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/libmetal/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/quicklogic/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/lvgl/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/mbedtls/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/mcumgr/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/nxp/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/open-amp/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/loramac-node/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/openthread/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/segger/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/tinycbor/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/tinycrypt/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/littlefs/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/mipi-sys-t/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/nrf_hw_models/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/xtensa/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/modules/tfm/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/kernel/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/cmake/flash/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/cmake/usage/cmake_install.cmake")
  include("/Users/valentin/research/official_repos/eAFH/src/build/zephyr/cmake/reports/cmake_install.cmake")

endif()

