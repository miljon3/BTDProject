# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/gflor/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/tmp"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/src"
  "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/gflor/espidf-projects/btd-project/screen/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
