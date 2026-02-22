# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/duke/esp/v5.5.1/components/bootloader/subproject"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/tmp"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/src/bootloader-stamp"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/src"
  "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/duke/ESP32-C6-Touch-AMOLED-2.06/examples/ESP-IDF-v5.5.1/05_Spec_Analyzer/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
