include(Arduino)

enable_arduino_toolchain()

add_arduino_library(atlas-helper "main.cpp")
find_package(Wire)
target_link_libraries(atlas-helper Wire)
add_arduino_firmware(atlas-helper)
