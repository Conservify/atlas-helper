set(SOURCE_FILES
  main.cpp
)

set(LIBRARIES
  Wire
)

include_directories(../../)

arduino(atlas-helper "${SOURCE_FILES}" "${LIBRARIES}")
