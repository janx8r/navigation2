add_library(map_lib SHARED
  map.c
  map_range.c
  map_draw.c
  map_cspace.cpp
)

ament_target_dependencies(map_lib
  nav2_ndt_map
)

install(TARGETS
  map_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
