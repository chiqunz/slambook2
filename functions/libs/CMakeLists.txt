add_library(LINALG STATIC linalg.c)
target_include_directories(LINALG PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})