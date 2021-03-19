add_subdirectory("./exploration")

add_cf_executable(rr)
target_link_libraries(rr PRIVATE exploration)
target_include_directories(rr PRIVATE "./include")
target_sources(rr PRIVATE "./src/main.cpp")
