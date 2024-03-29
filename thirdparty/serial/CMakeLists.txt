cmake_minimum_required(VERSION 3.15)
project(serial)

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

if (APPLE)
    # If OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif (UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else ()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif ()

## Add serial library
add_library(${PROJECT_NAME} ${serial_SRCS})
if (APPLE)
    target_link_libraries(${PROJECT_NAME} IOKit Foundation)
elseif (UNIX)
    target_link_libraries(${PROJECT_NAME} rt pthread)
else ()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif ()

## Include headers
target_include_directories(${PROJECT_NAME} PUBLIC include)