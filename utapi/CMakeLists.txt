cmake_minimum_required(VERSION 3.5)
project(utapi)


include_directories("./")

add_library(utapi STATIC
    common/network.cpp    
    common/preempt_rt.cpp    
    common/socket_serial.cpp
    common/socket_tcp.cpp
    base/servo_api_base.cpp
    adra/adra_api_base.cpp
    adra/adra_api_serial.cpp

    base/arm_api_base.cpp
    base/arm_report_status.cpp
    base/arm_report_config.cpp
    utra/utra_api_tcp.cpp
    utra/utra_report_status.cpp
    utra/utra_report_config.cpp
    )

target_link_libraries(utapi rt pthread)
