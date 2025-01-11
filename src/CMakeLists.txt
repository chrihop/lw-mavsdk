set (LWMAVSDK_SRC
    connection.c
    connection_factory.c
    vehicle.c
    microservice.c
    protocol.c
    command.c
    command_factory.c
    )


if (BUILD_FOR STREQUAL "posix")
    list (APPEND LWMAVSDK_SRC
        posix/serial.c
        posix/udp_client.c
        posix/udp.c
        certikos_user/partee.c
        )
elseif (BUILD_FOR STREQUAL "certikos_user")
    list (APPEND LWMAVSDK_SRC
        certikos_user/serial.c
        certikos_user/thinros.c
        certikos_user/partee.c
        )
elseif (BUILD_FOR STREQUAL "certikos_user_musl")
    list (APPEND LWMAVSDK_SRC
        posix/serial.c
        posix/udp_client.c
        posix/udp.c
        certikos_user/partee.c
        )
endif()

add_library(lwmavsdk
    STATIC
    ${LWMAVSDK_SRC})

target_include_directories(lwmavsdk
    PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${LWMAVSDK_INCLUDE_DIR}
    )

add_dependencies(lwmavsdk
    mavlink-headers)
