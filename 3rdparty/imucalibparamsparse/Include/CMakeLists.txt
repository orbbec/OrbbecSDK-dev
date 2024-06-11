file(GLOB headers "*.h*")

install(FILES ${headers}
        DESTINATION ${CALIBPARAM_INCLUDE_INSTALL_PATH}/
        COMPONENT dev
        )
