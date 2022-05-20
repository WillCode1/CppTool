# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

#-------------------------------------------------------------------------------
# Upload to Vibrante board target
#-------------------------------------------------------------------------------
if(VIBRANTE_BUILD)
    set(VIBRANTE_INSTALL_PATH "/home/nvidia/driveworks" CACHE STRING "Directory on the target board where to install the SDK")
    set(VIBRANTE_USER "nvidia" CACHE STRING "User used for ssh to upload files over to the board")
    set(VIBRANTE_PASSWORD "nvidia" CACHE STRING "Password of the specified user")
    set(VIBRANTE_HOST "192.168.10.10" CACHE STRING "Hostname or IP address of the tegra board")
    set(VIBRANTE_PORT "22" CACHE STRING "SSH port of the tegra board")

    if(SDK_DEPLOY_BUILD OR ${PROJECT_NAME} MATCHES "DriveworksSDK-Samples")
        if(NOT TARGET upload)
                add_custom_target(upload
                    # create installation
                    COMMAND "${CMAKE_COMMAND}" --build ${SDK_BINARY_DIR} --target install
                    # create installation folder on target
                    COMMAND sshpass -p "${VIBRANTE_PASSWORD}" ssh -o StrictHostKeyChecking=no -p ${VIBRANTE_PORT} ${VIBRANTE_USER}@${VIBRANTE_HOST} "mkdir -p ${VIBRANTE_INSTALL_PATH}"
                    # upload installation
                    COMMAND sshpass -p "${VIBRANTE_PASSWORD}" rsync --progress -rltgDz -e "ssh -p ${VIBRANTE_PORT}" ${CMAKE_INSTALL_PREFIX}/ ${VIBRANTE_USER}@${VIBRANTE_HOST}:${VIBRANTE_INSTALL_PATH}/
                    )
        endif()
    endif()
endif()
