set( xslam-usb-sdk_LIBS xslam-usb-sdk )
set( xslam-hid-sdk_LIBS xslam-hid-sdk )
set( xslam-uvc-sdk_LIBS xslam-uvc-sdk )
set( xslam-vsc-sdk_LIBS xslam-vsc-sdk )
set( xslam-edge-sdk_LIBS xslam-edge-sdk )
set( xslam-slam-sdk_LIBS "" )

set(xvsdk_FOUND TRUE)
set(xvsdk_VERSION 3.2.0)
if( WIN32 )
	set(xvsdk_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../include/)
	set(xvsdk_BINARY_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../bin/)
	set(xvsdk_LIBRARY_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../lib/)
	set(xvsdk_LIBRARY xvsdk ${xslam-usb-sdk_LIBS} ${xslam-hid-sdk_LIBS} ${xslam-uvc-sdk_LIBS} ${xslam-vsc-sdk_LIBS} ${xslam-edge-sdk_LIBS} ${xslam-slam-sdk_LIBS})
	set(xvsdk_LIBRARIES ${xvsdk_LIBRARY} )
	set(xvsdk_LIBS ${xvsdk_LIBRARY} )
else()
	set(xvsdk_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../../include/)
	set(xvsdk_LIBRARY_DIRS ${CMAKE_CURRENT_LIST_DIR}/../../../lib/)
	set(xvsdk_LIBRARY xvsdk ${xslam-usb-sdk_LIBS} ${xslam-hid-sdk_LIBS} ${xslam-uvc-sdk_LIBS} ${xslam-vsc-sdk_LIBS} ${xslam-edge-sdk_LIBS} ${xslam-slam-sdk_LIBS})
	set(xvsdk_LIBRARIES ${xvsdk_LIBRARY} )
	set(xvsdk_LIBS ${xvsdk_LIBRARY} )
endif()

include_directories( ${xvsdk_INCLUDE_DIRS} )
include_directories( ${xvsdk_INCLUDE_DIRS}/xvsdk )
link_directories( ${xvsdk_LIBRARY_DIRS} )
