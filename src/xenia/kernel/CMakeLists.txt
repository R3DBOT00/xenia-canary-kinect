add_library(xenia-kernel STATIC)
xe_platform_sources(xenia-kernel ${CMAKE_CURRENT_SOURCE_DIR} RECURSIVE)
target_compile_definitions(xenia-kernel PRIVATE WITH_GZFILEOP)
if(XE_TARGET_X86_64)
  target_compile_definitions(xenia-kernel PRIVATE
    X86_FEATURES
    X86_HAVE_XSAVE_INTRIN
    X86_SSE2
    X86_SSSE3
    X86_SSE42
    X86_AVX2
    X86_AVX512
    X86_AVX512VNNI
    X86_PCLMULQDQ_CRC
    X86_VPCLMULQDQ_CRC
  )
endif()
target_link_libraries(xenia-kernel PUBLIC
  aes_128 fmt zlib-ng pugixml xenia-apu xenia-base xenia-cpu xenia-hid xenia-vfs
)

if(WIN32)
  find_path(XENIA_KINECT_V1_INCLUDE_DIR
    NAMES NuiApi.h
    HINTS
      "C:/Program Files/Microsoft SDKs/Kinect/v1.8"
      "C:/Program Files (x86)/Microsoft SDKs/Kinect/v1.8"
    PATH_SUFFIXES inc
  )
  find_library(XENIA_KINECT_V1_LIBRARY
    NAMES Kinect10
    HINTS
      "C:/Program Files/Microsoft SDKs/Kinect/v1.8/lib/amd64"
      "C:/Program Files (x86)/Microsoft SDKs/Kinect/v1.8/lib/amd64"
  )
  if(XENIA_KINECT_V1_INCLUDE_DIR AND XENIA_KINECT_V1_LIBRARY)
    target_include_directories(xenia-kernel SYSTEM PRIVATE
      "${XENIA_KINECT_V1_INCLUDE_DIR}"
    )
    target_compile_definitions(xenia-kernel PRIVATE XENIA_HAS_KINECT_V1_SDK=1)
    target_link_libraries(xenia-kernel PRIVATE "${XENIA_KINECT_V1_LIBRARY}")
    message(STATUS "Xenia: Kinect v1 SDK backend enabled")
  else()
    message(STATUS "Xenia: Kinect v1 SDK not found; NUI backend disabled")
  endif()
endif()

xe_target_defaults(xenia-kernel)

if(XENIA_BUILD_TESTS)
  set(CMAKE_FOLDER "tests")
  add_subdirectory(testing)
endif()
