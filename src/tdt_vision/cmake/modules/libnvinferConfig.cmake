# - Try to find nvinfer
#
# The following variables are optionally searched for defaults
#  nvinfer_ROOT_DIR:            Base directory where all nvinfer components are found
#
# The following are set after configuration is done:
#  nvinfer_FOUND
#  nvinfer_INCLUDE_DIRS
#  nvinfer_LIBRARIES
#  nvinfer_LIBRARYRARY_DIRS

include(FindPackageHandleStandardArgs)

set(nvinfer_ROOT_DIR "" CACHE PATH "Folder contains nvinfer")

if(WIN32)
    find_path(nvinfer_INCLUDE_DIR NvInfer.h
        PATHS ${nvinfer_ROOT_DIR})
else()
    find_path(nvinfer_INCLUDE_DIR NvInfer.h
        PATHS ${nvinfer_ROOT_DIR})
endif()

if(MSVC)
    find_library(nvinfer_LIBRARY_RELEASE nvinfer
        PATHS ${nvinfer_ROOT_DIR}
        PATH_SUFFIXES Release)

    find_library(nvinfer_LIBRARY_DEBUG nvinfer
        PATHS ${nvinfer_ROOT_DIR}
        PATH_SUFFIXES Debug)

    set(nvinfer_LIBRARY optimized ${nvinfer_LIBRARY_RELEASE} debug ${nvinfer_LIBRARY_DEBUG})
else()
    find_library(nvinfer_LIBRARY nvinfer
        PATHS ${nvinfer_ROOT_DIR}
        PATH_SUFFIXES lib lib64)
endif()

find_package_handle_standard_args(nvinfer DEFAULT_MSG nvinfer_INCLUDE_DIR nvinfer_LIBRARY)

if(nvinfer_FOUND)
  set(nvinfer_INCLUDE_DIRS ${nvinfer_INCLUDE_DIR})
  set(nvinfer_LIBRARIES ${nvinfer_LIBRARY})
  message(STATUS "Found nvinfer    (include: ${nvinfer_INCLUDE_DIR}, library: ${nvinfer_LIBRARY})")
  mark_as_advanced(nvinfer_ROOT_DIR nvinfer_LIBRARY_RELEASE nvinfer_LIBRARY_DEBUG
                                 nvinfer_LIBRARY nvinfer_INCLUDE_DIR)
endif()