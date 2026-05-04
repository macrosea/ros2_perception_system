cmake_minimum_required(VERSION 3.14)
project(perception_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ── 依赖 ──────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tracetools REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(FFMPEG_ROOT "$ENV{VCPKG_ROOT}/installed/x64-linux-dynamic")
find_library(FFMPEG_avcodec_lib  avcodec  PATHS "${FFMPEG_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(FFMPEG_avformat_lib avformat PATHS "${FFMPEG_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(FFMPEG_avutil_lib   avutil   PATHS "${FFMPEG_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(FFMPEG_swscale_lib  swscale  PATHS "${FFMPEG_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
set(FFMPEG_LIBRARIES ${FFMPEG_avcodec_lib} ${FFMPEG_avformat_lib} ${FFMPEG_avutil_lib} ${FFMPEG_swscale_lib})
set(FFMPEG_INCLUDE_DIRS "${FFMPEG_ROOT}/include")
set(FFMPEG_LIBRARY_DIRS "${FFMPEG_ROOT}/lib")

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES 86)

set(TRT_ROOT "$ENV{TRT_ROOT}" CACHE PATH "TensorRT install root")
if(NOT TRT_ROOT)
  set(TRT_ROOT "$ENV{HOME}/depends/tensorrt/TensorRT-10.7.0.23")
endif()
find_library(TRT_nvinfer_lib nvinfer PATHS "${TRT_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
set(TRT_INCLUDE_DIRS "${TRT_ROOT}/include")

rosidl_generate_interfaces(${PROJECT_NAME}
  "src/interfaces/msg/TraceStamp.msg"
  "src/interfaces/msg/ProcessedImage.msg"
  "src/interfaces/msg/InferenceResult.msg"
  "src/interfaces/srv/ResetNode.srv"
  DEPENDENCIES builtin_interfaces sensor_msgs
)

set(DEPS
   builtin_interfaces
   rclcpp
   rclcpp_components
   rclcpp_lifecycle
   lifecycle_msgs
   sensor_msgs
   vision_msgs
)

# ── 公共头文件 ────────────────────────────────────────────────────────────────
include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── Component 共享库 ──────────────────────────────────────────────────────────
add_library(camera_component SHARED src/camera_component.cpp src/capture/capture_source.cpp)
ament_target_dependencies(camera_component ${DEPS})
target_include_directories(camera_component PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_directories(camera_component PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(camera_component ${FFMPEG_LIBRARIES} ${OpenCV_LIBS} "${cpp_typesupport_target}")
rclcpp_components_register_node(camera_component
   PLUGIN "perception_system::CameraComponent"
   EXECUTABLE camera_node)

add_library(stage_tracepoint SHARED src/stage_tracepoint_provider.cpp)
ament_target_dependencies(stage_tracepoint tracetools)
target_link_libraries(stage_tracepoint tracetools::tracetools)

add_library(preprocessing SHARED
  src/preprocessing/preprocessor.cpp
  src/preprocessing/letterbox_cuda.cu
)
ament_target_dependencies(preprocessing sensor_msgs builtin_interfaces)
target_link_libraries(preprocessing CUDA::cudart)
target_include_directories(preprocessing PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
set_target_properties(preprocessing PROPERTIES CUDA_ARCHITECTURES "86" CUDA_SEPARABLE_COMPILATION ON)

add_library(image_proc_component SHARED src/image_proc_component.cpp)
ament_target_dependencies(image_proc_component ${DEPS})
target_link_libraries(image_proc_component "${cpp_typesupport_target}" stage_tracepoint tracetools::tracetools preprocessing)
rclcpp_components_register_node(image_proc_component
   PLUGIN "perception_system::ImageProcComponent"
   EXECUTABLE image_proc_node)

add_library(inference SHARED
  src/inference/trt_engine.cpp
  src/inference/trt_context.cpp
  src/inference/inference_adapter.cpp
)
target_include_directories(inference PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CUDAToolkit_INCLUDE_DIRS} ${TRT_INCLUDE_DIRS})
target_link_libraries(inference CUDA::cudart ${TRT_nvinfer_lib})
target_compile_options(inference PRIVATE -Wno-deprecated-declarations)
set_target_properties(inference PROPERTIES CUDA_ARCHITECTURES "86" CUDA_SEPARABLE_COMPILATION ON)

add_library(detector_component SHARED src/detector_component.cpp)
ament_target_dependencies(detector_component ${DEPS})
target_link_libraries(detector_component "${cpp_typesupport_target}" stage_tracepoint tracetools::tracetools inference)
target_compile_options(detector_component PRIVATE -Wno-deprecated-declarations)
rclcpp_components_register_node(detector_component
   PLUGIN "perception_system::DetectorComponent"
   EXECUTABLE detector_node)

add_library(visualize_component SHARED src/visualize_component.cpp src/visualize/yolov8_postprocess.cpp src/visualize/visualizer.cpp)
ament_target_dependencies(visualize_component ${DEPS})
target_link_libraries(visualize_component "${cpp_typesupport_target}" stage_tracepoint tracetools::tracetools ${OpenCV_LIBS})
rclcpp_components_register_node(visualize_component
   PLUGIN "perception_system::VisualizeComponent"
   EXECUTABLE visualize_node)

add_library(lifecycle_manager SHARED src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager ${DEPS})
target_link_libraries(lifecycle_manager "${cpp_typesupport_target}")

add_library(health_monitor_component SHARED src/health_monitor_component.cpp)
ament_target_dependencies(health_monitor_component ${DEPS})
target_link_libraries(health_monitor_component "${cpp_typesupport_target}")
rclcpp_components_register_node(health_monitor_component
   PLUGIN "perception_system::HealthMonitorComponent"
   EXECUTABLE health_monitor_node)

# ── 主容器可执行文件 ──────────────────────────────────────────────────────────
add_executable(perception_container src/perception_container.cpp)
target_link_libraries(perception_container
   camera_component
    stage_tracepoint
    image_proc_component
    detector_component
    visualize_component
    lifecycle_manager
    health_monitor_component
    "${cpp_typesupport_target}"
)
ament_target_dependencies(perception_container
    ${DEPS}
    lifecycle_msgs   # perception_container 直接用了 ChangeState，需显式声明
)
target_compile_options(perception_container PRIVATE -Wno-deprecated-declarations)


# ── 安装 ──────────────────────────────────────────────────────────────────────
install(TARGETS
   camera_component
   stage_tracepoint
   preprocessing
   inference
   image_proc_component
   detector_component
   visualize_component
   lifecycle_manager
   health_monitor_component
   perception_container
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
)

install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  builtin_interfaces
  rosidl_default_runtime
  sensor_msgs
  tracetools
)

ament_package()
