cmake_minimum_required(VERSION 3.22)
# rosidl は C のターゲットも作るので C も有効にする
project(mini_shirasu_ros C CXX)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 26)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robomas_plugins REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- メッセージ ---
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Status.msg"
	DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(mini_shirasu_msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- プロトコル (ROS非依存) ---
add_library(mini_shirasu_protocol STATIC
	src/protocol.cpp
)
target_include_directories(mini_shirasu_protocol
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)
# std::expected を使うので C++23 以上
target_compile_features(mini_shirasu_protocol PUBLIC cxx_std_23)
set_target_properties(mini_shirasu_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- ノード ---
add_executable(mini_shirasu_node
	src/mini_shirasu_node.cpp
)
target_link_libraries(mini_shirasu_node
	PRIVATE
		mini_shirasu_protocol
		${mini_shirasu_msgs_target}
		rclcpp::rclcpp
		${std_msgs_TARGETS}
		${std_srvs_TARGETS}
		${robomas_plugins_TARGETS}
)

# --- 手動テスト用: 模擬基板 ---
add_executable(fake_mini_shirasu
	test/fake_mini_shirasu.cpp
)
target_link_libraries(fake_mini_shirasu
	PRIVATE
		mini_shirasu_protocol
		rclcpp::rclcpp
		${robomas_plugins_TARGETS}
)

install(TARGETS mini_shirasu_node fake_mini_shirasu
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
	DESTINATION include
)
install(DIRECTORY config launch
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	# 符号化・復号・パーサ・CAN への分割のテスト (ROS不要)
	add_executable(mini_shirasu_protocol_test test/protocol_test.cpp)
	target_link_libraries(mini_shirasu_protocol_test PRIVATE mini_shirasu_protocol)
	ament_add_test(mini_shirasu_protocol
		COMMAND "$<TARGET_FILE:mini_shirasu_protocol_test>"
		GENERATE_RESULT_FOR_RETURN_CODE_ZERO
	)
endif()

ament_package()
