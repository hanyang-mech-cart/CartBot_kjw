# CMake generated Testfile for 
# Source directory: /home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test
# Build directory: /home/kong/my_ws/build/dwb_critics/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(prefer_forward_tests "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/prefer_forward_tests.gtest.xml" "--package-name" "dwb_critics" "--output-file" "/home/kong/my_ws/build/dwb_critics/ament_cmake_gtest/prefer_forward_tests.txt" "--command" "/home/kong/my_ws/build/dwb_critics/test/prefer_forward_tests" "--gtest_output=xml:/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/prefer_forward_tests.gtest.xml")
set_tests_properties(prefer_forward_tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kong/my_ws/build/dwb_critics/test/prefer_forward_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/kong/my_ws/build/dwb_critics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;1;ament_add_gtest;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;0;")
add_test(base_obstacle_tests "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/base_obstacle_tests.gtest.xml" "--package-name" "dwb_critics" "--output-file" "/home/kong/my_ws/build/dwb_critics/ament_cmake_gtest/base_obstacle_tests.txt" "--command" "/home/kong/my_ws/build/dwb_critics/test/base_obstacle_tests" "--gtest_output=xml:/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/base_obstacle_tests.gtest.xml")
set_tests_properties(base_obstacle_tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kong/my_ws/build/dwb_critics/test/base_obstacle_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/kong/my_ws/build/dwb_critics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;4;ament_add_gtest;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;0;")
add_test(obstacle_footprint_tests "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/obstacle_footprint_tests.gtest.xml" "--package-name" "dwb_critics" "--output-file" "/home/kong/my_ws/build/dwb_critics/ament_cmake_gtest/obstacle_footprint_tests.txt" "--command" "/home/kong/my_ws/build/dwb_critics/test/obstacle_footprint_tests" "--gtest_output=xml:/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/obstacle_footprint_tests.gtest.xml")
set_tests_properties(obstacle_footprint_tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kong/my_ws/build/dwb_critics/test/obstacle_footprint_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/kong/my_ws/build/dwb_critics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;7;ament_add_gtest;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;0;")
add_test(alignment_util_tests "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/alignment_util_tests.gtest.xml" "--package-name" "dwb_critics" "--output-file" "/home/kong/my_ws/build/dwb_critics/ament_cmake_gtest/alignment_util_tests.txt" "--command" "/home/kong/my_ws/build/dwb_critics/test/alignment_util_tests" "--gtest_output=xml:/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/alignment_util_tests.gtest.xml")
set_tests_properties(alignment_util_tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kong/my_ws/build/dwb_critics/test/alignment_util_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/kong/my_ws/build/dwb_critics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;10;ament_add_gtest;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;0;")
add_test(twirling_tests "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/twirling_tests.gtest.xml" "--package-name" "dwb_critics" "--output-file" "/home/kong/my_ws/build/dwb_critics/ament_cmake_gtest/twirling_tests.txt" "--command" "/home/kong/my_ws/build/dwb_critics/test/twirling_tests" "--gtest_output=xml:/home/kong/my_ws/build/dwb_critics/test_results/dwb_critics/twirling_tests.gtest.xml")
set_tests_properties(twirling_tests PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/kong/my_ws/build/dwb_critics/test/twirling_tests" TIMEOUT "60" WORKING_DIRECTORY "/home/kong/my_ws/build/dwb_critics/test" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;13;ament_add_gtest;/home/kong/my_ws/src/navigation2/nav2_dwb_controller/dwb_critics/test/CMakeLists.txt;0;")
subdirs("../gtest")
