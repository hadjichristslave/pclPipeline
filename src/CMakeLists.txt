set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#project(cloud_viewer)
#project(extract_indices)
#project(statistical_removal)
#project(matrix_transform)
#project(correspondence_grouping)
#project(cluster_extraction)
#project(region_growing_segmentation)
#project(conditional_euclidean_clustering)
#project(iterative_closest_point)
#project(region_growing_rgb_segmentation)
#project(compress)
#project(octree_change_detection)
project(pipeline)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (cloud_viewer cloud_viewer.cpp)
#target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

#add_executable(extract_indices extract_indices.cpp)
#target_link_libraries(extract_indices ${PCL_LIBRARIES})


#add_executable (statistical_removal statistical_removal.cpp)
#target_link_libraries (statistical_removal ${PCL_LIBRARIES})


#add_executable (matrix_transform matrix_transform.cpp)
#target_link_libraries (matrix_transform ${PCL_LIBRARIES})

#add_executable (correspondence_grouping correspondence_grouping.cpp)
#target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})


#add_executable (cluster_extraction cluster_extraction.cpp)
#target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

#add_executable (region_growing_segmentation region_growing_segmentation.cpp)
#target_link_libraries (region_growing_segmentation ${PCL_LIBRARIES})
#add_executable (conditional_euclidean_clustering conditional_euclidean_clustering.cpp)
#target_link_libraries (conditional_euclidean_clustering ${PCL_LIBRARIES})
#
#add_executable (iterative_closest_point iterative_closest_point.cpp)
#target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
#
#add_executable (region_growing_rgb_segmentation region_growing_rgb_segmentation.cpp)
#target_link_libraries (region_growing_rgb_segmentation ${PCL_LIBRARIES})


#add_executable (compress compress.cpp)
#target_link_libraries (compress ${PCL_LIBRARIES})

#add_executable (octree_change_detection octree_change_detection.cpp)
#target_link_libraries (octree_change_detection ${PCL_LIBRARIES})

add_executable(main pipeline/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})
