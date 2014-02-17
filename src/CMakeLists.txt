cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include"
					"${PROJECT_SOURCE_DIR}/src"
)

# file(
        # GLOB_RECURSE
        # source_files
        # ./*
		# ${PROJECT_SOURCE_DIR}/include/bullet/JointBullet.h
		# ${PROJECT_SOURCE_DIR}/include/bullet/Exports.h
# )
 
add_definitions(-DMANIPCORE_DLLEXPORT)
add_library(
        manipulability_reduced
        STATIC
        ./Exports.h
        ./tools/MatrixDefs.h
        ./tools/MatrixDefs.cpp
        ./tools/MatrixDefsInternal.h
        ./tools/Pi.h
        ./skeleton/Tree.h
        ./skeleton/Tree.cpp
        ./skeleton/Robot.h
        ./skeleton/Robot.cpp
        ./skeleton/Jacobian.h
        ./skeleton/Jacobian.cpp
        ./skeleton/Joint.h
        ./skeleton/Joint.cpp
		./sampling/Sample.h
		./sampling/Sample.cpp
		./sampling/SampleGenerator.h
		./sampling/SampleGenerator.cpp
		./sampling/SampleGeneratorVisitor_ABC.h
		./sampling/SampleGeneratorVisitor_ABC.cpp
		./sampling/filters/Filter_ABC.h
		./sampling/filters/Filter_ABC.cpp
		./sampling/filters/FilterDistance.h
		./sampling/filters/FilterDistance.cpp
		./world/Obstacle.h
		./world/Obstacle.cpp
		./world/ObstacleVisitor_ABC.h
		./world/ObstacleVisitor_ABC.cpp
		./world/Intersection.h
		./world/Intersection.cpp
		./world/CollisionHandler_ABC.h
		./world/CollisionHandler_ABC.cpp
		./world/CollisionHandlerDefault.h
		./world/CollisionHandlerDefault.cpp
		./world/World.h
		./world/World.cpp
		./
)

 
if(MSVC)
  # StringPooling: true == /GF  false == /GF-
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GF")
  # RuntimeLibrary
  # 0 (MultiThreaded) == /MT
  # 1 (MultiThreadedDebug) == /MTd
  # 2 (MultiThreadedDLL) == /MD
  # 3 (MultiThreadedDebugDLL) == /MDd
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

  # TreatWChar_tAsBuiltInType: true == /Zc:wchar_t  false == /Zc:wchar_t-
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:wchar_t")

  # WarningLevel: /W<level 0 to 4> or /Wall
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
endif()
 
 # if(MSVC10)
# target_link_libraries(
	# bullet_integration
	# debug bullet_integration
	# debug BulletDynamics_vs2010_x64_debug  
	# debug BulletCollision_vs2010_x64_debug 
	# debug LinearMath_vs2010_x64_debug  
	# debug OpenGLSupport_vs2010_x64_debug
	# optimized BulletDynamics_vs2010_x64_release  
	# optimized BulletCollision_vs2010_x64_release
	# optimized LinearMath_vs2010_x64_release  
	# optimized OpenGLSupport_vs2010_x64_release  
	# optimized bullet_integration
	# ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
# endif()

 